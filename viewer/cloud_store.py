"""Bounded-memory PLY import and persistent spatial chunk index (metres, XYZ)."""
from contextlib import closing
from collections import OrderedDict
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
import uuid

import numpy as np

VERSION = 1
DEFAULT_CACHE = Path(__file__).resolve().parent / '.cache' / 'clouds'
_TYPES = {'char':'i1', 'int8':'i1', 'uchar':'u1', 'uint8':'u1',
          'short':'i2', 'int16':'i2', 'ushort':'u2', 'uint16':'u2',
          'int':'i4', 'int32':'i4', 'uint':'u4', 'uint32':'u4',
          'float':'f4', 'float32':'f4', 'double':'f8', 'float64':'f8'}


def _check_cancel(cancel):
    if cancel is not None and (cancel.is_set() if hasattr(cancel, 'is_set') else cancel()):
        raise InterruptedError('Indexación cancelada; el archivo original no se ha modificado')


def _header(stream):
    magic = stream.readline(65537)
    if len(magic) > 65536 or magic.strip() != b'ply':
        raise ValueError('No es un archivo PLY')
    fmt = None
    count = None
    props = []
    element = None
    for _ in range(10000):
        raw = stream.readline(65537)
        if not raw or len(raw) > 65536:
            raise ValueError('Cabecera PLY incompleta o demasiado larga')
        try:
            fields = raw.decode('ascii').split()
        except UnicodeDecodeError as exc:
            raise ValueError('Cabecera PLY inválida') from exc
        if not fields or fields[0] in ('comment', 'obj_info'):
            continue
        if fields[0] == 'format':
            if len(fields) != 3 or fields[2] != '1.0':
                raise ValueError('Versión PLY no soportada')
            fmt = fields[1]
        elif fields[0] == 'element':
            if count is None and fields[1] != 'vertex' and int(fields[2]) != 0:
                raise ValueError('El elemento vertex debe ser el primero del PLY')
            element = fields[1]
            if element == 'vertex':
                if count is not None:
                    raise ValueError('Elemento vertex repetido')
                count = int(fields[2])
        elif fields[0] == 'property' and element == 'vertex':
            if len(fields) != 3 or fields[1] not in _TYPES:
                raise ValueError('Solo se admiten propiedades escalares numéricas por vértice; use tabla JSON para etiquetas textuales')
            if any(name == fields[2] for name, _ in props):
                raise ValueError('Propiedad PLY repetida')
            props.append((fields[2], _TYPES[fields[1]]))
        elif fields[0] == 'end_header':
            break
    else:
        raise ValueError('Cabecera PLY demasiado larga')
    if fmt not in ('ascii', 'binary_little_endian', 'binary_big_endian'):
        raise ValueError('Formato PLY no soportado')
    if count is None or count < 3 or not {'x','y','z'} <= {p[0] for p in props}:
        raise ValueError('Se requieren al menos tres vértices XYZ')
    order = '>' if fmt == 'binary_big_endian' else '<'
    return fmt, count, np.dtype([(name, order + typ) for name, typ in props])


def _mapping(path, mapping):
    sidecar = path.with_suffix(path.suffix + '.labels.json')
    if mapping is None and sidecar.is_file():
        if sidecar.stat().st_size > 1_000_000:
            raise ValueError('Tabla de etiquetas demasiado grande')
        mapping = json.loads(sidecar.read_text(encoding='utf-8'))
    if mapping is None:
        return {}
    if not isinstance(mapping, dict):
        raise ValueError('La tabla de etiquetas debe ser un objeto ID: categoría')
    allowed = {'grava', 'roca', 'plantas', 'barro', 'tierra', 'desconocido'}
    result = {}
    for key, value in mapping.items():
        if str(int(key)) != str(key) or value not in allowed:
            raise ValueError('Tabla de etiquetas inválida: use IDs enteros y categorías admitidas')
        result[str(key)] = value
    return result


class CloudStore:
    """SQLite stores spatially grouped array blobs, never one row per point.

    Each import block is partitioned into 10 m XY tiles, indexed by an RTree.
    Queries filter exact float64 coordinates after the conservative RTree lookup.
    Query limits raise, never silently change the terrain used by physics.
    Connections are per operation so the store can cross HTTP worker threads.
    """
    _query_cache_limit = 32 * 1024 * 1024
    _query_cache_entries = 8

    def __init__(self, index_path):
        self.index_path = Path(index_path)
        self._query_cache = OrderedDict()
        self._query_cache_bytes = 0
        self._query_cache_lock = threading.Lock()
        with closing(self._connect()) as conn:
            required = {'metadata', 'chunks', 'spatial', 'overview'}
            tables = {row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            if not required <= tables:
                raise ValueError('Índice incompleto: faltan tablas')
            row = conn.execute("SELECT value FROM metadata WHERE key='manifest'").fetchone()
            if row is None:
                raise ValueError('Índice incompleto')
            self.metadata = json.loads(row[0])
            if not isinstance(self.metadata, dict) or self.metadata.get('version') != VERSION or not self.metadata.get('complete'):
                raise ValueError('Índice incompatible o incompleto')
            self._validate_index(conn)

    def _cached_query(self, key):
        if key is None:
            return None
        with self._query_cache_lock:
            entry = self._query_cache.get(key)
            if entry is None:
                return None
            self._query_cache.move_to_end(key)
            # Callers may modify returned arrays; cached source data stays private.
            return tuple(v.copy() if isinstance(v,np.ndarray) else v for v in entry[0])

    def _cache_query(self, key, result):
        if key is None:
            return
        size = len(key[-1]) + sum(v.nbytes for v in result if isinstance(v,np.ndarray))
        with self._query_cache_lock:
            if size > self._query_cache_limit or self._query_cache_entries < 1:
                return
            previous = self._query_cache.pop(key,None)
            if previous is not None:
                self._query_cache_bytes -= previous[1]
            while self._query_cache and (self._query_cache_bytes+size > self._query_cache_limit or
                                        len(self._query_cache) >= self._query_cache_entries):
                _, (_, removed_size) = self._query_cache.popitem(last=False)
                self._query_cache_bytes -= removed_size
            stored = tuple(v.copy() if isinstance(v,np.ndarray) else v for v in result)
            self._query_cache[key] = (stored,size)
            self._query_cache_bytes += size

    def _validate_index(self, conn):
        """Check completeness using BLOB lengths, without reading payload pages.

        A complete manifest alone cannot prove that all terrain blocks survived.
        SQLite computes length(BLOB) from its record header, so these checks do
        not load the potentially gigabyte-sized point arrays into memory.
        """
        count = self.metadata.get('count')
        expected_chunks = self.metadata.get('chunks')
        if type(count) is not int or count < 3 or type(expected_chunks) is not int or expected_chunks < 1:
            raise ValueError('Índice incompleto: recuentos inválidos')
        chunks, xyz_bytes, malformed = conn.execute(
            'SELECT count(*),coalesce(sum(length(xyz)),0),'
            'coalesce(sum(CASE WHEN typeof(xyz)!=\'blob\' OR typeof(labels)!=\'blob\' '
            'OR length(xyz)=0 OR length(xyz)%24!=0 OR length(labels)*3!=length(xyz) '
            'THEN 1 ELSE 0 END),0) FROM chunks').fetchone()
        if chunks != expected_chunks or xyz_bytes != count*24 or malformed:
            raise ValueError('Índice incompleto: faltan bloques o datos de puntos')
        spatial_count = conn.execute('SELECT count(*) FROM spatial').fetchone()[0]
        missing = conn.execute(
            'SELECT count(*) FROM chunks c LEFT JOIN spatial s ON c.id=s.id WHERE s.id IS NULL').fetchone()[0]
        if spatial_count != chunks or missing:
            raise ValueError('Índice incompleto: bloques espaciales inconsistentes')
        overview = conn.execute('SELECT length(xyz),length(labels),typeof(xyz),typeof(labels) FROM overview').fetchall()
        if len(overview) != 1:
            raise ValueError('Índice incompleto: vista global ausente')
        xyz_size, label_size, xyz_type, label_type = overview[0]
        if (xyz_type != 'blob' or label_type != 'blob' or xyz_size%24 or
                not 1 <= xyz_size//24 <= min(count,30000) or label_size*3 != xyz_size):
            raise ValueError('Índice incompleto: vista global inválida')

    def _connect(self):
        # Read-only connections cannot accidentally create an empty missing index.
        return sqlite3.connect(self.index_path.resolve().as_uri() + '?mode=ro', uri=True)

    @classmethod
    def open(cls, path, cache_dir=None, progress=None, cancel=None, label_mapping=None):
        source = Path(path).expanduser().resolve(strict=True)
        if not source.is_file() or source.suffix.lower() != '.ply':
            raise ValueError('Selecciona un archivo PLY local')
        labels = _mapping(source, label_mapping)
        cache = Path(cache_dir) if cache_dir is not None else DEFAULT_CACHE
        cache.mkdir(parents=True, exist_ok=True)
        initial = source.stat()
        digest = hashlib.sha256()
        with source.open('rb') as stream:
            done = 0
            while True:
                _check_cancel(cancel)
                block = stream.read(4 * 1024 * 1024)
                if not block:
                    break
                digest.update(block)
                done += len(block)
                if progress:
                    progress({'stage':'fingerprint', 'processed':done, 'total':initial.st_size})
        sha = digest.hexdigest()
        mapping_sha = hashlib.sha256(json.dumps(labels, sort_keys=True).encode()).hexdigest()[:16]
        target = cache / f'v{VERSION}-{sha}-{mapping_sha}.sqlite'
        if target.exists():
            try:
                store = cls(target)
                # The same content can be safely relocated; expose the selected path.
                store.metadata['path'] = str(source)
                store._verify_source(source, initial)
                return store
            except (sqlite3.DatabaseError, ValueError):
                # Damaged indexes are rebuilt; original cloud remains untouched.
                pass
        temp = cache / (target.name + '.' + uuid.uuid4().hex + '.partial')
        started = time.monotonic()
        conn = None
        try:
            conn = sqlite3.connect(temp)
            conn.execute('PRAGMA journal_mode=OFF')
            conn.execute('PRAGMA synchronous=OFF')
            conn.execute('PRAGMA cache_size=-8192')
            conn.execute('CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT)')
            conn.execute('CREATE TABLE chunks(id INTEGER PRIMARY KEY, xyz BLOB NOT NULL, labels BLOB NOT NULL)')
            conn.execute('CREATE VIRTUAL TABLE spatial USING rtree(id,xmin,xmax,ymin,ymax,zmin,zmax)')
            conn.execute('CREATE TABLE overview(xyz BLOB NOT NULL, labels BLOB NOT NULL)')
            lo = np.full(3, np.inf)
            hi = np.full(3, -np.inf)
            imported = 0
            chunk_id = 0
            sampled_points, sampled_labels = [], []
            with source.open('rb') as stream:
                fmt, count, dtype = _header(stream)
                prop_names = dtype.names
                label_prop = next((p for p in ('classification','class','label') if p in prop_names), None)
                stride = max(1, math.ceil(count / 30000))
                for start in range(0, count, 100000):
                    _check_cancel(cancel)
                    size = min(100000, count - start)
                    if fmt == 'ascii':
                        records = []
                        for _ in range(size):
                            raw = stream.readline(65537)
                            values = raw.split()
                            if len(raw) > 65536 or len(values) != len(prop_names):
                                raise ValueError('Fila PLY ASCII incompleta o incompatible')
                            try:
                                records.append(tuple(values))
                            except Exception as exc:
                                raise ValueError('Datos PLY ASCII inválidos') from exc
                        try:
                            data = np.array(records, dtype=dtype)
                        except (ValueError, OverflowError) as exc:
                            raise ValueError('Datos PLY ASCII inválidos') from exc
                    else:
                        raw = stream.read(size * dtype.itemsize)
                        if len(raw) != size * dtype.itemsize:
                            raise ValueError('Archivo PLY truncado')
                        data = np.frombuffer(raw, dtype=dtype)
                    points = np.column_stack([data[n] for n in ('x','y','z')]).astype('<f8')
                    if not np.isfinite(points).all():
                        raise ValueError('Se requieren coordenadas XYZ finitas')
                    if label_prop:
                        raw_labels = data[label_prop]
                        if not np.isfinite(raw_labels).all() or np.any(raw_labels != np.floor(raw_labels)) or np.any(np.abs(raw_labels) > 2**53):
                            raise ValueError('Las etiquetas deben ser IDs enteros finitos')
                        class_ids = raw_labels.astype('<i8')
                    else:
                        class_ids = np.full(size, -1, dtype='<i8')
                    lo = np.minimum(lo, points.min(axis=0))
                    hi = np.maximum(hi, points.max(axis=0))
                    sample_offset = (-start) % stride
                    sampled_points.append(points[sample_offset::stride].copy())
                    sampled_labels.append(class_ids[sample_offset::stride].copy())
                    tiles = np.floor(points[:, :2] / 10)
                    order = np.lexsort((tiles[:,1], tiles[:,0]))
                    sorted_tiles = tiles[order]
                    splits = np.flatnonzero(np.any(sorted_tiles[1:] != sorted_tiles[:-1], axis=1)) + 1
                    for ids in np.split(order, splits):
                        p = points[ids]
                        lab = class_ids[ids]
                        chunk_id += 1
                        pmin, pmax = p.min(0), p.max(0)
                        conn.execute('INSERT INTO chunks VALUES(?,?,?)', (chunk_id,p.tobytes(),lab.tobytes()))
                        conn.execute('INSERT INTO spatial VALUES(?,?,?,?,?,?,?)',
                                     (chunk_id,float(pmin[0]),float(pmax[0]),float(pmin[1]),float(pmax[1]),float(pmin[2]),float(pmax[2])))
                    imported += size
                    if progress:
                        progress({'stage':'index', 'processed':imported, 'total':count})
            cls._verify_source(source, initial)
            overview = np.concatenate(sampled_points)
            overview_labels = np.concatenate(sampled_labels)
            conn.execute('INSERT INTO overview VALUES(?,?)', (overview.tobytes(),overview_labels.tobytes()))
            manifest = {'version':VERSION, 'complete':True, 'count':imported,
                        'path':str(source), 'sha256':sha, 'bounds':[lo.tolist(),hi.tolist()],
                        'center':((lo+hi)/2).tolist(), 'label_property':label_prop,
                        'label_mapping':labels, 'coordinate_frame':'PLY XYZ; metres; Z up assumed',
                        'source_size':initial.st_size, 'source_mtime_ns':initial.st_mtime_ns,
                        'index_seconds':time.monotonic()-started, 'chunks':chunk_id}
            conn.execute('INSERT INTO metadata VALUES(?,?)', ('manifest',json.dumps(manifest)))
            conn.commit()
            conn.close()
            conn = None
            _check_cancel(cancel)
            os.replace(temp, target)
            return cls(target)
        except BaseException:
            if conn is not None:
                conn.close()
            temp.unlink(missing_ok=True)
            raise

    @staticmethod
    def _verify_source(path, initial):
        current = path.stat()
        if current.st_size != initial.st_size or current.st_mtime_ns != initial.st_mtime_ns:
            raise ValueError('La nube cambió durante la indexación; vuelve a importarla')

    def all_points(self):
        """Return the complete cloud with aligned labels for persistent rendering."""
        points_out, labels_out = [], []
        with closing(self._connect()) as conn:
            for xyz, labels in conn.execute('SELECT xyz,labels FROM chunks ORDER BY id'):
                points_out.append(np.frombuffer(xyz, dtype='<f8').reshape(-1, 3))
                labels_out.append(np.frombuffer(labels, dtype='<i8'))
        return np.concatenate(points_out), np.concatenate(labels_out)

    def overview(self, limit=30000):
        if not isinstance(limit, int) or limit < 1:
            raise ValueError('Límite de vista inválido')
        with closing(self._connect()) as conn:
            xyz, labels = conn.execute('SELECT xyz,labels FROM overview').fetchone()
        points = np.frombuffer(xyz, dtype='<f8').reshape(-1,3).copy()
        labels = np.frombuffer(labels, dtype='<i8').copy()
        step = max(1, math.ceil(len(points)/limit))
        return points[::step], labels[::step]

    def _query(self, low, high, limit=None, sphere=None):
        if limit is not None and (not isinstance(limit,int) or limit < 1):
            raise ValueError('Límite de consulta inválido')
        points_out, labels_out = [], []
        total = 0
        # RTree float32 bounds are outward-rounded by SQLite, exact test below.
        sql = ('SELECT c.xyz,c.labels FROM spatial s JOIN chunks c ON c.id=s.id '
               'WHERE s.xmax>=? AND s.xmin<=? AND s.ymax>=? AND s.ymin<=? AND s.zmax>=? AND s.zmin<=? ORDER BY c.id')
        with closing(self._connect()) as conn:
            for xyz, labels in conn.execute(sql, (float(low[0]),float(high[0]),float(low[1]),float(high[1]),float(low[2]),float(high[2]))):
                points = np.frombuffer(xyz,dtype='<f8').reshape(-1,3)
                class_ids = np.frombuffer(labels,dtype='<i8')
                mask = np.all((points >= low) & (points <= high),axis=1)
                if sphere is not None:
                    center, radius = sphere
                    mask &= np.sum((points-center)**2,axis=1) <= radius**2
                selected = points[mask]
                total += len(selected)
                if limit is not None and total > limit:
                    raise ValueError(f'Densidad de nube superior al límite de consulta ({limit} puntos); reduzca la región explícitamente')
                if len(selected):
                    points_out.append(selected)
                    labels_out.append(class_ids[mask])
        if not points_out:
            return np.empty((0,3),dtype=np.float64), np.empty(0,dtype=np.int64)
        return np.concatenate(points_out),np.concatenate(labels_out)

    def sphere(self, center, radius=5, limit=None):
        center = np.asarray(center,dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all() or not math.isfinite(radius) or radius <= 0:
            raise ValueError('Centro o radio esférico inválido')
        return self._query(center-radius,center+radius,limit,(center,radius))

    def sphere_overview(self, center, radius=5, limit=100000):
        """Bounded deterministic reservoir for rendering; reports exact total count.

        Sampling applies only after the exact XYZ sphere test. This is a visual
        query and must never replace nearest_xy for physical terrain evaluation.
        """
        center = np.asarray(center,dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all() or not math.isfinite(radius) or radius <= 0:
            raise ValueError('Centro o radio esférico inválido')
        if not isinstance(limit,int) or limit < 1:
            raise ValueError('Límite de vista inválido')
        key = ('sphere_overview',float(radius),limit,center.tobytes())
        cached = self._cached_query(key)
        if cached is not None:
            return cached
        low, high = center-radius,center+radius
        sql = ('SELECT c.xyz,c.labels FROM spatial s JOIN chunks c ON c.id=s.id '
               'WHERE s.xmax>=? AND s.xmin<=? AND s.ymax>=? AND s.ymin<=? AND s.zmax>=? AND s.zmin<=? ORDER BY c.id')
        kept = np.empty((0,3),dtype=np.float64)
        kept_labels = np.empty(0,dtype=np.int64)
        priorities = np.empty(0,dtype=np.float64)
        random = np.random.default_rng(0)
        total = 0
        with closing(self._connect()) as conn:
            for xyz, labels in conn.execute(sql,(float(low[0]),float(high[0]),float(low[1]),float(high[1]),float(low[2]),float(high[2]))):
                points = np.frombuffer(xyz,dtype='<f8').reshape(-1,3)
                class_ids = np.frombuffer(labels,dtype='<i8')
                mask = np.sum((points-center)**2,axis=1) <= radius**2
                selected = points[mask]
                if not len(selected):
                    continue
                total += len(selected)
                kept = np.concatenate([kept,selected])
                kept_labels = np.concatenate([kept_labels,class_ids[mask]])
                priorities = np.concatenate([priorities,random.random(len(selected))])
                if len(kept) > limit:
                    ids = np.argpartition(priorities,limit-1)[:limit]
                    kept, kept_labels, priorities = kept[ids], kept_labels[ids], priorities[ids]
        result = (kept,kept_labels,total)
        self._cache_query(key,result)
        return result

    def region_xy(self, bounds, limit=None):
        bounds = np.asarray(bounds,dtype=np.float64)
        if bounds.shape != (4,) or not np.isfinite(bounds).all() or np.any(bounds[:2] > bounds[2:]):
            raise ValueError('Límites XY inválidos')
        return self._query(np.array([bounds[0],bounds[1],-np.inf]),np.array([bounds[2],bounds[3],np.inf]),limit)

    query_xy = region_xy

    def nearest_xy(self, query_xy, max_distance=2.0):
        """Exact nearest XY points within a bounded radius, without loading a patch.

        At most one import chunk's KDTree is alive at a time. Exact repeated
        queries reuse a bounded result cache, never an approximate centre/grid.
        Missing neighbours return distance inf, XYZ nan and label -1.
        """
        from scipy.spatial import cKDTree
        query = np.asarray(query_xy, dtype=np.float64)
        if query.ndim != 2 or query.shape[1] != 2 or not np.isfinite(query).all():
            raise ValueError('Consultas XY inválidas')
        radius = float(max_distance)
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Radio de vecindad inválido')
        # Include every float64 coordinate, its order and radius: no hashing or
        # rounding can alias distinct physical queries. Skip oversized cache keys.
        key = ('nearest_xy',radius,query.shape,query.tobytes()) if len(query)*56 <= self._query_cache_limit else None
        cached = self._cached_query(key)
        if cached is not None:
            return cached
        distances = np.full(len(query), np.inf)
        nearest = np.full((len(query), 3), np.nan)
        labels_out = np.full(len(query), -1, dtype=np.int64)
        if not len(query):
            return distances, nearest, labels_out
        low, high = query.min(0)-radius, query.max(0)+radius
        sql = ('SELECT c.xyz,c.labels,s.xmin,s.xmax,s.ymin,s.ymax '
               'FROM spatial s JOIN chunks c ON c.id=s.id '
               'WHERE s.xmax>=? AND s.xmin<=? AND s.ymax>=? AND s.ymin<=? ORDER BY c.id')
        with closing(self._connect()) as conn:
            for xyz, labels, xmin, xmax, ymin, ymax in conn.execute(sql, (float(low[0]),float(high[0]),float(low[1]),float(high[1]))):
                q_ids = np.flatnonzero((query[:,0]>=xmin-radius)&(query[:,0]<=xmax+radius)&
                                       (query[:,1]>=ymin-radius)&(query[:,1]<=ymax+radius))
                if not len(q_ids):
                    continue
                points = np.frombuffer(xyz,dtype='<f8').reshape(-1,3)
                class_ids = np.frombuffer(labels,dtype='<i8')
                tree = cKDTree(points[:,:2])
                # nextafter preserves points exactly on the inclusive radius.
                d, ids = tree.query(query[q_ids], distance_upper_bound=np.nextafter(radius,np.inf))
                improved = (d <= radius) & (d < distances[q_ids])
                update = q_ids[improved]
                distances[update] = d[improved]
                nearest[update] = points[ids[improved]]
                labels_out[update] = class_ids[ids[improved]]
        result = (distances,nearest,labels_out)
        self._cache_query(key,result)
        return result

    def upper_xy(self, query_xy, max_distance=.35):
        """Highest observed surface in each XY neighbourhood, with source labels.

        Unlike nearest_xy, coincident returns on lower layers cannot replace the
        upper surface used to place a vehicle. Queries and decoded chunks stay
        bounded; missing support returns inf distance, nan XYZ and label -1.
        """
        from scipy.spatial import cKDTree
        query = np.asarray(query_xy, dtype=np.float64)
        if query.ndim != 2 or query.shape[1] != 2 or not np.isfinite(query).all():
            raise ValueError('Consultas XY inválidas')
        radius = float(max_distance)
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Radio de vecindad inválido')
        key = ('upper_xy',radius,query.shape,query.tobytes()) if len(query)*56 <= self._query_cache_limit else None
        cached = self._cached_query(key)
        if cached is not None:
            return cached
        distances = np.full(len(query), np.inf)
        observed = np.full((len(query), 3), np.nan)
        labels_out = np.full(len(query), -1, dtype=np.int64)
        if not len(query):
            return distances, observed, labels_out
        low, high = query.min(0)-radius, query.max(0)+radius
        sql = ('SELECT c.xyz,c.labels,s.xmin,s.xmax,s.ymin,s.ymax '
               'FROM spatial s JOIN chunks c ON c.id=s.id '
               'WHERE s.xmax>=? AND s.xmin<=? AND s.ymax>=? AND s.ymin<=? ORDER BY c.id')
        with closing(self._connect()) as conn:
            for xyz, labels, xmin, xmax, ymin, ymax in conn.execute(sql, (float(low[0]),float(high[0]),float(low[1]),float(high[1]))):
                q_ids = np.flatnonzero((query[:,0]>=xmin-radius)&(query[:,0]<=xmax+radius)&
                                       (query[:,1]>=ymin-radius)&(query[:,1]<=ymax+radius))
                if not len(q_ids):
                    continue
                points = np.frombuffer(xyz,dtype='<f8').reshape(-1,3)
                class_ids = np.frombuffer(labels,dtype='<i8')
                tree = cKDTree(points[:,:2])
                # Bound neighbourhood lists independently of the query grid size.
                for start in range(0,len(q_ids),512):
                    batch = q_ids[start:start+512]
                    neighbours = tree.query_ball_point(query[batch],radius)
                    for q_id, ids in zip(batch,neighbours):
                        if not ids:
                            continue
                        ids = np.asarray(ids)
                        top = np.max(points[ids,2])
                        ids = ids[points[ids,2] == top]
                        d = np.linalg.norm(points[ids,:2]-query[q_id],axis=1)
                        best = int(np.argmin(d))
                        if not np.isfinite(distances[q_id]) or top > observed[q_id,2] or (top == observed[q_id,2] and d[best] < distances[q_id]):
                            distances[q_id] = d[best]
                            observed[q_id] = points[ids[best]]
                            labels_out[q_id] = class_ids[ids[best]]
        result = (distances,observed,labels_out)
        self._cache_query(key,result)
        return result

    def close(self):
        """No persistent connections are held; safe for callers using finally."""
        with self._query_cache_lock:
            self._query_cache.clear()
            self._query_cache_bytes = 0

