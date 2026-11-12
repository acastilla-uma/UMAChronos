"""Portable project archives with embedded cloud, scene state and results."""
import copy
import hashlib
import io
import zipfile
import json
import math
import tempfile
from pathlib import Path

def validate(value, depth=0):
    if depth > 30:
        raise ValueError('Documento demasiado anidado')
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('El documento contiene números no finitos')
    if isinstance(value, dict):
        for item in value.values():
            validate(item, depth + 1)
    elif isinstance(value, list):
        for item in value:
            validate(item, depth + 1)


# A portable project contains only these known members; archive paths are never
# extracted. Sizes are checked before decompressing untrusted content.
ARCHIVE_VERSION = 2
CLOUD_LIMIT = 100_000_000
DOCUMENT_LIMIT = 64_000_000
CSV_LIMIT = 128_000_000
ARCHIVE_LIMIT = CLOUD_LIMIT + DOCUMENT_LIMIT + CSV_LIMIT + 1_000_000
MEMBER_LIMITS = {'project.json': DOCUMENT_LIMIT, 'cloud.ply': CLOUD_LIMIT,
                 'results.csv': CSV_LIMIT}


def validate_project(project):
    if not isinstance(project, dict):
        raise ValueError('Contenido de proyecto inválido')
    validate(project)
    if 'csv_available' in project and not isinstance(project['csv_available'], bool):
        raise ValueError('Estado CSV de proyecto inválido')
    if 'playback_index' in project and (type(project['playback_index']) is not int or project['playback_index'] < 0):
        raise ValueError('Posición de reproducción inválida')
    applied_pose = project.get('applied_pose')
    if applied_pose is not None:
        if not isinstance(applied_pose, dict):
            raise ValueError('Pose aplicada inválida')
        for key, size in (('position', 3), ('orientation', 4)):
            vector = applied_pose.get(key)
            if (not isinstance(vector, list) or len(vector) != size or any(
                    not isinstance(v, (int, float)) or isinstance(v, bool) for v in vector)):
                raise ValueError('Pose aplicada inválida')
    for key in ('pose', 'terrain'):
        if key in project and not isinstance(project[key], dict):
            raise ValueError('Campo de proyecto inválido: ' + key)
    if 'vehicle' in project and (not isinstance(project['vehicle'], str) or
                                  project['vehicle'] not in ('MAN_5t', 'MAN_7t', 'MAN_10t')):
        raise ValueError('Vehículo de proyecto no compatible')
    for value in project.get('pose', {}).values():
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError('Pose de proyecto inválida')
    for key in ('route', 'frames', 'trajectory'):
        values = project.get(key, [])
        if not isinstance(values, list) or any(not isinstance(item, dict) for item in values):
            raise ValueError('Campo de proyecto inválido: ' + key)
    if len(project.get('route', [])) > 10000:
        raise ValueError('Demasiados puntos de trayectoria')
    for point in project.get('route', []):
        for key in ('x', 'y', 'z', 'speed', 'acceleration'):
            value = point.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError('Punto de trayectoria inválido')
        if not 0 <= point['speed'] <= 100 or not -20 <= point['acceleration'] <= 20:
            raise ValueError('Objetivo de trayectoria inválido')
    for frame in project.get('frames', []) + project.get('trajectory', []):
        for key in ('time', 'tracking_error', 'speed_error', 'tangential_acceleration', 'acceleration_error'):
            if key in frame and (not isinstance(frame[key], (int, float)) or isinstance(frame[key], bool)):
                raise ValueError('Métrica de simulación inválida')
        for key in ('points', 'normal', 'friction', 'moments'):
            if key in frame and (not isinstance(frame[key], list) or any(
                    not isinstance(vector, list) or len(vector) != 3 or any(
                        not isinstance(v, (int, float)) or isinstance(v, bool) for v in vector)
                    for vector in frame[key])):
                raise ValueError('Contactos de simulación inválidos')
        if 'contact' in frame and (not isinstance(frame['contact'], list) or any(
                not isinstance(v, (int, float)) or isinstance(v, bool) for v in frame['contact'])):
            raise ValueError('Contactos de simulación inválidos')
        for key, size in (('position', 3), ('orientation', 4), ('velocity', 3)):
            if key not in frame:
                continue
            vector = frame[key]
            if (not isinstance(vector, list) or len(vector) != size or
                    any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in vector)):
                raise ValueError('Estado de simulación inválido: ' + key)
    if 'warnings' in project and not isinstance(project['warnings'], str):
        raise ValueError('Avisos de proyecto inválidos')
    terrain = project.get('terrain', {})
    if not isinstance(terrain.get('classes', {}), dict) or not isinstance(terrain.get('zones', []), list):
        raise ValueError('Terreno de proyecto inválido')
    allowed_classes = ('desconocido', 'grava', 'roca', 'plantas', 'barro', 'tierra')
    for name, properties in terrain.get('classes', {}).items():
        if name not in allowed_classes or not isinstance(properties, dict):
            raise ValueError('Clase de terreno inválida')
        for key, lo, hi in (('friction', 0, 3), ('stiffness', 1000, 1e8)):
            v = properties.get(key)
            if not isinstance(v, (int, float)) or isinstance(v, bool) or not lo <= v <= hi:
                raise ValueError('Propiedad de terreno inválida')
    for zone in terrain.get('zones', []):
        if not isinstance(zone, dict):
            raise ValueError('Zona de terreno inválida')
        center, radius = zone.get('center'), zone.get('radius')
        if (not isinstance(center, list) or len(center) != 3 or
                any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in center) or
                not isinstance(radius, (int, float)) or isinstance(radius, bool) or radius <= 0 or
                zone.get('class') not in ('desconocido', 'grava', 'roca', 'plantas', 'barro', 'tierra')):
            raise ValueError('Zona de terreno inválida')


def _read_bounded(path, maximum):
    with Path(path).open('rb') as stream:
        data = stream.read(maximum + 1)
    if not data or len(data) > maximum:
        raise ValueError('Archivo vacío o demasiado grande para el proyecto')
    return data


def export_archive(project, cloud, csv_path=None):
    """Create one downloadable document containing source, state and results."""
    validate_project(project)
    if not isinstance(cloud, dict) or not cloud.get('path'):
        raise ValueError('Carga primero una nube')
    source = _read_bounded(cloud['path'], CLOUD_LIMIT)
    sha = hashlib.sha256(source).hexdigest()
    if sha != cloud.get('sha256'):
        raise ValueError('La nube original ha cambiado; vuelve a cargarla antes de guardar')
    data = copy.deepcopy(project)
    data.pop('config', None)
    data.pop('configuration', None)
    data['cloud'] = {key: copy.deepcopy(value) for key, value in cloud.items() if key != 'path'}
    doc = {'format': 'fusionforce-project', 'version': ARCHIVE_VERSION, 'project': data}
    encoded = json.dumps(doc, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(encoded) > DOCUMENT_LIMIT:
        raise ValueError('El estado del proyecto supera 64 MB')
    csv_data = _read_bounded(csv_path, CSV_LIMIT) if csv_path and project.get('csv_available') is True else None
    output = io.BytesIO()
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('project.json', encoded)
        archive.writestr('cloud.ply', source)
        if csv_data is not None:
            archive.writestr('results.csv', csv_data)
    return output.getvalue()


def read_archive(raw):
    """Validate every member and its fingerprint before a simulation changes."""
    if not raw or len(raw) > ARCHIVE_LIMIT:
        raise ValueError('Tamaño de proyecto inválido')
    try:
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            entries = archive.infolist()
            names = [member.filename for member in entries]
            if (len(set(names)) != len(names) or not {'project.json', 'cloud.ply'} <= set(names)
                    or not set(names) <= set(MEMBER_LIMITS)):
                raise ValueError('Contenido del archivo de proyecto inválido')
            for member in entries:
                if member.flag_bits & 1 or not 0 < member.file_size <= MEMBER_LIMITS[member.filename]:
                    raise ValueError('Miembro del proyecto vacío, cifrado o demasiado grande')
            document = json.loads(archive.read('project.json'))
            if (not isinstance(document, dict) or document.get('format') != 'fusionforce-project'
                    or document.get('version') != ARCHIVE_VERSION):
                raise ValueError('Formato o versión de proyecto no compatible')
            project = document.get('project')
            validate_project(project)
            reference = project.get('cloud')
            if not isinstance(reference, dict):
                raise ValueError('Faltan los datos de la nube en el proyecto')
            mapping = reference.get('label_mapping', {})
            if not isinstance(mapping, dict):
                raise ValueError('Tabla de clases de proyecto inválida')
            for key, value in mapping.items():
                if str(int(key)) != key or value not in ('desconocido', 'grava', 'roca', 'plantas', 'barro', 'tierra'):
                    raise ValueError('Tabla de clases de proyecto inválida')
            source = archive.read('cloud.ply')
            if hashlib.sha256(source).hexdigest() != reference.get('sha256'):
                raise ValueError('La huella de la nube no coincide con el proyecto')
            csv_data = archive.read('results.csv') if 'results.csv' in names else None
    except (zipfile.BadZipFile, KeyError, TypeError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError('Archivo de proyecto dañado o no compatible') from exc
    return project, source, csv_data


def import_archive(sim, raw, cache_dir):
    project, source, csv_data = read_archive(raw)
    directory = Path(cache_dir)
    directory.mkdir(parents=True, exist_ok=True)
    working = Path(tempfile.mkdtemp(prefix='project-', dir=directory))
    path = working / 'cloud.ply'
    try:
        path.write_bytes(source)
        path.with_suffix('.ply.labels.json').write_text(
            json.dumps(project['cloud'].get('label_mapping', {})), encoding='utf-8')
        csv_path = working / 'results.csv'
        if csv_data is not None:
            csv_path.write_bytes(csv_data)
        info = sim.load_path(path, expected_sha256=project['cloud']['sha256'],
                             expected_mapping=project['cloud'].get('label_mapping', {}))
    except Exception:
        # Only these fixed members were written; no archive path is extracted.
        for file in working.iterdir():
            file.unlink()
        working.rmdir()
        raise
    sim.csv_path = str(csv_path) if csv_data is not None else None
    return {'project': project, 'info': info, 'csv_available': csv_data is not None}
