"""Loopback HTTP service and cancellable cloud indexing jobs."""
import json
import threading
import uuid
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

from projects import ARCHIVE_LIMIT, DOCUMENT_LIMIT, export_archive, import_archive


def create_server(sim, root, port=0):
    """Serve a simulation whose methods return independently owned JSON data.

    Responses remain valid after the simulation lock is released. Bulk point and
    frame lists must already belong to the response rather than mutable state;
    copying them here would cost more than encoding them.
    """
    root = Path(root)
    lock = threading.Lock()
    job_lock = threading.Lock()
    job = {'status': 'idle'}
    cancel = threading.Event()
    simulation_job = None

    def progress(*args, **kwargs):
        with job_lock:
            job['progress'] = kwargs or (args[0] if len(args) == 1 else list(args))

    def index(path):
        try:
            result = sim.load_path(path, progress=progress, cancel=cancel)
            with job_lock:
                job.update(status='complete', info=result)
        except Exception as e:
            with job_lock:
                job.update(status='cancelled' if cancel.is_set() else 'error', error=str(e))
        finally:
            lock.release()

    def simulate(job_id, payload):
        def update(value):
            with job_lock:
                if simulation_job['job_id'] == job_id:
                    # Keep fractional progress below completion until evaluate
                    # returns a fully published, independently owned result.
                    published = dict(value)
                    if published.get('stage') == 'complete':
                        published['stage'] = 'finalizing'
                    published['fraction'] = min(.999, max(0., float(value.get('fraction', 0.))))
                    simulation_job['progress'] = published
        try:
            result = sim.evaluate(payload, progress=update)
            with job_lock:
                final = dict(simulation_job['progress'], stage='complete', fraction=1.)
                simulation_job.update(status='complete', result=result, progress=final)
        except Exception as exc:
            with job_lock:
                simulation_job.update(status='error', error=str(exc))
        finally:
            lock.release()

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=str(root), **kw)

        def send_json(self, data, status=200):
            payload = json.dumps(data, allow_nan=False, separators=(',', ':')).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def authorized(self):
            host = self.headers.get('Host', '')
            expected = {'127.0.0.1:' + str(port), 'localhost:' + str(port)}
            origin = self.headers.get('Origin')
            if host not in expected or (origin and urlparse(origin).netloc != host):
                raise ValueError('Origen local no autorizado')
            if self.headers.get('Sec-Fetch-Site') == 'cross-site':
                raise ValueError('Petición de otro sitio rechazada')

        def do_GET(self):
            try:
                self.authorized()
                parsed = urlparse(self.path)
                if parsed.path == '/api/simulation/job':
                    job_id = parse_qs(parsed.query).get('job_id', [None])[0]
                    with job_lock:
                        result = dict(simulation_job) if simulation_job and simulation_job['job_id'] == job_id else None
                    if result is None:
                        self.send_json({'error': 'Simulación no encontrada'}, 404)
                    else:
                        self.send_json(result)
                elif self.path == '/api/job':
                    with job_lock:
                        # Published info/progress values are replaced, not mutated.
                        result = dict(job)
                    self.send_json(result)
                elif self.path == '/api/info':
                    if not lock.acquire(blocking=False):
                        self.send_json({'error': 'Operación en curso'}, 409)
                        return
                    try:
                        result = sim.info()
                    finally:
                        lock.release()
                    self.send_json(result)
                elif self.path == '/api/export.csv':
                    with lock:
                        path = getattr(sim, 'csv_path', None)
                        if not path or not Path(path).is_file():
                            raise ValueError('No hay un CSV disponible. Activa el registro CSV y vuelve a calcular.')
                        # Retain the completed export's handle while subsequent
                        # requests are free to produce a different export.
                        f = open(path, 'rb')
                    with f:
                        self.send_response(200)
                        self.send_header('Content-Type', 'text/csv; charset=utf-8')
                        self.send_header('Content-Disposition', 'attachment; filename="fusionforce.csv"')
                        f.seek(0, 2)
                        self.send_header('Content-Length', str(f.tell()))
                        f.seek(0)
                        self.end_headers()
                        while block := f.read(1024 * 1024):
                            self.wfile.write(block)
                elif self.path in ('/', '/index.html', '/app.js', '/styles.css', '/assets/uma-logo.png') or (
                        self.path.startswith('/assets/data/vehicle/') and
                        '..' not in self.path and '%' not in self.path and '?' not in self.path):
                    super().do_GET()
                else:
                    self.send_json({'error': 'Ruta desconocida'}, 404)
            except Exception as e:
                self.send_json({'error': str(e)}, 400)

        def do_POST(self):
            nonlocal simulation_job
            try:
                self.authorized()
                n = int(self.headers.get('Content-Length', '0'))
                maximum = (100_000_000 if self.path == '/api/load' else ARCHIVE_LIMIT
                           if self.path == '/api/project/import' else DOCUMENT_LIMIT
                           if self.path == '/api/project/export' else 8_000_000)
                if not 0 < n <= maximum:
                    raise ValueError('Tamaño de petición inválido; el archivo PLY debe ocupar entre 1 byte y 100 MB')
                raw = self.rfile.read(n)
                if len(raw) != n:
                    raise ValueError('Petición incompleta')
                data = None if self.path in ('/api/load', '/api/project/import') else json.loads(raw)
                if self.path == '/api/cancel':
                    cancel.set()
                    self.send_json({'status': 'cancelling'})
                    return
                if self.path == '/api/simulation/start':
                    if not isinstance(data, dict):
                        raise ValueError('Parámetros de simulación inválidos')
                    if not lock.acquire(blocking=False):
                        self.send_json({'error': 'Operación en curso'}, 409)
                        return
                    job_id = uuid.uuid4().hex
                    with job_lock:
                        simulation_job = dict(job_id=job_id, status='running',
                                              progress=dict(stage='preparation', processed=0,
                                                            total=0, fraction=0.))
                    try:
                        threading.Thread(target=simulate, args=(job_id, data), daemon=True).start()
                    except Exception as exc:
                        with job_lock:
                            simulation_job.update(status='error', error=str(exc))
                        lock.release()
                        raise
                    self.send_json({'job_id': job_id, 'status': 'running'})
                    return
                if self.path == '/api/index':
                    path = data.get('path') if isinstance(data, dict) else None
                    if not isinstance(path, str) or not path.strip():
                        raise ValueError('Introduce la ruta de una nube PLY')
                    if not lock.acquire(blocking=False):
                        self.send_json({'error': 'Operación en curso'}, 409)
                        return
                    with job_lock:
                        cancel.clear()
                        job.clear()
                        job.update(status='running', progress=0)
                    try:
                        threading.Thread(target=index, args=(path,), daemon=True).start()
                    except Exception as e:
                        with job_lock:
                            job.update(status='error', error=str(e))
                        lock.release()
                        raise
                    self.send_json({'status': 'running'})
                    return
                if not lock.acquire(blocking=False):
                    self.send_json({'error': 'Operación en curso'}, 409)
                    return
                try:
                    if self.path == '/api/load':
                        result = sim.load(raw)
                    elif self.path == '/api/view':
                        result = sim.local(data['center'], radius=5)
                    elif self.path == '/api/place':
                        result = sim.place(data)
                    elif self.path == '/api/evaluate':
                        result = sim.evaluate(data)
                    elif self.path == '/api/project/export':
                        cloud = getattr(getattr(sim, 'store', None), 'metadata', None)
                        if cloud is None:
                            cloud = sim.info().get('cloud')
                        result = export_archive(data['project'], cloud, getattr(sim, 'csv_path', None))
                    elif self.path == '/api/project/import':
                        result = import_archive(sim, raw, root / '.cache' / 'projects')
                    else:
                        raise ValueError('Ruta desconocida')
                finally:
                    lock.release()
                # Simulation methods return owned snapshots, so encoding and
                # slow client writes do not need the simulation lock.
                if self.path == '/api/project/export':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/zip')
                    self.send_header('Content-Disposition', 'attachment; filename="fusionforce.fusionforce"')
                    self.send_header('Content-Length', str(len(result)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.wfile.write(result)
                else:
                    self.send_json(result)
            except Exception as e:
                self.send_json({'error': str(e)}, 400)

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    port = server.server_port
    return server


def serve(sim, root, port):
    server = create_server(sim, root, port)
    print(f'Abre http://127.0.0.1:{server.server_port}', flush=True)
    server.serve_forever()
