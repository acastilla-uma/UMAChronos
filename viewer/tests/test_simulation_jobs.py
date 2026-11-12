"""Real simulation progress and non-blocking job publication contracts."""
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from webapp import create_server


class JobSimulation:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.fail = False
        self.placed = []

    def evaluate(self, payload, progress=None):
        progress(dict(stage='simulation', processed=2, total=5, fraction=.4, time=.2, duration=.5))
        self.entered.set()
        if not self.release.wait(3):
            raise TimeoutError('Test did not release simulation')
        if self.fail:
            raise ValueError('Sin cobertura')
        progress(dict(stage='complete', processed=5, total=5, fraction=1., time=.5, duration=.5))
        return {'frames': [{'time': .5}], 'csv': None, 'status': 'complete'}

    def place(self, payload):
        self.placed.append(payload)
        return {'position': [payload['x'], payload['y'], 2], 'orientation': [1, 0, 0, 0]}


class SimulationJobTest(unittest.TestCase):
    def setUp(self):
        self.sim = JobSimulation()
        self.server = create_server(self.sim, ROOT)

    def tearDown(self):
        self.sim.release.set()
        self.server.server_close()

    def request(self, method, path, data=None):
        raw = json.dumps(data).encode() if data is not None else b''
        cls = self.server.RequestHandlerClass
        handler = cls.__new__(cls)
        handler.path = path
        handler.headers = {'Host': f'127.0.0.1:{self.server.server_port}', 'Content-Length': str(len(raw))}
        handler.rfile = io.BytesIO(raw)
        handler.wfile = io.BytesIO()
        response = {}
        handler.send_response = lambda code: response.update(status=code)
        handler.send_header = lambda key, value: response.update({key: value})
        handler.end_headers = lambda: None
        getattr(handler, 'do_' + method)()
        return response['status'], json.loads(handler.wfile.getvalue())

    def poll_complete(self, job_id):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            status, data = self.request('GET', '/api/simulation/job?job_id=' + job_id)
            if data['status'] != 'running':
                return status, data
            time.sleep(.005)
        self.fail('Simulation job did not complete')

    def test_progress_is_readable_while_simulation_holds_mutation_lock(self):
        status, start = self.request('POST', '/api/simulation/start', {'duration': .5})
        self.assertEqual(status, 200)
        self.assertTrue(self.sim.entered.wait(2))
        status, running = self.request('GET', '/api/simulation/job?job_id=' + start['job_id'])
        self.assertEqual(status, 200)
        self.assertEqual(running['progress']['processed'], 2)
        self.assertEqual(running['progress']['fraction'], .4)
        self.assertNotIn('result', running)
        self.assertEqual(self.request('POST', '/api/simulation/start', {})[0], 409)
        self.assertEqual(self.request('POST', '/api/place', {'x': 1, 'y': 2})[0], 409)
        self.assertEqual(self.request('POST', '/api/load', {})[0], 409)
        self.assertEqual(self.request('GET', '/api/simulation/job?job_id=unrelated')[0], 404)
        self.sim.release.set()
        status, complete = self.poll_complete(start['job_id'])
        self.assertEqual(status, 200)
        self.assertEqual(complete['status'], 'complete')
        self.assertEqual(complete['progress']['fraction'], 1)
        self.assertEqual(complete['result']['frames'], [{'time': .5}])
        self.assertEqual(self.request('POST', '/api/place', {'x': 1, 'y': 2})[0], 200)

    def test_failed_job_releases_lock_and_does_not_publish_stale_result(self):
        self.sim.fail = True
        _, start = self.request('POST', '/api/simulation/start', {})
        self.assertTrue(self.sim.entered.wait(2))
        self.sim.release.set()
        _, failed = self.poll_complete(start['job_id'])
        self.assertEqual(failed['status'], 'error')
        self.assertEqual(failed['error'], 'Sin cobertura')
        self.assertLess(failed['progress']['fraction'], 1)
        self.assertNotIn('result', failed)
        self.sim.fail = False
        _, next_job = self.request('POST', '/api/simulation/start', {})
        self.assertNotEqual(start['job_id'], next_job['job_id'])
        self.assertEqual(self.request('GET', '/api/simulation/job?job_id=' + start['job_id'])[0], 404)
        _, complete = self.poll_complete(next_job['job_id'])
        self.assertEqual(complete['status'], 'complete')

    def test_worker_launch_failure_releases_reserved_lock(self):
        from unittest.mock import patch
        with patch('webapp.threading.Thread.start', side_effect=RuntimeError('Thread unavailable')):
            status, data = self.request('POST', '/api/simulation/start', {})
        self.assertEqual(status, 400)
        self.assertEqual(data['error'], 'Thread unavailable')
        self.assertEqual(self.request('POST', '/api/place', {'x': 1, 'y': 2})[0], 200)


class ActualProgressTest(unittest.TestCase):
    def test_physics_reports_completed_steps_and_only_finishes_after_csv(self):
        from server import Simulation
        from cloud_store import CloudStore
        import numpy as np
        with tempfile.TemporaryDirectory(dir=ROOT / 'tests') as temporary:
            root = Path(temporary)
            path = root / 'plane.ply'
            axis = np.arange(-10, 10.01, .5)
            x, y = np.meshgrid(axis, axis)
            xyz = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)]).astype('<f4')
            header = f'ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
            path.write_bytes(header.encode() + xyz.tobytes())
            sim = Simulation()
            sim.store = CloudStore.open(path, cache_dir=root / 'clouds')
            sim.center = np.array([0., 0., 0.])
            snapshots = []
            def progress(value):
                if value['stage'] == 'complete':
                    self.assertTrue(Path(sim.csv_path).is_file())
                snapshots.append(value)
            try:
                result = sim.evaluate(dict(x=0, y=0, z=1.2, vehicle='MAN_10t', duration=.004, dt=.002), progress=progress)
                self.assertEqual(result['samples'], 3)
                computed = [v for v in snapshots if v['stage'] == 'simulation']
                self.assertEqual([v['processed'] for v in computed], [1, 2, 3])
                self.assertTrue(all(v['total'] == 3 for v in computed))
                self.assertEqual(snapshots[0]['stage'], 'preparation')
                self.assertEqual(snapshots[-2]['stage'], 'finalizing')
                self.assertEqual(snapshots[-1]['stage'], 'complete')
                self.assertTrue(all(v['fraction'] < 1 for v in snapshots[:-1]))
                self.assertEqual(snapshots[-1]['fraction'], 1.)
                self.assertEqual(sorted(v['fraction'] for v in snapshots), [v['fraction'] for v in snapshots])
            finally:
                sim.store.close()
                if sim.csv_path:
                    Path(sim.csv_path).unlink(missing_ok=True)


if __name__ == '__main__':
    unittest.main()
