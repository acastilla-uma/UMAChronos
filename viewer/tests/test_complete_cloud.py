import ast
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from cloud_store import CloudStore


class CompleteCloudTest(unittest.TestCase):
    def test_import_and_reopen_keep_every_point_and_label(self):
        # Load Simulation's actual cloud methods without the optional physics stack.
        tree = ast.parse((ROOT / 'server.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Simulation')
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('__init__', 'load_path', 'info')]
        with tempfile.TemporaryDirectory(dir=ROOT / 'tests') as folder:
            root = Path(folder)
            namespace = dict(np=np, ROOT=root, deepcopy=deepcopy, json=json,
                             catalog=lambda: [{}], TERRAIN_DEFAULTS={})
            exec(compile(ast.Module(body=[cls], type_ignores=[]), 'server.py', 'exec'), namespace)
            count = 60001
            source = root / 'map.ply'
            data = np.zeros(count, dtype=[('x', '<f8'), ('y', '<f8'), ('z', '<f8'), ('classification', '<i4')])
            data['x'] = np.arange(count) % 200
            data['y'] = np.arange(count) // 200
            data['z'] = np.arange(count) * .01
            data['classification'] = np.arange(count) % 5
            header = f'ply\nformat binary_little_endian 1.0\nelement vertex {count}\nproperty double x\nproperty double y\nproperty double z\nproperty int classification\nend_header\n'
            source.write_bytes(header.encode() + data.tobytes())
            sim = namespace['Simulation']()
            for _ in range(2):
                info = sim.load_path(source)
                points = np.array(info['points'])
                labels = np.array(info['labels'])
                self.assertEqual(len(points), count)
                self.assertEqual(len(labels), count)
                # Z uniquely identifies the original point despite spatial reordering.
                ids = np.rint(points[:, 2] * 100).astype(int)
                np.testing.assert_array_equal(np.sort(ids), np.arange(count))
                np.testing.assert_array_equal(labels, ids % 5)
                self.assertGreater(np.linalg.norm(points.max(0) - points.min(0)), 500)
            sim.store.close()


if __name__ == '__main__':
    unittest.main()

