import ast
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from cloud_store import CloudStore


def write_cloud(path, points, labels):
    data = np.empty(len(points), dtype=[('x', '<f8'), ('y', '<f8'), ('z', '<f8'), ('classification', '<i4')])
    for i, name in enumerate(('x', 'y', 'z')):
        data[name] = np.asarray(points)[:, i]
    data['classification'] = labels
    header = f'ply\nformat binary_little_endian 1.0\nelement vertex {len(data)}\nproperty double x\nproperty double y\nproperty double z\nproperty int classification\nend_header\n'
    path.write_bytes(header.encode() + data.tobytes())


class UpperSurfaceTest(unittest.TestCase):
    def test_highest_layer_across_chunks_labels_and_cache(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'tests') as temporary:
            root = Path(temporary)
            path = root / 'stacked.ply'
            # The highest point lies in the next spatial chunk, farther in XY.
            write_cloud(path, [[9.9, 0, 0], [9.9, 0, 2], [10.1, 0, 3]], [1, 2, 3])
            store = CloudStore.open(path, cache_dir=root / 'cache')
            try:
                queries = np.array([[9.9, 0], [40, 40]])
                distance, points, labels = store.upper_xy(queries)
                self.assertAlmostEqual(distance[0], .2)
                np.testing.assert_allclose(points[0], [10.1, 0, 3])
                self.assertEqual(labels[0], 3)
                self.assertTrue(np.isinf(distance[1]))
                self.assertTrue(np.isnan(points[1]).all())
                self.assertEqual(labels[1], -1)
                points[0, 2] = -100
                self.assertEqual(store.upper_xy(queries)[1][0, 2], 3)
                # A tight neighbourhood cannot borrow support from the next tile.
                self.assertEqual(store.upper_xy(queries, max_distance=.1)[1][0, 2], 2)
            finally:
                store.close()

    def test_physical_patch_keeps_upper_height_and_material(self):
        # Execute the actual nested patch builder without importing the physics SDK.
        source = ast.parse((ROOT / 'server.py').read_text(encoding='utf-8'))
        simulation = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Simulation')
        evaluate = next(n for n in simulation.body if isinstance(n, ast.FunctionDef) and n.name == 'evaluate')
        patch_builder = next(n for n in evaluate.body if isinstance(n, ast.FunctionDef) and n.name == 'physical_patch')
        factory = ast.parse('def make_patch():\n    patch = None\n    return physical_patch\n').body[0]
        factory.body.insert(1, patch_builder)
        module = ast.fix_missing_locations(ast.Module(body=[factory], type_ignores=[]))
        with tempfile.TemporaryDirectory(dir=ROOT / 'tests') as temporary:
            root = Path(temporary)
            path = root / 'stacked.ply'
            x, y = np.meshgrid(np.arange(-2, 2.01, .2), np.arange(-2, 2.01, .2))
            lower = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
            points = np.concatenate([lower, lower + [0, 0, 2]])
            write_cloud(path, points, np.repeat([1, 2], len(lower)))
            store = CloudStore.open(path, cache_dir=root / 'cache')
            try:
                namespace = dict(np=np, cKDTree=cKDTree, self=SimpleNamespace(store=store), origin=np.array([0, 0, 2.629]),
                                 torch=SimpleNamespace(tensor=lambda value, **kwargs: value, float32=None),
                                 WorldConfig=lambda x, y, z, *args: SimpleNamespace(z=z))
                exec(compile(module, 'server.py', 'exec'), namespace)
                tree, observed, labels, world = namespace['make_patch']()(np.zeros(2))
                np.testing.assert_allclose(observed[:, 2], 2)
                np.testing.assert_array_equal(labels, 2)
                np.testing.assert_allclose(world.z, -.629)
            finally:
                store.close()


if __name__ == '__main__':
    unittest.main()
