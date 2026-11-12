"""Placement resolves contacts before the recorded simulation starts."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import Simulation


class Ground:
    metadata = {}

    def __init__(self, height=0.):
        self.height = height

    def elevations(self, xy):
        return self.height(xy[:, 0], xy[:, 1]) if callable(self.height) else np.full(len(xy), self.height)

    def region_xy(self, bounds):
        x, y = np.meshgrid(np.arange(bounds[0], bounds[2]+.1, .2),
                           np.arange(bounds[1], bounds[3]+.1, .2))
        xy = np.column_stack([x.ravel(), y.ravel()])
        points = np.column_stack([xy, self.elevations(xy)])
        return points, np.zeros(len(points), dtype=int)

    def upper_xy(self, xy, max_distance):
        return (np.zeros(len(xy)), np.column_stack([xy, self.elevations(xy)]),
                np.zeros(len(xy), dtype=int))


class RestingPlacementTest(unittest.TestCase):
    def simulation(self, height=0.):
        simulation = Simulation()
        simulation.store = Ground(height)
        return simulation

    def assert_supported_start(self, simulation, request):
        placed = simulation.place(request)
        self.assertTrue(placed['grounded'])
        self.assertFalse(placed['settled'])
        self.assertLessEqual(placed['settling_time'], 2.)
        playback = simulation.evaluate(dict(
            request, **dict(zip(('x', 'y', 'z'), placed['position'])),
            roll=placed['roll'], pitch=placed['pitch'], yaw=placed['yaw'],
            duration=.1, disable_csv=True))
        frames = playback['frames']
        np.testing.assert_allclose(frames[0]['position'], placed['position'], atol=1e-6)
        np.testing.assert_allclose(frames[0]['orientation'], placed['orientation'], atol=1e-6)
        self.assertEqual(frames[0]['time'], 0.)
        points = np.asarray(frames[0]['points'])
        gap = points[:, 2]-simulation.store.elevations(points[:, :2])
        self.assertLess(gap.min(), .02)
        self.assertGreater(np.asarray(frames[0]['normal'])[:, 2].sum(), 100.)
        self.assertIsNone(frames[0]['stability']['pltr']['value'])
        return placed

    def test_all_man_models_start_supported_without_drop(self):
        for vehicle in ('MAN_5t', 'MAN_7t', 'MAN_10t'):
            with self.subTest(vehicle=vehicle):
                self.assert_supported_start(self.simulation(),
                                            dict(x=0, y=0, yaw=35, vehicle=vehicle))

    def test_active_material_and_elevated_survey_coordinates(self):
        placed = self.assert_supported_start(self.simulation(300.), dict(
            x=500000., y=4100000., yaw=-25, vehicle='MAN_5t',
            terrain={'classes': {'desconocido': {'friction': .8, 'stiffness': 2e6}}}))
        self.assertGreater(placed['position'][2], 300.)

    def test_lightweight_contacts_start_supported(self):
        self.assert_supported_start(self.simulation(), dict(
            x=0, y=0, yaw=0, vehicle='MAN_5t', lightweight_physics=True))

    def test_gentle_slope_and_step_start_from_same_supported_pose(self):
        for surface in (lambda x, y: .0015*x,
                        lambda x, y: np.where(x < -2., .08, 0.)):
            with self.subTest(surface=surface):
                self.assert_supported_start(self.simulation(surface), dict(
                    x=.037, y=.013, yaw=0, vehicle='MAN_5t'))

    def test_unstable_slopes_are_accepted_without_waiting_for_equilibrium(self):
        for surface in (lambda x, y: .4*x, lambda x, y: .4*y):
            with self.subTest(surface=surface):
                self.assert_supported_start(self.simulation(surface), dict(
                    x=0, y=0, yaw=0, vehicle='MAN_5t'))

    def test_route_waypoint_does_not_require_parking_equilibrium(self):
        simulation = self.simulation(lambda x, y: .3*x)
        with patch.object(simulation, 'evaluate', side_effect=AssertionError('must not settle route target')):
            target = simulation.place(dict(x=0, y=0, vehicle='MAN_5t', purpose='waypoint'))
        self.assertNotIn('settled', target)

    def test_missing_dynamic_coverage_falls_back_to_observed_ground_contact(self):
        simulation = self.simulation()
        simulation.csv_path = Path('previous.csv')
        original = simulation.evaluate

        def unsupported(*args, **kwargs):
            with patch.object(simulation.store, 'upper_xy', side_effect=ValueError('sin apoyo')):
                return original(*args, **kwargs)

        with patch.object(simulation, 'evaluate', side_effect=unsupported):
            placed = simulation.place(dict(x=0, y=0, vehicle='MAN_5t'))
        self.assertTrue(placed['grounded'])
        self.assertFalse(placed['settled'])
        self.assertEqual(placed['settling_time'], 0.)
        self.assertAlmostEqual(min(p[2] for p in placed['wheel_contacts']), 0., places=5)
        self.assertIsNone(simulation.csv_path)


if __name__ == '__main__':
    unittest.main()
