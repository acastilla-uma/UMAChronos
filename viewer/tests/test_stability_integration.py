"""Indicators use real engine samples without changing the simulated motion."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import Simulation


class FlatTerrain:
    metadata = {}

    def upper_xy(self, xy, max_distance):
        return (np.zeros(len(xy)), np.column_stack([xy, np.zeros(len(xy))]),
                np.zeros(len(xy), dtype=int))


def simulate(vehicle='MAN_5t', lightweight=False):
    simulation = Simulation()
    simulation.store = FlatTerrain()
    return simulation.evaluate(dict(
        vehicle=vehicle, x=0, y=0, z=.65, yaw=35,
        dt=.01, duration=.4, speed=0, disable_csv=True,
        lightweight_physics=lightweight,
    ))


class StabilityIntegrationTest(unittest.TestCase):
    def test_real_models_publish_finite_indicators_and_physics_rate_derivatives(self):
        for vehicle, mass in [('MAN_5t', 7085), ('MAN_7t', 8285), ('MAN_10t', 10785)]:
            with self.subTest(vehicle=vehicle):
                result = simulate(vehicle)
                json.dumps(result, allow_nan=False)
                self.assertEqual(result['samples'], 41)
                self.assertEqual(len(result['frames']), 11)
                supported = [f['stability'] for f in result['frames']
                             if f['stability']['pltr']['value'] is not None]
                self.assertTrue(supported)
                for values in supported:
                    self.assertEqual(set(values), {'ltr', 'pltr', 'zml', 'si'})
                    self.assertAlmostEqual(values['pltr']['params']['derivative_dt_s'], .01)
                    self.assertEqual(values['si']['params']['mass_kg'], mass)
                    self.assertEqual(len(values['zml']['value']), 2)

    def test_lightweight_contact_layout_is_supported(self):
        result = simulate(lightweight=True)
        self.assertEqual(result['contact_mode'], 'four_corners')
        self.assertTrue(any(f['stability']['ltr']['value'] is not None
                            for f in result['frames']))
        json.dumps(result, allow_nan=False)

    def test_monitor_does_not_change_motion_or_contact_forces(self):
        actual = simulate()

        class NoIndicators:
            def __init__(self, **kwargs):
                pass

            def update(self, **kwargs):
                return {}

        with patch('server.StabilityMonitor', NoIndicators):
            baseline = simulate()
        self.assertEqual(actual['samples'], baseline['samples'])
        for measured, original in zip(actual['frames'], baseline['frames']):
            measured = dict(measured)
            measured.pop('stability')
            original = dict(original)
            original.pop('stability')
            self.assertEqual(measured, original)


if __name__ == '__main__':
    unittest.main()
