"""Analytical checks of diagnostics independent of the force integrator."""
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stability import StabilityMonitor


CENTERS = np.array([[1., 1., 0.], [1., -1., 0.], [-1., 1., 0.], [-1., -1., 0.]])


def quaternion(roll=0., pitch=0., yaw=0.):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def monitor(**overrides):
    args = dict(mass=10., inertia_body=np.diag([2., 3., 4.]),
                cog_body=[0., 0., .5], wheel_centers=CENTERS,
                tire_radius=.5, wheel_sample_count=4)
    return StabilityMonitor(**(args | overrides))


def sample(diagnostic, time=0., **overrides):
    args = dict(time=time, points=CENTERS, forces=np.tile([0., 0., 10.], (4, 1)),
                cog=[0., 0., 1.], orientation=[1., 0., 0., 0.], velocity=[0., 0., 0.],
                acceleration=[0., 0., 0.], angular_velocity=[0., 0., 0.])
    return diagnostic.update(**(args | overrides))


class StabilityTest(unittest.TestCase):
    def test_symmetric_loads_and_nominal_energy_threshold(self):
        result = sample(monitor())
        self.assertEqual(result['ltr']['value'], 0.)
        self.assertEqual(result['si']['value'], 1.)
        self.assertEqual(result['zml']['value'], [0., 0.])
        p = result['si']['params']
        self.assertEqual(p['mass_kg'], 10.)
        self.assertEqual(p['h_m'], 1.)
        self.assertAlmostEqual(p['phi_critical_rad'], math.pi/4)
        self.assertAlmostEqual(p['omega_critical_rad_s'],
                               math.sqrt(2*10*9.81*(math.sqrt(2)-1)/(2+10*2)))
        self.assertIsNone(result['pltr']['value'])
        self.assertIn('anterior', result['pltr']['reason'])

    def test_ltr_sign_excludes_chassis_and_is_yaw_invariant(self):
        forces = np.array([[0., 0., v] for v in (10, 30, 10, 30, 1000)])
        points = np.vstack([CENTERS, [0, 0, 0]])
        result = sample(monitor(), points=points, forces=forces)
        self.assertAlmostEqual(result['ltr']['value'], .5)
        self.assertEqual(result['ltr']['params']['fz_total_N'], 80.)
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        rotated = sample(monitor(), points=points @ rotation.T, forces=forces,
                         orientation=quaternion(yaw=math.pi/2))
        self.assertAlmostEqual(rotated['ltr']['value'], .5)

    def test_full_wheel_samples_keep_identity_and_ignore_body_contacts(self):
        diagnostic = monitor(wheel_sample_count=8)
        points = np.vstack([np.repeat(CENTERS, 2, axis=0), [0., 0., 0.]])
        forces = np.array([[0., 0., v] for v in (5, 5, 15, 15, 5, 5, 15, 15, 500)])
        result = sample(diagnostic, points=points, forces=forces)
        self.assertAlmostEqual(result['ltr']['value'], .5)

    def test_lightweight_four_corners_with_eight_actual_wheels(self):
        eight_centers = np.vstack([CENTERS, CENTERS + [2., 0., 0.]])
        result = sample(monitor(wheel_centers=eight_centers),
                        forces=np.array([[0., 0., v] for v in (5, 15, 5, 15)]))
        self.assertAlmostEqual(result['ltr']['value'], .5)

    def test_causal_pltr_has_known_linear_acceleration_and_roll_derivatives(self):
        diagnostic = monitor(horizon=.5)
        first = sample(diagnostic)
        roll = .02
        rotation = np.array([[1., 0., 0.], [0., math.cos(roll), -math.sin(roll)],
                             [0., math.sin(roll), math.cos(roll)]])
        result = sample(diagnostic, time=.1, orientation=quaternion(roll=roll),
                        acceleration=rotation @ [0., .3, 0.])
        p = result['pltr']['params']
        self.assertAlmostEqual(p['ay_dot_m_s3'], 3.)
        self.assertAlmostEqual(p['phi_dot_rad_s'], .2)
        expected = (.3/9.81+.02)+(3.+9.81*.2)/9.81*.5
        self.assertAlmostEqual(result['pltr']['value'], expected)
        preserved = json.dumps(result)
        sample(diagnostic, time=.2, acceleration=[0., 100., 0.])
        self.assertEqual(json.dumps(result), preserved)
        self.assertIsNone(first['pltr']['value'])

    def test_roll_derivative_unwraps_the_euler_branch(self):
        diagnostic = monitor()
        sample(diagnostic, orientation=quaternion(roll=math.radians(179)))
        result = sample(diagnostic, time=.1, orientation=quaternion(roll=math.radians(-179)))
        self.assertAlmostEqual(result['pltr']['params']['phi_dot_rad_s'], math.radians(2)/.1)

    def test_si_uses_body_roll_rate_and_keeps_negative_values(self):
        roll, yaw = .2, .9
        unrotated = sample(monitor(), orientation=quaternion(roll=roll),
                           angular_velocity=[4., 0., 0.])
        rotated = sample(monitor(), orientation=quaternion(roll=roll, yaw=yaw),
                         angular_velocity=[4*math.cos(yaw), 4*math.sin(yaw), 0.])
        self.assertAlmostEqual(unrotated['si']['value'], rotated['si']['value'])
        self.assertLess(rotated['si']['value'], 0.)
        self.assertAlmostEqual(rotated['si']['params']['omega_body_x_rad_s'], 4.)

    def test_zml_full_wrench_and_reference_plane(self):
        forces = np.tile([2., 3., 10.], (4, 1))
        points = CENTERS + [0., 0., .4]
        result = sample(monitor(), points=points, forces=forces)
        p = result['zml']['params']
        self.assertAlmostEqual(p['z_ref_m'], -.6)
        self.assertAlmostEqual(p['dx_dz'], .2)
        self.assertAlmostEqual(p['dy_dz'], .3)
        self.assertAlmostEqual(p['mx_Nm'], 7.2)
        self.assertAlmostEqual(p['my_Nm'], -4.8)
        np.testing.assert_allclose(result['zml']['value'], [0., 0.], atol=1e-14)
        translated = sample(monitor(), points=points+[100., -200., 3.],
                            forces=forces, cog=[100., -200., 4.])
        np.testing.assert_allclose(translated['zml']['value'], result['zml']['value'], atol=1e-13)

    def test_zml_reference_plane_weights_heights_by_positive_load(self):
        points = CENTERS.copy()
        points[:, 2] = [0., .5, 0., .5]
        forces = np.array([[0., 0., v] for v in (10., 30., 10., 30.)])
        result = sample(monitor(), points=points, forces=forces)
        self.assertAlmostEqual(result['zml']['params']['z_ref_m'], -.625)
        self.assertAlmostEqual(result['zml']['value'][1], -.5)

    def test_no_support_or_zero_resultant_is_not_a_safe_zero(self):
        result = sample(monitor(), forces=np.zeros((4, 3)))
        for entry in result.values():
            self.assertIsNone(entry['value'])
            self.assertIsNotNone(entry['reason'])
        opposing = np.array([[0., 0., v] for v in (10., -10., 10., -10.)])
        self.assertIsNone(sample(monitor(), forces=opposing)['zml']['value'])

    def test_nonfinite_input_clears_derivative_history_and_serializes_strictly(self):
        diagnostic = monitor()
        sample(diagnostic)
        invalid = sample(diagnostic, time=.1, acceleration=[float('nan'), 0., 0.])
        json.dumps(invalid, allow_nan=False)
        self.assertTrue(all(entry['value'] is None for entry in invalid.values()))
        next_result = sample(diagnostic, time=.2)
        self.assertIsNone(next_result['pltr']['value'])

    def test_singular_pitch_and_invalid_metadata_are_unavailable(self):
        tilted = sample(monitor(), orientation=quaternion(pitch=math.pi/2))
        self.assertIsNone(tilted['si']['value'])
        self.assertIsNone(tilted['pltr']['value'])
        invalid = sample(monitor(mass=float('nan')))
        self.assertIsNone(invalid['si']['value'])
        json.dumps(invalid, allow_nan=False)

    def test_nonincreasing_time_does_not_divide_by_zero(self):
        diagnostic = monitor()
        sample(diagnostic)
        repeated = sample(diagnostic)
        self.assertIsNone(repeated['pltr']['value'])
        self.assertIn('creciente', repeated['pltr']['reason'])


if __name__ == '__main__':
    unittest.main()
