"""Destination runs exercise the real simulation loop with deterministic forces."""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import Simulation, RouteDestination


class Plane:
    metadata = {}

    def upper_xy(self, xy, max_distance):
        return np.zeros(len(xy)), np.column_stack([xy, np.zeros(len(xy))]), np.zeros(len(xy), dtype=int)


class MovingEngine:
    velocity = 1.

    def __init__(self, config, vehicle, lightweight):
        self.dt = config.dt
        self.config = config
        self.robot_model = SimpleNamespace(total_mass=1000.)
        self.inertia = torch.eye(3)*1000
        self.cog = torch.tensor([0., 0., 1.])
        self.centers = torch.tensor([[1., 1., 0.], [1., -1., 0.], [-1., 1., 0.], [-1., -1., 0.]])
        self.radius = .5
        self.ring = torch.zeros(1, 3)
        self.vehicle_id = vehicle
        self.contact_mode = 'full'

    def assemble_and_transform_robot(self, state, controls):
        return state.x[:, None, :], None, state.x, None

    def __call__(self, state, controls, world):
        next_state = SimpleNamespace(x=state.x.clone(), xd=state.xd.clone(), q=state.q.clone(), omega=state.omega.clone())
        next_state.x[0, 0] += self.velocity*self.dt
        next_state.x[0, 2] = max(-.2, float(next_state.x[0, 2]) - self.dt)
        next_state.xd[0, 0] = self.velocity
        derivative = SimpleNamespace(xdd=torch.zeros(1, 3), omega_d=torch.zeros(1, 3))
        aux = SimpleNamespace(F_spring=torch.zeros(1, 1, 3), F_friction=torch.zeros(1, 1, 3),
                              in_contact=torch.ones(1, 1, 1), torque=torch.zeros(1, 3))
        return next_state, derivative, aux


class DestinationTest(unittest.TestCase):
    def run_route(self, distance, velocity=1., via=None):
        simulation = Simulation()
        simulation.store = Plane()
        simulation.center = np.zeros(3)
        route = [dict(x=x, y=0, z=.2, speed=1, acceleration=1) for x in ([0] + (via or []) + [distance])]
        progress = []
        with patch('server.TruckEngine', MovingEngine), patch.object(MovingEngine, 'velocity', velocity):
            result = simulation.evaluate(dict(x=0, y=0, z=.2, duration=2, dt=.01, route=route,
                                              until_destination=True, disable_csv=True), progress.append)
        return result, progress

    def test_stops_on_real_xy_arrival_before_two_seconds_despite_height(self):
        result, progress = self.run_route(1)
        self.assertEqual(result['status'], 'destination_reached')
        self.assertLess(result['frames'][-1]['time'], 2)
        self.assertLessEqual(abs(result['frames'][-1]['position'][0]-1), .5)
        self.assertEqual(result['distance_remaining'], 0)
        self.assertTrue(progress[-1]['until_destination'])
        self.assertIn('time_limit', progress[-1])
        self.assertTrue(all(p['fraction'] < 1 for p in progress[:-1]))

    def test_continues_after_two_seconds_without_teleporting(self):
        result, _ = self.run_route(4, via=[2])
        self.assertEqual(result['status'], 'destination_reached')
        self.assertGreater(result['frames'][-1]['time'], 3)
        self.assertNotIn('Desvío de ruta superior a 2 m', result['warnings'])
        x = [f['position'][0] for f in result['frames']]
        self.assertLess(max(np.diff(x)), .051)

    def test_unmoving_vehicle_reports_destination_not_reached(self):
        result, _ = self.run_route(4, velocity=0)
        self.assertEqual(result['status'], 'destination_not_reached')
        self.assertEqual(result['stopped_reason'], 'stall')
        self.assertFalse(result['destination_reached'])
        self.assertGreaterEqual(result['frames'][-1]['time'], 15)

    def test_long_route_budget_exceeds_sixty_seconds_and_is_bounded(self):
        def route(x):
            return [dict(x=0, y=0, speed=1), dict(x=x, y=0, speed=1)]
        self.assertGreater(RouteDestination(route(100), [0, 0]).time_limit, 60)
        self.assertEqual(RouteDestination(route(1e6), [0, 0]).time_limit, 1800)

    def test_actual_run_can_exceed_sixty_seconds(self):
        result, _ = self.run_route(62)
        self.assertEqual(result['status'], 'destination_reached')
        self.assertGreater(result['frames'][-1]['time'], 60)

    def test_time_limit_reports_failure_instead_of_success(self):
        original = RouteDestination.__init__
        def short_budget(tracker, route, origin):
            original(tracker, route, origin)
            tracker.time_limit = .1
        with patch.object(RouteDestination, '__init__', short_budget):
            result, _ = self.run_route(4)
        self.assertEqual(result['status'], 'destination_not_reached')
        self.assertEqual(result['stopped_reason'], 'timeout')
        self.assertFalse(result['destination_reached'])

    def test_real_full_contact_reaches_short_route_from_above_ground(self):
        from cloud_store import CloudStore
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as temporary:
            root=Path(temporary)
            path=root/'plane.ply'
            axis=np.arange(-12,12.01,.2)
            x,y=np.meshgrid(axis,axis)
            xyz=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]).astype('<f4')
            path.write_bytes(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'.encode()+xyz.tobytes())
            simulation=Simulation()
            simulation.store=CloudStore.open(path,cache_dir=root/'clouds')
            simulation.center=np.zeros(3)
            try:
                pose=simulation.place(dict(x=0,y=0,yaw=0,vehicle='MAN_5t'))
                route=[dict(x=x,y=0,z=0,speed=1,acceleration=1) for x in [0,2]]
                result=simulation.evaluate(dict(x=0,y=0,z=pose['position'][2]+.2,
                    vehicle='MAN_5t',dt=.01,until_destination=True,disable_csv=True,route=route))
                self.assertEqual(result['status'],'destination_reached')
                self.assertLessEqual(np.linalg.norm(np.array(result['frames'][-1]['position'])[:2]-[2,0]),.5)
                self.assertLess(result['frames'][-1]['time'],2)
            finally:
                simulation.store.close()

    def test_progress_uses_route_steps_at_point_zero_one_not_timeout(self):
        result, progress = self.run_route(4)
        preparation = next(p for p in progress if p['stage']=='preparation' and p.get('total'))
        self.assertEqual(preparation['dt'], .01)
        self.assertEqual(preparation['estimated_steps'], 401)
        self.assertEqual(preparation['total'], 401)
        self.assertGreater(preparation['limit_steps'], preparation['total'])
        halfway = next(p for p in progress if p['stage']=='simulation' and p['processed']==200)
        self.assertAlmostEqual(halfway['fraction'], .98*200/401)
        self.assertGreater(halfway['fraction'], .48)
        fractions=[p['fraction'] for p in progress]
        self.assertEqual(fractions,sorted(fractions))
        self.assertEqual(fractions[-1],1.)
        self.assertEqual(result['status'],'destination_reached')

    def test_exceeding_estimated_steps_never_finishes_progress_early(self):
        result, progress = self.run_route(4,velocity=.5)
        running=[p for p in progress if p['stage']=='simulation']
        self.assertGreater(result['samples'],running[0]['estimated_steps'])
        self.assertTrue(all(p['fraction']<=.98 for p in running))
        self.assertEqual([p['fraction'] for p in progress],sorted(p['fraction'] for p in progress))
        self.assertEqual(progress[-1]['fraction'],1.)
        self.assertEqual(progress[-1]['processed'],progress[-1]['total'])

    def test_final_near_start_requires_visiting_intermediate_waypoint(self):
        tracker = RouteDestination([dict(x=x, y=0, speed=1) for x in [0, 4, 0]], [0, 0])
        tracker.observe(np.zeros(2), 0)
        self.assertFalse(tracker.reached)
        self.assertEqual(tracker.index, 1)
        tracker.observe(np.array([4, 0]), 4)
        tracker.observe(np.zeros(2), 8)
        self.assertTrue(tracker.reached)


if __name__ == '__main__':
    unittest.main()
