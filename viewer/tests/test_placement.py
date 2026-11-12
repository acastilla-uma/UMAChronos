import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from placement import place_vehicle, DROP_CLEARANCE


class Store:
    def __init__(self, points): self.points = points
    def region_xy(self, bounds):
        lo, hi = np.array(bounds[:2]), np.array(bounds[2:])
        p = self.points[np.all((self.points[:, :2] >= lo) & (self.points[:, :2] <= hi), axis=1)]
        return p, np.full(len(p), -1)


def cloud(height=lambda x, y: np.zeros_like(x), x=(-12, 12), y=(-12, 12), step=.1):
    xx, yy = np.meshgrid(np.arange(x[0], x[1]+step*.1, step), np.arange(y[0], y[1]+step*.1, step))
    return np.column_stack([xx.ravel(), yy.ravel(), height(xx, yy).ravel()])


def engine(axles=(2., -2.)):
    radius, width = .6, .4
    angle = np.linspace(0, 2*np.pi, 24, endpoint=False)
    ring = np.array([[radius*np.sin(a), y, radius*np.cos(a)] for y in (-width/2, width/2) for a in angle])
    return SimpleNamespace(centers=np.array([[x, y, 0.] for x in axles for y in (-1., 1.)]),
                           ring=ring, radius=radius, width=width, body=np.array([[x, y, .5] for x in (-3., 3.) for y in (-1.2, 1.2)]), vehicle_id='MAN_5t')


class PlacementTest(unittest.TestCase):
    def test_flat_top_layer_leaves_clearance_for_gravity(self):
        floor = cloud()
        model = engine()
        result = place_vehicle(Store(np.concatenate([floor, floor+[0,0,2]])), model, [0,0], 0)
        self.assertAlmostEqual(result['position'][2], 2.6+DROP_CLEARANCE)
        self.assertEqual(result['roll'], 0)
        self.assertEqual(result['pitch'], 0)
        self.assertEqual(result['wheel_centers'], model.centers.tolist())
        for point in result['wheel_contacts']: self.assertAlmostEqual(point[2], 2+DROP_CLEARANCE)

    def test_slopes_and_abrupt_steps_are_allowed_with_level_heading(self):
        for height in (lambda x,y: .8*x, lambda x,y: -.7*y, lambda x,y: np.where(x>1,1.,0.)):
            with self.subTest(height=height):
                result = place_vehicle(Store(cloud(height)), engine((2.,0.,-2.)), [0,0], 53)
                self.assertEqual(result['roll'], 0)
                self.assertEqual(result['pitch'], 0)
                self.assertEqual(result['yaw'], 53)
                self.assertGreater(result['position'][2], .6)

    def test_missing_wheel_support_does_not_reject_selected_point(self):
        points=cloud()
        points=points[np.linalg.norm(points[:,:2]-[2.,1.],axis=1)>.9]
        self.assertTrue(place_vehicle(Store(points),engine(),[0,0],0))

    def test_empty_selected_location_is_rejected_even_with_nearby_terrain(self):
        points=cloud()
        points=points[np.linalg.norm(points[:,:2],axis=1)>1.3]
        with self.assertRaisesRegex(ValueError,'no hay puntos'):
            place_vehicle(Store(points),engine(),[0,0],0)
        with self.assertRaisesRegex(ValueError,'no hay puntos'):
            place_vehicle(Store(cloud()),engine(),[100,0],0)

    def test_edge_and_rotation_are_allowed(self):
        store=Store(cloud(x=(-5,5),y=(-2,2)))
        for xy,yaw in (([0,0],90),([5,2],25)):
            self.assertTrue(place_vehicle(store,engine(),xy,yaw))

    def test_single_observed_point_is_sufficient(self):
        result=place_vehicle(Store(np.array([[0.,0.,3.]])),engine(),[0,0],0)
        self.assertAlmostEqual(result['position'][2],3.6+DROP_CLEARANCE)

    def test_all_actual_man_axle_geometries_fit_flat_ground(self):
        base = ROOT / 'assets/data/vehicle/MAN'
        filenames = {'MAN_5t':'MAN_5t_Vehicle_4WD.json', 'MAN_7t':'MAN_7t_Vehicle_6WD.json', 'MAN_10t':'MAN_10t_Vehicle_8WD.json'}
        store = Store(cloud())
        for name, filename in filenames.items():
            with self.subTest(vehicle=name):
                vehicle = json.loads((base/'vehicle'/filename).read_text())
                model = engine()
                centers = []
                for axle in vehicle['Axles']:
                    susp = json.loads((base/axle['Suspension Input File'].replace('MAN/', '')).read_text())
                    loc = np.array(axle['Suspension Location']) + np.array(susp['Spindle']['COM'])
                    centers.extend([loc, loc*[1, -1, 1]])
                model.centers = np.array(centers)
                model.vehicle_id = name
                result = place_vehicle(store, model, [0, 0], 25)
                self.assertEqual(len(result['wheel_contacts']), len(centers))
                self.assertEqual(result['support_clearance'], DROP_CLEARANCE)

    def test_survey_coordinates_do_not_change_support(self):
        offset = np.array([500000., 4100000., 300.])
        result = place_vehicle(Store(cloud() + offset), engine(), offset[:2], 30)
        np.testing.assert_allclose(result['position'], offset+[0, 0, .6+DROP_CLEARANCE], atol=1e-6)

    def test_rejects_invalid_inputs(self):
        for xy, yaw in (([float('nan'), 0],0), ([0],0), ([0,0],float('inf')), ([0,0],True)):
            with self.assertRaises(ValueError): place_vehicle(Store(cloud()), engine(), xy, yaw)


if __name__ == '__main__': unittest.main()
