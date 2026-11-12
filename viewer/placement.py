"""Build a collision-free seed for the hidden physical settling pass."""
import math

import numpy as np
from scipy.spatial.transform import Rotation

SUPPORT_RADIUS = .35
COVERAGE_RADIUS = .8
DROP_CLEARANCE = .2


def _array(value):
    return np.asarray(value.detach().cpu().numpy() if hasattr(value, 'detach') else value,
                      dtype=np.float64)


def place_vehicle(store, engine, xy, yaw):
    """Return a drop seed, never the final placement shown to the user."""
    xy = np.asarray(xy, dtype=np.float64)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise ValueError('Selecciona una posición XY válida sobre el terreno')
    if isinstance(yaw, bool):
        raise ValueError('La orientación debe ser un ángulo válido')
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError('La orientación debe ser un ángulo válido')
    yaw = (yaw + 180.) % 360. - 180.
    centers = _array(engine.centers)
    ring = _array(engine.ring)
    body = _array(getattr(engine, 'body', np.empty((0, 3))))
    radius, width = float(engine.radius), float(engine.width)
    if (centers.ndim != 2 or centers.shape[1] != 3 or len(centers) < 4 or
            ring.ndim != 2 or ring.shape[1] != 3 or not len(ring) or
            not np.isfinite(centers).all() or not np.isfinite(ring).all() or
            not math.isfinite(radius) or not math.isfinite(width) or min(radius, width) <= 0):
        raise ValueError('Geometría de vehículo no compatible')
    wheels = centers[:, None, :] + ring[None, :, :]
    geometry = np.concatenate([wheels.reshape(-1, 3), body])
    # Only the clicked location needs observed data. Uneven terrain, missing
    # support under a wheel and a footprint crossing the map edge are allowed.
    reach = float(np.linalg.norm(geometry[:, :2], axis=1).max()) + COVERAGE_RADIUS
    points, _ = store.region_xy([xy[0]-reach, xy[1]-reach, xy[0]+reach, xy[1]+reach])
    points = np.asarray(points, dtype=np.float64)
    if not len(points) or not np.any(np.linalg.norm(points[:, :2]-xy, axis=1) <= COVERAGE_RADIUS):
        raise ValueError('Posición no válida: no hay puntos en el lugar seleccionado')

    # Spawn level, clear of the highest observation below the rotated footprint.
    # The hidden preparation pass resolves gravity/contact before previewing.
    rotation = Rotation.from_euler('z', yaw, degrees=True)
    local_xy = rotation.inv().apply(points - [xy[0], xy[1], 0])[:, :2]
    low, high = geometry[:, :2].min(0), geometry[:, :2].max(0)
    below = np.all((local_xy >= low-SUPPORT_RADIUS) & (local_xy <= high+SUPPORT_RADIUS), axis=1)
    terrain_top = float(points[below, 2].max()) if below.any() else float(points[:, 2].max())
    z = terrain_top - float(geometry[:, 2].min()) + DROP_CLEARANCE
    q = rotation.as_quat()
    wheel_contacts = (rotation.apply(centers + [0, 0, -radius]) + [xy[0], xy[1], z]).tolist()
    return dict(position=[float(xy[0]), float(xy[1]), z],
                orientation=q[[3, 0, 1, 2]].tolist(),
                roll=0., pitch=0., yaw=yaw,
                wheel_contacts=wheel_contacts, wheel_centers=centers.tolist(),
                tire_radius=radius, tire_width=width,
                support_clearance=DROP_CLEARANCE,
                vehicle=getattr(engine, 'vehicle_id', 'MAN_10t'))
