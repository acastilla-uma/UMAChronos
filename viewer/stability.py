"""Diagnostics for the simulated rigid model; no forces or states are modified.

Angles use radians, forces N and moments N m.  The PLTR is a causal kinematic
adaptation, not Li's single-track tyre predictor.  ZML values are coordinates
of the COP line on a stated reference plane, never a normalized safety index.
"""
import math

import numpy as np


_EPS = 1e-8


def _entry(value=None, params=None, reason=None):
    return {"value": value, "params": params or {}, "reason": reason}


def _finite(value):
    """Keep diagnostics serializable even if an extreme calculation overflows."""
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_finite(item) for item in value]
    if isinstance(value, (float, int, np.number)):
        return float(value) if math.isfinite(float(value)) else None
    return value


class StabilityMonitor:
    """Calculate one sample at a time, before the playback frame stride.

    ``wheel_sample_count`` is the total number of leading wheel points in
    ``points``. Full contact samples are contiguous by wheel in the same order
    as ``wheel_centers``. A count of four denotes the optional fixed corner
    model, in front-left/right, rear-left/right order.
    """

    def __init__(self, mass, inertia_body, cog_body, wheel_centers, tire_radius,
                 wheel_sample_count, horizon=.5, gravity=9.81):
        self.mass = float(mass)
        self.inertia = np.asarray(inertia_body, dtype=float)
        self.cog_body = np.asarray(cog_body, dtype=float)
        self.centers = np.asarray(wheel_centers, dtype=float)
        self.radius = float(tire_radius)
        self.wheel_sample_count = int(wheel_sample_count)
        self.horizon = float(horizon)
        self.gravity = float(gravity)
        self.previous = None

        self.track = self.height = self.ixx = float('nan')
        self.left = self.right = None
        if self.inertia.shape == (3, 3):
            self.ixx = float(self.inertia[0, 0])
        if (self.centers.ndim == 2 and self.centers.shape[1] == 3
                and len(self.centers) and self.cog_body.shape == (3,)
                and np.isfinite(self.centers).all() and np.isfinite(self.cog_body).all()):
            self.track = float(np.ptp(self.centers[:, 1]))
            self.height = float(self.cog_body[2] - np.min(self.centers[:, 2]) + self.radius)
            if self.wheel_sample_count == 4:
                self.left = np.array([True, False, True, False])
            elif (self.wheel_sample_count > 0
                  and self.wheel_sample_count % len(self.centers) == 0):
                per_wheel = self.wheel_sample_count // len(self.centers)
                self.left = np.repeat(self.centers[:, 1] > 0, per_wheel)
            if self.left is not None:
                self.right = ~self.left

    def update(self, time, points, forces, cog, orientation, velocity,
               acceleration, angular_velocity):
        """Return finite JSON diagnostics; unavailable results carry a reason."""
        try:
            time = float(time)
            points = np.asarray(points, dtype=float)
            forces = np.asarray(forces, dtype=float)
            cog = np.asarray(cog, dtype=float)
            quat = np.asarray(orientation, dtype=float)
            velocity = np.asarray(velocity, dtype=float)
            acceleration = np.asarray(acceleration, dtype=float)
            omega = np.asarray(angular_velocity, dtype=float)
            valid = (points.ndim == 2 and points.shape[1] == 3 and forces.shape == points.shape
                     and cog.shape == velocity.shape == acceleration.shape == omega.shape == (3,)
                     and quat.shape == (4,) and math.isfinite(time)
                     and all(np.isfinite(v).all() for v in
                             (points, forces, cog, quat, velocity, acceleration, omega))
                     and np.linalg.norm(quat) > _EPS)
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            self.previous = None
            return {name: _entry(reason="Datos físicos ausentes o no finitos")
                    for name in ('ltr', 'pltr', 'zml', 'si')}

        w, x, y, z = quat / np.linalg.norm(quat)
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])
        phi = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        singular = abs(rotation[2, 0]) >= 1 - _EPS
        ay = float((rotation.T @ acceleration)[1])
        omega_x = float((rotation.T @ omega)[0])
        support = float(np.maximum(forces[:, 2], 0).sum()) > _EPS
        no_support = None if support else "Sin apoyo vertical: índice no disponible"
        ltr = self._ltr(forces, no_support)
        pltr = self._pltr(time, ay, phi, singular, no_support)
        si = self._si(phi, omega_x, singular, no_support)
        zml = self._zml(points, forces, cog, yaw, no_support)
        result = _finite(dict(ltr=ltr, pltr=pltr, zml=zml, si=si))
        for diagnostic in result.values():
            value = diagnostic['value']
            if diagnostic['reason'] is None and (value is None or
                    isinstance(value, list) and any(v is None for v in value)):
                diagnostic.update(value=None, reason="Resultado no finito")
        return result

    def _ltr(self, forces, no_support):
        if (self.left is None or self.wheel_sample_count > len(forces)
                or not self.left.any() or not self.right.any()):
            return _entry(reason="No se puede identificar el lado de las ruedas")
        wheel_loads = forces[:self.wheel_sample_count, 2]
        left = float(wheel_loads[self.left].sum())
        right = float(wheel_loads[self.right].sum())
        total = left + right
        params = dict(fz_left_N=left, fz_right_N=right, fz_total_N=total)
        reason = no_support or ("Ruedas sin carga vertical" if total <= _EPS else None)
        return _entry(None if reason else (right-left)/total, params, reason)

    def _pltr(self, time, ay, phi, singular, no_support):
        params = dict(h_m=self.height, track_m=self.track, gravity_m_s2=self.gravity,
                      ay_body_m_s2=ay, phi_rad=phi, ay_dot_m_s3=None,
                      phi_dot_rad_s=None, horizon_s=self.horizon, derivative_dt_s=None,
                      base_ltr_kinematic=None, prediction_term=None)
        previous = self.previous
        self.previous = None if singular else (time, ay, phi)
        reason = None
        if singular:
            reason = "Balanceo Euler no definido cerca de cabeceo ±90°"
        elif not all(math.isfinite(v) and v > 0 for v in
                     (self.height, self.track, self.gravity, self.horizon)):
            reason = "Geometría, gravedad u horizonte no válidos"
        elif previous is None:
            reason = "Se necesita una muestra anterior para la derivada causal"
        else:
            elapsed = time - previous[0]
            if elapsed <= _EPS:
                reason = "Tiempo no creciente: derivada no disponible"
            else:
                ay_dot = (ay-previous[1])/elapsed
                phi_delta = math.atan2(math.sin(phi-previous[2]), math.cos(phi-previous[2]))
                phi_dot = phi_delta/elapsed
                base = 2*self.height/self.track*(ay/self.gravity+phi)
                prediction = (2*self.height/(self.track*self.gravity)
                              * (ay_dot+self.gravity*phi_dot)*self.horizon)
                params.update(ay_dot_m_s3=ay_dot, phi_dot_rad_s=phi_dot,
                              derivative_dt_s=elapsed, base_ltr_kinematic=base,
                              prediction_term=prediction)
        reason = reason or no_support
        value = None if reason else params['base_ltr_kinematic']+params['prediction_term']
        return _entry(value, params, reason)

    def _si(self, phi, omega_x, singular, no_support):
        params = dict(mass_kg=self.mass, half_track_m=self.track/2, h_m=self.height,
                      distance_m=None, inertia_cg_kg_m2=self.ixx,
                      inertia_pivot_kg_m2=None, gravity_m_s2=self.gravity,
                      phi_rad=phi, omega_body_x_rad_s=omega_x, phi_critical_rad=None,
                      omega_critical_rad_s=None, k1=1.15, k2=2.05,
                      roll_term=None, omega_term=None)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.mass, self.track, self.height, self.ixx, self.gravity)):
            return _entry(params=params, reason="Masa, geometría o inercia no válidas")
        half_track = self.track/2
        distance = math.hypot(half_track, self.height)
        pivot_inertia = self.ixx+self.mass*distance**2
        phi_critical = math.atan2(half_track, self.height)
        omega_critical = math.sqrt(2*self.mass*self.gravity*(distance-self.height)/pivot_inertia)
        if omega_critical <= _EPS:
            return _entry(params=params, reason="Umbral energético degenerado")
        roll_term = 1.15*abs(phi)/phi_critical
        omega_term = 2.05*(omega_x/omega_critical)**2
        params.update(distance_m=distance, inertia_pivot_kg_m2=pivot_inertia,
                      phi_critical_rad=phi_critical, omega_critical_rad_s=omega_critical,
                      roll_term=roll_term, omega_term=omega_term)
        reason = ("Balanceo Euler no definido cerca de cabeceo ±90°" if singular else no_support)
        return _entry(None if reason else 1-roll_term-omega_term, params, reason)

    def _zml(self, points, forces, cog, yaw, no_support):
        c, s = math.cos(yaw), math.sin(yaw)
        heading_to_world = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        relative_points = (points-cog) @ heading_to_world
        heading_forces = forces @ heading_to_world
        force = heading_forces.sum(axis=0)
        moment = np.cross(relative_points, heading_forces).sum(axis=0)
        fx, fy, fz = map(float, force)
        mx, my, mz = map(float, moment)
        params = dict(fx_N=fx, fy_N=fy, fz_N=fz, mx_Nm=mx, my_Nm=my, mz_Nm=mz,
                      z_ref_m=None, x_ref_m=None, y_ref_m=None, dx_dz=None, dy_dz=None,
                      torsion_residual_Nm=None, cog_x_m=cog[0], cog_y_m=cog[1],
                      cog_z_m=cog[2], yaw_rad=yaw)
        reason = no_support or ("Resultante vertical nula: línea no definida" if abs(fz) <= _EPS else None)
        if reason:
            return _entry(params=params, reason=reason)
        weights = np.maximum(heading_forces[:, 2], 0)
        z_ref = float(np.dot(weights, relative_points[:, 2])/weights.sum())
        x_ref = (-my+z_ref*fx)/fz
        y_ref = (mx+z_ref*fy)/fz
        params.update(z_ref_m=z_ref, x_ref_m=x_ref, y_ref_m=y_ref,
                      dx_dz=fx/fz, dy_dz=fy/fz, torsion_residual_Nm=float(moment @ force)/fz)
        return _entry([x_ref, y_ref], params)
