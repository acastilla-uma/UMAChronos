"""Local MAN contact inspector using Project Chrono (pychrono.core + pychrono.vehicle)."""
import csv
from copy import deepcopy
from contextlib import nullcontext
import os
import tempfile
import argparse
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from plyfile import PlyData
from stability import StabilityMonitor

ROOT = Path(__file__).resolve().parent


def _configure_chrono_runtime():
    candidates = [
        os.environ.get('CHRONO_RUNTIME_DIR'),
        ROOT / '.cache' / 'chrono_runtime',
        ROOT.parent.parent / 'Chrono-docker' / '.runtime',
    ]
    for candidate in candidates:
        if not candidate:
            continue
        rt = Path(candidate)
        bin_dir = rt / 'Library' / 'bin'
        site_dir = rt / 'Lib' / 'site-packages'
        data_dir = rt / 'Library' / 'data'
        if bin_dir.is_dir() and site_dir.is_dir():
            os.environ['PATH'] = str(bin_dir) + os.pathsep + os.environ.get('PATH', '')
            if hasattr(os, 'add_dll_directory'):
                os.add_dll_directory(str(bin_dir))
            if str(site_dir) not in sys.path:
                sys.path.insert(0, str(site_dir))
            import pychrono.core as _chrono
            import pychrono.vehicle as _veh
            if data_dir.is_dir():
                _chrono.SetChronoDataPath(str(data_dir).replace('\\', '/') + '/')
                veh_data = data_dir / 'vehicle'
                if veh_data.is_dir():
                    _veh.SetVehicleDataPath(str(veh_data).replace('\\', '/') + '/')
            return _chrono, _veh
    import pychrono.core as _chrono
    import pychrono.vehicle as _veh
    return _chrono, _veh


chrono, veh = _configure_chrono_runtime()


def q_to_R(q):
    q = torch.as_tensor(q, dtype=torch.float32)
    q = q / torch.linalg.norm(q, dim=-1, keepdim=True).clamp(min=1e-12)
    w, x, y, z = q.unbind(-1)
    return torch.stack([
        torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], dim=-1),
        torch.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], dim=-1),
        torch.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], dim=-1),
    ], dim=-2)


def _normalized(v, eps=1e-9):
    return v / torch.linalg.norm(v, dim=-1, keepdim=True).clamp(min=eps)


class PhysicsEngineConfig:
    def __init__(self, num_robots=1, dt=0.01, gravity=9.81, torque_limit=1e7,
                 damping_alpha=1.0, soft_contact_sigma=0.01):
        self.num_robots = int(num_robots)
        self.dt = float(dt)
        self.gravity = float(gravity)
        self.torque_limit = float(torque_limit)
        self.damping_alpha = float(damping_alpha)
        self.soft_contact_sigma = float(soft_contact_sigma)


class WorldConfig:
    def __init__(self, x_grid, y_grid, z_grid, grid_res=0.2, max_coord=16.0,
                 k_stiffness=40_000.0, k_friction_lon=0.5, k_friction_lat=0.2):
        self.x_grid = x_grid
        self.y_grid = y_grid
        self.z_grid = z_grid
        self.grid_res = float(grid_res)
        self.max_coord = float(max_coord)
        self.k_stiffness = k_stiffness
        self.k_friction_lon = k_friction_lon
        self.k_friction_lat = k_friction_lat
        self.z_grid_grad = torch.stack(
            torch.gradient(self.z_grid, spacing=self.grid_res, dim=(2, 1), edge_order=2), dim=1
        )


class PhysicsState:
    def __init__(self, x, xd, q, omega, thetas=None, batch_size=None):
        self.x = x
        self.xd = xd
        self.q = q
        self.omega = omega
        self.thetas = torch.empty(1, 0) if thetas is None else thetas
        self.batch_size = [1] if batch_size is None else batch_size


def read_ply(data):
    if len(data) > 100_000_000:
        raise ValueError('Archivo mayor de 100 MB')
    v = PlyData.read(io.BytesIO(data))['vertex']
    p = np.column_stack([v[n] for n in ('x', 'y', 'z')]).astype(np.float32)
    if not 3 <= len(p) <= 2_000_000 or not np.isfinite(p).all():
        raise ValueError('Se requieren entre 3 y 2 millones de puntos XYZ finitos')
    return p


class RouteDestination:
    """Track observed XY waypoint visits; the time budget never moves the body."""
    tolerance = .5
    stall_seconds = 15.

    def __init__(self, route, origin):
        self.route = route
        self.points = np.array([[w['x'], w['y']] for w in route])
        self.index = 0
        self.last_improvement = 0.
        self.best_distance = float('inf')
        previous = np.vstack([np.asarray(origin)[:2], self.points[:-1]])
        lengths = np.linalg.norm(self.points - previous, axis=1)
        speeds = np.array([max(.25, min(20., w['speed'])) for w in route])
        self.estimated_duration = float(np.sum(lengths / speeds))
        self.time_limit = min(1800., max(30., self.estimated_duration*3 + 20.))
        self.remaining = float(lengths.sum())
        self.reached = False

    def observe(self, xy, time):
        while self.index < len(self.points) and np.linalg.norm(self.points[self.index]-xy) <= self.tolerance:
            self.index += 1
            self.best_distance = float('inf')
            self.last_improvement = time
        self.reached = self.index == len(self.points)
        if self.reached:
            self.remaining = 0.
            return None
        distance = float(np.linalg.norm(self.points[self.index]-xy))
        self.remaining = distance + float(np.linalg.norm(np.diff(self.points[self.index:], axis=0), axis=1).sum())
        if distance < self.best_distance - .05:
            self.best_distance = distance
            self.last_improvement = time
        return dict(self.route[self.index])


class TruckEngine:
    """Project Chrono (pychrono.core + pychrono.vehicle) MAN vehicle contact engine."""
    def __init__(self, cfg, vehicle_id="MAN_10t", lightweight_physics=False):
        if not isinstance(lightweight_physics, bool):
            raise ValueError('La opción de física ligera debe ser booleana')
        self.lightweight_physics = lightweight_physics
        self.contact_mode = 'four_corners' if lightweight_physics else 'full'
        self.contact_ids = ('front_left','front_right','rear_left','rear_right') if lightweight_physics else None
        base = ROOT / 'assets/data/vehicle/MAN'
        load = lambda p: json.loads((base / p).read_text())
        if vehicle_id not in ('MAN_5t', 'MAN_7t', 'MAN_10t'): raise ValueError('Vehículo no compatible')
        body = load(f'chassis/{vehicle_id}_Chassis.json')['Components'][0]
        vehicle = load('vehicle/'+{'MAN_5t':'MAN_5t_Vehicle_4WD.json','MAN_7t':'MAN_7t_Vehicle_6WD.json','MAN_10t':'MAN_10t_Vehicle_8WD.json'}[vehicle_id])
        self.vehicle_id = vehicle_id
        self.steering_factors = torch.tensor([factor for axle in vehicle['Axles'] for factor in [1. if axle.get('Steering Index') == 0 else .7 if 'Steering Index' in axle else 0.]*2])
        tire = load('tire/MAN_RigidMeshTire.json')

        self.cog = torch.tensor(body['Centroidal Frame']['Location'], dtype=torch.float32)
        self.inertia = torch.diag(torch.tensor(body['Moments of Inertia'], dtype=torch.float32))
        self.centers = []
        mass = body['Mass']
        radius = tire.get('Radius', 0.645)
        width = tire.get('Width', 0.385)
        for axle in vehicle['Axles']:
            susp = load(axle['Suspension Input File'].replace('MAN/', ''))
            loc = np.array(axle['Suspension Location']) + np.array(susp['Spindle']['COM'])
            self.centers.extend([loc, loc * [1,-1,1]])
        self.centers = torch.tensor(np.array(self.centers), dtype=torch.float32)
        self.radius, self.width = radius, width
        angle = np.linspace(0, 2*np.pi, 24, endpoint=False)
        self.ring = torch.tensor([[radius*np.sin(a), y, radius*np.cos(a)] for y in (-width/2, width/2) for a in angle], dtype=torch.float32)
        verts = []
        for line in (base/f'meshes/{vehicle_id}_chassis_col.obj').read_text().splitlines():
            if line.startswith('v '): verts.append([float(x) for x in line.split()[1:4]])
        self.body = torch.tensor(np.unique(np.array(verts), axis=0)[::max(1,len(verts)//250)], dtype=torch.float32)
        if lightweight_physics:
            wheel_points=(self.centers[:,None,:]+self.ring[None,:,:]).reshape(-1,3)
            geometry=torch.cat([wheel_points,self.body])
            low,high=geometry.amin(0),geometry.amax(0)
            self.corner_points=torch.stack([
                torch.stack([high[0],high[1],low[2]]),
                torch.stack([high[0],low[1],low[2]]),
                torch.stack([low[0],high[1],low[2]]),
                torch.stack([low[0],low[1],low[2]]),
            ])
            front=self.centers[:,0].argmax();rear=self.centers[:,0].argmin()
            self.corner_steering_factors=self.steering_factors[torch.stack([front,front,rear,rear])]
        self.config = cfg
        self.robot_model = SimpleNamespace(total_mass=float(mass), driving_direction=torch.tensor([1.,0.,0.]))
        self._F_g = torch.tensor([0.0, 0.0, -self.robot_model.total_mass * self.config.gravity], dtype=torch.float32)
        self._truck = None
        self._system = None
        self._terrain = None
        self._terrain_bodies = []
        self._chassis_acc = None
        self._last_z_grid = None
        self._last_mu_k = None
        self._last_x = None
        self._sys_mass = float(mass)

    def assemble_and_transform_robot(self, state, controls):
        steer = controls[0,1]
        if self.lightweight_physics:
            angles=self.corner_steering_factors*steer
            pts=self.corner_points
            thrust=torch.stack([angles.cos(),angles.sin(),torch.zeros(4)],-1)*controls[0,0]
        else:
            angles = self.steering_factors*steer
            n_wheels = len(self.centers)
            c,s = angles.cos(), angles.sin()
            rot = torch.zeros(n_wheels,3,3); rot[:,0,0]=c; rot[:,0,1]=-s; rot[:,1,0]=s; rot[:,1,1]=c; rot[:,2,2]=1
            wheels = torch.einsum('nij,pj->npi',rot,self.ring) + self.centers[:,None,:]
            pts = torch.cat([wheels.reshape(-1,3),self.body])
            drive = torch.stack([c,s,torch.zeros(n_wheels)],-1)[:,None,:].expand(-1,len(self.ring),-1).reshape(-1,3)*controls[0,0]
            thrust = torch.cat([drive,torch.zeros_like(self.body)])
        R = q_to_R(state.q)[0]
        return ((pts@R.T+state.x[0])[None],(thrust@R.T)[None],(self.cog@R.T+state.x[0])[None],(R@self.inertia@R.T)[None])

    def _init_chrono(self, state, world):
        veh_cls = {'MAN_5t': veh.MAN_5t, 'MAN_7t': veh.MAN_7t, 'MAN_10t': veh.MAN_10t}[self.vehicle_id]
        truck = veh_cls()
        truck.SetContactMethod(chrono.ChContactMethod_SMC)
        truck.SetChassisCollisionType(veh.CollisionType_NONE)
        x0 = state.x[0].detach().cpu().tolist()
        q0 = (state.q[0] / torch.linalg.norm(state.q[0]).clamp(min=1e-12)).detach().cpu().tolist()
        sub_dt = min(0.005, float(self.config.dt))
        truck.SetInitPosition(chrono.ChCoordsysd(
            chrono.ChVector3d(float(x0[0]), float(x0[1]), float(x0[2])),
            chrono.ChQuaterniond(float(q0[0]), float(q0[1]), float(q0[2]), float(q0[3]))
        ))
        truck.SetTireType(veh.TireModelType_TMEASY)
        truck.SetTireStepSize(sub_dt)
        truck.Initialize()
        sys_smc = truck.GetSystem()
        sys_smc.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -float(self.config.gravity)))
        chassis_body = truck.GetChassisBody()
        self._chassis_acc = chassis_body.AddAccumulator()
        for link in sys_smc.GetLinks():
            name = link.GetName()
            if 'spring' in name or 'shock' in name:
                tsda = chrono.CastToChLinkTSDA(link)
                tsda.RegisterForceFunctor(None)
                tsda.SetRestLength(tsda.GetLength())
                if 'spring' in name:
                    tsda.SetSpringCoefficient(2.5e6)
                    tsda.SetDampingCoefficient(1.8e5)
                else:
                    tsda.SetSpringCoefficient(0.0)
                    tsda.SetDampingCoefficient(6.0e4)
        xd0 = state.xd[0].detach().cpu().tolist()
        w0 = state.omega[0].detach().cpu().tolist()
        v_vec = chrono.ChVector3d(float(xd0[0]), float(xd0[1]), float(xd0[2]))
        w_vec = chrono.ChVector3d(float(w0[0]), float(w0[1]), float(w0[2]))
        ref_pos = chassis_body.GetFrameRefToAbs().GetPos()
        sys_mass = 0.0
        for body in sys_smc.GetBodies():
            if not body.IsFixed():
                sys_mass += float(body.GetMass())
                r_rel = body.GetPos() - ref_pos
                body.SetPosDt(v_vec + w_vec.Cross(r_rel))
                body.SetAngVelParent(w_vec)
        self._sys_mass = max(sys_mass, self.robot_model.total_mass)
        self._truck = truck
        self._system = sys_smc
        self._update_terrain(world)
        self._last_x = np.array(x0, dtype=float)

    def _update_terrain(self, world):
        z_tensor = world.z_grid
        mu_mean = float(torch.as_tensor(world.k_friction_lon).mean())
        k_mean = float(torch.as_tensor(world.k_stiffness).mean())
        if (self._terrain is not None and self._last_z_grid is not None
                and torch.equal(z_tensor, self._last_z_grid)
                and self._last_mu_k == (round(mu_mean, 4), round(k_mean, 1))):
            return
        before_bodies = {b.GetIdentifier() for b in self._system.GetBodies()}
        for old_body in self._terrain_bodies:
            try:
                self._system.Remove(old_body)
            except Exception:
                pass
        self._terrain_bodies = []
        terrain = veh.RigidTerrain(self._system)
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(max(0.01, mu_mean))
        mat.SetKn(float(np.clip(k_mean, 1e4, 1e7)))
        z_np = z_tensor[0].detach().cpu().numpy().astype(float)
        if float(z_np.max() - z_np.min()) < 1e-5:
            z0 = float(z_np[0, 0])
            terrain.AddPatch(mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, z0), chrono.QUNIT), 32.0, 32.0)
        else:
            x_np = world.x_grid[0].detach().cpu().numpy().astype(float)[::4, ::4]
            y_np = world.y_grid[0].detach().cpu().numpy().astype(float)[::4, ::4]
            z_sub = z_np[::4, ::4]
            ny, nx = z_sub.shape
            verts = np.column_stack([x_np.ravel(), y_np.ravel(), z_sub.ravel()])
            idx = np.arange(1, nx * ny + 1).reshape(ny, nx)
            f1 = np.column_stack([idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel()])
            f2 = np.column_stack([idx[:-1, :-1].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()])
            faces = np.vstack([f1, f2])
            cache_dir = ROOT / '.cache' / 'chrono_meshes'
            cache_dir.mkdir(parents=True, exist_ok=True)
            obj_path = cache_dir / f'patch_{os.getpid()}.obj'
            v_lines = '\n'.join(f'v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}' for v in verts)
            f_lines = '\n'.join(f'f {f[0]} {f[1]} {f[2]}' for f in faces)
            obj_path.write_text(v_lines + '\n' + f_lines + '\n', encoding='ascii')
            terrain.AddPatch(mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1e-4), chrono.QUNIT), str(obj_path), True, 1e-4, False)
        terrain.Initialize()
        self._system.Setup()
        self._system.Update()
        self._system.ComputeCollisions()
        self._terrain = terrain
        self._terrain_bodies = [b for b in self._system.GetBodies() if b.GetIdentifier() not in before_bodies]
        self._last_z_grid = z_tensor.clone()
        self._last_mu_k = (round(mu_mean, 4), round(k_mean, 1))

    def _evaluate_forces(self, state, controls, world):
        robot_points, global_thrust_vectors, global_cogs, inertia = self.assemble_and_transform_robot(state, controls)
        query = robot_points[..., :2]
        norm_query = torch.clamp(query / world.max_coord, -1.0, 1.0).unsqueeze(2)
        z_points = torch.nn.functional.grid_sample(
            world.z_grid.unsqueeze(1), norm_query, align_corners=True, mode='bilinear'
        ).squeeze(1)
        grad_query = torch.nn.functional.grid_sample(
            world.z_grid_grad, norm_query, align_corners=True, mode='bilinear'
        ).squeeze(-1).transpose(1, 2)
        n = _normalized(torch.dstack([-grad_query, torch.ones((1, query.shape[1], 1), dtype=torch.float32)]))
        dh_points = (robot_points[..., 2:3] - z_points) * n[..., 2:3]
        in_contact = 0.5 * (1.0 + torch.tanh((-dh_points / self.config.soft_contact_sigma) * (3.0 ** 0.5)))
        dh_points = dh_points * in_contact

        cog_corrected_points = robot_points - global_cogs
        xd_points = state.xd.unsqueeze(1) + torch.cross(state.omega.unsqueeze(1), cog_corrected_points, dim=-1)
        num_contacts = in_contact.sum(dim=1, keepdim=True).clamp_min(1.0)
        k_damping = self.config.damping_alpha * 2.0 * (self.robot_model.total_mass * world.k_stiffness / num_contacts) ** 0.5
        xd_points_n = (xd_points * n).sum(dim=-1, keepdim=True)
        F_spring = -torch.mul((world.k_stiffness * dh_points + k_damping * xd_points_n), n) * in_contact / num_contacts

        N = torch.norm(F_spring, dim=2, keepdim=True)
        R = q_to_R(state.q)
        global_driving_dir = torch.matmul(R, self.robot_model.driving_direction.view(1, 3, 1)).transpose(1, 2)
        forward_dir = _normalized(global_driving_dir - (global_driving_dir * n).sum(dim=-1, keepdim=True) * n)
        lateral_dir = _normalized(torch.cross(forward_dir, n, dim=-1))
        dv = global_thrust_vectors - xd_points
        dv_n = (dv * n).sum(dim=-1, keepdim=True)
        dv_tau = torch.tanh(dv - dv_n * n)
        dv_lon = (dv_tau * forward_dir).sum(dim=-1, keepdim=True) * forward_dir
        dv_lat = (dv_tau * lateral_dir).sum(dim=-1, keepdim=True) * lateral_dir
        F_friction = world.k_friction_lon * N * dv_lon + world.k_friction_lat * N * dv_lat

        act_force = F_spring + F_friction
        torque = torch.sum(torch.cross(cog_corrected_points, act_force, dim=-1), dim=1)
        torque = torch.clamp(torque, -self.config.torque_limit, self.config.torque_limit)
        omega_d = torch.linalg.solve_ex(inertia, torque)[0]
        F_cog = self._F_g + act_force.sum(dim=1)
        xdd = F_cog / self.robot_model.total_mass
        return F_spring, F_friction, in_contact, torque, xdd, omega_d, robot_points, global_thrust_vectors, inertia

    def __call__(self, state, controls, world):
        if self._truck is None:
            self._init_chrono(state, world)
        else:
            x_in = state.x[0].detach().cpu().numpy().astype(float)
            delta = x_in - self._last_x
            if float(np.linalg.norm(delta)) > 1e-5:
                d_vec = chrono.ChVector3d(float(delta[0]), float(delta[1]), float(delta[2]))
                for body in self._system.GetBodies():
                    if not body.IsFixed():
                        body.SetPos(body.GetPos() + d_vec)
                self._last_x = x_in.copy()
            self._update_terrain(world)

        F_spring, F_friction, in_contact, torque, xdd, omega_d, robot_points, global_thrust_vectors, _ = (
            self._evaluate_forces(state, controls, world)
        )

        # Advance Project Chrono multibody system (MAN vehicle + TMeasy tires + RigidTerrain)
        dt = float(self.config.dt)
        sub_dt = min(0.005, dt)
        n_sub = max(1, int(round(dt / sub_dt)))
        step_dt = dt / n_sub
        drv = veh.DriverInputs()
        cmd_speed = float(controls[0, 0])
        steer_rad = float(controls[0, 1])
        drv.m_steering = float(np.clip(steer_rad / np.deg2rad(39.0), -1.0, 1.0))
        chassis_body = self._truck.GetChassisBody()

        cur_state = PhysicsState(
            x=state.x.clone(), xd=state.xd.clone(), q=state.q.clone(), omega=state.omega.clone()
        )
        for s in range(n_sub):
            if s > 0:
                fs_s, ff_s, _, tq_s, xdd_s, omegad_s, _, _, _ = self._evaluate_forces(cur_state, controls, world)
            else:
                fs_s, ff_s, tq_s, xdd_s, omegad_s = F_spring, F_friction, torque, xdd, omega_d

            v_lon = float(cur_state.xd[0, 0])
            if abs(cmd_speed) > 0.05:
                drv.m_throttle = float(np.clip((cmd_speed - v_lon) * 0.5, 0.0, 1.0))
                drv.m_braking = float(np.clip((v_lon - cmd_speed) * 0.5, 0.0, 1.0))
            else:
                drv.m_throttle = 0.0
                drv.m_braking = 1.0

            t_ch = self._system.GetChTime()
            self._truck.Synchronize(t_ch, drv, self._terrain)
            self._terrain.Synchronize(t_ch)

            # Query Chrono TMeasy tire forces and apply residual contact wrench on chassis
            tmeasy_f = chrono.ChVector3d(0, 0, 0)
            for axle in self._truck.GetVehicle().GetAxles():
                for wheel in axle.m_wheels:
                    tf = wheel.GetTire().ReportTireForce(self._terrain)
                    tmeasy_f += tf.force
                    sp = wheel.GetSpindle()
                    sp.EmptyAccumulator(1)

            f_net = (fs_s + ff_s).sum(dim=1)[0].detach().cpu().numpy().astype(float)
            mass_scale = self._sys_mass / self.robot_model.total_mass
            f_sys = chrono.ChVector3d(
                float(f_net[0] * mass_scale),
                float(f_net[1] * mass_scale),
                float(f_net[2] * mass_scale),
            )
            chassis_body.EmptyAccumulator(self._chassis_acc)
            chassis_body.AccumulateForce(self._chassis_acc, f_sys, chassis_body.GetPos(), False)

            self._truck.Advance(step_dt)
            self._terrain.Advance(step_dt)

            # Synchronize rigid-body rotational & translational kinematics with Chrono step
            next_xd_s = cur_state.xd + xdd_s * step_dt
            next_x_s = cur_state.x + next_xd_s * step_dt
            next_omega_s = cur_state.omega + omegad_s * step_dt
            omega_norm = torch.linalg.norm(next_omega_s, dim=-1, keepdim=True)
            half_angle = 0.5 * omega_norm * step_dt
            dq_xyz = torch.where(
                omega_norm > 1e-8,
                next_omega_s / omega_norm.clamp(min=1e-8) * torch.sin(half_angle),
                0.5 * next_omega_s * step_dt,
            )
            dq_w = torch.cos(half_angle)
            w1, x1, y1, z1 = dq_w[0, 0], dq_xyz[0, 0], dq_xyz[0, 1], dq_xyz[0, 2]
            w2, x2, y2, z2 = cur_state.q[0].unbind(-1)
            next_q_s = torch.stack([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
            ], dim=-1).view(1, 4)
            next_q_s = next_q_s / torch.linalg.norm(next_q_s, dim=-1, keepdim=True).clamp(min=1e-12)

            x_np = next_x_s[0].detach().cpu().tolist()
            xd_np = next_xd_s[0].detach().cpu().tolist()
            q_np = next_q_s[0].detach().cpu().tolist()
            w_np = next_omega_s[0].detach().cpu().tolist()
            old_ref = chassis_body.GetFrameRefToAbs().GetPos()
            new_ref = chrono.ChVector3d(float(x_np[0]), float(x_np[1]), float(x_np[2]))
            new_rot = chrono.ChQuaterniond(float(q_np[0]), float(q_np[1]), float(q_np[2]), float(q_np[3]))
            d_ref = new_ref - old_ref
            chassis_body.SetFrameRefToAbs(chrono.ChFramed(new_ref, new_rot))
            v_ref = chrono.ChVector3d(float(xd_np[0]), float(xd_np[1]), float(xd_np[2]))
            w_ref = chrono.ChVector3d(float(w_np[0]), float(w_np[1]), float(w_np[2]))
            for body in self._system.GetBodies():
                if not body.IsFixed() and body.GetIdentifier() != chassis_body.GetIdentifier():
                    body.SetPos(body.GetPos() + d_ref)
            r_com = chassis_body.GetPos() - new_ref
            chassis_body.SetPosDt(v_ref + w_ref.Cross(r_com))
            chassis_body.SetAngVelParent(w_ref)
            cur_state = PhysicsState(x=next_x_s, xd=next_xd_s, q=next_q_s, omega=next_omega_s)

        ref_frame = chassis_body.GetFrameRefToAbs()
        pos = ref_frame.GetPos()
        rot = ref_frame.GetRot()
        next_state = PhysicsState(
            x=torch.tensor([[pos.x, pos.y, pos.z]], dtype=torch.float32),
            xd=cur_state.xd.clone(),
            q=torch.tensor([[rot.e0, rot.e1, rot.e2, rot.e3]], dtype=torch.float32),
            omega=cur_state.omega.clone(),
        )
        self._last_x = next_state.x[0].detach().cpu().numpy().astype(float).copy()
        der = SimpleNamespace(xd=state.xd, xdd=xdd, omega_d=omega_d)
        aux = SimpleNamespace(
            F_spring=F_spring,
            F_friction=F_friction,
            in_contact=in_contact,
            torque=torque,
            global_robot_points=robot_points,
            global_thrust_vectors=global_thrust_vectors,
        )
        return next_state, der, aux





TERRAIN_DEFAULTS = {
    'desconocido': {'friction': .7, 'stiffness': 1e6},
    'grava': {'friction': .6, 'stiffness': 7e5},
    'roca': {'friction': .85, 'stiffness': 3e6},
    'plantas': {'friction': .45, 'stiffness': 2e5},
    'barro': {'friction': .25, 'stiffness': 1e5},
    'tierra': {'friction': .65, 'stiffness': 5e5},
}


def catalog():
    result = []
    for model in ('MAN_5t', 'MAN_7t', 'MAN_10t'):
        body = json.loads((ROOT / f'assets/data/vehicle/MAN/chassis/{model}_Chassis.json').read_text())['Components'][0]
        result.append(dict(id=model, name=model.replace('_', ' ')+' · rígido experimental',
                           mass=body['Mass'], family='ruedas', status='experimental',
                           limitations='Masa e inercia del chasis; suspensión fija, sin motor, transmisión ni neumáticos deformables. No validado como vehículo completo.',
                           mesh=f'/assets/data/vehicle/MAN/meshes/{model}_chassis.obj'))
    return result


class Simulation:
    def __init__(self):
        self.store = None
        self.points = None
        self.csv_path = None
        self.center = np.zeros(3)

    def load_path(self, path, progress=None, cancel=None, expected_sha256=None, expected_mapping=None):
        from cloud_store import CloudStore
        candidate = CloudStore.open(path, cache_dir=ROOT / '.cache' / 'clouds', progress=progress, cancel=cancel)
        if expected_sha256 is not None and candidate.metadata['sha256'] != expected_sha256:
            candidate.close()
            raise ValueError('La huella de la nube no coincide con el proyecto')
        if expected_mapping is not None and candidate.metadata.get('label_mapping') != expected_mapping:
            candidate.close()
            raise ValueError('La tabla de clases de la nube cambió desde el guardado')
        # Complete and validate before replacing a working cloud.
        points, labels = candidate.all_points()
        previous = self.store
        self.store = candidate
        self.points = points
        self.labels = labels
        self.center = np.array(candidate.metadata['center'], dtype=np.float64)
        self.csv_path = None
        if previous: previous.close()
        return self.info()

    def load(self, data):
        # Compatibility upload path. HTTP uploads are streamed by webapp instead.
        directory = ROOT / '.cache' / 'uploads'
        directory.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=directory, suffix='.ply', delete=False) as f:
            f.write(data)
            path = f.name
        try:
            return self.load_path(path)
        except Exception:
            Path(path).unlink(missing_ok=True)
            raise

    def info(self):
        return dict(points=[] if self.points is None else self.points.tolist(),
                    labels=[] if self.points is None else self.labels.tolist(),
                    center=self.center.tolist(), cloud=None if self.store is None else deepcopy(self.store.metadata),
                    catalog=catalog(), catalog_audit=json.loads((ROOT/'catalog.json').read_text(encoding='utf-8')) if (ROOT/'catalog.json').exists() else {}, model=catalog()[-1], terrain_defaults=deepcopy(TERRAIN_DEFAULTS))

    def local(self, center, radius=5):
        if self.store is None: raise ValueError('Carga primero una nube PLY')
        center = np.asarray(center, dtype=float)
        if center.shape != (3,) or not np.isfinite(center).all() or radius != 5:
            raise ValueError('La vista local requiere un centro XYZ y radio de 5 m')
        points, labels, total = self.store.sphere_overview(center, radius=5, limit=100000)
        return dict(points=points.tolist(), labels=labels.tolist(), radius=5, visual_only=True, total=total, sampled=total>len(points))

    def place(self, p):
        if self.store is None:
            raise ValueError('Carga primero una nube PLY')
        if not isinstance(p, dict):
            raise ValueError('Pose inválida')
        from placement import place_vehicle
        purpose = p.get('purpose', 'initial')
        if purpose not in ('initial', 'waypoint'):
            raise ValueError('Tipo de colocación inválido')
        engine = TruckEngine(PhysicsEngineConfig(num_robots=1, dt=.01, torque_limit=1e7),
                             p.get('vehicle', 'MAN_10t'), p.get('lightweight_physics', False))
        result = place_vehicle(self.store, engine, [p.get('x'), p.get('y')], p.get('yaw', 0))
        if purpose == 'waypoint':
            # A route may cross a slope where the truck cannot remain parked.
            self.csv_path = None
            return result
        # The clearance pose is only a safe seed. Resolve gravity/contact before
        # showing the vehicle, using exactly the simulation's terrain pipeline.
        seed = dict(p, **dict(zip(('x', 'y', 'z'), result['position'])))
        seed.update(roll=result['roll'], pitch=result['pitch'], yaw=result['yaw'])
        rested = self.evaluate(seed, _settle=True)
        rotation = Rotation.from_quat(np.asarray(rested['orientation'])[[1, 2, 3, 0]])
        angles = rotation.as_euler('xyz', degrees=True)
        result.update(position=rested['position'], orientation=rested['orientation'],
                      roll=float(angles[0]), pitch=float(angles[1]), yaw=float(angles[2]),
                      support_clearance=0., grounded=True, settled=False, settling_time=rested['settling_time'])
        result['wheel_contacts'] = (rotation.apply(engine.centers.numpy()+[0, 0, -engine.radius])
                                    +np.asarray(result['position'])).tolist()
        # An accepted new placement invalidates any previous simulation export.
        self.csv_path = None
        return result

    def evaluate(self, p, progress=None, *, _settle=False):
        if _settle:
            # Private preparation pass: no drive, exports, or recorded timeline.
            p = dict(p, dt=.01, duration=2., until_destination=False, route=[],
                     speed=0., steering=0., disable_csv=True,
                     vx=0., vy=0., vz=0., wx=0., wy=0., wz=0.)
        def report(stage, processed=0, total=0, fraction=0., **extra):
            if progress is not None:
                progress(dict(stage=stage, processed=processed, total=total,
                              fraction=fraction, **extra))
        report('preparation')
        if self.store is None: raise ValueError('Carga primero una nube PLY')
        disable_csv = p.get('disable_csv', False)
        if not isinstance(disable_csv, bool):
            raise ValueError('La opción de registro CSV debe ser booleana')
        lightweight_physics = p.get('lightweight_physics', False)
        if not isinstance(lightweight_physics, bool):
            raise ValueError('La opción de física ligera debe ser booleana')
        record_csv = not disable_csv
        # A new calculation must never expose an older export as its result.
        if not _settle:
            self.csv_path = None
        def number(key, default, lo, hi, source=p):
            v = float(source.get(key, default))
            if not np.isfinite(v) or not lo <= v <= hi: raise ValueError('Valor inválido: '+key)
            return v
        dt = number('dt', .01, .0005, .01)
        until_destination = p.get('until_destination', False)
        if not isinstance(until_destination, bool):
            raise ValueError('La opción de llegada al destino debe ser booleana')
        duration = number('duration', 0, 0, 60) if not until_destination else 0.
        engine = TruckEngine(PhysicsEngineConfig(num_robots=1, dt=dt, torque_limit=1e7), p.get('vehicle', 'MAN_10t'), lightweight_physics)
        stability = StabilityMonitor(
            mass=engine.robot_model.total_mass,
            inertia_body=engine.inertia.numpy(), cog_body=engine.cog.numpy(),
            wheel_centers=engine.centers.numpy(), tire_radius=engine.radius,
            wheel_sample_count=4 if lightweight_physics else len(engine.centers)*len(engine.ring),
            gravity=engine.config.gravity,
        )
        origin = np.array([number('x', self.center[0], -1e12, 1e12), number('y', self.center[1], -1e12, 1e12), number('z', 0, -1e12, 1e12)])
        euler = [number(k, 0, -180, 180) for k in ('roll', 'pitch', 'yaw')]
        q = Rotation.from_euler('xyz', euler, degrees=True).as_quat()[[3,0,1,2]]
        def ground_seed():
            # Observed geometry is sufficient even where the dynamic footprint
            # lacks coverage. Remove the seed's air gap; never demand equilibrium.
            from placement import DROP_CLEARANCE
            position = origin.copy()
            position[2] -= DROP_CLEARANCE
            return dict(position=position.tolist(), orientation=q.tolist(), settling_time=0.)
        vec = lambda keys: torch.tensor([[number(k,0,-100,100) for k in keys]], dtype=torch.float32)
        state = PhysicsState(x=torch.zeros(1,3), xd=vec(('vx','vy','vz')), q=torch.tensor(q[None], dtype=torch.float32), omega=vec(('wx','wy','wz')), thetas=torch.empty(1,0), batch_size=[1])
        route = p.get('route', [])
        if not isinstance(route, list) or len(route) > 10000: raise ValueError('Ruta inválida')
        route = [dict(x=number('x',0,-1e12,1e12,w),y=number('y',0,-1e12,1e12,w),z=number('z',origin[2],-1e12,1e12,w),speed=number('speed',0,0,100,w),acceleration=number('acceleration',1,-20,20,w)) for w in route]
        if until_destination and len(route) < 2:
            raise ValueError('Se requieren al menos dos puntos para llegar al destino')
        destination = RouteDestination(route, origin) if until_destination else None
        if destination:
            duration = destination.time_limit
        limit_steps = int(duration/dt) + 1
        # Arrival time is estimated from segment lengths and target speeds. The
        # independent safety timeout must not be the progress denominator.
        estimated_steps = max(1, int(np.ceil(min(duration, destination.estimated_duration)/dt)) + 1) if destination else limit_steps
        total_steps = estimated_steps
        def progress_details(time):
            details = dict(time=time, duration=duration, until_destination=until_destination, dt=dt,
                           estimated_steps=estimated_steps, limit_steps=limit_steps)
            if destination:
                details.update(time_limit=duration, distance_remaining=destination.remaining,
                               waypoint_index=min(destination.index, len(route)-1), waypoints_total=len(route))
            return details
        report('preparation', total=total_steps, **progress_details(0.))
        terrain = p.get('terrain') or {}
        classes = {k: dict(v) for k,v in TERRAIN_DEFAULTS.items()}
        classes['desconocido'] = dict(friction=number('friction',.7,0,3),stiffness=number('stiffness',1e6,1000,1e8))
        for name, values in terrain.get('classes', {}).items():
            if name not in classes: raise ValueError('Clase de terreno desconocida: '+name)
            classes[name] = dict(friction=number('friction',classes[name]['friction'],0,3,values),stiffness=number('stiffness',classes[name]['stiffness'],1000,1e8,values))
        zones = terrain.get('zones', [])
        if not isinstance(zones,list) or len(zones)>10000: raise ValueError('Demasiadas zonas')
        for zone in zones:
            center = np.asarray(zone.get('center'),dtype=float)
            if center.shape != (3,) or not np.isfinite(center).all() or zone.get('class') not in classes:
                raise ValueError('Zona de terreno inválida')
            number('radius',1,.001,1e6,zone)
        labels_map = self.store.metadata.get('label_mapping', {})
        def classify(points, labels):
            names = np.array([labels_map.get(str(int(label)), 'desconocido') for label in labels], dtype=object)
            names[~np.isin(names,list(classes))] = 'desconocido'
            # Last edited zone wins; explicit spherical selection in global XYZ.
            for zone in zones:
                selected = np.sum((points-np.array(zone['center']))**2,axis=1) <= float(zone['radius'])**2
                names[selected] = zone['class']
            return names
        # A rolling, independent physical patch; never derived from the visual sphere.
        patch = None
        def physical_patch(global_xy):
            nonlocal patch
            if patch is not None and np.linalg.norm(global_xy-patch[0]) < 4:
                return patch[1:]
            half = 16.
            # Anchor every patch to the same global 20 cm lattice. Settling can
            # shift XY; restarting there must not change the sampled surface.
            patch_center = np.round(np.asarray(global_xy)/.2)*.2
            axis=np.linspace(-half,half,161)
            x,y=np.meshgrid(axis+patch_center[0],axis+patch_center[1])
            queries=np.column_stack([x.ravel(),y.ravel()])
            from placement import SUPPORT_RADIUS
            distances, observed, observed_labels = self.store.upper_xy(queries,max_distance=SUPPORT_RADIUS)
            valid=np.isfinite(distances)
            if valid.sum()<3: raise ValueError('Huella fuera de cobertura: no hay datos locales suficientes')
            # Use actual observed source points returned at the 20cm grid. This
            # bounds memory independently of cloud density. Coverage is conservative:
            # a valid point omitted between grid samples can cause a stop, never
            # invented support. Use the same upper observed envelope as placement,
            # including its source labels, so stacked returns cannot lower a wheel.
            points=observed[valid]
            labels=observed_labels[valid]
            tree=cKDTree(points[:,:2])
            _, fallback=tree.query(queries)
            z=(points[fallback,2]-origin[2]).reshape(161,161)
            z.ravel()[valid]=observed[valid,2]-origin[2]
            # Interpolation assumes origin-centred grid. Re-centre state in the patch only during engine call.
            xx,yy=np.meshgrid(axis,axis)
            world=WorldConfig(torch.tensor(xx[None],dtype=torch.float32),torch.tensor(yy[None],dtype=torch.float32),torch.tensor(z[None],dtype=torch.float32),.2,half)
            patch=(patch_center,tree,points,labels,world)
            return patch[1:]
        # Timed piecewise-linear target. Speed and acceleration are goals, not imposed state.
        route_times = [0.]
        for a,b in zip(route,route[1:]):
            distance=np.hypot(b['x']-a['x'],b['y']-a['y'])
            route_times.append(route_times[-1]+max(dt,distance/max(.1,(a['speed']+b['speed'])/2)))
        def target_at(t):
            if not route: return None
            j=min(int(np.searchsorted(route_times,t,side='right')-1),len(route)-1)
            if j==len(route)-1: return dict(route[j])
            alpha=np.clip((t-route_times[j])/(route_times[j+1]-route_times[j]),0,1)
            return {k:route[j][k]*(1-alpha)+route[j+1][k]*alpha for k in route[j]}
        csv_name = target_csv = None
        if record_csv:
            cache = ROOT / '.cache' / 'exports'
            cache.mkdir(parents=True,exist_ok=True)
            fd, csv_name = tempfile.mkstemp(dir=cache,suffix='.tmp')
            target_csv = Path(csv_name).with_suffix('.csv')
            # Long format: one body row and N stable contact rows per integration sample.
            columns=['time_s','record','contact_id','x_m','y_m','z_m','qw','qx','qy','qz','vx_m_s','vy_m_s','vz_m_s','wx_rad_s','wy_rad_s','wz_rad_s','ax_m_s2','ay_m_s2','az_m_s2','alphax_rad_s2','alphay_rad_s2','alphaz_rad_s2','fx_N','fy_N','fz_N','mx_Nm','my_Nm','mz_Nm','normal_x_N','normal_y_N','normal_z_N','friction_x_N','friction_y_N','friction_z_N','contact_weight','terrain_class','friction_mu','stiffness_N_m','target_x_m','target_y_m','target_z_m','target_speed_m_s','target_acceleration_m_s2','tracking_error_m','speed_error_m_s','tangential_acceleration_m_s2','acceleration_error_m_s2']
            if lightweight_physics:
                columns.append('contact_mode')
        frames=[]; warnings=[]; samples=0; steps=int(duration/dt); stride=max(1,int(.04/dt))
        first_contact_time = None
        contact_pose = None
        commanded=number('speed',0,-20,20)
        status='complete'
        stopped_reason = None
        # Keep only the latest raw sample between visual frames. Conversion to
        # nested Python lists is needed at display cadence, not at physics rate.
        frame=None
        def visual_frame(raw):
            return {key:value.tolist() if isinstance(value,(np.ndarray,torch.Tensor)) else value
                    for key,value in raw.items()}
        try:
            with (os.fdopen(fd,'w',newline='',encoding='utf-8') if record_csv else nullcontext(None)) as f, torch.no_grad():
                if record_csv:
                    writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader()
                for i in range(steps+1):
                    time=i*dt
                    global_position=state.x[0].numpy().astype(float)+origin
                    target=destination.observe(global_position[:2], time) if destination else target_at(time)
                    if destination and destination.reached:
                        status='destination_reached'
                    elif destination and time-destination.last_improvement >= destination.stall_seconds:
                        status='destination_not_reached'; stopped_reason='stall'
                    steering=np.deg2rad(number('steering',0,-39,39))
                    tracking_error=0.;speed_error=0.
                    if target:
                        delta=np.array([target['x'],target['y']])-global_position[:2]
                        tracking_error=float(np.linalg.norm(delta)) if destination else float(np.linalg.norm(np.array([target['x'],target['y'],target['z']])-global_position))
                        if destination:
                            segment_start=origin[:2] if destination.index == 0 else destination.points[destination.index-1]
                            segment=destination.points[destination.index]-segment_start
                            fraction=np.clip(np.dot(global_position[:2]-segment_start,segment)/max(float(np.dot(segment,segment)),1e-12),0,1)
                            tracking_error=float(np.linalg.norm(global_position[:2]-(segment_start+fraction*segment)))
                        # Look ahead on the time-parameterized route for steering direction.
                        look=target if destination else target_at(time+.7)
                        direction=np.array([look['x'],look['y']])-global_position[:2]
                        rotation=q_to_R(state.q)[0].numpy(); heading=np.arctan2(rotation[1,0],rotation[0,0])
                        error=np.arctan2(np.sin(np.arctan2(direction[1],direction[0])-heading),np.cos(np.arctan2(direction[1],direction[0])-heading))
                        steering=np.clip(error,-np.deg2rad(39),np.deg2rad(39))
                        desired=min(20.,target['speed']+min(3.,float(np.linalg.norm(delta))))
                        if destination:
                            previous_speed=route[max(0,destination.index-1)]['speed']
                            cruise=max(.25,(previous_speed+target['speed'])/2)
                            desired=min(20.,cruise,np.sqrt(2*max(.5,abs(target['acceleration']))*float(np.linalg.norm(delta))))
                        elif time>=route_times[-1] and tracking_error<.5: desired=0.
                        # Signed acceleration feed-forward plus wheel-speed target correction.
                        # These commands drive contact forces, never the body state.
                        acceleration_command=np.clip(target['acceleration']+2*(desired-commanded),-20.,20.)
                        commanded=float(np.clip(commanded+acceleration_command*dt,-20.,20.))
                        speed_error=target['speed']-float(np.linalg.norm(state.xd[0,:2].numpy()))
                    controls=torch.tensor([[commanded,steering]],dtype=torch.float32)
                    pts,_,cog,_=engine.assemble_and_transform_robot(state,controls)
                    global_points=pts[0].numpy().astype(float)+origin
                    try:
                        tree,points,labels,world=physical_patch(global_position[:2])
                        distances,indices=tree.query(global_points[:,:2])
                        if np.any(distances>.8): raise ValueError('Huella fuera de cobertura: se requieren datos a menos de 0,8 m')
                    except ValueError as exc:
                        if _settle:
                            return contact_pose or ground_seed()
                        if i==0: raise
                        warnings.append(str(exc));status='coverage_stop'
                        if destination:
                            destination.reached=False
                            destination.remaining=float(np.linalg.norm(destination.points[-1]-global_position[:2]))
                        break
                    names=classify(points[indices], labels[indices])
                    mus=np.array([classes[name]['friction'] for name in names],dtype=np.float32)
                    stiffness=np.array([classes[name]['stiffness'] for name in names],dtype=np.float32)
                    world.k_stiffness=torch.tensor(stiffness[None,:,None]);world.k_friction_lon=torch.tensor(mus[None,:,None]);world.k_friction_lat=world.k_friction_lon
                    # Match the rolling map's origin, restoring state after the engine call.
                    shift=torch.tensor([patch[0][0]-origin[0],patch[0][1]-origin[1],0],dtype=torch.float32)
                    state.x-=shift
                    next_state,der,aux=engine(state,controls,world)
                    state.x+=shift;next_state.x+=shift
                    if not all(torch.isfinite(v).all() for v in (next_state.x,next_state.xd,next_state.q,next_state.omega,der.xdd,der.omega_d)):
                        if _settle:
                            return contact_pose or ground_seed()
                        raise ValueError('Simulación inestable; reduce el paso o cambia la pose')
                    total_forces=(aux.F_spring+aux.F_friction)
                    # Evaluate causal derivatives at the physics cadence before
                    # reducing samples for playback. This never changes forces/state.
                    stability_values = None if _settle else stability.update(
                        time=time, points=global_points,
                        forces=total_forces[0].numpy(), cog=cog[0].numpy().astype(float)+origin,
                        orientation=state.q[0].numpy(), velocity=state.xd[0].numpy(),
                        acceleration=der.xdd[0].numpy(), angular_velocity=state.omega[0].numpy(),
                    )
                    moments=torch.cross(pts-cog[:,None,:],total_forces,dim=-1)
                    normal_values=aux.F_spring[0].numpy()
                    friction_values=aux.F_friction[0].numpy()
                    moment_values=moments[0].numpy()
                    contact_values=aux.in_contact[0,:,0].numpy()
                    horizontal_velocity=state.xd[0,:2].numpy()
                    horizontal_speed=float(np.linalg.norm(horizontal_velocity))
                    tangent=horizontal_velocity/horizontal_speed if horizontal_speed>.1 else q_to_R(state.q)[0,:2,0].numpy()
                    tangent=tangent/max(float(np.linalg.norm(tangent)),1e-9)
                    tangential_acceleration=float(np.dot(der.xdd[0,:2].numpy(),tangent))
                    acceleration_error=(target['acceleration']-tangential_acceleration) if target else 0.
                    if record_csv:
                        # Preserve the existing CSV scalar formatting. Skip all
                        # row preparation as well as writing when disabled.
                        force_values=total_forces[0].numpy()
                        arr=lambda t:t[0].numpy().tolist()
                        common={'tangential_acceleration_m_s2':tangential_acceleration,'acceleration_error_m_s2':acceleration_error,'time_s':time,'target_x_m':target['x'] if target else '', 'target_y_m':target['y'] if target else '', 'target_z_m':target['z'] if target else '', 'target_speed_m_s':target['speed'] if target else '', 'target_acceleration_m_s2':target['acceleration'] if target else '', 'tracking_error_m':tracking_error,'speed_error_m_s':speed_error}
                        if lightweight_physics:
                            common['contact_mode']=engine.contact_mode
                        def components(row,keys,values):
                            row.update(zip(keys,values));return row
                        body=dict(common,record='body',contact_id='')
                        for keys,values in [(columns[3:6],global_position),(columns[6:10],arr(state.q)),(columns[10:13],arr(state.xd)),(columns[13:16],arr(state.omega)),(columns[16:19],arr(der.xdd)),(columns[19:22],arr(der.omega_d)),(columns[22:25],total_forces.sum(1)[0].numpy()),(columns[25:28],arr(aux.torque))]: components(body,keys,values)
                        writer.writerow(body)
                        for j,point in enumerate(global_points):
                            row=dict(common,record='contact',contact_id=engine.contact_ids[j] if lightweight_physics else f'p{j:04d}',contact_weight=float(contact_values[j]),terrain_class=names[j],friction_mu=float(mus[j]),stiffness_N_m=float(stiffness[j]))
                            for keys,values in [(columns[3:6],point),(columns[22:25],force_values[j]),(columns[25:28],moment_values[j]),(columns[28:31],normal_values[j]),(columns[31:34],friction_values[j])]:components(row,keys,values)
                            writer.writerow(row)
                    samples+=1
                    if _settle:
                        # Drop to contact, not to equilibrium. Accept sliding,
                        # tilting and unbalanced loads; they belong in playback.
                        touching = float(total_forces[0, :, 2].sum()) > .05*engine.robot_model.total_mass*engine.config.gravity
                        if touching:
                            if first_contact_time is None:
                                first_contact_time = time
                            contact_pose = dict(position=global_position.tolist(),
                                                orientation=state.q[0].tolist(), settling_time=time)
                            # End near first maximum compression, or shortly
                            # after touching if a slope keeps the body descending.
                            if float(next_state.xd[0, 2]) >= 0 or time-first_contact_time >= .35:
                                return contact_pose
                        state = next_state
                        continue
                    frame=dict(time=time,points=global_points,normal=normal_values,friction=friction_values,moments=moment_values,contact=contact_values,acceleration=der.xdd[0],angular_acceleration=der.omega_d[0],torque=aux.torque[0],position=global_position,orientation=state.q[0],velocity=state.xd[0],angular_velocity=state.omega[0],terrain=names,target=target,tracking_error=tracking_error,speed_error=speed_error,tangential_acceleration=tangential_acceleration,acceleration_error=acceleration_error)
                    frame['stability'] = stability_values
                    if lightweight_physics:
                        frame['contact_ids']=engine.contact_ids
                    if i%stride==0 or i==steps: frames.append(visual_frame(frame))
                    state=next_state
                    if samples == 1 or samples == total_steps or samples % max(1, min(total_steps//200, int(.1/dt))) == 0:
                        report('simulation', samples, total_steps, min(.98, .98*samples/total_steps),
                               **progress_details(time))
                    if destination and (destination.reached or stopped_reason):
                        break
                if _settle:
                    return contact_pose or ground_seed()
                if destination and status == 'complete':
                    status='destination_not_reached'; stopped_reason='timeout'
                if samples and (not frames or frames[-1]['time'] != frame['time']): frames.append(visual_frame(frame))
                report('finalizing', samples, samples if destination else total_steps, .99,
                       **progress_details(0. if frame is None else float(frame['time'])))
                if record_csv:
                    f.flush();os.fsync(f.fileno())
            if record_csv:
                os.replace(csv_name,target_csv)
        except Exception:
            if csv_name is not None:
                Path(csv_name).unlink(missing_ok=True)
            raise
        self.csv_path=target_csv
        if route and frames:
            if max(frame['tracking_error'] for frame in frames)>2: warnings.append('Desvío de ruta superior a 2 m')
            if duration>=1 and abs(frames[-1]['speed_error'])>1: warnings.append('Velocidad objetivo no alcanzada (error > 1 m/s)')
            if max(abs(frame['acceleration_error']) for frame in frames)>1:
                warnings.append('Aceleración objetivo no alcanzada (error > 1 m/s²)')
            last_position=np.array(frames[-1]['position'])
            if (destination and not destination.reached) or (not destination and np.linalg.norm(last_position-[route[-1]['x'],route[-1]['y'],route[-1]['z']])>.5):
                warnings.append('Punto final no alcanzado: tolerancia 0,5 m')
        if destination and not destination.reached:
            if status == 'coverage_stop':
                status='destination_not_reached'; stopped_reason='coverage'
            warnings.append({'stall':'Destino no alcanzado: 15 s sin avance hacia el siguiente punto',
                             'timeout':'Destino no alcanzado: límite de tiempo de la ruta agotado',
                             'coverage':'Destino no alcanzado: la ruta sale de la cobertura'}[stopped_reason])
        result = dict(frames=frames,warnings=warnings,status=status,samples=samples,csv='/api/export.csv' if record_csv else None,frame='global XYZ (m), Z vertical',vehicle=engine.vehicle_id,contact_mode=engine.contact_mode)
        if destination:
            result.update(destination_reached=destination.reached, stopped_reason=stopped_reason,
                          distance_remaining=destination.remaining, time_limit=duration)
        report('complete', samples, samples if destination else total_steps, 1.,
               **progress_details(0. if frame is None else float(frame['time'])))
        return result


sim=Simulation()
if __name__=='__main__':
    from webapp import serve
    parser=argparse.ArgumentParser();parser.add_argument('--ply',type=Path);parser.add_argument('--port',type=int,default=8765);args=parser.parse_args()
    if args.ply: sim.load_path(args.ply)
    print(f'Abre http://127.0.0.1:{args.port}',flush=True)
    serve(sim,ROOT,args.port)

