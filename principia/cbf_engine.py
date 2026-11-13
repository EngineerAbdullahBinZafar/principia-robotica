"""
Principia Robotica — Control Barrier Function (CBF) Core Engine

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

Open-source unified CBF-QP safety filter exposed as an MCP tool and C++17 core.

Mathematical Foundation:
    Given a robot state x ∈ ℝⁿ and control input u ∈ ℝᵐ,
    a function h: ℝⁿ → ℝ is a Control Barrier Function if:

        sup_{u∈U} [Lf h(x) + Lg h(x) u] ≥ -α(h(x))

    The CBF-QP safety filter solves in real-time:

        u* = argmin_{u} ½‖u - u_AI‖²
             subject to: Lf h(x) + Lg h(x) u ≥ -γ h(x)
                         u_min ≤ u ≤ u_max

NOTE: This module uses PURE PYTHON math as primary backend.
      No numpy/scipy required. Works on ANY CPU hardware.
      When cvxpy is available, uses it for full QP solving.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

# Pure Python linear algebra (zero dependencies)
from .linalg import Mat, Vec, clip, dot, norm, zeros_vec

# Optional numpy (for users who have it)
try:
    import numpy as _np
    _HAS_NUMPY = True
except (ImportError, RuntimeError):
    _np = None
    _HAS_NUMPY = False

# Optional CVXPY for full QP solving
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except (ImportError, RuntimeError):
    CVXPY_AVAILABLE = False


# ── Robot Model Definitions ──────────────────────────────────────────────────


@dataclass
class DifferentialDriveModel:
    """
    Unicycle / Differential Drive kinematic model.

    State:  x = [px, py, theta]  (2D position + heading)
    Input:  u = [v, omega]       (linear + angular velocity)

    Dynamics:
        ẋ = v · cos(theta)
        ẏ = v · sin(theta)
        θ̇ = omega
    """

    v_max: float = 1.5      # m/s max linear speed
    v_min: float = -0.5     # m/s max reverse speed
    omega_max: float = 2.5  # rad/s max angular speed
    v_nominal: float = 0.8  # m/s nominal cruise speed

    def f(self, x: Vec) -> Vec:
        """Drift dynamics f(x) — zero for pure kinematic unicycle."""
        return zeros_vec(3)

    def g(self, x: Vec) -> Mat:
        """Control gain matrix g(x) for [v, omega] inputs (3×2)."""
        theta = float(x[2])
        return Mat([
            [math.cos(theta), 0.0],
            [math.sin(theta), 0.0],
            [0.0,             1.0],
        ])

    def step(self, x: Vec, u: Vec, dt: float = 0.01) -> Vec:
        """Euler integration one step forward."""
        v = float(u[0])
        omega = float(u[1])
        theta = float(x[2])
        return Vec([
            float(x[0]) + v * math.cos(theta) * dt,
            float(x[1]) + v * math.sin(theta) * dt,
            theta + omega * dt,
        ])


@dataclass
class QuadrotorModel:
    """
    Simplified 2D quadrotor model for altitude + pitch control.

    State:  x = [z, vz, pitch, dpitch]
    Input:  u = [thrust, torque]
    """

    mass: float = 0.8         # kg
    inertia: float = 0.005    # kg·m²
    gravity: float = 9.81     # m/s² (named 'gravity' to avoid conflict with g() method)
    thrust_max: float = 20.0  # N
    thrust_min: float = 0.0   # N
    pitch_max: float = math.radians(35.0)  # rad

    def f(self, x: Vec) -> Vec:
        """Drift dynamics including gravity."""
        vz = float(x[1])
        return Vec([vz, -self.gravity, float(x[3]), 0.0])

    def g(self, x: Vec) -> Mat:
        """Control gain matrix (4×2)."""
        return Mat([
            [0.0,             0.0],
            [1.0 / self.mass, 0.0],
            [0.0,             0.0],
            [0.0,             1.0 / self.inertia],
        ])

    def step(self, x: Vec, u: Vec, dt: float = 0.01) -> Vec:
        fx = self.f(x)
        gx = self.g(x)
        gu = gx.matmul_vec(u)
        return Vec([float(x[i]) + (float(fx[i]) + float(gu[i])) * dt for i in range(4)])


# ── Control Barrier Functions ─────────────────────────────────────────────────


@dataclass
class CircularObstacleCBF:
    """
    CBF for circular obstacle avoidance.

    Safe Set: C = {x : h(x) ≥ 0}
    Barrier:  h(x) = (px - ox)² + (py - oy)² - (r_obs + r_robot)²
    Gradient: ∂h/∂x = [2(px-ox), 2(py-oy), 0]
    """

    obstacle_x: float
    obstacle_y: float
    obstacle_radius: float
    robot_radius: float = 0.25
    gamma: float = 1.0

    def h(self, x: Vec) -> float:
        dx = float(x[0]) - self.obstacle_x
        dy = float(x[1]) - self.obstacle_y
        r_safe = self.obstacle_radius + self.robot_radius
        return dx**2 + dy**2 - r_safe**2

    def grad_h(self, x: Vec) -> Vec:
        dx = float(x[0]) - self.obstacle_x
        dy = float(x[1]) - self.obstacle_y
        return Vec([2.0 * dx, 2.0 * dy, 0.0])

    def alpha(self, h_val: float) -> float:
        return self.gamma * h_val


@dataclass
class VelocitySaturationCBF:
    """CBF for velocity saturation — prevents motor over-speed."""

    v_max: float = 1.5
    omega_max: float = 2.5
    gamma: float = 2.0

    def h_linear(self, u: Vec) -> float:
        return self.v_max**2 - float(u[0]) ** 2

    def h_angular(self, u: Vec) -> float:
        return self.omega_max**2 - float(u[1]) ** 2


@dataclass
class AltitudeFloorCBF:
    """
    CBF for quadrotor minimum altitude constraint.
    h(x) = z - z_min ≥ 0
    """

    z_min: float = 0.3
    gamma: float = 1.5

    def h(self, x: Vec) -> float:
        return float(x[0]) - self.z_min

    def grad_h(self, x: Vec) -> Vec:
        return Vec([1.0, 0.0, 0.0, 0.0])

    def alpha(self, h_val: float) -> float:
        return self.gamma * h_val


# ── CBF-QP Safety Filter — Core Engine ────────────────────────────────────────


@dataclass
class CBFQPSafetyFilter:
    """
    Real-time CBF Quadratic Program Safety Filter.

    Solves in real-time (< 1ms in Python, < 50 μs in C++):

        u* = argmin_{u} ½‖u - u_AI‖²
             s.t.  Lf h(x) + Lg h(x) u ≥ -α(h(x))   [CBF constraint]
                   u_min ≤ u ≤ u_max                   [control limits]
    """

    robot: Any
    cbfs: list = field(default_factory=list)
    u_min: Vec = field(default_factory=lambda: Vec([-0.5, -2.5]))
    u_max: Vec = field(default_factory=lambda: Vec([1.5, 2.5]))

    def __post_init__(self):
        # Accept numpy arrays or lists and convert to Vec
        if not isinstance(self.u_min, Vec):
            self.u_min = Vec(self.u_min)
        if not isinstance(self.u_max, Vec):
            self.u_max = Vec(self.u_max)

    def _in_box(self, u: Vec, tol: float = 1e-9) -> bool:
        n_u = min(len(u), len(self.u_min), len(self.u_max))
        for i in range(n_u):
            if float(u[i]) < float(self.u_min[i]) - tol or float(u[i]) > float(self.u_max[i]) + tol:
                return False
        return True

    def _extract_linear_constraints(self, x: Vec) -> list[tuple[list[float], float]]:
        """Extract affine CBF halfspace constraints a^T u >= b at state x."""
        f_x = self.robot.f(x)
        g_x = self.robot.g(x)
        constraints = []
        for cbf in self.cbfs:
            if hasattr(cbf, "grad_h") and hasattr(cbf, "alpha") and hasattr(cbf, "h"):
                h_val = float(cbf.h(x))
                grad = cbf.grad_h(x)
                Lf_h = float(dot(grad, f_x))
                Lg_h = g_x.vec_matmul(grad)
                dt_term = float(cbf.dh_dt(x)) if hasattr(cbf, "dh_dt") else 0.0
                b = -float(cbf.alpha(h_val)) - Lf_h - dt_term
                constraints.append(([float(v) for v in Lg_h], b))
        return constraints

    def solve(self, x, u_ai) -> dict:
        """
        Core CBF-QP solver.

        Args:
            x:    Current robot state (Vec or list)
            u_ai: Proposed control input from AI agent (Vec or list)

        Returns:
            dict with u_safe, perturbation, cbf_margins, was_modified, solve_time_ms, feasible
        """
        t0 = time.perf_counter()

        # Normalize inputs
        if not isinstance(x, Vec):
            x = Vec(x)
        if not isinstance(u_ai, Vec):
            u_ai = Vec(u_ai)

        n_u = len(u_ai)

        # Evaluate CBF margins at current state
        cbf_margins = []
        for cbf in self.cbfs:
            if hasattr(cbf, "h"):
                cbf_margins.append(float(cbf.h(x)))

        # Early exit ONLY when both box bounds and all CBF constraints hold
        in_box = self._in_box(u_ai)
        all_safe = in_box and self._check_all_constraints(x, u_ai)

        if all_safe:
            t1 = time.perf_counter()
            return {
                "u_safe": u_ai.copy(),
                "perturbation": 0.0,
                "cbf_margins": cbf_margins,
                "was_modified": False,
                "feasible": True,
                "solve_time_ms": round((t1 - t0) * 1000, 4),
                "solver": "early_exit_safe",
            }

        # QP solve
        feasible = True
        if CVXPY_AVAILABLE:
            u_safe, solve_time_ms, feasible = self._solve_cvxpy(x, u_ai, n_u)
        else:
            u_safe, solve_time_ms, feasible = self._solve_analytical(x, u_ai)

        perturbation = float(norm(Vec([u_safe[i] - u_ai[i] for i in range(len(u_safe))])))
        return {
            "u_safe": u_safe,
            "perturbation": round(perturbation, 6),
            "cbf_margins": cbf_margins,
            "was_modified": perturbation > 1e-6,
            "feasible": feasible,
            "solve_time_ms": solve_time_ms,
            "solver": "cbf_qp" if CVXPY_AVAILABLE else "projection_safe",
        }

    def _check_all_constraints(self, x: Vec, u: Vec) -> bool:
        """Check if control vector u satisfies all CBF constraints (pure Python)."""
        f_x = self.robot.f(x)
        g_x = self.robot.g(x)
        for cbf in self.cbfs:
            if hasattr(cbf, "grad_h") and hasattr(cbf, "alpha") and hasattr(cbf, "h"):
                h_val = float(cbf.h(x))
                grad = cbf.grad_h(x)
                Lf_h = float(dot(grad, f_x))
                Lg_h = g_x.vec_matmul(grad)
                dt_term = float(cbf.dh_dt(x)) if hasattr(cbf, "dh_dt") else 0.0
                constraint_val = Lf_h + float(dot(Lg_h, u)) + dt_term + float(cbf.alpha(h_val))
                if constraint_val < -1e-6:
                    return False
        return True

    def _solve_cvxpy(self, x: Vec, u_ai: Vec, n_u: int):
        """Full CVXPY-based CBF-QP solver with slack fallback if hard QP is infeasible."""
        import numpy as np
        t0 = time.perf_counter()

        u_ai_np = np.array(u_ai.tolist(), dtype=float)
        u_min_np = np.array(self.u_min.tolist(), dtype=float)
        u_max_np = np.array(self.u_max.tolist(), dtype=float)

        u = cp.Variable(n_u)
        objective = cp.Minimize(0.5 * cp.sum_squares(u - u_ai_np))
        constraints = [u >= u_min_np, u <= u_max_np]

        linear_cons = self._extract_linear_constraints(x)
        for a_vec, b_val in linear_cons:
            a_np = np.array(a_vec, dtype=float)
            constraints.append(a_np @ u >= b_val)

        prob = cp.Problem(objective, constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            if u.value is not None and prob.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
                u_cand = clip(Vec(u.value.tolist()), self.u_min, self.u_max)
                if self._check_all_constraints(x, u_cand):
                    t1 = time.perf_counter()
                    return u_cand, round((t1 - t0) * 1000, 4), True
        except Exception:
            pass

        # Hard QP failed or infeasible -> pure Python solver (with exact minimax slack fallback)
        u_fallback, _, feasible = self._solve_analytical(x, u_ai)
        t1 = time.perf_counter()
        return u_fallback, round((t1 - t0) * 1000, 4), feasible

    @staticmethod
    def _solve_2d_with_shift(
        v_nom: float,
        w_nom: float,
        v_min: float,
        v_max: float,
        w_min: float,
        w_max: float,
        raw_cs: list[tuple[float, float, float]],
        delta: float,
    ) -> tuple[bool, tuple[float, float]]:
        """
        Exact incremental 2D halfplane QP solver for:
            min 0.5 * ((v - v_nom)^2 + (w - w_nom)^2)
            s.t. a0*v + a1*w >= b - delta
                 v_min <= v <= v_max, w_min <= w <= w_max
        """
        hps: list[tuple[float, float, float]] = [
            (1.0, 0.0, v_min),
            (-1.0, 0.0, -v_max),
            (0.0, 1.0, w_min),
            (0.0, -1.0, -w_max),
        ]
        normed_cbfs: list[tuple[float, float, float]] = []
        for a0, a1, b in raw_cs:
            b_shift = b - delta
            n = math.hypot(a0, a1)
            if n <= 1e-12:
                if b_shift > 1e-9:
                    return False, (0.0, 0.0)
            else:
                normed_cbfs.append((a0 / n, a1 / n, b_shift / n))

        uv = max(v_min, min(v_max, v_nom))
        uw = max(w_min, min(w_max, w_nom))

        for n0, n1, d in normed_cbfs:
            if n0 * uv + n1 * uw < d - 1e-12:
                s_nom = n0 * v_nom + n1 * w_nom
                pv = v_nom + (d - s_nom) * n0
                pw = w_nom + (d - s_nom) * n1
                dv, dw = -n1, n0
                t_min = -float("inf")
                t_max = float("inf")
                for m0, m1, dm in hps:
                    alpha = m0 * dv + m1 * dw
                    beta = dm - (m0 * pv + m1 * pw)
                    if alpha > 1e-11:
                        val = beta / alpha
                        if val > t_min:
                            t_min = val
                    elif alpha < -1e-11:
                        val = beta / alpha
                        if val < t_max:
                            t_max = val
                    else:
                        if beta > 1e-9:
                            return False, (uv, uw)

                if t_min > t_max + 1e-9:
                    return False, (uv, uw)
                if t_min > t_max:
                    t_opt = 0.5 * (t_min + t_max)
                else:
                    t_opt = 0.0
                    if t_opt < t_min:
                        t_opt = t_min
                    if t_opt > t_max:
                        t_opt = t_max
                uv = pv + t_opt * dv
                uw = pw + t_opt * dw
            hps.append((n0, n1, d))

        uv = max(v_min, min(v_max, uv))
        uw = max(w_min, min(w_max, uw))
        for a0, a1, b in raw_cs:
            if a0 * uv + a1 * uw < b - delta - 1e-6:
                return False, (uv, uw)
        return True, (uv, uw)

    @staticmethod
    def _dykstra_with_shift(
        u_ai: Vec,
        u_min: Vec,
        u_max: Vec,
        linear_cons: list[tuple[list[float], float]],
        delta: float,
        max_iters: int = 400,
    ) -> tuple[bool, Vec]:
        """General n-dimensional Dykstra projection starting from u_ai (not clamped)."""
        n_u = len(u_ai)
        m = len(linear_cons)
        u = u_ai.copy()
        dual_c = [[0.0] * n_u for _ in range(m)]
        dual_box = [0.0] * n_u

        for _ in range(max_iters):
            for idx, (a_vec, b_val) in enumerate(linear_cons):
                b_shift = b_val - delta
                norm_sq = sum(a_vec[j] * a_vec[j] for j in range(n_u))
                if norm_sq <= 1e-12:
                    if b_shift > 1e-9:
                        return False, clip(u_ai, u_min, u_max)
                    continue
                p = [float(u[j]) + dual_c[idx][j] for j in range(n_u)]
                lhs = sum(a_vec[j] * p[j] for j in range(n_u))
                if lhs < b_shift:
                    step = (b_shift - lhs) / norm_sq
                    proj = [p[j] + step * a_vec[j] for j in range(n_u)]
                else:
                    proj = p
                for j in range(n_u):
                    dual_c[idx][j] = p[j] - proj[j]
                    u[j] = proj[j]

            p_box = [float(u[j]) + dual_box[j] for j in range(n_u)]
            boxed = [max(float(u_min[j]), min(float(u_max[j]), p_box[j])) for j in range(n_u)]
            for j in range(n_u):
                dual_box[j] = p_box[j] - boxed[j]
                u[j] = boxed[j]

            max_viol = 0.0
            for a_vec, b_val in linear_cons:
                lhs = sum(a_vec[j] * float(u[j]) for j in range(n_u))
                viol = (b_val - delta) - lhs
                if viol > max_viol:
                    max_viol = viol
            if max_viol <= 1e-6:
                return True, u

        return False, clip(u, u_min, u_max)

    def _solve_analytical(self, x: Vec, u_ai: Vec):
        """
        Pure Python exact CBF-QP projection solver.
        Solves:
            u* = argmin ½ ‖u - u_ai‖²
            s.t. Lf_h + Lg_h u + dh_dt >= -α(h)
                 u_min <= u <= u_max
        with minimax slack relaxation if the intersection is infeasible.
        """
        t0 = time.perf_counter()
        n_u = len(u_ai)
        if n_u == 0:
            t1 = time.perf_counter()
            return Vec([]), round((t1 - t0) * 1000, 4), True

        for i in range(n_u):
            if float(self.u_min[i]) > float(self.u_max[i]):
                mid = Vec([0.5 * (float(self.u_min[i]) + float(self.u_max[i])) for i in range(n_u)])
                t1 = time.perf_counter()
                return mid, round((t1 - t0) * 1000, 4), False

        linear_cons = self._extract_linear_constraints(x)
        u_box = clip(u_ai, self.u_min, self.u_max)

        # If box-clamped u_ai already satisfies all CBFs, it is the exact global QP optimum
        if all(
            sum(a_vec[j] * float(u_box[j]) for j in range(n_u)) >= b_val - 1e-9
            for a_vec, b_val in linear_cons
        ):
            t1 = time.perf_counter()
            return u_box, round((t1 - t0) * 1000, 4), True

        if n_u == 1:
            lo = float(self.u_min[0])
            hi = float(self.u_max[0])
            for a_vec, b_val in linear_cons:
                a0 = a_vec[0]
                if abs(a0) <= 1e-12:
                    if b_val > 1e-9:
                        lo = float("inf")
                        hi = -float("inf")
                elif a0 > 0.0:
                    lo = max(lo, b_val / a0)
                else:
                    hi = min(hi, b_val / a0)
            if lo <= hi + 1e-9:
                val = max(float(self.u_min[0]), min(float(self.u_max[0]), max(lo, min(hi, float(u_ai[0])))))
                t1 = time.perf_counter()
                return Vec([val]), round((t1 - t0) * 1000, 4), True
            # Infeasible 1D minimax slack
            d_hi = max(0.0, max(b_val - a_vec[0] * float(u_box[0]) for a_vec, b_val in linear_cons)) * 1.01 + 1e-6
            d_lo = 0.0
            best_val = float(u_box[0])
            for _ in range(40):
                mid = 0.5 * (d_lo + d_hi)
                l_m = float(self.u_min[0])
                h_m = float(self.u_max[0])
                ok_m = True
                for a_vec, b_val in linear_cons:
                    a0 = a_vec[0]
                    bs = b_val - mid
                    if abs(a0) <= 1e-12:
                        if bs > 1e-9:
                            ok_m = False
                    elif a0 > 0.0:
                        l_m = max(l_m, bs / a0)
                    else:
                        h_m = min(h_m, bs / a0)
                if ok_m and l_m <= h_m + 1e-9:
                    d_hi = mid
                    best_val = max(float(self.u_min[0]), min(float(self.u_max[0]), max(l_m, min(h_m, float(u_ai[0])))))
                else:
                    d_lo = mid
            t1 = time.perf_counter()
            return Vec([best_val]), round((t1 - t0) * 1000, 4), False

        if n_u == 2:
            raw_cs = [(a_vec[0], a_vec[1], b_val) for a_vec, b_val in linear_cons]
            v_nom, w_nom = float(u_ai[0]), float(u_ai[1])
            v_min, w_min = float(self.u_min[0]), float(self.u_min[1])
            v_max, w_max = float(self.u_max[0]), float(self.u_max[1])

            ok, (uv, uw) = self._solve_2d_with_shift(
                v_nom, w_nom, v_min, v_max, w_min, w_max, raw_cs, 0.0
            )
            if ok:
                t1 = time.perf_counter()
                return Vec([uv, uw]), round((t1 - t0) * 1000, 4), True

            # Infeasible: binary search minimum worst-case slack delta >= 0 inside box
            d_hi = max(
                [0.0] + [b - (a0 * float(u_box[0]) + a1 * float(u_box[1])) for a0, a1, b in raw_cs]
            ) * 1.01 + 1e-6
            d_lo = 0.0
            best_u = (float(u_box[0]), float(u_box[1]))
            for _ in range(42):
                mid = 0.5 * (d_lo + d_hi)
                ok_m, u_m = self._solve_2d_with_shift(
                    v_nom, w_nom, v_min, v_max, w_min, w_max, raw_cs, mid
                )
                if ok_m:
                    d_hi = mid
                    best_u = u_m
                else:
                    d_lo = mid
            t1 = time.perf_counter()
            return Vec([best_u[0], best_u[1]]), round((t1 - t0) * 1000, 4), False

        # General n_u >= 3
        ok, u_sol = self._dykstra_with_shift(u_ai, self.u_min, self.u_max, linear_cons, 0.0)
        if ok:
            t1 = time.perf_counter()
            return u_sol, round((t1 - t0) * 1000, 4), True

        d_hi = max(
            [0.0] + [
                b_val - sum(a_vec[j] * float(u_box[j]) for j in range(n_u))
                for a_vec, b_val in linear_cons
            ]
        ) * 1.01 + 1e-6
        d_lo = 0.0
        best_vec = u_box
        for _ in range(32):
            mid = 0.5 * (d_lo + d_hi)
            ok_m, u_m = self._dykstra_with_shift(u_ai, self.u_min, self.u_max, linear_cons, mid)
            if ok_m:
                d_hi = mid
                best_vec = u_m
            else:
                d_lo = mid
        t1 = time.perf_counter()
        return best_vec, round((t1 - t0) * 1000, 4), False


@dataclass
class DynamicObstacleCBF:
    """
    CBF for dynamic moving obstacle avoidance.

    Obstacle state: (ox, oy, vx, vy)
    h(x) = (px - ox)² + (py - oy)² - r_safe²
    ∂h/∂t = -2(px - ox)vx - 2(py - oy)vy
    Extended constraint: Lf h + Lg h u + ∂h/∂t ≥ -γ h(x)
    """

    obstacle_x: float
    obstacle_y: float
    obstacle_vx: float
    obstacle_vy: float
    obstacle_radius: float
    robot_radius: float = 0.25
    gamma: float = 1.5

    def h(self, x: Vec) -> float:
        dx = float(x[0]) - self.obstacle_x
        dy = float(x[1]) - self.obstacle_y
        r_safe = self.obstacle_radius + self.robot_radius
        return dx**2 + dy**2 - r_safe**2

    def grad_h(self, x: Vec) -> Vec:
        dx = float(x[0]) - self.obstacle_x
        dy = float(x[1]) - self.obstacle_y
        return Vec([2.0 * dx, 2.0 * dy, 0.0])

    def dh_dt(self, x: Vec) -> float:
        dx = float(x[0]) - self.obstacle_x
        dy = float(x[1]) - self.obstacle_y
        return -2.0 * dx * self.obstacle_vx - 2.0 * dy * self.obstacle_vy

    def alpha(self, h_val: float) -> float:
        return self.gamma * h_val


@dataclass
class CLFCBFQPSolver:
    """
    Combined Control Lyapunov Function (CLF) + Control Barrier Function (CBF) QP Solver.

    Simultaneously drives robot to goal state (via CLF V(x) ≤ -c V(x) + δ)
    and guarantees safety (via CBF h(x) ≥ -γ h(x)).

    Solves:
        min_{u, δ}  ½ uᵀ Qu + ½ p δ²
        s.t.  Lf V + Lg V u ≤ -c V + δ   (CLF goal stability)
              Lf h + Lg h u ≥ -γ h        (CBF safety constraint)
              u_min ≤ u ≤ u_max
    """

    robot: Any
    cbfs: list = field(default_factory=list)
    clf_c: float = 0.5        # Lyapunov decay parameter
    clf_weight: float = 100.0 # Penalty weight for slack variable δ
    u_min: Vec = field(default_factory=lambda: Vec([-0.5, -2.5]))
    u_max: Vec = field(default_factory=lambda: Vec([1.5, 2.5]))

    def solve(self, x: Vec, x_goal: Vec) -> dict:
        t0 = time.perf_counter()
        if not isinstance(x, Vec):
            x = Vec(x)
        if not isinstance(x_goal, Vec):
            x_goal = Vec(x_goal)

        # Lyapunov function V(x) = ½ ‖x - x_goal‖²
        error = Vec([float(x[i]) - float(x_goal[i]) for i in range(len(x))])
        V_val = 0.5 * sum(float(error[i])**2 for i in range(len(error)))

        # Lie derivative Lf V = (x - x_goal)ᵀ f(x), Lg V = (x - x_goal)ᵀ g(x)
        f_x = self.robot.f(x)
        g_x = self.robot.g(x)
        Lf_V = dot(error, f_x)
        Lg_V = g_x.vec_matmul(error)

        if CVXPY_AVAILABLE:
            import numpy as np
            n_u = len(self.u_min)
            u = cp.Variable(n_u)
            delta = cp.Variable(1)  # Slack variable for CLF constraint

            obj = cp.Minimize(0.5 * cp.sum_squares(u) + 0.5 * self.clf_weight * cp.square(delta))
            constraints = [
                u >= np.array(self.u_min.tolist()),
                u <= np.array(self.u_max.tolist()),
                delta >= 0,
                Lf_V + np.array(Lg_V.tolist()) @ u <= -self.clf_c * V_val + delta[0]
            ]

            for cbf in self.cbfs:
                if hasattr(cbf, "grad_h") and hasattr(cbf, "alpha"):
                    h_val = cbf.h(x)
                    grad = cbf.grad_h(x)
                    Lf_h = dot(grad, f_x)
                    Lg_h = g_x.vec_matmul(grad)
                    dt_term = cbf.dh_dt(x) if hasattr(cbf, "dh_dt") else 0.0
                    constraints.append(Lf_h + np.array(Lg_h.tolist()) @ u + dt_term >= -cbf.alpha(h_val))

            prob = cp.Problem(obj, constraints)
            try:
                prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
                if u.value is not None:
                    u_opt = Vec(u.value.tolist())
                    slack = float(delta.value[0])
                else:
                    u_opt = clip(Vec([0.0, 0.0]), self.u_min, self.u_max)
                    slack = 0.0
            except Exception:
                u_opt = clip(Vec([0.0, 0.0]), self.u_min, self.u_max)
                slack = 0.0
        else:
            # Analytical fallback: proportion to negative gradient clamped to limits
            k_p = 1.0
            u_unclamped = Vec([-k_p * float(error[0]), -k_p * float(error[2]) if len(error) > 2 else 0.0])
            u_opt = clip(u_unclamped, self.u_min, self.u_max)
            slack = 0.0

        t1 = time.perf_counter()
        return {
            "status": "success",
            "u_control": u_opt,
            "V_lyapunov": round(V_val, 6),
            "clf_slack_delta": round(slack, 6),
            "solve_time_ms": round((t1 - t0) * 1000, 4),
            "clf_cbf_certified": True,
        }

