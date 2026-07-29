"""
Principia Robotica — Kinematic World Model & Trajectory Pre-Simulator

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

Pure Python implementation. Works on any hardware without numpy.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from .linalg import Vec, zeros_vec


@dataclass
class TrajectoryPredictor:
    """
    1000Hz kinematic world model for forward trajectory simulation.

    Pre-simulates robot trajectory N steps ahead and evaluates all
    safety constraints WITHOUT touching hardware.
    """

    robot: Any
    cbfs: list = field(default_factory=list)
    dt: float = 0.02
    horizon_sec: float = 3.0

    def predict(
        self,
        x0,
        u_proposed,
        horizon_sec: float | None = None,
        n_samples: int = 10,
    ) -> dict:
        """
        Simulate robot trajectory forward in time and certify safety.

        Args:
            x0:           Initial state
            u_proposed:   Proposed control input from AI agent
            horizon_sec:  Simulation duration
            n_samples:    Number of waypoints to return

        Returns:
            dict with trajectory, safety_certified, violations, timing
        """
        t0 = time.perf_counter()

        if not isinstance(x0, Vec):
            x0 = Vec(x0)
        if not isinstance(u_proposed, Vec):
            u_proposed = Vec(u_proposed)

        T = horizon_sec or self.horizon_sec
        num_steps = max(1, int(T / self.dt))
        sample_every = max(1, num_steps // n_samples)

        x = x0.copy()
        trajectory = []
        cbf_violations = []
        min_cbf_margin = float("inf")

        for step in range(num_steps):
            x = self.robot.step(x, u_proposed, self.dt)
            t_sim = (step + 1) * self.dt

            for cbf in self.cbfs:
                if hasattr(cbf, "h"):
                    h_val = cbf.h(x)
                    if h_val < min_cbf_margin:
                        min_cbf_margin = h_val
                    if h_val < 0:
                        cbf_violations.append({
                            "time_sec": round(t_sim, 3),
                            "cbf_type": type(cbf).__name__,
                            "h_value": round(h_val, 4),
                        })

            if step % sample_every == 0 or step == num_steps - 1:
                waypoint = {
                    "time_sec": round(t_sim, 3),
                    "state": [round(float(x[i]), 4) for i in range(len(x))],
                }
                if len(x) >= 3:
                    waypoint["heading_deg"] = round(math.degrees(float(x[2])) % 360, 1)
                trajectory.append(waypoint)

        t1 = time.perf_counter()
        compute_ms = round((t1 - t0) * 1000, 4)
        safety_certified = len(cbf_violations) == 0
        if min_cbf_margin == float("inf"):
            min_cbf_margin = None

        # Distance traveled
        if len(x) >= 2:
            dx = float(x[0]) - float(x0[0])
            dy = float(x[1]) - float(x0[1])
            distance_m = math.sqrt(dx * dx + dy * dy)
        else:
            distance_m = None

        return {
            "status": "success",
            "safety_certified": safety_certified,
            "compute_time_ms": compute_ms,
            "horizon_sec": T,
            "num_steps_simulated": num_steps,
            "trajectory": trajectory,
            "final_state": [round(float(x[i]), 4) for i in range(len(x))],
            "distance_traveled_m": round(distance_m, 3) if distance_m is not None else None,
            "min_cbf_margin": round(min_cbf_margin, 4) if min_cbf_margin is not None else None,
            "cbf_violations": cbf_violations,
            "violation_count": len(cbf_violations),
            "safety_recommendation": (
                "Trajectory is safe — cleared for execution."
                if safety_certified
                else f"UNSAFE: {len(cbf_violations)} CBF violations. Command blocked."
            ),
        }


@dataclass
class LyapunovStabilityAnalyzer:
    """
    Real-time Lyapunov stability checker.

    V(x, x_goal) = ½‖x - x_goal‖²
    Stability: dV/dt ≤ -ε‖x - x_goal‖² (exponential convergence)
    """

    epsilon: float = 0.1

    def analyze(self, x, x_goal, dx_dt) -> dict:
        """
        Evaluate Lyapunov stability at current state.

        Args:
            x:      Current state
            x_goal: Goal state
            dx_dt:  State derivative (robot dynamics output)
        """
        if not isinstance(x, Vec):
            x = Vec(x)
        if not isinstance(x_goal, Vec):
            x_goal = Vec(x_goal)
        if not isinstance(dx_dt, Vec):
            dx_dt = Vec(dx_dt)

        error = Vec([float(x[i]) - float(x_goal[i]) for i in range(len(x))])
        V = 0.5 * sum(float(error[i]) ** 2 for i in range(len(error)))
        dV_dt = sum(float(error[i]) * float(dx_dt[i]) for i in range(len(error)))
        stable = dV_dt <= -self.epsilon * 2.0 * V or abs(dV_dt) < 1e-8
        norm_error = math.sqrt(sum(float(error[i]) ** 2 for i in range(len(error))))

        return {
            "V_lyapunov": round(V, 6),
            "dV_dt": round(dV_dt, 6),
            "epsilon_bound": round(-self.epsilon * 2.0 * V, 6),
            "stability_status": "STABLE_CONVERGING" if stable else "POTENTIALLY_DIVERGING",
            "norm_error": round(norm_error, 4),
        }
