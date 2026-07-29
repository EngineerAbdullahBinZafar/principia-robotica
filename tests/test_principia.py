"""
Principia Robotica — Complete Test Suite

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

Pure Python tests — no numpy required.
"""

import json
import math
import sys

import pytest

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent))

from principia.cbf_engine import (
    AltitudeFloorCBF,
    CBFQPSafetyFilter,
    CircularObstacleCBF,
    DifferentialDriveModel,
    QuadrotorModel,
    VelocitySaturationCBF,
)
from principia.linalg import Vec
from principia.world_model import LyapunovStabilityAnalyzer, TrajectoryPredictor

# ─────────────────────────────────────────────────────────────────────────────
# §1  Robot Model Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestDifferentialDriveModel:
    def test_instantiation(self):
        robot = DifferentialDriveModel()
        assert robot.v_max == 1.5
        assert robot.omega_max == 2.5

    def test_f_returns_zeros(self):
        robot = DifferentialDriveModel()
        x = Vec([0.0, 0.0, 0.0])
        f = robot.f(x)
        assert all(abs(float(f[i])) < 1e-10 for i in range(len(f)))

    def test_g_shape(self):
        robot = DifferentialDriveModel()
        x = Vec([0.0, 0.0, 0.0])
        g = robot.g(x)
        assert g.shape == (3, 2)

    def test_g_heading_zero(self):
        robot = DifferentialDriveModel()
        x = Vec([0.0, 0.0, 0.0])
        g = robot.g(x)
        assert abs(g[0][0] - 1.0) < 1e-10
        assert abs(g[1][0] - 0.0) < 1e-10

    def test_g_heading_pi_over_2(self):
        robot = DifferentialDriveModel()
        x = Vec([0.0, 0.0, math.pi / 2])
        g = robot.g(x)
        assert abs(g[0][0]) < 1e-5
        assert abs(g[1][0] - 1.0) < 1e-5

    def test_step_moves_forward(self):
        robot = DifferentialDriveModel()
        x = Vec([0.0, 0.0, 0.0])
        u = Vec([1.0, 0.0])
        x_new = robot.step(x, u, dt=0.1)
        assert float(x_new[0]) > 0.0
        assert abs(float(x_new[1])) < 1e-10
        assert abs(float(x_new[2])) < 1e-10

    def test_step_rotation(self):
        robot = DifferentialDriveModel()
        x = Vec([0.0, 0.0, 0.0])
        u = Vec([0.0, 1.0])
        x_new = robot.step(x, u, dt=0.1)
        assert abs(float(x_new[2]) - 0.1) < 1e-10


class TestQuadrotorModel:
    def test_instantiation(self):
        quad = QuadrotorModel()
        assert quad.mass == 0.8
        assert abs(quad.gravity - 9.81) < 1e-10  # 'gravity' attr, 'g' is the control gain method

    def test_f_includes_gravity(self):
        quad = QuadrotorModel()
        x = Vec([1.0, 0.0, 0.0, 0.0])
        f = quad.f(x)
        assert abs(float(f[1]) + 9.81) < 1e-10

    def test_g_shape(self):
        quad = QuadrotorModel()
        x = Vec([1.0, 0.0, 0.0, 0.0])
        g = quad.g(x)  # g is the control gain method, called with state x
        assert g.shape == (4, 2)

    def test_step_gravity_decreases_vz(self):
        quad = QuadrotorModel()
        x = Vec([5.0, 0.0, 0.0, 0.0])
        u = Vec([0.0, 0.0])
        x_new = quad.step(x, u, dt=0.01)
        assert float(x_new[1]) < 0


# ─────────────────────────────────────────────────────────────────────────────
# §2  CBF Function Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestCircularObstacleCBF:
    def test_h_positive_outside(self):
        cbf = CircularObstacleCBF(obstacle_x=0.0, obstacle_y=0.0, obstacle_radius=0.5, robot_radius=0.25)
        x = Vec([2.0, 0.0, 0.0])
        assert cbf.h(x) > 0

    def test_h_negative_inside(self):
        cbf = CircularObstacleCBF(obstacle_x=0.0, obstacle_y=0.0, obstacle_radius=0.5, robot_radius=0.25)
        x = Vec([0.1, 0.0, 0.0])
        assert cbf.h(x) < 0

    def test_h_zero_at_boundary(self):
        r_safe = 0.75
        cbf = CircularObstacleCBF(obstacle_x=0.0, obstacle_y=0.0, obstacle_radius=0.5, robot_radius=0.25)
        x = Vec([r_safe, 0.0, 0.0])
        assert abs(cbf.h(x)) < 1e-10

    def test_grad_h_shape(self):
        cbf = CircularObstacleCBF(obstacle_x=1.0, obstacle_y=0.0, obstacle_radius=0.3)
        x = Vec([2.0, 0.0, 0.0])
        assert len(cbf.grad_h(x)) == 3

    def test_grad_h_direction(self):
        cbf = CircularObstacleCBF(obstacle_x=0.0, obstacle_y=0.0, obstacle_radius=0.3)
        x = Vec([1.0, 0.0, 0.0])
        grad = cbf.grad_h(x)
        assert float(grad[0]) > 0

    def test_alpha_linear(self):
        cbf = CircularObstacleCBF(obstacle_x=0.0, obstacle_y=0.0, obstacle_radius=0.5, gamma=2.0)
        assert abs(cbf.alpha(1.0) - 2.0) < 1e-10
        assert abs(cbf.alpha(0.0)) < 1e-10


class TestVelocitySaturationCBF:
    def test_h_linear_positive_within(self):
        cbf = VelocitySaturationCBF(v_max=1.5)
        u = Vec([1.0, 0.0])
        assert cbf.h_linear(u) > 0

    def test_h_linear_negative_beyond(self):
        cbf = VelocitySaturationCBF(v_max=1.5)
        u = Vec([2.0, 0.0])
        assert cbf.h_linear(u) < 0


class TestAltitudeFloorCBF:
    def test_h_positive_above_floor(self):
        cbf = AltitudeFloorCBF(z_min=0.3)
        x = Vec([1.0, 0.0, 0.0, 0.0])
        assert abs(cbf.h(x) - 0.7) < 1e-10

    def test_h_negative_below_floor(self):
        cbf = AltitudeFloorCBF(z_min=0.3)
        x = Vec([0.1, 0.0, 0.0, 0.0])
        assert cbf.h(x) < 0

    def test_grad_h_shape(self):
        cbf = AltitudeFloorCBF()
        x = Vec([1.0, 0.0, 0.0, 0.0])
        assert len(cbf.grad_h(x)) == 4


# ─────────────────────────────────────────────────────────────────────────────
# §3  CBF-QP Safety Filter Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestCBFQPSafetyFilter:
    def _make_filter(self, obs_x=2.0, obs_y=0.0, obs_r=0.5):
        robot = DifferentialDriveModel()
        cbf = CircularObstacleCBF(obstacle_x=obs_x, obstacle_y=obs_y, obstacle_radius=obs_r)
        return CBFQPSafetyFilter(
            robot=robot,
            cbfs=[cbf],
            u_min=Vec([-0.5, -2.5]),
            u_max=Vec([1.5, 2.5]),
        )

    def test_early_exit_when_safe(self):
        sf = self._make_filter(obs_x=10.0)
        x = Vec([0.0, 0.0, 0.0])
        u_ai = Vec([0.5, 0.0])
        result = sf.solve(x, u_ai)
        assert result["was_modified"] is False
        assert result["solver"] == "early_exit_safe"

    def test_output_within_bounds(self):
        sf = self._make_filter()
        x = Vec([1.5, 0.0, 0.0])
        u_ai = Vec([2.0, 4.0])
        result = sf.solve(x, u_ai)
        u_safe = result["u_safe"]
        assert float(u_safe[0]) <= 1.5 + 1e-6
        assert float(u_safe[1]) <= 2.5 + 1e-6

    def test_result_has_expected_keys(self):
        sf = self._make_filter()
        x = Vec([0.0, 0.0, 0.0])
        u_ai = Vec([0.5, 0.0])
        result = sf.solve(x, u_ai)
        for k in {"u_safe", "perturbation", "cbf_margins", "was_modified", "solve_time_ms"}:
            assert k in result

    def test_solve_time_nonneg(self):
        sf = self._make_filter()
        result = sf.solve(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert result["solve_time_ms"] >= 0.0

    def test_cbf_margins_list(self):
        sf = self._make_filter()
        result = sf.solve(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert isinstance(result["cbf_margins"], list)

    def test_perturbation_zero_when_safe(self):
        sf = self._make_filter(obs_x=100.0)
        result = sf.solve(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert result["perturbation"] == 0.0

    def test_perturbation_nonnegative(self):
        sf = self._make_filter()
        for v in [0.0, 0.5, 1.0, 1.5]:
            result = sf.solve(Vec([1.0, 0.0, 0.0]), Vec([v, 0.0]))
            assert result["perturbation"] >= 0.0

    def test_no_cbf_filter_is_noop(self):
        robot = DifferentialDriveModel()
        sf = CBFQPSafetyFilter(robot=robot, cbfs=[], u_min=Vec([-0.5, -2.5]), u_max=Vec([1.5, 2.5]))
        result = sf.solve(Vec([0.0, 0.0, 0.0]), Vec([0.8, 0.3]))
        assert result["was_modified"] is False

    def test_accepts_lists(self):
        sf = self._make_filter(obs_x=100.0)
        result = sf.solve([0.0, 0.0, 0.0], [0.5, 0.0])
        assert result["status"] if "status" in result else True
        assert "u_safe" in result


# ─────────────────────────────────────────────────────────────────────────────
# §4  Trajectory Predictor Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestTrajectoryPredictor:
    def _make_predictor(self, obs_x=5.0):
        robot = DifferentialDriveModel()
        cbf = CircularObstacleCBF(obstacle_x=obs_x, obstacle_y=0.0, obstacle_radius=0.5)
        return TrajectoryPredictor(robot=robot, cbfs=[cbf], dt=0.05, horizon_sec=1.0)

    def test_status_success(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert result["status"] == "success"

    def test_has_trajectory(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert isinstance(result["trajectory"], list)
        assert len(result["trajectory"]) > 0

    def test_waypoints_have_state(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.3, 0.1]))
        for wp in result["trajectory"]:
            assert "state" in wp
            assert "time_sec" in wp

    def test_final_state_shape(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert len(result["final_state"]) == 3

    def test_dangerous_trajectory_detected(self):
        robot = DifferentialDriveModel()
        cbf = CircularObstacleCBF(obstacle_x=1.0, obstacle_y=0.0, obstacle_radius=0.5)
        pred = TrajectoryPredictor(robot=robot, cbfs=[cbf], dt=0.05, horizon_sec=2.0)
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([1.5, 0.0]), horizon_sec=2.0)
        assert result["violation_count"] > 0 or result["safety_certified"] is False

    def test_zero_velocity_safe(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.0, 0.0]))
        assert result["safety_certified"] is True

    def test_distance_nonneg(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        if result["distance_traveled_m"] is not None:
            assert result["distance_traveled_m"] >= 0.0

    def test_compute_time_positive(self):
        pred = self._make_predictor()
        result = pred.predict(Vec([0.0, 0.0, 0.0]), Vec([0.5, 0.0]))
        assert result["compute_time_ms"] > 0.0


# ─────────────────────────────────────────────────────────────────────────────
# §5  Lyapunov Stability Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestLyapunovStabilityAnalyzer:
    def test_at_goal_V_zero(self):
        analyzer = LyapunovStabilityAnalyzer()
        x = Vec([1.0, 2.0, 0.5])
        x_goal = Vec([1.0, 2.0, 0.5])
        dx_dt = Vec([0.0, 0.0, 0.0])
        result = analyzer.analyze(x, x_goal, dx_dt)
        assert abs(result["V_lyapunov"]) < 1e-10
        assert abs(result["norm_error"]) < 1e-10

    def test_V_positive_away_from_goal(self):
        analyzer = LyapunovStabilityAnalyzer()
        x = Vec([0.0, 0.0, 0.0])
        x_goal = Vec([1.0, 0.0, 0.0])
        dx_dt = Vec([0.5, 0.0, 0.0])
        result = analyzer.analyze(x, x_goal, dx_dt)
        assert result["V_lyapunov"] > 0

    def test_has_expected_keys(self):
        analyzer = LyapunovStabilityAnalyzer()
        result = analyzer.analyze(Vec([0.0, 0.0, 0.0]), Vec([1.0, 0.0, 0.0]), Vec([0.5, 0.0, 0.0]))
        for k in ["V_lyapunov", "dV_dt", "stability_status", "norm_error"]:
            assert k in result

    def test_toward_goal_dV_negative(self):
        analyzer = LyapunovStabilityAnalyzer()
        x = Vec([0.0, 0.0, 0.0])
        x_goal = Vec([1.0, 0.0, 0.0])
        dx_dt = Vec([0.8, 0.0, 0.0])
        result = analyzer.analyze(x, x_goal, dx_dt)
        assert result["dV_dt"] < 0


# ─────────────────────────────────────────────────────────────────────────────
# §6  MCP Tool Handler Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestMCPTools:
    def test_cbf_filter_velocity_safe(self):
        from principia.tools import handle_cbf_filter_velocity
        result = handle_cbf_filter_velocity({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "proposed_v": 0.5, "proposed_omega": 0.0, "obstacles": [],
        })
        assert result["status"] == "success"
        assert "safe_command" in result

    def test_cbf_filter_with_obstacle(self):
        from principia.tools import handle_cbf_filter_velocity
        result = handle_cbf_filter_velocity({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "proposed_v": 1.5, "proposed_omega": 0.0,
            "obstacles": [{"x": 0.8, "y": 0.0, "radius": 0.5}],
        })
        assert result["status"] == "success"

    def test_predict_trajectory(self):
        from principia.tools import handle_predict_safe_trajectory
        result = handle_predict_safe_trajectory({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "proposed_v": 0.5, "proposed_omega": 0.0, "horizon_sec": 2.0,
        })
        assert result["status"] == "success"
        assert "safety_certified" in result

    def test_lyapunov_check(self):
        from principia.tools import handle_lyapunov_stability_check
        result = handle_lyapunov_stability_check({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "goal_x": 2.0, "goal_y": 0.0, "goal_theta": 0.0,
            "current_v": 0.5, "current_omega": 0.0,
        })
        assert result["status"] == "success"
        assert "V_lyapunov" in result

    def test_quadrotor_altitude(self):
        from principia.tools import handle_cbf_quadrotor_altitude
        result = handle_cbf_quadrotor_altitude({
            "altitude_m": 1.0, "vertical_vel": 0.0, "pitch_rad": 0.0,
            "pitch_rate": 0.0, "proposed_thrust": 8.0, "proposed_torque": 0.0, "z_min": 0.3,
        })
        assert result["status"] == "success"
        assert "safe_thrust" in result

    def test_safety_report(self):
        from principia.tools import handle_get_cbf_safety_report
        result = handle_get_cbf_safety_report({
            "state_x": 5.0, "state_y": 0.0, "state_theta": 0.0,
            "obstacles": [{"x": 0.0, "y": 0.0, "radius": 0.5}],
        })
        assert result["status"] == "success"
        assert "overall_safe" in result

    def test_minimal_perturbation_proof(self):
        from principia.tools import handle_minimal_perturbation_proof
        result = handle_minimal_perturbation_proof({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "proposed_v": 0.8, "proposed_omega": 0.3, "obstacles": [],
        })
        assert result["status"] == "success"
        assert "mathematical_proof" in result
        assert "L2_norm_perturbation" in result["mathematical_proof"]

    def test_safety_report_violation_detected(self):
        from principia.tools import handle_get_cbf_safety_report
        result = handle_get_cbf_safety_report({
            "state_x": 0.1, "state_y": 0.0, "state_theta": 0.0,
            "obstacles": [{"x": 0.0, "y": 0.0, "radius": 0.5}],
        })
        assert result["overall_safe"] is False

    def test_clf_cbf_qp_solver(self):
        from principia.tools import handle_clf_cbf_qp_solver
        result = handle_clf_cbf_qp_solver({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "goal_x": 2.0, "goal_y": 0.0, "goal_theta": 0.0,
        })
        assert result["status"] == "success"
        assert "control_command" in result
        assert result["clf_cbf_certified"] is True

    def test_swarm_cbf_fleet_safety(self):
        from principia.tools import handle_swarm_cbf_fleet_safety
        result = handle_swarm_cbf_fleet_safety({
            "robots": [
                {"id": "r1", "x": 0.0, "y": 0.0},
                {"id": "r2", "x": 0.2, "y": 0.0},  # Distance 0.2 < min 0.6
            ],
            "min_distance_m": 0.6,
        })
        assert result["status"] == "success"
        assert result["overall_fleet_safe"] is False
        assert result["violation_count"] == 1

    def test_dynamic_obstacle_cbf(self):
        from principia.tools import handle_dynamic_obstacle_cbf
        result = handle_dynamic_obstacle_cbf({
            "state_x": 0.0, "state_y": 0.0, "state_theta": 0.0,
            "proposed_v": 0.8, "proposed_omega": 0.0,
            "obs_x": 2.0, "obs_y": 0.0, "obs_vx": -0.5, "obs_vy": 0.0,
        })
        assert result["status"] == "success"
        assert "safe_command" in result

    def test_get_cbf_spatial_map(self):
        from principia.tools import handle_get_cbf_spatial_map
        result = handle_get_cbf_spatial_map({
            "state_x": 0.0, "state_y": 0.0,
            "obstacles": [{"x": 1.0, "y": 0.0, "radius": 0.5}],
        })
        assert result["status"] == "success"
        assert "ascii_radar_map" in result

    def test_batch_cbf_filter(self):
        from principia.tools import handle_batch_cbf_filter
        result = handle_batch_cbf_filter({
            "requests": [
                {"id": "r1", "state_x": 0.0, "state_y": 0.0, "proposed_v": 0.5},
                {"id": "r2", "state_x": 5.0, "state_y": 5.0, "proposed_v": 0.8},
            ]
        })
        assert result["status"] == "success"
        assert result["batch_size"] == 2
        assert len(result["results"]) == 2


# ─────────────────────────────────────────────────────────────────────────────
# §7  ROS2 Bridge Tests (simulation mode)
# ─────────────────────────────────────────────────────────────────────────────

class TestROS2Bridge:
    def test_state_update_from_dict(self):
        from principia.ros2_bridge import PrincipiaROS2Bridge
        bridge = PrincipiaROS2Bridge()
        bridge.update_state_from_odom({"x": 1.0, "y": 2.0, "theta": 0.5})
        assert abs(float(bridge._current_state[0]) - 1.0) < 1e-10
        assert abs(float(bridge._current_state[1]) - 2.0) < 1e-10
        assert abs(float(bridge._current_state[2]) - 0.5) < 1e-10

    def test_filter_returns_result(self):
        from principia.ros2_bridge import PrincipiaROS2Bridge
        bridge = PrincipiaROS2Bridge()
        result = bridge.filter_velocity(0.5, 0.0)
        assert "u_safe" in result
        assert "was_modified" in result

    def test_stats_track_commands(self):
        from principia.ros2_bridge import PrincipiaROS2Bridge
        bridge = PrincipiaROS2Bridge()
        bridge.filter_velocity(0.5, 0.0)
        bridge.filter_velocity(0.3, 0.1)
        stats = bridge.get_stats()
        assert stats["total_commands"] == 2

    def test_stats_has_state(self):
        from principia.ros2_bridge import PrincipiaROS2Bridge
        bridge = PrincipiaROS2Bridge()
        bridge.update_state_from_odom({"x": 3.0, "y": 4.0, "theta": 1.0})
        stats = bridge.get_stats()
        assert abs(stats["current_state"]["x"] - 3.0) < 1e-10


# ─────────────────────────────────────────────────────────────────────────────
# §8  Server Module Tests
# ─────────────────────────────────────────────────────────────────────────────

class TestServerModule:
    def test_version(self):
        from principia.server import __version__
        assert __version__ == "1.0.0"

    def test_tool_registry_not_empty(self):
        from principia.server import TOOL_REGISTRY
        assert len(TOOL_REGISTRY) >= 6

    def test_all_tools_have_description(self):
        from principia.server import TOOL_REGISTRY
        for name, meta in TOOL_REGISTRY.items():
            assert "description" in meta
            assert len(meta["description"]) > 10

    def test_handle_initialize(self):
        from principia.server import handle_initialize
        resp = handle_initialize(1, {})
        assert resp["jsonrpc"] == "2.0"
        assert resp["id"] == 1
        assert "protocolVersion" in resp["result"]

    def test_handle_tools_list(self):
        from principia.server import handle_tools_list
        resp = handle_tools_list(2)
        assert isinstance(resp["result"]["tools"], list)
        assert len(resp["result"]["tools"]) >= 6

    def test_invalid_tool_error(self):
        from principia.server import handle_tool_call
        resp = handle_tool_call(3, {"name": "does_not_exist", "arguments": {}}, {})
        assert "error" in resp

    def test_valid_tool_call(self):
        from principia.server import _get_handlers, handle_tool_call
        handlers = _get_handlers()
        resp = handle_tool_call(4, {"name": "principia_status", "arguments": {}}, handlers)
        assert "result" in resp
        content = json.loads(resp["result"]["content"][0]["text"])
        assert content["version"] == "1.0.0"
        assert content["status"] == "running"

    def test_principia_ui_tool_handler(self):
        from principia.server import _get_handlers, handle_tool_call
        handlers = _get_handlers()
        resp = handle_tool_call(5, {"name": "principia_ui", "arguments": {"port": 8080}}, handlers)
        assert "result" in resp
        content = json.loads(resp["result"]["content"][0]["text"])
        assert content["status"] == "success"
        assert "http://localhost:8080" in content["url"]


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
