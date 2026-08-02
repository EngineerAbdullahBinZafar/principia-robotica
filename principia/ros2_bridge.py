"""
Principia Robotica — ROS2 Integration Layer

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

Bridges the CBF-QP safety engine with live ROS2 topics.
When rclpy is available:
  - Subscribes to /odom for live robot state
  - Publishes CBF-filtered commands to /cmd_vel
  - Publishes safety metrics to /principia/cbf_status

When rclpy is NOT available (simulation mode):
  - Runs with mock state injection
  - All math still works identically
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

from .cbf_engine import (
    CBFQPSafetyFilter,
    CircularObstacleCBF,
    DifferentialDriveModel,
    VelocitySaturationCBF,
)
from .linalg import Vec

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


@dataclass
class PrincipiaROS2Bridge:
    """
    Real-time ROS2 CBF Safety Bridge.

    Intercepts /cmd_vel_raw from AI agent, applies CBF-QP safety filter,
    and republishes safe commands to /cmd_vel hardware controller.

    Architecture:
        AI Agent → cmd_vel_raw → [CBF-QP Filter] → cmd_vel → Hardware
                                       ↑
                              /odom (live state feedback)
    """

    obstacles: list = field(default_factory=list)
    cbf_gamma: float = 1.0
    robot_radius: float = 0.25
    _state_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _current_state: Vec = field(
        default_factory=lambda: Vec([0.0, 0.0, 0.0]), init=False, repr=False
    )
    _commands_filtered: int = field(default=0, init=False, repr=False)
    _commands_total: int = field(default=0, init=False, repr=False)
    _node: object = field(default=None, init=False, repr=False)

    def _build_filter(self) -> CBFQPSafetyFilter:
        robot = DifferentialDriveModel()
        cbfs = []
        for obs in self.obstacles:
            cbfs.append(CircularObstacleCBF(
                obstacle_x=float(obs["x"]),
                obstacle_y=float(obs["y"]),
                obstacle_radius=float(obs["radius"]),
                robot_radius=self.robot_radius,
                gamma=self.cbf_gamma,
            ))
        cbfs.append(VelocitySaturationCBF())
        return CBFQPSafetyFilter(
            robot=robot,
            cbfs=cbfs,
            u_min=Vec([robot.v_min, -robot.omega_max]),
            u_max=Vec([robot.v_max, robot.omega_max]),
        )

    def update_state_from_odom(self, odom_data: dict):
        """Update state from odometry. Works with both ROS2 msg and dict."""
        with self._state_lock:
            if isinstance(odom_data, dict):
                self._current_state = Vec([
                    float(odom_data.get("x", 0.0)),
                    float(odom_data.get("y", 0.0)),
                    float(odom_data.get("theta", 0.0)),
                ])
            elif ROS2_AVAILABLE:
                pos = odom_data.pose.pose.position
                q = odom_data.pose.pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                self._current_state = Vec([pos.x, pos.y, yaw])

    def filter_velocity(self, v_proposed: float, omega_proposed: float) -> dict:
        """
        Apply CBF-QP filter to proposed velocity command.

        This is the core real-time safety function called at control rate.
        """
        with self._state_lock:
            x = self._current_state.copy()

        u_ai = Vec([v_proposed, omega_proposed])
        safety_filter = self._build_filter()
        result = safety_filter.solve(x, u_ai)

        self._commands_total += 1
        if result["was_modified"]:
            self._commands_filtered += 1

        return result

    def get_stats(self) -> dict:
        """Return safety filter statistics."""
        rate = (
            round(100.0 * self._commands_filtered / self._commands_total, 1)
            if self._commands_total > 0
            else 0.0
        )
        with self._state_lock:
            state = self._current_state
        return {
            "total_commands": self._commands_total,
            "filtered_commands": self._commands_filtered,
            "filter_rate_percent": rate,
            "current_state": {
                "x": round(float(state[0]), 3),
                "y": round(float(state[1]), 3),
                "theta_deg": round(math.degrees(float(state[2])), 1),
            },
        }

    def start_ros2_node(self, node_name: str = "principia_safety_bridge"):
        """Launch ROS2 node (requires rclpy)."""
        if not ROS2_AVAILABLE:
            raise RuntimeError("rclpy not available. Install ROS2 and source setup.bash.")

        rclpy.init()
        self._node = rclpy.create_node(node_name)

        # Subscriber: /cmd_vel_raw (from AI agent)
        # Publisher:  /cmd_vel     (to hardware)
        publisher = self._node.create_publisher(Twist, "/cmd_vel", 10)

        def cmd_raw_callback(msg):
            result = self.filter_velocity(msg.linear.x, msg.angular.z)
            u_safe = result["u_safe"]
            safe_msg = Twist()
            safe_msg.linear.x = float(u_safe[0])
            safe_msg.angular.z = float(u_safe[1])
            publisher.publish(safe_msg)
            if result["was_modified"]:
                self._node.get_logger().warn(
                    f"[CBF] Command modified: perturbation={result['perturbation']:.4f}"
                )

        self._node.create_subscription(Twist, "/cmd_vel_raw", cmd_raw_callback, 10)

        # Subscriber: /odom
        def odom_callback(msg):
            self.update_state_from_odom(msg)

        self._node.create_subscription(Odometry, "/odom", odom_callback, 10)

        self._node.get_logger().info(
            "Principia Safety Bridge started — filtering /cmd_vel_raw → /cmd_vel"
        )

        try:
            rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            rclpy.shutdown()


def main():
    """1-Line CLI entry point for ROS2 safety bridge."""
    bridge = PrincipiaROS2Bridge()
    try:
        bridge.start_ros2_node()
    except Exception as exc:
        print(f"[Principia ROS2 Bridge] {exc}")


if __name__ == "__main__":
    main()

