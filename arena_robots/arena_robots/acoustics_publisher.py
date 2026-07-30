"""Acoustics publisher node implementing M4 Ego-Noise Model"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from arena_rclpy_mixins.spin import spin_node
from arena_robots_msgs.msg import Acoustics
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class AcousticsPublisher(Node):
    """ROS 2 node that computes M4 acoustic ego-noise level from joint states."""

    def __init__(self, **kwargs) -> None:
        super().__init__("acoustics_publisher", **kwargs)

        robot_name_param = str(self.declare_parameter("robot_name", "jackal").value)
        profile_path_param = str(self.declare_parameter("profile_path", "").value)

        profile_file: Path | None = None
        if profile_path_param:
            p = Path(profile_path_param)
            if p.is_file():
                profile_file = p

        if profile_file is None:
            try:
                from ament_index_python.packages import get_package_share_directory

                share_dir = Path(get_package_share_directory("arena_robots"))
                candidates = [
                    share_dir / "robots" / robot_name_param / "acoustic_profile.yaml",
                    share_dir / "config" / "acoustic_profile.yaml",
                    Path(r"u:\src\Arena\arena_robots\arena_robots\robots") / robot_name_param / "acoustic_profile.yaml",
                ]
                for cand in candidates:
                    if cand.is_file():
                        profile_file = cand
                        break
            except Exception:
                pass

        if profile_file is None or not profile_file.is_file():
            self.get_logger().fatal(
                f"Acoustic profile file not found for robot '{robot_name_param}' (path: {profile_path_param})"
            )
            raise SystemExit(1)

        with open(profile_file) as f:
            cfg = yaml.safe_load(f)

        self._L_base_0: float = float(cfg.get("L_base_0", 42.0))
        self._beta_0: float = float(cfg.get("beta_0", 45.0))
        self._beta_1: float = float(cfg.get("beta_1", 18.0))
        self._beta_2: float = float(cfg.get("beta_2", 5.0))
        self._beta_3: float = float(cfg.get("beta_3", 3.0))
        self._beta_scrub_0: float = float(cfg.get("beta_scrub_0", 40.0))
        self._beta_scrub_1: float = float(cfg.get("beta_scrub_1", 12.0))
        self._omega_ref: float = float(cfg.get("omega_ref", 5.0))
        self._tau_ref: float = float(cfg.get("tau_ref", 10.0))
        self._omega_deadband: float = float(cfg.get("omega_deadband", 0.05))
        self._omega_active: float = float(cfg.get("omega_active", 0.20))
        self._sigma_base: float = float(cfg.get("sigma_base", 1.5))
        self._sigma_dynamic: float = float(cfg.get("sigma_dynamic", 2.0))
        self._sigma_no_effort: float = float(cfg.get("sigma_no_effort", 4.0))

        # Precompute baseline acoustic power
        self._P_base: float = 10.0 ** (self._L_base_0 / 10.0)

        topic_param = str(self.declare_parameter("topic", "/acoustics").value)

        # State tracking for acceleration
        self._last_time: float | None = None
        self._last_omega_eq: float | None = None
        self._warned_empty_effort: bool = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._acoustics_pub = self.create_publisher(Acoustics, topic_param, qos)

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, qos)

        self.get_logger().info(
            f"AcousticsPublisher ready — profile={profile_file}, topic={topic_param}, L_base_0={self._L_base_0} dBA"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        n_joints = len(msg.velocity)
        has_velocity = n_joints > 0
        has_effort = len(msg.effort) > 0

        if not has_effort and not self._warned_empty_effort:
            self.get_logger().warning(
                "JointState has no effort data. Acoustic model will set T_eq=0 and apply uncertainty penalty."
            )
            self._warned_empty_effort = True

        # 1. Equivalent wheel speed Omega_eq
        if has_velocity:
            omega_eq = math.sqrt(sum(w**2 for w in msg.velocity) / n_joints)
        else:
            omega_eq = 0.0

        # 2. Equivalent joint effort T_eq
        if has_effort and len(msg.effort) > 0:
            t_eq = sum(abs(tau) for tau in msg.effort) / len(msg.effort)
        else:
            t_eq = 0.0

        # 3. Activation lambda(Omega)
        delta_active = self._omega_active - self._omega_deadband
        if delta_active > 0:
            lambda_omega = max(0.0, min(1.0, (omega_eq - self._omega_deadband) / delta_active))
        else:
            lambda_omega = 1.0 if omega_eq >= self._omega_active else 0.0

        # 4. Acceleration a_eq
        if self._last_time is not None and self._last_omega_eq is not None:
            dt = current_time - self._last_time
            dt_safe = max(dt, 0.001)
            delta_omega_eq = omega_eq - self._last_omega_eq
            a_eq = min(abs(delta_omega_eq / dt_safe), 50.0)
        else:
            a_eq = 0.0

        self._last_time = current_time
        self._last_omega_eq = omega_eq

        # 5. Drivetrain Acoustic Power P_drive
        p_drive = (
            lambda_omega
            * (10.0 ** (self._beta_0 / 10.0))
            * ((max(omega_eq, self._omega_active) / self._omega_ref) ** (self._beta_1 / 10.0))
            * ((1.0 + t_eq / self._tau_ref) ** (self._beta_2 / 10.0))
            * (10.0 ** (self._beta_3 * a_eq / 10.0))
        )

        # 6. Scrubbing term P_scrub
        left_vels: list[float] = []
        right_vels: list[float] = []
        if has_velocity:
            for name, vel in zip(msg.name, msg.velocity):
                n_lower = name.lower()
                is_left = (
                    "left" in n_lower
                    or n_lower.startswith("l_")
                    or "_l_" in n_lower
                    or n_lower.endswith("_l")
                    or "fl" in n_lower
                    or "rl" in n_lower
                )
                is_right = (
                    "right" in n_lower
                    or n_lower.startswith("r_")
                    or "_r_" in n_lower
                    or n_lower.endswith("_r")
                    or "fr" in n_lower
                    or "rr" in n_lower
                )
                if is_left and not is_right:
                    left_vels.append(vel)
                elif is_right and not is_left:
                    right_vels.append(vel)

            if not left_vels and not right_vels and n_joints >= 2:
                half = n_joints // 2
                left_vels = list(msg.velocity[:half])
                right_vels = list(msg.velocity[half:])

        if left_vels and right_vels:
            omega_left = math.sqrt(sum(w**2 for w in left_vels) / len(left_vels))
            omega_right = math.sqrt(sum(w**2 for w in right_vels) / len(right_vels))
            delta_omega_lr = abs(omega_left - omega_right)
        else:
            delta_omega_lr = 0.0

        if delta_active > 0:
            lambda_scrub = max(0.0, min(1.0, (delta_omega_lr - self._omega_deadband) / delta_active))
        else:
            lambda_scrub = 1.0 if delta_omega_lr >= self._omega_active else 0.0

        p_scrub = (
            lambda_scrub
            * (10.0 ** (self._beta_scrub_0 / 10.0))
            * ((max(delta_omega_lr, self._omega_active) / self._omega_ref) ** (self._beta_scrub_1 / 10.0))
        )

        # 7. Total Power and Sound Levels
        p_total = self._P_base + p_drive + p_scrub
        l_1m = 10.0 * math.log10(p_total) if p_total > 0.0 else 0.0

        # Baseline & Drivetrain levels in dBA
        l_base = self._L_base_0
        p_dynamic = p_drive + p_scrub
        l_drivetrain = 10.0 * math.log10(p_dynamic) if p_dynamic > 1e-12 else 0.0

        # 8. Uncertainty 1-sigma
        effort_unc = 0.0 if has_effort else (self._sigma_no_effort**2)
        sigma_total = math.sqrt(
            self._sigma_base**2 + (self._sigma_dynamic * omega_eq / self._omega_ref) ** 2 + effort_unc
        )

        # 9. Validity flags
        validity_flags = 0
        if not has_effort:
            validity_flags |= 2  # bit 1: no effort
        if not has_velocity:
            validity_flags |= 4  # bit 2: no velocity

        # 10. Publish message
        out_msg = Acoustics()
        out_msg.header = msg.header
        out_msg.total_level_aeq_dba = float(l_1m)
        out_msg.total_level_zeq_db = float(l_1m)
        out_msg.baseline_level_dba = float(l_base)
        out_msg.drivetrain_level_dba = float(l_drivetrain)
        out_msg.uncertainty_1sigma_dba = float(sigma_total)
        out_msg.validity_flags = int(validity_flags)

        self._acoustics_pub.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = AcousticsPublisher()
    try:
        spin_node(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
