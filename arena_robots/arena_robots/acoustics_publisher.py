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
    """ROS 2 node that computes acoustic ego-noise level from joint states."""

    def __init__(self, **kwargs: object) -> None:
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
                    share_dir / "robots" / robot_name_param / "telemetry" / "acoustics.yaml",
                    share_dir / "config" / "acoustic_profile.yaml",
                ]
                for cand in candidates:
                    if cand.is_file():
                        profile_file = cand
                        break
            except Exception:
                self.get_logger().exception("Failed to resolve acoustic profile share directory")

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
        self._beta_scrub_0: float = float(cfg.get("beta_scrub_0", 40.0))
        self._beta_scrub_1: float = float(cfg.get("beta_scrub_1", 15.0))
        self._omega_ref: float = float(cfg.get("omega_ref", 5.0))
        self._tau_ref: float = float(cfg.get("tau_ref", 10.0))
        self._omega_deadband: float = float(cfg.get("omega_deadband", 0.05))
        self._omega_active: float = float(cfg.get("omega_active", 0.20))
        self._sigma_base: float = float(cfg.get("sigma_base", 1.5))
        self._sigma_dynamic: float = float(cfg.get("sigma_dynamic", 2.0))
        self._sigma_no_effort: float = float(cfg.get("sigma_no_effort", 4.0))

        # Precompute baseline acoustic power
        self._P_base: float = 10.0 ** (self._L_base_0 / 10.0)

        topic_param = str(self.declare_parameter("topic", "acoustics").value)

        # State tracking for IEC 61672-1 Fast time weighting (tau_F = 0.125s)
        self._last_time: float | None = None
        self._ema_p_drive: float = 0.0
        self._warned_empty_effort: bool = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._acoustics_pub = self.create_publisher(Acoustics, topic_param, qos)

        self.create_subscription(JointState, "joint_states", self._on_joint_state, qos)

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

        # Validity flags (bit 0: DATA_INVALID, bit 1: NO_EFFORT, bit 2: NO_VELOCITY)
        validity_flags = 0
        if not has_effort:
            validity_flags |= 2  # bit 1: no effort
        if not has_velocity:
            validity_flags |= 4  # bit 2: no velocity

        # If mandatory telemetry is missing, invalidate calculation
        if not has_effort or not has_velocity:
            validity_flags |= 1  # bit 0: DATA_INVALID
            out_msg = Acoustics()
            out_msg.header = msg.header
            out_msg.total_level_af_dba = float("nan")
            out_msg.total_level_zf_db = float("nan")
            out_msg.baseline_level_dba = float(self._L_base_0)
            out_msg.drivetrain_level_dba = float("nan")
            out_msg.uncertainty_1sigma_dba = float("nan")
            out_msg.validity_flags = int(validity_flags)
            self._acoustics_pub.publish(out_msg)
            return

        # Select wheel joints if joint names are provided
        wheel_indices = [
            i for i, name in enumerate(msg.name)
            if "wheel" in name.lower()
        ] if msg.name else []

        if not wheel_indices:
            # Fallback to all joints if no names or no joint contains 'wheel'
            wheel_indices = list(range(n_joints))

        wheel_velocities = [msg.velocity[i] for i in wheel_indices if i < len(msg.velocity)]
        wheel_efforts = [msg.effort[i] for i in wheel_indices if i < len(msg.effort)]

        if not wheel_velocities or not wheel_efforts:
            validity_flags |= 1
            out_msg = Acoustics()
            out_msg.header = msg.header
            out_msg.total_level_af_dba = float("nan")
            out_msg.total_level_zf_db = float("nan")
            out_msg.baseline_level_dba = float(self._L_base_0)
            out_msg.drivetrain_level_dba = float("nan")
            out_msg.uncertainty_1sigma_dba = float("nan")
            out_msg.validity_flags = int(validity_flags)
            self._acoustics_pub.publish(out_msg)
            return

        # 1. Equivalent wheel speed Omega_eq
        omega_eq = math.sqrt(sum(w**2 for w in wheel_velocities) / len(wheel_velocities))

        # 2. Equivalent joint effort T_eq
        t_eq = sum(abs(tau) for tau in wheel_efforts) / len(wheel_efforts)

        # 3. Activation lambda(Omega)
        delta_active = self._omega_active - self._omega_deadband
        if delta_active > 0:
            lambda_omega = max(0.0, min(1.0, (omega_eq - self._omega_deadband) / delta_active))
        else:
            lambda_omega = 1.0 if omega_eq >= self._omega_active else 0.0

        # Compute dt for EMA time weighting
        dt = 0.0
        if self._last_time is not None:
            dt = current_time - self._last_time
        self._last_time = current_time

        # 4. Drivetrain Acoustic Power P_drive (raw) - Speed & Wheel Torque Scaling
        p_drive_raw = (
            lambda_omega
            * (10.0 ** (self._beta_0 / 10.0))
            * ((max(omega_eq, self._omega_active) / self._omega_ref) ** (self._beta_1 / 10.0))
            * ((1.0 + t_eq / self._tau_ref) ** (self._beta_2 / 10.0))
        )

        # 5. Scrubbing term P_scrub (raw - uses signed arithmetic mean for left/right wheel groups)
        left_vels: list[float] = []
        right_vels: list[float] = []
        if has_velocity:
            for name, vel in zip(msg.name, msg.velocity, strict=True):
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
            omega_left = sum(left_vels) / len(left_vels)
            omega_right = sum(right_vels) / len(right_vels)
            delta_omega_lr = abs(omega_left - omega_right)
        else:
            delta_omega_lr = 0.0

        if delta_active > 0:
            lambda_scrub = max(0.0, min(1.0, (delta_omega_lr - self._omega_deadband) / delta_active))
        else:
            lambda_scrub = 1.0 if delta_omega_lr >= self._omega_active else 0.0

        p_scrub_raw = (
            lambda_scrub
            * (10.0 ** (self._beta_scrub_0 / 10.0))
            * ((max(delta_omega_lr, self._omega_active) / self._omega_ref) ** (self._beta_scrub_1 / 10.0))
        )

        # 6. IEC 61672-1 Fast Time Weighting EMA (tau_F = 125ms = 0.125s)
        TAU_FAST = 0.125
        p_dynamic_raw = p_drive_raw + p_scrub_raw
        if dt > 0.0:
            alpha = 1.0 - math.exp(-dt / TAU_FAST)
            self._ema_p_drive = (1.0 - alpha) * self._ema_p_drive + alpha * p_dynamic_raw
        else:
            self._ema_p_drive = p_dynamic_raw

        p_drive = self._ema_p_drive

        # 6. Total Power and Sound Levels
        p_total = self._P_base + p_drive
        l_1m = 10.0 * math.log10(p_total) if p_total > 0.0 else self._L_base_0

        # Baseline & Drivetrain levels in dBA
        l_base = self._L_base_0
        l_drivetrain = 10.0 * math.log10(p_drive) if p_drive > 1e-12 else 0.0

        # 7. Uncertainty 1-sigma (heteroscedastic)
        sigma_total = math.sqrt(
            self._sigma_base**2 + (self._sigma_dynamic * omega_eq / self._omega_ref) ** 2
        )

        # 8. Publish message
        out_msg = Acoustics()
        out_msg.header = msg.header
        out_msg.total_level_af_dba = float(l_1m)
        out_msg.total_level_zf_db = float("nan")  # Broadband proxy only supports A-weighted dBA
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
