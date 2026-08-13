"""Acoustics publisher node implementing M4 Ego-Noise Model"""

from __future__ import annotations

import math
import re
from pathlib import Path

import rclpy
import yaml
from arena_rclpy_mixins.spin import spin_node
from arena_robots_msgs.msg import Acoustics
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

_TOKEN_SPLIT = re.compile(r"[_\-]+")
_LEFT_TOKENS = frozenset({"left", "l", "fl", "rl", "lf", "lh"})
_RIGHT_TOKENS = frozenset({"right", "r", "fr", "rr", "rf", "rh"})


def _wheel_side(name: str) -> str:
    """Side of a wheel/leg joint by whole-token match. Empty when unknown or ambiguous."""
    tokens = {t for t in _TOKEN_SPLIT.split(name.lower()) if t}
    is_left = bool(tokens & _LEFT_TOKENS)
    is_right = bool(tokens & _RIGHT_TOKENS)
    if is_left == is_right:
        return ""
    return "left" if is_left else "right"


class AcousticsPublisher(Node):
    """ROS 2 node that computes acoustic ego-noise level from joint states."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__("acoustics_publisher", **kwargs)

        robot_name_param = str(self.declare_parameter("robot_name", "").value)
        profile_path_param = str(self.declare_parameter("profile_path", "").value)

        profile_file = Path(profile_path_param)
        if not profile_path_param or not profile_file.is_file():
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

        # The profile is a fitted parametric model, not a calibration against a sound level meter
        self._calibration_status: str = (
            f"uncalibrated_parametric_model:{robot_name_param or profile_file.stem}"
        )

        topic_param = str(self.declare_parameter("topic", "acoustics").value)

        # State tracking for acceleration and IEC 61672-1 Fast time weighting (tau_F = 0.125s)
        self._last_time: float | None = None
        self._last_omega_eq: float | None = None
        self._ema_p_drive: float = 0.0
        self._ema_p_scrub: float = 0.0
        self._warned_empty_effort: bool = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._acoustics_pub = self.create_publisher(Acoustics, topic_param, qos)

        self.create_subscription(JointState, "joint_states", self._on_joint_state, qos)

        self.get_logger().info(
            f"AcousticsPublisher ready: profile={profile_file}, topic={topic_param}, L_base_0={self._L_base_0} dBA"
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

        # Equivalent wheel speed Omega_eq
        if has_velocity:
            omega_eq = math.sqrt(sum(w**2 for w in msg.velocity) / n_joints)
        else:
            omega_eq = 0.0

        # Equivalent joint effort T_eq
        if has_effort and len(msg.effort) > 0:
            t_eq = sum(abs(tau) for tau in msg.effort) / len(msg.effort)
        else:
            t_eq = 0.0

        # Activation lambda(Omega)
        delta_active = self._omega_active - self._omega_deadband
        if delta_active > 0:
            lambda_omega = max(0.0, min(1.0, (omega_eq - self._omega_deadband) / delta_active))
        else:
            lambda_omega = 1.0 if omega_eq >= self._omega_active else 0.0

        # Acceleration a_eq (clamped to dt_min=0.01s and a_max=10.0 rad/s^2)
        dt = 0.0
        if self._last_time is not None and self._last_omega_eq is not None:
            dt = current_time - self._last_time
            dt_safe = max(dt, 0.01)
            delta_omega_eq = omega_eq - self._last_omega_eq
            a_eq = min(abs(delta_omega_eq / dt_safe), 10.0)
        else:
            a_eq = 0.0

        self._last_time = current_time
        self._last_omega_eq = omega_eq

        # Drivetrain acoustic power P_drive (raw)
        p_drive_raw = (
            lambda_omega
            * (10.0 ** (self._beta_0 / 10.0))
            * ((max(omega_eq, self._omega_active) / self._omega_ref) ** (self._beta_1 / 10.0))
            * ((1.0 + t_eq / self._tau_ref) ** (self._beta_2 / 10.0))
            * (10.0 ** (self._beta_3 * a_eq / 10.0))
        )

        # Scrubbing term P_scrub (raw - uses signed arithmetic mean for left/right wheel groups)
        left_vels: list[float] = []
        right_vels: list[float] = []
        sides_from_names = False
        if has_velocity:
            for i, name in enumerate(msg.name):
                if i >= n_joints:
                    break
                side = _wheel_side(name)
                if side == "left":
                    left_vels.append(msg.velocity[i])
                elif side == "right":
                    right_vels.append(msg.velocity[i])

            sides_from_names = bool(left_vels and right_vels)
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

        # IEC 61672-1 Fast time weighting EMA (tau_F = 125ms = 0.125s)
        TAU_FAST = 0.125
        if dt > 0.0:
            alpha = 1.0 - math.exp(-dt / TAU_FAST)
            self._ema_p_drive = (1.0 - alpha) * self._ema_p_drive + alpha * p_drive_raw
            self._ema_p_scrub = (1.0 - alpha) * self._ema_p_scrub + alpha * p_scrub_raw
        else:
            self._ema_p_drive = p_drive_raw
            self._ema_p_scrub = p_scrub_raw

        p_drive = self._ema_p_drive
        p_scrub = self._ema_p_scrub

        # Total power and sound levels
        p_total = self._P_base + p_drive + p_scrub
        l_1m = 10.0 * math.log10(p_total) if p_total > 0.0 else 0.0

        # Baseline and drivetrain levels in dBA
        l_base = self._L_base_0
        p_dynamic = p_drive + p_scrub
        l_drivetrain = 10.0 * math.log10(p_dynamic) if p_dynamic > 1e-12 else 0.0

        # Uncertainty 1-sigma
        effort_unc = 0.0 if has_effort else (self._sigma_no_effort**2)
        sigma_total = math.sqrt(
            self._sigma_base**2 + (self._sigma_dynamic * omega_eq / self._omega_ref) ** 2 + effort_unc
        )

        # Validity flags
        validity_flags = 0
        if has_velocity and not sides_from_names:
            validity_flags |= Acoustics.FLAG_SCRUB_UNCLASSIFIED
        if not has_effort:
            validity_flags |= Acoustics.FLAG_NO_EFFORT
        if not has_velocity:
            validity_flags |= Acoustics.FLAG_NO_VELOCITY

        if lambda_scrub > 0.0:
            operating_state = "scrubbing"
        elif lambda_omega > 0.0:
            operating_state = "driving"
        else:
            operating_state = "idle"

        # Publish message
        out_msg = Acoustics()
        out_msg.header = msg.header
        out_msg.total_level_af_dba = float(l_1m)
        out_msg.total_level_zf_db = float("nan")  # Broadband proxy only supports A-weighted dBA
        out_msg.baseline_level_dba = float(l_base)
        out_msg.drivetrain_level_dba = float(l_drivetrain)
        out_msg.uncertainty_1sigma_dba = float(sigma_total)
        out_msg.validity_flags = int(validity_flags)
        out_msg.operating_state = operating_state
        out_msg.calibration_status = self._calibration_status

        self._acoustics_pub.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = AcousticsPublisher()
    try:
        spin_node(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
