"""Power publisher node"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from arena_rclpy_mixins.spin import spin_node
from arena_robots_msgs.msg import Energy, Power
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class PowerPublisher(Node):
    """ROS 2 node that computes instantaneous power, integrates energy, and
    tracks battery State of Charge"""

    def __init__(self, **kwargs) -> None:
        super().__init__("power_publisher", **kwargs)

        config_path = self.declare_parameter("config_path", "").value
        if not config_path:
            self.get_logger().fatal("'config_path' parameter is required")
            raise SystemExit(1)

        config_file = Path(config_path)
        if not config_file.is_file():
            self.get_logger().fatal(f"Config file not found: {config_file}")
            raise SystemExit(1)

        with open(config_file) as f:
            cfg = yaml.safe_load(f)

        power_system = cfg["power_system"]
        self._efficiency: float = float(power_system["global_drivetrain_efficiency"])
        self._heating_coeff: float = float(power_system["heating_coefficient_ch"])
        self._battery_capacity_wh: float = float(power_system["battery_capacity_wh"])

        # Physical drivetrain & rolling resistance parameters (with backward compatible defaults)
        self._drivetrain_damping: float = float(power_system.get("drivetrain_damping", 0.0))
        self._drivetrain_friction: float = float(power_system.get("drivetrain_friction", 0.0))
        self._rolling_resistance_crr: float = float(power_system.get("rolling_resistance_crr", 0.0))
        self._robot_mass_kg: float = float(power_system.get("robot_mass_kg", 0.0))
        self._wheel_radius_m: float = float(power_system.get("wheel_radius_m", 0.0))
        self._num_wheels: int = int(power_system.get("num_wheels", 1))

        if self._num_wheels > 0 and self._robot_mass_kg > 0.0 and self._wheel_radius_m > 0.0:
            self._tau_roll_per_wheel: float = (
                self._rolling_resistance_crr * self._robot_mass_kg * 9.81 * self._wheel_radius_m
            ) / float(self._num_wheels)
        else:
            self._tau_roll_per_wheel: float = 0.0

        joint_metrics = cfg["joint_metrics"]
        joint_topic: str = str(joint_metrics["topic"])

        static_cfg = cfg["static_power_w"]
        self._compute_core_w: float = float(static_cfg["compute_core"])
        self._idle_motors_w: float = float(static_cfg["idle_motors"])

        components_static_power = float(self.declare_parameter("components_static_power_w", 0.0).value)

        # Compute static power once
        self._static_power_w: float = (
            self._compute_core_w + self._idle_motors_w + components_static_power
        )

        # State for energy integration
        self._last_time: float | None = None
        self._total_energy_consumed_wh: float = 0.0
        self._warned_empty_effort: bool = False

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._power_pub = self.create_publisher(Power, "~/power", qos)
        self._energy_pub = self.create_publisher(Energy, "~/energy", qos)

        # Subscription
        self.create_subscription(JointState, joint_topic, self._on_joint_state, qos)

        self.get_logger().info(
            f"PowerPublisher ready — static={self._static_power_w:.1f} W, "
            f"η={self._efficiency}, c_h={self._heating_coeff}, "
            f"battery={self._battery_capacity_wh} Wh, "
            f"drivetrain_losses(b={self._drivetrain_damping}, tau_c={self._drivetrain_friction}, tau_roll={self._tau_roll_per_wheel:.3f} Nm)"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        n_joints = len(msg.name)
        has_effort = len(msg.effort) > 0
        has_velocity = len(msg.velocity) > 0

        if not has_effort and not self._warned_empty_effort:
            self.get_logger().warning(
                "JointState has no effort data. Mechanical and thermal power cannot be calculated and will be invalidated (NaN)."
            )
            self._warned_empty_effort = True

        if not has_effort or not has_velocity:
            # Strict Invalidation: Publish NaN to prevent logging false 0W dynamic energy
            power_msg = Power()
            power_msg.header = msg.header
            power_msg.total_power_w = float("nan")
            power_msg.static_power_w = float(self._static_power_w)
            power_msg.total_mechanical_power_w = float("nan")
            power_msg.total_thermal_power_w = float("nan")
            power_msg.joint_names = list(msg.name)
            power_msg.joint_mechanical_power_w = [float("nan")] * n_joints
            power_msg.joint_thermal_power_w = [float("nan")] * n_joints
            power_msg.joint_total_power_w = [float("nan")] * n_joints
            self._power_pub.publish(power_msg)

            energy_msg = Energy()
            energy_msg.header = msg.header
            energy_msg.total_energy_consumed_wh = float("nan")
            energy_msg.battery_soc_percent = float("nan")
            self._energy_pub.publish(energy_msg)
            return

        joint_names: list[str] = []
        joint_mech: list[float] = []
        joint_therm: list[float] = []
        joint_total: list[float] = []

        for i in range(n_joints):
            name = msg.name[i]
            effort = abs(msg.effort[i]) if i < len(msg.effort) else 0.0
            velocity = abs(msg.velocity[i]) if i < len(msg.velocity) else 0.0

            if velocity > 1e-4:
                tau_parasitic = (
                    self._drivetrain_friction
                    + self._drivetrain_damping * velocity
                    + self._tau_roll_per_wheel
                )
            else:
                tau_parasitic = 0.0

            total_effort = effort + tau_parasitic

            p_mech = (total_effort * velocity) / self._efficiency
            p_therm = self._heating_coeff * total_effort * total_effort

            joint_names.append(name)
            joint_mech.append(p_mech)
            joint_therm.append(p_therm)
            joint_total.append(p_mech + p_therm)

        total_mech = math.fsum(joint_mech)
        total_therm = math.fsum(joint_therm)
        total_power = self._static_power_w + total_mech + total_therm

        # Energy integration
        if self._last_time is not None:
            dt = current_time - self._last_time
            if dt > 0.0:
                self._total_energy_consumed_wh += (total_power * dt) / 3600.0
        self._last_time = current_time

        if self._battery_capacity_wh > 0.0:
            soc = (1.0 - self._total_energy_consumed_wh / self._battery_capacity_wh) * 100.0
        else:
            soc = 0.0

        # Publish Power
        power_msg = Power()
        power_msg.header = msg.header
        power_msg.total_power_w = total_power
        power_msg.static_power_w = self._static_power_w
        power_msg.total_mechanical_power_w = total_mech
        power_msg.total_thermal_power_w = total_therm
        power_msg.joint_names = joint_names
        power_msg.joint_mechanical_power_w = joint_mech
        power_msg.joint_thermal_power_w = joint_therm
        power_msg.joint_total_power_w = joint_total
        self._power_pub.publish(power_msg)

        # Publish Energy
        energy_msg = Energy()
        energy_msg.header = msg.header
        energy_msg.total_energy_consumed_wh = self._total_energy_consumed_wh
        energy_msg.battery_soc_percent = soc
        self._energy_pub.publish(energy_msg)


def main() -> None:
    rclpy.init()
    node = PowerPublisher()
    try:
        spin_node(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
