"""Power publisher node — ecological energy metrics for robot navigation.

Subscribes to ``sensor_msgs/msg/JointState``, applies a thermo-mechanical
power model to **all** published joints, integrates energy over time, and
publishes granular ``Power`` / ``Energy`` messages on every callback.

Configuration is loaded from a hierarchical YAML whose path is supplied via
the ``config_path`` ROS parameter.  Only sensors whose keys appear in the
``active_sensors`` string-array parameter contribute to static power.
"""

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
    tracks battery State of Charge from ``/joint_states``."""

    def __init__(self) -> None:
        super().__init__("power_publisher")

        config_path = self.declare_parameter("config_path", "").value
        if not config_path:
            self.get_logger().fatal("'config_path' parameter is required")
            raise SystemExit(1)
        active_sensors_param: list[str] = list(
            self.declare_parameter("active_sensors", [""]).value
        )
        active_sensors_param = [s for s in active_sensors_param if s]

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

        joint_metrics = cfg["joint_metrics"]
        joint_topic: str = str(joint_metrics["topic"])

        static_cfg = cfg["static_power_w"]
        self._compute_core_w: float = float(static_cfg["compute_core"])
        self._idle_motors_w: float = float(static_cfg["idle_motors"])
        self._sensor_power_map: dict[str, float] = {
            str(k): float(v) for k, v in static_cfg.get("sensors", {}).items()
        }

        if active_sensors_param:
            active_sensors = active_sensors_param
            unknown = set(active_sensors) - set(self._sensor_power_map)
            if unknown:
                self.get_logger().warning(
                    f"active_sensors keys not found in config: {sorted(unknown)}"
                )
        else:
            active_sensors = list(self._sensor_power_map.keys())

        # Compute static power once 
        sensor_power_sum = sum(
            self._sensor_power_map[s]
            for s in active_sensors
            if s in self._sensor_power_map
        )
        self._static_power_w: float = (
            self._compute_core_w + self._idle_motors_w + sensor_power_sum
        )

        # State for energy integration 
        self._last_time: float | None = None
        self._total_energy_consumed_j: float = 0.0
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
            f"active_sensors={active_sensors}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        n_joints = len(msg.name)
        has_effort = len(msg.effort) > 0
        has_velocity = len(msg.velocity) > 0

        if not has_effort and not self._warned_empty_effort:
            self.get_logger().warning(
                "JointState has no effort data — mechanical & thermal power "
                "will be zero until effort is published."
            )
            self._warned_empty_effort = True

        joint_names: list[str] = []
        joint_mech: list[float] = []
        joint_therm: list[float] = []
        joint_total: list[float] = []

        for i in range(n_joints):
            name = msg.name[i]
            effort = msg.effort[i] if i < len(msg.effort) else 0.0
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0

            p_mech = abs(effort * velocity) / self._efficiency
            p_therm = self._heating_coeff * effort * effort

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
                self._total_energy_consumed_j += total_power * dt
        self._last_time = current_time

        energy_wh = self._total_energy_consumed_j / 3600.0
        if self._battery_capacity_wh > 0.0:
            soc = (1.0 - energy_wh / self._battery_capacity_wh) * 100.0
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
        energy_msg.total_energy_consumed_j = self._total_energy_consumed_j
        energy_msg.total_energy_consumed_wh = energy_wh
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
