"""Power publisher node"""

from __future__ import annotations

import math

import rclpy
from arena_rclpy_mixins.spin import spin_node
from arena_robots_msgs.msg import Energy, Power
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class PowerPublisher(Node):
    """Instantaneous power and integrated energy from joint effort and velocity."""

    def __init__(self) -> None:
        super().__init__("power_publisher")

        self._static_power_w: float = float(self.declare_parameter("static_power_w", 0.0).value)
        self._efficiency: float = float(self.declare_parameter("drivetrain_efficiency", 1.0).value)
        self._heating_coeff: float = float(self.declare_parameter("heating_coefficient_ch", 0.0).value)
        self._battery_capacity_wh: float = float(self.declare_parameter("battery_capacity_wh", 0.0).value)

        self._last_time: float | None = None
        self._total_energy_consumed_wh: float = 0.0
        self._warned_empty_effort: bool = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._power_pub = self.create_publisher(Power, "~/power", qos)
        self._energy_pub = self.create_publisher(Energy, "~/energy", qos)

        self.create_subscription(JointState, "joint_states", self._on_joint_state, qos)

        self.get_logger().info(f"PowerPublisher ready: static={self._static_power_w:.1f} W, efficiency={self._efficiency}, c_h={self._heating_coeff}, battery={self._battery_capacity_wh} Wh")

    def _on_joint_state(self, msg: JointState) -> None:
        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        n_joints = len(msg.name)
        has_effort = len(msg.effort) > 0
        has_velocity = len(msg.velocity) > 0

        if not has_effort and not self._warned_empty_effort:
            self.get_logger().warning("JointState carries no effort, mechanical and thermal power stay zero until it is published.")
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

        if self._last_time is not None:
            dt = current_time - self._last_time
            if dt > 0.0:
                self._total_energy_consumed_wh += (total_power * dt) / 3600.0
        self._last_time = current_time

        if self._battery_capacity_wh > 0.0:
            soc = max(0.0, (1.0 - self._total_energy_consumed_wh / self._battery_capacity_wh) * 100.0)
        else:
            soc = 0.0

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
