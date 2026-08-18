"""Tests for PowerPublisher ROS 2 Node."""

from __future__ import annotations

import math
from pathlib import Path

from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


def test_power_publisher_initialization(tmp_path: Path) -> None:
    from arena_robots.power_publisher import PowerPublisher

    config_content = """
power_system:
  system_voltage_v: 24.0
  global_drivetrain_efficiency: 0.70
  heating_coefficient_ch: 1.25
  battery_capacity_wh: 270.0

joint_metrics:
  topic: "joint_states"

static_power_w:
  compute_core: 30.0
  idle_motors: 5.0
"""
    config_path = tmp_path / "power.yaml"
    config_path.write_text(config_content)

    node = PowerPublisher(
        parameter_overrides=[
            Parameter("config_path", value=str(config_path)),
            Parameter("components_static_power_w", value=8.0),
        ]
    )

    try:
        assert node._efficiency == 0.70
        assert node._heating_coeff == 1.25
        assert node._battery_capacity_wh == 270.0
        assert node._static_power_w == 30.0 + 5.0 + 8.0
    finally:
        node.destroy_node()


def test_power_publisher_on_valid_joint_state(tmp_path: Path) -> None:
    from arena_robots.power_publisher import PowerPublisher

    config_content = """
power_system:
  system_voltage_v: 24.0
  global_drivetrain_efficiency: 0.80
  heating_coefficient_ch: 2.0
  battery_capacity_wh: 100.0

joint_metrics:
  topic: "joint_states"

static_power_w:
  compute_core: 20.0
  idle_motors: 5.0
"""
    config_path = tmp_path / "power.yaml"
    config_path.write_text(config_content)

    node = PowerPublisher(
        parameter_overrides=[
            Parameter("config_path", value=str(config_path)),
        ]
    )

    try:
        msg = JointState()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.name = ["left_wheel_joint", "right_wheel_joint"]
        msg.velocity = [2.0, 2.0]
        msg.effort = [3.0, 3.0]

        # First callback sets _last_time
        node._on_joint_state(msg)

        # Expected instantaneous:
        # P_mech = 2 * (3.0 * 2.0 / 0.80) = 2 * 7.5 = 15.0 W
        # P_heat = 2 * (2.0 * 3.0^2) = 2 * 18.0 = 36.0 W
        # P_static = 25.0 W
        # P_total = 25.0 + 15.0 + 36.0 = 76.0 W

        # Second callback after 1.0 second
        msg2 = JointState()
        msg2.header.stamp.sec = 11
        msg2.header.stamp.nanosec = 0
        msg2.name = ["left_wheel_joint", "right_wheel_joint"]
        msg2.velocity = [2.0, 2.0]
        msg2.effort = [3.0, 3.0]
        node._on_joint_state(msg2)

        # dt = 1.0 s -> E += 76.0 * 1.0 / 3600 Wh
        expected_energy = 76.0 / 3600.0
        assert abs(node._total_energy_consumed_wh - expected_energy) < 1e-4
    finally:
        node.destroy_node()


def test_power_publisher_on_missing_effort_invalidation(tmp_path: Path) -> None:
    from arena_robots.power_publisher import PowerPublisher

    config_content = """
power_system:
  system_voltage_v: 24.0
  global_drivetrain_efficiency: 0.70
  heating_coefficient_ch: 1.25
  battery_capacity_wh: 270.0

joint_metrics:
  topic: "joint_states"

static_power_w:
  compute_core: 30.0
  idle_motors: 5.0
"""
    config_path = tmp_path / "power.yaml"
    config_path.write_text(config_content)

    node = PowerPublisher(
        parameter_overrides=[
            Parameter("config_path", value=str(config_path)),
        ]
    )

    try:
        # JointState with velocity but empty effort
        msg = JointState()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.name = ["wheel_left", "wheel_right"]
        msg.velocity = [1.5, 1.5]
        msg.effort = []  # Missing effort!

        node._on_joint_state(msg)
        # Verify energy was not integrated with bogus zeros
        assert node._total_energy_consumed_wh == 0.0
    finally:
        node.destroy_node()
