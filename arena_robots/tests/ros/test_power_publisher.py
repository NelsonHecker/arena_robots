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


def test_power_publisher_with_drivetrain_augmentation(tmp_path: Path) -> None:
    from arena_robots.power_publisher import PowerPublisher

    config_content = """
power_system:
  system_voltage_v: 24.0
  global_drivetrain_efficiency: 0.80
  heating_coefficient_ch: 1.0
  battery_capacity_wh: 200.0
  drivetrain_damping: 0.010
  drivetrain_friction: 0.050
  rolling_resistance_crr: 0.020
  robot_mass_kg: 20.0
  wheel_radius_m: 0.10
  num_wheels: 2

joint_metrics:
  topic: "joint_states"

static_power_w:
  compute_core: 10.0
  idle_motors: 0.0
"""
    config_path = tmp_path / "power.yaml"
    config_path.write_text(config_content)

    node = PowerPublisher(
        parameter_overrides=[
            Parameter("config_path", value=str(config_path)),
        ]
    )

    try:
        # tau_roll = (0.020 * 20.0 * 9.81 * 0.10) / 2 = 0.1962 Nm
        # At omega = 5.0 rad/s with raw effort = 0.0 (steady state cruise in Gazebo):
        # tau_parasitic = 0.050 + 0.010 * 5.0 + 0.1962 = 0.2962 Nm
        # total_effort = 0.0 + 0.2962 = 0.2962 Nm
        # P_mech per wheel = (0.2962 * 5.0) / 0.80 = 1.85125 W
        # Total P_mech (2 wheels) = 3.7025 W
        # P_therm per wheel = 1.0 * (0.2962^2) = 0.08773444 W
        # Total P_therm (2 wheels) = 0.17546888 W
        # Total power = 10.0 + 3.7025 + 0.17546888 = 13.87796888 W

        msg = JointState()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.name = ["wheel_l", "wheel_r"]
        msg.velocity = [5.0, 5.0]
        msg.effort = [0.0, 0.0]

        node._on_joint_state(msg)

        msg2 = JointState()
        msg2.header.stamp.sec = 1
        msg2.header.stamp.nanosec = 0
        msg2.name = ["wheel_l", "wheel_r"]
        msg2.velocity = [5.0, 5.0]
        msg2.effort = [0.0, 0.0]

        node._on_joint_state(msg2)

        expected_energy = 13.87796888 / 3600.0
        assert abs(node._total_energy_consumed_wh - expected_energy) < 1e-4
    finally:
        node.destroy_node()

