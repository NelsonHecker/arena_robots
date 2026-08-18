"""Tests for AcousticsPublisher ROS 2 Node."""

from __future__ import annotations

from pathlib import Path

from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


def test_acoustics_publisher_initialization(tmp_path: Path) -> None:
    from arena_robots.acoustics_publisher import AcousticsPublisher

    profile_content = """
L_base_0: 40.0
beta_0: 45.0
beta_1: 18.0
beta_2: 5.0
beta_3: 3.0
omega_ref: 5.0
tau_ref: 10.0
omega_deadband: 0.05
omega_active: 0.20
sigma_base: 1.5
sigma_dynamic: 2.0
sigma_no_effort: 4.0
"""
    profile_path = tmp_path / "test_profile.yaml"
    profile_path.write_text(profile_content)

    node = AcousticsPublisher(
        parameter_overrides=[
            Parameter("profile_path", value=str(profile_path)),
            Parameter("topic", value="/test_acoustics"),
        ]
    )

    try:
        assert node._L_base_0 == 40.0
        assert abs(node._P_base - 10.0**4.0) < 1e-5
    finally:
        node.destroy_node()


def test_acoustics_publisher_on_joint_state(tmp_path: Path) -> None:
    from arena_robots.acoustics_publisher import AcousticsPublisher

    profile_content = """
L_base_0: 40.0
beta_0: 45.0
beta_1: 18.0
beta_2: 5.0
beta_3: 3.0
omega_ref: 5.0
tau_ref: 10.0
omega_deadband: 0.05
omega_active: 0.20
sigma_base: 1.5
sigma_dynamic: 2.0
sigma_no_effort: 4.0
"""
    profile_path = tmp_path / "test_profile.yaml"
    profile_path.write_text(profile_content)

    node = AcousticsPublisher(
        parameter_overrides=[
            Parameter("profile_path", value=str(profile_path)),
            Parameter("topic", value="/test_acoustics"),
        ]
    )

    try:
        msg = JointState()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.name = ["front_left_wheel_joint", "front_right_wheel_joint"]
        msg.velocity = [1.0, 1.5]
        msg.effort = [2.0, 2.5]

        # Call joint state handler
        node._on_joint_state(msg)
        assert node._ema_p_drive > 0.0
    finally:
        node.destroy_node()

