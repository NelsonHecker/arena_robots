"""Tests for arena_robots.bringup.arm.moveit - MoveItArmBringup multi-instance
contract (phase3 item10: N-instance launch loop, sole-arm namespace parity).

Uses the legacy static ``caps/arm.yaml`` (dict-keyed, multi-instance) path so
these tests don't depend on the assembly/catalog rendering pipeline (item9,
not yet landed) or on the installed share tree being rebuilt."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

_ARM_ENTRY: dict = {
    "base_link": "chassis_link",
    "tip_link": "tool0",
    "chain": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    "controller": "arm_controller",
    "moveit": {"package": "ur_moveit_config", "planning_group": "ur_manipulator"},
}


def _write_robot(tmp_path: Path, *, arm_cap: dict) -> Path:
    rd = tmp_path / "robot"
    rd.mkdir()
    (rd / "model_params.yaml").write_text(yaml.dump({}))
    caps_dir = rd / "caps"
    caps_dir.mkdir()
    (caps_dir / "arm.yaml").write_text(yaml.dump(arm_cap))
    return rd


def _single_arm_robot(tmp_path: Path) -> Path:
    return _write_robot(tmp_path, arm_cap={"arm": _ARM_ENTRY})


def _dual_arm_robot(tmp_path: Path) -> Path:
    return _write_robot(tmp_path, arm_cap={"arm0": _ARM_ENTRY, "arm1": _ARM_ENTRY})


class TestArmsAndNamespaceSoleInstance:
    def test_arms_has_one_entry(self, tmp_path: Path):
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        view = RobotView(_single_arm_robot(tmp_path))
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        assert list(bringup.arms()) == ["arm"]

    def test_arm_namespace_has_no_extra_level(self, tmp_path: Path):
        """Sole-arm robots must keep today's flat topology: no `{mount}` level."""
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        view = RobotView(_single_arm_robot(tmp_path))
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        assert bringup.arm_namespace("arm") == bringup.namespace
        assert str(bringup.arm_namespace("arm")) == "/env_0/myrobot"

    def test_launch_actions_pass_empty_instance(self, tmp_path: Path):
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        view = RobotView(_single_arm_robot(tmp_path))
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        (action,) = bringup._launch_actions()
        launch_args = dict(action.launch_arguments)
        assert launch_args["namespace"] == "/env_0/myrobot"
        assert launch_args["instance"] == ""


class TestArmsAndNamespaceMultiInstance:
    def test_arms_has_two_entries(self, tmp_path: Path):
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        view = RobotView(_dual_arm_robot(tmp_path))
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        assert set(bringup.arms()) == {"arm0", "arm1"}

    def test_arm_namespace_adds_mount_level(self, tmp_path: Path):
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        view = RobotView(_dual_arm_robot(tmp_path))
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        assert str(bringup.arm_namespace("arm0")) == "/env_0/myrobot/arm0"
        assert str(bringup.arm_namespace("arm1")) == "/env_0/myrobot/arm1"

    def test_launch_actions_one_per_instance_with_named_instance(self, tmp_path: Path):
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        view = RobotView(_dual_arm_robot(tmp_path))
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        actions = bringup._launch_actions()
        assert len(actions) == 2
        namespaces = {dict(a.launch_arguments)["namespace"] for a in actions}
        assert namespaces == {"/env_0/myrobot/arm0", "/env_0/myrobot/arm1"}
        instances = {dict(a.launch_arguments)["instance"] for a in actions}
        assert instances == {"arm0", "arm1"}


class TestArmsRequiresCap:
    def test_no_arm_cap_raises(self, tmp_path: Path):
        from arena_robots.bringup.arm.moveit import MoveItArmBringup
        from arena_robots.Robot import RobotView

        rd = tmp_path / "robot"
        rd.mkdir()
        (rd / "model_params.yaml").write_text(yaml.dump({}))
        view = RobotView(rd)
        bringup = MoveItArmBringup(view, "/env_0/myrobot")
        with pytest.raises(ValueError, match="arm cap required but absent"):
            bringup.arms()
