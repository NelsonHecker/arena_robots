"""Tests for arena_robots.caps."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml


@pytest.fixture()
def caps_dir(tmp_path: Path) -> Path:
    return tmp_path / "caps"


def _write_cap(caps_dir: Path, name: str, data: dict) -> None:
    caps_dir.mkdir(parents=True, exist_ok=True)
    (caps_dir / f"{name}.yaml").write_text(yaml.dump(data))


class TestCapConfig:
    def test_sub_returns_empty_dict_when_key_missing(self, caps_dir):
        from arena_robots.caps import CapConfig

        cfg = CapConfig(path=caps_dir / "mobile.yaml", raw={"foo": "bar"})
        assert cfg.sub("missing") == {}

    def test_sub_returns_nested_dict(self, caps_dir):
        from arena_robots.caps import CapConfig

        cfg = CapConfig(path=caps_dir / "mobile.yaml", raw={"nav2": {"planner": "GridBased"}})
        assert cfg.sub("nav2") == {"planner": "GridBased"}

    def test_sub_raises_on_non_dict(self, caps_dir):
        from arena_robots.caps import CapConfig

        cfg = CapConfig(path=caps_dir / "mobile.yaml", raw={"nav2": "not_a_dict"})
        with pytest.raises(ValueError, match="must be a mapping"):
            cfg.sub("nav2")


class TestMobileSpec:
    def test_defaults(self, caps_dir):
        from arena_robots.caps import MobileSpec

        spec = MobileSpec(path=caps_dir / "mobile.yaml", raw={})
        assert spec.odom_frame == "odom"
        assert spec.sensor_frame is None
        assert spec.radius is None
        assert spec.is_holonomic is False

    def test_explicit_values(self, caps_dir):
        from arena_robots.caps import MobileSpec

        spec = MobileSpec(
            path=caps_dir / "mobile.yaml",
            raw={
                "odom_frame": "robot_odom",
                "sensor_frame": "laser_frame",
                "radius": 0.3,
                "is_holonomic": True,
            },
        )
        assert spec.odom_frame == "robot_odom"
        assert spec.sensor_frame == "laser_frame"
        assert spec.radius == pytest.approx(0.3)
        assert spec.is_holonomic is True

    def test_real_minimal_yaml_load(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "mobile.yaml").write_text(yaml.dump({"odom_frame": "myodom"}))
        robot_caps = RobotCaps(caps_dir=caps)
        mob = robot_caps.mobile
        assert mob.odom_frame == "myodom"
        assert mob.sensor_frame is None


class TestRobotCaps:
    def test_available_empty_dir(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        rc = RobotCaps(caps_dir=caps)
        assert rc.available == frozenset()

    def test_available_no_dir(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        rc = RobotCaps(caps_dir=tmp_path / "nonexistent")
        assert rc.available == frozenset()

    def test_available_lists_yaml_stems(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "mobile.yaml").write_text("{}")
        (caps / "arm.yaml").write_text("{}")
        rc = RobotCaps(caps_dir=caps)
        assert rc.available == frozenset({"mobile", "arm"})

    def test_load_cap_file_caches(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "mobile.yaml").write_text(yaml.dump({"odom_frame": "odom"}))
        rc = RobotCaps(caps_dir=caps)
        first = rc._load_cap_file("mobile")
        second = rc._load_cap_file("mobile")
        assert first is second

    def test_load_cap_file_missing_raises(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        rc = RobotCaps(caps_dir=caps)
        with pytest.raises(FileNotFoundError, match="not declared"):
            rc._load_cap_file("mobile")

    def test_load_cap_file_non_dict_raises(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "mobile.yaml").write_text("- list_item\n")
        rc = RobotCaps(caps_dir=caps)
        with pytest.raises(ValueError, match="must be a mapping"):
            rc._load_cap_file("mobile")


class TestInstanceSpecs:
    def test_arm_instances(self, tmp_path: Path):
        from arena_robots.caps import ArmSpec, RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "arm.yaml").write_text(
            yaml.dump({"my_arm": {"base_link": "base", "tip_link": "tip", "chain": ["j1"], "controller": "arm_ctrl"}})
        )
        rc = RobotCaps(caps_dir=caps)
        arms = rc.arm
        assert "my_arm" in arms
        arm = arms["my_arm"]
        assert isinstance(arm, ArmSpec)
        assert arm.base_link == "base"
        assert arm.tip_link == "tip"
        assert arm.chain == ["j1"]
        assert arm.controller == "arm_ctrl"

    def test_arm_non_dict_entry_raises(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "arm.yaml").write_text(yaml.dump({"my_arm": "not_a_dict"}))
        rc = RobotCaps(caps_dir=caps)
        with pytest.raises(ValueError, match="must be a mapping"):
            _ = rc.arm

    def test_lift_instances(self, tmp_path: Path):
        from arena_robots.caps import LiftSpec, RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "lift.yaml").write_text(yaml.dump({"lift1": {"joint": "lift_joint", "controller": "lift_ctrl"}}))
        rc = RobotCaps(caps_dir=caps)
        lifts = rc.lift
        assert "lift1" in lifts
        lift = lifts["lift1"]
        assert isinstance(lift, LiftSpec)
        assert lift.joint == "lift_joint"
        assert lift.controller == "lift_ctrl"

    def test_lift_missing_joint_raises(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "lift.yaml").write_text(yaml.dump({"lift1": {"controller": "ctrl"}}))
        rc = RobotCaps(caps_dir=caps)
        with pytest.raises(ValueError, match="missing 'joint'"):
            rc.lift["lift1"].joint

    def test_gripper_instances(self, tmp_path: Path):
        from arena_robots.caps import GripperSpec, RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "gripper.yaml").write_text(
            yaml.dump({"gripper1": {"arm": "my_arm", "joint": "gripper_joint", "controller": "gripper_ctrl"}})
        )
        rc = RobotCaps(caps_dir=caps)
        grippers = rc.gripper
        assert "gripper1" in grippers
        g = grippers["gripper1"]
        assert isinstance(g, GripperSpec)
        assert g.arm == "my_arm"
        assert g.joint == "gripper_joint"
        assert g.controller == "gripper_ctrl"

    def test_gripper_arm_none_when_absent(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "gripper.yaml").write_text(yaml.dump({"g": {"joint": "j", "controller": "c"}}))
        rc = RobotCaps(caps_dir=caps)
        assert rc.gripper["g"].arm is None

    def test_gripper_missing_joint_raises(self, tmp_path: Path):
        from arena_robots.caps import RobotCaps

        caps = tmp_path / "caps"
        caps.mkdir()
        (caps / "gripper.yaml").write_text(yaml.dump({"g": {"controller": "c"}}))
        rc = RobotCaps(caps_dir=caps)
        with pytest.raises(ValueError, match="missing 'joint'"):
            rc.gripper["g"].joint


class TestArmSpecSRDF:
    def _make_srdf_xml(self, group_name: str, base: str, tip: str, joints: list[str]) -> str:
        joints_xml = "".join(f'<joint name="{j}"/>' for j in joints)
        return f"""<robot><group name="{group_name}"><chain base_link="{base}" tip_link="{tip}"/>{joints_xml}</group></robot>"""

    def test_parse_srdf_group_direct_xml(self, tmp_path: Path):
        from arena_robots.caps import _parse_srdf_group, _resolve_find_ref

        srdf_path = tmp_path / "robot.srdf"
        srdf_path.write_text(self._make_srdf_xml("arm", "base", "tip", ["j1", "j2"]))
        result = _parse_srdf_group(str(srdf_path), "arm")
        assert result["base_link"] == "base"
        assert result["tip_link"] == "tip"
        assert result["chain"] == ["j1", "j2"]

    def test_parse_srdf_group_missing_group_raises(self, tmp_path: Path):
        from arena_robots.caps import _parse_srdf_group

        srdf_path = tmp_path / "robot.srdf"
        srdf_path.write_text('<robot><group name="other_arm"></group></robot>')
        with pytest.raises(ValueError, match="no <group"):
            _parse_srdf_group(str(srdf_path), "arm")

    def test_parse_srdf_group_no_chain_no_joints(self, tmp_path: Path):
        from arena_robots.caps import _parse_srdf_group

        srdf_path = tmp_path / "robot.srdf"
        srdf_path.write_text('<robot><group name="arm"></group></robot>')
        result = _parse_srdf_group(str(srdf_path), "arm")
        assert "base_link" not in result
        assert "tip_link" not in result
        assert "chain" not in result

    def test_parse_srdf_xacro_variant(self, tmp_path: Path):
        from arena_robots.caps import _parse_srdf_group

        xacro_path = tmp_path / "robot.srdf.xacro"
        xacro_path.write_text(self._make_srdf_xml("my_arm", "bl", "tl", []))
        xml_output = self._make_srdf_xml("my_arm", "bl", "tl", [])
        with patch("subprocess.check_output", return_value=xml_output):
            result = _parse_srdf_group(str(xacro_path), "my_arm")
        assert result["base_link"] == "bl"
        assert result["tip_link"] == "tl"

    def test_arm_spec_uses_srdf_fallback(self, tmp_path: Path):
        from arena_robots.caps import ArmSpec

        srdf_path = tmp_path / "robot.srdf"
        srdf_path.write_text(self._make_srdf_xml("arm", "b", "t", ["jA"]))
        arm = ArmSpec(
            path=tmp_path / "arm.yaml",
            raw={"srdf": str(srdf_path), "controller": "ctrl"},
            name="arm",
        )
        assert arm.base_link == "b"
        assert arm.tip_link == "t"
        assert arm.chain == ["jA"]

    def test_arm_explicit_wins_over_srdf(self, tmp_path: Path):
        from arena_robots.caps import ArmSpec

        srdf_path = tmp_path / "robot.srdf"
        srdf_path.write_text(self._make_srdf_xml("arm", "srdf_base", "srdf_tip", []))
        arm = ArmSpec(
            path=tmp_path / "arm.yaml",
            raw={"srdf": str(srdf_path), "base_link": "explicit_base", "tip_link": "explicit_tip", "chain": ["explicit_j"], "controller": "ctrl"},
            name="arm",
        )
        assert arm.base_link == "explicit_base"
        assert arm.tip_link == "explicit_tip"
        assert arm.chain == ["explicit_j"]

    def test_arm_missing_controller_always_raises(self, tmp_path: Path):
        from arena_robots.caps import ArmSpec

        arm = ArmSpec(
            path=tmp_path / "arm.yaml",
            raw={"base_link": "b", "tip_link": "t", "chain": ["j"]},
            name="arm",
        )
        with pytest.raises(ValueError, match="missing 'controller'"):
            _ = arm.controller

    def test_arm_no_srdf_no_base_link_raises(self, tmp_path: Path):
        from arena_robots.caps import ArmSpec

        arm = ArmSpec(path=tmp_path / "arm.yaml", raw={"controller": "ctrl"}, name="arm")
        with pytest.raises(ValueError, match="no base_link"):
            _ = arm.base_link


class TestResolveRef:
    def test_plain_path(self, tmp_path: Path):
        from arena_robots.caps import _resolve_find_ref

        result = _resolve_find_ref(str(tmp_path))
        assert result == tmp_path

    def test_find_ref_calls_ament(self, tmp_path: Path):
        from arena_robots.caps import _resolve_find_ref

        with patch("ament_index_python.packages.get_package_share_path", return_value=tmp_path) as mock_get:
            result = _resolve_find_ref("$(find some_pkg)/robots/robot.srdf")
        mock_get.assert_called_once_with("some_pkg")
        assert result == tmp_path / "robots/robot.srdf"

    def test_strips_leading_slash_from_rest(self, tmp_path: Path):
        from arena_robots.caps import _resolve_find_ref

        with patch("ament_index_python.packages.get_package_share_path", return_value=tmp_path):
            result = _resolve_find_ref("$(find pkg)/file.srdf")
        assert result == tmp_path / "file.srdf"
