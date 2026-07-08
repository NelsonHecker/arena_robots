"""Tests for components/arm/{ur5e,ur10e} (phase3 sec2.10.1). `effective_caps` (item5)
now consumes the `caps:` block via `RobotCaps`/`_substitute_keys`; render_wrapper_xacro
wiring of `ros2_control`/`control` is still separate work."""

from __future__ import annotations

import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from arena_rclpy_mixins.yaml_replace import YAMLReplacer
from arena_robots.caps import ArmSpec
from arena_robots.catalog import Catalog, ComponentSpec

COMPONENTS_ROOT = Path(__file__).resolve().parent.parent / "components"
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


@pytest.fixture(params=["ur5e", "ur10e"])
def variant(request: pytest.FixtureRequest) -> str:
    return request.param


class TestComponentSpecFromYaml:
    """component.yaml parses via the landed ComponentSpec schema (catalog.py:77)."""

    def test_parses(self, variant: str) -> None:
        spec = ComponentSpec.from_yaml(COMPONENTS_ROOT / "arm" / variant / "component.yaml")
        assert spec.xacro_include == "$(find ur_description)/urdf/ur_macro.xacro"
        assert spec.xacro_macro == "ur_robot"
        assert spec.attach["name"] == variant
        # the arm renders its own native ros2_control tag, the loader's post-render
        # injection (_inject_ros2_control_joints) drives the merge from the
        # ros2_control.joints patch below, not from this attach key.
        assert "generate_ros2_control_tag" not in spec.attach

    def test_ros2_control_joints(self, variant: str) -> None:
        spec = ComponentSpec.from_yaml(COMPONENTS_ROOT / "arm" / variant / "component.yaml")
        assert len(spec.ros2_control_joints) == 6
        names = [j["name"] for j in spec.ros2_control_joints]
        assert names == [f"${{prefix}}${{mount}}_{j}" for j in ARM_JOINTS]
        for j in spec.ros2_control_joints:
            assert j["command_interfaces"] == ["position"]
            assert j["state_interfaces"] == ["position", "velocity"]

    def test_control_block(self, variant: str) -> None:
        spec = ComponentSpec.from_yaml(COMPONENTS_ROOT / "arm" / variant / "component.yaml")
        assert spec.control["controller"] == "${mount}_controller"
        assert spec.control["type"] == "joint_trajectory_controller/JointTrajectoryController"
        params = spec.control["ros__parameters"]
        assert len(params["joints"]) == 6
        assert params["state_publish_rate"] == 50.0

    def test_caps_block_shape(self, variant: str) -> None:
        spec = ComponentSpec.from_yaml(COMPONENTS_ROOT / "arm" / variant / "component.yaml")
        caps = spec.caps
        assert caps["base_link"] == "${prefix}${parent}"
        assert caps["tip_link"] == "${prefix}${mount}_tool0"
        assert len(caps["chain"]) == 6
        assert caps["moveit"]["planning_group"] == "${mount}_manipulator"
        assert caps["moveit"]["srdf"]["path"] == f"components/arm/{variant}/srdf/{variant}.srdf.xacro"
        assert caps["moveit"]["joint_limits"]["path"] == f"components/arm/{variant}/joint_limits.yaml"
        assert set(caps["named_poses"]) == {"stow", "ready", "wave_up", "wave_l", "wave_r"}
        assert caps["workspace"]["type"] == "box"


class TestRenderedCapsIsArmSpecCompatible:
    """Rendering caps: with a concrete mount (as `effective_caps`, item5, now does)
    must produce a dict ArmSpec (caps.py:388) can front, including named_poses.
    named_poses joint names are dict KEYS; YAMLReplacer alone only substitutes dict
    values (and `**spread` keys), so `arena_robots.caps._substitute_keys` does a
    second pass over the already-value-rendered tree to fix those up too."""

    def _rendered_caps(self, variant: str) -> dict:
        from arena_robots.caps import _substitute_keys

        spec = ComponentSpec.from_yaml(COMPONENTS_ROOT / "arm" / variant / "component.yaml")
        context = {"mount": "arm0", "prefix": "robot_", "parent": "chassis_link"}
        rendered = YAMLReplacer(context).replace(copy.deepcopy(spec.caps))
        return _substitute_keys(rendered, context)

    def test_scalar_fields_resolve(self, variant: str) -> None:
        caps = self._rendered_caps(variant)
        arm = ArmSpec(path=Path("test"), raw=caps, name="arm0")
        assert arm.base_link == "robot_chassis_link"
        assert arm.tip_link == "robot_arm0_tool0"
        assert arm.chain == [f"robot_arm0_{j}" for j in ARM_JOINTS]
        assert arm.controller == "arm0_controller"
        assert arm.planning_group == "arm0_manipulator"
        assert arm.workspace is not None

    def test_named_poses_keys_are_mount_substituted(self, variant: str) -> None:
        caps = self._rendered_caps(variant)
        arm = ArmSpec(path=Path("test"), raw=caps, name="arm0")
        stow_joints = arm.named_poses["stow"]
        assert "robot_arm0_shoulder_pan_joint" in stow_joints
        assert "${prefix}${mount}_shoulder_pan_joint" not in stow_joints


class TestSrdfFragment:
    def test_parses_as_xml(self, variant: str) -> None:
        root = ET.parse(COMPONENTS_ROOT / "arm" / variant / "srdf" / f"{variant}.srdf.xacro").getroot()
        group = root.find("group")
        assert group.get("name") == "$(arg mount)_manipulator"
        assert len(root.findall("group_state")) == 2
        # 8 intra-arm + 2 mount-adjacency (chassis_link vs arm base_link/base_link_inertia,
        # moved in from the legacy monolith so the fragment is self-contained, phase3 item6)
        assert len(root.findall("disable_collisions")) == 10

    def test_ur5e_and_ur10e_fragments_are_identical(self) -> None:
        """group_state joint values are generic UR presets, not ur_type-dependent
        (evidence: rbrobout_plus's ur10e srdf carries the same values as the ur5e
        _plus robots); only the <robot name=...> attribute should differ."""
        ur5e = ET.tostring(ET.parse(COMPONENTS_ROOT / "arm" / "ur5e" / "srdf" / "ur5e.srdf.xacro").getroot())
        ur10e = ET.tostring(ET.parse(COMPONENTS_ROOT / "arm" / "ur10e" / "srdf" / "ur10e.srdf.xacro").getroot())
        assert ur5e.replace(b"ur5e", b"ur10e") == ur10e


class TestJointLimits:
    def test_parses_and_has_six_joints(self, variant: str) -> None:
        data = yaml.safe_load((COMPONENTS_ROOT / "arm" / variant / "joint_limits.yaml").read_text())
        limits = data["joint_limits"]
        assert set(limits) == {f"${{prefix}}${{mount}}_{j}" for j in ARM_JOINTS}
        for entry in limits.values():
            assert entry["has_velocity_limits"] is True
            assert entry["max_acceleration"] == 15.0

    def test_ur10e_velocity_differs_from_ur5e_defect(self) -> None:
        """fitsweep defect: rbrobout_plus (ur10e) byte-copied ur5e's joint_limits.yaml.
        The ur10e component must NOT reproduce that: shoulder joints are slower (UR10e
        120deg/s vs UR5e's 180deg/s here) and wrists are NOT doubled to 2*pi."""
        ur5e = yaml.safe_load((COMPONENTS_ROOT / "arm" / "ur5e" / "joint_limits.yaml").read_text())["joint_limits"]
        ur10e = yaml.safe_load((COMPONENTS_ROOT / "arm" / "ur10e" / "joint_limits.yaml").read_text())["joint_limits"]
        assert ur5e != ur10e
        assert ur10e["${prefix}${mount}_shoulder_pan_joint"]["max_velocity"] == pytest.approx(math.radians(120))
        assert ur10e["${prefix}${mount}_wrist_1_joint"]["max_velocity"] == pytest.approx(math.radians(180))
        assert ur5e["${prefix}${mount}_wrist_1_joint"]["max_velocity"] == pytest.approx(2 * math.pi)


class TestCatalogGet:
    def test_catalog_resolves_both_variants(self, variant: str) -> None:
        catalog = Catalog(root=COMPONENTS_ROOT)
        spec = catalog.get("arm", variant)
        assert spec.caps["tip_link"] == "${prefix}${mount}_tool0"


class TestEffectiveCapsRbvoguiPlusParity:
    """rbvogui_plus collapse checklist step4 (phase3 sec 'rbvogui_plus collapse
    checklist'): `RobotCaps` rendering the ur5e component at mount='arm' must
    reproduce today's static robots/rbvogui_plus/caps/arm.yaml for every field
    `effective_caps` (item5) actually owns. `moveit.srdf`/`moveit.joint_limits`
    paths and `moveit.planning_group` are excluded: those now correctly point at
    the component's own files/group naming (item2, not item5's rendering surface),
    a legitimate divergence from the pre-migration static file, not a byte-for-byte
    target."""

    def test_matches_static_caps_for_owned_fields(self) -> None:
        from arena_robots.assembly import Mount, Placement, ResolvedAssembly
        from arena_robots.caps import RobotCaps

        static = yaml.safe_load((COMPONENTS_ROOT.parent / "robots" / "rbvogui_plus" / "caps" / "arm.yaml").read_text())["arm"]

        mount = Mount(name="arm", parent="chassis_link", xyz=(0.0, 0.0, 0.235), accepts=frozenset({"arm"}))
        resolved = ResolvedAssembly(placements=[Placement(type="arm", variant="ur5e", mount=mount)])
        catalog = Catalog(root=COMPONENTS_ROOT)
        rc = RobotCaps(caps_dir=Path("/dev/null"), resolved=resolved, catalog=catalog)
        rendered = rc.arm["arm"].raw

        owned = {"base_link", "tip_link", "chain", "controller", "workspace", "named_poses"}
        assert {k: rendered[k] for k in owned} == {k: static[k] for k in owned}


class TestEffectiveCapsHonorsMountFrame:
    """A mount's ``frame`` (identity stem, sec2.x) drives caps content via
    ``catalog._frame_stem``, not the mount's addressing ``name``; the instance dict
    key stays the addressing name regardless."""

    def test_tip_link_and_chain_use_frame_not_name(self) -> None:
        from arena_robots.assembly import Mount, Placement, ResolvedAssembly
        from arena_robots.caps import RobotCaps

        mount = Mount(name="arm", parent="chassis_link", xyz=(0.0, 0.0, 0.235), accepts=("arm",), frame="arm0")
        resolved = ResolvedAssembly(placements=[Placement(type="arm", variant="ur5e", mount=mount)])
        catalog = Catalog(root=COMPONENTS_ROOT)
        rc = RobotCaps(caps_dir=Path("/dev/null"), resolved=resolved, catalog=catalog)

        arms = rc.arm
        assert set(arms) == {"arm"}
        arm = arms["arm"].raw
        assert arm["tip_link"] == "robot_arm0_tool0"
        assert arm["chain"][0] == "robot_arm0_shoulder_pan_joint"


class TestEffectiveCapsHonorsChassisPrefix:
    """A zero-prefix chassis (e.g. jackal, ``assembly.yaml`` ``prefix: ""``) must
    render arm caps without the ``robot_`` default baked in."""

    def test_zero_prefix_chassis_renders_unprefixed(self) -> None:
        from arena_robots.assembly import Mount, Placement, ResolvedAssembly
        from arena_robots.caps import RobotCaps

        mount = Mount(name="arm", parent="chassis_link", xyz=(0.0, 0.0, 0.235), accepts=("arm",))
        resolved = ResolvedAssembly(placements=[Placement(type="arm", variant="ur5e", mount=mount)])
        catalog = Catalog(root=COMPONENTS_ROOT)
        rc = RobotCaps(caps_dir=Path("/dev/null"), resolved=resolved, catalog=catalog, prefix="")

        arm = rc.arm["arm"].raw
        assert arm["base_link"] == "chassis_link"
        assert arm["tip_link"] == "arm_tool0"
        assert arm["chain"][0] == "arm_shoulder_pan_joint"
