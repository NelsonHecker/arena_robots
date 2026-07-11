"""Tests for the components/arm/ur family component (phase3 sec2.10.1, UR-family
collapse). One templated `arm/ur` component serves the whole UR family via the catalog's
`variants:` fallback; `${variant}` threads the ur_type into attach name, ur_description
config dir, and the per-variant MoveIt joint_limits path. `effective_caps` (item5)
consumes the `caps:` block via `RobotCaps`/`_substitute_keys`."""

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
FAMILY = COMPONENTS_ROOT / "arm" / "ur"
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
UR_VARIANTS = ["ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur8long", "ur10", "ur10e", "ur12e", "ur15", "ur16e", "ur18", "ur20", "ur30"]


class TestFamilyComponentSpec:
    """The single `arm/ur/component.yaml` parses and declares the family via `variants:`;
    its variant-specific surfaces (attach name, ur_type, joint_limits path) stay templated
    on `${variant}` in the un-rendered spec."""

    @pytest.fixture(scope="class")
    def spec(self) -> ComponentSpec:
        return ComponentSpec.from_yaml(FAMILY / "component.yaml")

    def test_declares_ur_family_variants(self, spec: ComponentSpec) -> None:
        assert spec.variants == UR_VARIANTS

    def test_attach_and_urtype_templated_on_variant(self, spec: ComponentSpec) -> None:
        assert spec.xacro_include == "$(find ur_description)/urdf/ur_macro.xacro"
        assert spec.xacro_macro == "ur_robot"
        assert spec.attach["name"] == "${variant}"
        assert spec.attach["joint_limits_parameters_file"] == "$(find ur_description)/config/${variant}/joint_limits.yaml"
        assert spec.caps["moveit"]["args"]["ur_type"] == "${variant}"

    def test_ros2_control_and_control_blocks(self, spec: ComponentSpec) -> None:
        names = [j["name"] for j in spec.ros2_control_joints]
        assert names == [f"${{prefix}}${{mount}}_{j}" for j in ARM_JOINTS]
        assert spec.control["controller"] == "${mount}_controller"
        assert spec.control["ros__parameters"]["state_publish_rate"] == 50.0

    def test_exports_tip_frame_for_gripper_chaining(self, spec: ComponentSpec) -> None:
        assert spec.frames == {"tip": "${prefix}${mount}_tool0"}

    def test_caps_moveit_paths_point_at_shared_family_files(self, spec: ComponentSpec) -> None:
        caps = spec.caps
        assert caps["base_link"] == "${prefix}${parent}"
        assert caps["tip_link"] == "${prefix}${mount}_tool0"
        assert caps["moveit"]["planning_group"] == "${mount}_manipulator"
        assert caps["moveit"]["srdf"]["path"] == "components/arm/ur/srdf/ur.srdf.xacro"
        assert caps["moveit"]["joint_limits"]["path"] == "components/arm/ur/joint_limits/${variant}.yaml"
        assert set(caps["named_poses"]) == {"stow", "ready", "wave_up", "wave_l", "wave_r"}


class TestCatalogFamilyResolution:
    """A variant with no dedicated dir resolves against the family component whose
    `variants:` names it (catalog family-fallback); every UR variant maps to `arm/ur`."""

    @pytest.fixture(scope="class")
    def catalog(self) -> Catalog:
        return Catalog(root=COMPONENTS_ROOT)

    @pytest.mark.parametrize("variant", UR_VARIANTS)
    def test_every_variant_resolves_to_family(self, catalog: Catalog, variant: str) -> None:
        spec = catalog.get("arm", variant)
        assert variant in spec.variants
        assert spec.caps["tip_link"] == "${prefix}${mount}_tool0"

    def test_resolution_is_cached(self, catalog: Catalog) -> None:
        assert catalog.get("arm", "ur5e") is catalog.get("arm", "ur5e")

    def test_unknown_variant_lists_family_variants(self, catalog: Catalog) -> None:
        with pytest.raises(RuntimeError) as excinfo:
            catalog.get("arm", "ur999")
        msg = str(excinfo.value)
        assert "arm/ur999" in msg
        assert "ur5e" in msg and "ur10e" in msg


class TestRenderedCapsIsArmSpecCompatible:
    """Rendering caps with a concrete mount/variant (as `effective_caps`, item5, does)
    must produce a dict `ArmSpec` (caps.py) can front, including mount-substituted
    `named_poses` keys (`_substitute_keys` second pass over the value-rendered tree)."""

    def _rendered_caps(self, variant: str) -> dict:
        from arena_robots.caps import _substitute_keys

        spec = Catalog(root=COMPONENTS_ROOT).get("arm", variant)
        context = {"mount": "arm0", "variant": variant, "prefix": "robot_", "parent": "chassis_link"}
        rendered = YAMLReplacer(context).replace(copy.deepcopy(spec.caps))
        return _substitute_keys(rendered, context)

    @pytest.mark.parametrize("variant", ["ur3e", "ur5e", "ur10e"])
    def test_scalar_fields_resolve(self, variant: str) -> None:
        caps = self._rendered_caps(variant)
        arm = ArmSpec(path=Path("test"), raw=caps, name="arm0")
        assert arm.base_link == "robot_chassis_link"
        assert arm.tip_link == "robot_arm0_tool0"
        assert arm.chain == [f"robot_arm0_{j}" for j in ARM_JOINTS]
        assert arm.controller == "arm0_controller"
        assert arm.planning_group == "arm0_manipulator"
        assert arm.workspace is not None

    def test_named_poses_keys_are_mount_substituted(self) -> None:
        caps = self._rendered_caps("ur5e")
        arm = ArmSpec(path=Path("test"), raw=caps, name="arm0")
        stow_joints = arm.named_poses["stow"]
        assert "robot_arm0_shoulder_pan_joint" in stow_joints
        assert "${prefix}${mount}_shoulder_pan_joint" not in stow_joints


class TestSrdfFragment:
    """The shared `arm/ur/srdf/ur.srdf.xacro` is UR-type-agnostic (link/joint names are
    identical across the family); only prefix/mount/parent are xacro args."""

    def test_parses_as_xml(self) -> None:
        root = ET.parse(FAMILY / "srdf" / "ur.srdf.xacro").getroot()
        group = root.find("group")
        assert group.get("name") == "$(arg mount)_manipulator"
        assert len(root.findall("group_state")) == 2
        # 8 intra-arm + 2 mount-adjacency (chassis_link vs arm base_link/base_link_inertia)
        assert len(root.findall("disable_collisions")) == 10


class TestJointLimits:
    """Per-variant MoveIt planning joint_limits under `arm/ur/joint_limits/<variant>.yaml`:
    velocity from ur_description (matches the URDF/controller), uniform `max_acceleration`,
    templated `${prefix}${mount}_*` keys. ur5e keeps its shipped 360deg/s wrists."""

    def _limits(self, variant: str) -> dict:
        return yaml.safe_load((FAMILY / "joint_limits" / f"{variant}.yaml").read_text())["joint_limits"]

    @pytest.mark.parametrize("variant", UR_VARIANTS)
    def test_each_variant_has_six_templated_joints(self, variant: str) -> None:
        limits = self._limits(variant)
        assert set(limits) == {f"${{prefix}}${{mount}}_{j}" for j in ARM_JOINTS}
        for entry in limits.values():
            assert entry["has_velocity_limits"] is True
            assert entry["max_acceleration"] == 15.0

    def test_ur5e_keeps_shipped_360_wrists(self) -> None:
        limits = self._limits("ur5e")
        assert limits["${prefix}${mount}_shoulder_pan_joint"]["max_velocity"] == pytest.approx(math.pi)
        assert limits["${prefix}${mount}_wrist_1_joint"]["max_velocity"] == pytest.approx(2 * math.pi)

    def test_ur10e_differs_from_ur5e(self) -> None:
        """The prior fitsweep defect byte-copied ur5e's limits onto ur10e. The family must
        not: ur10e shoulders are slower (120deg/s) and wrists are 180deg/s, not 360deg/s."""
        ur5e, ur10e = self._limits("ur5e"), self._limits("ur10e")
        assert ur5e != ur10e
        assert ur10e["${prefix}${mount}_shoulder_pan_joint"]["max_velocity"] == pytest.approx(math.radians(120))
        assert ur10e["${prefix}${mount}_wrist_1_joint"]["max_velocity"] == pytest.approx(math.radians(180))

    def test_cb_ur5_follows_ur_description_not_ur5e(self) -> None:
        """CB-series ur5 has no shipped consumer, so it takes ur_description's conservative
        180deg/s wrists (the URDF/controller value), unlike ur5e's authored 360deg/s."""
        assert self._limits("ur5")["${prefix}${mount}_wrist_1_joint"]["max_velocity"] == pytest.approx(math.pi)


class TestEffectiveCapsRbvoguiPlusParity:
    """rbvogui_plus collapse checklist step4: `RobotCaps` rendering the ur5e variant at
    mount='arm' must reproduce today's static robots/rbvogui_plus/caps/arm.yaml for every
    field `effective_caps` (item5) owns. `moveit.srdf`/`moveit.joint_limits`/`planning_group`
    are excluded: those correctly point at the component's own files/group naming."""

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
    """A mount's ``frame`` (identity stem) drives caps content via ``catalog._frame_stem``,
    not the mount's addressing ``name``; the instance dict key stays the addressing name."""

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
    """A zero-prefix chassis (e.g. jackal, ``assembly.yaml`` ``prefix: ""``) must render
    arm caps without the ``robot_`` default baked in."""

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
