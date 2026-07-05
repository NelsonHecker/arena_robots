"""Tests for arena_robots.moveit_factory's instance parameter (phase3 item8:
`build_moveit_params` gains `instance: str | None`, sibling to the bringup/adapter
`len(arms) != 1` guard removal) and the allocation-derived rendering gate (phase3a
rbvogui_plus collapse checklist items 6/7: SRDF composition + joint_limits parity)."""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET

import pytest

_XACRO = shutil.which("xacro")


class TestSelectArm:
    """`_select_arm` is the pure selection rule shared by `build_moveit_params`;
    it returns the `(key, entry)` pair so callers can render per-placement
    SRDF/joint_limits off the mount name."""

    def test_none_selects_sole_instance(self):
        from arena_robots.moveit_factory import _select_arm

        arms = {"arm": "the_arm"}
        assert _select_arm("robot", arms, None) == ("arm", "the_arm")

    def test_none_with_multiple_raises_value_error_listing_instances(self):
        from arena_robots.moveit_factory import _select_arm

        arms = {"arm0": "a0", "arm1": "a1"}
        with pytest.raises(ValueError, match=r"arm0.*arm1|arm1.*arm0"):
            _select_arm("robot", arms, None)

    def test_named_instance_selects_it(self):
        from arena_robots.moveit_factory import _select_arm

        arms = {"arm0": "a0", "arm1": "a1"}
        assert _select_arm("robot", arms, "arm1") == ("arm1", "a1")

    def test_unknown_instance_raises_key_error_listing_instances(self):
        from arena_robots.moveit_factory import _select_arm

        arms = {"arm0": "a0", "arm1": "a1"}
        with pytest.raises(KeyError, match=r"arm0.*arm1|arm1.*arm0"):
            _select_arm("robot", arms, "bogus")


class TestBuildMoveitParamsInstanceParity:
    """rbvogui_plus is single-arm (author key "arm"): instance=None and the
    explicit instance="arm" must build byte-identical params."""

    def test_none_and_explicit_instance_agree(self):
        from arena_robots.moveit_factory import build_moveit_params

        default_params = build_moveit_params("rbvogui_plus")
        explicit_params = build_moveit_params("rbvogui_plus", instance="arm")
        assert default_params is not None
        assert default_params == explicit_params

    def test_unknown_instance_raises_key_error(self):
        from arena_robots.moveit_factory import build_moveit_params

        with pytest.raises(KeyError, match="arm"):
            build_moveit_params("rbvogui_plus", instance="bogus")


def _norm(value: str) -> str:
    """The documented rename map: the mount-specific group name renames wholesale
    (arm0_manipulator has no `arm_`-substring counterpart in the legacy naming,
    it's ur_manipulator), everything else just drops the mount digit."""
    if value == "arm0_manipulator":
        return "ur_manipulator"
    return value.replace("arm0_", "arm_")


def _canon_srdf(xml_text: str) -> frozenset:
    root = ET.fromstring(xml_text)
    items: set[tuple] = set()
    for group in root.iter("group"):
        items.add(("group", _norm(group.get("name", ""))))
        for chain in group.findall("chain"):
            items.add(("chain", _norm(group.get("name", "")), _norm(chain.get("base_link", "")), _norm(chain.get("tip_link", ""))))
    for gs in root.iter("group_state"):
        joints = tuple(sorted((_norm(j.get("name", "")), j.get("value", "")) for j in gs.findall("joint")))
        items.add(("group_state", gs.get("name", ""), _norm(gs.get("group", "")), joints))
    for dc in root.iter("disable_collisions"):
        items.add(("disable_collisions", frozenset({_norm(dc.get("link1", "")), _norm(dc.get("link2", ""))})))
    return frozenset(items)


def _arm_link_names(urdf_text: str) -> frozenset:
    root = ET.fromstring(urdf_text)
    return frozenset(_norm(link.get("name", "")) for link in root.iter("link") if "arm" in link.get("name", ""))


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
class TestBuildMoveitParamsAllocationParity:
    """Phase3a gate (parametrized-robots-phase3.md, rbvogui_plus collapse checklist
    items 6/7): moveit params for the allocation-derived rbvogui[arm=ur5e] must match
    rbvogui_plus's hand-authored ones, modulo the documented rename map (arm0_ -> arm_,
    group arm0_manipulator -> ur_manipulator)."""

    @pytest.fixture(scope="class")
    def params_pair(self) -> tuple[dict, dict]:
        from arena_robots.assembly import RequestPart
        from arena_robots.moveit_factory import build_moveit_params

        mine = build_moveit_params("rbvogui", tf_prefix="robot_", parts={"arm": [RequestPart(variant="ur5e")]})
        plus = build_moveit_params("rbvogui_plus", tf_prefix="robot_")
        assert mine is not None
        assert plus is not None
        return mine, plus

    def test_srdf_semantic_parity(self, params_pair: tuple[dict, dict]) -> None:
        mine, plus = params_pair
        assert _canon_srdf(mine["robot_description_semantic"]) == _canon_srdf(plus["robot_description_semantic"])

    def test_joint_limits_parity(self, params_pair: tuple[dict, dict]) -> None:
        mine, plus = params_pair
        mine_jl = mine["robot_description_planning"]["joint_limits"]
        plus_jl = plus["robot_description_planning"]["joint_limits"]
        assert {_norm(k): v for k, v in mine_jl.items()} == {_norm(k): v for k, v in plus_jl.items()}

    def test_robot_description_arm_link_set_parity(self, params_pair: tuple[dict, dict]) -> None:
        mine, plus = params_pair
        assert _arm_link_names(mine["robot_description"]) == _arm_link_names(plus["robot_description"])

    def test_legacy_path_unchanged(self, params_pair: tuple[dict, dict]) -> None:
        """rbvogui_plus's own output must be untouched by the allocation-derived
        rendering: its SRDF keeps the ur_manipulator group + swerve-corner DCs, and
        its robot_description still comes from the static chassis xacro, not a
        wrapper render (evidenced by the hand-spliced ros2_control tag name,
        "rbvogui_plus_arm", from urdf/base_hw/rbvogui_plus.ros2_control.urdf, rather
        than the wrapper's synthesized "<robot>_system" merged-tag name)."""
        _, plus = params_pair
        root = ET.fromstring(plus["robot_description_semantic"])
        assert "ur_manipulator" in {g.get("name") for g in root.iter("group")}
        assert any("front_right_base_wheel" in (dc.get("link1", "") + dc.get("link2", "")) for dc in root.iter("disable_collisions"))
        assert 'name="rbvogui_plus_arm"' in plus["robot_description"]
        assert "rbvogui_plus_system" not in plus["robot_description"]
