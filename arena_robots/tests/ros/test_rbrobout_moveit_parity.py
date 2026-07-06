"""Phase3b collapse gate, moveit params surface (parametrized-robots-phase3.md, "Phase
3b design" sec6): moveit params for the allocation-derived
rbrobout[structure=rbrobout_top_cover,lift=ewellix_900mm,arm=ur10e] must match
rbrobout_plus's frozen moveit params golden for SRDF semantics and the arm/lift link
set, modulo the documented rename map (lift_->lift0_, arm_->arm0_, group
arm0_manipulator->ur_manipulator).

Joint limits are the one surface that intentionally does NOT reach parity: fitsweep
logged rbrobout_plus's srdf/joint_limits.yaml as a byte-copy-from-ur5e defect (wrong
UR10e velocity/acceleration limits); the ur10e component (phase3 item2) carries the
corrected values, so this gate checks joint-name KEY parity only and asserts the
VALUES diverge by design, mirroring test_arm_components.py's defect test."""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

_XACRO = shutil.which("xacro")
GOLDEN = Path(__file__).resolve().parent.parent / "golden"


def _norm(value: str) -> str:
    if value == "arm0_manipulator":
        return "ur_manipulator"
    return value.replace("arm0_", "arm_").replace("lift0_", "lift_")


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


def _mechanism_link_names(urdf_text: str) -> frozenset:
    root = ET.fromstring(urdf_text)
    return frozenset(
        _norm(link.get("name", ""))
        for link in root.iter("link")
        if any(tok in link.get("name", "") for tok in ("arm", "lift", "top_cover"))
    )


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
class TestRboroutBuildMoveitParamsAllocationParity:
    @pytest.fixture(scope="class")
    def params(self) -> tuple[dict, dict]:
        """``assembled`` is built live; ``reference`` is rbrobout_plus's frozen moveit params golden."""
        from arena_robots.assembly import RequestPart
        from arena_robots.moveit_factory import build_moveit_params

        assembled = build_moveit_params(
            "rbrobout",
            tf_prefix="robot_",
            parts={
                "structure": [RequestPart(variant="rbrobout_top_cover")],
                "lift": [RequestPart(variant="ewellix_900mm")],
                "arm": [RequestPart(variant="ur10e")],
            },
        )
        reference = {
            "robot_description_semantic": (GOLDEN / "rbrobout_plus_moveit_semantic.srdf").read_text(),
            "robot_description": (GOLDEN / "rbrobout_plus_moveit_description.urdf").read_text(),
            "robot_description_planning": {"joint_limits": yaml.safe_load((GOLDEN / "rbrobout_plus_moveit_joint_limits.yaml").read_text())},
        }
        assert assembled is not None
        return assembled, reference

    def test_srdf_semantic_parity(self, params: tuple[dict, dict]) -> None:
        assembled, reference = params
        assert _canon_srdf(assembled["robot_description_semantic"]) == _canon_srdf(reference["robot_description_semantic"])

    def test_robot_description_arm_lift_link_set_parity(self, params: tuple[dict, dict]) -> None:
        assembled, reference = params
        assert _mechanism_link_names(assembled["robot_description"]) == _mechanism_link_names(reference["robot_description"])

    def test_joint_limits_keys_match_values_diverge_by_design(self, params: tuple[dict, dict]) -> None:
        assembled, reference = params
        assembled_jl = assembled["robot_description_planning"]["joint_limits"]
        reference_jl = reference["robot_description_planning"]["joint_limits"]
        assert {_norm(k) for k in assembled_jl} == {_norm(k) for k in reference_jl}
        assert assembled_jl != {_norm(k): v for k, v in reference_jl.items()}
