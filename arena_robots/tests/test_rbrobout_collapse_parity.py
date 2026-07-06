"""Phase 3b collapse gate (parametrized-robots-phase3.md, "Phase 3b design" sec6):
``rbrobout[structure=rbrobout_top_cover,lift=ewellix_900mm,arm=ur10e]`` must reproduce
rbrobout_plus's top_cover/lift/arm subtree and merged ros2_control tag. Unlike the
rbvogui/rbkairos pairs, rbrobout's laser/imu placement has NO drift against
rbrobout_plus (parent/xyz/rpy verified byte-identical against rbrobout_plus.urdf.xacro's
front/rear_laser_offset_*/imu_offset_* properties), so this gate also checks the FULL
sensor inventory, not just the arm/lift/structure surface."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from arena_robots.assembly import RequestPart, resolve
from arena_robots.catalog import Catalog, render_wrapper_xacro
from arena_robots.Robot import RobotIdentifier

COMPONENTS_ROOT = Path(__file__).resolve().parent.parent / "components"
GOLDEN = Path(__file__).resolve().parent / "golden"

_XACRO = shutil.which("xacro")


def _render(path: Path) -> ET.Element:
    rendered = subprocess.run([_XACRO, str(path)], capture_output=True, text=True, check=True).stdout
    return ET.fromstring(rendered)


def _norm(name: str) -> str:
    """The documented rename map (phase3b sec6): lift_->lift0_, arm_->arm0_.
    top_cover is unrenamed (mount-independent naming, no `${mount}` in its attach)."""
    return name.replace("arm0_", "arm_").replace("lift0_", "lift_")


def _canon(el: ET.Element) -> dict:
    out: dict = {"tag": el.tag, "attrs": {k: _canon_value(v) for k, v in sorted(el.attrib.items())}}
    kids = [_canon(c) for c in el]
    if kids:
        out["kids"] = sorted(kids, key=repr)
    if el.text and el.text.strip():
        out["text"] = el.text.strip()
    return out


def _canon_value(value: str) -> str | tuple[float, ...]:
    parts = value.split()
    try:
        return tuple(round(float(p), 6) for p in parts)
    except ValueError:
        return _norm(value)


def _mechanism_elements(root: ET.Element) -> dict[tuple[str, str], dict]:
    return {
        (el.tag, _norm(el.get("name", ""))): _canon(el)
        for el in root
        if el.tag in ("link", "joint") and any(tok in el.get("name", "") for tok in ("arm", "lift", "top_cover"))
    }


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
class TestRboroutCollapseParity:
    @pytest.fixture(scope="class")
    def trees(self, tmp_path_factory: pytest.TempPathFactory) -> tuple[ET.Element, ET.Element]:
        """``assembled`` is the live rbrobout[parts] render; ``reference`` is rbrobout_plus's
        frozen collapse golden."""
        view = RobotIdentifier("rbrobout").resolve_sync()
        assert view.assembly is not None
        resolved = resolve(
            view.assembly,
            {
                "structure": [RequestPart(variant="rbrobout_top_cover")],
                "lift": [RequestPart(variant="ewellix_900mm")],
                "arm": [RequestPart(variant="ur10e")],
            },
        )
        wrapper_path = tmp_path_factory.mktemp("collapse") / "rbrobout_full.urdf.xacro"
        wrapper_path.write_text(render_wrapper_xacro(view, resolved, catalog=Catalog(root=COMPONENTS_ROOT)))
        assembled = _render(wrapper_path)
        reference = ET.parse(GOLDEN / "rbrobout_plus_collapse.urdf").getroot()
        return assembled, reference

    def test_arm_lift_structure_subtree_structurally_identical(self, trees: tuple[ET.Element, ET.Element]) -> None:
        assembled, reference = trees
        assert _mechanism_elements(assembled) == _mechanism_elements(reference)

    def test_one_merged_ros2_control_tag_with_reference_joint_set(self, trees: tuple[ET.Element, ET.Element]) -> None:
        assembled, reference = trees
        assembled_tags = assembled.findall("ros2_control")
        reference_tags = reference.findall("ros2_control")
        assert len(assembled_tags) == 1
        assert len(reference_tags) == 1
        assembled_joints = {_norm(j.get("name", "")) for j in assembled_tags[0].iter("joint")}
        reference_joints = {j.get("name", "") for j in reference_tags[0].iter("joint")}
        assert assembled_joints == reference_joints
        plugins = [p.text for p in assembled_tags[0].iter("plugin")]
        assert plugins == ["gz_ros2_control/GazeboSimSystem"]

    def test_gz_sensor_inventory_matches(self, trees: tuple[ET.Element, ET.Element]) -> None:
        """No documented drift for the rbrobout/rbrobout_plus laser/imu pair (unlike
        rbkairos/rbvogui): this gate checks the full sensor inventory, not just
        arm/lift/structure."""
        assembled, reference = trees

        def inventory(root: ET.Element) -> set[tuple[str, str]]:
            return {
                (sensor.get("name", ""), sensor.get("type", ""))
                for gz in root.iter("gazebo")
                for sensor in gz.findall("sensor")
            }

        assert inventory(assembled) == inventory(reference)
