"""End-to-end arm-on-any-chassis (compose->transform->deliver): jackal
has no bespoke arm gate, yet ``jackal[top=arm/ur5e]`` must seat a ur5e on the new top
plate and merge its ros2_control joints into jackal's own GazeboSimSystem tag via the
loader's post-render injection. Exercises the whole chain: mount-centric grammar
(``build_request``) -> ``resolve`` placement -> ``render_wrapper_xacro`` -> xacro ->
``render_control_joints`` -> ``_inject_ros2_control_joints``."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from arena_robots.assembly import build_request, resolve
from arena_robots.catalog import Catalog, render_control_joints, render_wrapper_xacro
from arena_robots.Robot import RobotIdentifier
from arena_simulation_setup.utils.models.urdf import _inject_ros2_control_joints

COMPONENTS_ROOT = Path(__file__).resolve().parent.parent / "components"
_XACRO = shutil.which("xacro")
_UR_MACRO = Path(__file__).resolve().parent.parent.parent / "deps" / "ur_description" / "urdf" / "ur_macro.xacro"
_UR5E_JOINT_SUFFIXES = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
@pytest.mark.skipif(not _UR_MACRO.is_file(), reason="deps/ur_description not initialized; run `arena feature robots add arm/ur`")
class TestJackalArmEndToEnd:
    @pytest.fixture(scope="class")
    def merged(self, tmp_path_factory: pytest.TempPathFactory) -> ET.Element:
        """The rendered ``jackal[top=arm/ur5e]`` URDF after loader control-joint injection."""
        view = RobotIdentifier("jackal").resolve_sync()
        assert view.assembly is not None
        request, cleared_sockets, cleared_types = build_request(view.assembly, {"top": ["arm/ur5e"]})
        resolved = resolve(view.assembly, request, cleared_sockets=cleared_sockets, cleared_types=cleared_types)
        catalog = Catalog(root=COMPONENTS_ROOT)
        wrapper_path = tmp_path_factory.mktemp("jackal_arm") / "jackal_arm.urdf.xacro"
        wrapper_path.write_text(render_wrapper_xacro(view, resolved, catalog=catalog))
        rendered = ET.fromstring(subprocess.run([_XACRO, str(wrapper_path)], capture_output=True, text=True, check=True).stdout)
        _inject_ros2_control_joints(rendered, render_control_joints(resolved, catalog, prefix=view.assembly.prefix))
        return rendered

    def test_arm_seated_on_top_plate(self, merged: ET.Element) -> None:
        """The ur5e subtree rendered and attached (a chassis with no native arm support)."""
        links = {el.get("name", "") for el in merged.iter("link")}
        assert any("shoulder" in n or "wrist" in n for n in links), "ur5e arm links absent from the jackal URDF"

    def test_single_merged_gazebo_control_tag(self, merged: ET.Element) -> None:
        """Injection collapses jackal's own tag + the arm's native tag into one."""
        tags = merged.findall("ros2_control")
        assert len(tags) == 1
        assert [p.text for p in tags[0].iter("plugin")] == ["gz_ros2_control/GazeboSimSystem"]

    def test_arm_joints_injected_into_jackal_control(self, merged: ET.Element) -> None:
        """All six ur5e joints land in jackal's GazeboSimSystem tag, alongside the base drive joints."""
        control = merged.find("ros2_control")
        assert control is not None
        joint_names = {j.get("name", "") for j in control.iter("joint")}
        for suffix in _UR5E_JOINT_SUFFIXES:
            assert any(suffix in n for n in joint_names), f"ur5e '{suffix}' joint missing from merged ros2_control ({sorted(joint_names)})"
