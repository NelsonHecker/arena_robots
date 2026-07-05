"""Tests for arena_robots.catalog: component.yaml loading + sensor-template rendering
(.claude/parametrized-robots.md sec2.5; parametrized-robots-fitsweep.md sec4)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from arena_robots.assembly import Mount, Placement, ResolvedAssembly
from arena_robots.catalog import Catalog, ComponentSpec, render_effective_control, render_effective_sensors, render_wrapper_xacro
from arena_robots.Robot import RobotView

LIDAR_COMPONENT = {
    "xacro": {
        "include": "sick_s300.xacro",
        "macro": "sick_s300",
        "attach": {"parent": "${prefix}${mount}_mount"},
        "args": {"min_angle": -2.269, "max_angle": 2.269},
    },
    "sensor": {
        "gz": [
            {"name": "${name:-lidar}", "type": "laserscan", "topic": "${topic:-scan}", "frame": "${prefix}${mount}_link", "sensor": "${mount}"},
            {"name": "${name:-lidar}_points", "type": "pointcloud", "topic": "${topic:-scan}/points", "frame": "${prefix}${mount}_link", "sensor": "${mount}"},
        ]
    },
}

CAMERA_COMPONENT = {
    "xacro": {
        "include": "d435.xacro",
        "macro": "d435_rgbd",
        "attach": {"parent": "${prefix}${mount}_mount"},
        "args": {},
    },
    "sensor": {
        "gz": [
            {
                "name": "${name:-${mount}}_camera_image",
                "type": "image",
                "topic": "${topic:-${mount}}_camera/image",
                "frame": "${prefix}${mount}_rgbd_camera_color_frame",
                "sensor": "${mount}_camera_color",
            },
            {
                "name": "${name:-${mount}}_camera_info",
                "type": "camera_info",
                "topic": "${topic:-${mount}}_camera/camera_info",
                "frame": "${prefix}${mount}_rgbd_camera_color_frame",
                "sensor": "${mount}_camera_color",
            },
        ]
    },
}


ARM_COMPONENT = {
    "xacro": {
        "include": "ur_macro.xacro",
        "macro": "ur_robot",
        "attach": {"parent": "${prefix}${mount}_parent"},
        "args": {},
    },
    "ros2_control": {
        "joints": [
            {"name": "${prefix}${mount}_shoulder_pan_joint", "command_interfaces": ["position"], "state_interfaces": ["position", "velocity"]},
            {"name": "${prefix}${mount}_shoulder_lift_joint", "command_interfaces": ["position"], "state_interfaces": ["position", "velocity"]},
        ]
    },
    "control": {
        "controller": "${mount}_controller",
        "type": "joint_trajectory_controller/JointTrajectoryController",
        "ros__parameters": {
            "joints": ["${prefix}${mount}_shoulder_pan_joint", "${prefix}${mount}_shoulder_lift_joint"],
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"],
            "state_publish_rate": 50.0,
            "action_monitor_rate": 20.0,
        },
    },
    "caps": {
        "base_link": "${prefix}${mount}_base_link",
        "tip_link": "${prefix}${mount}_tool0",
    },
}


def _write_component(root: Path, type_: str, variant: str, data: dict) -> Path:
    d = root / type_ / variant
    d.mkdir(parents=True, exist_ok=True)
    path = d / "component.yaml"
    with open(path, "w") as f:
        yaml.safe_dump(data, f)
    return path


def _mount(name: str, accepts: list[str]) -> Mount:
    return Mount(name=name, parent="base_link", xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), accepts=frozenset(accepts))


def _write_robot_view(tmp_path: Path, name: str = "testbot", *, control_gate: bool = True) -> RobotView:
    """Minimal on-disk robot dir: just enough for ``render_wrapper_xacro`` (a
    chassis xacro with one ``xacro:arg`` and a stub base_hw ros2_control.urdf).
    ``RobotView`` takes a bare path (arena_simulation_setup.tree.PathView); no
    ament index / ROS environment involved."""
    robot_dir = tmp_path / "robots" / name
    urdf_dir = robot_dir / "urdf"
    urdf_dir.mkdir(parents=True)
    gate = '  <xacro:arg name="generate_ros2_control_tag" default="true"/>\n' if control_gate else ""
    (urdf_dir / f"{name}.urdf.xacro").write_text(
        '<?xml version="1.0"?>\n'
        f'<robot name="{name}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:arg name="prefix" default="robot_"/>\n'
        f"{gate}"
        "</robot>\n"
    )
    base_hw_dir = urdf_dir / "base_hw"
    base_hw_dir.mkdir()
    (base_hw_dir / f"{name}.ros2_control.urdf").write_text("<robot/>\n")
    return RobotView(robot_dir)


class TestComponentSpecFromYaml:
    def test_parses_xacro_and_sensor_gz(self, tmp_path: Path):
        path = _write_component(tmp_path, "lidar", "sick_s300", LIDAR_COMPONENT)
        spec = ComponentSpec.from_yaml(path)
        assert spec.xacro_include == "sick_s300.xacro"
        assert spec.xacro_macro == "sick_s300"
        assert spec.args == {"min_angle": -2.269, "max_angle": 2.269}
        assert len(spec.sensor_gz) == 2
        assert spec.sensor_gz[0]["type"] == "laserscan"

    def test_rejects_missing_xacro_macro(self, tmp_path: Path):
        path = _write_component(tmp_path, "lidar", "broken", {"xacro": {"include": "x.xacro"}})
        with pytest.raises(ValueError, match="xacro"):
            ComponentSpec.from_yaml(path)

    def test_parses_ros2_control_control_caps(self, tmp_path: Path):
        path = _write_component(tmp_path, "arm", "ur5e", ARM_COMPONENT)
        spec = ComponentSpec.from_yaml(path)
        assert len(spec.ros2_control_joints) == 2
        assert spec.ros2_control_joints[0]["name"] == "${prefix}${mount}_shoulder_pan_joint"
        assert spec.control["controller"] == "${mount}_controller"
        assert spec.control["ros__parameters"]["state_publish_rate"] == 50.0
        assert spec.caps["tip_link"] == "${prefix}${mount}_tool0"

    def test_ros2_control_control_caps_default_empty(self, tmp_path: Path):
        path = _write_component(tmp_path, "lidar", "sick_s300", LIDAR_COMPONENT)
        spec = ComponentSpec.from_yaml(path)
        assert spec.ros2_control_joints == []
        assert spec.control == {}
        assert spec.caps == {}


class TestCatalogGet:
    def test_missing_variant_lists_available(self, tmp_path: Path):
        _write_component(tmp_path, "lidar", "sick_s300", LIDAR_COMPONENT)
        _write_component(tmp_path, "lidar", "vlp16", LIDAR_COMPONENT)
        catalog = Catalog(root=tmp_path)
        with pytest.raises(RuntimeError) as excinfo:
            catalog.get("lidar", "nope")
        msg = str(excinfo.value)
        assert "lidar/nope" in msg
        assert "sick_s300" in msg and "vlp16" in msg

    def test_get_caches_component_instance(self, tmp_path: Path):
        _write_component(tmp_path, "lidar", "sick_s300", LIDAR_COMPONENT)
        catalog = Catalog(root=tmp_path)
        assert catalog.get("lidar", "sick_s300") is catalog.get("lidar", "sick_s300")


class TestRenderEffectiveSensors:
    """Reproduces rbtheron's lidar+camera SensorSpec ground truth (model_params.yaml)
    from a two-mount default (front) + overridden (rear) assembly, to prove the
    override vocabulary (name, topic) is sufficient for the fleet's asymmetric case."""

    @pytest.fixture
    def catalog(self, tmp_path: Path) -> Catalog:
        _write_component(tmp_path, "lidar", "sick_s300", LIDAR_COMPONENT)
        _write_component(tmp_path, "camera", "d435_rgbd", CAMERA_COMPONENT)
        return Catalog(root=tmp_path)

    def _resolved(self) -> ResolvedAssembly:
        front_laser = _mount("front_laser", ["lidar"])
        rear_laser = _mount("rear_laser", ["lidar"])
        front_cam = _mount("front", ["camera"])
        rear_cam = _mount("rear", ["camera"])
        return ResolvedAssembly(
            placements=[
                Placement(type="lidar", variant="sick_s300", mount=front_laser),
                Placement(type="lidar", variant="sick_s300", mount=rear_laser, overrides={"name": "lidar_rear", "topic": "scan/rear"}),
                Placement(type="camera", variant="d435_rgbd", mount=front_cam),
                Placement(type="camera", variant="d435_rgbd", mount=rear_cam),
            ]
        )

    def test_exact_sensorspec_tuples_match_rbtheron(self, catalog: Catalog):
        sensors = render_effective_sensors(self._resolved(), catalog)
        got = [(s.name, s.type, s.topic, s.frame, s.sensor) for s in sensors]
        assert got == [
            ("lidar", "laserscan", "${namespace}/scan", "robot_front_laser_link", "front_laser"),
            ("lidar_points", "pointcloud", "${namespace}/scan/points", "robot_front_laser_link", "front_laser"),
            ("lidar_rear", "laserscan", "${namespace}/scan/rear", "robot_rear_laser_link", "rear_laser"),
            ("lidar_rear_points", "pointcloud", "${namespace}/scan/rear/points", "robot_rear_laser_link", "rear_laser"),
            ("front_camera_image", "image", "${namespace}/front_camera/image", "robot_front_rgbd_camera_color_frame", "front_camera_color"),
            ("front_camera_info", "camera_info", "${namespace}/front_camera/camera_info", "robot_front_rgbd_camera_color_frame", "front_camera_color"),
            ("rear_camera_image", "image", "${namespace}/rear_camera/image", "robot_rear_rgbd_camera_color_frame", "rear_camera_color"),
            ("rear_camera_info", "camera_info", "${namespace}/rear_camera/camera_info", "robot_rear_rgbd_camera_color_frame", "rear_camera_color"),
        ]

    def test_topic_default_fallback_without_override(self, catalog: Catalog):
        front_laser = _mount("front_laser", ["lidar"])
        resolved = ResolvedAssembly(placements=[Placement(type="lidar", variant="sick_s300", mount=front_laser)])
        sensors = render_effective_sensors(resolved, catalog)
        assert sensors[0].topic == "${namespace}/scan"
        assert sensors[1].topic == "${namespace}/scan/points"

    def test_override_wins_over_default(self, catalog: Catalog):
        rear_laser = _mount("rear_laser", ["lidar"])
        resolved = ResolvedAssembly(
            placements=[Placement(type="lidar", variant="sick_s300", mount=rear_laser, overrides={"name": "lidar_rear", "topic": "scan/rear"})]
        )
        sensors = render_effective_sensors(resolved, catalog)
        assert sensors[0].name == "lidar_rear"
        assert sensors[0].topic == "${namespace}/scan/rear"

    def test_deepcopy_isolation_across_placements(self, catalog: Catalog):
        """Renders front-then-rear from the SAME cached ComponentSpec; if render_effective_sensors
        failed to deepcopy the shared template dicts (YAMLReplacer.replace mutates in place),
        the rear placement's overrides would bleed backward into the component's stored
        templates and corrupt a subsequent front render."""
        before = [dict(e) for e in catalog.get("lidar", "sick_s300").sensor_gz]

        render_effective_sensors(self._resolved(), catalog)

        after = catalog.get("lidar", "sick_s300").sensor_gz
        assert after == before

        front_laser = _mount("front_laser", ["lidar"])
        resolved_again = ResolvedAssembly(placements=[Placement(type="lidar", variant="sick_s300", mount=front_laser)])
        sensors = render_effective_sensors(resolved_again, catalog)
        assert sensors[0].name == "lidar"
        assert sensors[0].topic == "${namespace}/scan"


class TestRenderWrapperXacroMergedControl:
    """Phase3 sec2.10 merged ros2_control: no placement declares joints -> wrapper
    is byte-identical to today (chassis handles its own tag); a placement with
    ``ros2_control.joints`` -> wrapper synthesizes exactly one merged tag."""

    def test_no_joints_no_ros2_control_tag(self, tmp_path: Path):
        _write_component(tmp_path / "components", "lidar", "sick_s300", LIDAR_COMPONENT)
        catalog = Catalog(root=tmp_path / "components")
        view = _write_robot_view(tmp_path)
        front_laser = _mount("front_laser", ["lidar"])
        resolved = ResolvedAssembly(placements=[Placement(type="lidar", variant="sick_s300", mount=front_laser)])

        wrapper = render_wrapper_xacro(view, resolved, catalog=catalog)

        assert "<ros2_control" not in wrapper
        assert "base_hw_joints" not in wrapper

    def test_arm_joints_synthesize_one_merged_tag(self, tmp_path: Path):
        _write_component(tmp_path / "components", "arm", "ur5e", ARM_COMPONENT)
        catalog = Catalog(root=tmp_path / "components")
        view = _write_robot_view(tmp_path, "testbot")
        arm_mount = _mount("arm0", ["arm"])
        resolved = ResolvedAssembly(placements=[Placement(type="arm", variant="ur5e", mount=arm_mount)])

        wrapper = render_wrapper_xacro(view, resolved, catalog=catalog)

        assert wrapper.count("<ros2_control ") == 1
        assert '<ros2_control name="testbot_system" type="system">' in wrapper
        assert "<plugin>gz_ros2_control/GazeboSimSystem</plugin>" in wrapper
        base_hw_path = view.path / "urdf" / "base_hw" / "testbot.ros2_control.urdf"
        assert f'<xacro:include filename="{base_hw_path}"/>' in wrapper
        assert '<xacro:testbot_base_hw_joints prefix="$(arg prefix)"/>' in wrapper
        assert wrapper.count("<joint ") == 2
        assert '<joint name="$(arg prefix)arm0_shoulder_pan_joint">' in wrapper
        assert '<joint name="$(arg prefix)arm0_shoulder_lift_joint">' in wrapper
        assert '<command_interface name="position"/>' in wrapper
        assert '<state_interface name="velocity"/>' in wrapper
        # the merged tag closes once, after both placement joints
        assert wrapper.index("</ros2_control>") > wrapper.rindex("<joint ")
        # the chassis's internal tag is suppressed in favor of the merged one
        assert '<xacro:arg name="generate_ros2_control_tag" default="false"/>' in wrapper

    def test_ungated_chassis_with_joints_raises(self, tmp_path: Path):
        _write_component(tmp_path / "components", "arm", "ur5e", ARM_COMPONENT)
        catalog = Catalog(root=tmp_path / "components")
        view = _write_robot_view(tmp_path, control_gate=False)
        arm_mount = _mount("arm0", ["arm"])
        resolved = ResolvedAssembly(placements=[Placement(type="arm", variant="ur5e", mount=arm_mount)])

        with pytest.raises(RuntimeError, match="generate_ros2_control_tag"):
            render_wrapper_xacro(view, resolved, catalog=catalog)


RBVOGUI_BASE_CONTROL = {
    "controller_manager": {
        "ros__parameters": {
            "use_sim_time": True,
            "update_rate": 50,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "robotnik_base_controller": {"type": "arena_swerve_controller/SwerveController"},
        }
    },
    "joint_state_broadcaster": {"ros__parameters": {"use_sim_time": True, "publish_rate": 50}},
    "robotnik_base_controller": {"ros__parameters": {"use_sim_time": True}},
}


class TestRenderEffectiveControl:
    """rbvogui[arm=ur5e] control.yaml synthesis must reproduce rbvogui_plus's
    hand-authored control.yaml diff exactly: one controller_manager entry + one
    top-level controller section per arm placement."""

    def test_merges_arm_controller_into_rbvogui_shaped_base(self, tmp_path: Path):
        _write_component(tmp_path, "arm", "ur5e", ARM_COMPONENT)
        catalog = Catalog(root=tmp_path)
        arm_mount = _mount("arm", ["arm"])
        resolved = ResolvedAssembly(placements=[Placement(type="arm", variant="ur5e", mount=arm_mount)])

        merged, extra = render_effective_control(resolved, RBVOGUI_BASE_CONTROL, catalog)

        assert extra == ["arm_controller"]
        assert merged["controller_manager"]["ros__parameters"]["arm_controller"] == {
            "type": "joint_trajectory_controller/JointTrajectoryController"
        }
        assert merged["arm_controller"]["ros__parameters"]["joints"] == [
            "robot_arm_shoulder_pan_joint",
            "robot_arm_shoulder_lift_joint",
        ]
        assert merged["arm_controller"]["ros__parameters"]["state_publish_rate"] == 50.0
        # existing controller_manager entries survive the merge
        assert merged["controller_manager"]["ros__parameters"]["robotnik_base_controller"] == {
            "type": "arena_swerve_controller/SwerveController"
        }
        # base_control is not mutated
        assert "arm_controller" not in RBVOGUI_BASE_CONTROL["controller_manager"]["ros__parameters"]
        assert "arm_controller" not in RBVOGUI_BASE_CONTROL

    def test_no_control_declared_is_a_no_op(self, tmp_path: Path):
        _write_component(tmp_path, "lidar", "sick_s300", LIDAR_COMPONENT)
        catalog = Catalog(root=tmp_path)
        front_laser = _mount("front_laser", ["lidar"])
        resolved = ResolvedAssembly(placements=[Placement(type="lidar", variant="sick_s300", mount=front_laser)])

        merged, extra = render_effective_control(resolved, RBVOGUI_BASE_CONTROL, catalog)

        assert extra == []
        assert merged == RBVOGUI_BASE_CONTROL
