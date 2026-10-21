from __future__ import annotations

import math
import re
from pathlib import Path

import launch
import pytest
import yaml
from arena_bringup.substitutions import YAMLMergeSubstitution
from arena_rclpy_mixins.yaml_replace import YAMLReplacer
from arena_robots.nav2 import (
    KINEMATIC_KEYS,
    Nav2KinematicsDerivedYAML,
    SensorsDerivedYAML,
    compile_sensors_to_collision_monitor,
    compile_sensors_to_nav2,
    compile_sensors_to_stvl,
    split_planar_sensors,
)
from arena_robots.Sensor import SensorSpec, SensorType
from arena_robots.sensor_geometry import SensorGeometry

PKG = Path(__file__).resolve().parents[2] / "arena_robots"
ROBOTS_DIR = PKG / "robots"
NAV2_YAML = PKG / "config" / "nav2" / "nav2.yaml"
SOCIAL_OVERLAY = PKG / "config" / "nav2" / "overlays" / "social_cost_layer.yaml"
CONTROLLERS_DIR = PKG / "config" / "nav2" / "controllers"
DWB_CONTROLLER = CONTROLLERS_DIR / "dwb" / "controller_config.yaml"

_NAV2_CONTROLLERS = frozenset(
    {
        "dwb",
        "mppi",
        "hateb",
        "regulated_pure_pursuit",
        "graceful",
        "rotation_shim",
    }
)

_DRL_CONTROLLERS = frozenset(
    {
        "crowdnav",
        "crowdnav_attngraph",
        "drlvo",
        "rosnav_rl",
        "sicnav",
    }
)

_SUBSTITUTION_RE = re.compile(r"\$\{([^}]+)\}")


def _substitution_names(value: object) -> set[str]:
    if isinstance(value, str):
        return {match.group(1).split(":-", 1)[0].strip() for match in _SUBSTITUTION_RE.finditer(value)}
    if isinstance(value, dict):
        children: list[object] = [*value.keys(), *value.values()]
    elif isinstance(value, list):
        children = value
    else:
        return set()
    return set().union(*(_substitution_names(child) for child in children))


def _controller_config(controller: str) -> dict[str, object]:
    return yaml.safe_load((CONTROLLERS_DIR / controller / "controller_config.yaml").read_text())


def _kinematics_mobile(tmp_path: Path) -> Path:
    mobile_yaml = tmp_path / "mobile.yaml"
    mobile_yaml.write_text("velocity_limits:\n  linear: [-1.5, 2.25]\n  lateral: [-0.8, 1.2]\n  angular: [-2.0, 3.5]\nacceleration_limits:\n  linear: 1.75\n  lateral: 1.1\n  angular: 2.25\n")
    return mobile_yaml


def _context(**values: str) -> launch.LaunchContext:
    context = launch.LaunchContext()
    context.launch_configurations.update(values)
    return context


def _resolve_controller(
    controller: str,
    tmp_path: Path,
    overrides: dict[str, str],
) -> dict[str, object]:
    derived = Nav2KinematicsDerivedYAML(str(_kinematics_mobile(tmp_path)), overrides=overrides).perform_load(_context(**overrides))
    resolved = YAMLReplacer(derived).replace(_controller_config(controller))
    return resolved["controller_plugins_dict"]["FollowPath"]


def _spec(name: str, type_: SensorType, topic: str) -> SensorSpec:
    return SensorSpec(name=name, type=type_, topic=topic, frame="")


def test_collision_sources_typed_by_sensor():
    sources = compile_sensors_to_collision_monitor(
        [
            _spec("lidar", SensorType.LASERSCAN, "ns/lidar"),
            _spec("rgbd_camera_points", SensorType.POINTCLOUD, "ns/rgbd_camera/points"),
            _spec("rgbd_camera_image", SensorType.IMAGE, "ns/rgbd_camera/image"),
        ]
    )
    assert sources["lidar"] == {"type": "scan", "topic": "ns/lidar"}
    assert sources["rgbd_camera_points"] == {
        "type": "pointcloud",
        "topic": "ns/rgbd_camera/points",
        "min_height": 0.1,
        "max_height": 2.0,
    }
    assert "rgbd_camera_image" not in sources


def _lidar_spec(name: str, sensor: str, *, cloud: bool = True) -> list[SensorSpec]:
    scan = SensorSpec(name=name, type=SensorType.LASERSCAN, topic=f"ns/{name}", frame="", sensor=sensor)
    points = SensorSpec(name=f"{name}_points", type=SensorType.POINTCLOUD, topic=f"ns/{name}/points", frame="", sensor=sensor)
    return [scan, points] if cloud else [scan]


_LIDAR_3D = SensorGeometry("lidar", 2 * math.pi, -0.2618, 0.2618, 0.08, 12.0)
_LIDAR_2D = SensorGeometry("lidar", 4.5379, 0.0, 0.0, 0.1, 10.0)
_CAMERA = SensorGeometry("camera", 1.5184, -0.6, 0.6, 0.105, 8.0)


def _stvl(specs: list[SensorSpec], geometry: dict[str, SensorGeometry], default_range: float) -> dict:
    planar, volumetric = split_planar_sensors(specs, geometry)
    assert planar == []
    return compile_sensors_to_stvl(volumetric, geometry, default_range=default_range)


def test_stvl_marks_each_sensor_once_from_its_cloud():
    sources = _stvl(_lidar_spec("lidar", "gpu_lidar"), {"gpu_lidar": _LIDAR_3D}, 30.0)
    assert list(sources) == ["lidar_points", "lidar_points_clearing"]
    marking, frustum = sources["lidar_points"], sources["lidar_points_clearing"]
    assert (marking["marking"], marking["clearing"]) == (True, False)
    assert (frustum["marking"], frustum["clearing"]) == (False, True)
    assert marking["topic"] == frustum["topic"] == "ns/lidar/points"
    assert marking["obstacle_range"] == 11.0
    assert marking["min_obstacle_height"] == 0.1
    assert (frustum["min_z"], frustum["max_z"]) == (0.08, 12.0)
    assert frustum["vertical_fov_angle"] == pytest.approx(0.5236)
    assert frustum["horizontal_fov_angle"] > 6.27


def test_planar_lidar_raytraces_its_scan_outside_the_voxel_layer():
    specs = _lidar_spec("front", "front_laser")
    planar, volumetric = split_planar_sensors(specs, {"front_laser": _LIDAR_2D})
    assert (planar, volumetric) == ([specs[0]], [])
    source = compile_sensors_to_nav2(planar, max_range=10.0)["front"]
    assert (source["data_type"], source["marking"], source["clearing"]) == ("LaserScan", True, True)
    assert source["min_obstacle_height"] == 0.05


def test_planar_lidar_without_a_scan_raytraces_its_cloud():
    cloud = _lidar_spec("front", "front_laser")[1]
    assert split_planar_sensors([cloud], {"front_laser": _LIDAR_2D}) == ([cloud], [])


def test_scan_only_sensor_without_geometry_is_planar():
    specs = _lidar_spec("rear", "rear_laser", cloud=False)
    assert split_planar_sensors(specs, {}) == (specs, [])


def test_stvl_camera_frustum_stays_inside_the_image_corners():
    camera = SensorSpec(name="cam_points", type=SensorType.POINTCLOUD, topic="ns/cam/points", frame="", sensor="cam")
    frustum = _stvl([camera], {"cam": _CAMERA}, 30.0)["cam_points_clearing"]
    assert frustum["horizontal_fov_angle"] == pytest.approx(1.5184)
    half = frustum["vertical_fov_angle"] / 2
    assert math.tan(half) == pytest.approx(math.tan(0.6) * math.cos(1.5184 / 2))
    assert (frustum["min_z"], frustum["max_z"]) == (0.105, 8.0)


def test_stvl_sensor_without_geometry_marks_only():
    sources = _stvl(_lidar_spec("lidar", "gpu_lidar"), {}, 12.0)
    assert list(sources) == ["lidar_points"]
    assert sources["lidar_points"]["obstacle_range"] == 11.0


def test_stvl_lidar_looking_only_upward_marks_only():
    upward = SensorGeometry("lidar", 2 * math.pi, 0.1, 0.5, 0.1, 10.0)
    assert list(_stvl(_lidar_spec("lidar", "gpu_lidar"), {"gpu_lidar": upward}, 30.0)) == ["lidar_points"]


def test_obstacle_layer_cloud_source_marks_and_clears():
    sources = compile_sensors_to_nav2([_spec("lidar_points", SensorType.POINTCLOUD, "ns/lidar/points")], max_range=12.0)
    assert list(sources) == ["lidar_points"]
    assert (sources["lidar_points"]["marking"], sources["lidar_points"]["clearing"]) == (True, True)


def _costmap_layers(robot: str) -> tuple[dict, dict, dict]:
    pytest.importorskip("xacro")
    robot_dir = ROBOTS_DIR / robot
    derived = SensorsDerivedYAML(str(robot_dir / "model_params.yaml"), str(robot_dir / "caps" / "mobile.yaml")).perform(launch.LaunchContext())
    variables = {"namespace": "ns", **yaml.safe_load(Path(derived).read_text())}
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["plugins"] == ["obstacle_layer", "voxel_layer", "inflation_layer"]
    global_ = nav2["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    replacer = YAMLReplacer(variables)
    return replacer.replace(local["obstacle_layer"]), replacer.replace(local["voxel_layer"]), replacer.replace(global_)


@pytest.mark.parametrize(
    ("robot", "planar_sources", "voxel_sources", "global_sources"),
    [
        ("jackal", [], ["lidar_points", "lidar_points_clearing"], ["lidar", "lidar_points"]),
        ("a1", [], ["camera_face_points", "camera_face_points_clearing", "lidar_points", "lidar_points_clearing"], ["lidar_points"]),
        ("mpo700", ["lidar", "lidar_rear"], [], ["lidar", "lidar_points", "lidar_rear", "lidar_rear_points"]),
        (
            "boxer",
            ["front_laser", "rear_laser"],
            ["front_realsense_mount_link_points", "front_realsense_mount_link_points_clearing"],
            ["front_laser", "front_laser_points", "rear_laser", "rear_laser_points", "front_realsense_mount_link_points"],
        ),
    ],
)
def test_costmap_layers_list_their_own_sources(robot, planar_sources, voxel_sources, global_sources):
    planar, voxel, global_ = _costmap_layers(robot)
    assert planar["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert voxel["plugin"] == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert isinstance(planar["observation_sources"], str)
    assert isinstance(voxel["observation_sources"], str)
    assert planar["observation_sources"].split() == planar_sources
    assert voxel["observation_sources"].split() == voxel_sources
    assert global_["observation_sources"].split() == global_sources
    for layer in (planar, voxel, global_):
        for name in layer["observation_sources"].split():
            assert layer[name]["topic"].startswith("ns/"), name
    for layer, names in ((planar, planar_sources), (global_, global_sources)):
        for name in names:
            assert (layer[name]["marking"], layer[name]["clearing"]) == (True, True)


def test_social_cost_layer_overlay_adds_the_layer_to_both_costmaps():
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    overlay = yaml.safe_load(SOCIAL_OVERLAY.read_text())
    for costmap in ("local_costmap", "global_costmap"):
        base = nav2[costmap][costmap]["ros__parameters"]
        over = overlay[costmap][costmap]["ros__parameters"]
        assert "social_cost_layer" not in base["plugins"]
        assert over["plugins"] == [*base["plugins"][: base["plugins"].index("inflation_layer")], "social_cost_layer", *base["plugins"][base["plugins"].index("inflation_layer") :]]
        assert over["social_cost_layer"]["plugin"] == "arena_social_cost_layer/SocialCostLayer"
        assert over["social_cost_layer"]["max_cost_value"] <= 252


def test_social_cost_layer_overlay_retunes_the_dwb_obstacle_critic():
    overlay = yaml.safe_load(SOCIAL_OVERLAY.read_text())
    assert overlay["substitutions"] == {"controller_plugins_dict": {"FollowPath": {"BaseObstacle.sum_scores": True, "BaseObstacle.scale": 0.3}}}
    dwb = yaml.safe_load(DWB_CONTROLLER.read_text())
    assert "BaseObstacle.sum_scores" not in dwb["controller_plugins_dict"]["FollowPath"]
    assert dwb["controller_plugins_dict"]["FollowPath"]["BaseObstacle.scale"] == 0.02
    variables = YAMLMergeSubstitution._recursive_merge({"goal_checker_plugin_class": "nav2_controller::SimpleGoalChecker", **dwb}, overlay["substitutions"])
    controller = YAMLReplacer(variables).replace(yaml.safe_load(NAV2_YAML.read_text())["controller_server"]["ros__parameters"])
    follow_path = controller["FollowPath"]
    assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
    assert (follow_path["BaseObstacle.sum_scores"], follow_path["BaseObstacle.scale"]) == (True, 0.3)
    assert follow_path["PathAlign.scale"] == 32.0


def test_nav2_kinematics_derived_yaml_baseline(tmp_path: Path) -> None:
    data = Nav2KinematicsDerivedYAML(str(_kinematics_mobile(tmp_path))).perform_load(launch.LaunchContext())

    assert data["max_linear_vel"] == 2.25
    assert data["min_linear_vel"] == -1.5
    assert data["max_lateral_vel"] == 1.2
    assert data["min_lateral_vel"] == -0.8
    assert data["max_angular_vel"] == 3.5
    assert data["min_angular_vel"] == -2.0
    assert data["linear_acc"] == 1.75
    assert data["linear_decel"] == -1.75
    assert data["lateral_acc"] == 1.1
    assert data["lateral_decel"] == -1.1
    assert data["angular_acc"] == 2.25
    assert data["angular_decel"] == -2.25


def test_nav2_kinematics_derived_yaml_overrides(tmp_path: Path) -> None:
    overrides = {
        "max_linear_vel": "0.42",
        "linear_acc": "0.6",
        "max_lateral_vel": "0.3",
        "lateral_acc": "0.5",
        "max_angular_vel": "0.9",
        "angular_acc": "1.4",
    }
    sub = Nav2KinematicsDerivedYAML(str(_kinematics_mobile(tmp_path)), overrides=overrides)
    data = sub.perform_load(_context(**overrides))

    assert data["max_linear_vel"] == 0.42
    assert data["min_linear_vel"] == -0.42
    assert data["linear_acc"] == 0.6
    assert data["linear_decel"] == -0.6
    assert data["max_lateral_vel"] == 0.3
    assert data["min_lateral_vel"] == -0.3
    assert data["lateral_acc"] == 0.5
    assert data["lateral_decel"] == -0.5
    assert data["max_angular_vel"] == 0.9
    assert data["min_angular_vel"] == -0.9
    assert data["angular_acc"] == 1.4
    assert data["angular_decel"] == -1.4


def test_nav2_kinematics_derived_yaml_keeps_explicit_negative_limit(tmp_path: Path) -> None:
    overrides = {"max_linear_vel": "0.42", "min_linear_vel": "-0.2"}
    sub = Nav2KinematicsDerivedYAML(str(_kinematics_mobile(tmp_path)), overrides=overrides)
    data = sub.perform_load(_context(**overrides))

    assert data["max_linear_vel"] == 0.42
    assert data["min_linear_vel"] == -0.2


def test_nav2_kinematics_derived_yaml_skips_unset_overrides() -> None:
    sub = Nav2KinematicsDerivedYAML(
        str(ROBOTS_DIR / "jackal" / "caps" / "mobile.yaml"),
        overrides=("max_linear_vel", "linear_acc"),
    )

    baseline = sub.perform_load(launch.LaunchContext())
    assert baseline["max_linear_vel"] == 0.5
    assert baseline["min_linear_vel"] == -0.5
    assert baseline["linear_acc"] == 2.5
    assert baseline["linear_decel"] == -2.5

    overridden = sub.perform_load(_context(linear_acc="0.6"))
    assert overridden["max_linear_vel"] == 0.5
    assert overridden["linear_acc"] == 0.6
    assert overridden["linear_decel"] == -0.6


@pytest.mark.parametrize("bad_value", ["not_a_float", "nan", "inf", "-inf"])
def test_nav2_kinematics_derived_yaml_rejects_invalid_override(tmp_path: Path, bad_value: str) -> None:
    sub = Nav2KinematicsDerivedYAML(str(_kinematics_mobile(tmp_path)), overrides=("max_linear_vel",))
    with pytest.raises(ValueError):
        sub.perform_load(_context(max_linear_vel=bad_value))


def test_nav2_kinematics_derived_yaml_rejects_unknown_override(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        Nav2KinematicsDerivedYAML(str(_kinematics_mobile(tmp_path)), overrides=("max_lin_vel",))


def test_every_controller_directory_is_classified() -> None:
    found = {path.parent.name for path in CONTROLLERS_DIR.glob("*/controller_config.yaml")}
    assert found == _NAV2_CONTROLLERS | _DRL_CONTROLLERS


@pytest.mark.parametrize("controller", sorted(_NAV2_CONTROLLERS))
def test_nav2_controller_references_a_configured_cap(controller: str) -> None:
    referenced = _substitution_names(_controller_config(controller))
    assert referenced & set(KINEMATIC_KEYS)


def test_every_kinematic_key_is_referenced() -> None:
    referenced = _substitution_names(yaml.safe_load(NAV2_YAML.read_text()))
    for controller in _NAV2_CONTROLLERS:
        referenced |= _substitution_names(_controller_config(controller))
    assert set(KINEMATIC_KEYS) <= referenced


@pytest.mark.parametrize("controller", sorted(_DRL_CONTROLLERS))
def test_drl_controller_is_explicitly_excluded(controller: str) -> None:
    referenced = _substitution_names(_controller_config(controller))
    assert not referenced & set(KINEMATIC_KEYS)


@pytest.mark.parametrize(
    ("controller", "defaults"),
    [
        ("graceful", {"v_linear_min": 0.1, "v_linear_max": 0.5, "v_angular_max": 5.0, "v_angular_min_in_place": 0.25}),
        ("rotation_shim", {"rotate_to_heading_angular_vel": 1.8, "max_angular_accel": 3.2}),
    ],
)
def test_controller_defaults_are_unchanged_without_overrides(controller: str, defaults: dict[str, float], tmp_path: Path) -> None:
    follow_path = _resolve_controller(controller, tmp_path, {})
    for key, expected in defaults.items():
        assert follow_path[key] == expected


@pytest.mark.parametrize(
    ("controller", "overrides"),
    [
        ("graceful", {"v_linear_min": "0.05", "v_linear_max": "0.35", "v_angular_max": "0.8", "v_angular_min_in_place": "0.4"}),
        ("rotation_shim", {"rotate_to_heading_angular_vel": "0.9", "max_angular_accel": "1.1"}),
    ],
)
def test_controller_specific_overrides_are_applied(controller: str, overrides: dict[str, str], tmp_path: Path) -> None:
    follow_path = _resolve_controller(controller, tmp_path, overrides)
    for key, expected in overrides.items():
        assert follow_path[key] == pytest.approx(float(expected))


def test_generic_caps_do_not_override_controller_tuning(tmp_path: Path) -> None:
    graceful = _resolve_controller(
        "graceful",
        tmp_path,
        {"max_linear_vel": "0.35", "max_angular_vel": "0.8", "angular_acc": "1.1"},
    )
    assert graceful["v_linear_max"] == 0.5
    assert graceful["v_angular_max"] == 5.0
    rotation_shim = _resolve_controller(
        "rotation_shim",
        tmp_path,
        {"max_angular_vel": "0.8", "angular_acc": "1.1"},
    )
    assert rotation_shim["rotate_to_heading_angular_vel"] == 1.8
    assert rotation_shim["max_angular_accel"] == 3.2
