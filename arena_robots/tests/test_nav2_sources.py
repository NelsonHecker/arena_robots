from __future__ import annotations

from arena_robots.nav2 import compile_sensors_to_collision_monitor
from arena_robots.Sensor import SensorSpec, SensorType


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


def test_nav2_kinematics_derived_yaml_baseline(tmp_path):
    import yaml
    from arena_robots.nav2 import Nav2KinematicsDerivedYAML
    import launch

    mobile_yaml = tmp_path / "mobile.yaml"
    mobile_yaml.write_text(
        "velocity_limits:\n"
        "  linear: [-2.0, 2.0]\n"
        "  angular: [-3.0, 3.0]\n"
        "acceleration_limits:\n"
        "  linear: 2.5\n"
        "  angular: 3.2\n"
    )

    sub = Nav2KinematicsDerivedYAML(str(mobile_yaml))
    context = launch.LaunchContext()
    path = sub.perform(context)

    with open(path) as f:
        data = yaml.safe_load(f)

    assert data["max_linear_vel"] == 2.0
    assert data["min_linear_vel"] == -2.0
    assert data["max_angular_vel"] == 3.0
    assert data["min_angular_vel"] == -3.0
    assert data["linear_acc"] == 2.5
    assert data["linear_decel"] == -2.5
    assert data["angular_acc"] == 3.2
    assert data["angular_decel"] == -3.2


def test_nav2_kinematics_derived_yaml_overrides(tmp_path):
    import yaml
    from arena_robots.nav2 import Nav2KinematicsDerivedYAML
    import launch

    mobile_yaml = tmp_path / "mobile.yaml"
    mobile_yaml.write_text(
        "velocity_limits:\n"
        "  linear: [-2.0, 2.0]\n"
        "  angular: [-3.0, 3.0]\n"
        "acceleration_limits:\n"
        "  linear: 2.5\n"
        "  angular: 3.2\n"
    )

    sub = Nav2KinematicsDerivedYAML(
        str(mobile_yaml),
        overrides={
            "max_linear_vel": "0.35",
            "linear_acc": "0.75",
            "max_angular_vel": "0.80",
        },
    )
    context = launch.LaunchContext()
    path = sub.perform(context)

    with open(path) as f:
        data = yaml.safe_load(f)

    assert data["max_linear_vel"] == 0.35
    assert data["min_linear_vel"] == -0.35
    assert data["linear_acc"] == 0.75
    assert data["linear_decel"] == -0.75
    assert data["max_angular_vel"] == 0.80
    assert data["min_angular_vel"] == -0.80

