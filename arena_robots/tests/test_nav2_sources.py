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
