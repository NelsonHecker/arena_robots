"""Arena-native sensor declarations shared by robots and adapters."""

from __future__ import annotations

import enum
import sys
from collections.abc import Sequence

import attrs

from arena_robots.sensors import output_topics


class SensorType(enum.StrEnum):
    """Canonical sensor-type vocabulary for the Arena-native sensors schema."""

    LASERSCAN = "laserscan"
    POINTCLOUD = "pointcloud"
    IMAGE = "image"
    DEPTH = "depth"
    CAMERA_INFO = "camera_info"
    IMU = "imu"
    CONTACT = "contact"


@attrs.define(frozen=True)
class SensorSpec:
    """Arena-native sensor declaration."""

    name: str
    type: SensorType | str
    topic: str
    frame: str
    sensor: str | None = None
    """Name of the backing URDF <sensor> element, used for topic injection at load time."""


SensorTypeOrStr = SensorType | str

_NAMESPACE_PLACEHOLDER = '${namespace}'


def topic_elements(sensors: Sequence[SensorSpec], prefix: str) -> list[tuple[str, str, str]]:
    """Element patches `(sensor name, child element path, value)` for URDF topic injection.

    `prefix` is the per-instance gz topic scope, empty outside gazebo.
    """
    grouped: dict[str, dict[str, str]] = {}
    for spec in sensors:
        if spec.sensor is None:
            continue
        rel = spec.topic.removeprefix(_NAMESPACE_PLACEHOLDER).lstrip('/')
        grouped.setdefault(spec.sensor, {})[str(spec.type)] = rel

    patches: list[tuple[str, str, str]] = []
    for sensor_name, outputs in grouped.items():
        if 'laserscan' in outputs:
            sensor_type, anchor = 'gpu_lidar', outputs['laserscan']
        elif 'image' in outputs and ('depth' in outputs or 'pointcloud' in outputs):
            sensor_type, anchor = 'rgbd_camera', outputs['image'].removesuffix('/image')
            if anchor == outputs['image']:
                print(f"[sensor_topics] {sensor_name!r}: rgbd image topic {outputs['image']!r} does not end in /image, skipping", file=sys.stderr)
                continue
        elif 'image' in outputs:
            sensor_type, anchor = 'camera', outputs['image']
        elif 'depth' in outputs:
            sensor_type, anchor = 'depth', outputs['depth']
        elif 'imu' in outputs:
            sensor_type, anchor = 'imu', outputs['imu']
        else:
            print(f"[sensor_topics] {sensor_name!r}: no anchor topic among {sorted(outputs)}", file=sys.stderr)
            continue

        derived = output_topics(sensor_type, anchor, outputs.get('camera_info'))
        assert derived is not None
        for kind, topic in outputs.items():
            if kind in derived and derived[kind] != topic:
                print(f"[sensor_topics] {sensor_name!r}: {kind} topic {topic!r} is not {derived[kind]!r}, gz publishes the latter", file=sys.stderr)

        patches.append((sensor_name, 'topic', f'{prefix}/{anchor}'))
        if 'camera_info' in outputs and sensor_type in ('camera', 'depth', 'rgbd_camera'):
            patches.append((sensor_name, 'camera/camera_info_topic', f"{prefix}/{outputs['camera_info']}"))
    return patches
