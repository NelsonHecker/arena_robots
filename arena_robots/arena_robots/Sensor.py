"""Arena-native sensor declarations shared by robots and adapters."""

from __future__ import annotations

import enum

import attrs


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


SensorTypeOrStr = SensorType | str
