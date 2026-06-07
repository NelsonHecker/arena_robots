"""Canonical sensor-type vocabulary shared across arena_robots and arena_planners."""

from __future__ import annotations

SENSOR_TYPES: frozenset[str] = frozenset(
    {
        "laserscan",
        "pointcloud",
        "image",
        "depth",
        "imu",
        "contact",
        "camera_info",
    }
)
