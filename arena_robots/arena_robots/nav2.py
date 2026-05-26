"""Nav2 launch helpers."""

import tempfile
import typing
from pathlib import Path

import launch
import yaml
from arena_bringup.substitutions import YAMLFileSubstitution

from arena_robots.caps import MobileSpec, stringify_float_matrix
from arena_robots.Robot import ModelParams
from arena_robots.Sensor import SensorSpec, SensorType

_TYPE_TO_NAV2: dict[str, str] = {
    SensorType.LASERSCAN.value: "LaserScan",
    SensorType.POINTCLOUD.value: "PointCloud2",
}


def compile_sensors_to_nav2(
    sensors: list[SensorSpec],
    *,
    max_obstacle_height: float = 2.0,
    clearing: bool = True,
    marking: bool = True,
) -> dict[str, dict[str, typing.Any]]:
    """Compile SensorSpec entries with a nav2 costmap data_type into nav2's observation_sources_dict shape."""
    out: dict[str, dict[str, typing.Any]] = {}
    for spec in sensors:
        type_str = spec.type.value if isinstance(spec.type, SensorType) else str(spec.type)
        data_type = _TYPE_TO_NAV2.get(type_str)
        if data_type is None:
            continue
        out[spec.name] = {
            "topic": spec.topic,
            "data_type": data_type,
            "max_obstacle_height": max_obstacle_height,
            "clearing": clearing,
            "marking": marking,
        }
    return out


def _load_mobile(path_str: str) -> MobileSpec:
    with open(path_str) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path_str}: mobile.yaml must be a mapping at top level")
    return MobileSpec(path=Path(path_str), raw=data)


class SensorsDerivedYAML(YAMLFileSubstitution):
    """Emit a temp YAML file with `observation_sources{,_string,_dict}` derived
    from the `sensors:` block of model_params.yaml. Keeps the three nav2 costmap
    forms in sync from one source."""

    def __init__(self, model_params_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(model_params_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        sensors = ModelParams.from_yaml(path_str).sensors
        sources = compile_sensors_to_nav2(sensors)
        derived = {
            'observation_sources_string': ' '.join(sources.keys()),
            'observation_sources': list(sources.keys()),
            'observation_sources_dict': sources,
        }
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(derived, tmp)
        tmp.close()
        return tmp.name


class Nav2SubBlockYAML(YAMLFileSubstitution):
    """Extract the `nav2:` sub-block from caps/mobile.yaml and emit it at top level
    as a temp YAML file. Lets YAMLMergeSubstitution treat adapter-specific config
    (footprint, polygons*, planner_plugins*) as flat merge-time keys while they
    stay nested in the authored file."""

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        raw = _load_mobile(path_str).sub('nav2')
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(raw, tmp)
        tmp.close()
        return tmp.name


class Nav2KinematicsDerivedYAML(YAMLFileSubstitution):
    """Emit controller-agnostic velocity/acceleration keys from the top-level
    ``velocity_limits``/``acceleration_limits`` in caps/mobile.yaml.

    These are the planner envelope (what nav2 may sample), not the hardware
    envelope (motor firmware / ``diff_drive_controller`` clip downstream).

    Controller plugin configs reference these via ``${max_linear_vel}`` etc.,
    letting each controller map the generic envelope onto its plugin-specific
    field names. Controllers wanting a lower cap can drop the ``${...}`` ref
    and hardcode the literal instead.
    """

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile = _load_mobile(path_str)

        out: dict[str, typing.Any] = {}
        vel = mobile.velocity_limits
        if vel is not None:
            out['min_linear_vel'] = vel.linear.min
            out['max_linear_vel'] = vel.linear.max
            out['min_angular_vel'] = vel.angular.min
            out['max_angular_vel'] = vel.angular.max
            if vel.lateral is not None:
                out['min_lateral_vel'] = vel.lateral.min
                out['max_lateral_vel'] = vel.lateral.max

        acc = mobile.acceleration_limits
        if acc is not None:
            out['linear_acc'] = acc.linear
            out['angular_acc'] = acc.angular
            out['linear_decel'] = -acc.linear
            out['angular_decel'] = -acc.angular
            if acc.lateral is not None:
                out['lateral_acc'] = acc.lateral
                out['lateral_decel'] = -acc.lateral

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(out if out else {}, tmp)
        tmp.close()
        return tmp.name


class Nav2CollisionDerivedYAML(YAMLFileSubstitution):
    """Compile top-level `footprint` and `polygons_dict` from caps/mobile.yaml
    into the stringified form nav2's collision_monitor expects, overriding any
    raw float lists emitted by the preceding YAMLFileSubstitution(mobile_path)."""

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile = _load_mobile(path_str)
        raw = mobile.raw

        out: dict[str, typing.Any] = {}

        footprint_raw = raw.get('footprint')
        if isinstance(footprint_raw, list):
            out['footprint'] = stringify_float_matrix([[float(c) for c in pt] for pt in footprint_raw])

        polygons_raw = raw.get('polygons_dict')
        if isinstance(polygons_raw, dict) and polygons_raw:
            out['polygons'] = list(polygons_raw.keys())
            compiled: dict[str, typing.Any] = {}
            for name, entry in polygons_raw.items():
                ptype = entry.get('type')
                polygon_entry: dict[str, typing.Any] = {}
                for field in ('type', 'action_type', 'polygon_pub_topic', 'min_points', 'visualize', 'enabled', 'slowdown_ratio'):
                    if field in entry:
                        polygon_entry[field] = entry[field]
                if ptype == 'polygon':
                    pts = entry.get('points')
                    if isinstance(pts, list):
                        polygon_entry['points'] = stringify_float_matrix([[float(c) for c in pt] for pt in pts])
                    else:
                        polygon_entry['points'] = pts
                elif ptype == 'circle':
                    if 'radius' in entry:
                        polygon_entry['radius'] = entry['radius']
                compiled[name] = polygon_entry
            out['polygons_dict'] = compiled

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(out if out else {}, tmp)
        tmp.close()
        return tmp.name
