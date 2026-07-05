"""Component catalog + sensor-template rendering. Launch-free and pure: loads
``components/<type>/<variant>/component.yaml`` and renders a robot's
:class:`arena_robots.assembly.ResolvedAssembly` into its effective gz ``SensorSpec``
list. No xacro loading, no ROS: this module only decides what a placement's sensor
declarations *say*, not how they get instantiated.

``component.yaml`` shape::

    xacro:
      include: <macro file, robot-urdf-dir-relative>
      macro: <macro name>
      attach: {...}           # attach-interface template (parent/xyz/rpy keys, mount-templated)
      args: {...}             # declared overridable macro-arg surface with defaults
    sensor:
      gz:
        - {name: <SensorSpec.name>, type: <SensorSpec.type>, topic: <relative topic>,
           frame: <SensorSpec.frame>, sensor: <backing gz <sensor> name>}
        # one entry per gz output the component contributes (a component is a
        # LIST of outputs, e.g. gpu_lidar -> laserscan + pointcloud)
    ros2_control:      # OPTIONAL (arm components only); absent -> []
      joints:
        - {name: <joint name template>, command_interfaces: [...], state_interfaces: [...]}
    control:           # OPTIONAL; absent -> {}
      controller: <controller name template, e.g. ${mount}_controller>
      type: <controller plugin type>
      ros__parameters: {...}   # templated verbatim
    caps:              # OPTIONAL; absent -> {}
      ...              # full caps/arm.yaml raw shape, templated verbatim (caps.py ArmSpec)
    frames:            # OPTIONAL; absent -> {} (phase3b sec1, mount-on-part chaining)
      <name>: <template>   # e.g. {top: "${prefix}${mount}_ewellix_lift_top_link"}

Every string value in a ``sensor.gz`` entry is a template rendered per-placement through
``arena_rclpy_mixins.yaml_replace.YAMLReplacer``. The substitution context is::

    {'mount': placement.mount.name, 'prefix': prefix, **placement.params, **placement.overrides}

i.e. the mount name and frame prefix are always available, a placement's resolved macro
``params`` are spread in next, and ``placement.overrides`` (assembly.py, authoring-only,
never from a fleet-def request) are spread in *last* so they win.

Override-key vocabulary (the minimal set a component author needs to reproduce an
asymmetric multi-instance robot like rbtheron's dual lidar):

- ``name``  overrides a placement's ``SensorSpec.name`` base (``${name:-<component-default>}``
  in the template); the component's own per-output literal suffix (e.g. ``_points``) still
  appends after it, so overriding once fixes every output of that placement consistently
  (``name: lidar_rear`` -> ``lidar_rear`` / ``lidar_rear_points``).
- ``topic`` overrides a placement's base relative topic (``${topic:-<component-default>}``);
  again the component's own per-output suffix (e.g. ``/points``) still appends
  (``topic: scan/rear`` -> ``scan/rear`` / ``scan/rear/points``).

``frame`` and ``sensor`` are templated directly off ``${mount}``/``${prefix}`` with no
override indirection: mount naming alone is expressive enough (mount frames adopt the
chassis's own frame names), so a component rarely needs more than the two keys above.
Components are, of course, free to reference ``${name:-...}``/``${topic:-...}`` inside
their ``frame``/``sensor`` templates too if a real robot needs it.
"""

from __future__ import annotations

import copy
import typing
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import quoteattr

import attrs
import yaml
from ament_index_python.packages import get_package_share_path
from arena_rclpy_mixins.yaml_replace import YAMLReplacer

from arena_robots.assembly import Mount, ResolvedAssembly
from arena_robots.Sensor import SensorSpec

if typing.TYPE_CHECKING:
    from arena_robots.Robot import RobotView


@attrs.define
class ComponentSpec:
    """One parsed ``components/<type>/<variant>/component.yaml``."""

    xacro_include: str
    xacro_macro: str
    attach: dict[str, str]
    args: dict[str, object]
    sensor_gz: list[dict[str, str]]
    """Ordered, un-rendered ``sensor.gz`` output templates; render per-placement via
    :func:`render_effective_sensors`, never mutate this list in place (shared across
    every placement of this component)."""
    ros2_control_joints: list[dict[str, object]] = attrs.field(factory=list)
    """Un-rendered ``ros2_control.joints`` entries; empty for
    components with no actuator axes (sensors). Render per-placement via
    :func:`render_wrapper_xacro`'s merged-tag synthesis."""
    control: dict[str, object] = attrs.field(factory=dict)
    """Un-rendered ``control`` block (``controller``/``type``/``ros__parameters``);
    empty for components with no controller. Render per-placement via
    :func:`render_effective_control`."""
    caps: dict[str, object] = attrs.field(factory=dict)
    """Un-rendered ``caps`` block, the full ``caps/arm.yaml`` raw shape; empty for
    components with no caps contribution. Rendered by ``RobotCaps``/``effective_caps``
    (caps.py), not this module."""
    frames: dict[str, str] = attrs.field(factory=dict)
    """Un-rendered ``frames`` templates (phase3b sec1): named frames this component
    exports, for another mount's ``Mount.parent`` to chain onto (e.g. a lift's
    ``top`` frame). Empty for components nothing else mounts onto. Resolved per
    placement by :func:`resolve_mount_parent`."""

    @classmethod
    def from_yaml(cls, path: Path) -> ComponentSpec:
        with open(path) as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError(f"component.yaml at {path} must be a mapping; got {type(data).__name__}")

        xacro = data.get('xacro', {})
        if not isinstance(xacro, dict) or 'include' not in xacro or 'macro' not in xacro:
            raise ValueError(f"component.yaml at {path}: 'xacro' must declare 'include' and 'macro'")

        sensor = data.get('sensor', {})
        gz = sensor.get('gz', []) if isinstance(sensor, dict) else []
        if not isinstance(gz, list):
            raise ValueError(f"component.yaml at {path}: 'sensor.gz' must be a list; got {type(gz).__name__}")

        ros2_control = data.get('ros2_control', {})
        joints = ros2_control.get('joints', []) if isinstance(ros2_control, dict) else []
        if not isinstance(joints, list):
            raise ValueError(f"component.yaml at {path}: 'ros2_control.joints' must be a list; got {type(joints).__name__}")

        control = data.get('control', {})
        if not isinstance(control, dict):
            raise ValueError(f"component.yaml at {path}: 'control' must be a mapping; got {type(control).__name__}")

        caps = data.get('caps', {})
        if not isinstance(caps, dict):
            raise ValueError(f"component.yaml at {path}: 'caps' must be a mapping; got {type(caps).__name__}")

        frames = data.get('frames', {})
        if not isinstance(frames, dict):
            raise ValueError(f"component.yaml at {path}: 'frames' must be a mapping; got {type(frames).__name__}")

        return cls(
            xacro_include=str(xacro['include']),
            xacro_macro=str(xacro['macro']),
            attach=dict(xacro.get('attach', {})),
            args=dict(xacro.get('args', {})),
            sensor_gz=[dict(entry) for entry in gz],
            ros2_control_joints=[dict(joint) for joint in joints],
            control=dict(control),
            caps=dict(caps),
            frames={str(k): str(v) for k, v in frames.items()},
        )


class Catalog:
    """``components/<type>/<variant>/component.yaml`` loader, cached per (type, variant).

    ``root`` defaults lazily to ``<arena_robots share>/components``; pass an explicit
    ``root`` (e.g. a ``tmp_path`` fixture tree) to keep tests off the ament index.
    """

    def __init__(self, root: Path | None = None) -> None:
        self._root = root if root is not None else get_package_share_path('arena_robots') / 'components'
        self._cache: dict[tuple[str, str], ComponentSpec] = {}

    def get(self, type_: str, variant: str) -> ComponentSpec:
        key = (type_, variant)
        if key not in self._cache:
            path = self._root / type_ / variant / 'component.yaml'
            if not path.is_file():
                type_dir = self._root / type_
                available = sorted(p.name for p in type_dir.iterdir() if p.is_dir()) if type_dir.is_dir() else []
                raise RuntimeError(f"component '{type_}/{variant}' not found; available '{type_}' variants: {available}")
            self._cache[key] = ComponentSpec.from_yaml(path)
        return self._cache[key]


def render_effective_sensors(resolved: ResolvedAssembly, catalog: Catalog, *, prefix: str = 'robot_') -> list[SensorSpec]:
    """Render every placement's ``sensor.gz`` templates into the robot's full effective
    ``SensorSpec`` list, in placement then per-component-output order. Each template dict
    is deep-copied before rendering (``YAMLReplacer.replace`` mutates its input in place;
    the source lives on the shared, cached :class:`ComponentSpec`)."""
    out: list[SensorSpec] = []
    for placement in resolved.placements:
        component = catalog.get(placement.type, placement.variant)
        context: dict[str, typing.Any] = {
            'mount': placement.mount.name,
            'prefix': prefix,
            **placement.params,
            **placement.overrides,
        }
        for template in component.sensor_gz:
            rendered = YAMLReplacer(context).replace(copy.deepcopy(template))
            out.append(
                SensorSpec(
                    name=str(rendered['name']),
                    type=str(rendered['type']),
                    topic='${namespace}/' + str(rendered['topic']),
                    frame=str(rendered['frame']),
                    sensor=str(rendered['sensor']) if 'sensor' in rendered else None,
                )
            )
    return out


def resolve_mount_parent(resolved: ResolvedAssembly, catalog: Catalog, mount: Mount) -> str:
    """Resolve a mount's ``parent`` for templating (phase3b sec2, chained mounts).

    An ordinary mount returns its literal parent frame name unchanged. A chained
    mount (``"@<mount>:<frame>"``) resolves to the referenced placement's rendered
    ``frames`` template, substituted with an EMPTY ``prefix`` so the caller's own
    ``${prefix}${parent}`` attach/caps template re-applies the real prefix exactly
    once (the same shape a literal parent already goes through)."""
    chained = mount.chained_parent
    if chained is None:
        return mount.parent
    ref_mount_name, frame_name = chained
    ref = next((p for p in resolved.placements if p.mount.name == ref_mount_name), None)
    if ref is None:
        raise RuntimeError(f"mount '{mount.name}' parent references unpopulated mount '{ref_mount_name}'")
    component = catalog.get(ref.type, ref.variant)
    if frame_name not in component.frames:
        raise RuntimeError(
            f"component '{ref.type}/{ref.variant}' (mount '{ref_mount_name}') does not export frame "
            f"'{frame_name}'; declared frames: {sorted(component.frames)}"
        )
    context: dict[str, typing.Any] = {'mount': ref.mount.name, 'prefix': '', **ref.params, **ref.overrides}
    return YAMLReplacer(context).replace(component.frames[frame_name])


_XACRO_NS = 'http://www.ros.org/wiki/xacro'


def _chassis_args(chassis_path: Path) -> list[tuple[str, str]]:
    """``(name, default)`` for every top-level ``<xacro:arg>`` in the chassis xacro, in
    document order. The wrapper forwards exactly this surface.

    Matched by local tag name against direct children of ``<robot>`` only: chassis
    files across the fleet declare ``xmlns:xacro`` with at least four different URIs
    (``ros.org/wiki/xacro``, ``wiki.ros.org/xacro``, ``www.ros.org/wiki/xacro``, and a
    typo'd ``wiki.ros.org.xacro``), all functionally equivalent to the ``xacro`` CLI, so
    a fully-qualified-name match would miss most of the fleet. Direct-children-only
    (rather than a full subtree ``.iter()``) also skips non-top-level ``<xacro:arg>``
    elements some chassis files carry as inert children of an unrelated tag (e.g.
    turtlebot's ``<xacro:include>...<xacro:arg name="gazebo" .../></xacro:include>``,
    which xacro itself ignores with a warning)."""
    root = ET.parse(chassis_path).getroot()
    return [(el.get('name', ''), el.get('default', '')) for el in root if el.tag.rsplit('}', 1)[-1] == 'arg']


def _joint_xml(joint: dict[str, typing.Any]) -> list[str]:
    """Rendered ``<joint>`` element lines for one ``ros2_control.joints`` entry."""
    lines = [f'    <joint name={quoteattr(str(joint["name"]))}>']
    for iface in joint.get('command_interfaces', []):
        lines.append(f'      <command_interface name={quoteattr(str(iface))}/>')
    for iface in joint.get('state_interfaces', []):
        lines.append(f'      <state_interface name={quoteattr(str(iface))}/>')
    lines.append('    </joint>')
    return lines


def render_wrapper_xacro(view: RobotView, resolved: ResolvedAssembly, *, catalog: Catalog | None = None) -> str:
    """Generate a complete wrapper ``.urdf.xacro`` document for ``view``: forwards the
    chassis's own ``xacro:arg`` surface, includes the chassis xacro, then includes +
    invokes each placement's component macro at its mount's parent/origin.
    Deterministic: placements render in ``resolved.placements`` order
    (assembly.yaml defaults order); a component's macro file is included once even if
    placed at multiple mounts.

    The attach-template substitution context mirrors :func:`render_effective_sensors`
    (``mount``/``prefix``/``params``/``overrides``) plus xacro-side-only keys
    (``parent``, ``namespace``, ``gazebo_classic``, ``gazebo_ignition``) whose values are
    literal ``$(arg ...)`` references, left for the actual xacro run to resolve.
    ``parent`` is resolved via :func:`resolve_mount_parent` (phase3b sec2): an ordinary
    mount's literal parent name passes through; a chained mount (``"@<mount>:<frame>"``)
    resolves to the referenced placement's rendered frame instead.

    When any placement's component declares ``ros2_control.joints`` (phase3 sec2.10,
    merged ros2_control), one merged ``<ros2_control name="${robot}_system">`` tag is
    synthesized: the chassis's joints-only ``${robot}_base_hw_joints`` macro (contract
    with the base_hw refactor: included from ``urdf/base_hw/<robot>.ros2_control.urdf``,
    takes a ``prefix`` arg) plus every placement's rendered ``<joint>`` elements. When no
    placement declares joints, nothing is emitted here: the chassis xacro keeps handling
    its own tag exactly as it does today.
    """
    catalog = catalog if catalog is not None else Catalog()
    chassis_path = view.path / 'urdf' / f'{view.name}.urdf.xacro'
    args = _chassis_args(chassis_path)

    body_lines: list[str] = []
    included: set[str] = set()
    control_joint_lines: list[str] = []
    for placement in resolved.placements:
        component = catalog.get(placement.type, placement.variant)
        # package-qualified ($(find ...)) and absolute includes pass through verbatim,
        # everything else resolves relative to the robot's urdf/ tree
        if component.xacro_include.startswith('$(') or Path(component.xacro_include).is_absolute():
            macro_ref = component.xacro_include
        else:
            macro_ref = str(view.path / 'urdf' / component.xacro_include)
        if macro_ref not in included:
            body_lines.append(f'  <xacro:include filename={quoteattr(macro_ref)}/>')
            included.add(macro_ref)

        context: dict[str, typing.Any] = {
            'mount': placement.mount.name,
            'parent': resolve_mount_parent(resolved, catalog, placement.mount),
            'prefix': '$(arg prefix)',
            'namespace': '$(arg namespace)',
            'gazebo_classic': '$(arg gazebo_classic)',
            'gazebo_ignition': '$(arg gazebo_ignition)',
            **placement.params,
            **placement.overrides,
        }
        attach = YAMLReplacer(context).replace(copy.deepcopy(component.attach))
        attrs_str = ' '.join(f'{k}={quoteattr(str(v))}' for k, v in attach.items())
        x, y, z = placement.mount.xyz
        r, p, yw = placement.mount.rpy

        body_lines.append(f'  <xacro:{component.xacro_macro} {attrs_str}>')
        body_lines.append(f'    <origin xyz="{x} {y} {z}" rpy="{r} {p} {yw}"/>')
        body_lines.append(f'  </xacro:{component.xacro_macro}>')

        if component.ros2_control_joints:
            joints = YAMLReplacer(context).replace(copy.deepcopy(component.ros2_control_joints))
            for joint in joints:
                control_joint_lines.extend(_joint_xml(joint))

    if control_joint_lines:
        # the merged tag replaces the chassis's internal one, the chassis must gate it
        if 'generate_ros2_control_tag' not in {name for name, _ in args}:
            raise RuntimeError(
                f"{chassis_path}: chassis must declare a 'generate_ros2_control_tag' xacro:arg "
                'to gate its internal ros2_control tag before joint-bearing parts can be mounted'
            )
        args = [(n, 'false' if n == 'generate_ros2_control_tag' else d) for n, d in args]

    lines = ['<?xml version="1.0"?>', f'<robot name="{view.name}" xmlns:xacro="{_XACRO_NS}">']
    for name, default in args:
        lines.append(f'  <xacro:arg name={quoteattr(name)} default={quoteattr(default)}/>')
    lines.append(f'  <xacro:include filename={quoteattr(str(chassis_path))}/>')
    lines.extend(body_lines)

    if control_joint_lines:
        base_hw_path = view.path / 'urdf' / 'base_hw' / f'{view.name}.ros2_control.urdf'
        lines.append(f'  <ros2_control name="{view.name}_system" type="system">')
        lines.append('    <hardware>')
        lines.append('      <plugin>gz_ros2_control/GazeboSimSystem</plugin>')
        lines.append('    </hardware>')
        lines.append(f'    <xacro:include filename={quoteattr(str(base_hw_path))}/>')
        lines.append(f'    <xacro:{view.name}_base_hw_joints prefix="$(arg prefix)"/>')
        lines.extend(control_joint_lines)
        lines.append('  </ros2_control>')

    lines.append('</robot>')
    return '\n'.join(lines) + '\n'


def render_effective_control(
    resolved: ResolvedAssembly, base_control: dict, catalog: Catalog, *, prefix: str = 'robot_'
) -> tuple[dict, list[str]]:
    """Deep-merge every placement's rendered ``control`` block into a copy of the
    chassis's ``control.yaml`` (control synthesis): one
    ``controller_manager.ros__parameters.<controller>: {type: ...}`` entry plus one
    top-level ``<controller>: {ros__parameters: ...}`` section per placement that
    declares a ``control`` block. Returns ``(merged, extra_controller_names)``; ``base_control``
    is not mutated. Same templating context as :func:`render_effective_sensors`."""
    merged = copy.deepcopy(base_control)
    cm_params = merged.setdefault('controller_manager', {}).setdefault('ros__parameters', {})
    extra_controllers: list[str] = []
    for placement in resolved.placements:
        component = catalog.get(placement.type, placement.variant)
        if not component.control:
            continue
        context: dict[str, typing.Any] = {
            'mount': placement.mount.name,
            'prefix': prefix,
            **placement.params,
            **placement.overrides,
        }
        rendered = YAMLReplacer(context).replace(copy.deepcopy(component.control))
        controller_name = str(rendered['controller'])
        cm_params[controller_name] = {'type': str(rendered['type'])}
        merged[controller_name] = {'ros__parameters': rendered.get('ros__parameters', {})}
        extra_controllers.append(controller_name)
    return merged, extra_controllers
