# Component catalog

`components/<type>/<variant>/component.yaml`, one directory per `(type, variant)`,
loaded by [`arena_robots.catalog.Catalog`](../arena_robots/catalog.py). See that
module's docstring for the authoritative `component.yaml` schema and the
sensor-template substitution context.

## Schema at a glance

```yaml
xacro:                    # how the wrapper invokes this component
  include: <path>         # robot-tree-relative, OR "$(find pkg)/..." / absolute (passed verbatim)
  macro: <macro name>     # must accept an <origin> block param (*origin)
  attach: {...}           # macro args; ${mount}/${parent}/${prefix}/${key:-default} templated
  args: {...}             # declared overridable arg surface with defaults (documentation)
sensor:
  gz:                     # LIST of bridged outputs (one component may emit several)
    - name: "${name:-lidar}"
      type: laserscan     # BRIDGE_TYPES in Sensor.py drive derived bridge rows
      topic: "${topic:-scan}"
      frame: "${prefix}${mount}_link"
ros2_control:             # joint-bearing parts only (arms): merged-tag contribution
  joints: [...]
control: {...}            # controller block deep-merged into the chassis control.yaml
caps: {...}               # full caps/<cap>.yaml shape, rendered per placement
```

## Conventions

- **Variants encode capability, not SKU** (`d435_color` vs a hypothetical `d435_rgbd`).
- **Include paths**: a robot-tree-relative `include` only resolves for the robot that owns
  the macro file; any component shared across robots must use a package-qualified
  `$(find arena_robots)/...` include.
- **Origin contract**: the wrapper appends an `<origin>` block to every invocation. Macros
  taking scalar offsets instead (the `arena_default_*` family) get a small `*origin` shim
  macro in the component dir; the original macro files are never modified.
- **Multi-output components**: `sensor.gz` is a list, one macro may contribute several
  sensors (e.g. an RGBD camera bundling an IMU).
- **Sim-only sensors** (rendered under gz but unbridged, e.g. GPS/navsat): the macro renders
  them, `sensor.gz` simply omits them; no SensorSpec, no bridge row.
- **Arm components** contribute four artifacts (`xacro`, `ros2_control.joints`, `control`,
  `caps`) and require the chassis to gate its internal ros2_control tag behind a
  `generate_ros2_control_tag` xacro:arg (the wrapper flips it when it synthesizes the
  merged tag; an ungated chassis raises).
- Per-instance tuning lives in `assembly.yaml defaults[].params/overrides`, never in the
  request grammar (parametrized-robots spec sec4).
