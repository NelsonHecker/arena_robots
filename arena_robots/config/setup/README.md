# Robot setup files

A robot setup file (`config/setup/<name>.yaml`) is a YAML list declaring which
robot instances to spawn for a given scenario. Resolved at runtime by
[`RobotSetupIdentifier`](../../arena_robots/SetupFile.py): `RobotSetupIdentifier('demo')`
loads `demo.yaml` and returns a flat `list[Config]` (one per instance, with
`count` already expanded).

## Schema

Each list entry is either a **string** (shorthand) or a **dict** (full form).
Both resolve into one or more [`Config`](../../arena_robots/SetupFile.py) objects.

### Shorthand: one instance of the named robot

```yaml
- jackal
- turtlebot
```

Equivalent to `{ robot: jackal, name: jackal }` followed by
`{ robot: turtlebot, name: turtlebot }`.

### Dict form: full control

Parsed by [`Config.parse`](../../arena_robots/SetupFile.py). Keys route by
grammar, not a fixed field list:

```yaml
- robot: jackal            # required: directory name under robots/<robot>/
  name: jackal_lead        # optional: instance name / namespace prefix (default: unset, see below)
  count: 3                 # optional: expand into N independent Configs (default: 1)
  pos: [1.0, 2.0, 0.0]     # optional: initial pose [x, y, yaw], consumed by Robot.parse (default: (0, 0, 0))
  adapters:                # optional: cap -> adapter kind
    mobile: nav2
    arm: moveit
  mobile.adapter: nav2     # optional: dotted shorthand, equivalent to the adapters: block above
  frames:                  # optional: mount -> frame stem override, validated against the robot's assembly mounts
    top: custom_laser_link
  extra:                   # optional: dict merged verbatim into the parsed robot's extra data
    foo: bar
  top: lidar/sick          # any other bare key is a morphology directive (mount=type/variant, mount=variant, type=variant, or lhs=none), requires the robot to have an assembly.yaml
```

`count` is consumed during parsing and never appears on the resulting
`Config`: set `count: 3` and you get three independent instances (each with
its own deep-copied `extra`/`parts`/`frames`), all sharing the same `name`
field. If you need distinct names per instance, emit multiple list entries
instead of using `count`.

`name` omitted (dict form) leaves the instance anonymous: robots_manager
groups anonymous instances by robot model and assigns `<robot>` or, when more
than one anonymous instance of the same robot exists, `<robot>_0`,
`<robot>_1`, etc. The shorthand string form (`- jackal`) always sets an
explicit `name` equal to the robot name, so it is never anonymous.

`robot`, `name`, `count`, `parts`, `frames`, `adapters`, `extra` are reserved
field names and cannot be reused as morphology directive keys. A top-level
`parts:` dict is rejected for the same reason: directives must be given as
individual bare keys. The only accepted dotted key form is
`<cap>.adapter: <kind>`, any other dotted key raises. `adapters` values are
validated against the registered adapter kinds per cap (`mobile`:
`nav2`, `external`, `manual`, `rosnav_rl`, `drl`, `none`, `test-collision`,
`arm`: `moveit`, `none`, see
[tasks/robots/adapters](../../../../task_generator/task_generator/tasks/robots/adapters/README.md)).
Morphology directives and `frames` are only valid for a robot whose
`assembly.yaml` defines mounts. Both raise if the model has none.

Nav2 planner/controller/behavior-tree tuning is not a robot setup file key:
it is selected at launch time (`robot.planner:=`, see
[arena_bringup/launch/README.md](../../../../arena_bringup/launch/README.md)).

## Examples shipped with arena_robots

- [`all_robots.yaml`](all_robots.yaml): one of each known robot using
  shorthand; useful for smoke-testing every URDF and mapping file.
- [`demo.yaml`](demo.yaml): three jackals via `count: 3`.
