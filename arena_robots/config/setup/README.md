# Robot setup files

A robot setup file (`config/setup/<name>.yaml`) is a YAML list declaring which
robot instances to spawn for a given scenario. Resolved at runtime by
[`RobotSetupIdentifier`](../../arena_robots/SetupFile.py) — `RobotSetupIdentifier('demo')`
loads `demo.yaml` and returns a flat `list[Config]` (one per instance, with
`count` already expanded).

## Schema

Each list entry is either a **string** (shorthand) or a **dict** (full form).
Both resolve into one or more [`Config`](../../arena_robots/SetupFile.py) objects.

### Shorthand — one instance of the named robot

```yaml
- jackal
- turtlebot
```

Equivalent to `{ robot: jackal, name: jackal }` followed by
`{ robot: turtlebot, name: turtlebot }`.

### Dict form — full control

```yaml
- robot: jackal        # required: directory name under robots/<robot>/
  name: jackal_lead    # optional: instance name / namespace prefix (default: unset)
  count: 3             # optional: expand into N identical Configs (default: 1)
  planner: SmacPlanner # optional: nav2 planner override
  controller: MPPI     # optional: nav2 controller override
  behavior: default    # optional: nav2 behavior tree override
  mobile: nav2         # optional: mobile adapter kind (overrides robot.mobile_adapter)
  arm: moveit          # optional: arm adapter kind (overrides robot.arm_adapter)
  extra:               # optional: free-form pass-through dict
    foo: bar
```

`count` is consumed during parsing and never appears on the resulting
`Config` — set `count: 3` and you get three identical instances, each with
the same `name` field. If you need distinct names per instance, emit
multiple list entries instead of using `count`.

## Examples shipped with arena_robots

- [`all_robots.yaml`](all_robots.yaml) — one of each known robot using
  shorthand; useful for smoke-testing every URDF and mapping file.
- [`demo.yaml`](demo.yaml) — three jackals via `count: 3`.
