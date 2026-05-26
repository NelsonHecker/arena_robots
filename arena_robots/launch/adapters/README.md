# Bringup adapter launch files

This directory contains per-bringup-kind launch files. They are implementation
details of `arena_robots.bringup.*` classes — not public entry points.

The public entry point is [`launch/bringup.launch.py`](../bringup.launch.py).
Users invoke that file; it selects and includes the appropriate file here based
on the `bringup` argument.

## Files

### `nav2.launch.py`

Internals of `arena_robots.bringup.nav2.Nav2Bringup`. Instantiates a full nav2
stack (map server, AMCL, planner, controller, bt_navigator, lifecycle manager)
in the given namespace. Accepts launch arguments for planner selection
(`global_planner`, `local_planner`, `inter_planner`), sim time, costmap frame,
and an optional `task_generator_node` name for map topic remapping (empty
string = no remap, which is the standalone default). Planner selection is
typically driven from
[cap-scoped overrides](../../../../arena_bringup/BRINGUP.md#cap-scoped-overrides)
at launch (`mobile.local_planner:=teb`, etc.) which the task-generator
forwards to this launch file.

### `none.launch.py`

Internals of `arena_robots.bringup.none.NoneBringup`. Spins up no navigation
stack. The `task_server` will publish goals directly to a goal-pose topic.
Use this for robots driven by an external planner that subscribes to a goal
topic.

## Adding a new bringup kind

Create `<kind>.launch.py` here, then add a `Bringup` subclass in
`arena_robots/arena_robots/arena_robots/bringup/<cap>/<kind>.py` whose
`_launch_actions()` includes this file. Implement handlers under
`arena_robots/arena_robots/arena_robots/task_server_handlers/<task_kind>/`
and declare them on the `Bringup` subclass via a `task_handlers: ClassVar`
mapping `TaskKind` to a zero-arg loader function. The `task_server` reads
that mapping directly, no central registry to update.
