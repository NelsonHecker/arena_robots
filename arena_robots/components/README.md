# Component catalog

`components/<type>/<variant>/component.yaml`, one directory per `(type, variant)`,
loaded by [`arena_robots.catalog.Catalog`](../arena_robots/catalog.py). See that
module's docstring for the `component.yaml` schema, the sensor-template substitution
context, and the override-key vocabulary (`name`, `topic`).

Empty for now: no robot has migrated a sensor type yet (parametrized-robots.md sec2.8,
Stage M). This directory exists so the install rule in `CMakeLists.txt` has something to
install; the first entries land with the pilot robot's migration (rbtheron, per the
fit-sweep ranking).
