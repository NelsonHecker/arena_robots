"""Pure, launch-free parametrized-robot assembler. Resolves a robot's typed mounts +
declared defaults against a per-type part request into a concrete placement list via
maximum bipartite matching. No xacro/yaml rendering, no ROS: this module only decides
*what goes where*.
"""

from __future__ import annotations

import re
import typing

import attrs


class AssemblyError(RuntimeError):
    """Infeasible or malformed assembly request. The message is the product: name the
    offending type/mount and the robot's actual inventory, never a generic failure."""


_CHAINED_PARENT_RE = re.compile(r'^@(?P<mount>[^:]+):(?P<frame>.+)$')


def parse_chained_parent(parent: str) -> tuple[str, str] | None:
    """Parse a ``Mount.parent`` of the form ``"@<mount>:<frame>"`` (phase3b sec2):
    a parent chained through another mount's component-exported frame, rather than
    a literal chassis link name. Returns ``(mount_name, frame_name)``, or ``None``
    for an ordinary literal-frame parent."""
    m = _CHAINED_PARENT_RE.match(parent)
    return (m.group('mount'), m.group('frame')) if m else None


@attrs.define
class Mount:
    """A typed attachment slot declared by the chassis."""

    name: str
    parent: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accepts: frozenset[str] = attrs.field(factory=frozenset)

    @property
    def chained_parent(self) -> tuple[str, str] | None:
        return parse_chained_parent(self.parent)


@attrs.define
class DefaultPart:
    """One entry of a robot's declared default assembly; always pinned to a mount."""

    variant: str
    mount: str
    params: dict[str, object] = attrs.field(factory=dict)
    overrides: dict[str, str] = attrs.field(factory=dict)
    """Per-placement sensor-template overrides (catalog.py), e.g. rbtheron's rear
    lidar ``{name: lidar_rear, topic: scan/rear}``. Authoring-only: never flows from
    a fleet-def request, only from a robot's own ``assembly.yaml defaults:``."""


@attrs.define
class RequestPart:
    """One requested part instance from a fleet-def morphology item; ``mount`` set only
    for an explicit ``@pin``. ``params`` is reserved for later bracket-level tuning and is
    accepted but not propagated into the resolved assembly."""

    variant: str
    mount: str | None = None
    params: dict[str, object] = attrs.field(factory=dict)


def _validate_mount_dag(mounts: dict[str, Mount]) -> None:
    """Chained-parent references (phase3b sec2/3) must resolve to declared mounts
    and form a DAG; raises :class:`AssemblyError` naming the unknown mount or the
    cycle otherwise. Purely structural: independent of which mounts end up placed."""
    state: dict[str, int] = {}

    def visit(name: str, path: list[str]) -> None:
        if state.get(name) == 2:
            return
        if state.get(name) == 1:
            cycle = ' -> '.join([*path[path.index(name) :], name])
            raise AssemblyError(f"chained mount parents form a cycle: {cycle}")
        state[name] = 1
        chained = mounts[name].chained_parent
        if chained is not None:
            ref, _frame = chained
            if ref not in mounts:
                raise AssemblyError(f"mount '{name}' parent references unknown mount '{ref}'; declared mounts: {sorted(mounts)}")
            visit(ref, [*path, name])
        state[name] = 2

    for name in mounts:
        visit(name, [])


@attrs.define
class AssemblySpec:
    """Parsed ``assembly.yaml``: mounts + declared defaults."""

    mounts: dict[str, Mount] = attrs.field(factory=dict)
    defaults: dict[str, list[DefaultPart]] = attrs.field(factory=dict)
    prefix: str = 'robot_'
    """Frame-templating prefix for ``catalog.render_effective_sensors`` (mount frames
    render as ``${prefix}${mount}_link``). Defaults to ``robot_`` (the Robotnik-family
    convention rbtheron/rbrobout/rbvogui all share); a chassis with no such convention
    (mpo700) declares ``prefix: ""`` here to match its own xacro's ``prefix`` arg default."""

    @classmethod
    def parse(cls, data: dict[str, typing.Any]) -> AssemblySpec:
        """Parse the ``assembly.yaml`` shape (mounts/defaults keys, sec2.5).
        Validates that defaults reference declared mounts that accept their type;
        raises :class:`AssemblyError` otherwise. Allocation preference among mounts
        accepting the same type follows mount declaration order (first-declared wins)."""
        mounts_raw = data.get('mounts', {})
        if not isinstance(mounts_raw, dict):
            raise AssemblyError(f"assembly.yaml 'mounts' must be a mapping; got {type(mounts_raw).__name__}")

        mounts: dict[str, Mount] = {}
        for name, m in mounts_raw.items():
            xyz = m.get('xyz', [0.0, 0.0, 0.0])
            rpy = m.get('rpy', [0.0, 0.0, 0.0])
            mounts[name] = Mount(
                name=str(name),
                parent=str(m['parent']),
                xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
                accepts=frozenset(str(a) for a in m.get('accepts', [])),
            )
        _validate_mount_dag(mounts)

        defaults: dict[str, list[DefaultPart]] = {}
        for t, entries in data.get('defaults', {}).items():
            parts: list[DefaultPart] = []
            for entry in entries:
                mount_name = str(entry['mount'])
                if mount_name not in mounts:
                    raise AssemblyError(f"defaults['{t}']: unknown mount '{mount_name}'; declared mounts: {sorted(mounts)}")
                if t not in mounts[mount_name].accepts:
                    raise AssemblyError(
                        f"defaults['{t}']: mount '{mount_name}' does not accept '{t}' "
                        f"(accepts: {sorted(mounts[mount_name].accepts)})"
                    )
                parts.append(
                    DefaultPart(
                        variant=str(entry['variant']),
                        mount=mount_name,
                        params=dict(entry.get('params', {})),
                        overrides=dict(entry.get('overrides', {})),
                    )
                )
            defaults[str(t)] = parts

        return cls(mounts=mounts, defaults=defaults, prefix=str(data.get('prefix', 'robot_')))


@attrs.define
class Placement:
    """One resolved (type, variant) instance bound to a concrete mount."""

    type: str
    variant: str
    mount: Mount
    params: dict[str, object] = attrs.field(factory=dict)
    overrides: dict[str, str] = attrs.field(factory=dict)
    """Copied verbatim from the originating ``DefaultPart`` (empty for request-sourced
    placements); consumed only by ``catalog.render_effective_sensors``."""


@attrs.define
class ResolvedAssembly:
    placements: list[Placement] = attrs.field(factory=list)
    warnings: list[str] = attrs.field(factory=list)


@attrs.define(slots=True)
class _Item:
    """Internal: one part instance flattened out of defaults/request, pre-matching."""

    type: str
    variant: str
    mount: str | None
    params: dict[str, object]
    local_index: int
    overrides: dict[str, str] = attrs.field(factory=dict)


def _mounts_accepting(spec: AssemblySpec, t: str) -> list[str]:
    return [m.name for m in spec.mounts.values() if t in m.accepts]


def _candidates(item: _Item, mounts_pool: list[str], mounts: dict[str, Mount]) -> list[str]:
    """Mounts accepting ``item.type``, in mount declaration order (the allocation
    preference: first-declared wins)."""
    return [m for m in mounts_pool if item.type in mounts[m].accepts]


def _match(items: list[_Item], mounts_pool: list[str], mounts: dict[str, Mount]) -> tuple[dict[int, str], list[int]]:
    """Maximum bipartite matching via Kuhn's augmenting paths. Returns (item-index ->
    mount-name assignment, indices left unmatched). Complete: a perfect assignment of
    all ``items`` exists iff the returned unmatched list is empty."""
    mount_owner: dict[str, int] = {}

    def try_assign(idx: int, visited: set[str]) -> bool:
        for mount_name in _candidates(items[idx], mounts_pool, mounts):
            if mount_name in visited:
                continue
            visited.add(mount_name)
            if mount_name not in mount_owner or try_assign(mount_owner[mount_name], visited):
                mount_owner[mount_name] = idx
                return True
        return False

    unmatched: list[int] = []
    for idx in range(len(items)):
        if not try_assign(idx, set()):
            unmatched.append(idx)

    return {idx: mount for mount, idx in mount_owner.items()}, unmatched


def resolve(spec: AssemblySpec, request: dict[str, list[RequestPart]]) -> ResolvedAssembly:
    """Resolve a per-type part request against ``spec`` into concrete placements.

    Replace-on-touch (sec2.3): a type absent from ``request`` keeps ``spec.defaults``;
    a type present in ``request`` (even as ``[]``, the ``=none`` clear) discards its
    defaults entirely. Per-type gating (sec2.8): touching a type with no accepting
    mount anywhere is an error, except clearing it, which is a warning. Allocation is
    maximum bipartite matching over ALL parts of ALL types against ALL mounts jointly
    (sec2.6): ``@pin``s are fixed edges, checked first.
    """
    warnings: list[str] = []
    effective: dict[str, list[_Item]] = {}

    for t, default_parts in spec.defaults.items():
        if t in request:
            continue
        effective[t] = [
            _Item(type=t, variant=p.variant, mount=p.mount, params=p.params, overrides=p.overrides, local_index=i)
            for i, p in enumerate(default_parts)
        ]

    for t, requested in request.items():
        mounts_for_type = _mounts_accepting(spec, t)
        if not mounts_for_type:
            if len(requested) == 0:
                warnings.append(f"'{t}' cleared but robot declares no '{t}' mounts (no-op)")
                continue
            inventory = ", ".join(f"{m.name} accepts {sorted(m.accepts)}" for m in spec.mounts.values()) or "(no mounts declared)"
            raise AssemblyError(f"robot declares no '{t}' mounts; mount inventory: {inventory}")
        effective[t] = [_Item(type=t, variant=p.variant, mount=p.mount, params={}, local_index=i) for i, p in enumerate(requested)]

    flat: list[_Item] = [item for parts in effective.values() for item in parts]

    occupied: dict[str, _Item] = {}
    for item in flat:
        if item.mount is None:
            continue
        if item.mount not in spec.mounts:
            raise AssemblyError(f"unknown mount '{item.mount}' pinned for '{item.type}'; declared mounts: {sorted(spec.mounts)}")
        target = spec.mounts[item.mount]
        if item.type not in target.accepts:
            raise AssemblyError(f"mount '{item.mount}' does not accept '{item.type}' (accepts: {sorted(target.accepts)})")
        other = occupied.get(item.mount)
        if other is not None:
            raise AssemblyError(f"mount '{item.mount}' has two parts pinned to it: '{other.type}={other.variant}' and '{item.type}={item.variant}'")
        occupied[item.mount] = item

    pinned = [item for item in flat if item.mount is not None]
    unpinned = [item for item in flat if item.mount is None]
    mounts_pool = [name for name in spec.mounts if name not in occupied]

    assignment, unmatched = _match(unpinned, mounts_pool, spec.mounts)

    if unmatched:
        by_type: dict[str, list[int]] = {}
        for idx in unmatched:
            by_type.setdefault(unpinned[idx].type, []).append(idx)

        lines: list[str] = []
        for t in by_type:
            n_req = len(effective.get(t, []))
            inv = _mounts_accepting(spec, t)
            plural = "s" if len(inv) != 1 else ""
            lines.append(f"requested {n_req}x {t}, only {len(inv)} {t}-mount{plural} ({', '.join(inv)}); drop one or add a mount")
        for idx in unmatched:
            item = unpinned[idx]
            cands = _candidates(item, mounts_pool, spec.mounts)
            lines.append(f"{item.type}#{item.local_index} candidates: [{', '.join(cands)}]" if cands else f"{item.type}#{item.local_index} candidates: none")
        raise AssemblyError("; ".join(lines))

    unpinned_mounts = iter(assignment[i] for i in range(len(unpinned)))
    placements: list[Placement] = []
    for item in flat:
        mount_name = item.mount if item.mount is not None else next(unpinned_mounts)
        placements.append(
            Placement(
                type=item.type,
                variant=item.variant,
                mount=spec.mounts[mount_name],
                params=dict(item.params),
                overrides=dict(item.overrides),
            )
        )

    placed_mounts = {p.mount.name for p in placements}
    for p in placements:
        chained = p.mount.chained_parent
        if chained is not None:
            ref_mount, _frame = chained
            if ref_mount not in placed_mounts:
                raise AssemblyError(f"'{p.mount.name}' requires '{ref_mount}'")

    return ResolvedAssembly(placements=placements, warnings=warnings)


def warn_if_blind(resolved: ResolvedAssembly, required_types: set[str]) -> list[str]:
    """Caller-side helper: the assembler itself never hard-errors on "no
    lidar" (localization is ground-truth in Arena sim), but a bound adapter that needs an
    observation source can ask for a warning here."""
    present = {p.type for p in resolved.placements}
    return [f"no {t} placed: nav2 costmap will run without observation sources" for t in required_types if t not in present]
