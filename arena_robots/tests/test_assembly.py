"""Tests for arena_robots.assembly, the pure parametrized-robots resolver
(.claude/parametrized-robots.md sec2.3, sec2.5, sec2.6, sec2.8;
parametrized-robots-fitsweep.md sec4)."""

from __future__ import annotations

import pytest
from arena_robots.assembly import Mount


def _mount(
    name: str,
    accepts: list[str],
    parent: str = "base_link",
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Mount:
    return Mount(name=name, parent=parent, xyz=xyz, rpy=rpy, accepts=frozenset(accepts))


class TestResolveDefaults:
    def test_default_only_resolve_uses_declared_defaults(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            priority={"lidar": ["top"]},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        assert len(resolved.placements) == 1
        p = resolved.placements[0]
        assert (p.type, p.variant, p.mount.name) == ("lidar", "sick_s300", "top")
        assert resolved.warnings == []

    def test_untouched_type_keeps_defaults(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["camera"])},
            defaults={
                "lidar": [DefaultPart(variant="sick_s300", mount="top")],
                "camera": [DefaultPart(variant="d435", mount="front")],
            },
        )
        resolved = resolve(spec, {"lidar": [RequestPart(variant="vlp16")]})
        by_type = {p.type: p.variant for p in resolved.placements}
        assert by_type["lidar"] == "vlp16"
        assert by_type["camera"] == "d435"

    def test_touched_type_discards_defaults(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {"lidar": [RequestPart(variant="vlp16")]})
        variants = [p.variant for p in resolved.placements]
        assert variants == ["vlp16"]


class TestResolveClear:
    def test_explicit_clear_removes_all_instances_of_type(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {"lidar": []})
        assert resolved.placements == []
        assert resolved.warnings == []

    def test_clear_of_mountless_type_warns_not_errors(self):
        from arena_robots.assembly import AssemblySpec, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"arm": []})
        assert resolved.placements == []
        assert len(resolved.warnings) == 1
        assert "arm" in resolved.warnings[0]

    def test_nonempty_request_of_mountless_type_errors(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="arm"):
            resolve(spec, {"arm": [RequestPart(variant="panda")]})


class TestResolveAppend:
    def test_two_request_parts_accumulate(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick"), RequestPart(variant="sick")]})
        assert len(resolved.placements) == 2
        assert {p.mount.name for p in resolved.placements} == {"top", "front"}


class TestResolvePins:
    def test_pin_success(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick", mount="front")]})
        assert resolved.placements[0].mount.name == "front"

    def test_pin_to_unknown_mount_raises_and_lists_declared(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="top"):
            resolve(spec, {"lidar": [RequestPart(variant="sick", mount="nope")]})

    def test_pin_to_mount_not_accepting_type_raises(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"front": _mount("front", ["camera"]), "top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="does not accept"):
            resolve(spec, {"lidar": [RequestPart(variant="sick", mount="front")]})

    def test_two_parts_pinned_to_one_mount_raises(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="two parts pinned"):
            resolve(spec, {"lidar": [RequestPart(variant="a", mount="top"), RequestPart(variant="b", mount="top")]})


class TestResolveMatching:
    def test_greedy_trap_resolved_by_augmenting_paths(self):
        """camera only fits A; lidar fits A or B. Processed camera-then-lidar, a
        first-come allocator would burn A on camera's greedy pick and strand lidar;
        matching must still land both."""
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"A": _mount("A", ["lidar", "camera"]), "B": _mount("B", ["lidar"])},
            priority={"lidar": ["A", "B"], "camera": ["A"]},
        )
        request = {"camera": [RequestPart(variant="d435")], "lidar": [RequestPart(variant="sick")]}
        resolved = resolve(spec, request)
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type["camera"] == "A"
        assert by_type["lidar"] == "B"

    def test_matching_reassigns_earlier_pick_when_needed(self):
        """Reverse order: lidar processed first, greedily prefers A; matcher must
        reassign it to B once camera (A-only) needs the mount."""
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"A": _mount("A", ["lidar", "camera"]), "B": _mount("B", ["lidar"])},
            priority={"lidar": ["A", "B"], "camera": ["A"]},
        )
        request = {"lidar": [RequestPart(variant="sick")], "camera": [RequestPart(variant="d435")]}
        resolved = resolve(spec, request)
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type == {"lidar": "B", "camera": "A"}

    def test_priority_preference_respected_when_slack_exists(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])},
            priority={"lidar": ["front", "top"]},
        )
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick")]})
        assert resolved.placements[0].mount.name == "front"

    def test_cross_type_contention_over_single_shared_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"A": _mount("A", ["lidar", "camera"])})
        with pytest.raises(AssemblyError):
            resolve(spec, {"lidar": [RequestPart(variant="sick")], "camera": [RequestPart(variant="d435")]})

    def test_over_capacity_failure_message_content(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])})
        request = {"lidar": [RequestPart(variant="sick") for _ in range(3)]}
        with pytest.raises(AssemblyError) as excinfo:
            resolve(spec, request)
        msg = str(excinfo.value)
        assert "3x lidar" in msg
        assert "2 lidar-mount" in msg
        assert "top" in msg and "front" in msg
        assert "drop one or add a mount" in msg


class TestResolveParams:
    def test_default_params_flow_into_placement(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top", params={"max_angle": 2.1})]},
        )
        resolved = resolve(spec, {})
        assert resolved.placements[0].params == {"max_angle": 2.1}

    def test_request_params_are_not_propagated(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick", params={"max_angle": 1.0})]})
        assert resolved.placements[0].params == {}


class TestWarnIfBlind:
    def test_warns_when_required_type_absent(self):
        from arena_robots.assembly import ResolvedAssembly, warn_if_blind

        resolved = ResolvedAssembly(placements=[])
        warnings = warn_if_blind(resolved, {"lidar"})
        assert len(warnings) == 1
        assert "lidar" in warnings[0]

    def test_no_warning_when_required_type_present(self):
        from arena_robots.assembly import Placement, ResolvedAssembly, warn_if_blind

        resolved = ResolvedAssembly(placements=[Placement(type="lidar", variant="sick", mount=_mount("top", ["lidar"]))])
        assert warn_if_blind(resolved, {"lidar"}) == []


class TestAssemblySpecParse:
    def test_parse_builds_mounts_priority_defaults(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0.2], "accepts": ["lidar"]}},
                "priority": {"lidar": ["top"]},
                "defaults": {"lidar": [{"variant": "sick_s300", "mount": "top", "params": {"max_angle": 2.1}}]},
            }
        )
        assert spec.mounts["top"].accepts == frozenset({"lidar"})
        assert spec.priority == {"lidar": ["top"]}
        assert spec.defaults["lidar"][0].params == {"max_angle": 2.1}

    def test_parse_rejects_priority_referencing_unknown_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="unknown mount"):
            AssemblySpec.parse({"mounts": {}, "priority": {"lidar": ["top"]}})

    def test_parse_rejects_default_mount_not_accepting_type(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="does not accept"):
            AssemblySpec.parse(
                {
                    "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["camera"]}},
                    "defaults": {"lidar": [{"variant": "sick_s300", "mount": "top"}]},
                }
            )

    def test_parse_passes_overrides_onto_default_part(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"rear": {"parent": "base_link", "xyz": [0, 0, 0.2], "accepts": ["lidar"]}},
                "defaults": {
                    "lidar": [{"variant": "sick_s300", "mount": "rear", "overrides": {"name": "lidar_rear", "topic": "scan/rear"}}]
                },
            }
        )
        assert spec.defaults["lidar"][0].overrides == {"name": "lidar_rear", "topic": "scan/rear"}

    def test_parse_default_overrides_defaults_to_empty(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]}},
                "defaults": {"lidar": [{"variant": "sick_s300", "mount": "top"}]},
            }
        )
        assert spec.defaults["lidar"][0].overrides == {}


class TestChainedMounts:
    """Phase3b sec2/3: a mount's ``parent`` may be ``"@<mount>:<frame>"``, chaining
    through another mount's placed component's exported frame."""

    def test_parse_chained_parent(self):
        from arena_robots.assembly import parse_chained_parent

        assert parse_chained_parent("@lift0:top") == ("lift0", "top")
        assert parse_chained_parent("base_link") is None

    def test_unchained_mount_has_no_chained_parent(self):
        m = _mount("top", ["lidar"])
        assert m.chained_parent is None

    def test_chained_mount_parses_ref(self):
        m = _mount("arm0", ["arm"], parent="@lift0:top")
        assert m.chained_parent == ("lift0", "top")

    def test_parse_rejects_chained_parent_to_unknown_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="unknown mount 'lift0'"):
            AssemblySpec.parse(
                {
                    "mounts": {
                        "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                    }
                }
            )

    def test_parse_rejects_two_mount_cycle(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="cycle"):
            AssemblySpec.parse(
                {
                    "mounts": {
                        "a": {"parent": "@b:top", "xyz": [0, 0, 0], "accepts": ["x"]},
                        "b": {"parent": "@a:top", "xyz": [0, 0, 0], "accepts": ["y"]},
                    }
                }
            )

    def test_parse_rejects_self_referencing_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="cycle"):
            AssemblySpec.parse({"mounts": {"a": {"parent": "@a:top", "xyz": [0, 0, 0], "accepts": ["x"]}}})

    def test_parse_accepts_valid_chain(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        assert spec.mounts["arm0"].chained_parent == ("lift0", "top")

    def test_empty_chained_mount_is_fine(self):
        """An unpopulated chained mount that nothing is placed on is not an error."""
        from arena_robots.assembly import AssemblySpec, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        resolved = resolve(spec, {})
        assert resolved.placements == []

    def test_placed_part_stranded_on_unpopulated_chain_raises(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        with pytest.raises(AssemblyError, match="'arm0' requires 'lift0'"):
            resolve(spec, {"arm": [RequestPart(variant="ur10e")]})

    def test_placed_part_on_populated_chain_succeeds(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        resolved = resolve(spec, {"lift": [RequestPart(variant="ewellix_900mm")], "arm": [RequestPart(variant="ur10e")]})
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type == {"lift": "lift0", "arm": "arm0"}


class TestResolveOverrides:
    def test_default_overrides_flow_onto_placement(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"rear": _mount("rear", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="rear", overrides={"name": "lidar_rear", "topic": "scan/rear"})]},
        )
        resolved = resolve(spec, {})
        assert resolved.placements[0].overrides == {"name": "lidar_rear", "topic": "scan/rear"}

    def test_request_placements_carry_no_overrides(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick")]})
        assert resolved.placements[0].overrides == {}
