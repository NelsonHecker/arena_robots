"""Tests for arena_robots.SetupFile.Config.parse against the parametrized-robot grammar:
mount-centric addressing."""

from __future__ import annotations

import pytest


class TestConfigParseIdentity:
    def test_parse_string_returns_single_named_config(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse("jackal")
        assert len(configs) == 1
        c = configs[0]
        assert c.robot == "jackal"
        assert c.name == "jackal"

    def test_parse_dict_count_multiplier_produces_independent_configs(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "count": 3})
        assert len(configs) == 3
        for c in configs:
            assert c.robot == "jackal"

    def test_parse_dict_count_isolates_mutable_extra(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "count": 2, "pos": [1, 2, 0]})
        assert len(configs) == 2
        configs[0].extra["pos"].append(99)
        assert 99 not in configs[1].extra["pos"]


class TestConfigParseAdapters:
    def test_dotted_adapter_shorthand_routes_to_adapters(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "mobile.adapter": "drl"})
        assert configs[0].adapters == {"mobile": "drl"}

    def test_adapters_block_routes_verbatim(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "adapters": {"mobile": "nav2"}})
        assert configs[0].adapters == {"mobile": "nav2"}

    def test_adapters_block_non_dict_raises(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError):
            Config.parse({"robot": "jackal", "adapters": "nav2"})


class TestConfigParseFrames:
    def test_frames_block_routes_verbatim(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "frames": {"top": "custom_laser_link"}})
        assert configs[0].frames == {"top": "custom_laser_link"}

    def test_frames_block_non_dict_raises(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError):
            Config.parse({"robot": "jackal", "frames": "custom_laser_link"})

    def test_frames_defaults_to_empty(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal"})
        assert configs[0].frames == {}


class TestConfigParseIdentityLaneExtra:
    def test_pos_routes_to_extra_without_error(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "pos": (1, 2, 0)})
        assert len(configs) == 1
        assert configs[0].extra["pos"] == (1, 2, 0)


class TestConfigParseGrammarRejections:
    @pytest.mark.parametrize("bad_key", ["mobile.kp", "x.y.z", "arm.speed"])
    def test_dotted_non_adapter_key_raises(self, bad_key: str):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError, match=r"mobile\.kp|x\.y\.z|arm\.speed"):
            Config.parse({"robot": "jackal", bad_key: "1"})

    def test_reserved_key_parts_raises(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError, match="parts"):
            Config.parse({"robot": "jackal", "parts": {"lidar": ["sick"]}})


class TestConfigParseMorphologyDirectives:
    """Bare keys (mount-centric ``mount=type/variant`` / ``mount=variant`` or
    shorthand ``type=variant``) are grammar-collected here as raw value strings only;
    mount-vs-type disambiguation and ``none`` semantics are resolved later, per-robot,
    by ``arena_assembly.build_request`` at ``Robot.parse`` resolution time."""

    def test_shorthand_key_populates_parts_as_raw_strings(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "lidar": "sick"})
        assert configs[0].parts == {"lidar": ["sick"]}

    def test_mount_centric_type_slash_variant_populates_parts_as_raw_strings(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "top": "lidar/sick"})
        assert configs[0].parts == {"top": ["lidar/sick"]}

    def test_multi_instance_directive_populates_list(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "lidar": ["a", "b"]})
        assert configs[0].parts == {"lidar": ["a", "b"]}

    def test_multiple_directives_populate_independent_keys(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "lidar": "sick", "top": "camera/d435"})
        assert configs[0].parts == {"lidar": ["sick"], "top": ["camera/d435"]}

    def test_none_value_passed_through_uninterpreted(self):
        """Config.parse collects 'none' as a raw value like any other, the clear grammar
        being interpreted downstream in build_request."""
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "lidar": "none"})
        assert configs[0].parts == {"lidar": ["none"]}

    def test_parts_defaults_to_empty(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal"})
        assert configs[0].parts == {}

    def test_runtime_spawn_shaped_dict_routes_bare_mount_into_parts(self):
        """The runtime spawn path (``arena robot jackal top:=arm/ur5e``) builds a flat
        dict with ``robot``/``name``/``pos`` plus bare mount directives; the bare key
        must land in ``parts``, not get dropped."""
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "name": "j0", "pos": (1, 2, 0), "top": "arm/ur5e"})
        assert len(configs) == 1
        c = configs[0]
        assert c.parts == {"top": ["arm/ur5e"]}
        assert c.name == "j0"
        assert c.extra["pos"] == (1, 2, 0)
