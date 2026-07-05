"""Tests for arena_robots.SetupFile.Config.parse against the parametrized-robot grammar
(.claude/parametrized-robots.md sec2.1-2.3)."""

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


class TestConfigParseIdentityLaneExtra:
    def test_pos_routes_to_extra_without_error(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "pos": (1, 2, 0)})
        assert len(configs) == 1
        assert configs[0].extra["pos"] == (1, 2, 0)

    def test_record_data_dir_routes_to_extra_without_error(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "jackal", "record_data_dir": "/tmp/out"})
        assert configs[0].extra["record_data_dir"] == "/tmp/out"


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


class TestConfigParsePhase1MorphologyGate:
    """Phase 1 hard-errors on any morphology item (bare or @-mount keys); no silent acceptance."""

    def test_bare_morphology_key_raises_not_implemented(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError, match="not yet implemented"):
            Config.parse({"robot": "jackal", "lidar": "sick"})

    def test_mounted_morphology_key_raises_not_implemented(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError, match="not yet implemented"):
            Config.parse({"robot": "jackal", "lidar@front": "sick"})

    def test_multi_instance_morphology_raises_not_implemented(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError, match="not yet implemented"):
            Config.parse({"robot": "jackal", "lidar": ["a", "b"]})

    def test_none_morphology_raises_not_implemented(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError, match="not yet implemented"):
            Config.parse({"robot": "jackal", "lidar": "none"})


class TestConfigParseGrammarPrecedesGate:
    """Grammar errors (none-mixing, none+mount) must be raised even though the Phase 1
    gate would also reject the same key; the two must not be confused for each other."""

    def test_none_mixed_with_variant_is_grammar_error_not_gate(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError) as excinfo:
            Config.parse({"robot": "jackal", "lidar": ["sick", "none"]})
        assert "not yet implemented" not in str(excinfo.value)

    def test_none_with_mount_is_grammar_error_not_gate(self):
        from arena_robots.SetupFile import Config

        with pytest.raises(RuntimeError) as excinfo:
            Config.parse({"robot": "jackal", "lidar@front": "none"})
        assert "not yet implemented" not in str(excinfo.value)


class TestPart:
    def test_part_default_mount_is_none(self):
        from arena_robots.SetupFile import Part

        p = Part(variant="sick")
        assert p.variant == "sick"
        assert p.mount is None

    def test_part_mount_is_stored(self):
        from arena_robots.SetupFile import Part

        p = Part(variant="sick", mount="front")
        assert p.mount == "front"
