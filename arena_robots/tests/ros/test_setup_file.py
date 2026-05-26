"""Tests for arena_robots.SetupFile."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml


class TestConfigParse:
    def test_parse_string_returns_single(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse("my_robot")
        assert len(configs) == 1
        c = configs[0]
        assert c.robot == "my_robot"
        assert c.name == "my_robot"

    def test_parse_dict_count_1(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "turtlebot3", "name": "tb3"})
        assert len(configs) == 1
        assert configs[0].robot == "turtlebot3"
        assert configs[0].name == "tb3"

    def test_parse_dict_count_multiplier(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "turtlebot3", "count": 3})
        assert len(configs) == 3
        for c in configs:
            assert c.robot == "turtlebot3"

    def test_parse_dict_count_not_in_fields(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "tb3", "count": 2})
        for c in configs:
            with pytest.raises((AttributeError, KeyError)):
                _ = c.__dict__["count"]

    def test_parse_dict_extra_fields(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "tb3", "planner": "NavFn", "controller": "DWB"})
        assert len(configs) == 1
        assert configs[0].extra == {"planner": "NavFn", "controller": "DWB"}

    def test_parse_dict_mobile_field(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "tb3", "mobile": "rl"})
        assert configs[0].mobile == "rl"

    def test_parse_dict_arm_field(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "tb3", "arm": "moveit"})
        assert configs[0].arm == "moveit"

    def test_parse_dict_defaults(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "tb3"})
        c = configs[0]
        assert c.name is None
        assert c.mobile is None
        assert c.arm is None
        assert c.extra == {}

    def test_parse_deepcopy_count_isolation(self):
        from arena_robots.SetupFile import Config

        configs = Config.parse({"robot": "tb3", "extra": {"key": [1, 2, 3]}, "count": 2})
        configs[0].extra["key"].append(99)
        assert 99 not in configs[1].extra["key"]


class TestRobotSetupIdentifier:
    def test_load_valid_list(self, tmp_path: Path):
        from arena_robots.SetupFile import RobotSetupIdentifier

        path = tmp_path / "setup.yaml"
        path.write_text(yaml.dump(["robot_a", "robot_b"]))
        identifier = RobotSetupIdentifier(name="setup")
        result = identifier.load(path)
        assert len(result) == 2
        assert result[0].robot == "robot_a"
        assert result[1].robot == "robot_b"

    def test_load_list_with_dict_entries(self, tmp_path: Path):
        from arena_robots.SetupFile import RobotSetupIdentifier

        path = tmp_path / "setup.yaml"
        path.write_text(yaml.dump([{"robot": "tb3", "count": 2}]))
        identifier = RobotSetupIdentifier(name="setup")
        result = identifier.load(path)
        assert len(result) == 2

    def test_load_non_list_raises(self, tmp_path: Path):
        from arena_robots.SetupFile import RobotSetupIdentifier

        path = tmp_path / "setup.yaml"
        path.write_text(yaml.dump({"robot": "tb3"}))
        identifier = RobotSetupIdentifier(name="setup")
        with pytest.raises(ValueError, match="must be a list"):
            identifier.load(path)

    def test_load_flattens_count(self, tmp_path: Path):
        from arena_robots.SetupFile import RobotSetupIdentifier

        path = tmp_path / "setup.yaml"
        path.write_text(yaml.dump(["r1", {"robot": "r2", "count": 3}, "r3"]))
        identifier = RobotSetupIdentifier(name="setup")
        result = identifier.load(path)
        assert len(result) == 1 + 3 + 1
