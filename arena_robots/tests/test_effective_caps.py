"""Tests for RobotView.effective_caps and the effective_sensors consistency guard
(allocation-derived caps)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml


def _write_robot(
    tmp_path: Path,
    *,
    model_params: dict,
    assembly: dict | None = None,
    mobile_cap: dict | None = None,
) -> Path:
    rd = tmp_path / "robot"
    rd.mkdir()
    (rd / "model_params.yaml").write_text(yaml.dump(model_params))
    if assembly is not None:
        (rd / "assembly.yaml").write_text(yaml.dump(assembly))
    if mobile_cap is not None:
        caps_dir = rd / "caps"
        caps_dir.mkdir()
        (caps_dir / "mobile.yaml").write_text(yaml.dump(mobile_cap))
    return rd


class TestEffectiveCapsNoAssembly:
    def test_identity_when_no_assembly(self, tmp_path: Path):
        from arena_robots.Robot import RobotView

        rd = _write_robot(tmp_path, model_params={}, mobile_cap={"odom_frame": "odom"})
        view = RobotView(rd)
        assert view.assembly is None
        assert view.effective_caps({}) is view.caps


class TestEffectiveCapsRbtheron:
    """rbtheron declares sensors via assembly.yaml but declares no arm mount."""

    def test_available_unchanged_and_no_arm(self):
        from arena_robots.Robot import RobotIdentifier

        view = RobotIdentifier("rbtheron").resolve_sync()
        assert view.assembly is not None
        caps = view.effective_caps({})
        assert caps.available == view.caps.available
        assert caps.arm is None


class TestEffectiveSensorsGuard:
    """Owner decision 2: warn when model_params declares a sensor name the
    assembly's own DEFAULTS render doesn't produce."""

    _ASSEMBLY = {
        "mounts": {"front_laser": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]}},
        "defaults": {"lidar": [{"variant": "sick_s300", "mount": "front_laser"}]},
    }

    def test_guard_warns_on_missing_defaults_sensor(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]):
        from arena_robots.Robot import RobotView

        rd = _write_robot(
            tmp_path,
            model_params={"sensors": [{"name": "ghost_sensor", "type": "laserscan", "topic": "/scan", "frame": "ghost_frame"}]},
            assembly=self._ASSEMBLY,
        )
        view = RobotView(rd)
        view.effective_sensors({})
        err = capsys.readouterr().err
        assert "ghost_sensor" in err
        assert "assembly.yaml incomplete" in err

    def test_no_warning_when_defaults_cover_declared_sensors(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]):
        from arena_robots.Robot import RobotView

        rd = _write_robot(
            tmp_path,
            model_params={"sensors": [{"name": "lidar", "type": "laserscan", "topic": "/scan", "frame": "f"}]},
            assembly=self._ASSEMBLY,
        )
        view = RobotView(rd)
        view.effective_sensors({})
        assert capsys.readouterr().err == ""
