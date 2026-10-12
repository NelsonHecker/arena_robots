from __future__ import annotations

from pathlib import Path

import launch
import pytest
import yaml
from arena_rclpy_mixins.yaml_replace import YAMLReplacer
from arena_robots.caps import MobileSpec
from arena_robots.nav2 import Nav2CollisionDerivedYAML

PKG = Path(__file__).resolve().parents[2] / "arena_robots"
ROBOTS_DIR = PKG / "robots"
NAV2_YAML = PKG / "config" / "nav2" / "nav2.yaml"


def _inflation(robot: str) -> tuple[float, float]:
    derived = Nav2CollisionDerivedYAML(str(ROBOTS_DIR / robot / "caps" / "mobile.yaml")).perform(launch.LaunchContext())
    replacer = YAMLReplacer(yaml.safe_load(Path(derived).read_text()))
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    return tuple(replacer.replace(nav2[costmap][costmap]["ros__parameters"]["inflation_layer"])["inflation_radius"] for costmap in ("local_costmap", "global_costmap"))


@pytest.mark.parametrize(
    ("robot", "inflation"),
    [
        ("jackal", 0.3092 + 0.4),
        ("ridgeback", 0.6248 + 0.1 + 0.4),
        ("rbsummit", 0.6103 + 0.4),
        ("husky", 0.5 + 0.4),
    ],
)
def test_both_costmaps_inflate_the_outer_radius_plus_the_margin(robot, inflation):
    assert _inflation(robot) == pytest.approx((inflation, inflation), abs=1e-3)


def test_an_explicit_inflation_radius_wins():
    raw = {"radius": 0.3, "footprint": [[0.5, 0.5], [-0.5, -0.5]], "inflation_radius": 0.55}
    assert MobileSpec(path=Path("mobile.yaml"), raw=raw).inflation_radius == 0.55
