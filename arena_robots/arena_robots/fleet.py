"""Fleet robots bound to their model-derived frames, odometry topics and drivetrain facts."""

from __future__ import annotations

import functools
import math
import subprocess
import typing
from collections.abc import Mapping

import attrs
import yaml

from arena_robots.Robot import RobotIdentifier

if typing.TYPE_CHECKING:
    from task_generator_msgs.msg import RobotFleet


def _join_frame(*parts: str) -> str:
    return "/".join(part.strip("/") for part in parts if part.strip("/"))


DEFAULT_ODOM_TOPIC_TEMPLATE = "{namespace}/{name}_velocity_controller/odom"

_MODEL_ERRORS = (OSError, LookupError, ValueError, TypeError, yaml.YAMLError, subprocess.CalledProcessError)


@attrs.frozen(kw_only=True)
class _ModelFacts:
    base_frame: str
    odom_topic: str
    wheel_separation_m: float | None
    max_linear_mps: float


def _wheel_separation(control: Mapping[str, typing.Any]) -> float | None:
    for name, value in control.items():
        if name == "controller_manager" or not isinstance(value, Mapping):
            continue
        params = value.get("ros__parameters")
        if not isinstance(params, Mapping) or "wheel_separation" not in params:
            continue
        effective = float(params["wheel_separation"]) * float(params.get("wheel_separation_multiplier", 1.0))
        return effective if effective > 0.0 and math.isfinite(effective) else None
    return None


@functools.cache
def _model_facts(model: str) -> _ModelFacts:
    view = RobotIdentifier(model).resolve_sync()
    control = view.model_params.control
    mobile = view.mobile
    try:
        separation = _wheel_separation(view.control)
    except _MODEL_ERRORS:
        separation = None
    return _ModelFacts(
        base_frame=view.model_params.base_frame.strip("/"),
        odom_topic=control.odom_topic.strip("/") if control is not None else "",
        wheel_separation_m=separation,
        max_linear_mps=float(mobile.velocity_limits.linear.max) if mobile is not None and mobile.velocity_limits is not None else 0.0,
    )


@attrs.frozen(kw_only=True)
class RobotBinding:
    """A fleet robot with its model-derived frames, odometry topics and drivetrain facts."""

    name: str
    model: str
    namespace: str
    frame_prefix: str
    model_base_frame: str
    base_frame: str
    odom_topics: tuple[str, ...]
    wheel_separation_m: float | None
    max_linear_mps: float
    error: str = ""

    def frame(self, leaf: str) -> str:
        return _join_frame(self.frame_prefix, leaf)

    def mount(self, template: str) -> str:
        """Array mount frame of an array.mount_frame template: empty = base frame, {prefix} and {base_frame} expand, a bare leaf joins the prefix."""
        configured = template.format(prefix=self.frame_prefix, base_frame=self.model_base_frame).strip("/")
        if not configured:
            return self.base_frame
        if "/" in configured:
            return configured
        return self.frame(configured)

    @classmethod
    def resolve(cls, *, name: str, model: str, namespace: str, frame_prefix: str, odom_topic_template: str = DEFAULT_ODOM_TOPIC_TEMPLATE) -> RobotBinding:
        """Model lookups that fail fall back to base_link, no model odometry, no wheel separation and 0 m/s, with error set."""
        namespace = namespace.rstrip("/")
        prefix = frame_prefix.strip("/")
        error = ""
        try:
            facts = _model_facts(model)
        except _MODEL_ERRORS as exc:
            error = f"could not resolve robot model {model!r}: {exc}"
            facts = _ModelFacts(base_frame="base_link", odom_topic="", wheel_separation_m=None, max_linear_mps=0.0)
        topics = [odom_topic_template.format(namespace=namespace, name=name), f"{namespace}/odom"]
        if facts.odom_topic:
            topics.append(f"{namespace}/{facts.odom_topic}")
        return cls(
            name=name,
            model=model,
            namespace=namespace,
            frame_prefix=prefix,
            model_base_frame=facts.base_frame,
            base_frame=_join_frame(prefix, facts.base_frame),
            odom_topics=tuple(dict.fromkeys(topic for topic in topics if topic)),
            wheel_separation_m=facts.wheel_separation_m,
            max_linear_mps=facts.max_linear_mps,
            error=error,
        )


def robot_bindings(fleet: RobotFleet, *, odom_topic_template: str = DEFAULT_ODOM_TOPIC_TEMPLATE) -> tuple[RobotBinding, ...]:
    """Every named fleet robot, in fleet order."""
    return tuple(
        RobotBinding.resolve(
            name=str(state.descriptor.name).strip(),
            model=str(state.descriptor.model),
            namespace=str(state.descriptor.ns),
            frame_prefix=str(state.descriptor.frame),
            odom_topic_template=odom_topic_template,
        )
        for state in fleet.robots
        if str(state.descriptor.name).strip()
    )


def bind_robot(fleet: RobotFleet, wanted: str = "", *, odom_topic_template: str = DEFAULT_ODOM_TOPIC_TEMPLATE) -> RobotBinding | None:
    """The fleet robot named wanted, or the first one when empty. None until the fleet carries it."""
    for state in fleet.robots:
        name = str(state.descriptor.name).strip()
        if name and (not wanted or name == wanted):
            return RobotBinding.resolve(
                name=name,
                model=str(state.descriptor.model),
                namespace=str(state.descriptor.ns),
                frame_prefix=str(state.descriptor.frame),
                odom_topic_template=odom_topic_template,
            )
    return None
