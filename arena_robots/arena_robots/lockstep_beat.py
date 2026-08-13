"""Lockstep heartbeat for task_server handlers.

A handler acquires the beat for the lifetime of an active goal: acquire
registers a hard lockstep channel for this robot, each pulse republishes
controller activity as a coverage stamp, and release deregisters. A
wall-clock watchdog covers silent phases (MoveIt planning, nav2 recovery
behaviors) with forward keepalive stamps after a grace period, so a
producer that legitimately needs sim time to pass cannot freeze the sim.
"""

from __future__ import annotations

from arena_runtime_msgs.msg import LockstepChannel, LockstepHeartbeat, LockstepRegistration
from arena_runtime_msgs.srv import LockstepRegister
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.clock import Clock
from rclpy.duration import Duration

from arena_robots.task_server_handlers import _executor_sleep

_REGISTER_SERVICE = "/arena/sim_lifecycle/lockstep/register"
_REGISTER_TIMEOUT = 3.0
_WATCHDOG_PERIOD = 0.2
_GRACE_S = 2.0
_KEEPALIVE_COVER_S = 2.0


class LockstepBeat:
    def __init__(self, node: object, kind: str) -> None:
        ns = node.get_namespace().rstrip("/")
        self._node = node
        self._name = f"{kind}/{ns.rsplit('/', 1)[-1]}"
        self._topic = f"{ns}/lockstep/{kind}"
        self._env = ns.rsplit("/", 1)[0] or "/"
        self._pub = node.create_publisher(LockstepHeartbeat, self._topic, 10)
        self._client = node.create_client(LockstepRegister, _REGISTER_SERVICE)
        self._active = 0
        self._registered = False
        self._starved = False
        self._wall = Clock()
        self._last_pulse = self._wall.now()
        node.create_timer(_WATCHDOG_PERIOD, self._watchdog, clock=self._wall)

    async def acquire(self, period_s: float) -> None:
        """Register the channel on the first concurrent acquisition."""
        self._active += 1
        if self._active > 1:
            return
        self._starved = False
        self._last_pulse = self._wall.now()
        self._registered = await self._apply(
            [
                LockstepChannel(
                    name=self._name,
                    topic=self._topic,
                    type="arena_runtime_msgs/msg/LockstepHeartbeat",
                    period_s=period_s,
                    hard=True,
                )
            ]
        )
        if self._registered and self._active == 0:
            # released while the register call was in flight
            self._registered = False
            await self._apply([])

    async def release(self) -> None:
        """Deregister when the last concurrent acquisition ends."""
        self._active -= 1
        if self._active > 0 or not self._registered:
            return
        self._registered = False
        await self._apply([])

    def pulse(self, stamp: TimeMsg | None = None) -> None:
        """Publish one coverage stamp, defaulting to current sim time."""
        if self._active <= 0:
            return
        self._last_pulse = self._wall.now()
        self._starved = False
        msg = LockstepHeartbeat()
        msg.header.stamp = stamp if stamp is not None else self._node.get_clock().now().to_msg()
        self._pub.publish(msg)

    def _watchdog(self) -> None:
        if self._active <= 0 or not self._registered:
            return
        silent = (self._wall.now() - self._last_pulse).nanoseconds * 1e-9
        if silent < _GRACE_S:
            return
        if not self._starved:
            self._starved = True
            self._node.get_logger().warning(f"lockstep beat {self._name}: silent {silent:.1f}s, keepalive engaged")
        msg = LockstepHeartbeat()
        msg.header.stamp = (self._node.get_clock().now() + Duration(seconds=_KEEPALIVE_COVER_S)).to_msg()
        self._pub.publish(msg)

    async def _apply(self, channels: list[LockstepChannel]) -> bool:
        deadline = self._wall.now().nanoseconds * 1e-9 + _REGISTER_TIMEOUT
        while not self._client.service_is_ready():
            if self._wall.now().nanoseconds * 1e-9 >= deadline:
                self._node.get_logger().info("lockstep register service not available, skipping beat registration")
                return False
            await _executor_sleep(self._node, 0.1, wall=True)
        request = LockstepRegister.Request()
        request.registration = LockstepRegistration(caller=self._topic, env=self._env, channels=channels)
        call = self._client.call_async(request)
        while not call.done():
            if self._wall.now().nanoseconds * 1e-9 >= deadline:
                self._node.get_logger().warning(f"lockstep beat {self._name}: register call timed out")
                return False
            await _executor_sleep(self._node, 0.1, wall=True)
        response = call.result()
        if not response.success:
            self._node.get_logger().warning(f"lockstep beat {self._name}: registration failed: {response.error_msg}")
            return False
        return bool(channels)
