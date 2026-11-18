"""Thread-safe bridge from worker ``concurrent.futures.Future`` to rclpy.

The ROS executor is a ``SingleThreadedExecutor`` and does not run a nested
asyncio loop here.  A completed worker future triggers a guard condition; the
executor thread then fulfils a ``rclpy.task.Future`` awaited by the async service
callback.  A one-shot timer enforces the monotonic deadline.  Both resources are
destroyed on completion, timeout, caller cancellation or node shutdown.
"""

from __future__ import annotations

import math
import threading
from concurrent.futures import Future
from typing import TYPE_CHECKING

from rclpy.clock import Clock, ClockType
from rclpy.task import Future as RosFuture

if TYPE_CHECKING:  # pragma: no cover - typing only
    from rclpy.callback_groups import CallbackGroup
    from rclpy.node import Node

__all__ = ["RosFutureBridge"]


class RosFutureBridge:
    """One bounded service-call bridge owned by the ROS executor thread."""

    def __init__(
        self,
        node: Node,
        future: Future,
        *,
        timeout_s: float,
        timeout_reason: str,
        callback_group: CallbackGroup,
    ) -> None:
        if not isinstance(future, Future):
            raise TypeError("future must be a concurrent.futures.Future")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self._node = node
        self._future = future
        self._timeout_reason = timeout_reason
        self._ros_future = RosFuture()
        self._lock = threading.Lock()
        self._finished = False
        self._closed = False
        # Resources must exist before add_done_callback: a future can already
        # be complete, in which case the callback runs synchronously.
        self._guard = node.create_guard_condition(self._on_guard, callback_group=callback_group)
        self._timer = node.create_timer(
            timeout_s,
            self._on_timeout,
            callback_group=callback_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._future.add_done_callback(self._on_done)

    @property
    def ros_future(self) -> RosFuture:
        return self._ros_future

    async def wait(self) -> object:
        return await self._ros_future

    def _on_done(self, future: Future) -> None:
        with self._lock:
            if self._finished or self._closed:
                return
        try:
            self._guard.trigger()
        except Exception:  # noqa: BLE001 - node shutdown races are expected
            pass

    def _on_guard(self) -> None:
        self._deliver(timed_out=False)

    def _on_timeout(self) -> None:
        self._deliver(timed_out=True)

    def _deliver(self, *, timed_out: bool) -> None:
        with self._lock:
            if self._finished or self._closed:
                return
            self._finished = True
            future = self._future
        try:
            if timed_out and not future.done():
                # Cancellation succeeds only if the work has not started.  A
                # submit response still says OUTCOME_UNKNOWN: the request may
                # already own an accepted id even when cancellation lost the race.
                cancelled = future.cancel()
                error: BaseException = TimeoutError(
                    self._timeout_reason if not cancelled else f"{self._timeout_reason}_CANCELLED"
                )
                self._ros_future.set_exception(error)
            elif future.cancelled():
                self._ros_future.set_exception(RuntimeError("worker future cancelled"))
            elif future.exception() is not None:
                self._ros_future.set_exception(future.exception())
            else:
                self._ros_future.set_result(future.result())
        finally:
            self._destroy_entities()

    def close(self) -> None:
        """Release entities; cancel only work that has not started."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            future = self._future
        if not future.done():
            future.cancel()
        self._destroy_entities()

    def abort(self) -> None:
        """Resolve outstanding waiters before destroying node-owned entities."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._finished = True
            future = self._future
        if not future.done():
            future.cancel()
        if not self._ros_future.done():
            self._ros_future.set_exception(RuntimeError("BACKEND_CLOSED"))
        self._destroy_entities()

    def _destroy_entities(self) -> None:
        # Destroy is idempotent from rclpy's perspective; node.destroy_* returns
        # False after an earlier destroy or node shutdown.
        try:
            self._node.destroy_timer(self._timer)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._node.destroy_guard_condition(self._guard)
        except Exception:  # noqa: BLE001
            pass
