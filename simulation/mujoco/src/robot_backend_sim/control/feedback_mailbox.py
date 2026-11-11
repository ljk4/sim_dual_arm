"""Bounded feedback delivery that never evicts a command transition."""

from __future__ import annotations

import queue
import threading
from collections import deque
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_backend_sim.control.control_state import ControlStatus


class FeedbackMailbox:
    """One FIFO: periodic samples may be replaced; transitions remain ordered.

    If every slot contains a transition, a new periodic sample is discarded.
    A new transition raises Full so the owner can fail closed instead of hiding
    the missing fact. No disk or ROS work runs while the lock is held.
    """

    def __init__(self, capacity: int) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError("feedback capacity must be a positive integer")
        self.maxsize = capacity
        self._entries: deque[tuple[ControlStatus, bool]] = deque()
        self._lock = threading.Lock()
        self.dropped = 0

    def put_nowait(self, status: ControlStatus, *, critical: bool = False) -> bool:
        with self._lock:
            if len(self._entries) == self.maxsize:
                victim = next(
                    (i for i, (_, protected) in enumerate(self._entries) if not protected), None
                )
                if victim is None:
                    if critical:
                        raise queue.Full
                    self.dropped += 1
                    return False
                del self._entries[victim]
                self.dropped += 1
            self._entries.append((status, critical))
            return True

    def get_nowait(self) -> ControlStatus:
        with self._lock:
            if not self._entries:
                raise queue.Empty
            return self._entries.popleft()[0]

    def qsize(self) -> int:
        with self._lock:
            return len(self._entries)

    def empty(self) -> bool:
        return self.qsize() == 0
