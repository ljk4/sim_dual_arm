"""Immutable logging inputs, validation, and bounded JSON event rendering.

Rendering happens on the producer thread.  The writer owns all file I/O, so an
event that reaches the queue is already a validated byte string.
"""

from __future__ import annotations

import json
import math
import re
import traceback
from collections.abc import Callable, Mapping
from dataclasses import dataclass

type Scalar = bool | int | float | str | None

_MAX_IDENTIFIER_CHARS = 128
_MAX_EVENT_CHARS = 128
_MAX_REASON_CHARS = 128
_MAX_FIELD_COUNT = 64
_MAX_FIELD_KEY_CHARS = 64
_MAX_FIELD_STRING_CHARS = 2048
_MAX_MESSAGE_CHARS = 8192
_MAX_EXCEPTION_MESSAGE_CHARS = 4096
_MAX_STACK_CHARS = 16384
_MAX_CONSOLE_CHARS = 512

_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]*$")
_EVENT_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_.:-]*$")
_LEVEL_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]{0,15}$")
_FIELD_KEY_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]{0,63}$")


class LogFailure(RuntimeError):
    """The logger cannot guarantee complete, durable event delivery."""


@dataclass(frozen=True)
class LogSettings:
    """Small, explicit limits for one logger instance."""

    queue_capacity: int = 256
    max_event_bytes: int = 32768
    max_file_bytes: int = 8 * 1024 * 1024
    max_total_bytes: int = 256 * 1024 * 1024
    close_timeout_s: float = 3.0
    flush_interval_s: float = 0.2
    console: bool = False

    def __post_init__(self) -> None:
        for name in ("queue_capacity", "max_event_bytes", "max_file_bytes", "max_total_bytes"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if not self.max_event_bytes <= self.max_file_bytes <= self.max_total_bytes:
            raise ValueError("max_event_bytes <= max_file_bytes <= max_total_bytes is required")
        for name in ("close_timeout_s", "flush_interval_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ValueError(f"{name} must be a finite positive number")
            number = float(value)
            if not math.isfinite(number) or number <= 0:
                raise ValueError(f"{name} must be a finite positive number")
            object.__setattr__(self, name, number)
        if not isinstance(self.console, bool):
            raise ValueError("console must be a boolean")


@dataclass(frozen=True)
class EventContext:
    """Optional correlation and simulation-clock values for one event."""

    episode_id: str | None = None
    request_id: str | None = None
    command_id: str | None = None
    clock_epoch: int | None = None
    sim_time_ns: int | None = None


@dataclass(frozen=True)
class _ExceptionText:
    type_name: str
    message: str
    stacktrace: str


@dataclass(frozen=True)
class EncodedEvent:
    body: bytes
    critical: bool
    console: str | None


def validate_identifier(value: str, label: str, *, max_chars: int = _MAX_IDENTIFIER_CHARS) -> str:
    """Validate a stable identity without ever truncating it."""
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    if not 1 <= len(value) <= max_chars:
        raise ValueError(f"{label} must contain 1..{max_chars} characters")
    if not _IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(f"{label} contains unsupported characters")
    return value


def _validate_context(context: EventContext | None) -> tuple[str | None, ...]:
    if context is None:
        return (None, None, None, None, None)
    if not isinstance(context, EventContext):
        raise ValueError("context must be an EventContext or None")
    ids = []
    for label, value in (
        ("episode_id", context.episode_id),
        ("request_id", context.request_id),
        ("command_id", context.command_id),
    ):
        ids.append(None if value is None else validate_identifier(value, label))
    for label, value in (
        ("clock_epoch", context.clock_epoch),
        ("sim_time_ns", context.sim_time_ns),
    ):
        if value is not None and (isinstance(value, bool) or not isinstance(value, int)):
            raise ValueError(f"{label} must be an integer or None")
    return (ids[0], ids[1], ids[2], context.clock_epoch, context.sim_time_ns)


def _validate_fields(fields: Mapping[str, Scalar] | None) -> dict[str, Scalar]:
    if fields is None:
        return {}
    if not isinstance(fields, Mapping):
        raise ValueError("fields must be a mapping or None")
    if len(fields) > _MAX_FIELD_COUNT:
        raise ValueError(f"fields must contain at most {_MAX_FIELD_COUNT} entries")
    result: dict[str, Scalar] = {}
    for key, value in fields.items():
        if not isinstance(key, str) or not _FIELD_KEY_PATTERN.fullmatch(key):
            raise ValueError(f"field key {key!r} is not a supported name")
        if len(key) > _MAX_FIELD_KEY_CHARS:
            raise ValueError(f"field key {key!r} is too long")
        if value is None or isinstance(value, bool):
            result[key] = value
        elif isinstance(value, int):
            result[key] = value
        elif isinstance(value, float):
            if not math.isfinite(value):
                raise ValueError(f"field {key!r} must be finite")
            result[key] = value
        elif isinstance(value, str):
            if len(value) > _MAX_FIELD_STRING_CHARS:
                raise ValueError(
                    f"field {key!r} must contain at most {_MAX_FIELD_STRING_CHARS} characters"
                )
            result[key] = value
        else:
            raise ValueError(
                f"field {key!r} has unsupported type {type(value).__name__}; "
                "only finite numbers, bool, str, and None are allowed"
            )
    return result


def _exception_text(exception: BaseException | None) -> _ExceptionText | None:
    if exception is None:
        return None
    if not isinstance(exception, BaseException):
        raise ValueError("exception must be a BaseException or None")
    type_name = type(exception).__name__
    try:
        message = str(exception)
    except Exception:
        message = f"<{type_name} str() failed>"
    try:
        stacktrace = "".join(
            traceback.format_exception(type(exception), exception, exception.__traceback__)
        )
    except Exception:
        stacktrace = f"{type_name}: {message}"
    if not stacktrace:
        stacktrace = f"{type_name}: {message}"
    return _ExceptionText(type_name, message, stacktrace)


def _clip(text: str, cap: int, label: str) -> tuple[str, bool, int]:
    """Return text bounded to cap characters and an explicit truncation marker."""
    original = len(text)
    if original <= cap:
        return text, False, original
    marker = f"...[{label} truncated; original_chars={original}]"
    if cap >= len(marker):
        return text[: cap - len(marker)] + marker, True, original
    # The marker preserves the original length even when the requested cap is
    # smaller than the marker itself; the caller treats this as the minimum
    # representation and may reject the whole event if it still cannot fit.
    return marker, True, original


@dataclass(frozen=True)
class _PreparedEvent:
    run_id: str
    component: str
    instance_id: str
    level: str
    event: str
    reason: str | None
    message: str
    context: tuple[str | None, str | None, str | None, int | None, int | None]
    fields: dict[str, Scalar]
    critical: bool
    exception: _ExceptionText | None
    event_limit: int
    console_enabled: bool

    def _caps(self) -> tuple[int, int, int]:
        message_cap = min(len(self.message), _MAX_MESSAGE_CHARS)
        if self.exception is None:
            return message_cap, 0, 0
        return (
            message_cap,
            min(len(self.exception.message), _MAX_EXCEPTION_MESSAGE_CHARS),
            min(len(self.exception.stacktrace), _MAX_STACK_CHARS),
        )

    def _build(
        self,
        event_seq: int,
        wall_time_utc: str,
        elapsed_ns: int,
        message_cap: int,
        exception_message_cap: int,
        stacktrace_cap: int,
    ) -> dict[str, object]:
        message, _, _ = _clip(self.message, message_cap, "message")
        exception: dict[str, object] | None = None
        if self.exception is not None:
            exc_message, exc_truncated, exc_original = _clip(
                self.exception.message, exception_message_cap, "exception message"
            )
            stack, stack_truncated, stack_original = _clip(
                self.exception.stacktrace, stacktrace_cap, "exception stack"
            )
            exception = {
                "type": self.exception.type_name,
                "message": exc_message,
                "stacktrace": stack,
                "truncated": exc_truncated or stack_truncated,
                "message_original_chars": exc_original,
                "stacktrace_original_chars": stack_original,
            }
        episode_id, request_id, command_id, clock_epoch, sim_time_ns = self.context
        return {
            "run_id": self.run_id,
            "component": self.component,
            "instance_id": self.instance_id,
            "event_seq": event_seq,
            "wall_time_utc": wall_time_utc,
            "elapsed_ns": elapsed_ns,
            "level": self.level,
            "event": self.event,
            "reason": self.reason,
            "message": message,
            "episode_id": episode_id,
            "request_id": request_id,
            "command_id": command_id,
            "clock_epoch": clock_epoch,
            "sim_time_ns": sim_time_ns,
            "critical": self.critical,
            "fields": dict(self.fields),
            "exception": exception,
        }

    @staticmethod
    def _dump(record: dict[str, object]) -> bytes:
        return json.dumps(
            record,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")

    def _largest_fitting(
        self,
        high: int,
        make: Callable[[int], dict[str, object]],
    ) -> int:
        low = 0
        while low < high:
            middle = (low + high + 1) // 2
            if len(self._dump(make(middle))) + 1 <= self.event_limit:
                low = middle
            else:
                high = middle - 1
        return low

    def _fit(
        self,
        event_seq: int,
        wall_time_utc: str,
        elapsed_ns: int,
        caps: tuple[int, int, int],
    ) -> bytes:
        message_cap, exception_message_cap, stacktrace_cap = caps

        def build(msg_cap: int, exc_cap: int, stack_cap: int) -> dict[str, object]:
            return self._build(
                event_seq, wall_time_utc, elapsed_ns, msg_cap, exc_cap, stack_cap
            )

        body = self._dump(build(message_cap, exception_message_cap, stacktrace_cap))
        if len(body) + 1 <= self.event_limit:
            return body
        minimum = self._dump(build(0, 0, 0))
        if len(minimum) + 1 > self.event_limit:
            raise ValueError(
                f"event {self.event!r} cannot fit in max_event_bytes={self.event_limit}"
            )
        stacktrace_cap = self._largest_fitting(
            stacktrace_cap, lambda cap: build(message_cap, exception_message_cap, cap)
        )
        exception_message_cap = self._largest_fitting(
            exception_message_cap, lambda cap: build(message_cap, cap, stacktrace_cap)
        )
        message_cap = self._largest_fitting(
            message_cap, lambda cap: build(cap, exception_message_cap, stacktrace_cap)
        )
        body = self._dump(build(message_cap, exception_message_cap, stacktrace_cap))
        if len(body) + 1 > self.event_limit:
            raise ValueError(
                f"event {self.event!r} cannot fit in max_event_bytes={self.event_limit}"
            )
        return body

    def _console(self, wall_time_utc: str, event_seq: int) -> str | None:
        if not self.console_enabled:
            return None
        detail = " ".join(self.message.splitlines())
        if len(detail) > _MAX_CONSOLE_CHARS:
            detail = detail[: _MAX_CONSOLE_CHARS - 3] + "..."
        suffix = f" {detail}" if detail else ""
        console = f"{wall_time_utc} {self.level} {self.event} seq={event_seq}{suffix}"
        if len(console) > _MAX_CONSOLE_CHARS:
            console = console[: _MAX_CONSOLE_CHARS - 3] + "..."
        return console

    def render(
        self,
        *,
        event_seq: int,
        wall_time_utc: str,
        elapsed_ns: int,
    ) -> EncodedEvent:
        body = self._fit(
            event_seq,
            wall_time_utc,
            elapsed_ns,
            self._caps(),
        )
        return EncodedEvent(
            body=body,
            critical=self.critical,
            console=self._console(wall_time_utc, event_seq),
        )


def prepare_event(
    *,
    run_id: str,
    component: str,
    instance_id: str,
    event: str,
    message: str,
    level: str,
    reason: str | None,
    context: EventContext | None,
    fields: Mapping[str, Scalar] | None,
    exception: BaseException | None,
    critical: bool,
    event_limit: int,
    console_enabled: bool,
) -> _PreparedEvent:
    """Validate everything the writer may later need before taking a sequence."""
    if not isinstance(event, str) or not 1 <= len(event) <= _MAX_EVENT_CHARS:
        raise ValueError(f"event must contain 1..{_MAX_EVENT_CHARS} characters")
    if not _EVENT_PATTERN.fullmatch(event):
        raise ValueError("event contains unsupported characters")
    if not isinstance(level, str) or not _LEVEL_PATTERN.fullmatch(level):
        raise ValueError("level must be an uppercase severity token")
    if not isinstance(message, str):
        raise ValueError("message must be a string")
    if reason is not None:
        if not isinstance(reason, str):
            raise ValueError("reason must be a string or None")
        if reason == "":
            reason = None
        else:
            if len(reason) > _MAX_REASON_CHARS:
                raise ValueError(f"reason must contain at most {_MAX_REASON_CHARS} characters")
            if not _IDENTIFIER_PATTERN.fullmatch(reason):
                raise ValueError("reason contains unsupported characters")
    if not isinstance(critical, bool):
        raise ValueError("critical must be a boolean")
    return _PreparedEvent(
        run_id=run_id,
        component=component,
        instance_id=instance_id,
        level=level,
        event=event,
        reason=reason,
        message=message,
        context=_validate_context(context),
        fields=_validate_fields(fields),
        critical=critical,
        exception=_exception_text(exception),
        event_limit=event_limit,
        console_enabled=console_enabled,
    )
