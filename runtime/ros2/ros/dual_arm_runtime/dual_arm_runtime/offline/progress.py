"""Width-safe single-line terminal progress helpers for runtime offline tools."""

from __future__ import annotations

import os
import shutil
import sys
import unicodedata
from typing import TextIO


def is_progress_tty(stream: TextIO | None = None) -> bool:
    """Return True when live single-line terminal progress is enabled."""
    target = sys.stdout if stream is None else stream
    return bool(target.isatty()) or os.environ.get("RECORD_PROGRESS_TTY") == "1"


def terminal_columns(stream: TextIO | None = None) -> int:
    """Return current terminal width, honoring forwarded host COLUMNS."""
    target = sys.stdout if stream is None else stream
    try:
        columns = os.get_terminal_size(target.fileno()).columns
        if columns > 0:
            return max(20, columns)
    except (OSError, ValueError, AttributeError):
        pass
    env_columns = os.environ.get("COLUMNS", "").strip()
    if env_columns:
        try:
            parsed = int(env_columns)
            if parsed > 0:
                return max(20, parsed)
        except ValueError:
            pass
    return max(20, shutil.get_terminal_size((80, 24)).columns)


def display_width(text: str) -> int:
    """Estimate terminal column width (fullwidth/wide CJK glyphs take 2 columns)."""
    return sum(2 if unicodedata.east_asian_width(ch) in {"W", "F"} else 1 for ch in text)


def fit_terminal_line(text: str, max_width: int) -> str:
    """Truncate text so its terminal display width never exceeds max_width."""
    if max_width <= 0:
        return ""
    result: list[str] = []
    used = 0
    for ch in text:
        size = 2 if unicodedata.east_asian_width(ch) in {"W", "F"} else 1
        if used + size > max_width:
            break
        result.append(ch)
        used += size
    return "".join(result)


def format_duration(seconds: float | None) -> str:
    """Format seconds as HH:MM:SS or --:--:-- when unknown."""
    if seconds is None:
        return "--:--:--"
    total = max(0, round(seconds))
    minutes, secs = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def ascii_bar(done: int, total: int, width: int = 10) -> str:
    """Render an ASCII progress bar free of ambiguous-width Unicode blocks."""
    safe_total = max(1, total)
    ratio = min(1.0, max(0.0, done / safe_total))
    filled = int(width * ratio)
    return "#" * filled + "-" * (width - filled)


def write_progress_line(
    line: str, *, tty: bool, stream: TextIO | None = None
) -> None:
    """Write a single progress update in-place on TTY or as a plain line otherwise."""
    target = sys.stdout if stream is None else stream
    if tty:
        max_width = max(1, terminal_columns(target) - 1)
        fitted = fit_terminal_line(line, max_width)
        target.write(f"\r\x1b[2K{fitted}")
        target.flush()
    else:
        target.write(f"{line}\n")
        target.flush()


def finish_progress_line(*, tty: bool, stream: TextIO | None = None) -> None:
    """Terminate an active in-place TTY progress line with a newline."""
    if not tty:
        return
    target = sys.stdout if stream is None else stream
    target.write("\n")
    target.flush()
