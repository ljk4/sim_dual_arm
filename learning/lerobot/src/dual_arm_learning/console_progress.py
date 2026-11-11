"""Quiet operator output with one refreshed progress line and a full detail log."""

from __future__ import annotations

import contextlib
import os
import shutil
import sys
import threading
import time
import traceback
import unicodedata
from pathlib import Path
from typing import TextIO


def _duration(seconds: float | None) -> str:
    if seconds is None:
        return "--:--:--"
    value = max(0, round(seconds))
    hours, remainder = divmod(value, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def _fit(text: str, width: int) -> str:
    result, used = [], 0
    for character in text:
        size = 2 if unicodedata.east_asian_width(character) in {"W", "F"} else 1
        if used + size > width:
            break
        result.append(character)
        used += size
    return "".join(result)


class ConsoleProgress:
    def __init__(self, stream: TextIO, *, clock=time.monotonic) -> None:
        self.stream, self.clock = stream, clock
        self.tty = bool(stream.isatty())
        self.lock = threading.RLock()
        self.stopped = threading.Event()
        self.thread: threading.Thread | None = None
        self.label, self.unit, self.status = "", "", ""
        self.total: int | None = None
        self.done = self.initial = 0
        self.started = clock()
        self.disabled = False

    def start(self) -> None:
        if self.tty:
            self.thread = threading.Thread(
                target=self._refresh, name="console-progress", daemon=True
            )
            self.thread.start()

    def _refresh(self) -> None:
        while not self.stopped.wait(0.5):
            self.render()

    def phase(self, label: str, total: int | None = None, *, unit="", initial=0) -> None:
        with self.lock:
            if self.label and not self.tty:
                self.render(force=True)
            self.label, self.total, self.unit = label, total, unit
            self.done = self.initial = initial
            self.status, self.started = "", self.clock()
            self.render(force=True)

    def update(self, completed: int, *, detail: str | None = None) -> None:
        with self.lock:
            self.done = completed
            if detail is not None:
                self.status = detail

    def advance(self, amount: int = 1) -> None:
        with self.lock:
            self.done += amount

    def detail(self, text: str) -> None:
        with self.lock:
            self.status = text

    def message(self, text: str) -> None:
        with self.lock:
            if self.disabled:
                return
            prefix = "\r\x1b[2K" if self.tty else ""
            try:
                self.stream.write(prefix + text + "\n")
                self.stream.flush()
            except (OSError, ValueError):
                self.disabled = True
                return
            if self.tty:
                self.render(force=True)

    def line(self) -> str:
        elapsed = max(0.0, self.clock() - self.started)
        count = self.done - self.initial
        eta = None
        if self.total is None:
            counter = "完成" if self.status == "完成" else "处理中"
        else:
            fraction = min(1.0, self.done / max(1, self.total))
            blocks = round(fraction * 10)
            counter = (f"[{'#' * blocks}{'-' * (10 - blocks)}] "
                       f"{self.done}/{self.total} {self.unit} {fraction * 100:5.1f}%")
            if self.done >= self.total:
                eta = 0.0
            elif count > 0:
                eta = elapsed * (self.total - self.done) / count
        detail = f" | {self.status}" if self.status else ""
        return (f"{self.label} {counter} | 用时 {_duration(elapsed)} "
                f"| 剩余 {_duration(eta)}{detail}")

    def render(self, *, force=False) -> None:
        with self.lock:
            if not self.label or self.disabled or (not self.tty and not force):
                return
            text = self.line()
            if self.tty:
                try:
                    columns = os.get_terminal_size(self.stream.fileno()).columns
                except (OSError, ValueError):
                    columns = shutil.get_terminal_size((120, 24)).columns
                width = max(1, columns - 1)
                text = "\r\x1b[2K" + _fit(text, width)
            else:
                text += "\n"
            try:
                self.stream.write(text)
                self.stream.flush()
            except (OSError, ValueError):
                self.disabled = True

    def close(self, outcome: str) -> None:
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
        self.detail(outcome)
        self.render(force=True)
        if self.tty and not self.disabled:
            try:
                self.stream.write("\n")
                self.stream.flush()
            except (OSError, ValueError):
                self.disabled = True


class OperatorConsole:
    """Capture Python, native codec and child-process chatter in one log file."""

    def __init__(self, log_path: Path, title: str, settings: list[str]) -> None:
        self.log_path, self.title, self.settings = log_path, title, settings
        self.stack = contextlib.ExitStack()
        self.saved_fds: dict[int, int] = {}
        self.progress: ConsoleProgress | None = None

    def __enter__(self) -> OperatorConsole:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("a", encoding="utf-8", buffering=1)
        self.stream = os.fdopen(os.dup(2), "w", encoding="utf-8", buffering=1)
        description = "\n".join([self.title, *self.settings, f"详细日志：{self.log_path}"]) + "\n"
        self.stream.write(description)
        self.log.write(description)
        for stream in (sys.stdout, sys.stderr):
            stream.flush()
        try:
            for descriptor in (1, 2):
                self.saved_fds[descriptor] = os.dup(descriptor)
                os.dup2(self.log.fileno(), descriptor)
            self.stack.enter_context(contextlib.redirect_stdout(self.log))
            self.stack.enter_context(contextlib.redirect_stderr(self.log))
            self.progress = ConsoleProgress(self.stream)
            self.progress.start()
        except BaseException:
            self._restore()
            raise
        return self

    def _restore(self) -> None:
        self.stack.close()
        self.log.flush()
        for descriptor, saved in self.saved_fds.items():
            os.dup2(saved, descriptor)
            os.close(saved)
        self.saved_fds.clear()
        self.log.close()
        self.stream.close()

    def __exit__(self, kind, error, trace) -> bool:
        try:
            if error is not None:
                traceback.print_exception(kind, error, trace, file=self.log)
            if self.progress is not None:
                self.progress.close("完成" if error is None else "已停止，详情见日志")
        finally:
            self._restore()
        return False
