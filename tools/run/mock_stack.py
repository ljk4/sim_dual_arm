"""Small shared Compose/container helpers reused by the collect launcher.

Container identity validation stays in the already accepted ``tools/run/mock.py``
implementation; this module only adapts it for a launcher that needs an
injectable clock and an extra Compose override file.
"""

from __future__ import annotations

import os
import selectors
import signal
import subprocess
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

_TOOLS_RUN = Path(__file__).resolve().parent
if str(_TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(_TOOLS_RUN))

import dev_compose  # noqa: E402
import mock  # noqa: E402, UP026

ROOT = dev_compose.ROOT
BASE_COMPOSE_FILE = dev_compose.COMPOSE_FILE
COLLECT_COMPOSE_FILE = ROOT / "deploy/compose/mock_collect.yaml"
AUDIT_COMPOSE_FILE = ROOT / "deploy/compose/mock_audit.yaml"
Container = mock.Container
ExpectedStack = mock.ExpectedStack
HostError = mock.HostError


@dataclass(frozen=True)
class BoundedResult:
    """One command result whose stdout/stderr were retained with a hard byte cap."""

    returncode: int
    stdout: str
    stderr: str
    truncated: bool
    stdout_bytes: int
    stderr_bytes: int


def _kill_process_group(process: subprocess.Popen) -> None:
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except OSError:
        try:
            process.kill()
        except OSError:
            pass


class BoundedDocker:
    """Argument-list Docker invocation with a bounded stream capture and one clock."""

    def __init__(
        self,
        executable: str,
        *,
        runner: Callable[..., object] | None = None,
        clock: Callable[[], float] = time.monotonic,
        output_limit: int = 262144,
    ) -> None:
        if output_limit <= 0:
            raise ValueError("output_limit must be positive")
        self.executable = executable
        self._runner = runner
        self._clock = clock
        self.output_limit = output_limit

    def run(
        self,
        arguments: Sequence[str],
        deadline: float,
        *,
        environment: dict[str, str] | None = None,
    ):
        return self.run_argv([self.executable, *arguments], deadline, environment=environment)

    def run_argv(
        self,
        argv: Sequence[str],
        deadline: float,
        *,
        environment: dict[str, str] | None = None,
    ):
        remaining = deadline - self._clock()
        if remaining <= 0:
            raise HostError("COMMAND_TIMEOUT", "the shared command deadline expired")
        argv = list(argv)
        if self._runner is not None:
            try:
                return self._runner(
                    argv,
                    env=environment,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=remaining,
                )
            except (subprocess.TimeoutExpired, TimeoutError) as error:
                raise HostError(
                    "COMMAND_TIMEOUT",
                    f"command exceeded the shared deadline: {argv[0]}",
                    may_have_executed=True,
                ) from error
            except FileNotFoundError as error:
                raise HostError("DOCKER_UNAVAILABLE", f"cannot execute {argv[0]!r}") from error
            except OSError as error:
                raise HostError("DOCKER_ERROR", f"{type(error).__name__}: {error}") from error
        return self._run_streaming(argv, deadline, environment)

    def _run_streaming(
        self,
        argv: Sequence[str],
        deadline: float,
        environment: dict[str, str] | None,
    ) -> BoundedResult:
        try:
            process = subprocess.Popen(
                argv,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
        except FileNotFoundError as error:
            raise HostError("DOCKER_UNAVAILABLE", f"cannot execute {argv[0]!r}") from error
        except OSError as error:
            raise HostError("DOCKER_ERROR", f"{type(error).__name__}: {error}") from error
        assert process.stdout is not None and process.stderr is not None
        selector = selectors.DefaultSelector()
        buffers = {"stdout": bytearray(), "stderr": bytearray()}
        totals = {"stdout": 0, "stderr": 0}
        fd_names = {
            process.stdout.fileno(): "stdout",
            process.stderr.fileno(): "stderr",
        }
        for descriptor in fd_names:
            os.set_blocking(descriptor, False)
            selector.register(descriptor, selectors.EVENT_READ)
        timed_out = False
        drain_deadline: float | None = None
        drain_killed = False
        completed = False
        try:
            while selector.get_map():
                now = self._clock()
                limit_clock = drain_deadline if drain_deadline is not None else deadline
                remaining = limit_clock - now
                if remaining <= 0:
                    if drain_deadline is not None and drain_deadline < deadline:
                        if drain_killed:
                            break
                        _kill_process_group(process)
                        drain_killed = True
                        drain_deadline = min(deadline, now + 0.2)
                        continue
                    timed_out = True
                    _kill_process_group(process)
                    break
                events = selector.select(timeout=min(0.2, remaining))
                if process.poll() is not None and drain_deadline is None:
                    # Descendant plugins may hold the pipes after the CLI exits.
                    drain_deadline = min(deadline, self._clock() + 0.3)
                if not events:
                    continue
                for key, _mask in events:
                    try:
                        chunk = os.read(key.fd, 65536)
                    except BlockingIOError:
                        continue
                    except OSError:
                        chunk = b""
                    if not chunk:
                        selector.unregister(key.fd)
                        continue
                    name = fd_names[key.fd]
                    totals[name] += len(chunk)
                    buffer = buffers[name]
                    buffer.extend(chunk)
                    if len(buffer) > self.output_limit:
                        del buffer[: len(buffer) - self.output_limit]
            if timed_out:
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    _kill_process_group(process)
                    process.wait(timeout=1.0)
                raise HostError(
                    "COMMAND_TIMEOUT",
                    f"command exceeded the shared deadline: {argv[0]}",
                    may_have_executed=True,
                )
            try:
                process.wait(timeout=max(0.2, min(2.0, deadline - self._clock())))
            except subprocess.TimeoutExpired:
                _kill_process_group(process)
                process.wait(timeout=1.0)
                raise HostError(
                    "COMMAND_TIMEOUT",
                    f"command did not exit within its deadline: {argv[0]}",
                    may_have_executed=True,
                ) from None
            completed = True
        finally:
            # An asynchronous BaseException (for example KeyboardInterrupt or a
            # caller stop request) must not leave the CLI or its process group
            # running after the pipes are closed.  The normal and timeout paths
            # above retain their existing kill/reap and return semantics.
            if not completed:
                _kill_process_group(process)
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    try:
                        process.kill()
                    except OSError:
                        pass
                    try:
                        process.wait(timeout=1.0)
                    except (subprocess.TimeoutExpired, OSError):
                        pass
            selector.close()
            process.stdout.close()
            process.stderr.close()
        stdout = bytes(buffers["stdout"]).decode("utf-8", "replace")
        stderr = bytes(buffers["stderr"]).decode("utf-8", "replace")
        return BoundedResult(
            returncode=int(process.returncode),
            stdout=stdout,
            stderr=stderr,
            truncated=(
                totals["stdout"] > self.output_limit or totals["stderr"] > self.output_limit
            ),
            stdout_bytes=totals["stdout"],
            stderr_bytes=totals["stderr"],
        )


def project_ids(docker: BoundedDocker, project: str, deadline: float) -> list[str]:
    return mock._project_ids(docker, project, deadline)


def inspect_containers(
    docker: BoundedDocker,
    identifiers: Sequence[str],
    deadline: float,
    *,
    allow_missing: bool = False,
) -> list[Container]:
    return mock._inspect(docker, identifiers, deadline, allow_missing=allow_missing)


def resolve_stack(
    docker: BoundedDocker,
    expected: ExpectedStack,
    deadline: float,
) -> tuple[list[Container], Container, Container]:
    return mock._resolve_stack(docker, expected, deadline)


def validate_container(container: Container, expected: ExpectedStack, service: str) -> None:
    mock._validate_container(container, expected, service)


def compose_argv_with_override(
    *,
    docker: str,
    project: str,
    arguments: Sequence[str],
    audit: bool = False,
) -> list[str]:
    files = [str(BASE_COMPOSE_FILE), str(COLLECT_COMPOSE_FILE)]
    if audit:
        files.append(str(AUDIT_COMPOSE_FILE))
    argv = [docker, "compose", "-p", project, "--project-directory", str(ROOT)]
    for path in files:
        argv.extend(["-f", path])
    argv.extend(arguments)
    return argv
