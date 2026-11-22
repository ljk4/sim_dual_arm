"""Operator progress keeps useful state visible and preserves diagnostic output."""

import io
import os
import pty
import select
import struct
import subprocess
import sys
import termios
import textwrap
import time
import unicodedata

import pytest
from dual_arm_learning.console_progress import ConsoleProgress


class Terminal(io.StringIO):
    def isatty(self):
        return True


def test_terminal_progress_reports_counts_percent_and_eta_without_newlines():
    now = [0.0]
    stream = Terminal()
    progress = ConsoleProgress(stream, clock=lambda: now[0])
    progress.phase("转换训练集", 50, unit="条")
    for completed in range(1, 26):
        now[0] = completed * 2.0
        progress.update(completed)
        progress.render()
    line = stream.getvalue().split("\r")[-1]
    assert "25/50 条  50.0%" in line
    assert "用时 00:00:50" in line
    assert "剩余 00:00:50" in line
    assert "\n" not in stream.getvalue()
    progress.update(50)
    progress.close("完成")
    assert stream.getvalue().endswith("完成\n")
    assert stream.getvalue().count("\n") == 1


def test_resume_eta_counts_only_new_work():
    now = [0.0]
    progress = ConsoleProgress(Terminal(), clock=lambda: now[0])
    progress.phase("训练", 100, unit="step", initial=40)
    assert "剩余 --:--:--" in progress.line()
    now[0] = 20.0
    progress.update(50)
    assert "50/100 step  50.0%" in progress.line()
    assert "剩余 00:01:40" in progress.line()


def test_redirected_progress_emits_bounded_stage_summaries():
    stream = io.StringIO()
    progress = ConsoleProgress(stream)
    progress.phase("训练", 5000, unit="step")
    for step in range(1, 5001):
        progress.update(step, detail="loss=0.1")
        progress.render()
    progress.phase("保存检查点")
    progress.close("完成")
    lines = stream.getvalue().splitlines()
    assert len(lines) == 4
    assert "5000/5000 step 100.0%" in lines[1]
    assert "\x1b" not in stream.getvalue()


def test_closed_terminal_does_not_interrupt_work():
    stream = Terminal()
    progress = ConsoleProgress(stream)
    stream.close()
    progress.phase("训练", 100)
    progress.message("训练目录")
    progress.close("完成")
    assert progress.disabled


def test_real_terminal_refreshes_during_work_and_respects_its_width(tmp_path):
    log = tmp_path / "tty.log"
    script = textwrap.dedent(
        """
        import os
        import sys
        import time
        from pathlib import Path
        from dual_arm_learning.console_progress import OperatorConsole
        with OperatorConsole(Path(sys.argv[1]), "配置", []) as console:
            console.progress.phase("训练", 2, unit="step")
            console.progress.update(1, detail="loss=0.1")
            os.write(2, b"hidden-codec-message\\n")
            time.sleep(1.1)
            console.progress.update(2)
        """
    )
    master, slave = pty.openpty()
    import fcntl

    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 80, 0, 0))
    process = subprocess.Popen(
        [sys.executable, "-c", script, str(log)], stdout=subprocess.PIPE, stderr=slave,
    )
    os.close(slave)
    chunks = []
    deadline = time.monotonic() + 10
    try:
        while time.monotonic() < deadline:
            if not select.select([master], [], [], 0.2)[0]:
                continue
            try:
                chunk = os.read(master, 65536)
            except OSError:
                break
            if not chunk:
                break
            chunks.append(chunk)
        process.communicate(timeout=1)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
    assert process.returncode == 0
    output = b"".join(chunks).decode()
    assert "1/2 step  50.0%" in output  # Rendered by the timer while work is busy.
    assert "2/2 step 100.0%" in output
    assert "剩余" in output
    assert "hidden-codec-message" not in output
    assert "hidden-codec-message" in log.read_text()
    for frame in output.split("\x1b[2K")[1:]:
        line = frame.split("\r")[0]
        width = sum(2 if unicodedata.east_asian_width(c) in {"W", "F"} else 1 for c in line)
        assert width < 80  # Redirecting stdout/stderr must not lose the terminal width.


@pytest.mark.parametrize("fail", [False, True])
def test_console_captures_native_python_and_child_output_and_restores_streams(tmp_path, fail):
    log = tmp_path / "details.log"
    script = textwrap.dedent(
        """
        import logging
        import os
        import subprocess
        import sys
        from pathlib import Path
        from dual_arm_learning.console_progress import OperatorConsole

        logging.basicConfig(level=logging.INFO)
        try:
            with OperatorConsole(Path(sys.argv[1]), "测试配置", ["条数：50"]) as console:
                console.progress.phase("转换", 50, unit="条")
                print("python-detail")
                logging.info("logging-detail")
                os.write(1, b"native-stdout-detail\\n")
                os.write(2, b"native-stderr-detail\\n")
                subprocess.run([sys.executable, "-c", "print('child-detail')"], check=True)
                console.progress.update(50)
                if sys.argv[2] == "True":
                    raise RuntimeError("test-failure")
        except RuntimeError:
            print("error-summary", file=sys.stderr)
        print("stdout-restored")
        os.write(2, b"stderr-restored\\n")
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script, str(log), str(fail)],
        capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == "stdout-restored\n"
    assert "测试配置" in result.stderr
    assert str(log) in result.stderr
    assert "50/50 条 100.0%" in result.stderr
    assert "stderr-restored" in result.stderr
    details = log.read_text()
    for marker in ("python-detail", "logging-detail", "native-stdout-detail",
                   "native-stderr-detail", "child-detail"):
        assert marker in details
        assert marker not in result.stdout + result.stderr
    assert ("Traceback" in details) == fail
    assert ("error-summary" in result.stderr) == fail
    assert "stdout-restored" not in details
    assert "stderr-restored" not in details
