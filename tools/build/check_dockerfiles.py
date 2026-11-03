"""Reject embedded programs and first-party application COPY in Dockerfiles."""

from __future__ import annotations

import argparse
import json
import re
import shlex
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APPLICATION_ROOTS = {
    "components",
    "libraries",
    "contracts",
    "configs",
    "profiles",
    "tests",
    "tools",
}


def check_text(text: str) -> list[tuple[int, str]]:
    issues = []
    instruction = ""
    start = 0
    for number, line in enumerate(text.splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not instruction:
            start = number
        instruction += line.strip().removesuffix("\\") + " "
        if line.rstrip().endswith("\\"):
            continue
        command, _, body = instruction.strip().partition(" ")
        command = command.upper()
        if "<<" in body:
            issues.append((start, "embedded heredoc; put the program in tools/build"))
        if command in {"RUN", "CMD", "ENTRYPOINT"}:
            if re.search(r"\b(?:python[\d.]*|node|bash|sh)\s+-[ce]\b", body):
                issues.append((start, "inline interpreter program; call a named script"))
            if body.startswith("["):
                try:
                    arguments = json.loads(body)
                except json.JSONDecodeError:
                    arguments = []
                interpreter = Path(arguments[0]).name if arguments else ""
                if re.fullmatch(r"python[\d.]*|node|bash|sh", interpreter) and any(
                    flag in arguments for flag in ("-c", "-e")
                ):
                    issues.append((start, "inline interpreter program; call a named script"))
            if re.search(r"(?:^|[;&])\s*(?:for|while|if|case)\s", body):
                issues.append((start, "shell control flow; put it in a named build script"))
        if command in {"COPY", "ADD"} and not body.startswith("--from="):
            try:
                arguments = json.loads(body) if body.startswith("[") else shlex.split(body)
            except (ValueError, json.JSONDecodeError):
                arguments = []
            sources = [argument for argument in arguments[:-1] if not argument.startswith("--")]
            if any(
                source in {".", "./", "/", "*", "./*"}
                or source.lstrip("./").split("/")[0] in APPLICATION_ROOTS
                for source in sources
            ):
                issues.append((start, "application source must be a read-only runtime mount"))
        instruction = ""
    return issues


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    arguments = parser.parse_args()
    found = []
    for path in sorted((arguments.project_root / "deploy/docker").rglob("Dockerfile")):
        for line, message in check_text(path.read_text()):
            found.append(f"{path}:{line}: {message}")
    if found:
        print("\n".join(found))
        return 1
    print("Dockerfile boundaries passed: no application COPY or embedded programs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
