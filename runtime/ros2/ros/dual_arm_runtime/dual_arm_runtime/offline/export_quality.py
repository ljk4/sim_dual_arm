"""Runtime/backend log parsing and visibility quality evaluation for export."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Any


def _expand_logs(paths: list[Path]) -> list[Path]:
    expanded: list[Path] = []
    for path in paths:
        if path.is_dir():
            expanded.extend(sorted(set(path.glob("*.log*")) | set(path.rglob("events.*.jsonl"))))
        else:
            expanded.append(path)
    return expanded


def parse_log_events(paths: list[Path]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for path in _expand_logs(paths):
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                events.append(event)
    return events


def phase_observations(
    runtime_events: list[dict[str, Any]],
    observation_times: dict[int, int],
    phases: list[str] | tuple[str, ...] | None = None,
) -> dict[str, int]:
    """Map each scenario phase to the observation that first announced it.

    The phase comes from the accepted teacher chunk, which reflects the gate
    fact measured for that exact capture.  ``phases`` is the scenario's phase
    order; events for any other task's phase names are ignored.
    """
    allowed = None if phases is None else {str(value) for value in phases}
    result: dict[str, int] = {}
    by_time = {value: key for key, value in observation_times.items()}
    for event in runtime_events:
        if event.get("event") != "follower.chunk_accepted":
            continue
        fields = event.get("fields")
        if not isinstance(fields, dict):
            continue
        phase = str(fields.get("phase") or "")
        sim_time = event.get("sim_time_ns")
        observation_id = by_time.get(sim_time)
        if (
            phase
            and (allowed is None or phase in allowed)
            and observation_id is not None
            and phase not in result
        ):
            result[phase] = observation_id
    return result


def _rule_metrics(
    samples: list[tuple[int, int]], minimum: int, max_blackout_ns: int
) -> dict[str, Any]:
    if not samples:
        return {
            "samples": 0,
            "qualified_ratio": 0.0,
            "max_blackout_s": 0.0,
            "min_pixels": 0,
            "required_pixels": minimum,
            "ok": False,
        }
    passed = sum(1 for _time, pixels in samples if pixels >= minimum)
    longest = 0
    run_start: int | None = None
    for time, pixels in samples:
        if pixels < minimum:
            if run_start is None:
                run_start = time
        elif run_start is not None:
            longest = max(longest, time - run_start)
            run_start = None
    if run_start is not None:
        longest = max(longest, samples[-1][0] - run_start)
    ratio = passed / len(samples)
    return {
        "samples": len(samples),
        "qualified_ratio": round(ratio, 6),
        "max_blackout_s": round(longest / 1e9, 6),
        "min_pixels": min(pixels for _time, pixels in samples),
        "required_pixels": minimum,
    }


def _timelines(
    quality_events: list[dict[str, Any]],
) -> dict[tuple[str, str], list[tuple[int, int]]]:
    timelines: dict[tuple[str, str], list[tuple[int, int]]] = defaultdict(list)
    for event in quality_events:
        if event.get("event") != "backend.render.quality":
            continue
        fields = event.get("fields")
        if not isinstance(fields, dict):
            continue
        sim_time = event.get("sim_time_ns")
        if not isinstance(sim_time, int) or sim_time < 0:
            continue
        for name, value in fields.items():
            if not isinstance(name, str) or not name.endswith("_pixels"):
                continue
            camera, _, obj = name[: -len("_pixels")].rpartition("_")
            if camera and obj and isinstance(value, int):
                timelines[(camera, obj)].append((sim_time, value))
    for values in timelines.values():
        values.sort()
    return timelines


def _phase_intervals(
    phase_observations: dict[str, int], observation_times: dict[int, int]
) -> tuple[bool, dict[str, tuple[int, int]]]:
    ordered_times = sorted(observation_times.values())
    last_time = ordered_times[-1] if ordered_times else 0
    phase_starts = {
        phase: observation_times[observation_id]
        for phase, observation_id in phase_observations.items()
        if observation_id in observation_times
    }
    ordered_phases = sorted(phase_starts.items(), key=lambda item: item[1])
    intervals: dict[str, tuple[int, int]] = {}
    for index, (phase, start) in enumerate(ordered_phases):
        end = (
            ordered_phases[index + 1][1]
            if index + 1 < len(ordered_phases)
            else last_time + 250_000_000
        )
        intervals[phase] = (start, end)
    phase_based = bool(intervals)
    if not phase_based:
        intervals = {"global": (ordered_times[0] if ordered_times else 0, last_time + 1)}
    return phase_based, intervals


def _empty_document(
    max_blackout_ns: int, min_ratio: float, phase_observations: dict, rules: dict[str, Any]
) -> dict:
    return {
        "qualified": False,
        "phase_based": bool(phase_observations),
        "failure_reasons": ["no quality rules configured"],
        "rules": [],
        "phases": {},
        "configured_phases": _declared_phases(rules),
        "required_phases": _declared_required_phases(rules),
        "max_blackout_s": max_blackout_ns / 1e9,
        "min_qualified_ratio": min_ratio,
    }


def _declared_phases(rules: dict[str, Any]) -> list[str]:
    value = rules.get("phases")
    if not isinstance(value, list):
        return []
    return [str(item) for item in value if isinstance(item, str) and item]


def _declared_required_phases(rules: dict[str, Any]) -> list[str]:
    value = rules.get("required_phases")
    if not isinstance(value, list):
        return []
    return [str(item) for item in value if isinstance(item, str) and item]


def _evaluate_rule(
    rule: dict[str, Any],
    timelines: dict[tuple[str, str], list[tuple[int, int]]],
    phase_intervals: dict[str, tuple[int, int]],
    phase_based: bool,
    phase_observations: dict[str, int],
    max_blackout_ns: int,
    min_ratio: float,
    failures: list[str],
    phase_report: dict[str, Any],
) -> list[dict[str, Any]]:
    camera = str(rule["camera"])
    obj = str(rule["object"])
    mode = str(rule["mode"])
    if mode not in {"required", "record_only"}:
        raise ValueError(f"unsupported quality mode {mode!r}")
    minimum = int(rule.get("min_pixels", 0))
    phases = [str(value) for value in rule.get("phases", [])]
    if not phase_based:
        phases = ["global"]
    entries: list[dict[str, Any]] = []
    for phase in phases:
        interval = phase_intervals.get(phase)
        if interval is None:
            if mode == "required":
                failures.append(f"{phase}: no teacher phase interval for {camera}.{obj}")
            continue
        start, end = interval
        samples = [
            (time, pixels)
            for time, pixels in timelines.get((camera, obj), [])
            if start <= time <= end
        ]
        entry = _rule_metrics(samples, minimum, max_blackout_ns)
        entry.update({"camera": camera, "object": obj, "mode": mode, "phase": phase})
        entries.append(entry)
        phase_entry = phase_report.setdefault(
            phase,
            {
                "present": phase in phase_observations if phase_based else True,
                "start_sim_time_ns": start,
                "end_sim_time_ns": end,
                "rules": [],
            },
        )
        phase_entry["rules"].append(entry)
        if mode != "required":
            continue
        if not samples:
            failures.append(f"{phase}: no {camera}.{obj} quality samples")
        elif entry["qualified_ratio"] < min_ratio:
            failures.append(
                f"{phase}: {camera}.{obj} ratio {entry['qualified_ratio']:.3f} < {min_ratio:.3f}"
            )
        elif entry["max_blackout_s"] > max_blackout_ns / 1e9:
            failures.append(
                f"{phase}: {camera}.{obj} blackout {entry['max_blackout_s']:.3f}s "
                f"> {max_blackout_ns / 1e9:.3f}s"
            )
    return entries


def quality_summary(
    quality_events: list[dict[str, Any]],
    phase_observations: dict[str, int],
    observation_times: dict[int, int],
    rules: dict[str, Any],
    *,
    action_source: str = "",
) -> dict[str, Any]:
    """Evaluate required/record_only rules over the scenario's phase intervals.

    Phase names and the required-stage set come from the scenario quality
    recipe.  A missing phase boundary is an explicit failure: this function
    never falls back to a global visibility verdict.
    """
    max_blackout_ns = round(float(rules.get("max_blackout_s", 0.35)) * 1e9)
    min_ratio = float(rules.get("min_qualified_ratio", 0.85))
    configured_phases = _declared_phases(rules)
    required_phases = _declared_required_phases(rules)
    raw_rules = rules.get("rules")
    if not isinstance(raw_rules, list) or not raw_rules:
        return _empty_document(max_blackout_ns, min_ratio, phase_observations, rules)
    timelines = _timelines(quality_events)
    phase_based, phase_intervals = _phase_intervals(phase_observations, observation_times)
    failures: list[str] = []
    if not phase_based:
        failures.append(
            "no phase boundary from recorded follower chunk events; "
            "phase-based quality cannot be evaluated"
        )
    for phase in required_phases:
        if phase not in phase_observations:
            failures.append(f"missing required phase evidence: {phase}")
    phase_report: dict[str, Any] = {}
    rule_report: list[dict[str, Any]] = []
    for rule in raw_rules:
        if not isinstance(rule, dict):
            raise ValueError("quality rule must be an object")
        rule_report.extend(
            _evaluate_rule(
                rule,
                timelines,
                phase_intervals,
                phase_based,
                phase_observations,
                max_blackout_ns,
                min_ratio,
                failures,
                phase_report,
            )
        )
    if phase_based:
        ordered_observed = sorted(
            phase_report,
            key=lambda name: phase_intervals.get(name, (0, 0))[0],
        )
        for phase in (*configured_phases, *ordered_observed):
            if phase not in phase_report:
                phase_report[phase] = {
                    "present": phase in phase_observations,
                    "rules": [],
                }
    qualified = (
        bool(timelines) and not failures and bool(rule_report) and phase_based
    )
    return {
        "qualified": qualified,
        "phase_based": phase_based,
        "action_source": action_source,
        "failure_reasons": failures,
        "max_blackout_s": max_blackout_ns / 1e9,
        "min_qualified_ratio": min_ratio,
        "configured_phases": configured_phases,
        "required_phases": required_phases,
        "phases": phase_report,
        "rules": rule_report,
        "samples": {f"{camera}.{obj}": len(values) for (camera, obj), values in timelines.items()},
    }


__all__ = [
    "parse_log_events",
    "phase_observations",
    "quality_summary",
]
