"""Render and refresh training & validation loss curves from metrics.jsonl."""

from __future__ import annotations

import argparse
import json
import os
import sys
import uuid
from pathlib import Path
from typing import Any


def load_metrics(metrics_path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Load train and validation records from a `metrics.jsonl` file."""
    train_rows: list[dict[str, Any]] = []
    val_rows: list[dict[str, Any]] = []
    if not metrics_path.is_file():
        return train_rows, val_rows

    with metrics_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(row, dict):
                continue
            phase = row.get("phase")
            if phase == "train" and "step" in row:
                train_rows.append(row)
            elif phase == "validation" and "step" in row:
                val_rows.append(row)

    train_rows.sort(key=lambda item: int(item["step"]))
    val_rows.sort(key=lambda item: int(item["step"]))
    return train_rows, val_rows


def _moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1 or len(values) < window:
        return values
    smoothed: list[float] = []
    running_sum = 0.0
    for idx, val in enumerate(values):
        running_sum += val
        if idx >= window:
            running_sum -= values[idx - window]
        denom = min(idx + 1, window)
        smoothed.append(running_sum / denom)
    return smoothed


def render_loss_plot(
    metrics_path: Path,
    output_path: Path,
    *,
    title: str | None = None,
) -> Path | None:
    """Render train and validation loss curves to a PNG image atomically."""
    train_rows, val_rows = load_metrics(metrics_path)
    if not train_rows and not val_rows:
        return None

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_l1, ax_total) = plt.subplots(1, 2, figsize=(12.5, 4.6), dpi=130)

    # Left subplot: Train L1 vs Validation L1
    train_l1_pts = [
        (int(r["step"]), float(r["l1_loss"]))
        for r in train_rows
        if r.get("l1_loss") is not None
    ]
    if train_l1_pts:
        steps_l1 = [p[0] for p in train_l1_pts]
        vals_l1 = [p[1] for p in train_l1_pts]
        ax_l1.plot(steps_l1, vals_l1, color="#1f77b4", alpha=0.28, linewidth=1.0, label="Train L1 (raw)")
        if len(vals_l1) >= 5:
            win = max(3, min(15, len(vals_l1) // 4))
            smoothed = _moving_average(vals_l1, win)
            ax_l1.plot(steps_l1, smoothed, color="#1f77b4", linewidth=2.0, label=f"Train L1 (MA-{win})")

    val_l1_pts = [
        (int(r["step"]), float(r["val_l1"]))
        for r in val_rows
        if r.get("val_l1") is not None
    ]
    if val_l1_pts:
        v_steps = [p[0] for p in val_l1_pts]
        v_vals = [p[1] for p in val_l1_pts]
        ax_l1.plot(
            v_steps,
            v_vals,
            color="#d62728",
            marker="o",
            markersize=4.5,
            linewidth=1.8,
            label="Validation L1",
        )
        best_step, best_val = min(val_l1_pts, key=lambda p: p[1])
        ax_l1.scatter([best_step], [best_val], color="#ff7f0e", marker="*", s=140, zorder=5, label="Best Val L1")
        ax_l1.annotate(
            f"best={best_val:.4f}@{best_step}",
            xy=(best_step, best_val),
            xytext=(8, 8),
            textcoords="offset points",
            fontsize=8.5,
            color="#b24700",
        )

    ax_l1.set_title("L1 Loss (Train vs Validation)")
    ax_l1.set_xlabel("Step")
    ax_l1.set_ylabel("L1 Loss")
    ax_l1.grid(True, linestyle="--", alpha=0.35)
    if ax_l1.get_legend_handles_labels()[0]:
        ax_l1.legend(loc="upper right", fontsize=8.5)

    # Right subplot: Train Total Loss + KL Loss
    train_loss_pts = [
        (int(r["step"]), float(r["loss"]))
        for r in train_rows
        if r.get("loss") is not None
    ]
    if train_loss_pts:
        steps_tot = [p[0] for p in train_loss_pts]
        vals_tot = [p[1] for p in train_loss_pts]
        ax_total.plot(steps_tot, vals_tot, color="#2ca02c", alpha=0.30, linewidth=1.0, label="Total Loss (raw)")
        if len(vals_tot) >= 5:
            win = max(3, min(15, len(vals_tot) // 4))
            ax_total.plot(steps_tot, _moving_average(vals_tot, win), color="#2ca02c", linewidth=2.0, label=f"Total Loss (MA-{win})")

    kl_pts = [
        (int(r["step"]), float(r["kl_loss"]))
        for r in train_rows
        if r.get("kl_loss") is not None
    ]
    lines_left, labels_left = ax_total.get_legend_handles_labels()
    if kl_pts:
        ax_kl = ax_total.twinx()
        kl_steps = [p[0] for p in kl_pts]
        kl_vals = [p[1] for p in kl_pts]
        ax_kl.plot(kl_steps, kl_vals, color="#9467bd", linestyle="--", linewidth=1.4, alpha=0.85, label="KL Loss")
        ax_kl.set_ylabel("KL Loss", color="#9467bd")
        ax_kl.tick_params(axis="y", labelcolor="#9467bd")
        lines_right, labels_right = ax_kl.get_legend_handles_labels()
        lines_left.extend(lines_right)
        labels_left.extend(labels_right)

    ax_total.set_title("Total Loss & KL Loss")
    ax_total.set_xlabel("Step")
    ax_total.set_ylabel("Total Loss")
    ax_total.grid(True, linestyle="--", alpha=0.35)
    if lines_left:
        ax_total.legend(lines_left, labels_left, loc="upper right", fontsize=8.5)

    latest_step = max(
        [int(r["step"]) for r in train_rows] + [int(r["step"]) for r in val_rows]
    )
    suptitle = title or f"ACT Training Progress (step {latest_step})"
    fig.suptitle(suptitle, fontsize=11.5)
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = output_path.with_suffix(".tmp.png")
    fig.savefig(tmp_path, format="png", bbox_inches="tight")
    plt.close(fig)
    os.replace(tmp_path, output_path)
    return output_path


def display_loss_plot(image_path: Path, *, display_id: str | None = None) -> str | None:
    """Display or refresh a rendered PNG loss plot inline inside Jupyter/Colab."""
    if not image_path.is_file():
        return display_id
    try:
        from IPython.display import Image, display, update_display  # type: ignore[import-not-found]
    except ImportError:
        return display_id

    img = Image(filename=str(image_path))
    if display_id is None:
        handle_id = f"loss-plot-{uuid.uuid4().hex[:8]}"
        display(img, display_id=handle_id)
        return handle_id

    try:
        update_display(img, display_id=display_id)
    except Exception:
        display(img, display_id=display_id)
    return display_id


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render ACT train/validation loss curves from metrics.jsonl")
    parser.add_argument("metrics", type=Path, help="Path to metrics.jsonl")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path (defaults to <metrics_dir>/loss_curve.png)")
    parser.add_argument("--title", type=str, default=None, help="Optional figure title")
    args = parser.parse_args(argv)

    metrics_path = args.metrics.resolve()
    output_path = args.output.resolve() if args.output is not None else metrics_path.parent / "loss_curve.png"
    rendered = render_loss_plot(metrics_path, output_path, title=args.title)
    if rendered is None:
        print(f"[提示] 暂无可用于绘图的训练/验证记录: {metrics_path}", file=sys.stderr)
        return 1
    print(f"[完成] 已生成损失曲线: {rendered}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
