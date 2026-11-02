import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for source in (ROOT / "components/backend_sim/src", ROOT / "contracts/python/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.render_worker import Capture, RendererThread  # noqa: E402
from robot_backend_sim.rendering import Snapshot  # noqa: E402
from robot_contracts.models import RunContext  # noqa: E402


class _Renderer:
    def render_rgb(self, snapshot):
        return {"global": np.zeros((2, 2, 3), dtype=np.uint8)}

    def render_segmentation(self, snapshot):
        segmentation = np.zeros((2, 2, 2), dtype=np.int32)
        segmentation[:, :, 0] = 3
        segmentation[:, :, 1] = 5
        return {"global": segmentation}

    def close(self):
        return None


def test_renderer_counts_segmentation_on_the_same_capture():
    context = RunContext("r", "p", "b", "t", 1, "e", 1)
    snapshot = Snapshot(np.array([0.5, 0.0, 0.0]), 0.5)
    capture = Capture(
        context,
        1,
        500_000_000,
        ("joint",),
        (0.0,),
        (0.0,),
        "STARTED",
        None,
        snapshot,
    )
    thread = RendererThread(
        None,
        ("global",),
        width=2,
        height=2,
        snapshot_capacity=1,
        output_capacity=1,
        health_timeout_s=2.0,
        renderer_factory=lambda *args: _Renderer(),
        segmentation_geoms={"source": (3, 5)},
    )
    thread.start()
    try:
        thread.wait_ready(2.0)
        assert thread.submit(capture)
        rendered = ()
        for _ in range(200):
            rendered = thread.drain()
            if rendered:
                break
            time.sleep(0.005)
        assert rendered and rendered[0].capture == capture
        assert rendered[0].quality["global"]["source"] == 4
    finally:
        thread.close(2.0)
