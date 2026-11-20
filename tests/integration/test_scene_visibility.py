from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.render import rendering, visibility  # noqa: E402
from robot_backend_sim.common.render.rendering import SceneRenderer, SceneRenderError  # noqa: E402
from robot_backend_sim.marvin_sim.stack import visibility as marvin_visibility  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene import build_marvin_stack_scene  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene_config import load_marvin_stack_scene_config  # noqa: E402
from robot_backend_sim.tools import check_scene  # noqa: E402

CONFIG_PATH = ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/scene/marvin_stack.yaml"
CHECK_PATH = ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/checks/scene_check.yaml"


@pytest.fixture(scope="module")
def scene_config():
    return load_marvin_stack_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_marvin_stack_scene(
        scene_config,
    )


@pytest.fixture(scope="module")
def renderer(scene, scene_config):
    first = scene_config.cameras[0]
    renderer = SceneRenderer(
        scene.model,
        [camera.name for camera in scene_config.cameras],
        width=first.width,
        height=first.height,
    )
    yield renderer
    renderer.close()


def test_rgb_and_segmentation_come_from_one_immutable_snapshot(scene, renderer):
    scene.reset(11)
    snapshot = renderer.capture(scene.data)
    original_state = snapshot.state.copy()
    scene_state = {
        "qpos": scene.data.qpos.copy(),
        "qvel": scene.data.qvel.copy(),
        "time": float(scene.data.time),
    }

    rgb_before = renderer.render_rgb(snapshot)
    segmentation = renderer.render_segmentation(snapshot)
    rgb_after = renderer.render_rgb(snapshot)

    assert snapshot.state.flags.writeable is False
    assert set(rgb_before) == {"global", "left_wrist", "right_wrist"}
    assert set(segmentation) == set(rgb_before)
    for camera, image in rgb_before.items():
        assert image.dtype == np.uint8
        assert image.shape == (240, 320, 3)
        assert image.flags.writeable is False
        assert segmentation[camera].shape == (240, 320, 2)
        assert segmentation[camera].dtype.kind in ("i", "u")
        # EGL can round a channel by one 8-bit level after switching render modes.
        np.testing.assert_allclose(image, rgb_after[camera], rtol=0, atol=1)

    # Rendering only touched the renderer's private data.
    np.testing.assert_array_equal(scene.data.qpos, scene_state["qpos"])
    np.testing.assert_array_equal(scene.data.qvel, scene_state["qvel"])
    assert scene.data.time == scene_state["time"]

    # The old snapshot is independent of later changes to the physics data.
    scene.data.joint("source_free").qpos[0] += 0.01
    mujoco.mj_forward(scene.model, scene.data)
    assert not np.array_equal(renderer.capture(scene.data).state, original_state)
    scene.reset(23)
    np.testing.assert_array_equal(snapshot.state, original_state)

    check = marvin_visibility.load_check_config(CHECK_PATH, scene.config)
    geom_ids = {obj.name: scene.model.geom(f"{obj.name}_geom").id for obj in scene.config.objects}
    counts = {
        camera: {
            obj: visibility.count_visible_pixels(segmentation[camera], geom_id)
            for obj, geom_id in geom_ids.items()
        }
        for camera in segmentation
    }
    assert counts["global"]["source"] > 0
    assert counts["global"]["target"] > 0
    assert check.threshold("global", "source") == 32
    assert check.threshold("global", "target") == 64


def test_snapshot_time_matches_reported_state_and_mismatch_is_rejected(scene, renderer):
    scene.reset(11)
    scene.data.time = 3.5
    mujoco.mj_forward(scene.model, scene.data)
    snapshot = renderer.capture(scene.data)

    assert snapshot.state[0] == pytest.approx(3.5)
    assert snapshot.sim_time == snapshot.state[0]
    derived = renderer.derive_snapshot(snapshot, {})
    assert derived.state[0] == pytest.approx(3.5)
    assert derived.sim_time == derived.state[0]

    tampered = snapshot.state.copy()
    tampered[0] = 0.0
    with pytest.raises(ValueError, match="TIME"):
        rendering.Snapshot(state=tampered, sim_time=3.5)
    accepted = rendering.Snapshot(state=tampered, sim_time=0.0)
    assert accepted.sim_time == accepted.state[0] == 0.0
    scene.reset(11)


def test_full_check_entry_with_nominal_layout(tmp_path):
    arguments = check_scene.SceneCheckArguments(
        scene_config=CONFIG_PATH,
        check_config=CHECK_PATH,
        report_root=tmp_path / "reports",
        run_id="backend-one-layout",
    )

    code = check_scene.run(arguments)

    report = json.loads((arguments.report_root / arguments.run_id / "report.json").read_text())
    assert code == 0
    assert report["status"] == "succeeded"
    assert report["kind"] == "scene"
    assert report["logs_closed"] is True
    assert report["layout_summary"]["count"] == 1
    assert report["layout_summary"]["passed"] == 1
    assert len(report["layouts"]) == 1
    assert report["throughput"]["measured_groups"] == 20
    assert report["gl"]["nvidia"] is True
    layout = report["layouts"][0]
    assert layout["layout_id"] == "nominal"
    assert set(layout["rgb"]) == {"global", "left_wrist", "right_wrist"}
    for relative in layout["rgb"].values():
        assert (arguments.report_root / arguments.run_id / relative).is_file()
    assert (arguments.report_root / arguments.run_id / report["artifact"]).is_file()


def test_close_is_idempotent_and_render_after_close_is_clear(scene, scene_config):
    first = scene_config.cameras[0]
    renderer = SceneRenderer(
        scene.model,
        [camera.name for camera in scene_config.cameras],
        width=first.width,
        height=first.height,
    )
    scene.reset(11)
    snapshot = renderer.capture(scene.data)
    renderer.close()
    renderer.close()
    with pytest.raises(SceneRenderError, match="render called after close"):
        renderer.render_rgb(snapshot)
