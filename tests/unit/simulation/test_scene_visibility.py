from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from model_helpers import replace
from pydantic import ValidationError

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.render import visibility  # noqa: E402
from robot_backend_sim.marvin_sim.stack import visibility as marvin_visibility  # noqa: E402
from robot_backend_sim.marvin_sim.stack.checks import check_scene  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene_config import (  # noqa: E402
    load_marvin_stack_scene_config,
)

SCENE_CONFIG = ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/scene/marvin_stack.yaml"
CHECK_CONFIG = ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/checks/scene_check.yaml"


def write_yaml(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / "scene_check.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def default_document() -> dict:
    return yaml.safe_load(CHECK_CONFIG.read_text(encoding="utf-8"))


def scene_config():
    return load_marvin_stack_scene_config(SCENE_CONFIG)


def mutated(tmp_path: Path, mutate) -> Path:
    document = copy.deepcopy(default_document())
    mutate(document)
    return write_yaml(tmp_path, document)


def set_value(*path, value):
    def mutate(document):
        target = document
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value

    return mutate


def test_default_check_config_is_typed_frozen_and_cross_checked():
    config = marvin_visibility.load_check_config(CHECK_CONFIG, scene_config())

    assert config.camera_names == ("global", "left_wrist", "right_wrist")
    assert config.object_names == ("source", "target")
    assert config.threshold("global", "source") == 32
    assert config.threshold("left_wrist", "target") == 16
    assert config.threshold("right_wrist", "source") == 32
    assert config.throughput.warmup_groups == 3
    assert config.throughput.measure_groups == 20
    assert config.camera_resolutions["right_wrist"] == (320, 240)

    with pytest.raises(TypeError):
        config.min_visible_pixels["global"]["source"] = 1  # type: ignore[index]
    with pytest.raises(ValidationError):
        config.throughput = None  # type: ignore[misc,assignment]


@pytest.mark.parametrize(
    ("mutate", "field"),
    [
        (set_value("throughput", "measure_groups", value=0), "throughput.measure_groups"),
        (set_value("throughput", "warmup_groups", value=True), "throughput.warmup_groups"),
        (set_value("throughput", "max_elapsed_s", value=0.0), "throughput.max_elapsed_s"),
        (set_value("throughput", "min_fps", value=0.0), "throughput.min_fps"),
    ],
)
def test_check_config_rejects_invalid_typed_values(tmp_path, mutate, field):
    path = mutated(tmp_path, mutate)
    with pytest.raises(marvin_visibility.SceneCheckConfigError) as caught:
        marvin_visibility.load_check_config(path, scene_config())
    assert caught.value.field == field
    assert str(path) in str(caught.value)


@pytest.mark.parametrize(
    "value",
    [True, 0, -1, math.nan, math.inf, 320 * 240 + 1, 10**30],
)
def test_thresholds_reject_bool_zero_negative_nonfinite_and_huge(tmp_path, value):
    path = mutated(tmp_path, set_value("min_visible_pixels", "global", "source", value=value))
    with pytest.raises(marvin_visibility.SceneCheckConfigError) as caught:
        marvin_visibility.load_check_config(path, scene_config())
    assert caught.value.field == "min_visible_pixels.global.source"


def test_threshold_names_must_match_scene_cameras_and_objects(tmp_path):
    unknown_camera = mutated(tmp_path, set_value("min_visible_pixels", "overhead", value={}))
    with pytest.raises(marvin_visibility.SceneCheckConfigError) as caught:
        marvin_visibility.load_check_config(unknown_camera, scene_config())
    assert caught.value.field == "min_visible_pixels"

    duplicate_object = mutated(
        tmp_path,
        set_value("min_visible_pixels", "global", value={"source": 32, "target": 64, "other": 1}),
    )
    with pytest.raises(marvin_visibility.SceneCheckConfigError) as caught:
        marvin_visibility.load_check_config(duplicate_object, scene_config())
    assert caught.value.field == "min_visible_pixels.global"

    missing_object = mutated(
        tmp_path,
        set_value("min_visible_pixels", "global", value={"source": 32}),
    )
    with pytest.raises(marvin_visibility.SceneCheckConfigError) as caught:
        marvin_visibility.load_check_config(missing_object, scene_config())
    assert caught.value.field == "min_visible_pixels.global"


def test_unknown_and_duplicate_yaml_keys_are_rejected(tmp_path):
    path = mutated(tmp_path, set_value("unexpected", value=1))
    with pytest.raises(marvin_visibility.SceneCheckConfigError) as caught:
        marvin_visibility.load_check_config(path, scene_config())
    assert caught.value.field == "unexpected"

    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text(
        CHECK_CONFIG.read_text(encoding="utf-8") + "\nthroughput: {}\n", encoding="utf-8"
    )
    with pytest.raises(marvin_visibility.SceneCheckConfigError, match="duplicate key"):
        marvin_visibility.load_check_config(duplicate, scene_config())


def test_pixel_count_uses_exact_objid_and_objtype_not_colors():
    segmentation = [
        [[7, visibility.GEOM_OBJECT_TYPE], [7, visibility.GEOM_OBJECT_TYPE], [8, 5]],
        [[7, 1], [4, 2], [7, visibility.GEOM_OBJECT_TYPE]],
    ]
    assert visibility.count_visible_pixels(segmentation, 7) == 3
    assert visibility.count_visible_pixels(segmentation, 8) == 1
    assert visibility.count_visible_pixels(segmentation, 4) == 0

    with pytest.raises(visibility.VisibilityError, match=r"shape .*2\)|2-channel"):
        visibility.count_visible_pixels([[[1, 2, 3]]], 1)
    with pytest.raises(visibility.VisibilityError, match="integers"):
        visibility.count_visible_pixels([[[1.0, 5.0]]], 1)


def test_zero_pixels_fails_a_camera_and_layout():
    check_config = marvin_visibility.load_check_config(CHECK_CONFIG, scene_config())
    segmentation = [
        [[0, 0], [0, 0], [0, 0]],
        [[0, 0], [0, 0], [0, 0]],
    ]
    result = marvin_visibility.evaluate_layout_visibility(
        "seed-1",
        {camera: segmentation for camera in check_config.camera_names},
        {"source": 100, "target": 101},
        check_config,
    )
    assert result.total_pixels == 0
    assert result.minimum_margin == -64
    assert result.passed is False
    assert all(camera.passed is False for camera in result.cameras)


def test_contact_sheet_contains_one_row_of_nominal_camera_views(tmp_path):
    from PIL import Image

    config = marvin_visibility.load_check_config(CHECK_CONFIG, scene_config())
    run_dir = tmp_path / "run"
    rgb = {}
    for camera in config.camera_names:
        path = run_dir / "artifacts/layouts/nominal" / f"{camera}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.new("RGB", (8, 6), (20, 20, 30)).save(path)
        rgb[camera] = path.relative_to(run_dir).as_posix()
    layout = {"layout_id": "nominal", "total_pixels": 300, "passed": True, "rgb": rgb}

    relative = check_scene._write_contact_sheet(run_dir, layout, config)
    assert relative == "artifacts/contact_sheet.png"
    with Image.open(run_dir / relative) as image:
        assert image.size == (52, 52)


class _TestRGB:
    """Minimal array-interface object so Pillow runs without numpy on the host."""

    def __init__(self, width: int, height: int, color: tuple[int, int, int]) -> None:
        self.width = width
        self.height = height
        self._data = bytes(color) * (width * height)

    @property
    def __array_interface__(self):
        return {
            "shape": (self.height, self.width, 3),
            "typestr": "|u1",
            "strides": (self.width * 3, 3, 1),
            "version": 3,
        }

    def tobytes(self) -> bytes:
        return self._data


def test_save_rgb_images_writes_three_real_pngs_and_protects_layout_directory(tmp_path):
    from PIL import Image

    run_dir = tmp_path / "run"
    layout = marvin_visibility.NOMINAL_LAYOUT
    rgb = {
        camera: _TestRGB(40 + index, 30, (10 + index, 20, 30))
        for index, camera in enumerate(("global", "left_wrist", "right_wrist"))
    }

    paths = check_scene._save_rgb_images(run_dir, layout, rgb)

    assert set(paths) == set(rgb)
    for index, camera in enumerate(rgb):
        path = run_dir / paths[camera]
        assert path.is_file()
        with Image.open(path) as image:
            assert image.format == "PNG"
            assert image.size == (40 + index, 30)
    before = {camera: (run_dir / path).read_bytes() for camera, path in paths.items()}

    with pytest.raises(check_scene.SceneCheckFailure) as caught:
        check_scene._save_rgb_images(run_dir, layout, rgb)
    assert caught.value.reason == "artifact_path_exists"
    assert {camera: (run_dir / path).read_bytes() for camera, path in paths.items()} == before


def test_save_rgb_images_write_error_is_artifact_write_failed(tmp_path, monkeypatch):
    from PIL import Image

    run_dir = tmp_path / "run"
    layout = marvin_visibility.NOMINAL_LAYOUT
    rgb = {camera: _TestRGB(8, 6, (1, 2, 3)) for camera in ("global", "left_wrist")}

    def fail_save(*_args, **_kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(Image.Image, "save", fail_save)
    with pytest.raises(check_scene.SceneCheckFailure) as caught:
        check_scene._save_rgb_images(run_dir, layout, rgb)
    assert caught.value.reason == "artifact_write_failed"
    assert "disk full" in caught.value.message


def test_contact_sheet_write_error_has_clear_reason(tmp_path, monkeypatch):
    from PIL import Image

    config = marvin_visibility.load_check_config(CHECK_CONFIG, scene_config())
    run_dir = tmp_path / "run"
    layout = marvin_visibility.NOMINAL_LAYOUT
    paths = check_scene._save_rgb_images(
        run_dir,
        layout,
        {camera: _TestRGB(8, 6, (9, 8, 7)) for camera in config.camera_names},
    )
    layout_report = {
        "layout_id": layout.layout_id,
        "total_pixels": 300,
        "passed": True,
        "rgb": paths,
    }

    def fail_save(*_args, **_kwargs):
        raise OSError("sheet disk full")

    monkeypatch.setattr(Image.Image, "save", fail_save)
    with pytest.raises(check_scene.SceneCheckFailure) as caught:
        check_scene._write_contact_sheet(run_dir, layout_report, config)
    assert caught.value.reason == "contact_sheet_write_failed"
    assert "sheet disk full" in caught.value.message


def test_yaw_quaternion_is_normalized_and_has_both_signs():
    for yaw in (-10.0, 0.0, 10.0):
        quaternion = visibility.yaw_quaternion(yaw)
        assert sum(value * value for value in quaternion) == pytest.approx(1.0)
    assert visibility.yaw_quaternion(-10.0)[3] < 0
    assert visibility.yaw_quaternion(10.0)[3] > 0


def check_arguments(tmp_path: Path) -> check_scene.SceneCheckArguments:
    (tmp_path / "reports").mkdir()
    return check_scene.SceneCheckArguments(
        scene_config=SCENE_CONFIG,
        check_config=CHECK_CONFIG,
        report_root=tmp_path / "reports",
        run_id="scene-test",
    )


class FakeLogger:
    def __init__(self, directory: Path, *, fail_close: bool = False) -> None:
        self.directory = directory
        self.instance_id = directory.name
        self.fail_close = fail_close
        self.events: list[str] = []

    def emit(self, event, **_kwargs) -> bool:
        self.directory.mkdir(parents=True, exist_ok=True)
        self.events.append(event)
        return True

    def check_health(self) -> None:
        return None

    def close(self) -> None:
        if self.fail_close:
            raise RuntimeError("close boom")


class FakeModel:
    nq = 32
    nv = 30
    nu = 16
    ncam = 3
    ngeom = 20
    nkey = 0

    def __init__(self) -> None:
        self.vis = SimpleNamespace(global_=SimpleNamespace(offwidth=320, offheight=240))

    def geom(self, name: str):
        return SimpleNamespace(id={"source_geom": 100, "target_geom": 101}[name])

    def camera(self, name: str):
        return SimpleNamespace(id={"global": 0, "left_wrist": 1, "right_wrist": 2}[name])


class FakeScene:
    def __init__(self) -> None:
        self.model = FakeModel()
        self.indices = SimpleNamespace(
            environment=SimpleNamespace(object_free_joint_qpos={"source": 18, "target": 25})
        )
        self.data = SimpleNamespace(qpos=[0.0] * 32)
        self.reset(11)

    def reset(self, _seed: int) -> None:
        qpos = [0.0] * 32
        qpos[18:25] = [0.55, -0.08, 0.80, 1.0, 0.0, 0.0, 0.0]
        qpos[25:32] = [0.55, -0.30, 0.81, 1.0, 0.0, 0.0, 0.0]
        self.data.qpos = qpos


class FakeRenderer:
    def __init__(self, *, fail_close: bool = False) -> None:
        self.gl_info = SimpleNamespace(
            vendor="NVIDIA Corporation", renderer="fake", version="4.6", nvidia=True
        )
        self.camera_names = ("global", "left_wrist", "right_wrist")
        self.width = 320
        self.height = 240
        self.fail_close = fail_close
        self.closed = False

    def capture(self, _data):
        return object()

    def render_rgb(self, _snapshot):
        return {camera: None for camera in self.camera_names}

    def render_segmentation(self, _snapshot):
        return {camera: None for camera in self.camera_names}

    def close(self) -> None:
        self.closed = True
        if self.fail_close:
            raise RuntimeError("renderer close boom")


def fake_evaluate_layout_visibility(layout_id, _segmentations, geom_ids, check_config):
    cameras = []
    for camera in check_config.camera_names:
        items = tuple(
            marvin_visibility.ObjectVisibility(
                object=name,
                geom_id=geom_ids[name],
                pixels=check_config.threshold(camera, name),
                threshold=check_config.threshold(camera, name),
                passed=True,
            )
            for name in check_config.object_names
        )
        width, height = check_config.camera_resolutions[camera]
        cameras.append(
            marvin_visibility.CameraVisibility(
                camera=camera,
                width=width,
                height=height,
                objects=items,
                total_pixels=sum(item.pixels for item in items),
                passed=True,
            )
        )
    total = sum(camera.total_pixels for camera in cameras)
    return marvin_visibility.LayoutVisibility(
        layout_id=layout_id,
        cameras=tuple(cameras),
        total_pixels=total,
        minimum_margin=0,
        passed=True,
    )


def install_success_fakes(monkeypatch, tmp_path, *, renderer=None):
    renderer = renderer or FakeRenderer()
    monkeypatch.setattr(
        check_scene.probe_environment,
        "check_dependencies",
        lambda: {"python": {"version": "3.12"}, "packages": {}, "imports": {}},
    )
    monkeypatch.setattr(check_scene.probe_environment, "_verify_mounted_source", lambda *a: None)
    monkeypatch.setattr(check_scene, "_build_scene", lambda _config: FakeScene())
    monkeypatch.setattr(check_scene, "_make_renderer", lambda _scene, _config: renderer)
    monkeypatch.setattr(
        check_scene.visibility,
        "evaluate_layout_visibility",
        fake_evaluate_layout_visibility,
    )
    monkeypatch.setattr(
        check_scene,
        "_save_rgb_images",
        lambda _run_dir, layout, _rgb: {
            camera: f"artifacts/layouts/{layout.layout_id}/{camera}.png"
            for camera in renderer.camera_names
        },
    )
    monkeypatch.setattr(
        check_scene,
        "_write_contact_sheet",
        lambda *_args: "artifacts/contact_sheet.png",
    )
    return renderer


def test_run_success_publishes_report_after_logger_close(tmp_path, monkeypatch):
    arguments = check_arguments(tmp_path)
    renderer = install_success_fakes(monkeypatch, tmp_path)
    logger = FakeLogger(arguments.report_root / arguments.run_id / "logs")
    monkeypatch.setattr(check_scene, "_open_logger", lambda *_args: logger)

    code = check_scene.run(arguments)

    report = json.loads((arguments.report_root / arguments.run_id / "report.json").read_text())
    assert code == 0
    assert report["status"] == "succeeded"
    assert report["exit_code"] == 0
    assert report["kind"] == "scene"
    assert report["logs_closed"] is True
    assert report["artifact"] == "artifacts/contact_sheet.png"
    assert report["layout_summary"] == {
        "count": 1,
        "passed": 1,
        "failed": 0,
    }
    assert report["throughput"]["measured_groups"] == 20
    assert [layout["layout_id"] for layout in report["layouts"]] == ["nominal"]
    assert renderer.closed is True
    assert logger.events[-1] == "checks_finished"
    assert not any("complete" in event or "success" in event for event in logger.events)


@pytest.mark.parametrize("failure", ["renderer", "logger"])
def test_close_failures_turn_success_into_nonzero_exit(tmp_path, monkeypatch, failure):
    arguments = check_arguments(tmp_path)
    renderer = install_success_fakes(
        monkeypatch, tmp_path, renderer=FakeRenderer(fail_close=failure == "renderer")
    )

    logger = FakeLogger(
        arguments.report_root / arguments.run_id / "logs",
        fail_close=failure == "logger",
    )
    monkeypatch.setattr(check_scene, "_open_logger", lambda *_args: logger)

    code = check_scene.run(arguments)

    report = json.loads((arguments.report_root / arguments.run_id / "report.json").read_text())
    assert code == 1
    assert report["status"] == "failed"
    assert report["reason"] == f"{failure}_close_failed"
    assert renderer.closed is True
    if failure == "logger":
        assert report["logs_closed"] is False
    else:
        assert report["logs_closed"] is True


def test_report_write_failure_is_nonzero_and_loud(tmp_path, monkeypatch, capsys):
    arguments = check_arguments(tmp_path)
    bad_scene = tmp_path / "bad_scene.yaml"
    bad_scene.write_text("physics: [\n", encoding="utf-8")
    arguments = replace(arguments, scene_config=bad_scene)
    logger = FakeLogger(arguments.report_root / arguments.run_id / "logs")
    monkeypatch.setattr(check_scene, "_open_logger", lambda *_args: logger)
    monkeypatch.setattr(
        check_scene,
        "_write_json_atomic",
        lambda *_args: (_ for _ in ()).throw(OSError("disk full")),
    )

    code = check_scene.run(arguments)

    captured = capsys.readouterr()
    assert code == 1
    assert "report_write_failed" in captured.err
    assert "disk full" in captured.err


def test_run_refuses_an_existing_run_directory_before_starting(tmp_path):
    arguments = check_arguments(tmp_path)
    (arguments.report_root / arguments.run_id).mkdir()
    with pytest.raises(check_scene.SceneCheckUsageError, match="already exists"):
        check_scene.run(arguments)


def test_python_source_manifest_tracks_code_and_is_independent_of_mount_location(tmp_path):
    first = tmp_path / "host-package"
    second = tmp_path / "container-package"
    first.mkdir()
    second.mkdir()
    for directory in (first, second):
        (directory / "a.py").write_text("VALUE = 1\n")
        (directory / "b.py").write_text("OTHER = 2\n")
    original = check_scene._python_source_manifest(first)
    assert original["sha256"] == check_scene._python_source_manifest(second)["sha256"]
    (first / "preview.png").write_bytes(b"diagnostic image")
    assert check_scene._python_source_manifest(first)["sha256"] == original["sha256"]
    (first / "b.py").write_text("OTHER = 3\n")
    changed = check_scene._python_source_manifest(first)
    assert changed["sha256"] != original["sha256"]
    assert changed["files"]["a.py"] == original["files"]["a.py"]
    assert changed["files"]["b.py"] != original["files"]["b.py"]
