from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")
pytest.importorskip("mink")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.kinematics_config import load_kinematics_config  # noqa: E402
from robot_backend_sim.config.scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.kinematics import kinematics  # noqa: E402
from robot_backend_sim.physics.scene import build_scene  # noqa: E402
from robot_backend_sim.render.rendering import SceneRenderer  # noqa: E402
from robot_backend_sim.tools import check_kinematics  # noqa: E402

CONFIG_PATH = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
CHECK_PATH = ROOT / "simulation/mujoco/configs/checks/scene_check.yaml"
KINEMATICS_PATH = ROOT / "simulation/mujoco/configs/checks/kinematics_check.yaml"


@pytest.fixture(scope="module")
def scene_config():
    return load_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def kinematics_config(scene_config):
    return load_kinematics_config(KINEMATICS_PATH, scene_config)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_scene(
        scene_config,
    )


@pytest.fixture(scope="module")
def renderer(scene, scene_config):
    first = scene_config.cameras[0]
    instance = SceneRenderer(
        scene.model,
        [camera.name for camera in scene_config.cameras],
        width=first.width,
        height=first.height,
    )
    yield instance
    instance.close()


def initial_state(scene, scene_config, seed: int = 11):
    scene.reset(seed)
    qpos = scene.data.qpos.copy()
    object_addresses = {
        obj.name: scene.indices.object_free_joint_qpos[obj.name] for obj in scene_config.objects
    }
    poses = {
        name: kinematics.extract_object_pose(qpos, address, name)
        for name, address in object_addresses.items()
    }
    return qpos, poses


def make_targets(scene_config, kinematics_config, poses):
    source = scene_config.objects[0]
    target = scene_config.objects[1]
    return kinematics.build_targets(
        poses["source"],
        poses["target"],
        table_top_z=scene_config.table.top_z,
        source_half_z=source.half_extents[2],
        target_half_z=target.half_extents[2],
        config=kinematics_config.targets,
    )


def test_real_mink_solves_six_targets_and_freezes_every_other_dof(
    scene, scene_config, kinematics_config
):
    qpos, poses = initial_state(scene, scene_config)
    scene_data_before = {
        "qpos": scene.data.qpos.copy(),
        "qvel": scene.data.qvel.copy(),
        "time": float(scene.data.time),
    }
    pair_set = kinematics.build_collision_pairs(scene.model)
    targets = make_targets(scene_config, kinematics_config, poses)
    results = kinematics.solve_key_poses(
        scene.model,
        qpos,
        targets,
        kinematics_config.ik,
        kinematics_config.clearance,
        pair_set,
    )
    assert len(results) == 6
    assert all(result.converged for result in results)
    for result in results:
        assert result.position_error_m <= kinematics_config.ik.position_tolerance_m
        assert result.orientation_error_rad <= kinematics_config.ik.orientation_tolerance_rad
        assert result.frozen_max_deviation <= 1e-8
        assert np.isfinite(np.asarray(result.qpos_right)).all()
    # Only the seven right hinge qpos addresses may differ from the snapshot.
    right_addresses = set(scene.indices.arm_joint_qpos["right"])
    final = np.asarray(results[-1].qpos_full)
    for address in range(scene.model.nq):
        if address not in right_addresses:
            assert final[address] == pytest.approx(qpos[address], abs=1e-8)
    np.testing.assert_array_equal(scene.data.qpos, scene_data_before["qpos"])
    np.testing.assert_array_equal(scene.data.qvel, scene_data_before["qvel"])
    assert scene.data.time == scene_data_before["time"]


def test_unreachable_target_fails_instead_of_reporting_false_success(
    scene, scene_config, kinematics_config
):
    qpos, poses = initial_state(scene, scene_config)
    pair_set = kinematics.build_collision_pairs(scene.model)
    target = dataclasses.replace(
        make_targets(scene_config, kinematics_config, poses)[0],
        phase="unreachable",
        position=(5.0, 5.0, 5.0),
    )
    results = kinematics.solve_key_poses(
        scene.model,
        qpos,
        (target,),
        kinematics_config.ik,
        kinematics_config.clearance,
        pair_set,
    )
    assert len(results) == 1
    assert results[0].converged is False
    assert results[0].reason in {"ik_not_converged", "ik_solver_failed"}


def test_real_penetrating_endpoint_is_rejected_by_path_clearance(
    scene, scene_config, kinematics_config
):
    qpos, _poses = initial_state(scene, scene_config)
    pair_set = kinematics.build_collision_pairs(scene.model)
    addresses = tuple(scene.indices.arm_joint_qpos["right"])
    home = tuple(float(qpos[address]) for address in addresses)
    penetrating = None
    for joint2 in (-1.5, -1.0, -0.5, 0.0):
        for joint4 in (-3.0, -2.5, -2.0, -1.5):
            for joint6 in (0.0, 1.0, 2.0, 3.0):
                candidate = (0.0, joint2, 0.0, joint4, 0.0, joint6, 0.0)
                sample = np.array(qpos, copy=True)
                for address, value in zip(addresses, candidate, strict=True):
                    sample[address] = value
                check = kinematics.check_initial_pose(
                    scene.model,
                    sample,
                    pair_set,
                    kinematics_config.clearance,
                )
                if not check.passed:
                    penetrating = candidate
                    break
            if penetrating is not None:
                break
        if penetrating is not None:
            break
    assert penetrating is not None, "deterministic grid found no penetrating pose"
    path = kinematics.check_path(
        scene.model,
        qpos,
        home,
        penetrating,
        pair_set,
        kinematics_config.clearance,
        segment="real_penetration_probe",
    )
    assert path.passed is False
    assert path.failure_reason == "clearance_below_minimum"
    assert path.minimum.distance_m < kinematics_config.clearance.minimum_clearance_m


def test_plane_with_zero_rbound_is_not_culled_and_reports_penetration():
    max_model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="1 1 0.1"/>
            <body name="right_probe" pos="0 0 0.05">
              <joint name="right_joint1" type="hinge" axis="0 1 0" range="-1 1"/>
              <geom name="right_ball" type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(max_model)
    mujoco.mj_forward(max_model, data)
    pair_set = kinematics.CollisionPairSet(
        pairs=(
            kinematics.CollisionPair(
                geom1=0,
                geom2=1,
                body1="world",
                body2="right_probe",
                geom1_name="floor",
                geom2_name="right_ball",
                category="robot_scene",
            ),
        ),
        categories={"robot_scene": 1},
        exclusions={},
        total_physical_pairs=1,
        explicit_pair_count=0,
    )
    point = kinematics.measure_clearance(
        max_model,
        data,
        pair_set,
        detection_distance_m=0.03,
        sample_index=0,
        fraction=0.0,
    )
    assert max_model.geom_rbound[0] == 0.0
    assert point.measured_pairs == 1
    assert point.lower_bound is False
    assert point.distance_m < 0.0
    assert point.distance_m < 0.002


def test_one_layout_end_to_end_with_real_logger_scene_renderer_and_png(
    tmp_path, scene_config, kinematics_config, monkeypatch
):
    all_layouts = check_kinematics.visibility.build_layouts(
        check_kinematics.visibility.load_check_config(CHECK_PATH, scene_config),
        scene_config,
    )
    monkeypatch.setattr(
        check_kinematics.visibility,
        "build_layouts",
        lambda _check, _scene: all_layouts[:1],
    )

    def load_one_layout(arguments):
        scene = load_scene_config(arguments.scene_config)
        layout_config = check_kinematics.visibility.load_check_config(arguments.check_config, scene)
        kin = load_kinematics_config(arguments.kinematics_config, scene)
        return scene, layout_config, kin

    monkeypatch.setattr(check_kinematics, "_load_configs", load_one_layout)
    arguments = check_kinematics.KinematicsCheckArguments(
        scene_config=CONFIG_PATH,
        check_config=CHECK_PATH,
        kinematics_config=KINEMATICS_PATH,
        report_root=tmp_path / "reports",
        run_id="e2e-layout",
    )
    code = check_kinematics.run(arguments)
    run_dir = arguments.report_root / arguments.run_id
    report = json.loads((run_dir / "report.json").read_text())
    assert code == 0
    assert report["status"] == "succeeded"
    assert report["logs_closed"] is True
    assert len(report["layouts"]) == 1
    assert report["layouts"][0]["passed"] is True, report["layouts"][0]["failures"]
    assert report["artifacts"]["png_count"] == 19
    assert report["artifacts"]["overview"] is not None
    assert (run_dir / "artifacts" / "overview.png").is_file()
    log_dir = run_dir / report["log_directory"]
    assert log_dir.is_dir()
    assert any(log_dir.glob("events*.jsonl"))
    for phase in (
        "pregrasp",
        "grasp_pose",
        "lift_pose",
        "place_hover",
        "place_pose",
        "retreat",
    ):
        for camera in scene_config.cameras:
            assert (
                run_dir / "artifacts" / "layouts" / "seed-11" / phase / f"{camera.name}.png"
            ).is_file()


def test_collision_pair_enumeration_records_mujoco_exclusions(scene):
    pair_set = kinematics.build_collision_pairs(scene.model)
    assert pair_set.pairs
    assert len(pair_set.pairs) == 1148
    assert pair_set.categories.get("robot_self", 0) > 0
    assert pair_set.categories.get("robot_cross", 0) > 0
    assert pair_set.categories.get("robot_scene", 0) > 0
    assert pair_set.exclusions.get("visual_or_no_affinity", 0) > 0
    assert pair_set.exclusions.get("parent_child", 0) > 0
    # Fixed world attachments are already welded to world in this model; the
    # fixed-connection rule is an additional guard, not a required nonzero
    # count for the current Panda asset.
    assert (
        pair_set.exclusions.get("welded_body", 0) > 0
        or pair_set.exclusions.get("fixed_connection", 0) > 0
    )


def test_real_segmentation_fails_an_impossible_visibility_threshold(
    scene, scene_config, kinematics_config, renderer
):
    qpos, poses = initial_state(scene, scene_config)
    pair_set = kinematics.build_collision_pairs(scene.model)
    targets = make_targets(scene_config, kinematics_config, poses)
    result = kinematics.solve_key_poses(
        scene.model,
        qpos,
        targets[:1],
        kinematics_config.ik,
        kinematics_config.clearance,
        pair_set,
    )[0]
    assert result.converged
    snapshot = renderer.capture(scene.data)
    addresses = tuple(scene.indices.arm_joint_qpos["right"])
    updates = {
        address: (value,) for address, value in zip(addresses, result.qpos_right, strict=True)
    }
    pose_snapshot = renderer.derive_snapshot(snapshot, updates)
    segmentation = renderer.render_segmentation(pose_snapshot)
    strict_rules = tuple(
        dataclasses.replace(rule, min_pixels=10**6) if not rule.record_only else rule
        for rule in kinematics_config.visibility_rules
    )
    strict_config = dataclasses.replace(kinematics_config, visibility_rules=strict_rules)
    geom_ids = {
        "source": int(scene.model.geom("source_geom").id),
        "target": int(scene.model.geom("target_geom").id),
        "table": int(scene.model.geom("table_top").id),
    }
    rgb_paths = {camera: f"artifacts/{camera}.png" for camera in segmentation}
    document = check_kinematics._vision_document(
        "seed-11",
        result.phase,
        segmentation,
        geom_ids,
        strict_config,
        rgb_paths,
    )
    assert document["passed"] is False
    assert document["first_failure"] is not None


class FakeLogger:
    def __init__(self, directory: Path, *, fail_close: bool = False):
        self.directory = directory
        self.events: list[str] = []
        self.fail_close = fail_close

    def emit(self, event, **fields):
        self.events.append(event)

    def check_health(self):
        return None

    def close(self):
        if self.fail_close:
            raise OSError("close failed")


class FakeRenderer:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def fake_arguments(tmp_path: Path) -> check_kinematics.KinematicsCheckArguments:
    return check_kinematics.KinematicsCheckArguments(
        scene_config=CONFIG_PATH,
        check_config=CHECK_PATH,
        kinematics_config=KINEMATICS_PATH,
        report_root=tmp_path / "reports",
        run_id="close-test",
    )


def install_success_fakes(monkeypatch, tmp_path: Path, *, logger, renderer):
    arguments = fake_arguments(tmp_path)
    monkeypatch.setattr(check_kinematics, "_open_logger", lambda *_args: logger)
    monkeypatch.setattr(
        check_kinematics.probe_environment,
        "check_dependencies",
        lambda: {"python": {"version": "3.12"}, "packages": {}, "imports": {}},
    )
    monkeypatch.setattr(
        check_kinematics.probe_environment, "_verify_mounted_source", lambda *_args: None
    )
    monkeypatch.setattr(check_kinematics, "code_provenance", lambda: {})
    monkeypatch.setattr(check_kinematics, "_build_scene", lambda *_args: object())
    monkeypatch.setattr(check_kinematics, "_scene_info", lambda *_args: {})
    monkeypatch.setattr(check_kinematics, "_make_renderer", lambda *_args: renderer)
    monkeypatch.setattr(check_kinematics, "_run_layouts", lambda *_args: ([], None))
    monkeypatch.setattr(check_kinematics, "_write_overview", lambda *_args: None)
    return arguments


def test_logger_close_failure_turns_success_into_nonzero(tmp_path, monkeypatch):
    arguments = fake_arguments(tmp_path)
    logger = FakeLogger(
        arguments.report_root / arguments.run_id / "logs",
        fail_close=True,
    )
    renderer = FakeRenderer()
    arguments = install_success_fakes(monkeypatch, tmp_path, logger=logger, renderer=renderer)
    code = check_kinematics.run(arguments)
    report = json.loads((arguments.report_root / arguments.run_id / "report.json").read_text())
    assert code == 1
    assert report["status"] == "failed"
    assert report["reason"] == "logger_close_failed"
    assert report["logs_closed"] is False
    assert renderer.closed is True


def test_report_write_failure_is_nonzero_and_loud(tmp_path, monkeypatch, capsys):
    arguments = fake_arguments(tmp_path)
    logger = FakeLogger(arguments.report_root / arguments.run_id / "logs")
    install_success_fakes(monkeypatch, tmp_path, logger=logger, renderer=FakeRenderer())
    monkeypatch.setattr(
        check_kinematics,
        "write_json_atomic",
        lambda *_args: (_ for _ in ()).throw(OSError("disk full")),
    )
    code = check_kinematics.run(arguments)
    captured = capsys.readouterr()
    assert code == 1
    assert "report_write_failed" in captured.err
    assert "disk full" in captured.err
