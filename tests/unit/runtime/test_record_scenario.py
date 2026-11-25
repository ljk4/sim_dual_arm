"""Collection scenario selection: binding provenance, conflicts and preflight."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from collection_fixtures import BACKEND, ROBOT, SCENARIO_ID, TASK, build_collection_tree, sha
from dual_arm_runtime.offline.collection import campaign, record, record_preflight
from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collection.scenario_selection import (
    campaign_config_path,
    select_collection,
)
from dual_arm_runtime.offline.export import export_episode
from scenario_fixtures import ROOT


def _select(tree, *, backend: str = BACKEND, require_parallel: bool = True, **kwargs):
    return select_collection(
        robot=ROBOT,
        backend=backend,
        task=TASK,
        shared_root=tree.shared,
        project_root=tree.repo,
        require_parallel=require_parallel,
        **kwargs,
    )


def _rewrite(path, mutate):
    document = yaml.safe_load(path.read_text())
    mutate(document)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def test_selection_returns_binding_recipes_profiles_and_capabilities(tmp_path):
    tree = build_collection_tree(tmp_path)
    selection = _select(tree)
    assert selection["mode"] == "scenario"
    assert selection["scenario_id"] == SCENARIO_ID
    assert selection["binding"] == {
        "path": str(tree.scenario.binding_path.resolve()),
        "sha256": sha(tree.scenario.binding_path),
    }
    assert selection["recipes"]["campaign"]["path"] == str(tree.campaign_recipe.resolve())
    assert selection["recipes"]["runtime_teacher"]["path"] == str(tree.runtime_recipe.resolve())
    assert selection["recipes"]["simulation_teacher"]["path"] == str(tree.backend_recipe.resolve())
    assert selection["profiles"]["robot"]["path"] == str(
        (tree.scenario.root / "profiles/robot.json").resolve()
    )
    assert selection["capabilities"] == {
        "backend": "simulation/mujoco",
        "runtime": "ros2",
        "teacher": "marvin_teacher_api_msgs.teacher_chunk",
        "teacher_package": "marvin_teacher_api_msgs",
        "teacher_package_path": str(
            (tree.scenario.root / "extensions/ros/marvin_teacher_api_msgs").resolve()
        ),
    }
    assert selection["interfaces"]["teacher_api"] == ("marvin_teacher_api_msgs.teacher_chunk")


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("backend_kind", "hardware", "unsupported backend.kind"),
        ("backend_implementation", "isaac", "unsupported backend.implementation"),
        ("runtime_implementation", "ros1", "unsupported runtime.implementation"),
        ("common_ros", ["robot_api_msgs"], "unsupported Teacher capability"),
    ],
)
def test_unsupported_declared_capabilities_are_rejected(tmp_path, field, value, message):
    tree = build_collection_tree(tmp_path)
    document = tree.scenario.document
    selected_backend = BACKEND
    if field == "backend_kind":
        document["backend"]["kind"] = value
        document["backend"]["implementation"] = "mujoco"
    elif field == "backend_implementation":
        document["backend"]["implementation"] = value
        selected_backend = value
        new_root = tree.shared / f"scenarios/{ROBOT}/{value}/{TASK}"
        new_root.parent.mkdir(parents=True, exist_ok=True)
        tree.scenario.root.rename(new_root)
        tree.scenario.root = new_root
        tree.scenario.binding_path = new_root / "binding.yaml"
    elif field == "runtime_implementation":
        document["runtime"]["implementation"] = value
    else:
        document["interfaces"]["common_ros"] = value
    tree.scenario.write()
    with pytest.raises(CampaignError, match=message):
        _select(tree, backend=selected_backend)


def test_teacher_recipes_must_declare_teacher_mode(tmp_path):
    tree = build_collection_tree(tmp_path)
    _rewrite(tree.runtime_recipe, lambda document: document.update(action_source="policy"))
    with pytest.raises(CampaignError, match="action_source: teacher"):
        _select(tree)

    tree = build_collection_tree(tmp_path / "second")
    _rewrite(
        tree.backend_recipe,
        lambda document: document["robot"].update(task_mode="inference"),
    )
    with pytest.raises(CampaignError, match="task_mode: teacher"):
        _select(tree)


def test_parallel_selection_requires_render_quality(tmp_path):
    tree = build_collection_tree(tmp_path)
    _rewrite(
        tree.backend_recipe,
        lambda document: document["robot"]["render_quality"].update(enabled=False),
    )
    with pytest.raises(CampaignError, match="render_quality"):
        _select(tree, require_parallel=True)


def test_missing_teacher_service_definition_is_rejected(tmp_path):
    tree = build_collection_tree(tmp_path)
    (tree.scenario.root / "extensions/ros/marvin_teacher_api_msgs/srv/GetTeacherChunk.srv").unlink()
    with pytest.raises(CampaignError, match="no selected extension declares"):
        _select(tree)


def test_teacher_capability_must_match_the_selected_extension(tmp_path):
    tree = build_collection_tree(tmp_path)
    tree.scenario.document["interfaces"]["teacher_api"] = "other_api_msgs.teacher_chunk"
    tree.scenario.write()
    with pytest.raises(CampaignError, match="no selected extension declares"):
        _select(tree)

    tree = build_collection_tree(tmp_path / "second")
    _rewrite(
        tree.runtime_recipe,
        lambda document: document.update(teacher_api="other_api_msgs.teacher_chunk"),
    )
    with pytest.raises(CampaignError, match="teacher_api .* differs from binding"):
        _select(tree)


def test_recipe_profile_must_match_the_binding(tmp_path):
    tree = build_collection_tree(tmp_path)
    _rewrite(
        tree.runtime_recipe,
        lambda document: document.update(robot_profile="../profiles/observation.json"),
    )
    with pytest.raises(CampaignError, match="differs from scenario profile"):
        _select(tree)


def test_binding_digest_is_checked_against_preflight(tmp_path):
    tree = build_collection_tree(tmp_path)
    with pytest.raises(CampaignError, match="binding changed after preflight"):
        _select(tree, binding_sha256="0" * 64)


def _preflight_args(tree, *extra):
    return record_preflight.parser().parse_args(
        [
            "--project-root",
            str(tree.repo),
            "--robot",
            ROBOT,
            "--backend",
            BACKEND,
            "--task",
            TASK,
            "--shared-root",
            str(tree.shared),
            "--parallel",
            "--skip-sdk-check",
            *extra,
        ]
    )


def test_preflight_plan_is_read_only(tmp_path):
    tree = build_collection_tree(tmp_path)
    tree.campaign["launch"]["stop_grace_s"] = 45.0
    tree.campaign_recipe.write_text(yaml.safe_dump(tree.campaign))
    plan = record_preflight.build_plan(_preflight_args(tree))
    assert plan["selection"]["binding"]["sha256"] == sha(tree.scenario.binding_path)
    assert plan["campaign_config"] == str(tree.campaign_recipe.resolve())
    assert plan["runtime_config"] == str(tree.runtime_recipe.resolve())
    assert plan["simulation_template"] == str(tree.backend_recipe.resolve())
    assert "formal_target_qualified" not in plan["campaign_status"]
    assert plan["stop_grace_s"] == 45.0
    assert not (tree.repo / "output/synthetic_sim/mujoco/stack").exists()
    assert not (tree.repo / "output/synthetic_sim/mujoco/stack/logs/collection/record-pools").exists()


def test_preflight_rejects_unmounted_or_conflicting_selection(tmp_path):
    tree = build_collection_tree(tmp_path)
    args = _preflight_args(tree)
    args.config = tree.campaign_recipe
    with pytest.raises(CampaignError, match="cannot be combined"):
        record_preflight.build_plan(args)

    tree = build_collection_tree(tmp_path / "second")
    document = yaml.safe_load(tree.campaign_recipe.read_text())
    document["paths"]["required_paths"].append("shared/missing.yaml")
    tree.campaign_recipe.write_text(yaml.safe_dump(document, sort_keys=False))
    with pytest.raises(CampaignError, match="required path is missing"):
        record_preflight.build_plan(_preflight_args(tree))
    assert not (tree.repo / "output/synthetic_sim/mujoco/stack").exists()


def test_preflight_allows_arbitrary_positive_target_total(tmp_path):
    tree = build_collection_tree(tmp_path)
    plan = record_preflight.build_plan(_preflight_args(tree, "--target-total", "60"))
    assert plan["selection"]["binding"]["sha256"] == sha(tree.scenario.binding_path)
    with pytest.raises(CampaignError, match="positive"):
        record_preflight.build_plan(_preflight_args(tree, "--target-total", "0"))


def test_record_selection_uses_binding_recipes_for_runner_and_exporter(tmp_path):
    tree = build_collection_tree(tmp_path)
    parser = record.parser()
    args = parser.parse_args(
        [
            "--project-root",
            str(tree.repo),
            "--robot",
            ROBOT,
            "--backend",
            BACKEND,
            "--task",
            TASK,
            "--shared-root",
            str(tree.shared),
        ]
    )
    args.project_root = args.project_root.resolve()
    record._resolve_selection(args, parser)
    assert args.config == str(tree.campaign_recipe.resolve())
    assert args.runtime_config == str(tree.runtime_recipe.resolve())
    assert args.selection["binding"]["sha256"] == sha(tree.scenario.binding_path)
    command = record._collector_command(args)
    index = command.index("--campaign-config")
    assert command[index + 1] == str(tree.campaign_recipe.resolve())


def test_record_direct_mode_requires_explicit_recipes(tmp_path):
    tree = build_collection_tree(tmp_path)
    parser = record.parser()
    with pytest.raises(SystemExit) as error:
        args = parser.parse_args(["--project-root", str(tree.repo)])
        args.project_root = args.project_root.resolve()
        record._resolve_selection(args, parser)
    assert error.value.code == 2

    args = parser.parse_args(
        [
            "--project-root",
            str(tree.repo),
            "--config",
            str(tree.campaign_recipe),
            "--runtime-config",
            str(tree.runtime_recipe),
        ]
    )
    args.project_root = args.project_root.resolve()
    record._resolve_selection(args, parser)
    assert args.config == str(tree.campaign_recipe)
    assert args.runtime_config == str(tree.runtime_recipe)
    assert args.selection is None


def test_record_scenario_conflicts_fail_before_any_ledger(tmp_path):
    tree = build_collection_tree(tmp_path)
    base = [
        "--project-root",
        str(tree.repo),
        "--robot",
        ROBOT,
        "--backend",
        BACKEND,
        "--task",
        TASK,
        "--shared-root",
        str(tree.shared),
    ]
    for extra in (
        ["--config", str(tree.campaign_recipe)],
        ["--runtime-config", str(tree.runtime_recipe)],
    ):
        with pytest.raises(SystemExit) as error:
            record.main([*base, *extra])
        assert error.value.code == 2
    with pytest.raises(SystemExit) as error:
        record.main(["--project-root", str(tree.repo), "--shared-root", str(tree.shared)])
    assert error.value.code == 2
    assert not (tree.repo / "output/synthetic_sim/mujoco/stack").exists()


def test_campaign_scenario_uses_binding_and_rejects_conflicts(tmp_path, monkeypatch):
    tree = build_collection_tree(tmp_path)
    base = [
        "--project-root",
        str(tree.repo),
        "--robot",
        ROBOT,
        "--backend",
        BACKEND,
        "--task",
        TASK,
        "--shared-root",
        str(tree.shared),
    ]
    captured: dict = {}
    monkeypatch.setattr(campaign, "command_run", lambda args: captured.update(vars(args)) or 0)
    assert campaign.main([*base, "run", "--purpose", "pilot"]) == 0
    assert captured["config"] == str(tree.campaign_recipe.resolve())
    assert captured["runtime_config"] == str(tree.runtime_recipe.resolve())

    with pytest.raises(SystemExit) as error:
        campaign.main([*base, "--config", "explicit.yaml", "status"])
    assert error.value.code == 2
    with pytest.raises(SystemExit) as error:
        campaign.main([*base, "run", "--purpose", "pilot", "--runtime-config", "explicit.yaml"])
    assert error.value.code == 2
    assert not (tree.repo / "output/synthetic_sim/mujoco/stack/logs/collection/attempts.jsonl").exists()


def test_campaign_rejects_unsupported_binding_before_reserving(tmp_path, capsys):
    tree = build_collection_tree(tmp_path)
    tree.scenario.document["interfaces"]["common_ros"] = ["robot_api_msgs"]
    tree.scenario.write()
    code = campaign.main(
        [
            "--project-root",
            str(tree.repo),
            "--robot",
            ROBOT,
            "--backend",
            BACKEND,
            "--task",
            TASK,
            "--shared-root",
            str(tree.shared),
            "run",
            "--purpose",
            "pilot",
        ]
    )
    assert code == 2
    assert "unsupported Teacher capability" in capsys.readouterr().err
    assert not (tree.repo / "output/synthetic_sim/mujoco/stack").exists()


def test_export_campaign_config_selection_and_conflict(tmp_path):
    tree = build_collection_tree(tmp_path)
    path = campaign_config_path(
        campaign_config=None,
        robot=ROBOT,
        backend=BACKEND,
        task=TASK,
        shared_root=tree.shared,
        project_root=tree.repo,
        default=tree.campaign_recipe,
    )
    assert path == tree.campaign_recipe.resolve()
    with pytest.raises(CampaignError, match="cannot be combined"):
        campaign_config_path(
            campaign_config=tree.campaign_recipe,
            robot=ROBOT,
            backend=BACKEND,
            task=TASK,
            shared_root=tree.shared,
            project_root=tree.repo,
            default=tree.campaign_recipe,
        )
    episode = tmp_path / "episode"
    episode.mkdir()
    code = export_episode.main(
        [
            "--episode-dir",
            str(episode),
            "--out",
            str(tmp_path / "export"),
            "--robot",
            ROBOT,
            "--backend",
            BACKEND,
            "--task",
            TASK,
            "--shared-root",
            str(tree.shared),
            "--campaign-config",
            str(tree.campaign_recipe),
        ]
    )
    assert code == 2


def test_real_campaign_required_paths_use_the_actual_teacher_package():
    path = (
        ROOT / "shared/scenarios/marvin/mujoco/stack/configs/runtime/stack_campaign.yaml"
    )
    document = yaml.safe_load(path.read_text())
    required = document["paths"]["required_paths"]
    teacher = [item for item in required if item.endswith("GetTeacherChunk.srv")]
    assert teacher == [
        "shared/robots/marvin/extensions/ros/marvin_teacher_api_msgs/srv/GetTeacherChunk.srv"
    ]
    for item in required:
        assert (ROOT / item).exists(), item
    # The SDK marker is generated by build-ros-sdk, not shipped in a checkout.
    assert document["paths"]["sdk_marker"] == "output/build/ros-dev/contracts.json"


def test_direct_relative_recipes_reach_collector_as_mounted_paths(tmp_path, monkeypatch):
    tree = build_collection_tree(tmp_path)
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", str(tree.repo))
    monkeypatch.setenv("SHARED_ROOT", str(tree.shared))
    monkeypatch.chdir(tmp_path)
    parser = record.parser()
    args = parser.parse_args(
        [
            "--project-root",
            str(tree.repo),
            "--config",
            str(tree.campaign_recipe.relative_to(tree.repo)),
            "--runtime-config",
            str(tree.runtime_recipe.relative_to(tree.repo)),
        ]
    )
    record._resolve_selection(args, parser)
    command = record._collector_command(args)
    for flag, expected in [
        ("--campaign-config", tree.campaign_recipe),
        ("--runtime-config", tree.runtime_recipe),
    ]:
        actual = command[command.index(flag) + 1]
        assert actual == str(expected.resolve())
        assert Path(actual).read_text() == expected.read_text()
