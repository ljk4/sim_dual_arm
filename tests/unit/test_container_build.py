"""Deployment failure boundaries: SDK ownership/origin and image provenance."""

import hashlib
import json
import sys
from types import ModuleType

import pytest

from tools.build import check_dockerfiles, image_metadata, policy_sdk


def sdk_fixture(tmp_path):
    project = tmp_path / "project"
    proto = project / "contracts/policy/v1/policy.proto"
    proto.parent.mkdir(parents=True)
    proto.write_text('syntax = "proto3";\n')
    sdk = tmp_path / "sdk"
    for relative in policy_sdk.REQUIRED_FILES:
        path = sdk / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("")
    (sdk / "source.sha256").write_text(hashlib.sha256(proto.read_bytes()).hexdigest())
    return project, sdk


def test_stale_sdk_rejected_before_import(tmp_path, monkeypatch):
    project, sdk = sdk_fixture(tmp_path)
    (sdk / "source.sha256").write_text("0" * 64)
    monkeypatch.setattr(
        policy_sdk.importlib, "import_module", lambda _: pytest.fail("imported stale SDK")
    )
    with pytest.raises(policy_sdk.SDKError, match="POLICY_SDK_STALE"):
        policy_sdk.verify_sdk(project, sdk)


def test_incomplete_sdk_rejected(tmp_path):
    project, sdk = sdk_fixture(tmp_path)
    (sdk / "dual_arm_policy/v1/policy_pb2.py").unlink()
    with pytest.raises(policy_sdk.SDKError, match="POLICY_SDK_INCOMPLETE"):
        policy_sdk.verify_sdk(project, sdk)


def test_installed_sdk_cannot_replace_mounted_sdk(tmp_path, monkeypatch):
    project, sdk = sdk_fixture(tmp_path)
    module = ModuleType(policy_sdk.MODULES[0])
    module.__file__ = str(tmp_path / "installed/policy_pb2.py")
    monkeypatch.setitem(sys.modules, policy_sdk.MODULES[0], module)
    monkeypatch.syspath_prepend(str(sdk))
    with pytest.raises(policy_sdk.SDKError, match="POLICY_SDK_FALLBACK"):
        policy_sdk.verify_sdk(project, sdk)


def test_unknown_output_is_preserved(tmp_path):
    output = tmp_path / "unrelated"
    output.mkdir()
    important = output / "user-data.txt"
    important.write_text("keep")
    with pytest.raises(policy_sdk.SDKError, match="UNOWNED_OUTPUT"):
        policy_sdk.prepare_output(tmp_path / "project", output)
    assert important.read_text() == "keep"
    assert sorted(path.name for path in output.iterdir()) == ["user-data.txt"]


def test_symlink_output_not_adopted(tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    target = tmp_path / "user-file"
    target.write_text("keep")
    (output / policy_sdk.OWNER_FILE).symlink_to(target)
    with pytest.raises(policy_sdk.SDKError, match="symlink"):
        policy_sdk.prepare_output(tmp_path / "project", output)
    assert target.read_text() == "keep"


def test_ruckig_provenance_binds_wheel_and_locks(tmp_path, monkeypatch):
    source = tmp_path / "source.json"
    spec = dict(
        package="ruckig",
        version="0.15.3",
        license="MIT",
        source_url="https://example.invalid/ruckig",
        sdist_sha256="a" * 64,
    )
    source.write_text(json.dumps(spec))
    wheel = tmp_path / "ruckig.whl"
    wheel.write_bytes(b"wheel content")
    for name in ("ruckig-requirements.lock", "ruckig-build-requirements.lock"):
        (tmp_path / name).write_text(name)
    monkeypatch.setattr(image_metadata.importlib.metadata, "version", lambda _: "1.0")
    document = image_metadata.record_ruckig(source, tmp_path, tmp_path, tmp_path / "report.json")
    assert document["wheel_sha256"] == hashlib.sha256(wheel.read_bytes()).hexdigest()
    assert document["wheel_bytes"] == len(b"wheel content")
    assert (
        document["requirements_sha256"]["ruckig-requirements.lock"]
        == hashlib.sha256(b"ruckig-requirements.lock").hexdigest()
    )


@pytest.mark.parametrize(
    "instruction",
    [
        "RUN python - <<'PY'\nprint('hidden')\nPY\n",
        'RUN python -c "print(1)"\n',
        "COPY components /workspace/components\n",
        'COPY ["configs", "/workspace/configs"]\n',
        'COPY . /workspace\n',
        'RUN if test -f /tmp/input; then do_work; fi\n',
    ],
)
def test_dockerfile_rejects_embedded_programs_and_application_copy(instruction):
    assert check_dockerfiles.check_text(instruction)


def test_dockerfile_allows_dependency_files_and_compiled_artifacts():
    instructions = """COPY deploy/docker/sim-requirements.lock /opt/sim/requirements.lock
COPY --from=builder /opt/wheels /opt/wheels
RUN --mount=type=bind,source=tools/build,target=/build-tools,readonly \\
    python3 /build-tools/image_metadata.py ruckig-check --provenance /opt/build.json
"""
    assert check_dockerfiles.check_text(instructions) == []
