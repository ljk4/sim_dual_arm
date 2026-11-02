ARG ROS_BASE_IMAGE=sim-dual-arm/ros-base:stage2
ARG SIM_BASE_IMAGE=sim-dual-arm/dev-ros:stage2

# Stable simulation dependency layer.  It contains only the pinned wheel set
# and the EGL/OpenGL loaders; project source, assets, configs and data stay on
# read-only host mounts at run time.
FROM ${SIM_BASE_IMAGE} AS mujoco-dev

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 \
    libgl1 \
    libopengl0 \
    libglvnd0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/sim
COPY deploy/docker/sim-requirements.lock /opt/sim/sim-requirements.lock
COPY deploy/docker/sim-packages.json /opt/sim/sim-packages.json
RUN pip install --no-cache-dir --require-hashes -r /opt/sim/sim-requirements.lock

# Keep provenance in the image without embedding a fabricated image ID.
RUN python - <<'PY'
import importlib.metadata
import json
import platform
import subprocess
import sys

packages = json.load(open("/opt/sim/sim-packages.json", encoding="utf-8"))["packages"]
installed = {}
for entry in packages:
    try:
        installed[entry["name"]] = importlib.metadata.version(entry["name"])
    except importlib.metadata.PackageNotFoundError:
        installed[entry["name"]] = None
graphics = subprocess.run(
    [
        "dpkg-query",
        "-W",
        "-f=${Package}=${Version}\n",
        "libegl1",
        "libgl1",
        "libopengl0",
        "libglvnd0",
    ],
    check=True,
    capture_output=True,
    text=True,
).stdout.splitlines()
document = {
    "schema_version": 1,
    "python": {
        "implementation": platform.python_implementation(),
        "version": platform.python_version(),
        "executable": sys.executable,
    },
    "packages": installed,
    "graphics": dict(line.split("=", 1) for line in graphics),
}
with open("/opt/sim/versions.json", "w", encoding="utf-8") as stream:
    json.dump(document, stream, indent=2, sort_keys=True)
    stream.write("\n")
PY

# Independent reproducible builder for the locked Ruckig Community source.
# It installs only the pinned build toolchain, builds the locked sdist with
# isolation disabled, and records the actual wheel SHA plus every tool version.
FROM ${SIM_BASE_IMAGE} AS ruckig-builder

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    CMAKE_BUILD_PARALLEL_LEVEL=2

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ruckig-build
COPY deploy/docker/ruckig-build-requirements.lock /opt/ruckig-build/ruckig-build-requirements.lock
COPY deploy/docker/ruckig-requirements.lock /opt/ruckig-build/ruckig-requirements.lock
COPY deploy/docker/ruckig-source.json /opt/ruckig-build/ruckig-source.json
RUN pip install --no-cache-dir --require-hashes -r /opt/ruckig-build/ruckig-build-requirements.lock
# Never use build isolation: the locked cmake/nanobind/scikit-build-core set is
# the only toolchain this wheel may see.  --no-deps keeps the source wheel pure.
RUN pip wheel --no-cache-dir --no-build-isolation --no-deps --require-hashes \
    -r /opt/ruckig-build/ruckig-requirements.lock \
    --wheel-dir /opt/ruckig-wheel
RUN mkdir -p /opt/ruckig-provenance && python - <<'PY'
import glob
import hashlib
import importlib.metadata
import json
import os
import platform

source = json.load(open("/opt/ruckig-build/ruckig-source.json", encoding="utf-8"))
wheels = glob.glob("/opt/ruckig-wheel/*.whl")
assert len(wheels) == 1, wheels
wheel = wheels[0]
sha = hashlib.sha256(open(wheel, "rb").read()).hexdigest()
locks = {}
for name in ("ruckig-requirements.lock", "ruckig-build-requirements.lock"):
    path = os.path.join("/opt/ruckig-build", name)
    locks[name] = hashlib.sha256(open(path, "rb").read()).hexdigest()
tools = {}
for name in ("scikit-build-core", "nanobind", "cmake", "ninja", "packaging", "pathspec"):
    tools[name] = importlib.metadata.version(name)
document = {
    "schema_version": 1,
    "package": source["package"],
    "version": source["version"],
    "license": source["license"],
    "source_url": source["source_url"],
    "sdist_sha256": source["sdist_sha256"],
    "wheel_file": os.path.basename(wheel),
    "wheel_sha256": sha,
    "wheel_bytes": os.path.getsize(wheel),
    "python": {
        "implementation": platform.python_implementation(),
        "version": platform.python_version(),
        "executable": os.path.realpath(os.sys.executable),
    },
    "build_tools": tools,
    "requirements_sha256": locks,
}
with open("/opt/ruckig-provenance/build.json", "w", encoding="utf-8") as stream:
    json.dump(document, stream, indent=2, sort_keys=True)
    stream.write("\n")
PY

# Follower runtime layer: start from the accepted MuJoCo layer, copy only the
# generated wheel and its provenance, and install it without an index or build
# tools.  The default mock target below remains the Dockerfile default target.
FROM mujoco-dev AS follower-dev

COPY --from=ruckig-builder /opt/ruckig-wheel /opt/ruckig-wheel
COPY --from=ruckig-builder /opt/ruckig-provenance /opt/ruckig-provenance
RUN pip install --no-cache-dir --no-index --no-deps /opt/ruckig-wheel/ruckig-0.15.3-*.whl
RUN python - <<'PY'
import importlib.metadata
import json
import ruckig

provenance = json.load(open("/opt/ruckig-provenance/build.json", encoding="utf-8"))
version = importlib.metadata.version("ruckig")
assert version == provenance["version"], (version, provenance["version"])
print(f"follower-dev ruckig {version} {provenance['wheel_sha256']}")
PY

# Original CPU mock image target.  Keep it as the default target so existing
# stage2 builds and the mock backend behaviour are unchanged.
FROM ${ROS_BASE_IMAGE} AS mock
COPY contracts /workspace/contracts
COPY libraries/backend_core /workspace/libraries/backend_core
COPY libraries/ros_support /workspace/libraries/ros_support
COPY tools /workspace/tools
COPY profiles /workspace/profiles
COPY tests/doubles/ros /workspace/tests/doubles/ros
RUN pip install --no-build-isolation --no-deps \
    ./contracts/python ./libraries/backend_core ./libraries/ros_support
RUN source /opt/ros/jazzy/setup.bash && python tools/build/ros_workspace.py --log-base /tmp/colcon-log build \
    --base-paths contracts/ros tests/doubles/ros \
    --build-base /tmp/colcon-build --install-base /opt/project/install \
    --event-handlers console_direct+
CMD ["ros2", "run", "mock_robot_backend", "mock_backend", "--profile", "/workspace/profiles/robots/mock_dual_arm_v1/profile.json", "--schemas", "/workspace/contracts/schemas/v1"]
