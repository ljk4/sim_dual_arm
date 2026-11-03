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
RUN --mount=type=bind,source=tools/build,target=/build-tools,readonly \
    python /build-tools/image_metadata.py sim \
    --packages /opt/sim/sim-packages.json --output /opt/sim/versions.json

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
RUN --mount=type=bind,source=tools/build,target=/build-tools,readonly \
    python /build-tools/image_metadata.py ruckig-build \
    --source /opt/ruckig-build/ruckig-source.json --wheel-dir /opt/ruckig-wheel \
    --lock-dir /opt/ruckig-build --output /opt/ruckig-provenance/build.json

# Follower runtime layer: start from the accepted MuJoCo layer, copy only the
# generated wheel and its provenance, and install it without an index or build
# tools.  The default mock target below remains the Dockerfile default target.
FROM mujoco-dev AS follower-dev

COPY --from=ruckig-builder /opt/ruckig-wheel /opt/ruckig-wheel
COPY --from=ruckig-builder /opt/ruckig-provenance /opt/ruckig-provenance
RUN pip install --no-cache-dir --no-index --no-deps /opt/ruckig-wheel/ruckig-0.15.3-*.whl
RUN --mount=type=bind,source=tools/build,target=/build-tools,readonly \
    python /build-tools/image_metadata.py ruckig-check \
    --provenance /opt/ruckig-provenance/build.json

# Source-mounted CPU mock target. The inherited dev entrypoint verifies the
# mounted ROS SDK and imports runtime/backend modules from /workspace.
FROM ${SIM_BASE_IMAGE} AS mock
WORKDIR /workspace
CMD ["python", "-m", "mock_robot_backend.node", "--profile", "/workspace/profiles/robots/mock_dual_arm_v1/profile.json", "--schemas", "/workspace/contracts/schemas/v1"]
