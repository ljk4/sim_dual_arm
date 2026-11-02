# Global build scope: this argument is consumed by later FROM instructions.
# The default tag is portable; a machine with the probe image can override
# ACT_SYSTEM_IMAGE=sim-dual-arm/learning:system-probe-20261101-v3.
ARG ACT_SYSTEM_IMAGE=ubuntu:24.04

FROM python:3.12-slim-bookworm AS protocol
WORKDIR /workspace
COPY deploy/docker/requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-cache-dir -r /tmp/requirements-dev.txt
COPY contracts/policy /workspace/contracts/policy
COPY tools/contracts/generate_policy_sdk.py /workspace/tools/contracts/generate_policy_sdk.py
RUN python tools/contracts/generate_policy_sdk.py --output /tmp/policy-sdk \
    && pip install --no-build-isolation --no-deps /tmp/policy-sdk \
    && python -c "from dual_arm_policy.v1 import policy_pb2, policy_pb2_grpc"
# The one-shot protocol target remains the SDK/image build check.
CMD ["python", "-c", "from dual_arm_policy.v1 import policy_pb2; print(policy_pb2.DESCRIPTOR.package)"]

# ROS-free/GPU-free fake Policy service image.  Source and config are mounted
# read-only by the Compose overlay; this stage contains only Python, gRPC, the
# generated SDK and the production learning package discovery path.
FROM protocol AS policy
# Only the one small runtime dependency missing from the protocol image, with
# the exact PyYAML 6.0.3 hashes already pinned by the main learning lock.
COPY deploy/docker/learning-policy-requirements.lock /tmp/policy-requirements.lock
RUN pip install --no-cache-dir --require-hashes -r /tmp/policy-requirements.lock
ENV PYTHONPATH=/opt/policy-sdk:/workspace/components/learning/src:/workspace/tests/doubles/policy:/workspace/libraries/run_logging/src
COPY components/learning /workspace/components/learning
COPY libraries/run_logging /workspace/libraries/run_logging
WORKDIR /workspace
# Compose overrides this with the mounted tests/doubles/policy entry.  Keeping
# the standalone image command as an import check ensures a built SDK is real.
CMD ["python", "-c", "from dual_arm_policy.v1 import policy_pb2; print(policy_pb2.DESCRIPTOR.name, policy_pb2.DESCRIPTOR.package)"]

# ── ACT task-training system layer ─────────────────────────────────────────
# Keep the protocol/policy targets above frozen.  The ACT target uses the
# verified Ubuntu 24.04 system layer rather than a copied host virtualenv.
# Probe base ID: sha256:5546a1acc44e5e6d0581903fe37bec311996429498bca8826008f2305c43ffac.
FROM ${ACT_SYSTEM_IMAGE} AS act-system
ARG DEBIAN_FRONTEND=noninteractive
# The probe/system image may carry a Python ENTRYPOINT; ACT commands are
# explicit, so clear it in every ACT target.
ENTRYPOINT []
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3.12 python3.12-venv python3-pip ffmpeg libpython3.12 libgomp1 \
        libgl1 libglib2.0-0 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
# Cross-machine dependency bootstrap.  The 94-package / 2390-line lock is
# installed with hashes; source, data and weight caches are runtime mounts.
FROM act-system AS act-deps
COPY deploy/docker/learning-requirements.lock /tmp/learning-requirements.lock
COPY deploy/docker/learning-source.json /opt/learning/source.json
RUN python3.12 -m venv /opt/learning-venv \
    && /opt/learning-venv/bin/pip install --no-cache-dir \
        --require-hashes -r /tmp/learning-requirements.lock
ENV PATH=/opt/learning-venv/bin:${PATH}
ENV PYTHONDONTWRITEBYTECODE=1
ENV HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 TRANSFORMERS_OFFLINE=1

# Development/runtime entry.  Mount, do not COPY:
#   candidate source  -> /workspace                 (ro)
#   pinned LeRobot    -> /opt/lerobot/src           (ro)
#   official weights  -> /opt/learning-cache        (ro)
#   accepted datasets -> /data                      (ro)
#   training outputs  -> /output                    (rw)
FROM act-deps AS act
WORKDIR /workspace
ENV PYTHONPATH=/workspace/components/learning/src:/workspace/libraries/run_logging/src:/opt/lerobot/src
ENV LEARNING_SOURCE=/opt/lerobot/src
ENV LEARNING_CACHE_ROOT=/output/cache
VOLUME ["/workspace", "/opt/lerobot", "/opt/learning-cache", "/data", "/output"]
CMD ["python", "-m", "dual_arm_learning.cli", "--help"]

# Explicit development target for this machine: use the already verified host
# virtualenv as a read-only mount instead of reinstalling 94 packages.
#   -v /home/ljk/work/sim_dual_arm/.build/learning-dev-venv:/opt/host-learning-venv:ro
FROM act-system AS act-dev
ENTRYPOINT []
ENV PATH=/opt/host-learning-venv/bin:${PATH}
ENV PYTHONPATH=/workspace/components/learning/src:/workspace/libraries/run_logging/src:/opt/lerobot/src
ENV LEARNING_SOURCE=/opt/lerobot/src
ENV LEARNING_CACHE_ROOT=/output/cache
WORKDIR /workspace
VOLUME ["/workspace", "/opt/lerobot", "/opt/host-learning-venv", "/data", "/output"]
CMD ["python", "-m", "dual_arm_learning.cli", "--help"]
