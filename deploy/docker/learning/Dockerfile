# ACT dependencies can use the verified system-probe base on this machine.
ARG ACT_SYSTEM_IMAGE=ubuntu:24.04

# Source and the generated SDK must be mounted; no installed SDK fallback.
FROM python:3.12-slim-bookworm AS protocol
WORKDIR /workspace
COPY deploy/docker/requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-cache-dir -r /tmp/requirements-dev.txt
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/opt/policy-sdk:/workspace/components/learning/src:/workspace/tests/doubles/policy:/workspace/libraries/run_logging/src
ENTRYPOINT ["bash", "/workspace/deploy/docker/entrypoints/policy.sh"]
CMD ["python", "/workspace/tools/build/policy_sdk.py", "check"]

FROM protocol AS policy
COPY deploy/docker/learning-policy-requirements.lock /tmp/policy-requirements.lock
RUN pip install --no-cache-dir --require-hashes -r /tmp/policy-requirements.lock

# ── ACT task-training system layer ─────────────────────────────────────────
# The ACT target uses the verified Ubuntu 24.04 system layer.
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
