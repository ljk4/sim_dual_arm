# MuJoCo dependency image. Runtime paths and options live in configs/container.env.
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-jsonschema \
    python3-yaml \
    libegl1 \
    libgl1 \
    libopengl0 \
    libglvnd0 \
    && rm -rf /var/lib/apt/lists/*

# The venv uses ROS Python packages from the base image.
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

COPY requirements.lock \
     wheels/ruckig-0.15.3-cp312-cp312-linux_x86_64.whl \
     /opt/dependencies/
RUN pip install --no-cache-dir --require-hashes -r /opt/dependencies/requirements.lock

# The container stays idle until a user enters it and runs scripts/run.sh.
ENTRYPOINT []
CMD ["sleep", "infinity"]
