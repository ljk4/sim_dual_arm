# ROS 2 runtime dependency image. Mounted source and runtime settings stay outside the image.
ARG BASE_IMAGE=ros:jazzy-ros-base-noble
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-jsonschema \
    python3-yaml \
    python3-colcon-common-extensions \
    ros-jazzy-rosbag2-py \
    ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

COPY requirements-dev.txt /opt/dependencies/requirements-dev.txt
RUN pip install --no-cache-dir -r /opt/dependencies/requirements-dev.txt

ENTRYPOINT []
CMD ["sleep", "infinity"]
