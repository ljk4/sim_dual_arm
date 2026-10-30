FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-jsonschema python3-yaml \
    python3-colcon-common-extensions ros-jazzy-rosbag2-py \
    ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"
COPY deploy/docker/requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-cache-dir -r /tmp/requirements-dev.txt
COPY deploy/docker/entrypoints/ros.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds.xml
COPY deploy/docker/fastdds.xml /opt/fastdds.xml
WORKDIR /workspace
ENTRYPOINT ["/opt/entrypoint.sh"]
