# TianJi device environment; source and device configuration are mounted at runtime.
ARG BASE_IMAGE=ros:jazzy-ros-base-noble
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    ffmpeg \
    v4l-utils \
    usbutils \
    udev \
    libusb-1.0-0 \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

COPY requirements.txt /opt/dependencies/requirements.txt
RUN pip install --no-cache-dir -r /opt/dependencies/requirements.txt

ENTRYPOINT []
CMD ["sleep", "infinity"]
