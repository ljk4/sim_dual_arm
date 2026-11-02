# Shared runtime dependency image for the host-mounted mock development stack.
# It intentionally contains no project source, profiles, configs, data or models.
ARG ROS_BASE_IMAGE=sim-dual-arm/ros-base:stage2
FROM ${ROS_BASE_IMAGE}

ENV PYTHONUNBUFFERED=1 \
    DEV_ROS_ROOT=/opt/ros-dev

WORKDIR /workspace
# Run the entrypoint from the read-only repository mount so script fixes do not
# require rebuilding this dependency image.
ENTRYPOINT ["bash", "/workspace/deploy/docker/entrypoints/dev_ros.sh"]
CMD ["bash"]
