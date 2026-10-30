ARG ROS_BASE_IMAGE=sim-dual-arm/ros-base:stage2
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
