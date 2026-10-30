ARG ROS_BASE_IMAGE=sim-dual-arm/ros-base:stage2
FROM ${ROS_BASE_IMAGE} AS runtime
COPY contracts /workspace/contracts
COPY libraries/ros_support /workspace/libraries/ros_support
COPY tools /workspace/tools
COPY profiles /workspace/profiles
COPY configs /workspace/configs
COPY components/runtime /workspace/components/runtime
RUN pip install --no-build-isolation --no-deps ./contracts/python ./libraries/ros_support \
    && python tools/contracts/generate_policy_sdk.py --output /tmp/policy-sdk \
    && pip install --no-build-isolation --no-deps /tmp/policy-sdk
RUN source /opt/ros/jazzy/setup.bash && python tools/build/ros_workspace.py --log-base /tmp/colcon-log build \
    --base-paths contracts/ros components/runtime/ros \
    --build-base /tmp/colcon-build --install-base /opt/project/install \
    --event-handlers console_direct+
RUN source /opt/ros/jazzy/setup.bash && source /opt/project/install/setup.bash \
    && python -c "from robot_api_msgs.action import ExecuteTrajectory; from dual_arm_policy.v1 import policy_pb2, policy_pb2_grpc"
CMD ["ros2", "run", "dual_arm_runtime", "runtime", "--once"]

FROM runtime AS test
COPY libraries/backend_core /workspace/libraries/backend_core
RUN pip install --no-build-isolation --no-deps ./libraries/backend_core
COPY tests/unit /workspace/tests/unit
COPY tests/integration /workspace/tests/integration
COPY pyproject.toml /workspace/pyproject.toml
CMD ["python", "-m", "pytest", "-q", "tests/unit", "tests/integration"]
