# Runtime and test targets share dependencies; all project inputs are mounts.
ARG DEV_ROS_IMAGE=sim-dual-arm/dev-ros:stage2
FROM ${DEV_ROS_IMAGE} AS runtime
WORKDIR /workspace
CMD ["python", "-m", "dual_arm_runtime.node", "--once"]

FROM runtime AS test
CMD ["python", "-m", "pytest", "-p", "no:cacheprovider", "-q", "tests/unit", "tests/integration"]
