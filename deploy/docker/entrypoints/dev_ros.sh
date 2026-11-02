#!/usr/bin/env bash
# Shared entrypoint for the host-mounted mock development image.
# ROS setup scripts are not nounset-safe, so keep errexit/pipefail while
# sourcing ROS and the generated install, then enable nounset for the app.
set -eo pipefail

source /opt/ros/jazzy/setup.bash

if [[ "${DEV_ROS_BUILD:-0}" == "1" ]]; then
  set -u
  exec "$@"
fi

build_root="${DEV_ROS_ROOT:-/opt/ros-dev}"
workspace="${DEV_REPO_ROOT:-/workspace}"
# Diagnostics go to stderr so an application's stdout (for example audit JSON)
# remains directly parseable.
python3 "$workspace/tools/build/dev_workspace.py" check \
  --project-root "$workspace" --build-root "$build_root" 1>&2

# Source the freshly generated interface install only after the marker and
# generated modules have been verified.  There is no fallback to an older SDK.
source "$build_root/install/setup.bash"

# Source setup scripts may append older install paths.  Put the explicit source
# packages first so mounted code wins for both services.
set -u
source_paths=(
  "$workspace/contracts/python/src"
  "$workspace/libraries/backend_core/src"
  "$workspace/libraries/ros_support/src"
  "$workspace/libraries/run_logging/src"
  "$workspace/components/runtime/ros/dual_arm_runtime"
  "$workspace/tests/doubles/ros/mock_robot_backend"
)
python_path="${source_paths[0]}"
for path in "${source_paths[@]:1}"; do
  python_path+=":$path"
done
export PYTHONPATH="$python_path${PYTHONPATH:+:$PYTHONPATH}"

python3 "$workspace/tools/build/dev_workspace.py" status \
  --project-root "$workspace" --build-root "$build_root" 1>&2

exec "$@"
