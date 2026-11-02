cmake_minimum_required(VERSION 3.16)
project(robot_api_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RunContext.msg"
  "msg/BackendInfo.msg"
  "msg/RobotState.msg"
  "msg/CameraFrame.msg"
  "msg/TrajectoryCommand.msg"
  "msg/CommandEvent.msg"
  "msg/CommandStatus.msg"
  "msg/ObservationIndex.msg"
  "msg/RuntimeStatus.msg"
  "msg/ReferenceAnchor.msg"
  "msg/NativeCommand.msg"
  "msg/ReferenceSegment.msg"
  "msg/AcceptedReference.msg"
  "msg/FollowerReference.msg"
  "msg/NativeFinger.msg"
  "msg/NativeGripper.msg"
  "msg/NativeExecution.msg"
  "msg/NativeChunkStatus.msg"
  "msg/NativeControlEvent.msg"
  "msg/StackTaskState.msg"
  "srv/AcquireControl.srv"
  "srv/RenewControl.srv"
  "srv/ReleaseControl.srv"
  "srv/Stop.srv"
  "srv/QueryCommand.srv"
  "srv/StartEpisode.srv"
  "srv/EndEpisode.srv"
  "srv/SubmitNative.srv"
  "srv/SubmitFollower.srv"
  "srv/GetNativeAnchor.srv"
  "srv/QueryNativeCommand.srv"
  "srv/QueryNativeChunk.srv"
  "srv/ResetSimulation.srv"
  "srv/PrepareStackTask.srv"
  "srv/StartStackTask.srv"
  "srv/GetTeacherChunk.srv"
  "srv/GetStackTaskState.srv"
  "action/ExecuteTrajectory.action"
  DEPENDENCIES builtin_interfaces sensor_msgs trajectory_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
