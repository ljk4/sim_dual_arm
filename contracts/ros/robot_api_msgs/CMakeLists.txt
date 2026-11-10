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
  "msg/ObservationIndex.msg"
  "msg/RuntimeStatus.msg"
  "msg/ReferenceAnchor.msg"
  "msg/StreamCommand.msg"
  "msg/ReferenceSegment.msg"
  "msg/FollowerReference.msg"
  "msg/FingerFeedback.msg"
  "msg/GripperFeedback.msg"
  "msg/ExecutionFeedback.msg"
  "msg/StreamChunkStatus.msg"
  "msg/ControlEvent.msg"
  "msg/TaskState.msg"
  "srv/AcquireControl.srv"
  "srv/RenewControl.srv"
  "srv/ReleaseControl.srv"
  "srv/Stop.srv"
  "srv/StartEpisode.srv"
  "srv/EndEpisode.srv"
  "srv/SubmitFollower.srv"
  "srv/GetStreamAnchor.srv"
  "srv/QueryStreamCommand.srv"
  "srv/ResetSimulation.srv"
  "srv/PrepareTask.srv"
  "srv/StartTask.srv"
  "srv/GetTeacherChunk.srv"
  "srv/GetTaskState.srv"
  DEPENDENCIES builtin_interfaces sensor_msgs trajectory_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
