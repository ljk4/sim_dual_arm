cmake_minimum_required(VERSION 3.16)
project(sim_api_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(robot_api_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskState.msg"
  "srv/GetTaskState.srv"
  "srv/PrepareTask.srv"
  "srv/ResetSimulation.srv"
  "srv/StartTask.srv"
  DEPENDENCIES builtin_interfaces robot_api_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
