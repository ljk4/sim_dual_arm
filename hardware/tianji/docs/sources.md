# 设备源码归属

本包的设备实现取自 `TianJi_Robot_outside/TianJi_READY/lizhen_ros2_ws/` 中的天机设备源码。

| 本包路径 | 来源路径 |
| --- | --- |
| `tianji/apex_sdk/` | `src/apex_ros2/apex_sdk_new/python/apex_sdk/` |
| `tianji/arms.py` | `src/deeptouch_robot_hal/src/robot_hal/device/actuator/robot_apex.py` |
| `tianji/grippers/` | `src/deeptouch_robot_hal/src/robot_hal/device/actuator/gripper/changingtek_{closed_loop,90c}.py` |
| `tianji/grippers/serial_ports.py` | `src/deeptouch_teleop/control/gripper_controller.py` 中的串口设备识别 |
| `configs/devices.yaml` | `Record/config/record_config.yaml` 与 `src/deeptouch_robot_hal/config/apex_arms_only.toml` 中的设备参数 |
| `common/gripper.py` | `src/deeptouch_robot_hal/src/robot_hal/device/actuator/gripper/base.py` |
| `common/camera/capture.py` | `Record/record_utils/camera_capture.py` |
| `apex_protocol.md` | `src/apex_ros2/apex_sdk_new/doc/SDK通信协议文档.md`，示例地址已去除 |

设备源码目录未附独立 LICENSE 文件，本包不为这些源码另行声明开源许可。对外分发时需确认原权利人的授权条款。MinimalModbus、PySerial、RealSense、OpenCV 和 FFmpeg 依其各自发行版许可使用。
