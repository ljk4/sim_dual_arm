# 天机硬件后端

提供 TianJi/APEX 双臂、两只 Changingtek CTAG2F90-C 夹爪、RealSense RGB 与 UVC 视触觉相机的设备实现及独立 Docker 环境。设备装配通过同一份 YAML 配置构造各设备实例；导入、配置校验与对象构造均不连接设备。

## 目录结构

| 目录 | 职责 |
| --- | --- |
| `src/robot_backend_hardware/common/` | 设备生命周期接口、夹爪服务循环、相机采集和配置基础 |
| `src/robot_backend_hardware/tianji/` | APEX 双臂、90C 夹爪、设备配置与装配 |
| `src/robot_backend_hardware/tianji/apex_sdk/` | 随包提供的 APEX 通信实现 |
| `configs/` | 已填入设备参数的 `devices.yaml` 与容器环境变量 |
| `docker/` | Dockerfile、Compose 与固定依赖 |
| `scripts/` | 配置检查与离线测试入口 |
| `tests/` | 使用测试替身的驱动、协议与配置回归测试 |
| `docs/` | SDK 协议与源码归属 |

机器人和任务无关的实现归入 `common/`，机器人专属实现归入 `tianji/`。跨端 Robot Profile、观测/动作档案和任务配方的归属为 `shared/scenarios/<robot-id>/`。当前包提供设备 I/O 层；ROS 2 Robot API、轨迹执行调度、时间同步和 Runtime 接入由后续适配实现。

## 启动环境

从仓库根目录执行：

```bash
./env.sh build hardware
./env.sh up hardware
./env.sh exec hardware scripts/check_config.sh
./env.sh hardware
```

镜像提供 ROS 2 Jazzy、Python 3.12、NumPy、Pydantic、PyYAML、MinimalModbus、PySerial、RealSense、OpenCV、FFmpeg 和 V4L2 工具。源码与配置只读挂载为 `/app`，公共代码挂载为 `/shared`，运行输出挂载为 `/data`。容器使用宿主机网络并挂载 `/dev`、`/sys` 和 `/run/udev`，可访问 USB、串口和相机节点。容器启动后保持空闲，设备启动由驱动调用者管理。

## 设备配置

[configs/devices.yaml](configs/devices.yaml) 已包含现有设备参数，容器直接使用。`scripts/check_config.sh` 默认检查该文件并打印展开后的参数，也支持 `--config <path>`。

| 设备 | 当前配置 |
| --- | --- |
| APEX 双臂 | 控制器 `172.16.100.2`，臂名 `arm_left` / `arm_right`，启动不自动使能 |
| 左夹爪 | `/dev/gripper_left`，USB 物理口 `8-1.1` |
| 右夹爪 | `/dev/gripper_right`，USB 物理口 `1-3.3` |
| 夹爪协议 | Modbus RTU，115200 baud，ID 1，90 mm 行程，`invert_stroke=true` |
| RGB 相机 | `cam_high` / `cam_left_wrist` / `cam_right_wrist`，按配置中的 RealSense 序列号识别 |
| 视触觉 | 四路 V4L2 物理路径，自动曝光，每路独立 FFmpeg 进程 |
| 图像 | 默认 640×480、30 FPS |

夹爪识别优先匹配 USB 物理端口，再使用现有设备链接，最后尝试序列号。更换接线或设备时修改同一份 `devices.yaml`。

## 设备装配与接口

```python
from robot_backend_hardware.tianji.config import load_config
from robot_backend_hardware.tianji.assembly import build_devices

settings = load_config('/app/configs/devices.yaml')
devices = build_devices(settings)
# devices.arms、devices.grippers['left'/'right']、devices.cameras
```

装配结果提供实际驱动对象。设备生命周期由调用者显式管理，`start()` / `start_all()` 打开设备，`stop()` / `stop_all()` 关闭资源。双臂启动默认不自动使能。

| 设备 | 命令和反馈 | I/O 约束 |
| --- | --- | --- |
| APEX 双臂 | 每臂 7 个关节，关节命令和反馈为度；笛卡尔为 `[x,y,z,qx,qy,qz,qw]`，XYZ 为米 | TCP 9100 命令、单一 UDP 9101 状态接收；跟踪模式显式选择 `JOINT` 或 `CART` |
| 90C 夹爪 | `leftarm/rightarm_gripper_cmd_pos` 与 `*_state_pos` 为开口米数；底层驱动使用毫米 | `send_cmd()` 写入目标邮箱，单一服务线程调用 `service_step()` 完成串口写入后读取；`invert_stroke=true` 映射天机安装方向 |
| 相机 | `get_frame()` / `get_frames()` 返回 RGB HWC 数组，默认 640×480、30 FPS | RealSense、OpenCV V4L2、FFmpeg V4L2 三种后端；每路视触觉独立 FFmpeg 进程 |

双臂驱动保留跟踪前停稳、反馈同步、单步限幅、失败退避、故障检测和状态流恢复。夹爪断线或读失败时返回空测量，双臂只报告实际支持的臂状态。上述设备字段尚未映射到跨端 Robot API。

APEX 在线目标由控制器 OnlinePlanner 平滑。配置中的 `poll_hz` 用于状态流恢复请求，读取调度由调用者提供；实际控制周期、左右臂命令时间差和轨迹生效语义需要在执行后端接入时确定。

## 离线测试

测试使用 Fake SDK、Modbus 寄存器和图像管道，并禁止网络连接。在仓库根目录将源码送入已有的 Python 项目容器后执行：

```bash
tar -cf - hardware shared/common/tools |
  ./env.sh exec hardware bash -ec '
    test_root=$(mktemp -d /tmp/tianji-device-tests.XXXXXXXX)
    trap '\''rm -rf -- "$test_root"'\'' EXIT
    tar -xf - -C "$test_root"
    export PROJECT_ROOT="$test_root"
    export SHARED_ROOT="$test_root/shared"
    bash "$test_root/hardware/tianji/scripts/test.sh" -q "$test_root/hardware/tianji/tests"
  '
```

来源与许可信息见 [sources.md](docs/sources.md)，APEX 通信格式见 [apex_protocol.md](docs/apex_protocol.md)。
