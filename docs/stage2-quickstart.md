# 阶段 1～2：构建、运行与验证

当前实现是 CPU-only 的测试控制与采集链路。mock 后端提供合成关节反馈与 RGB 帧，runtime 通过公共 ROS API 执行脚本目标、同步观测并记录原生 rosbag2/MCAP。所有记录标记 `source=mock`、`test_only=true`，不作为正式示教数据。

2026-10-30 审查确认成功链路仍通过，但发现人工中断、观测身份和控制清理缺口。当前先进行阶段 2R 整改，见 [审查记录](maintainability-review.md)；下文演示命令不代表所有中断/故障路径已覆盖。

## 1. 运行要求

- Linux 上可访问的 Docker Engine，以及宿主机 Python 3，用于调用启动脚本。
- 首次构建需要下载 ROS/Python 镜像及 apt/Python 依赖。
- 本阶段不需要 GPU、真机、宿主机 ROS 或模型权重。

默认入口直接使用 Docker CLI，因此不要求 Compose 插件。`deploy/compose/mock.yaml` 提供可选的 Compose 定义；当前宿主机缺少 Compose 插件，实际容器间验证使用的是 Docker CLI 入口。

## 2. 构建

从仓库根执行：

```bash
bash tools/build/stage2.sh
```

生成以下本地镜像：

| 镜像 | 用途 |
| --- | --- |
| `sim-dual-arm/ros-base:stage2` | ROS Jazzy、接口构建、固定版本 Python 开发工具 |
| `sim-dual-arm/backend-mock:stage2` | backend-sim Dockerfile 的测试专用 mock target |
| `sim-dual-arm/runtime:stage2` | 公共接口客户端、episode、观测、MCAP 记录与审计 |
| `sim-dual-arm/learning:protocol` | 无 ROS 环境中的独立 Policy SDK 生成和导入验证 |

learning 镜像当前只覆盖协议层，不包含训练框架或模型服务。Gazebo 后端和真实设备驱动仍属于后续阶段。

## 3. 一次完整的控制采集演示

```bash
bash tools/run/mock_collect.sh
```

启动器会创建唯一命名的临时网络、数据卷和两个容器：

1. mock 后端发布公共 info、关节状态、64×48 RGB 图像及控制服务/action。
2. runtime 读取配置、校验 profile、获取控制 lease，执行三段 0.8 秒关节目标并持续记录。
3. 等待对应的 ACCEPTED / STARTED / COMPLETED 事件，停止并释放控制权。
4. 关闭 MCAP writer，校验样本引用、时间对齐、动作反馈和实际关节变化，写 manifest 并原子提交。
5. 在单独的离线容器中重新读取和审计记录，将数据导出到宿主机项目目录，最后清理本次创建的容器和网络。

成功后输出 `complete: true`、观测/帧/命令数量、关节运动范围和导出路径，例如：

```text
var/runs/sim-dual-arm-check-<id>/episode-<uuid>/
├── manifest.json
└── recording/
    ├── metadata.yaml
    └── recording_0.mcap
```

默认项目名带随机后缀，输出目录必须尚不存在。需要指定输出时：

```bash
python3 tools/run/mock_collect.py --output var/runs/my-first-mock
```

只有导出成功后才删除本次新建的数据卷；未能导出时会打印保留的数据卷名。经正常错误收尾的失败记录位于 `.partial-episode-*`，`manifest.complete=false`，不会作为完整 episode 提交。当前信号中断可能来不及生成 manifest，不能将进程退出码 0 单独当作完整记录的依据。

## 4. 修改脚本与机器人语义

| 修改内容 | 文件 |
| --- | --- |
| mock 关节顺序、单位、限位和相机尺寸 | `profiles/robots/mock_dual_arm_v1/profile.json` |
| 观测必需项、允许的源时间偏差/年龄 | `profiles/observations/mock_rgb_joint_v1.json` |
| 动作空间和关节顺序 | `profiles/actions/mock_joint_position_v1.json` |
| 三段脚本目标、超时与 lease 参数 | `configs/runtime/mock_collect.yaml` |

profile 修改后，观测/动作配置必须同步匹配。运行配置不携带真实设备地址；当前 runtime 会拒绝非 mock/test-only 后端，避免把测试脚本直接接到真机。

首次模型仅有左右各两个测试旋转关节与两个虚拟夹爪开度关节，属于接口测试夹具，不代表最终机械臂选型。

源码/配置变化后重新执行构建脚本。契约从唯一来源生成：ROS 类型位于 contracts/ros，Policy proto 位于 contracts/policy/v1；运行时不加载 docs 中的旧迁移占位文件。

## 5. 验证命令

原生 ROS/MCAP 测试：

```bash
docker build --target test -t sim-dual-arm/runtime-test:stage2 -f deploy/docker/runtime/Dockerfile .
docker run --rm --network none sim-dual-arm/runtime-test:stage2
```

测试包括：轨迹插值、非法维度/数值/限位、命令去重与乱序、过期 lease、停止与控制权交接、观测因果对齐、配置不匹配、MCAP 关联检查、损坏检测与不完整记录保留。

构建测试镜像后，还可通过真实 ROS 服务/action 检查后端，再执行完整采集：

```bash
python3 tools/run/mock_collect.py --probe-backend
```

该探针只接受 test-only mock，覆盖陈旧上下文、独占控制权、完整轨迹、重复请求、action 取消、停止保持和释放后的 lease 拒绝。

独立 Python 环境中的 SDK：

```bash
docker run --rm --network none sim-dual-arm/learning:protocol
```

本机开发可自行建立 Python 3.12 venv，安装 requirements-dev 和两个纯 Python 项目：

```bash
uv venv --python 3.12 .build/dev-venv
uv pip install --python .build/dev-venv/bin/python -r deploy/docker/requirements-dev.txt -e contracts/python -e libraries/backend_core
.build/dev-venv/bin/ruff check .
.build/dev-venv/bin/ruff format --check .
.build/dev-venv/bin/python -m pytest -q
```

本机 pytest 默认只运行不依赖 ROS 的 unit 测试；原生 MCAP 测试在上述 Docker target 中运行。

## 6. 编程与维护约定

- 公共数据采用有类型的不可变值；配置验证、协议转换、控制状态、观测整理、记录与审计分别放置。
- 后端核心不 import ROS/厂商 SDK，runtime 不 import 后端执行实现；ROS 转换辅助库只用于两个 ROS 侧适配器。
- 目标是 ROS 回调保持非阻塞；mock action 已由 tick 推进并完成 Future，runtime 的 episode 编排在独立工作线程，但同步写盘仍占用 ROS 回调，属于待整改项。
- 机器人时间用于源样本与接收截止时间，单调时钟用于 lease、执行插值与调用超时。
- 不覆盖 rclpy.Node 的内部字段；节点成员采用明确的业务名称，例如 `_run_context`、`_service_clients`。
- 错误在协议、线程和 I/O 边界转换为可记录原因；清理失败不把 episode 标为成功。
- 统一使用 Ruff 的格式、导入排序和静态检查；新增测试围绕可观察的控制与数据行为。

当前控制能力是立即开始、顺序执行的 joint_position 轨迹，支持 lease、停止、取消、幂等命令结果与有限历史。世界复位、完整 Runtime 管理 API、真实标定/TF、策略服务、Gazebo、模型训练和真机控制尚未实现。
