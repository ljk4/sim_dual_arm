# DSH-003：可控的中断收尾（R1）

前置：R2/R3 已验收，监督者提供其快照。只处理 R1，保持现有接口和脚本行为，不提交 Git，不启动其他代理。

允许修改 runtime 包中的 node.py，按需新增 shutdown.py，局部修改 recording.py（仅收尾所必需），新增 tests/integration/test_runtime_shutdown.py 和纯 Python 的相关测试。不要同时进行 RuntimeNode 大规模职责拆分、配置重构或通用记录改造。

问题：episode worker 是 daemon，main 在 SIGINT 后直接销毁 ROS 资源，导致 worker 未结束、没有 manifest，却退出 0。

设计要求：

1. 使用当前已安装 Jazzy 的 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`，显式处理 SIGINT/SIGTERM。信号处理器只记录停止请求，ROS/锁/文件操作放在正常执行路径。
2. 提供清楚的 request_shutdown / worker 完成检查或等价入口。停止接受新 episode，通知当前 episode 停止；executor 继续运行，直到 worker 完成控制清理与 writer 收尾，之后 join 并销毁 ROS 资源。不要只在现有 finally 增加阻塞 join。
3. PREPARING、ACTIVE、ENDING、尚无 episode 的 IDLE 均可退出；SIGINT/SIGTERM 有明确非零退出状态（建议 130/143），正常 --once 仍为 0，运行失败为非零。重复信号不能绕过普通收尾导致成功误报。
4. 最外层 worker finally 保证完成事件被设置；失败原因保留首要故障与 R3 清理错误。资源销毁只能发生在 worker 不再访问它们之后。
5. 提供有限的 shutdown grace（可用正数 CLI --shutdown-timeout，默认 15 秒）。超时必须明确 SHUTDOWN_TIMEOUT、非零退出，保留 partial 目录。必要的最终强制退出路径必须清晰标注；不能为了退出去阻塞在无期限的 join/flush 中，也不能在活跃 worker 上调用 destroy_node。不要宣称强制超时保证完整 manifest。
6. 正常可写存储下，动作中断应产生带原因的失败 manifest；如果中断发生在记录收尾过程中，已完成数据的完整性与进程中断状态不得相互矛盾或被静默覆盖。当前阶段 complete=false 表示失败；后续通用记录语义在独立任务处理。
7. 保留清晰线程和资源所有权注释，避免新增泛化的生命周期框架。

新增真实 ROS 子进程集成测试，可通过启动 mock backend 进程和 RuntimeNode 进程，订阅状态/命令后发送信号。测试必须有自己的超时和 finally 清理。验证至少：PREPARING 时 SIGINT、动作中 SIGINT/SIGTERM、ENDING 时中断、IDLE 退出、正常 --once、收尾超时失败。允许测试用可控注入延迟覆盖 ENDING/超时，但要保持真实 ROS 资源和进程退出路径，不用伪造模块替代 rclpy。

宿主机不提供 ROS；你运行纯 Python/Ruff 检查，ROS 测试由监督者在挂载候选源码的 Jazzy 容器运行。测试中 mock 后端源码位于 `/workspace/tests/doubles/ros/mock_robot_backend`，监督者会将其加入 PYTHONPATH；可通过 `python -m mock_robot_backend.node` 启动。测试输出使用 tmp_path；不得依赖主工作区旧镜像中的 runtime 源码。

交付实际执行结果、未执行检查、修改文件、退出顺序、超时路径限制。首轮上限 20 分钟，测试失败由监督者定点反馈。

## 首轮审查后的补充边界

首轮已有 78 项检查通过，独立动作中断探针也通过，但审查发现了未被这些测试覆盖的路径。允许一次定点返工，最多 10 分钟：

- 测试延迟仅放 tests 内，通过真实 ROS 子进程的测试 bootstrap 注入，不增加正式运行环境开关；覆盖 recorder.finish 内中断和确实发生的重复信号。
- 记录收尾错误保留首要故障与清理故障；状态发布错误不能跳过必要清理。
- IDLE 的运行异常也需非零退出，收尾中的 executor 异常不能未经等待就误报超时。
- 额外允许修改 `deploy/docker/runtime/Dockerfile`：test target 复制 mock 测试替身；runtime 的 exec-form CMD 直接运行 `python -m dual_arm_runtime.node --once`，避免 `ros2 run` 的子进程层阻断 Docker SIGTERM。
- 修正测试替身的 Python 路径，并验证默认重建测试镜像不依赖监督者额外设置的 PYTHONPATH。

监督者负责重建候选镜像、执行默认测试命令及实际 Docker 信号验收；dsh 继续只运行宿主机可用检查。详细反馈和原始结果保存在本地任务运行目录。
