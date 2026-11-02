# DSH-005：S1b 提取控制客户端

前置：DSH-004 通过独立验收后才能分派。本任务扩大到一个完整职责，仍不开始物理仿真。保持 mock 外部行为与 R1～R3 的中断、错误汇总及清理保证。

## 范围

允许修改 runtime 包中的 `node.py`，新增 `control_client.py`；更新 `tests/integration/test_runtime_cleanup.py`，新增 `tests/integration/test_control_client.py`；仅按新模块边界调整 `tests/integration/runtime_shutdown_bootstrap.py` 的故障注入目标；在 `docs/stage2-quickstart.md` 增加职责/线程说明。其他源码、配置、协议和已有测试断言不在范围内。不提交 Git、不启动其他代理。

阅读以上文件及已验收的 `configuration.py`；必要时阅读 `tests/integration/test_runtime_shutdown.py` 和 `robot_ros_support/conversions.py`。不要重新设计整个 runtime 或通用 transport 层。

## 设计

1. 新 `ControlClient` 拥有 ROS service/action clients、lease ID、续租 future/时刻/锁以及清理状态。构造时显式注入 Node（只用于创建 ROS 实体和读取节点名称）、RuntimeRecipe、停止 Event、故障回调；不访问 RuntimeNode 私有业务字段，不传万能上下文，不使用继承/mixin/getattr 转发。
2. 公共方法围绕实际职责命名：等待 action 就绪、acquire(context)、execute(command_message)、renew()、cleanup()，lease ID 用只读属性供轨迹构造。node 仍拥有 episode context、观测、记录、worker、信号和状态。续租 timer 可以由 node 创建并调用 client.renew；注明 executor 与 worker 的状态所有权及锁。
3. 原 `_wait_future`、`_call`、获取 lease、发送 goal/等待 result、续租和 stop/release 清理迁移到新模块。node 通过明确公共方法使用；不保留另一套 lease/clients 状态，不为兼容测试留下空壳委派方法。node 的 `_run_episode_body` 暂时保留，其职责提取是后续任务。
4. cleanup 禁用续租并原子消费控制权；等待在途 renew、stop、release 各自独立尝试。清理时忽略 episode stop 标志、仍遵守超时和 rclpy 生命周期。全部错误聚合成原有 CONTROL_CLEANUP_FAILED 信息，通过故障回调交给 node，保留主故障；重复 cleanup 无重复 RPC。如果回调本身异常，不能跳过余下适用清理操作。
5. 不改变有效期、等待时间、goal result 语义或后端权限；协议强化留到 S3。正常和故障情况下都只允许 worker 退出后销毁 node。
6. 为 ROS clients/action client 的返回值与回调补必要类型；不能借机建立大量包装类、宽泛 Any 状态袋或循环依赖。说明方法在哪个线程调用，回调不得持锁做阻塞等待。

## 验收与交付

将原 cleanup 测试迁移到新职责，保留全部 11 个场景及其断言意义；不能通过删测试换通过。新增针对 acquire/execute/renew 的行为测试：租约设置、续租拒绝/超时通知、goal 拒绝、result 失败、停止请求中断等待、清理独立性。fixture 可使用显式小型 transport doubles，不需要启动真实机器人。

本机仅运行可用的纯 Python 回归和 Ruff，ROS 测试由监督者在 Jazzy 容器运行。交付实际执行命令、修改说明与未执行项。监督者另跑原生 MCAP、现有10项进程中断测试和真实双容器 mock 链路；测试故障注入仍只能在 tests 中，不添加生产测试环境变量。

首轮执行上限 20 分钟，最多一次定点返工 10 分钟。只有独立验收通过后才整合并开始 episode 提取任务。
