# 架构重构归档（Phase 1～6）

本目录归档面向全仓代码瘦身、解耦与训练性能优化的 **6 份阶段重构方案**（均已执行完毕并通过量化验收）。

## 六阶段重构总览

| 阶段 | 任务文档 | 核心重构内容 | 验收结果 |
| --- | --- | --- | --- |
| **Phase 1** | [phase-01-cleanup-legacy-and-dead-splines.md](phase-01-cleanup-legacy-and-dead-splines.md) | 物理删除 S1 Mock 后端、S2/S3 非受限样条接口（`SubmitNative` / `ExecuteTrajectory`）、仿真端残留的 SciPy `chunk_plan.py` / `reference_compiler.py`、旧版 Schema（v2～v4）与废弃配置 | 删除 45+ 个历史文件，净减约 22,841 行，清理全部非流式控制死分支 |
| **Phase 2** | [phase-02-single-layer-pydantic-and-contracts.md](phase-02-single-layer-pydantic-and-contracts.md) | 公开 Config 与 Contract DTO 统一改为单层 Pydantic v2 `StrictContractModel`（消除私有 `_XxxModel` + `@dataclass` 双层套壳与手写 `__post_init__`）；将三次样条与受限跟随数学下沉至 `robot_contracts/trajectory/` | 全仓无私有 `_XxxModel` 套壳，`contracts/trajectory/` 集中维护，净减 8,639 行 |
| **Phase 3** | [phase-03-config-ssot-and-eliminate-native.md](phase-03-config-ssot-and-eliminate-native.md) | 控制配置收敛至 `profiles/control/` 单一数据源（SSOT）；全仓模块名、Python 类名（`StreamRequest`、`ExecutionFeedback`、`MotionController`、`ControlSession`）与 ROS 2 IDL（`StreamCommand.msg` 等）彻底去除 `Native*` 前缀 | 全仓源码与 ROS 2 IDL 中 `Native*` 残留清零，`target_count` 唯一定义于 `panda_follower_v1.yaml` |
| **Phase 4** | [phase-04-task-plugin-and-motion-composition.md](phase-04-task-plugin-and-motion-composition.md) | ROS 任务状态消息泛化为 `TaskState.msg`；抽离无 ROS 依赖的 `SimTaskPlugin` 协议与 `tasks/stack/` 插件；用 `BrakingController` + `GripperContactTracker` 显式组合拆分运动控制器；离线检查工具移入 `tools/` | 无 `*Mixin` 继承拆分，`simulation/` 单文件均 ≤ 720 行，任务插件可独立于 ROS 导入 |
| **Phase 5** | [phase-05-learning-pipeline-and-hash-unpinning.md](phase-05-learning-pipeline-and-hash-unpinning.md) | 合并 `act_contract.py` 入 `policy_config.py`；取消源 YAML 手写静态 SHA-256，改为准备阶段动态计算写入 Manifest；将会话锁、RGB 解码、准入与时间融合收归 `ChunkedPolicyPipeline` | `adapters/lerobot_act.py` 精简为纯模型适配器，源 YAML 配置无需手填文件哈希 |
| **Phase 6** | [phase-06-training-speedup-and-split-monoliths.md](phase-06-training-speedup-and-split-monoliths.md) | 训练默认切换为 `PyAV` 视频解码、`default_collate` 批处理、多 worker/`pin_memory` 与 `CUDA AMP`；按职责拆分学习端与运行端大文件及超长函数 | 全仓所有 `.py` 文件均 ≤ 720 行，核心域函数均 ≤ 85 行，较重构基线净减约 34,245 行 |

目录导航：[文档索引](../README.md) · [项目状态](../project-status.md) · [项目规范](../standards.md)
