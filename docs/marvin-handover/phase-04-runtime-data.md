# P04 Runtime 与小样本数据

前置：P03 至少一次完整示教与必要失败路径验证通过。目标：通过标准方案入口采集、导出并审计本任务数据；保留通用 runtime 与任务物理逻辑的边界。

runtime 内区分通用同步/租约/动作流/录制/收尾、Robot/Policy 接口适配和任务专用 Teacher/导出规则。两种机器人使用相同 Robot API 时复用已有客户端；仅实际协议或能力差异需要新增适配，不按机器人名称复制客户端。

## 方案准备与接口接通

1. 补齐本方案 binding 的 robot/observation/action、teacher/policy backend、runtime、campaign、dataset、training 与 policy_service 配方路径。模型保持未绑定，推理阶段再显式输入本次新 bundle。
2. 通过 shared scenario 的 collection 校验与 runtime 采集选择预检，再按 `--scenario marvin_handover` 构建实际 ROS SDK。新扩展必须进入所选 SDK，普通 Policy/control 路径保持不加载专用 Teacher 类型。
3. simulation/runtime 的 prepare 读取同一 binding 和 profile。专用 Teacher 客户端按 capability 选择扩展适配；TaskClient 继续负责通用生命周期，采集主流程不导入具体扩展包。runtime 不引入抓取位姿、双臂 IK 或接触评价。
4. 新方案使用独立命名空间、run/episode 身份、日志与 campaign 台账；种子、目录和 SDK 标记不落入 Panda 数据根。首轮单路完成即可，已有并行入口只需保持相关回归，不要求本任务新增并行调度能力。

## 录制、导出与质量规则

- 录制三路 RGB、实测机器人状态、TaskState、实际接受参考/执行反馈和事件。动作标签沿用当前受限执行参考在观测后 50 ms 的值；原始 Teacher 块另存，不能当作实际执行标签替换。
- 修改通用导出中固定 stack 阶段集合的假设，从当前方案的阶段定义与实际事件获得区间。缺少必需交接阶段、接稳/释放证据或时间对应关系时，导出不合格，不能静默退为全局可见性评价。
- 质量配方按交接阶段选择对象与相机。左手抓取关注左腕/全局，空中交接关注物块及两侧抓持区域，运输/放置关注右腕/全局与箱口。像素门槛和允许遮挡经真实渲染测量后填写；不要求左腕始终看见桌面，也不沿用堆叠 source/target 规则。
- simulation 提供分割/可见性事实，runtime 依据配方评价数据质量；任务成功仍来自 TaskState 的物理终态。二者分别保存，合格 episode 同时要求任务成功、录制完整、导出质量通过。
- 输出路径、export glob 和评估汇总统一来自选定配方/结果。当前 `stack/*/export` 是生产与消费端的真实约定；若改为任务中性布局，须同步 collect/export/learning/evaluation 及测试，不单改目录字符串。
- 保留原始时间戳、左右臂/夹爪状态与阶段边界。审计至少覆盖左手闭合、双手共持、左手释放和右手释放附近的图像、标签及实测反馈。

## 小样本预算与划分

首轮按以下有限规模建立链路，数值写入新方案自己的诊断配方：

| 用途 | 目标与预算 |
| --- | --- |
| pilot | 1 条合格试采，至多 3 次尝试；不进入训练/验证 |
| 诊断数据 | 6 条合格完整示教，至多 18 次尝试；失败与取消占用预算并保留原因 |
| training / validation | 按合格顺序前 4 条训练、后 2 条验证，筛选位置 `[4, 5]`；episode 与种子不交叉 |

沿用 campaign 现有 `pilot/formal` 用途时，“formal”只是工具的采集台账分类：这 6 条位于本任务独立诊断 campaign，dataset 明确 `diagnostic: true`，不据此成为正式训练证据。campaign target 与验证位置来自新配方；筛选输出的说明文字必须反映实际 6/4/2，不能仍写 50/45/5。

数据量足够后保存工具生成的 SELECTED 清单，转换器使用同一清单；不手工伪造成功台账、select 或 manifest。数据用于软件链路验证，固定布局小样本不证明泛化。确需增加规模时先给出具体失败原因和新预算，不自动扩成正式 50 条采集。

## 入口与完成条件

实现补齐后，从当前脚本提供的入口执行；将尖括号占位符替换为实际值，实际完整命令写交付记录：

```bash
# runtime 容器，/workspace；确认新方案及 SDK 已实现后执行
python3 shared/common/tools/scenario.py validate \
  --binding shared/scenarios/marvin_handover/binding.yaml --stage collection
runtime/ros2/scripts/run.sh build-ros-sdk --scenario marvin_handover --shared-root /shared

# 手动 Teacher backend 已按新方案 prepare 并启动；传实际 backend 日志目录
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose pilot \
  --count 1 --max-consecutive-failures 1 --backend-log "/data/runs/<teacher-run>/log"
```

后续正式台账分类的采集、`campaign --scenario marvin_handover status/select` 使用本方案诊断预算，不使用省略方案参数的 Panda 默认值。准备输出目录每次新建。

完成时具备 6 条合格数据、4/2 独立划分、完整审计结果；场景复位后重录不会继承上局任务状态。一次中断采集能停止控制、关闭录制并保留失败/中断台账。Panda 的采集选择、导出质量与标签测试仍通过。

新任务阶段、质量和专用导出内容通过配方或任务适配传入，通用录制/导出流程保持复用；具体 Teacher 扩展可以替换，而无需复制同步、租约、动作发送和进程收尾代码。

`deliveries/phase-04.md` 保存配方/清单、质量参数、原始与导出路径、审计边界帧、预算消耗和接口/阶段/路径耦合的修复结果。

目录导航：[任务书入口](README.md) · [前一阶段 P03](phase-03-task.md) · [下一阶段 P05](phase-05-learning.md)
