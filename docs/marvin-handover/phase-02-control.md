# P02A 标准控制接入与执行映射

状态：2026-11-12 已交付，实际结果见[交付记录](deliveries/phase-02a.md)。本文件保留阶段要求；完成后进入 P02B。

前置：P01A 完成。目标：Marvin 通过现有 Robot API 完成受限运动、反馈和停止；本阶段无任务插件、Teacher 或学习模型依赖。

## 接入任务

1. 在现有后端装配入口按所选场景/机器人建立场景与 I/O，替换 [worker.py](../../simulation/mujoco/src/robot_backend_sim/runtime/worker.py) 固定 Panda 配置加载和 [worker_owner.py](../../simulation/mujoco/src/robot_backend_sim/runtime/worker_owner.py) 固定 `build_scene/PandaIO` 构造。机器人选择与任务插件选择分别完成。
2. 基于 P01 `MarvinBinding` 实现机器人 I/O。公开顺序是左 7 关节、左总开度、右 7 关节、右总开度；物理 qpos 当前右臂在前，必须按具名 qpos/dof/actuator/TCP/指面绑定读写。
3. 收敛总开度测量、开度到闭合量、速度符号和执行器命令的正反映射。复用或迁入 P01 场景中的机器人语义，controller、任务和诊断入口不各写一套。
   P01A 的 `5e-4 m` 为离线档案/模型一致性容差，控制跟踪阈值需要独立实测。总开度与执行器闭合量方向相反，速度反馈和启用速度前馈时的映射均须验证，实际反馈不裁剪。
4. 沿 control、motion、接触跟踪调用链解除 `Scene/PandaIO/PandaControlConfig` 具体类型及 Panda 名称/限位依赖。以真实消费者需要的能力替代，保留同一编译场景、线程归属、profile/配置一致性和有限值验证；只改类名或删除类型检查不算完成。
5. 建立 Marvin backend、control、stream、follower 配方。公开限位/速度读 profile 或准备时核对，增益、加速度、jerk、跟踪阈值和超时按实际运动验证。保留轨迹、锚点、租约和首次停止原因。

## 无任务插件的启动路径

当前 `prepare --scenario ... --role teacher` 会要求 collection 阶段引用齐备，P02A 不走此入口。backend 配方同时省略 `robot.task` 与 `robot.task_mode`；使用已有 direct `--template` prepare，再启动真实 backend。

现有 simulation 节点仍加载 `sim_api_msgs`，无任务插件也需要该 SDK。runtime 容器中使用现有显式包选择构建 `robot_api_msgs` 与 `sim_api_msgs`；不依赖 Panda Teacher 扩展：

```bash
# runtime 容器 /workspace；构建前关闭占用旧 SDK 的本次测试进程
runtime/ros2/scripts/run.sh build-ros-sdk --shared-root /shared \
  --package robot_api_msgs --package sim_api_msgs

# simulation 容器 /app；先创建本阶段真实配方，将占位符替换为实际值
scripts/run.sh prepare --template "/shared/scenarios/marvin_handover/configs/simulation/backend/<control-recipe>.yaml" \
  --seed 0 --run-id "<run-id>" --output "/data/prepared/<run-id>"
scripts/run.sh backend --config "/data/prepared/<run-id>/backend.yaml" --run-id "<run-id>"
```

SDK 构建使用现有挂载根和选择标记；切回 Panda 示教回归前按 Panda 方案重建所需 SDK。先做公共 Robot API 请求/反馈集成测试，不借用 runtime 的 Teacher 或 Policy 动作源构造控制入口；当前 runtime 已无 motions 执行模式。

## 完成条件

- 通过真实 Robot API 验证单臂运动、另一臂保持、两臂协调、左右夹爪开合与总开度反馈，命令和实测按相同具名顺序对应。
  首轮使用 P01A 已验证区域附近的低幅目标；关节限位内的任意目标不自动视为碰撞安全，超出已验证区域时先取得必要碰撞依据。
- 至少一个有限 SubmitFollower 窗口经真实受限控制执行，保存请求、接受参考、实测状态、执行反馈和终态；写入 ctrl 或命令接受不单独算通过。
- 越限拒绝、运行中停止及相关控制异常能有界收尾，保留真实原因；持物夹爪停止时不自动张开。
- owner/controller/运动循环通过选定机器人能力工作，不增加 Marvin/handover 行为分支。受影响的 Panda I/O、控制、跟随和停止回归通过。

交付 `deliveries/phase-02a.md`。本阶段完成后进入 P02B；真实持物、交接候选路径和任务状态机分别在 P02B、P03 验证。

目录导航：[P02 总览](phase-02-motion.md) · [前一阶段 P01A](phase-01-readiness.md) · [下一阶段 P02B](phase-02-contact-kinematics.md)
