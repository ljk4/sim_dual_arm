# 程序示教、采集与标签重放

本机路径如下；更换机器或数据盘时统一修改变量，并确认数据盘已挂载。Docker镜像的存储位置独立于DATA_ROOT。

```bash
export ROOT=/home/ljk/work/sim_dual_arm
export PROJECT_PYTHON="$ROOT/.build/native-dev-venv/bin/python"
export DATA_ROOT="$ROOT/var/campaigns/panda_stack_v1"
export CAMPAIGN_CONFIG="$DATA_ROOT/campaign.yaml"
export ROS_SDK="$ROOT/.build/ros-dev"
export POLICY_SDK="$ROOT/.build/generated/policy-sdk"
export ASSETS_ROOT="$ROOT/third_party/src/mujoco_menagerie"
export ASSET_SUMMARY="$ROOT/.build/assets/panda.json"
findmnt -T "$ROOT/var"
df -hT "$ROOT/var"
```

现有campaign.yaml包含本机持久化路径。新机器可从configs/experiments/stack_campaign.yaml复制，检查paths的attempts_root/ledger/lock/run_lock及sdk_marker后使用；这些路径必须始终指向同一台账。不要换一个空目录来重置次数。

## 公共入口与职责

- stack_campaign.py：启动前RESERVED、运行/终态、次数/种子/存储、串行互斥及45/5选择；所有完整尝试先记账。
- stack_once.py：解析预留身份，生成配置，调用共用Compose启动器、等待退出、清理所属资源、审计/导出、保存launch_result.json。
- native_policy.py：共用后端/runtime的Compose生命周期；--stack-mode teacher只启动两服务，policy模式接入策略服务。源码/SDK/资产只读，数据目录单独挂载。
- native_stream.py：teacher/policy共用60×16预测、每块执行5点、50ms间隔、固定anchor、SubmitFollower、ACK查询/幂等重试、真实反馈和受控停止。
- backend的stack_task.py/teacher.py：独立物理判定及可选示教。推理部署是evaluation-only，不提供teacher/IK能力。
- stack_export.py：先独立审计MCAP，再重建逐20Hz的state+三RGB、实际参考标签及质量；学习转换不需要导入ROS。
- stack_replay.py：只重放已导出的applied标签，标记test-only/非ACT，共用同一任务入口和follower。

学习部署使用panda_stack_learning_v1/profile.json与panda_stack_learning.yaml，右腕FOV68度，三图320×240。profile摘要含标定；机械几何、限位与基线一致。旧panda_stack_v1是独立诊断基线，禁止混用标定。

## 一次采集

```bash
cd "$ROOT"
"$PROJECT_PYTHON" tools/run/stack_campaign.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" status

# 正式采集；试采则将formal改成pilot。种子与episode由台账分配。
"$PROJECT_PYTHON" tools/run/stack_campaign.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" run --purpose formal --mode teacher \
  --runtime-config configs/runtime/native_teacher.yaml -- \
  "$PROJECT_PYTHON" "$ROOT/tools/run/stack_once.py" \
  --campaign-config "$CAMPAIGN_CONFIG" --build-root "$ROS_SDK" \
  --policy-sdk "$POLICY_SDK" --assets-root "$ASSETS_ROOT" --asset-summary "$ASSET_SUMMARY"
```

同一attempt目录保存launch_result.json、配置快照、manifest/task_result/quality、stack下的原始MCAP与export。export内labels.json引用每帧rgb/<index>/<camera>.png及校验和；raw_predictions.json按块保存完整原始预测；preview/video仅供检查，不作为模型输入。运行时事件位于stack/<run>/runs/<run>/logs/runtime/<instance>；后端事件位于stack/<run>/<run>/log。

正式最多100次，取前50条物理成功、runtime正常、录制完整、视野合格且标签连续的数据；第10/20/30/40/50条合格样本验证，其余45训练。失败/中断/RESERVED崩溃仍计数。累计原始目录含失败与导出不超过30GiB，并检查剩余磁盘；软件错误立即停止批量尝试。

```bash
"$PROJECT_PYTHON" tools/run/stack_campaign.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" select
```

## 标签与时钟

每个capture t对应实际执行参考t+50ms。原始预测每5步才产生一次，不要求每帧重新预测；标签按真正STARTED/RETIRED/CANCELED块和整体STOPPING切换重建。pending、刹车后旧曲线和未执行raw尾部不作标签。完整state/三RGB可重建缺少的ObservationIndex，capture编号与原始预测observation编号分别保留；100ms间隔不能伪装50ms。夹爪原始/执行参考0与实测约0.04分别记录。

学习任务显式启用可靠传感流、32帧有界DDS历史及能力检查。缓存与观测时效检查仍有效，掉帧会造成质量拒绝。任务结束按真实Stop、整体及所有已接受块终态和参考事实收尾；查询与DDS乱序到达不能覆盖冻结终态，内容真正变化仍拒绝。第一条明确未接受的负ACK无执行终态可等待；未知ACK仍查询同一身份并保持受控停止。

Start之后计30秒仿真任务钟，初次等待也计时。释放、退开且14项物理条件连续2秒才成功。运行时正常托管结束可为stopped/TASK_SUCCEEDED；实际NativeExecution CANCELED不改成COMPLETED。task、runtime、recording、quality分别记录，进程退出0不等于合格数据。

## 仿真伺服校准

robot.velocity_feedforward仅在任务部署开启。固定Menagerie仿射PD原先期望速度为0，运动位置参考产生约kd/kp=0.1秒滞后；PandaIO按模型gainprm/biasprm加入已有参考速度项，夹爪采用半总开度的tendon速度。策略输出和训练标签仍是16维位置；底层ctrl与轨迹参考有明确区分，原生ctrl/force限幅照常生效，不改变模型参数、硬限位或跟踪容差。

只回放失败接近段的物理对比：最大位置误差0.0629→0.00939rad，最长连续超差1.992→0秒。另修复制动边界仅差1ns时累积速度相消造成的虚假加速度超限，改用整数剩余时间构造同一常减速度；限值保持不变。

## 标签重放

最多10次pilot包含teacher、重放及ACT；其中2次专门保留给ACT。重放必须使用原seed与相同profile/标定，按首个部署观察对齐首个记录观察，后续使用相对仿真时间；禁止直接使用两次运行的绝对时钟。来源文件/hash/映射保存于replay_source.json。

```bash
export TEACHER_SEED=1004
export LABELS="<已验收teacher导出目录>/labels.json"
"$PROJECT_PYTHON" tools/run/stack_campaign.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" run --purpose pilot --kind label_replay \
  --mode teacher --seed "$TEACHER_SEED" --runtime-config configs/runtime/native_replay.yaml -- \
  "$PROJECT_PYTHON" "$ROOT/tools/run/stack_replay.py" --labels "$LABELS" \
  --campaign-config "$CAMPAIGN_CONFIG" --build-root "$ROS_SDK" \
  --policy-sdk "$POLICY_SDK" --assets-root "$ASSETS_ROOT" --asset-summary "$ASSET_SUMMARY"
```

重放不进入正式集。policy来源没有teacher phase，视野输出全局时间统计，不能把它冒充示教phase合格；task物理结果和标签连续性独立。ACT部署随后复用相同控制与反馈链，仅由真实模型输出动作，三RGB+16测量之外的真值/phase不进入模型。
