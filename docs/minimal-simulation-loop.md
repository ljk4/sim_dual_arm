# 最小仿真闭环：运行与结果

本机路径如下。迁移时先修改变量并确认挂载；Docker镜像存储位置与DATA_ROOT独立。

```bash
export ROOT=/home/ljk/work/sim_dual_arm
export DATA_ROOT="$ROOT/var"
export CAMPAIGN_CONFIG="$DATA_ROOT/campaigns/panda_stack_v1/campaign.yaml"
export LEARNING_OUTPUT="$DATA_ROOT/learning"
export BUNDLE="$LEARNING_OUTPUT/bundles/full-best"
export ROS_PYTHON="$ROOT/.build/native-dev-venv/bin/python"
export ROS_SDK="$ROOT/.build/ros-dev"
export POLICY_SDK="$ROOT/.build/generated/policy-sdk"
export ASSETS_ROOT="$ROOT/third_party/src/mujoco_menagerie"
export ASSET_SUMMARY="$ROOT/.build/assets/panda.json"
export LEARNING_VENV="$ROOT/.build/learning-dev-venv"
export LEARNING_SOURCE="$ROOT/third_party/src/lerobot-act-v0.6.1"
findmnt -T "$DATA_ROOT"
df -hT "$DATA_ROOT"
cd "$ROOT"
```

## 本轮已经完成

- MuJoCo双Panda、柱/桌/真实物理方块、全局与左右腕三RGB相机；右臂堆叠，左臂动作也由ACT输出。
- 52次正式尝试取得50条合格程序示教，21836帧；失败原始数据保留。45训练19676帧、5验证2160帧，两个独立LeRobotDataset，统计量仅来自训练集。
- RTX4050 6GB：真实smoke128、overfit500、full5000。累计训练约35.3分钟，峰值保留显存约1.17GiB；最佳5000步全验证归一化L1=0.0755101。
- 真实ACT动作已通过RPC→受限跟随→MuJoCo→新观测→再次预测执行。固定20新种子全部完成真实闭环执行，每例19～56个动作块；物理堆叠成功0/20，均后续跟随偏差超限受控停止。完整录制20/20、图像质量15/20合格；失败没有剔除。

本轮结论见[结果说明](simulation-loop-result.md)，逐种子证据见[自动评估报告](../var/learning/evaluation-final/report.md)和[evaluation.json](../var/learning/evaluation-final/evaluation.json)。报告目录含20段三视角预览与60张关键帧。

模型为LeRobot ACT/ResNet18，三RGB320×240+16测量位置输入、60×16位置输出，50ms动作周期，每次执行5点后重规划；同一跟随器处理teacher与ACT，保留ACK、C1接续、实测反馈与受控停止。左臂关节不做脚本待机替换。模型不读取物体真值/teacher阶段，不用IK补救。

## 日常查看与复现入口

```bash
# 台账和预算：不启动任务
"$ROS_PYTHON" tools/run/stack_campaign.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" status
cat "$LEARNING_OUTPUT/configs/generation.json"
cat "$LEARNING_OUTPUT/full-run.json"
cat "$BUNDLE/metadata/verification.json"

# 重新生成只读汇总；每次输出到新的目录，保留原报告
"$ROS_PYTHON" tools/run/stack_evaluate.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" --purpose evaluation \
  --output "$LEARNING_OUTPUT/evaluation-review" --no-media
```

本轮台账不可删除或换空目录重置次数。正式50和固定测试种子用过后，启动命令会按预算拒绝。新实验先明确预算/种子和配置，不覆盖本轮数据/模型来挑选结果。

单次ACT的公共命令如下；只能用于有剩余配额且明确授权的campaign：

```bash
"$ROS_PYTHON" tools/run/stack_campaign.py --project-root "$ROOT" \
  --config "$CAMPAIGN_CONFIG" run --purpose evaluation --mode act \
  --runtime-config configs/runtime/native_task_act.yaml -- \
  "$ROS_PYTHON" "$ROOT/tools/run/stack_once.py" \
  --campaign-config "$CAMPAIGN_CONFIG" \
  --runtime-config configs/runtime/native_task_act.yaml --policy-mode act \
  --bundle "$BUNDLE" --build-root "$ROS_SDK" --policy-sdk "$POLICY_SDK" \
  --assets-root "$ASSETS_ROOT" --asset-summary "$ASSET_SUMMARY" \
  --learning-venv "$LEARNING_VENV" --learning-source "$LEARNING_SOURCE" \
  --learning-cache "$ROOT/.build/cache"
```

采集入口见[程序示教](native-teacher-runtime.md)；转换/训练/恢复见[训练手册](learning-training.md)。本轮数据来源统一使用容器别名/source-data，所以训练建议按Docker挂载运行，不能直接修改manifest路径/hash适配宿主。

```bash
# 例：只核对已接受的数据。训练时把最后一行改为train及对应显式配置。
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
  -e LEARNING_CACHE_ROOT=/learning-output/cache/hf \
  -e HF_HOME=/learning-output/cache/hf \
  -v "$ROOT":/workspace:ro \
  -v "$LEARNING_VENV":/opt/host-learning-venv:ro \
  -v "$LEARNING_SOURCE":/opt/lerobot:ro \
  -v "$ROOT/.build/cache/torch":/learning-cache:ro \
  -v "$DATA_ROOT/campaigns/panda_stack_v1":/source-data:ro \
  -v "$LEARNING_OUTPUT/panda_stack_learning_v1":/learning-data:ro \
  -v "$LEARNING_OUTPUT":/learning-output \
  sim-dual-arm/learning:act-dev python tools/run/learning.py verify-data \
  --config /learning-output/configs/full.yaml
```

## 文件职责与证据

| 路径 | 用途 |
| --- | --- |
| configs/simulation | 场景、任务判定、控制/跟随约束；学习场景与旧诊断场景分开 |
| configs/runtime | teacher/ACT运行配方；ACT模板由启动器填真实模型身份 |
| configs/learning | 数据/模型/训练与推理参数；本轮实际配置另存var/learning/configs |
| components/backend_sim | 单物理owner、渲染、独立判定与私有程序teacher |
| components/runtime/ros | 精确观测、控制权/ACK、连续执行、录制/独立审计 |
| components/learning | ROS-free转换、ACT训练、模型包、策略RPC与夹爪准入 |
| tools/run | 公共命令与生命周期，不承载新的模型/物理实现 |
| var/campaigns/panda_stack_v1 | 全局台账、每次配置、原始MCAP、三相机、标签和失败 |
| var/learning | 接受的数据、训练配置/预算、检查点、完整模型和评估报告 |
| var/runs/dsh | 任务快照、首轮/唯一返工、独立验收；不作为产品运行入口 |

每个attempt的launch_result.json、manifest/task_result/quality分别回答进程/资源、录制、物理结果和图像质量。runtime及后端结构化事件、ACT预测延迟和原始输出证据均能按run/episode/request身份关联。ACT原始输出、夹爪准入目标、执行参考和实测必须分开查看。

本轮获批夹爪准入仅处理左右总开度0～0.08m，修正最多0.03m，超过拒绝；机械臂仍严格检查。原始ACT与准入数组在act-policy-log/predictions/*.npz，执行参考与实测在MCAP/export。该处理不提高模型精度，不改变跟随预算或硬限位。

## 已知限制

这是一轮最小工程闭环，尚未学会可靠堆叠。20次ACT均因后续目标偏差超过跟随预算而受控失败；5次右腕目标可见性也未通过质量门槛。未放宽约束、补脚本或更换测试种子。下一轮应先定位模型目标与跟随参考的偏差来源，并复核动态视野；真机驱动、双臂协作和更大规模训练仍属后续工作。完整状态和决策见[执行记录](simulation-loop-progress.md)。
