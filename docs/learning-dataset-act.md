# 学习数据转换与 ACT 训练（DSH-035）

本页描述 ROS-free 学习侧的设计边界；逐步操作、预算/恢复/日志定位和容器挂载见 [学习训练操作手册](learning-training.md)。

本页只描述 ROS-free 学习侧。MCAP、控制审计和导出仍由 034/`tools/run/stack_export.py`
负责；学习进程不导入 `rclpy`，也不重新实现控制或审计。

## 1. 输入与边界

- 正式选择：`tools/run/stack_campaign.py select` 的 `SELECTED` JSON，前 50 个合格
  formal teacher 成功；第 10/20/30/40/50 个成功（零基 9/19/29/39/49）为验证。
- 每个 attempt 的导出位于 `<attempt_dir>/stack/<run_id>/export/`，至少包含
  `labels.json`、`raw_predictions.json`、`frame_index.json`、`quality.json`、
  `summary.json` 和 `rgb/`。
- `labels.json` 的标签是观测 `t` 对应的实际执行参考 `t+50 ms`；原始 teacher 60 点预测
  在 `raw_predictions.json`。转换逐帧核对 capture/label sim 时钟、50 ms 周期、执行
  block 边界、PNG/RGB hash 与三相机映射；不以数组相邻关系假定 20 Hz。
- 只读原始导出。转换输出先写 `.partial-*`，成功后原子改名；失败留下 `FAILED.json`
  半成品，不会被当作成功 manifest。

## 2. 两个独立数据集

`configs/learning/act_dataset.yaml` 固定：

- 45 条 `train` 和 5 条 `validation` 分别由两个 `LeRobotDataset.create` 根写入。
- 三路 RGB 320×240，H.264 `RGBEncoderConfig(vcodec=h264, g=2, crf=18,
  preset=veryfast)`；视频/数据文件大小有上限，图像写入同步有界。
- 16 维 state/action 顺序和单位来自
  `profiles/robots/panda_stack_learning_v1/profile.json`。
- `action` 是每帧 `t+50 ms` 实际执行参考；训练读取时用
  `delta_timestamps action = 0..59 × 0.05 s`，末尾由 LeRobot 生成
  `action_is_pad`，不跨 episode。原始 teacher 预测另存为 `raw_action_chunk`。
- 转换只用 `train/meta/stats.json` 构建并保存 processor 和 stats 摘要；
  `validation` 的 stats 不参与拟合。
- `manifest.json` 记录每个 source episode 的 attempt/export 引用、源文件 hash/字节、
  dataset episode 映射、帧数、起止 sim 时钟、分区、split 文件 hash、profile/LeRobot/
  配置/代码 hash 和 processor 文件 hash。

公共转换命令：

```bash
export LEARNING_SOURCE=/home/ljk/work/sim_dual_arm/third_party/src/lerobot-act-v0.6.1/src
export PYTHONPATH=$PWD/components/learning/src:$PWD/libraries/run_logging/src:$LEARNING_SOURCE
$PROJECT_PYTHON tools/run/learning.py convert \
  --select var/campaigns/panda_stack_v1/formal-select.json \
  --output-root var/learning/panda_stack_learning_v1
```

## 3. 训练模式与预算

`configs/learning/act_training.yaml` 提供三个可改模式（不是代码常量）：

| 模式 | 默认步数 | 数据 | 验证 |
| --- | ---: | --- | --- |
| `smoke` | 128 | 前 2 条 train | 否 |
| `overfit` | 500 | 1 条 train | 否 |
| `full` | 5000 | 完整 45/5 | 是 |

`full` 在训练前重新核对 45/5、互斥、固定验证成功位置、每 episode 帧数和所有
manifest 文件/source hash；数据不足或 synthetic 直接拒绝。smoke/overfit 可用
`--allow-synthetic` 跑合成组件预检，但 manifest/报告明确标记 synthetic，不冒充正式
50 条。

预算账本 `var/learning/training-budget.jsonl` 是追加式 JSONL，跨进程 `flock`；在真正
训练前先写 RESERVED/STARTED。异常、重启、保存和验证墙钟都计入；遗留 RUNNING 在下一
次进程中被一次性 `ORPHAN_CHARGED` 保守计费，不能通过重启清零。任务累计上限 14400 s。

```bash
# 可加 --data-root /path/to/DATA_ROOT 指向独立数据集
$PROJECT_PYTHON tools/run/learning.py train --mode smoke
$PROJECT_PYTHON tools/run/learning.py train --mode overfit
$PROJECT_PYTHON tools/run/learning.py train --mode full
$PROJECT_PYTHON tools/run/learning.py verify-data --mode full
$PROJECT_PYTHON tools/run/learning.py budget-status
```

本机权重缓存不在隔离工作树时，显式设置
`RESNET18_WEIGHTS=/home/ljk/work/sim_dual_arm/.build/cache/torch/hub/checkpoints/resnet18-f37072fd.pth`；
训练会校验配置中的 SHA256，且 `pretrained_backbone_weights=None` 后严格加载本地
ResNet18，不会隐式下载。

训练使用上游 `ACTPolicy.forward`（含 `action_is_pad`），验证使用 train processor 和
masked L1；模型选择只看验证。检查点默认 `latest.pt` + `best.pt` + 阶段结束
`final.pt`，含 optimizer/scheduler/RNG/step/config/budget 引用；保存前检查输出盘余量，
不删除其他文件。事件日志写入共享 `robot_run_logging`，另有
`metrics.jsonl`/`metrics.csv`（字段带 step 和单位）。

## 4. 离线模型包

```bash
$PROJECT_PYTHON tools/run/learning.py bundle \
  --run var/learning/runs/<run-id> --output var/learning/bundles/<name> \
  --checkpoint best.pt
$PROJECT_PYTHON tools/run/learning.py verify-bundle --bundle var/learning/bundles/<name>
```

bundle 包含 `model.safetensors`、`config.json`、pre/post processors、训练 stats、
profile/observation/action 摘要、dataset/split/seed/配置/上游引用和逐文件 SHA256。
加载路径只使用 safetensors；不执行任意 pickle，不下载权重，不裁剪输出。导出前用同一
实际 batch 做加载前后预测一致检查。

`dual_arm_learning.bundle.load_bundle()` + `predict_action_chunk()` 是给 036 的 typed
入口：每次调用上游 `predict_action_chunk`，返回新鲜完整 `(60,16)`；不使用
`select_action` 队列，不覆盖左臂。真实输出若不符合契约，由监督者记录和讨论，不用
脚本补救。

## 5. 组件测试

学习 venv 不含 pytest；监督者提供纯 Python 覆盖层。示例：

```bash
export PYTHONPATH=$PWD/components/learning/src:$PWD/libraries/run_logging/src:$LEARNING_SOURCE:/home/ljk/work/sim_dual_arm/.build/learning-test-tools
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1
$PROJECT_PYTHON -m pytest tests/unit/test_learning_*.py -q
DSH035_HEAVY=1 $PROJECT_PYTHON -m pytest tests/unit/test_learning_conversion_integration.py -q
```

重测试会生成 50 条明确标记 synthetic 的小导出，验证两个独立数据集、train-only
processor、未来 60 点顺序、末尾 padding、颜色通道和 hash 拒绝；它不读取正式 campaign。
