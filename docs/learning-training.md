# 学习数据转换与 ACT 训练操作手册

本页是 DSH-035 的独立运维入口。所有数字/路径都可覆盖；不要把宿主绝对路径写进
仓库默认配置，容器和跨机器按下面的变量重新绑定。

## 1. 先确认环境

```bash
export ROOT="/home/ljk/work/sim_dual_arm"
export CAMPAIGN_ROOT="$ROOT/var/campaigns/panda_stack_v1"
export FORMAL_SELECT="$CAMPAIGN_ROOT/formal-select.json"
export DATA_ROOT="$ROOT/var/learning/panda_stack_learning_v1"
export OUTPUT_ROOT="$ROOT/var/learning/runs"
export CACHE_ROOT="$ROOT/.build/cache/huggingface-learning"
export PROJECT_PYTHON="$ROOT/.build/learning-dev-venv/bin/python"
export LEARNING_SOURCE="$ROOT/third_party/src/lerobot-act-v0.6.1/src"
export PROJECT_RUFF="$ROOT/.build/dev-venv/bin/ruff"

findmnt -T "$ROOT"
findmnt -T "$DATA_ROOT"
df -hT "$ROOT" "$DATA_ROOT" "$OUTPUT_ROOT"
```

数据盘/上游源码/权重缓存只读；`OUTPUT_ROOT` 与 `CACHE_ROOT` 必须可写。训练前会
校验 cache 不在 `DATA_ROOT` 内，并实际写探针；不会把 HF cache 写回接受的数据。
如本机权重不在工作树：

```bash
export RESNET18_WEIGHTS="$ROOT/.build/cache/torch/hub/checkpoints/resnet18-f37072fd.pth"
```

官方权重 SHA256：
`f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec`。

## 2. 正式 convert → verify → train

```bash
cd "$ROOT"
export PYTHONPATH="$ROOT/components/learning/src:$ROOT/libraries/run_logging/src:$LEARNING_SOURCE"

# 034 select JSON（前 50 合格；验证为成功顺序 9/19/29/39/49）
$PROJECT_PYTHON tools/run/learning.py convert \
  --select "$FORMAL_SELECT" \
  --output-root "$DATA_ROOT"

# 完整 45/5、source facts、文件 hash、固定验证位置
$PROJECT_PYTHON tools/run/learning.py verify-data --mode full \
  --data-root "$DATA_ROOT"

# 先少量真实数据 smoke，再单 episode overfit，最后完整 45/5 full
$PROJECT_PYTHON tools/run/learning.py train --mode smoke --data-root "$DATA_ROOT"
$PROJECT_PYTHON tools/run/learning.py train --mode overfit --data-root "$DATA_ROOT"
$PROJECT_PYTHON tools/run/learning.py train --mode full --data-root "$DATA_ROOT"
```

`convert` 不读取/修改原始 MCAP；它只读 034 已验收 export，并拒绝 task 失败、runtime
失败、录制不完整、episode/seed 不一致，即使 `quality.json` 为 true。`select` 不能
单独伪造成功；dataset manifest 保存逐 episode 的 `source_facts` 和 hash。

## 3. 预算、恢复、日志和失败

- 稳定身份是训练 run（checkpoint/report 的 `run_id`）；每次进程调用有唯一
  `budget_segment_id`。同一 run 的多个 segment 全部相加，旧段不会被覆盖。
- 全局 lifecycle lock 防并发训练；活动进程的 status 显示 live charge，不会被
  当作 crash 冻结。进程崩溃后下一次才 `ORPHAN_CHARGED`，按实际 elapsed 或保留上限
  保守计费，不清零。
- 预算包含验证、最后 checkpoint、report 和 logger close；日志在最后 checkpoint 后
  才 finish。SIGINT/SIGTERM 只设请求标志，当前 step 结束后保存 latest/final 并以
  非零退出。
- latest/best/final 均含 step、optimizer、scheduler、RNG、配置和 budget run/segment
  引用；best 只由 validation masked L1 选择。

```bash
$PROJECT_PYTHON tools/run/learning.py budget-status
# 恢复：同一 stable run 的新 segment，日志独立 events-<segment>
$PROJECT_PYTHON tools/run/learning.py train --mode full --data-root "$DATA_ROOT" \
  --resume-run "$OUTPUT_ROOT/<run-id>"
```

失败定位：

```bash
run="$OUTPUT_ROOT/<run-id>"
cat "$run/training_report.json"          # reason/errors/checkpoints/segment
tail -n 20 "$run/metrics.jsonl"          # step/phase/loss/L1/KL/lr/grad/VRAM
ls "$run"/events-*/                       # 每次 segment 独立 robot_run_logging
find "$ROOT/var/learning" -name '*.jsonl' -o -name 'training-budget.jsonl'
```

保存失败、report 写失败、logger close 失败都会进入 `training_report.json` 的
`errors`，并使命令非零退出；不会仍报 `completed`。

## 4. 按验证 best 导出 bundle

```bash
run="$OUTPUT_ROOT/<full-run-id>"
bundle="$ROOT/var/learning/bundles/<name>"
$PROJECT_PYTHON tools/run/learning.py bundle --mode full \
  --run "$run" --output "$bundle" --checkpoint best.pt --data-root "$DATA_ROOT"
$PROJECT_PYTHON tools/run/learning.py verify-bundle --bundle "$bundle" --device cpu
```

bundle 只含 safetensors 模型/processor，加载前后用同一实际 batch 复核
`predict_action_chunk`；每次返回新鲜完整 `(60,16)`，不裁剪、不使用
`select_action` 队列。真实任务输出不合约时记录并由监督者讨论。

## 5. 容器：本机已验证依赖 vs 跨机器重建

本机优先用 `act-dev`，把已验证 venv 只读挂入，不重复装 94 项依赖：

```bash
docker build --target act-dev \
  --build-arg ACT_SYSTEM_IMAGE=sim-dual-arm/learning:system-probe-20261101-v3 \
  -f deploy/docker/learning/Dockerfile -t sim-dual-arm/learning-act:dev .

docker run --rm --gpus all --network none \
  -e LEARNING_CACHE_ROOT=/output/cache \
  -v "$PWD":/workspace:ro \
  -v "$ROOT/third_party/src/lerobot-act-v0.6.1":/opt/lerobot:ro \
  -v "$ROOT/.build/cache":/opt/learning-cache:ro \
  -v "$ROOT/.build/learning-dev-venv":/opt/host-learning-venv:ro \
  -v "$DATA_ROOT":/data:ro \
  -v "$OUTPUT_ROOT":/output:rw \
  sim-dual-arm/learning-act:dev \
  python tools/run/learning.py --project-root /workspace \
    train --config /workspace/configs/learning/act_training.container.yaml
```

跨机器不得复制未知 ABI venv；用 `act` target 在容器 Python 3.12 下按
`deploy/docker/learning-requirements.lock` 重建：

```bash
docker build --target act --build-arg ACT_SYSTEM_IMAGE=ubuntu:24.04 \
  -f deploy/docker/learning/Dockerfile -t sim-dual-arm/learning-act:locked .
docker run --rm --gpus all --network none \
  -e LEARNING_CACHE_ROOT=/output/cache \
  -v "$PWD":/workspace:ro -v "$ROOT/third_party/src/lerobot-act-v0.6.1":/opt/lerobot:ro \
  -v "$DATA_ROOT":/data:ro -v "$OUTPUT_ROOT":/output:rw \
  sim-dual-arm/learning-act:locked \
  python tools/run/learning.py --project-root /workspace \
    convert --config /workspace/configs/learning/act_dataset.container.yaml
```

容器内 `configs/learning/act_*.container.yaml` 把 select/data/cache/output/budget/
checkpoint 明确绑定到 `/data` 与 `/output`；`/workspace`、`/opt/lerobot`、
`/opt/learning-cache`、`/opt/host-learning-venv` 和 `/data` 只读，只有 `/output`
可写。旧 `protocol`/`policy` target 语义未变；`act-system` 始终清除 probe 镜像可能
携带的 Python ENTRYPOINT。

## 验证与恢复绑定

首轮每1000步验证一次，最多512个batch，覆盖完整5条验证episode；需核对metrics的val_frames与validation frame_count相等。模式/根目录等CLI覆盖也进入checkpoint有效配置摘要，恢复和导出必须使用相同参数。不能只凭同一YAML文件hash把模型接到不同数据集；接受的数据manifest内容变化同样拒绝。使用train --mode full时，bundle也指定--mode full；更推荐为各模式保留独立不可变配置文件。

容器转换时，select中的绝对attempt_dir必须能在容器内访问。可生成保留原始select哈希的派生select，把路径映射到固定/source-data/attempts，并将CAMPAIGN_ROOT只读挂载到/source-data；后续训练校验源文件也使用同一挂载。不要修改原始select或manifest中的来源哈希。这样数据盘换宿主路径时，容器内来源路径仍稳定。
