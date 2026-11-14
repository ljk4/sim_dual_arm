# LeRobot ACT 学习与策略服务

提供无 ROS 依赖的 LeRobot 数据集转换、ACT 训练、模型包（Bundle）导出与 gRPC Policy 推理服务。

## 目录结构

- [`src/dual_arm_learning/`](src/dual_arm_learning/README.md)：`dual_arm_learning` 核心 Python 包。
- [`configs/`](configs/README.md)：容器环境配置 `container.env` 与测试替身配置。
- [`docker/`](docker/README.md)：`Dockerfile`、`docker-compose.yml` 与源码声明。
- [`scripts/`](scripts/README.md)：容器内单职责入口脚本（`convert.sh`、`train.sh`、`bundle.sh`、`policy_backend.sh`、`test.sh`）。

## 常用脚本

在 `learning` 容器内执行，所有脚本均要求二选一传入 `--scenario <id>` 或 `--config <recipe>`，并支持 `--set dot.path=value` 覆盖参数：

- `scripts/convert.sh`：将合格采集数据转换为训练/验证 LeRobot 数据集（输出到 `var/<scenario-id>/datasets/`）。
- `scripts/train.sh`：训练 ACT 模型（输出到 `var/<scenario-id>/models/`）。
- `scripts/bundle.sh`：导出并校验可部署模型包（Bundle）。
- `scripts/policy_backend.sh`：启动 gRPC Policy 推理服务（默认端口 `50051`）。
- `scripts/test.sh`：运行容器内 `pytest` 测试。

```bash
scripts/convert.sh --scenario panda_stack
scripts/train.sh --scenario panda_stack --mode full
scripts/bundle.sh --scenario panda_stack --mode full \
  --run /data/panda_stack/models/runs/<run-dir> \
  --output /data/panda_stack/models/bundles/stack-act
scripts/policy_backend.sh --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/stack-act
```

> `convert.sh` 和 `train.sh` 启动时会打印配置文件与有效参数，终端使用原地刷新的单行进度条，详细日志写入启动时提示的日志文件。
