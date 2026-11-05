# Learning 容器入口

`run.sh` 是进入空闲容器后手动执行的功能脚本；按 case 分发到真实
CLI，支持 `--help` 与参数透传：`prepare-policy`、`convert`、
`validate`、`filter`、`train`、`bundle`、`verify-bundle`、`budget`、`policy`、`cli`、
`test`。默认使用 `configs/act_dataset.container.yaml` /
`act_training.container.yaml` 的 `/data` 路径；用户传入的 `--config` 仍可覆盖。

`prepare_policy.py` 只做 bundle 校验与配置生成，不加载 Torch 模型；所有学习命令
统一经过 `run.sh`。脚本在宿主执行会明确失败；先在 `learning/lerobot/docker` 运行
`docker compose exec learning bash`，再执行 `learning/lerobot/scripts/run.sh <功能> ...`。
一次性命令可用 `docker compose exec learning learning/lerobot/scripts/run.sh <功能> ...`。

目录导航：[上级目录](../README.md)。
