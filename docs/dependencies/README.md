# 依赖分析材料

当前设计见 [Docker 环境设计与依赖分析](../docker-dependency-analysis.md)。

这里的 `lerobot-py310.candidate.in` 和 `lerobot-py310.resolved.txt` 是此前 Humble / Python 3.10 方案的历史解析记录，保留用于追溯兼容问题，不作为当前方案的安装清单。

当前方案包含 backend-real、backend-sim、runtime、learning 四类镜像；真实/仿真后端二选一，模型镜像复用于训练与推理服务。职责与接口见 [接口设计](../architecture-and-interfaces.md)。精确版本锁在实现最小闭环时生成并验证。
