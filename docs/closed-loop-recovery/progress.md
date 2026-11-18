# 闭环成功率回退修复与工程规范收敛进度追踪

任务书：[taskbook.md](taskbook.md)  
开发分支：`fix/closed-loop-and-ownership`

| 阶段 | 任务内容 | 状态 | 改动文件与验证记录 |
| :--- | :--- | :--- | :--- |
| **Step 0** | 新建开发分支，建立任务书与进度追踪文件，登记文档索引 | 已完成 | `docs/closed-loop-recovery/taskbook.md`、`docs/closed-loop-recovery/progress.md`、`docs/README.md` |
| **P01** | 在 `AGENTS.md` 新增改动/新增代码归属判断与破例确认准则 | 已完成 | `AGENTS.md`（新增 §2.4） |
| **P02** | 多轴协同过渡重定时（`RobotProfile` 轴组、公共层等比例缩放、清除场景魔数、补充单测） | 已完成 | `models.py`、`transition_retiming.py`、`chunk_stream.py`、`test_transition_retiming.py` |
| **P03** | 修复 Panda Teacher 时钟对齐、拆分采集/推理 Follower 配置并修正静止帧过滤 | 已完成 | `GetTeacherChunk.srv`、`panda_sim/stack/{__init__,teacher,gate}.py`、`panda_follower_{v1,inference_v1}.yaml`、`sample_filter.py`、`act_sample_filter.yaml` 及对应单测 |
| **P04** | 启动预热增强（服务端 `OpenSession` + 3.0s settle 真实观测预热）与异步预取推理 | 已完成 | `pipeline.py`、`policy_service.py`、`runtime_config.py`、`chunk_stream.py`、`policy_{rollout,eval}.yaml` 及单测 |
| **P05** | 细化 Marvin 空中交接移交边界与启用训练侧视觉骨干分组学习率 | 已完成 | `marvin_sim/handover/evaluator.py`、`config.py`、`training_support.py`、`training_engine.py`、`act_training.yaml` 及对应单测 |
| **P06** | 代码归属清理、容器单元测试回归验证与文档同步 | 已完成 | `hardware/README.md`、`README.md` §5、`docs/experiments/closed-loop-triage.md`、`panda_sim/kinematics_config.py`；清理孤立 `.pyc`；ROS SDK 重建通过；容器验证：`runtime` 45/45、`simulation` 98/98 + 84/84、`learning` 24/24（+1 skipped，含 `DSH035_HEAVY=1` 1/1 通过） |

目录导航：[任务书](taskbook.md) · [文档索引](../README.md)
