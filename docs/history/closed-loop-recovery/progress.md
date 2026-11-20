# 闭环成功率回退修复与工程规范收敛进度追踪

任务书：[taskbook.md](taskbook.md)  
开发分支：`fix/closed-loop-and-ownership`（已提交 `73e67c1`） → 规范分支：`norm/closed-loop-and-ownership`

| 阶段 | 任务内容 | 状态 | 改动文件与验证记录 |
| :--- | :--- | :--- | :--- |
| **Step 0** | 新建开发分支，建立任务书与进度追踪文件，登记文档索引 | 已完成 | [任务书](taskbook.md)、本进度记录、[文档索引](../../README.md) |
| **P01** | 在 `AGENTS.md` 新增改动/新增代码归属判断与破例确认准则 | 已完成 | `AGENTS.md`（新增 §2.4） |
| **P02** | 多轴协同过渡重定时（`RobotProfile` 轴组、公共层等比例缩放、清除场景魔数、补充单测） | 已完成 | `models.py`、`transition_retiming.py`、`chunk_stream.py`、`test_transition_retiming.py` |
| **P03** | 修复 Panda Teacher 时钟对齐、拆分采集/推理 Follower 配置并修正静止帧过滤 | 已完成 | `GetTeacherChunk.srv`、`panda_sim/stack/{__init__,teacher,gate}.py`、`panda_follower_{v1,inference_v1}.yaml`、`sample_filter.py`、`act_sample_filter.yaml` 及对应单测 |
| **P04** | 启动预热增强（服务端 `OpenSession` + 3.0s settle 真实观测预热）与异步预取推理 | 已完成 | `pipeline.py`、`policy_service.py`、`runtime_config.py`、`chunk_stream.py`、`policy_{rollout,eval}.yaml` 及单测 |
| **P05** | 细化 Marvin 空中交接移交边界与启用训练侧视觉骨干分组学习率 | 已完成 | `marvin_sim/handover/evaluator.py`、`config.py`、`training_support.py`、`training_engine.py`、`act_training.yaml` 及对应单测 |
| **P06** | 代码归属清理、容器单元测试回归验证与文档同步 | 已完成 | `hardware/README.md`、`README.md` §5、`docs/experiments/closed-loop-triage.md`、`panda_sim/kinematics_config.py`；清理孤立 `.pyc`；ROS SDK 重建通过；容器验证：`runtime` 45/45、`simulation` 98/98 + 84/84、`learning` 24/24（+1 skipped，含 `DSH035_HEAVY=1` 1/1 通过） |
| **P07** | 修复 Marvin 训练 `episode_id` 校验、Panda 推理视觉骨干收敛与并行采集渲染快照缓冲（Issues 5, 7, 8） | 已完成 | `sample_filter.py`、`act_training.yaml`、`training_engine.py`、`backend_*_{teacher,policy,act_chunked}.yaml`、`fake_policy.yaml`、`test_learning_config.py`、`test_learning_sample_filter.py` |
| **P08** | 后端启动就绪输出反馈与并行采集/推理 ETA 剩余时间显示（Issues 1, 2） | 已完成 | `backend_cli.py`、`record_pool.py`、`inference_worker.py`、`test_record_parallel_host.py`；`teacher_backend.sh` 与 `inference_backend.sh` 连续两次就绪/停止冒烟实测通过 |
| **P09** | 统一全链路日志与 `run-id` 结尾为可读时间戳 `_YYYYMMDD_HHMMSS` 并支持重复运行（Issues 3, 4） | 已完成 | `cli.py`、`record.py`、`record_parallel.py`、`record_pool.py`、`inference_parallel.py`、`backend_cli.py`、`{teacher,inference}_backend.sh`、`client_{interactive,policy}.sh`、`server_policy.sh`、`artifact_paths.py`、`test_record.py` |
| **P10** | 更新 `README.md`（§4 补充 Marvin 可视化、移除 §5）与容器单测回归验收（Issue 6） | 已完成 | `README.md`；容器实测：`simulation` 324/324、`runtime` 208/208、`learning` 94/94（+1 skipped `DSH035_HEAVY=1`） |
| **P11** | 闭环根因修复与 4 项特性设计落地（真异步推理 + 时序不变静止推进、UMI 执行延迟匹配 `action_execution_latency_ns`、精准 `WAITING_FOR_FEEDBACK` 静止帧过滤、视觉平移增强 `image_shift_px`、数据集转换前置拒覆盖、Marvin 评估器连续左持刷新） | 已完成 | `docs/features/{README,异步流式推理与静止推进,精准静止锚点过滤,视觉平移鲁棒性增强,多爪负载转移通用守卫}.md`、`chunk_stream.py`、`runtime_config.py`、`policy_{eval,rollout}.yaml`、`sample_filter{,_rules}.py`、`act_sample_filter.yaml`、`config.py`、`training_{support,engine}.py`、`act_training.yaml`、`dataset.py`、`evaluator.py` 及单测；容器单测：`learning` 83/83、`runtime` 37/37（+2 skipped）+ `test_policy_stream` 20/20、`simulation` 46/46；Marvin 全量重训 `act_20261119_033841`（`best_val_l1=0.02352`）实机闭环 `seed=4157/20000/0` **3/3 `TASK_SUCCEEDED`**；Panda 全量重训 `act_20261119_044747`（验证集活动臂真实 MAE `0.0146 rad`），实机闭环在线横向漂移问题按用户指示暂缓、留待后续专项排查 |
| **P12** | 规范阶段收敛（清除死代码与 `legacy_*` 兼容层、仿真通用层插件化去硬编码、三端容器声明与单测便携性对齐、特性文档与现状全量同步） | 已完成 | 删除死模块 `offline/collect_artifacts.py`（448 行）；移除 `legacy_digest` / `GRIPPER_JOINT_NAMES` / `gripper_axes` / `--bundle` 等无消费符号；新增 `common/plugins.py` 与 `<robot-id>/plugin.py` 约定发现，`common/` 及三端通用代码实现零机器人/任务硬编码；三端 `deployment.yaml` 补齐 `/etc/bash.bashrc` 只读挂载声明；同步 3 篇并新增 `多轴协同过渡重定时.md`、`训练与推理全链路可观测性.md`（全中文命名）；容器全量单测 `simulation` 1,165、`runtime` 1,012、`learning` 113，终版修改后聚焦回归 `simulation` 131/131（+2 skipped）、`runtime` 142/142（+2 skipped）、`learning` 83/83 全通过；规范化后 Marvin `seed=0` 最小闭环冒烟实测 `TASK_SUCCEEDED`（退出码 0） |

目录导航：[任务书](taskbook.md) · [历史索引](../README.md)
