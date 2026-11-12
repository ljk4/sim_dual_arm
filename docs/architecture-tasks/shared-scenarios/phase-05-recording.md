# 阶段 05：采集、campaign 与并行 worker 接入

状态：待实施。本阶段遵守[总要求](README.md)，只完成本文件范围。

## 前置与读取范围

上一阶段 04 的必要验收通过；读取 `deliveries/phase-04.md` 的接口与限制。

必读总要求、本文件、已有 progress 和上阶段交接；源码按下面入口查找。除明确引用外，不要求阅读其他阶段全文。

## 目标与入口

让单路录制、campaign、宿主并行采集和 worker 使用同一个显式方案。普通启动能跑不代表这些路径已经迁移。

入口以当前源码为准：runtime/ros2/scripts/record.sh、record_parallel.sh，dual_arm_runtime/offline 的 record、record_parallel、record_pool、worker/campaign 相关模块，以及它们调用的 prepare/export/audit。

## 执行要求

1. 由 binding 选择 runtime_teacher、simulation_teacher、campaign 和相关档案；清除 teacher_collect/backend_stack 等旧目录默认值。
2. campaign.required_paths、SDK marker 和导出引用取真实目标，修正旧 GetTeacherChunk 路径漂移。本阶段包仍为 sim_api_msgs，阶段 06 再更新专用包路径。
3. --scenario 与同类配置选择参数冲突时报错；count/target-total、purpose、种子预算和续采规则保持原语义。
4. 先完成参数/路径/只读预检，再预留预算或分配种子。预检失败不能留下新的正式记录或启动后端。
5. 每个 worker 显式收到同一绑定身份和自己的 resolved recipe、ROS domain、namespace、run-id、输出根；不能靠公共全局默认值。
6. worker 容器内 setup/source paths 与宿主命令中的 /workspace/infrastructure 等旧路径一并更新。
7. 宿主并行入口继续在 runtime 内维护；普通 runtime 不获得 Docker socket，公共工具不承担采集生命周期。
8. 超时/中断只清理本轮 worker、进程组和容器；停止、租约释放、刷盘和导出失败仍独立报告，不删失败台账。
9. 正式台账和数据保持不动。回归只用独立诊断目录、台账和种子计划，不重置或挪用现有预算。

## 验收

- 单路与并行 CLI/配置/预算/清理相关原有测试通过；补充实际方案传递、不同 worker 隔离及预检不预留的测试。
- 使用替代测试 binding 能追踪到真实 worker 准备命令和输出，不只是顶层参数存在。
- 不在本阶段补采正式数据或重复评价；实际两个 worker 与中断验证留阶段 11。
- 交接提供新入口、worker 参数契约、诊断运行方式、预算与资源隔离信息。

## 交付与停止点

- 更新 `progress.md`，写 `deliveries/phase-05.md` 简短交接；基线清单只查询/补充相关行。
- 交接列出实际入口/接口、关键决定、命令与结果、未运行项及原因，状态必须真实。
- 完成本阶段后停止，不自动进入下一阶段。必要验证未完成时保留“待验证”，不得写通过。

目录导航：[总要求与阶段索引](README.md)
