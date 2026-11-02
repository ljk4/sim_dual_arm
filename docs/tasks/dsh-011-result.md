# DSH-011 结果：Compose 开发挂载与生成契约

2026-10-30。结论：已验收并整合 11 个文件，未提交 Git。[任务书](dsh-011-compose-development.md)。本任务完成 S1d 的开发环境部分；人工命令与整次运行监督尚未实现。

runtime 和 mock 后端共用稳定的 ROS 开发依赖镜像。源码、配置和生成契约只读挂载，原始数据与运行日志直接写入宿主机，容器按调用者 UID/GID 运行。Python/YAML 修改后用新 run id 重启即可生效；ROS 契约变化需先停服务再重建；依赖变化才重建镜像。

契约从 `contracts/ros` 单一来源生成，启动检查源摘要、生成标记和实际导入来源。构建输出通过独立归属标记与互斥锁保护；已有非空且未归属的目录会被拒绝，不清理其中的人工文件。运行入口拒绝复用已有 run id，停止和查看日志不受这项限制。

首轮 202 项 unit 自测通过，但监督者实际容器检查发现 ROS setup 在 nounset 下失败，独立临时夹具另发现未归属输出目录的文件被清理。一次 dsh 定点返工修复这两项，同时移除 HOME 重设、把开发入口改为从挂载读取、将入口诊断写到 stderr。主助手只修正文档：`--once` 示例保留 backend 依赖启动并补充清理命令，当前部署说明改用已选的 MuJoCo 路线。

最终验收：

- **207 项 unit 通过**，其中新增 40 项；Ruff lint/format 和 shell 语法通过。合入主工作区后再次确认 207 项通过，Ruff 通过。
- Docker Compose v5.6.1 下实际构建开发镜像和 ROS 契约，核对生成模块及应用源码导入路径、挂载权限和容器 UID/GID。
- 通过公开服务执行两次隔离采集；三段命令的真实 ACK、v2 manifest、MCAP 离线审计、配置快照和日志均通过。
- 修改候选 Python 标记与合法 YAML 值后，以相同镜像重新启动，确认两项修改均生效；修改契约但不重建时启动失败，重建后恢复。探针修改已逐字节还原。
- 重复 run id 被拒绝；SIGTERM 后 runtime 退出码为 143，日志完成排空；文档的 `--once` 示例另行实跑成功，退出码为 0。
- 未归属输出目录的独立复验通过，人工文件保留。验收 Compose 项目已清理。

证据目录：`var/runs/dsh/dsh-011/20261030T072111Z/`，其中 `acceptance/dev-stack-final`、`acceptance/once-doc`、`acceptance/unowned-final` 为最终实测，`accepted.patch` 为整合差异。输入快照为 `26face851f1ee2eb8403a1d3c4703d110a017e80`；验收镜像 `sim-dual-arm/dev-ros:dsh-011` 的 ID 为 `sha256:3a3dbdaa7c95752a981e4ea2651c3987b42b6a178bb966eba456321ffdc486e3`。验收使用隔离构建输出；默认 `.build/ros-dev` 和 `sim-dual-arm/dev-ros:stage2` 仍应按使用指南首次构建，不能把验收目录当作默认安装。

dsh 两轮共 869.471 s、119 steps、161 次工具调用；provider 输入 78,852、输出 129,070、cache-read 14,594,816、total 14,802,738 tokens。缓存读量不等于新增生成量或费用；主助手用量/费用未知，不能据此宣称节省比例。主暂存区保持不变。

按用户要求，在本任务完成后暂停。尚未分派下一任务：人工 start/status/end 命令、自动采集入口统一到 Compose，以及结合进程退出码、日志排空和记录审计的整次运行状态汇总。S1 尚未整体完成；S2～S7 未开始，尚无物理堆叠、正式 50 条数据或 ACT 训练/推理结果。
