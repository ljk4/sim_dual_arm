# 架构提纯与 `var/` 产物目录规范化：总要求

仓库：`/home/ljk/work/sim_dual_arm`。最高工程准则见 [`RULES.md`](../../../RULES.md)，阶段进度见 [progress.md](progress.md)。

## 当前阶段（P06）目标

P00～P05 已完成架构对称化、Runtime 去耦合、废除三端 `prepare` 一步直跑与采集 attempt 内部拍平。
本阶段（[P06](phase-06-var-directory-standardization.md)）执行 [`RULES.md`](../../../RULES.md) 第二节的新增强制规范：**将所有运行期产物严格收敛到 `var/<scenario-id>/{datasets,models,logs,diagnostics}/` 四个目录，严禁混放，严禁在 `var/` 根下创建脱离任务目录的野生顶层目录**。

目录导航：[进度索引](progress.md) · [P06 任务书](phase-06-var-directory-standardization.md) · [文档索引](../../README.md)
