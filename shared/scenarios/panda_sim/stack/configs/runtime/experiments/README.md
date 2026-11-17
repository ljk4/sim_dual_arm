# 采集实验活动配置

`stack_campaign.yaml` 定义双 Panda 堆叠任务正式数据采集的预算上限（试点次数、正式尝试次数、磁盘容量上限）、合格目标数量（50 条）、随机种子范围及台账输出路径。`quality.phases` 显式声明堆叠阶段顺序，`export.max_labels` 给出单条导出的标签上限；通用导出工具不再内置任务阶段。

- 采集台账与锁文件输出至 `/data/<robot>/<task>/logs/collection/`，原始录制数据输出至 `/data/<robot>/<task>/datasets/raw/`（宿主机对应 `var/`），不写入配置目录。

目录导航：[方案说明](../../../README.md)
