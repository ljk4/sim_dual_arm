# robot_run_logging

- `events.py`：事件 schema、级别、关联 ID、严格载荷和健康状态。
- `writer.py`：有界队列、JSONL 文件轮转、排空和关闭。

关键事件溢出或写失败必须持续报错；关闭有界，不能用日志成功掩盖业务失败。

目录导航：[上级目录](../README.md)。
