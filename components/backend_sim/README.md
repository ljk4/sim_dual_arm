# 仿真后端

MuJoCo 双 Panda 物理后端。`src/robot_backend_sim/` 是唯一 Python 包，ROS 入口为
`backend_cli.py` / `node.py`；场景与控制配置在 `configs/simulation/`，运行入口为
`tools/run/native_backend.py`，组合见 [deploy/compose](../../deploy/compose/README.md)。

物理对象由 worker 独占，渲染、规划、反馈和任务判定不跨线程直接改状态。历史实现见
[DSH-027](../../docs/history/dsh-027.md)、[DSH-030](../../docs/history/dsh-030.md)、
[DSH-034](../../docs/history/dsh-034.md)。

目录导航：[上级目录](../README.md)。
