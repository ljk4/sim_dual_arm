# Python 契约包

`pyproject.toml` 定义 `dual-arm-robot-contracts` 包，源码位于 [`src/robot_contracts/`](src/robot_contracts/README.md)。

- **依赖边界**：仅依赖 Python 标准库、`PyYAML`、`jsonschema` 与 `pydantic>=2.0`；严禁导入 ROS 2、MuJoCo、NumPy、SciPy 或 PyTorch，确保可在任意端轻量加载。

目录导航：[上级目录](../README.md)
