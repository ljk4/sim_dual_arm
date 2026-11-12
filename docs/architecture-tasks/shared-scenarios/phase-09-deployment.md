# 阶段 09：部署要求归属与公共检查

状态：待实施。遵守 [总要求](README.md) 与仓库 RULES。

## 读取与目标

读取 progress 和 `deliveries/phase-08.md`。源码集中在 `shared/common/tools/scripts/check_dockerfiles.py`、三个实现的 Docker/Compose、`container.env` 与相关测试。

公共检查器负责共同部署规则；GPU、源挂载位置、所需网络和入口命令由所属实现声明。增加 CPU 仿真配置时，公共检查器不因其 simulation 角色强制要求 NVIDIA。

## 实现

- 在三个现有实现中各放一个小型 YAML 声明，只描述检查器需要、且不能由 Compose 本身确定用途的要求。沿用现有 YAML 读取方式，不增加 Schema 生成器或部署平台。
- 把现有实现专用判断移到声明驱动的检查中。声明紧邻实现，公共代码不维护后端名称到要求的映射；不复制整份 Compose、依赖锁或完整配置模型。
- 保留只读源码、源码与产物隔离、依赖镜像、空闲启动和 Docker socket 边界等共同规则。runtime 不要求 GPU，learning 保持 ROS-free，宿主并行编排继续归 runtime。
- 当前 MuJoCo 的 NVIDIA/EGL、learning 的 GPU、各端的网络和挂载要求保持实际配置。按现状写声明，不靠删除要求使检查通过。
- 复用现有检查命令与错误格式；声明缺失或实际 Compose 违背其要求时，指明具体文件与字段。

## 验证与交付

对三个实际部署运行一次修改后的静态检查。用一个小型 CPU 仿真测试配置证明检查器按声明判断，再验证当前 MuJoCo 缺 GPU 或必需挂载会失败。复用现有共同规则测试，不为每个声明字段扩写测试矩阵。

本阶段验证静态配置，不需启动业务、重建镜像或安装第二个后端。已有容器无需为声明文件重建。

更新 progress，交付 `deliveries/phase-09.md`，说明声明位置、关键字段与验证命令。完成后停止。

目录导航：[阶段索引](README.md)
