# DSH-014 结果：仿真依赖、Panda 资产与 NVIDIA EGL

2026-10-30。已验收并整合 20 个文件，未提交 Git。[任务书](dsh-014-simulation-environment.md)。S2a 完成；这只是单 Panda 环境门槛，双臂/柱/桌/两块/三相机场景、正式数据和 ACT 尚未完成。

固定 15 个官方 wheel 版本及 SHA-256，基于 Python3.12/Jazzy/NumPy1.26.4 开发镜像新增 `mujoco-dev` target。应用、SDK、资产和配置使用只读挂载，报告可写；默认 mock target 保留。主助手实际构建了依赖镜像 `sim-dual-arm/backend-sim:mujoco-dev`，ID 为 `sha256:4422f876ae463391f520aacfa3966d8a9c4d9d15a40c7a571cc453133e81b10a`，`pip check` 通过。

主助手实际 sparse fetch 了 Menagerie 固定提交 `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`。Panda 清单 80 文件，Apache-2.0，内容摘要 `6a6099efd20d3340f8e73c41042df9fcb36b5d6921acdf6784eafb1af8857d43`；干净 checkout 可重复使用，摘要不写进 vendor 源码。

首版 329 项 unit 通过。独立原生检查中模型 step 与 DAQP 通过，EGL 因实际使用 Mesa llvmpipe 被正确拒绝；相同镜像显式 NVIDIA runtime 的对照定位为图形驱动注入配置。另实际复现按项目清理删除外来测试容器，以及截断前无界缓存输出。

唯一一次 dsh 定点返工补齐 NVIDIA runtime、随机工具容器身份、流式有界诊断、常规报告文件/容量保护和 Docker context 排除。复用既有 BoundedDocker，同时修正其异步异常退出时 kill/reap。本轮任务书原先漏列 `.dockerignore`，由监督者在返工中明确扩展允许范围。

主助手随后局部接管：

- 日志目录与事件使用同一个 instance_id；新增行为回归在修正前失败。
- 清理使用已核对的完整容器 ID，避免名称复用；外来容器保持不变。
- 已有资产 checkout 的脏检查覆盖全部目录和忽略文件，拒绝后保留人工文件及原摘要。初版接受根目录人工文件；忽略文件已被后续扫描拒绝，但错误阶段不一致。两项回归明确固定为 fetch 前拒绝。

最终验收：

- **338 项 unit、Ruff、shell 语法和 diff 检查通过**；整合后主目录复验通过，主暂存区保持不变。
- 真实单 Panda model：nq/nv=9、nu=8，按上游 keyframe 步进 20 次，状态有限。DAQP 解为约 `[1, 2]`，最大误差 `2.22e-16`。
- 实际 NVIDIA EGL：vendor=`NVIDIA Corporation`，renderer=`NVIDIA GeForce RTX 4050 Laptop GPU/PCIe/SSE2`，OpenGL4.6/driver580.178.04。320×240 RGB PNG 已人工查看，Panda 清晰可见；日志排空和报告/退出码匹配。
- 实际 inspect：源码/SDK/资产/摘要只读，报告 RW；调用者 UID/GID、network none、非 privileged、GPU/runtime 均正确。原生日志 close 和 Renderer close 故障均非零，不发布成功。
- 实际 timeout/SIGINT/SIGTERM 返回124/130/143，工具容器全部回收。外来同项目测试容器保留。每流产生2MiB时 Python 分配峰值约1.47MiB（首版约10MiB），捕获上限每流256KiB。
- 共享诊断helper修改后，旧mock collect与独立原生audit真实回归通过。
- 主目录按默认路径运行 `python3 tools/run/sim_check.py --run-id integrated-014 --timeout 120` 成功；修改应用源码无需重建依赖镜像。

证据 `var/runs/dsh/dsh-014/20261030T121710Z/`，重点为 `acceptance/native-final`、`mounts-and-faults-final`、`boundaries-final`、`interrupts-final`、`assets-reuse-final`、`units-accepted`、`main-native`；首版、返工前接管源码和失败证据保留。原始日志不写进 Git。

dsh 两轮合计1251.047s、159个已回报steps、182次工具调用；输入124924、输出199829、cache-read30346752、total30671505 tokens。主助手用量/费用未知，无节省比例结论。

下一任务搭建可配置、可确定性复位的双 Panda 场景模型；之后独立检查三视图、目标可见性和可达性，通过后才进入 ROS 物理控制。
