# 学习环境可行性预检

先核对本机路径和挂载；更换机器时修改这些变量。把数据放到另一块盘，需要把DATA_ROOT指向已挂载的数据盘，Docker镜像存储位置另行检查。

```bash
export ROOT="/home/ljk/work/sim_dual_arm"
export BUILD_ROOT="$ROOT/.build"
export DATA_ROOT="$ROOT/var"
export ASSETS_ROOT="$ROOT/third_party/src/mujoco_menagerie"
findmnt -T "$ROOT"
findmnt -T "$DATA_ROOT"
df -hT "$ROOT" "$DATA_ROOT"
```

2026-11-01，已完成隔离依赖、实际GPU和视频链路预检。**正式数据0，任务ACT尚未训练**。这里只证明候选环境能运行，未证明堆叠成功率或4小时训练效果。

## 环境边界

- 固定上游：[LeRobot v0.6.1源码](https://github.com/huggingface/lerobot/tree/7e241bd630a3719a56157a497ce5d08f244784f1)。未修改另一个项目的checkout；本项目只读挂载固定源码。
- Python3.12、Torch2.7.1/CUDA12.6、torchvision0.22.1、torchcodec0.4.0、NumPy2.2.6、PyAV15.1.0；94项依赖有锁定版本和分发哈希，uv检查无依赖冲突。
- ROS仿真使用NumPy1.26.4，学习进程单独运行；容器检查确认学习环境不能导入rclpy。
- 学习系统层为Ubuntu24.04/Python3/FFmpeg/libpython3.12/libgomp，依赖目录、源码、权重缓存只读挂载，数据/输出独立挂载。
- 预检曾发现python3链接和libpython3.12共享库缺失，已修复并实测；没有把宿主成功当容器成功。

当前是监督者原型，正式的依赖引导工具、Docker目标和部署配置尚待后续任务交付。跨机器应按锁文件在对应容器Python下重建依赖目录；不能直接假定其他宿主创建的虚拟环境可搬用。

## RTX4050实测

输入为三路320×240 RGB与16维状态，输出60×16；ACT使用ResNet18、hidden256、FF1024、encoder4、decoder1，关闭temporal ensemble。
官方ResNet18权重已校验SHA256：`f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec`。

| 项目 | 容器实测 |
| --- | --- |
| 参数量 | 18,728,464 |
| 合成反向检查 | batch8，10次更新，1.870s |
| 单步更新中位耗时 | 134.8ms |
| PyTorch峰值已分配/保留显存 | 1018/1082MiB |
| batch1完整动作块推理P95 | 12.3ms |
| 视频 | 真实H264编码/torchcodec解码，帧时刻0、0.25、0.95s核对通过 |
| 系统镜像ID | `sha256:5546a1acc44e5e6d0581903fe37bec311996429498bca8826008f2305c43ffac` |

上述输入均为合成数据，更新后得到的权重不用于任务执行。未测真实训练数据吞吐、长期热稳定性、渲染与模型同时推理的显存/延迟，训练配置还需按后续实测确定。

## 数据与后续门槛

正式转换保存独立的45条训练集和5条验证集；训练/推理处理器只采用训练集统计。固定版本按episodes选样时，不能直接假定全局meta.stats已经排除了验证episode。
动作标签仍为观测t对应的实际执行参考t+50ms，末尾padding必须有mask；这里尚未完成真实MCAP转换。
先完成S3运行时/策略接线，再试采、正式50条及转换验收，随后才启动任务训练；DSH-035 首轮短计划为 smoke 128 / overfit 500 / full 5000 步，任务训练累计墙钟上限仍为 4h，不为追高成功率自行耗满。

原始证据在`$BUILD_ROOT/reports/s6-preflight-20261101/`：源码/依赖摘要、锁文件、GPU/视频输出、失败证据与容器检查。
其中`container-check-4-pretrained-b8/`保存冻结输入哈希和镜像ID；`dataset-smoke-*`为合成数据接口检查，永不计入正式示教。


合成数据接口已通过：两个独立split各4帧、三路视频写入/解码、6×16动作块与末尾padding、官方ACT处理器保存。
训练集action均值约0.015，验证约1.015；采用训练统计时，零归一化动作正确反变换为0.015，没有混入验证统计。
对应证据为dataset-smoke-2/result.json。固定版本RGBEncoderConfig使用h264名称；初次libx264配置拒绝的记录保留在dataset-smoke-1。


DSH-035 的转换/训练/模型包命令与边界见 [learning-dataset-act.md](learning-dataset-act.md)。正式 50 条仍需监督者按 campaign select 运行，本文的 2026-11-01 预检结果不代表任务训练已经发生。
