# DSH-020：Panda原生执行器与测量映射已验收

2026-11-01。对应[任务书](dsh-020-panda-io.md)。11个允许文件已整合，无Git提交，主暂存区未变。

## 完成内容与阅读入口

- `backend_sim/panda_io.py` 提供 `PandaIO(scene, profile)`、不可变 `Measurement`、`measure()` 和 `apply_targets()`。按具名原生joint/actuator解析16维SI，不依赖qpos/qvel地址相同或元素相邻；两指位置/速度求和得到夹爪总开度。
- 目标完整验证后才写ctrl，夹爪0.04m连续映射为127.5；写入不改qpos/qvel/时间/物体/外力，也不步进模型。测量来自真实MjData，可与目标存在跟踪误差。
- 校验profile范围/home/单位、实际传动与单位gear、三相机声明及编译模型的父体/固定模式/分辨率/fovy/位姿。错误输入、模型、标定和线程使用均明确拒绝。
- CameraConfig保存不可变原始声明JSON；公开读取函数返回新副本，并通过同一解析器核对声明归一化后与有效配置一致。保持既定profile摘要、有效四元数和物理配置，绑定不依赖原始文件继续存在。

先看[API说明](../panda-io.md)，再读 `PandaIO` 的四个公开操作及构造检查；场景解析的 `camera_declared_description` 解释原始标定声明与有效姿态的区别。

## 实施与独立修正

本地headless session `session-009f4d36-ec31-4a5a-8ec6-92c30f8390bd`，输入快照 `04f715b0020e0b19c2db02d5e63e57b9cd9cf405`。12:42:32首轮启动，515.20s、exit0；12:52:48唯一返工启动，189.75s、exit0。

中间快照发现摘要归一化差异和7/8项strict zip错误；**dsh在正式首轮交付前已自行修正**，正式首轮16项原生测试通过，不能将中间问题算作首轮交付失败。监督者随后独立复现：删除配置源文件后绑定失效，以及创建线程退出后新Thread复用ID被错误允许。返工改为保留原始声明并使用Thread对象身份，还收紧了类型和ctrl写入代码。

返工后56项原生/配置测试通过；独立模型反例继续发现编译相机fovy或actuator gear改变仍被接受，主助手补实际模型校验及8项原生回归。未改上游增益、重力、接触参数或profile实例。

## 验收证据

| 检查 | 结果 |
| --- | --- |
| 单测 / 静态规范 | 499项unit通过；Ruff lint/format通过，91个Python文件 |
| 固定MuJoCo3.17原生回归 | 64项通过，无skip；含19项PandaIO、7项既有Scene及配置用例，部分与unit重叠 |
| 独立原生检查 | 10组通过；实际qpos/dof、不可变测量、ctrl映射、9个非法目标、7个错误profile、错误传动、线程复用、源文件脱离、相机与gear反例 |
| 真实动态对照 | 与独立按原生名字写ctrl的场景逐步重放，qpos/qvel逐元素完全相同；无MuJoCo warning |
| 空载响应 | 小幅右臂及夹爪运行2s，最大臂误差约0.00658rad；右夹爪误差约4.6e-11m。保留实际静差，不用目标冒充测量 |
| 配置兼容 | 新旧归一化Scene诊断文档逐字节一致；仅新增原始声明的不可变元数据 |
| 公开旧scene检查入口 | exit0、67布局、201个三视图PNG，日志/渲染器正常关闭，资源清理通过 |
| 整合一致性 | 主工作区51个业务Python文件与冻结最终源码逐文件一致；ROS契约未改，SDK无需重建 |

证据根 `var/runs/dsh/dsh-020/20261101T044224Z/`。区分 `source-interim`、`source-initial`、`source-rework`、`source-final`；`native-normalized-diagnostic`只是内存替换摘要以隔离中间错误的诊断，不计生产profile通过。正式最终证据在 `acceptance/native-oracle-final`、`native-suite-final`、`public-scene-final`，并保留源码归档/accepted.patch/配置与整合哈希检查。

两轮dsh累计704.95s、82 steps、104 tool calls；输入92,532、输出107,751、cache-read9,606,272、total9,806,555 tokens。实际费用与主助手用量未知，不推算节省比例。

## 剩余边界

这只完成原生执行器/测量层的空载验证。成功写ctrl不等于运动完成；插值、速度/加速度调度、跟踪/停止判定、lease/reset/epoch、ROS适配与同快照观测尚待后续。正式数据仍为0，ACT未开始；S3整体与抓取堆叠未验收。
