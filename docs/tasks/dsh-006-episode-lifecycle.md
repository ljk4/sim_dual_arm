# DSH-006：S1b 提取 episode 生命周期

前置：DSH-005 已通过独立验收。目标是让人能在 `episode.py` 阅读一次运行的开始、脚本执行、清理与记录收尾；`node.py` 负责 ROS 注册和消息转发。保持现有对外行为，不提前实现新日志/记录格式、MuJoCo 或三相机。

## 范围和必要阅读

允许修改 runtime 包中的 `node.py`，新增 `episode.py`；仅按职责变化修改 `tests/integration/test_runtime_cleanup.py` 的故障接收 fixture、`runtime_shutdown_bootstrap.py` 的注入目标；新增 `tests/integration/test_episode_lifecycle.py`；在 `docs/stage2-quickstart.md` 更新模块/线程说明。其他测试断言、配置、协议、ControlClient、recording、observation、shutdown 不在修改范围。不提交 Git，不启动其他代理。

阅读 node/configuration/control_client/recording/shutdown，以及当前 cleanup/shutdown 测试和 bootstrap。主助手提供的快照已包含 S1a 类型配置与 S1b 控制客户端，不得恢复旧字典或旧控制状态。

## 所有权和接口

1. 新 `EpisodeRunner`（或同样直观的名称）拥有每次运行的 context、worker、停止/关闭请求、done、失败原因、episode ID、记录目录与 recorder，以及开始互斥/故障汇总状态。使用有类型字段，必要处加锁，不能让 node 再持有第二份同名可变状态。
2. node 仍拥有 ROS 对象、backend info/观测缓冲与同步条件、已收到的 terminal event 引用，负责消息转换及转发；ControlClient 仍独占 lease/续租。停止 Event 可以在组装时创建后同时注入两者，但只有一个事件实例，语义所有者是 episode。
3. episode 不导入 node，不访问 node 私有字段，不把整个 RuntimeNode 当万能状态容器。可声明一个小型、方法明确的 `EpisodeIO` Protocol，提供等待后端、绑定/重置观测上下文、等待新观测/终止事件、当前机器人时间、发送命令/状态、记录话题及诊断输出。只暴露所需操作和有类型值，不使用 Any 字段包、getattr/动态转发、mixin 或一大组状态 setter。
4. node 的现有公共 shutdown/worker/done/status 入口可作为显式委派保留，保证 main 和信号流程清楚；内部 episode 逻辑、失败汇总与 recorder 所有权真正移出。不为缩短行数拆出大量薄文件或嵌套包。
5. 传感器回调通过 episode 的明确 record 入口追加当前 recorder；状态/观测等待能看到同一个停止请求。保留固定 observation ID 行为，新的 episode 清空相关索引和旧 terminal 集合。

## 生命周期行为

- 开始：拒绝 shutdown 后的新任务与并发 episode；清理本次状态，建立 context 和 recorder，获取控制，进入 ACTIVE。
- 执行：按类型配置构建脚本目标，经 ControlClient 执行；保留候选命令、终止事件等待及错误语义。
- 结束：即使状态发布失败，也独立清理控制和结束记录；保留主故障及后续收尾原因；读取/报告最终 manifest。worker 最外层 finally 一定设置 done。
- 中断：信号处理器仍不做 I/O；executor 继续运行帮助完成控制清理；main 只在 worker 完成后销毁 ROS 对象。记录收尾中发生中断仍不得被报告为成功；强制超时仍退出124并保留partial，不承诺完整manifest。
- 人工 EndEpisode 的现有语义本任务不改变，后续 S1d/R8 单独处理。

线程说明应回答：哪些入口由 executor 调用、哪些由 episode worker 调用、哪个锁保护哪些状态、谁关闭 recorder。锁内不做阻塞 RPC、join 或磁盘收尾。不要把旧的巨大 node 原样改名复制到 episode。

## 验收

新增行为测试至少覆盖：正常单次运行完成、并发 start 被拒绝、shutdown 后 start 被拒绝、前一 episode 的结果/索引不泄漏到下一次、开始/状态失败仍收尾、worker 完成信号、记录收尾失败保留主故障。可用明确的 IO/control/recorder doubles，不能只断言内部字段定义。

原11项 cleanup、10项真实进程中断和MCAP检查的场景/断言意义保持；只迁移故障注入位置，不能降低断言或增加生产测试开关。主助手独立在 Jazzy 中运行全部测试、成功双容器链路，并审查模块实际依赖方向。

本机运行可用的纯 Python 回归与 Ruff；ROS 验收由监督者运行，不申请 Docker 权限。只报告实际结果和未执行项，保存输出到 `.build/reports/dsh-006/`。首轮上限20分钟，最多一次定点返工10分钟。
