# DSH-035 数据转换/训练/模型包工具验收

2026-11-02，31文件整合，17个变更Python与冻结一致；主HEAD/暂存区不变，未提交。此时正式34条合格、2次失败，任务训练仍0；合成CPU测试不计任务额度。

- 本地headless同会话session-6fb33385-8c7b-4061-a5c5-cbfb77c435e0：首轮1316.192秒/182步/201工具；唯一返工753.946秒/107步/107工具；均退出0。之后主助手补充有效配置/数据绑定、完整验证覆盖与测试环境隔离。
- 数据转换重核task/runtime/recording/quality与每帧时钟/图像/hash，两个独立45/5 LeRobotDataset，processor只用训练统计。训练/推理均使用固定LeRobot v0.6.1、三RGB+16状态→60×16动作。
- 同run恢复原10+20仅计20秒的反例已修复：稳定run/唯一预算segment、跨进程lifecycle锁、活跃/孤儿区分、累计4小时，保存/报告/日志关闭在计费内。
- 验证后恢复train，验证模型输入只有observation字段。SIGTERM/SIGINT请求保存并非零；保存/日志错误不吞掉，恢复段日志独立。检查点绑定有效配置与数据manifest，不能用CLI覆盖悄悄切数据。
- 首轮full计划仍5000步、batch8/float32，默认每1000步验证、512batch上限覆盖完整5条验证轨迹；实际训练时须核对val_frames。所有参数在YAML，原始预测/执行参考/测量数据分开。
- 独立学习容器首轮12通过；返工/主修批次20通过、1缓存测试受外部HF_HOME影响失败。修复测试隔离后3专项通过；另18通过/2项按Torch隔离跳过。真实ACT合成CPU公共train→resume→best bundle→verify链通过，非假权重；不把其loss算堆叠成功。
- 修复Docker ARG/继承ENTRYPOINT。docker build --check与act-dev实际构建通过；实际RTX4050/Torch2.7.1+cu126/NumPy2.2.6、无rclpy、公共CLI退出0。系统/依赖/source/upstream/数据/模型使用职责明确的挂载；跨机器按94项哈希锁重建。

证据var/runs/dsh/dsh-035/20261102T061103Z/：首轮/返工归档、source-accepted、primary-learning-checks（含真实CPU生命周期）、cache-final-checks、native-isolation-check、docker-primary-check、docker-act-dev-build、act-dev-gpu-entry、integration-review.json。操作入口docs/learning-training.md。

后续036实际ACT服务/固定20种子评估；监督者继续正式50→真实转换→smoke/overfit/full训练，不把组件验收当作整个任务完成。
