# DSH-035 唯一集中返工

继续同一隔离工作树与会话；监督者保留首轮归档/冻结证据，主目录正式采集继续。PROJECT_PYTHON必须为/home/ljk/work/sim_dual_arm/.build/learning-dev-venv/bin/python；首轮准备header误把venv symlink resolve为系统Python，实际你已正确使用venv。本轮显式沿用该绝对venv、固定只读上游及纯测试覆盖层。预算30分钟，不Docker/联网/安装/完整任务/真实任务训练，不改主工作树、不提交。

首轮整体方向正确，但以下为阻塞验收问题，请集中修复。不要新增框架、重复实现模型或改现有Policy协议。

1. **恢复训练预算少计**。budget.begin允许同run_id再次写RESERVED/STARTED，_by_run覆盖前段。监督者无训练的真实反例：首段10s+同ID恢复20s，status仅20s而非30s（主目录.build/reports/dsh/dsh-035-budget-resume-counterexample.json，可只读）。将稳定training_run与每次调用的唯一budget_segment分开；拒绝重复segment。status不应把仍活跃进程当crash冻结计费。跨进程防并发耗额（可用明确训练生命周期锁），崩溃/重启保守计费，最终累计不能减掉旧段。恢复日志应有独立instance而非logger=None，checkpoint/report关联准确segment和stable run。
2. **验证后未恢复train**。_validation调用policy.eval，调用者从不policy.train，后续VAE/dropout训练行为改变。用finally恢复进入前模式；加入“训练→验证→训练”检查，确认后续VAE/KL/梯度仍参与，验证不使用标签作模型输入（只在计算masked L1时用真标签）。
3. **训练生命周期、日志与保存失败**。正常/预算/异常路径统一收尾；预算必须涵盖验证、最后checkpoint、报告与logger关闭，不在final save前ticket.finish。SIGTERM/SIGINT请求有界保存，记录中断原因并非零退出。不能吞掉checkpoint或logger.close失败后仍报completed；保持原始异常及保存/日志错误可查。检查点/恢复可拆为具名独立模块，避免training.py约950行中巨大try/except和重复保存；保留清晰易读的主训练循环与生命周期。合理保留latest/best/final，不新增复杂框架。
4. **容器入口真实可用**。Dockerfile的ACT_SYSTEM_IMAGE在首个FROM后声明，非全局ARG，不能用于后续FROM（监督者docker build --check已实际失败，stderr保留于主run）。移到正确作用域，保持旧protocol/policy语义。采用已验证probe作为base时要清除其Python ENTRYPOINT。增加一个明确的开发目标，可只读挂载本机已验证依赖目录，而不是每次重装94依赖；跨机器仍按锁在容器Python重建。数据RO、output RW、source/upstream/weightsRO。configure_local_caches当前无条件mkdir(data_root.parent/.hf-cache)，在只读数据盘会失败；训练/加载bundle/导出使用明确的可写cache位置（优先显式配置/环境并校验），不能写回接受的数据。
5. **来源事实检查不足**。_validate_export检查quality却不检查summary.task/runtime/recording。必须拒绝task失败、runtime失败、录制不完整、episode/seed身份不一致，即使quality=true和select写SELECTED；实际034summary字段是task:{outcome:succeeded,reason:TASK_SUCCEEDED},runtime:{outcome:stopped,reason:TASK_SUCCEEDED},recording:{complete:true,failure_reason:""}。将来源manifest/task事实摘要及hash加入dataset manifest，不能只信任人为拼的select。合成fixture也应明确构造对应事实，synthetic标记不能把失败事实冒充成功；不必引入ROS读取MCAP。监督者会独立对正式台账select核对前50与45/5。
6. **独立运维文档**。新增中文docs/learning-training.md，开头copyable ROOT/DATA_ROOT/OUTPUT_ROOT/venv/source变量及findmnt/df；提供真实公共convert→verify→smoke/overfit/full→按验证best bundle→verify命令、累计预算/恢复/日志/失败定位。Docker例子必须与数据RO/outputRW、配置绝对路径/可写缓存一致，含当前机器mount依赖目标和跨机器bootstrap两种真实入口；配置不写死宿主路径，少COPY多mount。

针对以上增加有意义反例：同stable run两段累计、活动status/崩溃/重复segment/并发锁；验证后继续训练；SIGTERM保存和非零、保存失败非零、恢复日志独立；readonly data+RW cache；有quality但task失败拒绝。合成CPU检查足够，不需GPU或大模型长训练。所有合成额度与数据标记不能计正式任务。报告更新真实通过/未测与公共命令，结束退出；不再第二次返工，剩余由监督者接管。
