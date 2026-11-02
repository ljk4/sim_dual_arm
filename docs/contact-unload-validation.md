# 接触卸载过渡验收

本机路径如下；更换数据盘先确认挂载，再修改DATA_ROOT，不能用空台账重置次数。

```bash
export ROOT=/home/ljk/work/sim_dual_arm
export DATA_ROOT="$ROOT/var/campaigns/panda_stack_v1"
findmnt -T "$DATA_ROOT"
df -hT "$DATA_ROOT"
```

2026-11-02，用户批准先比较、再使用有界接触卸载过渡。正式seed10001原运行仍为失败，保留原MCAP，不回写为成功。此前seed10000为合格正式数据。

原运行右夹爪闭合参考0m、被4cm方块挡住的实测开度约0.04m。放置后打开参考先经过0..0.04m；原控制把仍被物体约束的卸载过程累计为跟踪超差，随后TRACKING_TIMEOUT。机械臂各轴达标。新规则只影响跟踪资格，不改变参考、ACT/teacher原始输出、控制力、硬限位或最终实测张开要求。

状态机位于components/backend_sim/src/robot_backend_sim/contact_unload.py。先有符合原闭合规则的双指同物体接触才可激活；参考正在打开、仍比实测开度小一个位置容差，且当前双指仍接触同一对象、证据不歧义，才能短暂豁免该夹爪。2秒硬上限不能被暂停刷新；新一次真实闭合且接触合格才可重置。机械臂检查照常执行。

默认配置关闭。学习任务teacher/inference共用panda_task_stream.yaml，contact_unload_timeout_s=2.0，配置拒绝负值/布尔/非有限及大于2秒。旧panda_stream.yaml不变。NativeExecution新增明确CONTACT_UNLOADING质量值，MCAP记录参考/实测/指接触；后端日志记录进入/退出时对象、开度、速度和各指接触力，原生执行失败另记真实原因，避免仅显示后续anchor不可用。

相同失败前缀对比证据：.build/reports/contact-unload-comparison-v3-20261102/。未调用teacher/独立任务judge、未运行原任务完整结尾、未生成数据，不另计完整采集尝试。

- 原参考的真实物理重算：旧规则最长连续超差2.000秒，新规则0.760秒；新规则578个2ms tick处于卸载，时间17.204..18.358秒。两组相同参考、相同物理，最大机械臂位置误差均0.010500rad。
- 原始实测反馈独立重算：旧规则1.952秒（反馈采样截止，原500Hz控制已触发2秒），新规则0.700秒，27个反馈样本被标记卸载。
- 不把物理重算宣称为与历史运行逐位一致。首次探针读取返回值错误、第二次对旧行为的断言不符均保留；最后恢复原严格闭合速度判断后再完成对比。

纯状态/配置24项通过，更广真实ROS/物理/录制回归结果保存在.build/reports/contact-unload-review/related-tests/。后续正式数据仍须独立物理成功、录制完整和各phase图像/轨迹质量全部合格；不因规则获批自动接纳失败运行。
