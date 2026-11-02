# DSH-036 ACT服务与评估工具验收

2026-11-02，24文件整合，主HEAD/暂存区不变。以下首先记录组件验收时的状态；真实部署结果另见末尾补充。

- 同会话session-78d715de-0fb2-4be5-a584-ad5bf0db7c26：首轮1749.047秒/167步/191工具；唯一返工575.680秒/61步/69工具，均退出0。
- 既有Policy协议复用，真实bundle派生身份，完整60×16新鲜预测、严格范围拒绝、暖启动/结构化日志/关闭、独立ACT三服务Compose及原公共生命周期。
- 首轮独立44项通过；反例发现fake/no-digest/预测0也被算闭环，已修复。返工独立53项通过；主助手修正实际STARTED与raw预测/标签采样时间区分及日志clock_epoch，受影响19项与2项时间复验通过。旧断言失败保留。
- Compose静态检查通过：仅backend/runtime/act-policy；来源/模型/SDK/依赖RO，日志/cache RW，无fake。未启动完整任务，未消耗pilot/test。
- 物理task_success、可信ACT执行/闭环及runtime/recording/quality/run_success分别报告。固定20分母保留缺失/拒绝/超时，不伪造成功。
- 文档统一主ROOT与原持久campaign，不能新开工作树空台账重置次数。

证据var/runs/dsh/dsh-036/20261102T070414Z/，最终source-complete与integration-review；原型/失败/返工/专项报告均保留。用户决定保持严格拒绝，等待full训练后复核输出，不添加投影。

## 真实部署补充

full训练后用户另行批准仅夹爪准入，主助手完成具名模块和原始预测审计；修正真实bundle中训练YAML与架构JSON哈希混用，以及公共入口路径/runtime SDK挂载遗漏。两次启动失败留在pilot台账，未回写。

最终固定20种子均有可信ACT执行及反馈闭环，物理堆叠0/20，后续均跟随偏差超限受控失败。完整录制20/20，图像质量15/20，所属资源清理完成。后续控制失败不抹去此前真实执行，也不算完整运行通过；评估语义8项回归通过。最终结论与产物见[本轮结果](../simulation-loop-result.md)。
