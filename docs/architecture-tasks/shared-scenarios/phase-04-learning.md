# 阶段 04：学习端与模型绑定接入

状态：待实施。本阶段遵守[总要求](README.md)，只完成本文件范围。

## 前置与读取范围

上一阶段 03 的必要验收通过；读取 `deliveries/phase-03.md` 的接口与限制。

必读总要求、本文件、已有 progress 和上阶段交接；源码按下面入口查找。除明确引用外，不要求阅读其他阶段全文。

## 目标与入口

让 learning 的 convert/train/bundle、prepare-policy/policy 使用方案绑定，继续由 learning 拥有数据与模型的完整语义。配方已经由阶段 03 移入方案目录，本阶段不再次复制。

相关入口：learning/lerobot/scripts/run.sh、prepare_policy.py、dual_arm_learning 的 cli/config/dataset/training/bundle/policy_config，以及其相关测试。

## 执行要求

1. 复用 binding API，解析 dataset、training、policy_service、档案和模型引用。选择的 recipe 必须实际进入既有端模型。
2. --scenario 与同类直接 --config/--template 互斥，保留现有直接配置模式；--bundle 显式覆盖可用，记录最终真实路径和摘要。
3. checkpoint 变动不新建方案或改机器人身份。模型目录不存在时推理准备失败，collection 校验仍可完成。
4. prepare-policy 继续校验真实 bundle、processor/profile/observation/action 身份，生成 policy_binding.json；不能由 scenario binding 代替真实性检查。
5. 数据源、45/5 划分、动作标签、归一化、视频参数、ACT 超参数、融合/夹爪准入与原始输出留痕保持基线。
6. learning 仍不导入 ROS 或 runtime/simulation 业务；Policy SDK 只取公共 Proto。
7. 修正 recipe 移动后的相对路径、项目根推导、source path 和默认配置；不假定 /opt/act-bundle 或宿主 /data 有真实模型。
8. 新输出目录防覆写；旧数据集、bundle、prepared 和预算台账不改动。--help 不加载模型。

## 验收

- 解析、参数冲突、缺模型、摘要不一致、非法配置测试覆盖实际入口。
- 对现有可用数据和模型做只读校验；需要转换的测试使用合成/临时输入及新输出目录，不重跑正式转换或完整训练。
- 纯准备结果能被 runtime 按既有通用 Policy binding 消费，新增 scenario 元数据不改变模型身份规则。
- ROS-free 导入与相关学习单元测试通过；完整闭环留阶段 11。
- 交接说明每个命令如何选方案及 bundle、实际配置路径、输出字段和仍需验证项。

## 交付与停止点

- 更新 `progress.md`，写 `deliveries/phase-04.md` 简短交接；基线清单只查询/补充相关行。
- 交接列出实际入口/接口、关键决定、命令与结果、未运行项及原因，状态必须真实。
- 完成本阶段后停止，不自动进入下一阶段。必要验证未完成时保留“待验证”，不得写通过。

目录导航：[总要求与阶段索引](README.md)
