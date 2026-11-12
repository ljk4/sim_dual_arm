# 阶段 01：公共目录、档案与容器路径迁移

状态：待实施。本阶段遵守[总要求](README.md)，只完成本文件范围。

## 前置与读取范围

上一阶段 00 的必要验收通过；读取 `deliveries/phase-00.md` 的接口与限制。

必读总要求、本文件、已有 progress 和上阶段交接；源码按下面入口查找。除明确引用外，不要求阅读其他阶段全文。

## 目标与范围

只进行公共代码、接口和档案的物理归并及必需路径修复。保持接口载荷、字段、Profile 内容、物理配置和运行行为。

## 迁移映射

| 旧来源 | 本阶段目标 |
| --- | --- |
| contracts/policy | shared/common/interfaces/policy |
| contracts/ros | shared/common/interfaces/ros，两个包暂时完整保留 |
| contracts/schemas | shared/common/interfaces/schemas，失效格式此时仅标记 |
| contracts/python | shared/common/python/contracts |
| libraries/trajectory、ros_support、run_logging | shared/common/python 下对应独立包 |
| infrastructure 的通用 scripts/docker 内容 | shared/common/tools，保留 SDK、setup、DDS、测试工具职责 |
| profiles | shared/scenarios/panda_stack/profiles |

Teacher 包拆分、Schema 去重和语义调整留各自阶段；方案 recipes 暂留四端，修正它们到新档案和接口的位置。

## 涉及入口

四端 run.sh、prepare 脚本及其 `parents[n]`/ROOT 推导；SDK 工具；Compose/container.env；根 pytest 路径；Panda worker/render/viewer/tools；offline audit/export、record/campaign；learning config/bundle 与测试路径。

Python 包保持现有 pyproject + src 结构和导入名；不建旧路径软链接、兼容转发包或重复副本。

## 容器规则

本轮采用三端统一的 `SHARED_ROOT=/shared`，Compose 将仓库 `shared/` 只读绑定到 `/shared`。这是容器内部路径约定，不是宿主目录假设。

- simulation 继续自身目录挂载 `/app:ro`；runtime/learning 可以保留 `/workspace:ro` 工作区挂载。
- 删除迁移后失效的 `/contracts`、`/profiles`、`/deps/*` 等旧公共挂载与引用，不为它们创建软链接。
- 各端 `PROJECT_SOURCE_PATHS` 显式指向所需公共 Python 包源码与本端源码。learning 不加载 ROS 或轨迹执行依赖。
- 共享 setup 脚本只设置调用方声明的环境，不按角色拼接具体业务路径，不启动程序。
- `var/`、`.build/`、SDK 输出和第三方源码的既有数据挂载继续有效。先确认真实路径，不创建或搬移宿主 `/data`。
- 修复移动后的脚本根目录推导；`shared` 根与业务项目根是不同概念，SDK 工具使用显式输入，不能依赖错误的 `parents[n]`。

## 执行与阶段边界

1. 按阶段 00 清单移动，更新全部活跃路径。业务源码仍在四端，第三方与产物不动。
2. SDK 构建/校验先适配新根目录，包集行为暂保持基线。旧产物失效时显式重建到受控目录，不手改 marker。
3. 本阶段可修正部署检查器对新挂载和路径的要求，保留原有实现规则；阶段 09 才做声明化。
4. 活跃导航随移动更新到可用入口；最终全仓文档收口留阶段 12。
5. 记录每个临时过渡项，不能留下隐式旧路径 fallback。发现 baseline 的失效引用可修正其目标路径，不顺便调业务参数。

## 验收

- 公共包可在需要它的对应环境导入，ROS-free 依赖方向保持。
- 现有接口内容和 Profile/observation/action 摘要与基线相同；路径变动的 recipe 除路径外有效值相同。
- Compose 静态解析、shell 语法、相关测试及 SDK 在新位置达到基线；缺环境如实记录。
- 旧活跃引用扫描覆盖 Python、shell、YAML/JSON、IDL/Proto、CMake/package.xml 与文档；不修改历史产物里的旧路径。

## 交付与停止点

- 更新 `progress.md`，写 `deliveries/phase-01.md` 简短交接；基线清单只查询/补充相关行。
- 交接列出实际入口/接口、关键决定、命令与结果、未运行项及原因，状态必须真实。
- 完成本阶段后停止，不自动进入下一阶段。必要验证未完成时保留“待验证”，不得写通过。

目录导航：[总要求与阶段索引](README.md)
