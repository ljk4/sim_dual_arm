# DSH-011：S1d 开发挂载与生成契约

前置：DSH-010已验收。宿主机已验证Docker Compose v5.6.1。此任务只建立可重复的开发构建/挂载入口；人工start/status/end客户端、collect监督与顶层status留下一任务，不把S1整体标为完成。

## 目标和边界

修改Python源码或YAML后，重启服务即可生效；只在依赖变化时重建依赖镜像。ROS契约从唯一源`contracts/ros`生成到宿主机`.build`，两个ROS服务共用它；契约变化后必须显式重建，不能继续加载旧接口。

保持runtime、mock backend、原始数据和日志各自边界；不改业务、记录/日志库或任何ROS/Policy协议。已有发布/测试Docker target保持可用。本任务完成后Compose开发路径可运行；旧自动演示入口暂保留，到下一任务再统一接到Compose，不维护两套长期产品入口。

## 允许修改

- 新增 `deploy/docker/dev/Dockerfile`（或runtime Dockerfile中清楚命名的dev target，两者择一），新增 `deploy/docker/entrypoints/dev_ros.sh`。
- 修改 `deploy/compose/mock.yaml`，只针对当前mock开发组合；不提前放MuJoCo/ACT占位服务。
- 新增 `tools/build/dev.sh`、`tools/build/dev_workspace.py`，新增一个薄的Compose转发入口 `tools/run/dev_compose.py`（可选薄shell包装，避免重复Python逻辑）。主机入口只用标准库，不依赖宿主机ROS或额外pip安装。
- 新增 `tests/unit/test_dev_workspace.py`、必要的独立命名CLI行为测试；文档限 `deploy/README.md`、`docs/stage2-quickstart.md`、`docs/logging-and-recording.md`、README开发入口说明。
- `.gitignore/.dockerignore`仅在确有新增构建产物规则时修改；`.build`与`var`已忽略，通常无需变动。

不要修改已有runtime/backend/contract业务；不提交Git、不运行Docker、不调用其他代理。首轮最多20分钟，一次定点返工最多10分钟。必要检查通过后停止，不为报告耗时小数位反复跑测试。报告 `.build/reports/dsh-011/report.md`。

## 开发镜像与挂载

1. 用现有 `sim-dual-arm/ros-base:stage2` 作为稳定依赖基础；开发镜像不COPY项目源码、profiles、configs、数据或模型，不通过pip安装应用副本遮住挂载。可COPY稳定依赖清单。现有release/test target不作为新开发入口的隐式代码来源。
2. Compose中的backend/runtime使用同一ROS开发依赖镜像，分别启动自己的模块。仓库只读挂载到`/workspace`；生成契约安装目录只读挂载到服务；构建服务才有对应build/install/log输出写权限。构建输出在`.build/ros-dev/`，MCAP在`var/data/raw/<run_id>/`，日志在`var/runs/<run_id>/`，都实际落到宿主机，不用匿名卷或最后docker cp导出。
3. Compose build profile提供一次性builder，按`contracts/ros`运行colcon，使用已有`tools/build/ros_workspace.py`的解释器策略。不重复构建Python runtime/mock包；它们从源码挂载导入。Policy SDK和learning镜像暂不改。
4. 运行前先source ROS和新生成install，再把明确的源码包路径放到PYTHONPATH最前：contracts/python/src、libraries/backend_core/src、libraries/ros_support/src、libraries/run_logging/src、runtime ROS Python包、mock Python包。防止source setup脚本后顺序又被旧安装包覆盖。入口最终exec，不让多层shell吞信号。
5. 容器按调用者UID/GID写挂载，构建/日志缓存使用明确可写目录，不依赖/root可写，不修改宿主机HOME。主机薄入口为Compose提供正确UID/GID，创建必要父目录；Compose文件的相对路径从仓库位置解析，不能依赖调用者当前目录。允许镜像名/构建输出位置显式覆盖以便隔离验收。
6. runtime默认保持服务模式，等待人工start；CPU-only、无Docker socket、无特权和GPU要求。网络与项目名可隔离多个开发实例；停止宽限期应大于应用worker和日志关闭总预算，不再沿用5秒外层强杀。

## 生成契约的可追溯性

用少量明确函数实现，不引入插件/通用构建框架：

- 从ROS契约源的相对路径及内容计算稳定摘要，记录生成环境/解释器和源码摘要。只在成功构建后原子写完成标记；失败或缺失产物不能留下可用标记。
- 构建与校验使用明确输出根，拒绝危险清理路径；不要递归删除用户任意目录。重复构建可使用自己的colcon缓存。
- 启动检查完成标记、当前契约摘要与必要生成模块存在；缺少/过期/损坏时给出清楚的重建命令并非零退出，不能回退到镜像旧SDK。输出路径/标记不能靠一次命令退出0就假设存在。
- 明确约定接口变更需先停服务再构建；构建入口防止两个builder同时写同一目录。先满足当前开发需求，不建立热更新机制。
- 提供简洁的检查输出，监督者可以看到实际runtime/contracts/logger/ROS生成模块导入路径，以及生成源摘要。未知版本写未知，不伪造与HEAD一致的结论。

## 薄主机入口与run目录

本轮只负责准备路径/环境并转发Compose参数，不把episode业务放进tools。可用如 `python3 tools/run/dev_compose.py --run-id <新ID> <Compose参数...>`；文档给出确切命令。run ID按安全路径段校验，不含分隔符或`..`；不要自动复用旧运行覆盖产物。

runtime当前会独占创建`<runs-output>/<run-id>`。主机入口只创建runs父目录和raw挂载目录，不能预先创建runtime的run目录；启动器自己的全局状态归下一任务，不在本轮改RunSession为任意复用。down/ps/logs等操作不应因为run目录已存在而失败；启动前复用冲突必须清楚报告。只操作所指定的Compose项目，不能down别人的项目或清理历史数据。

## 行为验收

1. 纯Python检查：摘要稳定、源修改后不匹配、失败构建不产生有效标记、并发builder拒绝；路径和run ID校验；转发参数用list而非shell拼接。测试行为，不只比较实现字符串。
2. 监督者实际构建依赖镜像和ROS契约，运行Compose配置检查；查看容器mounts确认源码/安装只读、输出挂载可写、用户正确、无socket/特权；打印实际导入来源。
3. 监督者用Compose启动mock和runtime，以公开服务采集或覆写命令执行一次--once；新v2记录、事件日志和resolved配置实际出现在宿主机；三段真实ACK和离线audit通过。
4. 在隔离候选工作树改一个合法YAML值与可观察的源码标记，保持镜像ID不变，重启后能看到新值/新标记；再还原。生成契约内容改变而不重建时启动失败，重建后恢复。不能用修改镜像内文件伪造挂载生效。
5. SIGTERM仍进入已验收的有界收尾；run复用/源码路径遮蔽/SDK过期都可读地报错，失败产物保留。所有例子只证明mock开发入口，不宣称物理场景/ACT已完成。

自测unit、Ruff和必要脚本语法；Docker、ROS和挂载实测由监督者执行。文档写清“改Python/YAML重启、改ROS契约先停后构建、改依赖重建镜像”的区别，分别列出源码、生成输出、数据和日志路径。
