# P02B 交付：接触持物、连续交接路径与物理执行

日期：2026-11-12。**P02B 已完成，停在 P03 之前**。工作树、历史成功/失败证据保留；未暂存、提交或推送。P00/P01/P01A/P02A 交付记录未覆盖。

## 验收结果

全部动作通过标准 Robot API / SubmitFollower 执行。物块在执行中仅由真实接触运动改变；每个独立诊断仅在开始时建立新场景。

| 诊断 | 最终证据 | 实测结果 |
| --- | --- | --- |
| 左手独立持物 | [80761320](../../../var/reports/p02b/p02b-80761320/evidence.json) | 左端抓持，抬升 149.509 mm，持续双指接触且离桌，原稳态门槛通过 |
| 右手独立持物 | [9a6b46b1](../../../var/reports/p02b/p02b-9a6b46b1/evidence.json) | 抬升 149.334 mm，双指接触且离桌，原稳态门槛通过 |
| 双手共持全过程 | [c1880e90](../../../var/reports/p02b/p02b-c1880e90/evidence.json) | 抬升 79.813 mm；相对桌面抓持基准，左右最大位置变化 0.053 / 0.144 mm，姿态变化 0.000322 / 0.000513 rad；释放 COMPLETED |
| 带物停止 | [e5549b60](../../../var/reports/p02b/p02b-e5549b60/evidence.json) | CANCELED / P02B_HOLD_STOP，停止后双手持物、离桌；开度变化左 −0.000030 / 右 +0.000153 mm；后续释放 COMPLETED |
| 连续空中接管至入箱 | [38bce3cc](../../../var/reports/p02b/p02b-38bce3cc/evidence.json) | 72 个有限窗口全部完成；左手抓起 → 右手在空中接稳 → 左手松开退开 → 右手保持同一抓持运输 → 入箱释放稳定 |

共持全过程门槛仍为 **5 mm / 0.05 rad**；稳态物块速度门槛仍为 **0.02 m/s / 0.2 rad/s**，每指最低接触力 0.5 N。控制位置容差为 P02A 的臂 **0.010 rad**、夹爪 **0.001 m**；未扩大稳定、跟踪、停止或租约门槛。

连续诊断在空中双持、左手退开后的右手单持、入箱释放后分别取得 6 个持续采样。释放后两手均无物块接触；物块八个实际顶点位于箱内，箱底支撑力 **2.943 N**，最低顶点相对底板 −0.108 mm（接触压入），最终中心为 `(0.637024, -0.271983, 0.821892) m`。它证明连续动作可执行，尚不等同于 P03 的任务生命周期、Teacher 服务或正式采集完成。

[实测状态重建图](../../../var/reports/p02b/p02b-38bce3cc/artifacts/states.png) 使用快照物块位姿和最近实测关节状态，不是原始录像。

## 物理模型与执行修复

- 机器人装配显式启用自身重力补偿；自由物块保留重力。位置伺服不再有约 11 mm 的 TCP 自重静差，共持重规划避免将已下垂的测量直接重复变成伺服位置参考。
- 物理配方使用 `cone: elliptic`、`impratio: 10.0`、`noslip_iterations: 3`。隔离同目标回放确认椭圆摩擦锥降低切向蠕滑；有限接触面的扭转摩擦支持两端单手抓持。
- `object_contact_dim: 4`，`object_friction: [1.0, 0.02, 0.0001]`。滑动系数为 1.0；扭转系数 0.02 的单位为米，表示模拟接触斑的抗扭作用；滚动项在四维接触下未启用。参数是本场景的仿真模型，不是实物摩擦标定。求解设置依据见 [MuJoCo 防滑建模说明](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip)。
- 手指滑轨阻尼 `finger_damping: 200.0` 抑制 mimic/contact 微振；Marvin I/O 将两指阻尼按闭合量坐标投影到速度前馈，保留开度/闭合量的正确符号。源 URDF、STL、质量、惯量和碰撞排除未改变。
- 接触完成按最终参考是否仍窄于实测孔径、双指同物体接触、力和速度判断。增加参考开度但仍在受压区间属于减压；真正张开超过物块孔径必须满足位置完成。该规则与接触跟踪豁免一致，避免中间减压窗口被误判超时。
- 指尖最小孔径与整段指面的宽度分开解释：源 mesh 有渐变，测量站点不是覆盖全指长的平行碰撞面。TCP 和 profile 数值未变。

物块仍为 `[0.03, 0.10, 0.03] m` 半尺寸，箱位 `(0.64, -0.27)`、外尺寸 0.32 m、高 0.10 m。没有提高物块、增加指面代理、增加机器人碰撞排除或约束物块位姿来取得通过。

## 任务实现与职责

- [plan.py](../../../simulation/mujoco/src/robot_backend_sim/tasks/handover/plan.py) 生成 24 个目标、25 个候选段。连续主链为 `left_lift → left_handover → right_handover_wait/pregrasp/grasp → left_retreat → right_preplace → right_place`。左手 `+0.06 m`、右手 `−0.06 m / 20°` 抓持；右手从空中接管到放置保持同一个刚体抓持变换，不切换为中心抓持。
- [cohold.py](../../../simulation/mujoco/src/robot_backend_sim/tasks/handover/cohold.py) 接收一次固定的实测抓持基准及具名关节状态，为同一实际命令段求 IK、带物净空和双 TCP 相对几何。诊断仅调用模块、提交控制和保存证据，不维护另一套规划。
- 机器人适配提供具名关节、TCP、指面、阻尼/执行器映射；MuJoCo 数值求解留在 simulation；抓持点、交接/退避/放置目标及允许接触归 handover 任务和配方。
- 物理真值由 owner 捕获一致快照；规划使用私有状态，runtime 不读取 MjData。新证据保存 `input_configs`，包含本次运行的场景和规划数值。
- 世界旋转包含机器人基座与物块局部偏航；持物包络保存完整物块旋转，避免坐标变换丢失朝向。

最终规划报告：[p02b-verified-final-plan](../../../var/reports/handover-paths/p02b-verified-final-plan/plan.json)，所有目标/候选段通过，规划耗时 0.753 s，连续主链最小净空 **4.689 mm**；共持 waypoint 机器人净空 **17.378 mm**。

报告 `planned_duration_s=34.781 s` 是候选段累计（含独立/共持分支），不表示完整 Teacher 的执行预算。连续物理诊断跨度 **122.290 s 仿真时间**，测试耗时 **128.59 s**；P03 应根据其实际调度重测超时预算。

## 环境、命令与回归

simulation 容器 `/app`，ROS 2 Jazzy，MuJoCo 3.17.0。源码 `/app:ro`、方案 `/shared:ro`、产物 `/data`、SDK `/opt/ros-dev:ro`。SDK 包集 `robot_api_msgs + sim_api_msgs`，校验通过：`7ff026bfa3634a7d7983f115b0d9ad308180b89ef90b14a9cae987cef4ccbf18`。

```bash
# 宿主机仓库根目录；每个测试创建独立 backend，退出后释放进程/租约
# 将末尾测试路径替换为 left_hold、right_hold、dual_hold、stop_hold 可运行独立诊断
docker compose -f simulation/mujoco/docker/docker-compose.yml run --rm --no-deps \
  -v "$PWD:/workspace:ro" -v "$PWD/var:/data" \
  -e PROJECT_ROOT=/workspace -w /workspace simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && python3 -m pytest -p no:cacheprovider -q tests/integration/test_marvin_transfer_ros.py -s'

# simulation 容器 /app
scripts/run.sh marvin --run-id <new-scene-run-id>
scripts/run.sh handover --run-id <new-plan-run-id>
scripts/run.sh prepare \
  --template /shared/scenarios/marvin_handover/configs/simulation/backend/backend_marvin_control.yaml \
  --seed 0 --run-id <new-run-id> --output /data/prepared/<new-run-id>
```

最终准备产物：`var/prepared/p02b-verified/backend.yaml`。最终场景检查：[p02b-verified-final-scene](../../../var/reports/sim-check/p02b-verified-final-scene/report.json)，状态 succeeded，三路渲染保留。

| 验证 | 结果 | 日志 |
| --- | --- | --- |
| 左/右/共持/带物停止四个真实控制诊断 | 4 passed，315.74 s | `var/reports/p02b/takeover-verified-holds.log` |
| 连续空中接管与入箱 | 1 passed，128.59 s | `var/reports/p02b/takeover-transfer-02.log` |
| Panda/Marvin 场景、接触、控制、跟随、运动学、I/O | 322 passed，16.25 s | `var/reports/p02b/takeover-verified-regression.log` |
| Panda backend ROS、Marvin 标准控制/拒绝/停止、阶段几何 | 8 passed，21.12 s | `var/reports/p02b/takeover-verified-api-final.log` |
| 非零基座偏航与完整旋转持物变换 | 3 passed，1.28 s | `var/reports/p02b/takeover-verified-frames.log` |

历史未通过尝试保留在 `var/reports/p02b/`，不计入本次验收。原有 follower 配置测试按职责修正：配置验证形状/唯一性，关节顺序在绑定所选 robot profile 时校验；越限拒绝测试先等待上一有效动作收敛，再核对拒绝后的状态。

## P03 接入点

直接复用 `plan_handover`、`plan_cohold_step`、owner 接触快照和五个物理诊断。下一阶段实现任务生命周期插件、Teacher IDL/SDK、门禁/评价及 runtime 专用适配；采集和训练仍按 P04/P05 推进。当前尚无 Teacher、binding、正式数据或模型训练成果。

底座仍为仅渲染网格；后续路径进入未建模底座区域时须先补经验证代理。新的模型、材料或场景位姿须重验受影响的抓持与路径。
