# Apex SDK 通信协议文档

版本: 1.2.1
更新日期: 2026-07-20

---

## 修改记录

| 版本 | 日期 | 修改内容 | 修改人 |
|------|------|----------|--------|
| 1.0 | 2026-06-11 | 初版：TCP 命令通道 + UDP 实时通道完整定义 | wangyu |
| 1.0.1 | 2026-06-11 | 统一所有对外接口角度单位为度(°)，UDP 通道不再使用弧度 | wangyu |
| 1.0.2 | 2026-06-11 | 请求中 id 字段改为服务端自增，客户端不再需要传 id | wangyu |
| 1.0.3 | 2026-06-11 | trajectory_exec 添加 jointVelocityTarget/jointAccelerationTarget/target_*_velocity/acceleration 参数，全部可选 | wangyu |
| 1.1.0 | 2026-06-12 | 新增 forward_kinematics / inverse_kinematics 命令 | - |
| 1.2.0 | 2026-07-14 | 新增在线跟踪 set/get/update（笛卡尔 + 关节空间 mode） | - |
| 1.2.1 | 2026-07-20 | 修正 referenceFrame 枚举为 algo_core≥0.0.13 定义；澄清 UDP 位姿坐标系与在线跟踪默认一致（World） | - |

---

## 概述

Apex SDK 提供两个通信通道用于外部程序控制机械臂：

| 通道 | 端口 | 协议 | 用途 |
|------|------|------|------|
| 命令通道 | TCP 9100 | JSON (NDJSON) | 命令控制、配置查询、状态订阅 |
| 实时通道 | UDP 9101 | JSON (NDJSON) status-push | 状态推送（当前实现）；历史曾用定长二进制帧 |

### 参考坐标系 `referenceFrame`

笛卡尔相关命令（轨迹段、在线跟踪、示教等）共用下列枚举，与控制器 `KDL::ReferanceFrameType`（algo_core≥0.0.13）一致：

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | Base | 机械臂基座坐标系 |
| 1 | World | 世界坐标系（**默认**；与 UDP `currentPose` 一致） |
| 2 | Work | 当前工作坐标系 |
| 3 | Tool | 工具 / TCP 坐标系 |
| 4 | Tool（兼容） | 旧协议 Tool 编号，控制器仍映射为 Tool |

**相对旧协议的变更（易踩坑）：**

| 值 | 旧协议含义 | 当前含义 |
|----|------------|----------|
| 0 | Work | **Base** |
| 1 | World | World（不变） |
| 2 | Base | **Work** |
| 3 | Flange | **Tool** |
| 4 | Tool | Tool（仍兼容） |

未传 `referenceFrame` 时：轨迹 / 在线跟踪等默认 **1（World）**；笛卡尔示教默认 **3（Tool）**（与 JogPlanner 一致）。旧客户端若依赖「轨迹默认 Tool」或不传字段，行为会变成 World。

笛卡尔在线跟踪回灌建议：使用 UDP `currentPose`，并传 `referenceFrame=1`。

---

## 一、TCP 命令通道 (端口 9100)

### 1.1 协议格式

- 传输层：TCP 长连接
- 消息格式：每条消息为一行 JSON，以 `\n` 结尾（NDJSON）
- 编码：UTF-8
- 字节序：不适用（JSON 文本）

**请求格式：**

```json
{"id":1,"cmd":"命令名","arm":"arm_left","参数1":"值1"}\n
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| id | int64 | 是 | 请求序号，响应中原样返回 |
| cmd | string | 是 | 命令名称 |
| arm | string | 部分命令需要 | 目标臂名称（如 "arm_left"、"arm_right"） |

**成功响应：**

```json
{"id":1,"ok":true,"msg":"ok","data":{...}}\n
```

**失败响应：**

```json
{"id":1,"ok":false,"err":1001,"msg":"错误描述"}\n
```

| 字段 | 类型 | 说明 |
|------|------|------|
| id | int64 | 对应请求的 id |
| ok | bool | 是否成功 |
| msg | string | 结果消息 |
| err | int | 错误码（仅失败时） |
| data | object | 返回数据（部分命令有） |

### 1.2 错误码

| 错误码 | 说明 |
|--------|------|
| 1001 | 业务逻辑错误（如 planner busy） |
| 1002 | 未知命令 |
| 1003 | 无效的臂名称 |
| 1004 | 状态不可用 |

---

### 1.3 命令列表

#### 连接与会话

##### connect

建立连接，获取可用臂列表和协议版本。

```json
→ {"id":1,"cmd":"connect"}
← {"id":1,"ok":true,"version":1,"arms":["arm_left","arm_right"],"dof":7}
```

| 返回字段 | 类型 | 说明 |
|----------|------|------|
| version | int | 协议版本号 |
| arms | string[] | 可用臂名称列表 |
| dof | int | 关节数 |

##### heartbeat

心跳保活。

```json
→ {"id":2,"cmd":"heartbeat"}
← {"id":2,"ok":true,"msg":"ok"}
```

##### acquire_control

获取控制权（预留，当前直接返回成功）。

```json
→ {"id":3,"cmd":"acquire_control","arm":"arm_left"}
← {"id":3,"ok":true,"msg":"ok"}
```

##### release_control

释放控制权（预留，当前直接返回成功）。

```json
→ {"id":4,"cmd":"release_control","arm":"arm_left"}
← {"id":4,"ok":true,"msg":"ok"}
```

---

#### 使能与模式

##### enable

使能机械臂。

```json
→ {"id":5,"cmd":"enable","arm":"arm_left"}
← {"id":5,"ok":true,"msg":"ok"}
```

##### disable

下使能机械臂。

```json
→ {"id":6,"cmd":"disable","arm":"arm_left"}
← {"id":6,"ok":true,"msg":"ok"}
```

##### emergency_stop

急停（所有臂立即停止）。

```json
→ {"id":7,"cmd":"emergency_stop"}
← {"id":7,"ok":true,"msg":"ok"}
```

##### set_robot_mode

设置机器人控制模式。

```json
→ {"id":8,"cmd":"set_robot_mode","arm":"arm_left","mode":0}
← {"id":8,"ok":true,"msg":"ok"}
```

| 参数 | 类型 | 说明 |
|------|------|------|
| mode | int | 0=位置模式, 1=力矩模式 |

##### get_robot_mode

查询当前控制模式。

```json
→ {"id":9,"cmd":"get_robot_mode","arm":"arm_left"}
← {"id":9,"ok":true,"msg":"ok","data":{...}}
```

##### set_vel_scale

设置全局速度比例。

```json
→ {"id":10,"cmd":"set_vel_scale","arm":"arm_left","scale":0.5}
← {"id":10,"ok":true,"msg":"ok"}
```

| 参数 | 类型 | 说明 |
|------|------|------|
| scale | float | 速度比例 0.0-1.0 |

##### get_vel_scale

查询当前速度比例。

```json
→ {"id":11,"cmd":"get_vel_scale","arm":"arm_left"}
← {"id":11,"ok":true,"msg":"ok","data":{...}}
```

---

#### 示教

##### teaching_start

开始示教（点动）。

```json
→ {"id":12,"cmd":"teaching_start","arm":"arm_left","teachingMode":0,"jointAxis":0,"direction":1,"speedRatio":0.3,"isStep":false,"stepValue":0}
← {"id":12,"ok":true,"msg":"ok"}
```

| 参数 | 类型 | 说明 |
|------|------|------|
| teachingMode | int | 0=关节空间, 1=笛卡尔空间 |
| jointAxis | int | 关节轴索引 0-6（关节模式时） |
| cartesianAxis | int | 笛卡尔轴 0=X,1=Y,2=Z,3=RX,4=RY,5=RZ（笛卡尔模式时） |
| direction | int | 运动方向 1=正向, -1=负向 |
| speedRatio | float | 速度比例 0.0-1.0 |
| isStep | bool | 是否步进模式 |
| stepValue | float | 步进值（关节:度, 位置:米） |
| referenceFrame | int | 可选；笛卡尔示教参考系，见上文枚举；默认 **3=Tool** |

##### teaching_stop

停止示教。

```json
→ {"id":13,"cmd":"teaching_stop","arm":"arm_left"}
← {"id":13,"ok":true,"msg":"ok"}
```

---

#### 轨迹运动

##### trajectory_exec

执行轨迹运动。支持 MoveJ（关节运动）、MoveL（直线运动）、MoveC（圆弧运动）。

**单臂运动：**

```json
→ {"id":14,"cmd":"trajectory_exec","arm":"arm_left","segments":[
    {
      "armId": 0,
      "motionType": 0,
      "jointPositionTarget": [30, 20, 10, 0, -10, -20, -30],
      "velocityRatio": 0.5,
      "maxVelocity": 60.0,
      "maxAcceleration": 40.0,
      "blendRadius": 0.0
    }
  ]}
← {"id":14,"ok":true,"msg":"ok"}
```

**双臂同步运动（分别发送）：**

```json
→ {"id":14,"cmd":"trajectory_exec","arm":"arm_left","segments":[{"armId":0,...}]}
← {"id":14,"ok":true,"msg":"ok"}
→ {"id":15,"cmd":"trajectory_exec","arm":"arm_right","segments":[{"armId":1,...}]}
← {"id":15,"ok":true,"msg":"ok"}
```

**多段连续轨迹（一次规划多个点，中间不停顿）：**

```json
→ {"id":16,"cmd":"trajectory_exec","arm":"arm_left","segments":[
    {"armId":0,"motionType":0,"jointPositionTarget":[10,10,10,10,10,10,10],"velocityRatio":0.5,"maxVelocity":60,"maxAcceleration":40,"blendRadius":5.0},
    {"armId":0,"motionType":0,"jointPositionTarget":[20,20,20,20,20,20,20],"velocityRatio":0.5,"maxVelocity":60,"maxAcceleration":40,"blendRadius":5.0},
    {"armId":0,"motionType":0,"jointPositionTarget":[0,0,0,0,0,0,0],"velocityRatio":0.5,"maxVelocity":60,"maxAcceleration":40,"blendRadius":0.0}
  ]}
← {"id":16,"ok":true,"msg":"ok"}
```

**segments 参数说明：**

| 字段 | 类型 | 必填 | 单位 | 说明 |
|------|------|------|------|------|
| armId | int | 是 | - | 臂索引（0=第一臂, 1=第二臂） |
| motionType | int | 是 | - | 0=MoveJ(关节), 1=MoveL(直线), 2=MoveC(圆弧) |
| jointPositionTarget | float[7] | 可选 | 度 | 目标关节位置（MoveJ 时使用） |
| jointVelocityTarget | float[7] | 可选 | 度/s | 目标关节速度 |
| jointAccelerationTarget | float[7] | 可选 | 度/s² | 目标关节加速度 |
| cartesianTarget | object | 可选 | 位置:米, 姿态:度 | 目标位姿 {x,y,z,rx,ry,rz}（MoveL/C 时使用） |
| auxiliaryPose | object | 可选 | 位置:米, 姿态:度 | 圆弧辅助点 {x,y,z,rx,ry,rz}（MoveC 时使用） |
| velocityRatio | float | 可选 | - | 速度比例 0.0-1.0，默认 1.0 |
| accelerationRatio | float | 可选 | - | 加速度比例 0.0-1.0，默认 1.0 |
| target_linear_velocity | float | 可选 | m/s | 笛卡尔直线速度（MoveL/C 时使用） |
| target_angular_velocity | float | 可选 | 度/s | 笛卡尔角速度（MoveL/C 时使用） |
| target_linear_acceleration | float | 可选 | m/s² | 笛卡尔直线加速度 |
| target_angular_acceleration | float | 可选 | 度/s² | 笛卡尔角加速度 |
| blendRadius | float | 可选 | 米 | 混合半径，>0 时拐角处圆弧过渡，默认 0 |
| workFrameName | string | 可选 | - | 工作坐标系名称 |
| toolFrameName | string | 可选 | - | 工具坐标系名称 |
| referenceFrame | int | 可选 | 1 | 参考坐标系，见上文「参考坐标系 referenceFrame」；默认 1=World |

> 所有参数均为可选。不传时使用系统默认值（速度/加速度由配置文件中的限制决定）。
>
> 注意：轨迹执行期间再发 trajectory_exec 会返回 `planner busy`，需等待当前轨迹完成或先调用 trajectory_stop。

##### trajectory_pause

暂停当前轨迹执行。

```json
→ {"id":17,"cmd":"trajectory_pause","arm":"arm_left"}
← {"id":17,"ok":true,"msg":"ok"}
```

##### trajectory_resume

恢复暂停的轨迹。

```json
→ {"id":18,"cmd":"trajectory_resume","arm":"arm_left"}
← {"id":18,"ok":true,"msg":"ok"}
```

##### trajectory_stop

停止当前轨迹执行。对在线跟踪模式会触发 OnlinePlanner 软 S 曲线减速，期间需继续等到停稳（`get_online_tracking_state.softStopping=false`）。

```json
→ {"id":19,"cmd":"trajectory_stop","arm":"arm_left"}
← {"id":19,"ok":true,"msg":"ok"}
```

---

#### 在线跟踪 (Online Tracking)

在线跟踪将稀疏目标送入控制器 `OnlinePlanner`，由规划线程（约 2ms）输出平滑关节指令。支持笛卡尔与关节空间两种 mode。

##### set_online_tracking

开启 / 关闭在线跟踪。关闭时进入软停；再次开启同 mode 会 `resume`。

```json
→ {"cmd":"set_online_tracking","arm":"arm_left","enable":true,"mode":"joint","referenceFrame":1}
← {"ok":true,"msg":"ok"}

→ {"cmd":"set_online_tracking","arm":"arm_left","enable":false,"mode":"joint"}
← {"ok":true,"msg":"ok"}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| arm | string | 是 | 目标臂名称 |
| enable | bool | 是 | true=开启，false=软停后关闭 |
| mode | string/int | 否 | `"cartesian"`/`0`（默认）或 `"joint"`/`1` |
| referenceFrame | int | 否 | 笛卡尔参考系，见上文枚举；默认 **1=World**；关节模式可忽略 |

##### get_online_tracking_state

```json
→ {"cmd":"get_online_tracking_state","arm":"arm_left"}
← {"ok":true,"data":{"enable":true,"mode":1,"softStopping":false}}
```

| 字段 | 说明 |
|------|------|
| enable | 是否处于在线跟踪（软停过程中仍为 true） |
| mode | 0=Cartesian，1=JointSpace |
| softStopping | 是否正在软停减速 |

##### update_tracking_target

更新在线跟踪目标。笛卡尔与关节二选一；需与当前 mode 一致。

**笛卡尔目标**（位置 m，姿态 deg）：

```json
→ {"cmd":"update_tracking_target","arm":"arm_left",
   "x":0.4,"y":0.0,"z":0.5,"rx":0,"ry":90,"rz":0,"referenceFrame":1}
← {"ok":true,"msg":"ok"}
```

**关节目标**（度，稀疏 10–100Hz 即可）：

```json
→ {"cmd":"update_tracking_target","arm":"arm_left",
   "jointPositionTarget":[0,10,0,-60,0,90,0]}
← {"ok":true,"msg":"ok"}
```

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| x/y/z | number | m | 笛卡尔位置（笛卡尔模式） |
| rx/ry/rz | number | deg | 笛卡尔姿态（笛卡尔模式） |
| jointPositionTarget | number[] | deg | 7 维关节角（关节模式） |
| referenceFrame | int | - | 笛卡尔参考系，见上文枚举；建议与 UDP `currentPose` 一致时传 **1=World** |

**推荐流程（笛卡尔空间，与 UDP 位姿对齐）：**

1. `set_online_tracking(enable=true, mode=cartesian, referenceFrame=1)`
2. 读取 UDP `currentPose`（World）作为反馈
3. 循环 `update_tracking_target(x,y,z,rx,ry,rz, referenceFrame=1)`
4. `set_online_tracking(enable=false)` 后轮询 `get_online_tracking_state` 至停稳

**推荐流程（关节空间）：**

1. `trajectory_stop`（清残留轨迹）
2. `set_online_tracking(enable=true, mode=joint)`
3. 循环稀疏 `update_tracking_target(jointPositionTarget=...)`
4. `set_online_tracking(enable=false)` 或 `trajectory_stop`
5. 轮询 `get_online_tracking_state` 直到 `enable=false` 且 `softStopping=false`

---

#### 运动学计算

##### forward_kinematics

正向运动学：根据关节角度计算末端位姿。

```json
→ {"cmd":"forward_kinematics","arm":"arm_left","joint_positions":[0,0,0,0,0,0,0]}
← {"id":25,"ok":true,"data":{"pose":{"x":0.0,"y":0.0,"z":0.85,"rx":0.0,"ry":0.0,"rz":0.0}}}
```

| 参数 | 类型 | 必填 | 单位 | 说明 |
|------|------|------|------|------|
| arm | string | 是 | - | 目标臂名称 |
| joint_positions | float[7] | 是 | 度 | 输入关节角度 |

**返回值：**

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| data.pose.x/y/z | float | 米 | 末端位置 |
| data.pose.rx/ry/rz | float | 度 | 末端姿态（RPY） |

##### inverse_kinematics

逆向运动学：根据目标位姿计算关节角度。

```json
→ {"cmd":"inverse_kinematics","arm":"arm_left","pose":{"x":0.4,"y":0.0,"z":0.3,"rx":0,"ry":90,"rz":0},"seed":[0,0,0,0,0,0,0]}
← {"id":26,"ok":true,"data":{"joint_positions":[10.5,-20.3,5.1,-45.0,12.3,30.0,-5.2],"solver_return_code":0}}
```

| 参数 | 类型 | 必填 | 单位 | 说明 |
|------|------|------|------|------|
| arm | string | 是 | - | 目标臂名称 |
| pose | object | 是 | x/y/z:米, rx/ry/rz:度 | 目标末端位姿 |
| seed | float[7] | 可选 | 度 | 初始猜测关节角（不传则使用当前关节位置） |

**返回值：**

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| data.joint_positions | float[7] | 度 | 求解得到的关节角度 |
| data.solver_return_code | int | - | 求解器返回码，0=成功，负值=未收敛 |

> **注意**：IK 可能有多解或无解。`seed` 的选取会影响求解结果。若 `solver_return_code < 0`，表示在给定 seed 下未找到收敛解，可尝试不同的 seed。

---

#### 状态查询

##### get_state

查询当前全量状态（单次查询）。

```json
→ {"id":20,"cmd":"get_state"}
← {"id":20,"ok":true,"seq":12345,"arms":[
    {
      "armId": 0,
      "robotState": 2,
      "jointState": {
        "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "torques": [0.1, 0.2, 0.1, 0.05, 0.02, 0.01, 0.01]
      },
      "currentPose": {"x":0.5,"y":0.0,"z":0.4,"rx":0.0,"ry":90.0,"rz":0.0},
      "currentPoseTool": {"x":0.1,"y":0.0,"z":0.3,"rx":0.0,"ry":90.0,"rz":0.0}
    }
  ],
  "trajectoryProgress": [
    {"armId":0,"isFinished":true,"totalSegments":1,"currentSegment":0,"progress":1.0}
  ]}
```

**状态数据说明：**

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| positions | float[7] | 度 | 关节位置 |
| velocities | float[7] | 度/s | 关节速度 |
| torques | float[7] | Nm | 关节力矩 |
| currentPose.x/y/z | float | 米 | 世界坐标系末端位置 |
| currentPose.rx/ry/rz | float | 度 | 世界坐标系末端姿态 |
| currentPoseTool | object | 米/度 | 工具坐标系末端位姿（UDP status-push 及后续版本 get_state） |

**robotState 枚举：**

| 值 | 说明 |
|----|------|
| 0 | 未连接 |
| 1 | 空闲（已连接但未使能） |
| 2 | 已使能 |
| 3 | 示教模式 |
| 4 | 轨迹执行中 |
| 5 | 暂停 |
| 6 | 急停 |
| 7 | 故障 |

**trajectoryProgress 说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| armId | int | 臂索引 |
| isFinished | bool | 轨迹是否执行完成（到位判断依据） |
| totalSegments | int | 总段数 |
| currentSegment | int | 当前执行段索引 |
| progress | int | 执行进度百分比 0-100 |

---

#### 状态订阅

##### subscribe_state

订阅状态推送。订阅后服务端按指定频率主动推送状态。

```json
→ {"id":21,"cmd":"subscribe_state","freq":30}
← {"id":21,"ok":true,"msg":"ok"}
```

| 参数 | 类型 | 说明 |
|------|------|------|
| freq | int | 推送频率 Hz（范围 1-100，默认 10） |

订阅后服务端持续推送（无 id 字段）：

```json
← {"type":"state","seq":12346,"arms":[...]}\n
← {"type":"state","seq":12347,"arms":[...]}\n
...
```

##### unsubscribe_state

取消状态订阅。

```json
→ {"id":22,"cmd":"unsubscribe_state"}
← {"id":22,"ok":true,"msg":"ok"}
```

---

#### 实时控制管理

##### start_realtime

激活 UDP 实时通道。

```json
→ {"id":30,"cmd":"start_realtime","arm":"arm_left","freq":1000,"mode":"position"}
← {"id":30,"ok":true,"udp_port":9101}
```

| 参数 | 类型 | 说明 |
|------|------|------|
| arm | string | 目标臂名称 |
| freq | int | 频率 500 或 1000 Hz |
| mode | string | "position" / "velocity" / "torque" |

##### stop_realtime

停止 UDP 实时通道。

```json
→ {"id":31,"cmd":"stop_realtime","arm":"arm_left"}
← {"id":31,"ok":true,"msg":"ok"}
```

---

## 二、UDP 实时通道 (端口 9101)

### 2.1 概述

UDP 实时通道用于高频状态推送（如在线跟踪、力控监控）。通过 TCP 命令 `start_realtime` 激活，`stop_realtime` 停止。

工作模式：**JSON 单播推送**（NDJSON，每帧以 `\n` 结尾）

```json
← {"type":"status-push","seq":1,"timestamp_us":1234567890,"udp_ip":"<client-ip>","freq_hz":500,
   "armId":0,"arm":"arm_left","robotState":2,
   "jointState":{"positions":[...],"velocities":[...],"torques":[...]},
   "currentPose":{"x":0.5,"y":0.0,"z":0.4,"rx":0.0,"ry":90.0,"rz":0.0},
   "currentPoseReferenceFrame":1,
   "currentPoseTool":{"x":0.1,"y":0.0,"z":0.3,"rx":0.0,"ry":90.0,"rz":0.0},
   "currentPoseToolReferenceFrame":3}
```

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| currentPose | object | 米/度 | 世界坐标系末端位姿 |
| currentPoseReferenceFrame | int | — | 固定为 **1（World）** |
| currentPoseTool | object | 米/度 | 工具坐标系末端位姿 |
| currentPoseToolReferenceFrame | int | — | 固定为 **3（Tool）** |

> 历史版本曾使用定长二进制 `RtStatePacket`（见 2.2），当前控制器实现为 JSON 推送。

### 2.2 状态帧 RtStatePacket（控制器 → SDK）

总长度：288 字节，字节序：小端（Little-Endian）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 2 | magic | uint16 | 固定 0xAE01 |
| 2 | 2 | version | uint16 | 协议版本 (1) |
| 4 | 4 | sequence | uint32 | 递增序号 |
| 8 | 8 | timestamp_us | uint64 | 控制器微秒时间戳 |
| 16 | 1 | arm_id | uint8 | 臂 ID (0/1) |
| 17 | 1 | dof | uint8 | 关节数 (7) |
| 18 | 1 | robot_state | uint8 | RobotState 枚举 |
| 19 | 1 | control_mode | uint8 | 控制模式 |
| 20 | 4 | error_code | uint32 | 错误码 |
| 24 | 56 | q[7] | double[7] | 关节位置 (度) |
| 80 | 56 | dq[7] | double[7] | 关节速度 (度/s) |
| 136 | 56 | tau[7] | double[7] | 关节力矩 (Nm) |
| 192 | 48 | tcp_pose[6] | double[6] | 末端位姿 (x,y,z:米, rx,ry,rz:度) |
| 240 | 48 | external_wrench[6] | double[6] | 末端外力 (Fx,Fy,Fz:N, Tx,Ty,Tz:Nm) |

### 2.3 指令帧 RtCommandPacket（SDK → 控制器）

总长度：188 字节，字节序：小端（Little-Endian）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 2 | magic | uint16 | 固定 0xAE02 |
| 2 | 2 | version | uint16 | 协议版本 (1) |
| 4 | 4 | sequence | uint32 | 递增序号 |
| 8 | 8 | timestamp_us | uint64 | SDK 微秒时间戳 |
| 16 | 1 | arm_id | uint8 | 臂 ID (0/1) |
| 17 | 1 | dof | uint8 | 关节数 (7) |
| 18 | 1 | mode | uint8 | 0=位置, 1=速度, 2=力矩 |
| 19 | 1 | reserved | uint8 | 保留 (0) |
| 20 | 56 | target_q[7] | double[7] | 目标关节位置 (度) |
| 76 | 56 | target_dq[7] | double[7] | 目标关节速度 (度/s) |
| 132 | 56 | target_tau[7] | double[7] | 目标关节力矩 (Nm) |

### 2.4 使用流程

```
1. SDK 通过 TCP 发送 start_realtime
2. 控制器响应 ok 并返回 udp_port
3. SDK 向控制器 UDP:9101 发送第一个 RtCommandPacket（让控制器学习 SDK 地址）
4. 控制器开始向 SDK 推送 RtStatePacket
5. SDK 持续接收状态、计算、发送指令（每 1ms 一个循环）
6. 结束时 SDK 通过 TCP 发送 stop_realtime
```

### 2.5 安全机制

| 条件 | 控制器行为 |
|------|-----------|
| 连续 5 帧未收到指令 (5ms) | 保持最后一帧指令 |
| 超过 20ms 未收到指令 | 打印警告日志 |
| 超过 100ms 未收到指令 | 触发急停，自动退出实时模式 |
| TCP 连接断开 | 立即停止 UDP，安全停机 |
| sequence 号异常（回退） | 丢弃该帧 |

---

## 三、使用示例

### Python 示例（TCP 命令通道）

```python
import socket, json, time

class ApexSDK:
    def __init__(self, host, port=9100):
        self.sock = socket.create_connection((host, port), timeout=10)
        self.file = self.sock.makefile('r')
        self._seq = 0

    def send(self, cmd, **kwargs):
        self._seq += 1
        msg = json.dumps({"id": self._seq, "cmd": cmd, **kwargs}) + "\n"
        self.sock.sendall(msg.encode())
        return json.loads(self.file.readline())

    def close(self):
        self.sock.close()

# 使用
robot = ApexSDK("<controller-ip>")
print(robot.send("connect"))
robot.send("enable", arm="arm_left")
robot.send("trajectory_exec", arm="arm_left", segments=[{
    "armId": 0, "motionType": 0,
    "jointPositionTarget": [10, 10, 10, 10, 10, 10, 10],
    "velocityRatio": 0.3, "maxVelocity": 60.0,
    "maxAcceleration": 40.0, "blendRadius": 0.0
}])
time.sleep(3)
print(robot.send("get_state"))

# FK：计算末端位姿
fk = robot.send("forward_kinematics", arm="arm_left",
                joint_positions=[10, 10, 10, 10, 10, 10, 10])
print("FK result:", fk)

# IK：根据位姿求关节角
ik = robot.send("inverse_kinematics", arm="arm_left",
                pose={"x": 0.4, "y": 0.0, "z": 0.3, "rx": 0, "ry": 90, "rz": 0},
                seed=[0, 0, 0, 0, 0, 0, 0])
print("IK result:", ik)

robot.close()
```

### Python 示例（UDP 实时通道）

```python
import socket, struct, time

RT_STATE_FMT = "<HHI Q BB BB I 7d 7d 7d 6d 6d"
RT_STATE_SIZE = struct.calcsize(RT_STATE_FMT)  # 288

RT_CMD_FMT = "<HHI Q BB BB 7d 7d 7d"
RT_CMD_SIZE = struct.calcsize(RT_CMD_FMT)  # 188

# 先通过 TCP 发 start_realtime，然后：
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(("<device-ip>", 0))
server_addr = ("<controller-ip>", 9101)

seq = 0
while True:
    # 发送指令
    seq += 1
    cmd = struct.pack(RT_CMD_FMT,
        0xAE02, 1, seq, int(time.time() * 1e6),
        0, 7, 0, 0,
        *target_q, *target_dq, *target_tau)
    udp_sock.sendto(cmd, server_addr)

    # 接收状态
    data, addr = udp_sock.recvfrom(512)
    state = struct.unpack(RT_STATE_FMT, data)
    # state[8:15] = q[7], state[15:22] = dq[7], ...

    time.sleep(0.001)  # 1kHz
```

---

## 四、单位约定

所有对外接口统一使用**度(°)**作为角度单位：

| 通道 | 关节位置 | 关节速度 | 关节力矩 | 末端位置 | 末端姿态 |
|------|----------|----------|----------|----------|----------|
| TCP (JSON) | 度 (°) | 度/s | Nm | 米 (m) | 度 (°) |
| UDP (二进制) | 度 (°) | 度/s | Nm | 米 (m) | 度 (°) |

---

## 五、连接管理

- TCP 连接为长连接，客户端主动断开即释放资源
- 支持多个 TCP 客户端同时连接
- 同一臂同一时刻只能有一个客户端执行运动指令（第二个会收到 `planner busy`）
- 状态订阅和查询不受限制，多客户端可同时订阅
- UDP 实时通道需先通过 TCP 激活，TCP 断开后自动停止 UDP
