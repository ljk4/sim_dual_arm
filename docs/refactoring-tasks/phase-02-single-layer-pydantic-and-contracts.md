# Phase 2 任务书：单层 Pydantic v2 声明式模型重构（严禁双层套壳）与轨迹数学下沉

## 【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】
1. **前置条件**：确认 **Phase 1** 已完成且测试通过。你当前只负责执行 **Phase 2**。
2. **【严禁双层定义套壳红线（最重要！）】**：
   - **绝对禁止**在任何 `*_config.py` 或 `contracts/*.py` 中同时定义一个私有 `class _XxxModel(StrictContractModel)` **和** 一个公开 `@dataclass class XxxConfig`，然后在 `__post_init__` 里来回转换拷贝！
   - **公开的配置类与契约 DTO 类本身必须直接继承 `StrictContractModel`（或 `StrictMappingModel`）**！
   - **硬性禁令**：改造后的配置文件与契约 DTO 文件中，**严禁出现 `from dataclasses import dataclass`**，**严禁出现 `def __post_init__`**，**严禁出现 `class _.*Model` 影子类**，**严禁在 `validation.py` 中编写 `check_bounded_int` / `check_finite_number` 之类的过程式校验函数**！
3. **解除单元测试对旧手写错误文案的过度耦合**：
   - 原 `tests/unit/test_*config*.py` 和契约单测中存在大量精确匹配旧手写校验英文短语的断言（例如断言必须包含 `"booleans are not accepted"` 或 `"must be in 1..4096"`）。
   - **本阶段要求同步更新这些测试断言**：只需断言**抛出了正确的异常类型（如 `BackendConfigError`、`SceneConfigError`、`ContractError`）以及正确的错误码/出错字段路径**，不再强求匹配旧的自然语言描述，从而彻底释放 Pydantic 原生声明式校验的能力！

---

## 1. 环境依赖与统一校验基类 `contracts/python/src/robot_contracts/validation.py`

### 1.1 Dockerfile 与 `pyproject.toml` 依赖更新
1. 在 [`simulation/mujoco/docker/Dockerfile`](../../simulation/mujoco/docker/Dockerfile) 和 [`runtime/ros2/docker/Dockerfile`](../../runtime/ros2/docker/Dockerfile) 的 `apt-get install` 列表中加入 `python3-pydantic`（Ubuntu 24.04 官方源内置 Pydantic v2，无需修改 `requirements.lock`；若宿主机容器镜像尚未重建，确保容器内可通过 `python3-pydantic` 或 venv 使用 `pydantic>=2.0`）。
2. 在 [`contracts/python/pyproject.toml`](../../contracts/python/pyproject.toml) 的 `dependencies` 中加入 `"pydantic>=2.0"`。

### 1.2 新建精简的 `contracts/python/src/robot_contracts/validation.py`（严格控制在 **$\le 170$ 行**）
提供全仓通用的 Pydantic v2 声明式类型注解、严格模型基类与唯一键 YAML 加载器（消除此前全仓复制粘贴 9 遍的 `_UniqueKeyLoader`）：
- **声明式类型注解**：
  - `PositiveInt`、`NonNegativeInt`
  - `StrictFloat`（通过 `BeforeValidator` 允许 `int` 自动转 `float`，但对 `bool` 原样放行交给 `Field(strict=True, allow_inf_nan=False)` 拒绝）、`PositiveFloat`、`NonNegativeFloat`
  - `NonEmptyStr`、`SafeIdStr`、`Sha256HexStr`
  - `as_tuple`（将输入的 `list` 转为不可变 `tuple`）
- **基类与加载器**：
  - `StrictContractModel(BaseModel)`：`model_config = ConfigDict(frozen=True, extra="forbid", strict=True, allow_inf_nan=False, validate_default=True)`，提供 `validate_dict()`、`canonical_dict()`、`canonical_sha256()`。
  - `StrictMappingModel(StrictContractModel)`：用于配置模块，通过 `validate_config_model(model_cls, data, source, error_cls)` 统一将 `ValidationError` 转换为各模块的 `XxxConfigError(source, field, message)`。
  - `UniqueKeyYamlLoader`、`parse_unique_yaml()`、`read_unique_yaml()`：统一处理 YAML 重复键拒绝。
- **严禁在 `validation.py` 中塞入任何 `check_bounded_int`、`check_finite_vector` 等过程式函数！**

---

## 2. 将 `contracts/` 中的轨迹数学剥离下沉至 `robot_contracts/trajectory/` 子包并瘦身

当前 `contracts/python/src/robot_contracts/` 顶层塞入了近 3,000 行三次多项式样条求极值与 Ruckig 轨迹数学代码。将其下沉至 `contracts/python/src/robot_contracts/trajectory/` 子包，并用 `StrictContractModel` 大幅削减其中的手写类型检查：

1. **下沉并合并模块（删除顶层原文件，严禁留空壳）**：
   - `reference.py`（900 行）$\rightarrow$ `trajectory/spline.py`（**目标 $\le 420$ 行**）：
     - `ReferenceSegment`、`ReferenceSample`、`ReferencePlan` 直接继承 `StrictContractModel`，删除 400+ 行手写 `_integer` / `_finite_number` 循环检查，仅保留三次样条求值与极值根计算。
   - `follower_math.py`（497 行）+ `follower.py`（596 行）$\rightarrow$ 合并为 `trajectory/follower.py`（**目标 $\le 450$ 行**）：
     - `PreparedFollowerPlan`、`FollowerPhase`、`NormalizedFollowerPositions` 直接继承 `StrictContractModel`，删除原 `follower.py` 中 334 行的 `__post_init__` 手写类型校验。
   - `follower_reference.py`（921 行）+ `follower_reference_digest.py`（133 行）$\rightarrow$ 合并为 `trajectory/follower_reference.py`（**目标 $\le 420$ 行**）：
     - `FollowerReference`、`FollowerSegment` 直接继承 `StrictContractModel`。
   - 在 `trajectory/__init__.py` 中统一导出，并同步更新全仓 import。

---

## 3. 顶层契约 DTO 全面改为单层 `StrictContractModel`

重写 `contracts/python/src/robot_contracts/` 顶层其余文件，彻底消灭 `dataclass` 与 `__post_init__`：
1. `models.py`（**目标 $\le 110$ 行**）：`RunContext`、`JointSpec`、`CameraSpec`、`RobotProfile`、`Target`（删除已废弃的 `Trajectory`）。
2. `profiles.py`（当前 345 行 $\rightarrow$ **目标 $\le 140$ 行**）：直接用 `RobotProfile.validate_dict()` 验证 JSON profile，删除手写字段校验与 `_UniqueKeyLoader`。
3. `action_chunks.py`（当前 279 行 $\rightarrow$ **目标 $\le 140$ 行**）：`ActionWindow`、`ChunkAnchor` 直接继承 `StrictContractModel`。
4. `native_reference.py`（当前 349 行 $\rightarrow$ **目标 $\le 150$ 行**）与 `native_feedback.py`（当前 651 行 $\rightarrow$ **目标 $\le 240$ 行**）：所有类直接继承 `StrictContractModel`，删除数百行重复的 `_integer` / `_number` / `_string` 函数。
5. `control.py`（**目标 $\le 150$ 行**）与 `episodes.py`（**目标 $\le 90$ 行**）。

---

## 4. 全仓 9 个 YAML 配置模块改为单层 `StrictContractModel`（彻底删除 `_UniqueKeyLoader` 与 `dataclass`）

对以下 9 个配置文件，**直接让公开 Config 类继承 `StrictMappingModel` / `StrictContractModel`**，删除所有手写 `_integer` / `_number` / `_boolean` / `_mapping` / `_reject_unknown` / `_UniqueKeyLoader`：

1. `simulation/mujoco/src/robot_backend_sim/follower_config.py`（251 行 $\rightarrow$ **目标 $\le 95$ 行**）
2. `simulation/mujoco/src/robot_backend_sim/stream_config.py`（206 行 $\rightarrow$ **目标 $\le 95$ 行**）
3. `simulation/mujoco/src/robot_backend_sim/motion_config.py`（275 行 $\rightarrow$ **目标 $\le 110$ 行**）
4. `simulation/mujoco/src/robot_backend_sim/backend_config.py`（367 行 $\rightarrow$ **目标 $\le 150$ 行**）
5. `simulation/mujoco/src/robot_backend_sim/scene_config.py`（552 行 $\rightarrow$ **目标 $\le 200$ 行**）
6. `simulation/mujoco/src/robot_backend_sim/kinematics_config.py`（510 行 $\rightarrow$ **目标 $\le 190$ 行**）
7. `simulation/mujoco/src/robot_backend_sim/task_config.py`（464 行 $\rightarrow$ **目标 $\le 200$ 行**）
8. `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_config.py`（750 行 $\rightarrow$ **目标 $\le 260$ 行**，用嵌套 Pydantic 模型直接验证整个 YAML 文档，严禁保留后半部分 400 行手动字典拆解！）
9. `learning/lerobot/src/dual_arm_learning/config.py`（875 行 $\rightarrow$ **目标 $\le 260$ 行**，用嵌套 Pydantic 模型直接验证 `DatasetConfig` 与 `TrainingConfig`，删除所有 `mapping` / `integer` / `number` 手写函数！）

---

## 5. 本阶段量化验收标准（必须逐条执行并全部达标）

1. **反作弊静态检查（严禁存在 `_.*Model` 影子类、`__post_init__`、`dataclass` 或 `_UniqueKeyLoader`）**：
   ```bash
   find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
   # 以下 grep 输出必须完全为空！
   grep -rn "class _.*Model\|def __post_init__\|from dataclasses import\|_UniqueKeyLoader" \
     contracts/python/src/robot_contracts/ \
     simulation/mujoco/src/robot_backend_sim/*_config.py \
     runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_config.py \
     learning/lerobot/src/dual_arm_learning/config.py
   ```
2. **行数硬指标检查**：
   ```bash
   # 1. 10 个配置与 profile 文件合计必须 <= 1,700 行（原为 4,595 行）
   wc -l \
     contracts/python/src/robot_contracts/profiles.py \
     simulation/mujoco/src/robot_backend_sim/*_config.py \
     runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_config.py \
     learning/lerobot/src/dual_arm_learning/config.py

   # 2. contracts 顶层文件合计必须 <= 1,150 行，trajectory 子包合计必须 <= 1,350 行
   wc -l contracts/python/src/robot_contracts/*.py contracts/python/src/robot_contracts/trajectory/*.py
   ```
3. **代码检查与三端测试全部通过**：
   ```bash
   ruff check .
   ./simulation/mujoco/scripts/run.sh test
   ./runtime/ros2/scripts/run.sh test
   ./learning/lerobot/scripts/run.sh test
   ```
