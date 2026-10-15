"""实验矩阵（Task 9）+ 场地余量校验（SDD 台账 R15）。

## 矩阵怎么来的

Global Constraints 的四条动作上限，加上 brief 要求的覆盖：

| 动作 | 上限 | 矩阵覆盖（**目标行程**） |
|---|---|---|
| 直行 `straight` | 200 mm | ±200、±150 |
| 横移 `strafe` | 80 mm | ±80、±50 |
| 斜行 `diagonal` | 120 mm | 四个对角方向各 120 mm（45°） |
| 原地旋转 `rotate_in_place` | 45° | ±30°、±45° |

每条 `repeats = 3`（单次抖动用重复的中位数消化）。

## `duration_s` 从实测速度模型反解（R68）

固件把"速度"当命令收，但**命令速度在这一段是惰性的**：`interpDuty` 在表格首行
（110 → 13.02 rad/s）以下线性外推，`bodyToPWM` 随后把 `duty` 抬到
`MIN_DUTY = 110`（43.1%）——矩阵 16 条的命令轮速全都落在首行以下，**真机下发的
一律是同一个占空比**。所以真机行程只由 `duration_s` 决定，时长由 opencode 自己
标定的速度模型反解：

    s_mm = V_STEADY · (T_ms − DEAD_MS),   V_STEADY = 0.56 m/s,  DEAD_MS = 143 ms

（600 ms → 256.3 mm、350 → 110.3、300 → 88.6 三点拟合，截距一致；旋转速率由偏航腿
`RUN 0 0 ±0.20 200` 的 13.05–15.72°（均值 13.87°）导出 ≈ 243°/s，死区同为 143 ms。）
`ms_for_mm()` / `ms_for_deg()` 反解时长并**向下取整到 1 ms**（量化误差 < 0.6 mm，
保证真实行程不超过上限）。

⚠ 于是"命令量"和"目标行程"**故意不相等**，两者都要看得懂：

- `expected_travel_mm(exp)` / `expected_delta_mm(exp)` / `expected_yaw_deg(exp)`
  = **真实会走**多少（= 目标，见上表）；`expected_start_xy(exp)` = 把它对中后的起点；
- `compare._command_param_1` 记进表的"计划距离"是 `v·T` = **命令量**（#01 = 100 mm，
  它的 desc 与目标却是 200 mm）——固件按命令只该走 100 mm，实际走 ≈200 mm，
  这个 2× 差**就是** Task 14 要辨识的东西（速度映射在 MIN_DUTY 以下失效）。
- **不许**把修正预先烘焙进命令速度（令 `v·T` 恰等于实际）：那会让"理想 vs 实际"
  恒为 0，Task 14 的信号当场消失。

`assert_within_field` 的动作上限与几何分支一律用**期望行程**，不用命令量。

## `assert_within_field(exp, start_xy=(0.0, 0.0))` 校验什么

场地原点 = **四边界码质心**（R4），半长 252.5 / 半宽 125.9 mm（R4/R15 的实测值）；
车体半包络 135 × 74.5 mm（270 × 149 / 2）。

**（1）动作上限**（任何分支都查）：上面四个数，超一个 `ValueError`。

**（2）几何**，按起点分两支（下面的"位移"一律指**期望行程** `expected_delta_mm`，
不是命令量 `v·T` —— R68）：

- **默认 `(0.0, 0.0)` = 名义场地中心**（起点未归位）：查**全场可行性** ——
  位移 ≤ `505-270 = 235 mm`（x）、`252-149 = 103 mm`（y），
  也就是"换一个合适的偏心起点就一定放得下"（brief Step 4 的"净余量"口径）。
  ⚠ 从**正中心**出发的单侧净余量只有 `252.5-135 = 117.5` / `125.9-74.5 = 51.4` mm，
  **直行 200、横移 80 从中心出发物理上放不下** —— 这正是 R15 把起点参数化的原因。
  默认分支若改做绝对扫掠校验，brief 的四条测试（默认起点、逐条不许抛）当场全红，
  所以绝对校验只挂在显式起点分支上。
- **显式 `start_xy`（Task 12 归位后传入）**：车体半包络在**起点与终点**都必须落在
  `±252.5 × ±125.9` 内（平移类的扫掠区域 = 两个端点包络的并集，按轴取包络是保守
  超集，够用）。旋转类只查**起点落位**：≤45° 旋转时车体角点的扫掠圆半径
  `√(135² + 74.5²) ≈ 154 mm` 本来就超出半宽 —— 计划 §2.6 的裁决是只限制
  "≤45°"（90° 才不可行，270 mm > 252 mm），边界码位移由 Task 10 的 R18 复核兜底。

`start_xy` 只取前两个元素（视觉归位给三元组时可直接传）；起始 **yaw 不在校验内**
—— R15 的前提是归位后车头已对齐。
"""
from __future__ import annotations

import math
from collections import namedtuple

# ---- 场地与车体几何（R4：场地原点 = 四边界码质心）----------------------------------
FIELD_HALF_LENGTH_MM = 252.5     # 场地半长（505 / 2）
FIELD_HALF_WIDTH_MM = 125.9      # 场地半宽（实测，R4/R15）
CAR_HALF_LENGTH_MM = 135.0       # 车体半包络（270 / 2）
CAR_HALF_WIDTH_MM = 74.5         # 车体半包络（149 / 2）
# "全场可行性"净余量：整段行程要在场地里放得下（spec §2.6 的口径）
FIELD_X_SLACK_MM = 505.0 - 270.0     # 235
FIELD_Y_SLACK_MM = 252.0 - 149.0     # 103

# ---- Global Constraints 的四条动作上限 --------------------------------------------
MAX_STRAIGHT_MM = 200.0
MAX_STRAFE_MM = 80.0
MAX_DIAGONAL_MM = 120.0
MAX_ROTATION_DEG = 45.0

_TOL = 1e-9                      # 命令值是十进制浮点数，200.0 会算出 200.00000000000003

Experiment = namedtuple("Experiment", "id action desc vx vy om duration_s repeats")

# ---- 真机速度模型（R68：从 opencode 的标定数据拟合，见模块 docstring）--------------
V_STEADY_MPS = 0.56              # 稳态速度（600/350/300 ms → 256.3/110.3/88.6 mm）
DEAD_MS = 143.0                  # 起步死区（三个行程反解出的截距一致）
YAW_RATE_DEG_S = 243.0           # 原地旋转速率（13.87° / 0.2 s 的偏航腿）

# 45° 斜行的每轴速度：|v|·√2 = 0.12 m/s（命令量；真机行程由 duration_s 决定，R68）
_DIAG_V = 0.12 / math.sqrt(2.0)
_OM = math.pi / 4.0              # 45°/s（命令量）

EXPERIMENTS = [
    # ---- 直行：±200 / ±150（上限 200）----
    Experiment(1, "straight", "直行前进 200 mm", 0.200, 0.0, 0.0, 0.500, 3),
    Experiment(2, "straight", "直行后退 200 mm", -0.200, 0.0, 0.0, 0.500, 3),
    Experiment(3, "straight", "直行前进 150 mm", 0.150, 0.0, 0.0, 0.410, 3),
    Experiment(4, "straight", "直行后退 150 mm", -0.150, 0.0, 0.0, 0.410, 3),
    # ---- 横移：±80 / ±50（步长上限 80）----
    Experiment(5, "strafe", "横移左移 80 mm", 0.0, 0.080, 0.0, 0.285, 3),
    Experiment(6, "strafe", "横移右移 80 mm", 0.0, -0.080, 0.0, 0.285, 3),
    Experiment(7, "strafe", "横移左移 50 mm", 0.0, 0.050, 0.0, 0.232, 3),
    Experiment(8, "strafe", "横移右移 50 mm", 0.0, -0.050, 0.0, 0.232, 3),
    # ---- 斜行：四个对角方向，各 120 mm（上限 120）----
    Experiment(9, "diagonal", "斜行前左 45° 120 mm", _DIAG_V, _DIAG_V, 0.0, 0.357, 3),
    Experiment(10, "diagonal", "斜行前右 45° 120 mm", _DIAG_V, -_DIAG_V, 0.0, 0.357, 3),
    Experiment(11, "diagonal", "斜行后左 45° 120 mm", -_DIAG_V, _DIAG_V, 0.0, 0.357, 3),
    Experiment(12, "diagonal", "斜行后右 45° 120 mm", -_DIAG_V, -_DIAG_V, 0.0, 0.357, 3),
    # ---- 原地旋转：±30° / ±45°（上限 45°）----
    Experiment(13, "rotate_in_place", "原地逆时针 30°", 0.0, 0.0, _OM, 0.266, 3),
    Experiment(14, "rotate_in_place", "原地顺时针 30°", 0.0, 0.0, -_OM, 0.266, 3),
    Experiment(15, "rotate_in_place", "原地逆时针 45°", 0.0, 0.0, _OM, 0.328, 3),
    Experiment(16, "rotate_in_place", "原地顺时针 45°", 0.0, 0.0, -_OM, 0.328, 3),
]


def ms_for_mm(target_mm: float) -> int:
    """走 `target_mm` 需要多少 ms：`s_mm = V_STEADY·(T_ms − DEAD_MS)` 反解，向下取整。

    向下取整而不是四舍五入：1 ms ≈ 0.56 mm，向上取会让真实行程顶穿上限
    （横移 80 mm 的 285.86 ms 一进位就变 286 → 80.08 mm > 80）。
    """
    if target_mm < 0:
        raise ValueError(f"目标行程必须 ≥ 0，实际 {target_mm}。")
    return int(math.floor(target_mm / V_STEADY_MPS + DEAD_MS))


def ms_for_deg(target_deg: float) -> int:
    """原地转 `target_deg` 需要多少 ms（同一个死区，向下取整）。"""
    return int(math.floor(abs(target_deg) / YAW_RATE_DEG_S * 1000.0 + DEAD_MS))


def _dir_unit(exp) -> tuple:
    """命令方向的单位向量（纯平移）；速度全 0 时返回 (0, 0)。"""
    speed = math.hypot(exp.vx, exp.vy)
    if speed == 0.0:
        return (0.0, 0.0)
    return (exp.vx / speed, exp.vy / speed)


def body_to_field_delta(dx_body, dy_body, yaw_deg=0.0) -> tuple:
    """**车体系**位移 → 场地系位移（mm）。`yaw_deg=0`（车头对齐场地 +X）时 y 取负。

    R69（真机实测）：`(vx, vy)` 是车体系速度，而"车体在不在场地里"是**场地系**的
    问题。真机锚点：#05「横移左移 80 mm」（vy>0）起点 y = −34.9 → 终点 y = −66.3 ——
    车左在场地系里是 **−Y**。符号链与 `harness.body_delta_to_field` 逐字相同
    （`test_matrix` 里有一条测试对拍两者）；这里自己写一遍是因为 `harness` import
    `matrix`，反向 import 会成环。
    """
    th = math.radians(float(yaw_deg))
    c, s = math.cos(th), math.sin(th)
    dx, dy = float(dx_body), float(dy_body)
    return (dx * c - dy * s, -dx * s - dy * c)


def field_delta_mm(exp, yaw_deg=0.0) -> tuple:
    """**场地系**位移 `(dx, dy)`（mm）：期望行程沿命令方向、按 `yaw_deg` 转到场地系。

    `expected_delta_mm` 给的是车体系（命令方向）；场地几何校验与执行起点都要场地系
    那一侧，两者差一个 y 的符号（R69）。
    """
    dx, dy = expected_delta_mm(exp)
    return body_to_field_delta(dx, dy, yaw_deg)


def expected_travel_mm(exp) -> float:
    """这条实验**真机会走**的平移距离（mm）：`V_STEADY·(T−DEAD)`；旋转类返回 0。"""
    if exp.action == "rotate_in_place":
        return 0.0
    return max(0.0, V_STEADY_MPS * (exp.duration_s * 1000.0 - DEAD_MS))


def expected_yaw_deg(exp) -> float:
    """原地旋转**真机会转**的角度（度，符号跟 `om`）；平移类返回 0。"""
    if exp.action != "rotate_in_place":
        return 0.0
    turn = max(0.0, YAW_RATE_DEG_S * (exp.duration_s * 1000.0 - DEAD_MS) / 1000.0)
    return math.copysign(turn, float(exp.om))


def expected_delta_mm(exp) -> tuple:
    """**期望位移** `(dx, dy)`（mm）：沿命令方向、长度 = `expected_travel_mm`。"""
    if exp.action == "rotate_in_place":
        return (0.0, 0.0)
    ux, uy = _dir_unit(exp)
    travel = expected_travel_mm(exp)
    return (ux * travel, uy * travel)


def expected_start_xy(exp, yaw_deg=0.0) -> tuple:
    """把**场地系**期望位移对中后的起点：整段行程落在场地中心两侧各一半。

    ⚠ 对中必须用**场地系**位移（R69）：`expected_delta_mm` 给的是车体系（命令方向），
    车左（`vy>0`）在场地系里是 −Y —— 真机实测 #05「横移左移 80 mm」起点 y = −34.9
    → 终点 y = −66.3。按命令方向取中点会把横移/斜行的起点摆到错的一侧。

    平移就看这一条：起点 = −场地系位移/2 ⇒ 终点 = +场地系位移/2（Task 12 的 `drive_to`
    目标）。旋转原地不动 ⇒ `(0, 0)`。
    """
    if exp.action == "rotate_in_place":
        return (0.0, 0.0)
    dx, dy = field_delta_mm(exp, yaw_deg)
    return (-dx / 2.0, -dy / 2.0)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _check_motion_caps(exp, dx_mm: float, dy_mm: float, dpsi_deg: float) -> None:
    """四类动作各自的上限；纯动作不许混入其他分量（矩阵里没有复合动作）。"""
    if exp.action == "straight":
        _require(exp.vy == 0.0 and exp.om == 0.0,
                 f"实验 #{exp.id}（{exp.desc}）：straight 必须是纯直行（vy=om=0），"
                 f"实际 vy={exp.vy} om={exp.om}。")
        _require(abs(dx_mm) <= MAX_STRAIGHT_MM + _TOL,
                 f"实验 #{exp.id}（{exp.desc}）：直行 {abs(dx_mm):.3f} mm 超过上限 "
                 f"{MAX_STRAIGHT_MM:.0f} mm。")
    elif exp.action == "strafe":
        _require(exp.vx == 0.0 and exp.om == 0.0,
                 f"实验 #{exp.id}（{exp.desc}）：strafe 必须是纯横移（vx=om=0），"
                 f"实际 vx={exp.vx} om={exp.om}。")
        _require(abs(dy_mm) <= MAX_STRAFE_MM + _TOL,
                 f"实验 #{exp.id}（{exp.desc}）：横移 {abs(dy_mm):.3f} mm 超过上限 "
                 f"{MAX_STRAFE_MM:.0f} mm。")
    elif exp.action == "diagonal":
        _require(exp.om == 0.0,
                 f"实验 #{exp.id}（{exp.desc}）：diagonal 不带旋转（om=0），"
                 f"实际 om={exp.om}。")
        _require(math.hypot(dx_mm, dy_mm) <= MAX_DIAGONAL_MM + _TOL,
                 f"实验 #{exp.id}（{exp.desc}）：斜行位移 "
                 f"{math.hypot(dx_mm, dy_mm):.3f} mm 超过上限 {MAX_DIAGONAL_MM:.0f} mm。")
    elif exp.action == "rotate_in_place":
        _require(exp.vx == 0.0 and exp.vy == 0.0,
                 f"实验 #{exp.id}（{exp.desc}）：rotate_in_place 必须原地（vx=vy=0），"
                 f"实际 vx={exp.vx} vy={exp.vy}。")
        _require(abs(dpsi_deg) <= MAX_ROTATION_DEG + _TOL,
                 f"实验 #{exp.id}（{exp.desc}）：旋转 {abs(dpsi_deg):.3f}° 超过上限 "
                 f"{MAX_ROTATION_DEG:.0f}°（转 90° 后车宽 270 mm > 短边 252 mm）。")
    else:
        raise ValueError(
            f"实验 #{exp.id}（{exp.desc}）：未知动作 {exp.action!r}。"
            "矩阵只收 straight / strafe / diagonal / rotate_in_place。")


def assert_within_field(exp, start_xy=(0.0, 0.0)) -> None:
    """校验一条实验在给定起点下可执行；任何一条不过抛 `ValueError`。

    完整口径见模块 docstring。一句话版本：

    - 动作上限永远查（200 / 80 / 120 / 45°）；
    - `start_xy` 为默认 `(0.0, 0.0)`（名义场地中心）时只查"全场可行性"
      （位移 ≤ 235 / 103 mm）—— 规定上限的动作本来就只能从偏心起点做，
      绝对扫掠校验留给下面的显式起点分支；
    - 显式 `start_xy` 时，车体半包络在起点与终点都必须落在
      `±252.5 × ±125.9` 内（旋转类只查起点）。

    位移/转角一律取**期望值**（`expected_delta_mm` / `expected_yaw_deg`，真实会走多少），
    不是命令量 `v·T`：命令量在 MIN_DUTY 以下是惰性的（R68，见模块 docstring）。

    ⚠ 位移还要从**车体系**转到**场地系**（`field_delta_mm`，R69）：`(vx, vy)` 是车体系
    速度，车左（`vy>0`）在场地系里是 −Y。包络的**数值**两侧对称（不改变通过/不通过），
    但"扫掠在哪一侧"必须是真的，否则规划起点与校验会互相打架。
    """
    _require(exp.duration_s > 0,
             f"实验 #{exp.id}（{exp.desc}）：duration_s={exp.duration_s} 必须为正。")

    dx_mm, dy_mm = field_delta_mm(exp)
    dpsi_deg = expected_yaw_deg(exp)
    _check_motion_caps(exp, dx_mm, dy_mm, dpsi_deg)

    x0, y0 = float(start_xy[0]), float(start_xy[1])
    if (x0, y0) == (0.0, 0.0):
        # 名义中心：全场可行性（偏心起点下一定放得下）
        _require(abs(dx_mm) <= FIELD_X_SLACK_MM + _TOL,
                 f"实验 #{exp.id}（{exp.desc}）：x 位移 {abs(dx_mm):.3f} mm 超过全场净余量 "
                 f"{FIELD_X_SLACK_MM:.0f} mm（505-270）。")
        _require(abs(dy_mm) <= FIELD_Y_SLACK_MM + _TOL,
                 f"实验 #{exp.id}（{exp.desc}）：y 位移 {abs(dy_mm):.3f} mm 超过全场净余量 "
                 f"{FIELD_Y_SLACK_MM:.0f} mm（252-149）。")
        return

    # 显式起点：车体半包络（起点、终点都要）落在场地内
    _require(abs(x0) + CAR_HALF_LENGTH_MM <= FIELD_HALF_LENGTH_MM + _TOL,
             f"实验 #{exp.id}（{exp.desc}）：起点 x={x0:.2f} mm + 车体半长 "
             f"{CAR_HALF_LENGTH_MM:.1f} mm 越出场地半长 {FIELD_HALF_LENGTH_MM:.1f} mm"
             f"（车中心 |x| 最多 {FIELD_HALF_LENGTH_MM - CAR_HALF_LENGTH_MM:.1f} mm）。")
    _require(abs(y0) + CAR_HALF_WIDTH_MM <= FIELD_HALF_WIDTH_MM + _TOL,
             f"实验 #{exp.id}（{exp.desc}）：起点 y={y0:.2f} mm + 车体半宽 "
             f"{CAR_HALF_WIDTH_MM:.1f} mm 越出场地半宽 {FIELD_HALF_WIDTH_MM:.1f} mm"
             f"（车中心 |y| 最多 {FIELD_HALF_WIDTH_MM - CAR_HALF_WIDTH_MM:.1f} mm）。")
    if exp.action == "rotate_in_place":
        return

    x1, y1 = x0 + dx_mm, y0 + dy_mm
    _require(abs(x1) + CAR_HALF_LENGTH_MM <= FIELD_HALF_LENGTH_MM + _TOL,
             f"实验 #{exp.id}（{exp.desc}）：终点 x={x1:.2f} mm + 车体半长 "
             f"{CAR_HALF_LENGTH_MM:.1f} mm 越出场地半长 {FIELD_HALF_LENGTH_MM:.1f} mm"
             f"（扫掠包络越界）。换个更靠中心的起点，或缩短行程。")
    _require(abs(y1) + CAR_HALF_WIDTH_MM <= FIELD_HALF_WIDTH_MM + _TOL,
             f"实验 #{exp.id}（{exp.desc}）：终点 y={y1:.2f} mm + 车体半宽 "
             f"{CAR_HALF_WIDTH_MM:.1f} mm 越出场地半宽 {FIELD_HALF_WIDTH_MM:.1f} mm"
             f"（扫掠包络越界）。换个更靠中心的起点，或缩短行程。")
