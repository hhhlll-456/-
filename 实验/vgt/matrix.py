"""实验矩阵（Task 9）+ 场地余量校验（SDD 台账 R15）。

## 矩阵怎么来的

Global Constraints 的四条动作上限，加上 brief 要求的覆盖：

| 动作 | 上限 | 矩阵覆盖 |
|---|---|---|
| 直行 `straight` | 200 mm | ±200、±150 |
| 横移 `strafe` | 80 mm | ±80、±50 |
| 斜行 `diagonal` | 120 mm | 四个对角方向各 120 mm（45°） |
| 原地旋转 `rotate_in_place` | 45° | ±30°、±45° |

直行/横移/斜行统一 1.0 s 行程（速度就是命令值）；旋转用 45°/s（π/4 rad/s）：
30° 走 2/3 s、45° 走 1.0 s。每条 `repeats = 3`（单次抖动用重复的中位数消化）。

## `assert_within_field(exp, start_xy=(0.0, 0.0))` 校验什么

场地原点 = **四边界码质心**（R4），半长 252.5 / 半宽 125.9 mm（R4/R15 的实测值）；
车体半包络 135 × 74.5 mm（270 × 149 / 2）。

**（1）动作上限**（任何分支都查）：上面四个数，超一个 `ValueError`。

**（2）几何**，按起点分两支：

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

# 45° 斜行的每轴速度：|v|·√2 = 0.12 m/s ⇒ 1 s 走 120 mm
_DIAG_V = 0.12 / math.sqrt(2.0)
_OM = math.pi / 4.0              # 45°/s

EXPERIMENTS = [
    # ---- 直行：±200 / ±150（上限 200）----
    Experiment(1, "straight", "直行前进 200 mm", 0.200, 0.0, 0.0, 1.0, 3),
    Experiment(2, "straight", "直行后退 200 mm", -0.200, 0.0, 0.0, 1.0, 3),
    Experiment(3, "straight", "直行前进 150 mm", 0.150, 0.0, 0.0, 1.0, 3),
    Experiment(4, "straight", "直行后退 150 mm", -0.150, 0.0, 0.0, 1.0, 3),
    # ---- 横移：±80 / ±50（步长上限 80）----
    Experiment(5, "strafe", "横移左移 80 mm", 0.0, 0.080, 0.0, 1.0, 3),
    Experiment(6, "strafe", "横移右移 80 mm", 0.0, -0.080, 0.0, 1.0, 3),
    Experiment(7, "strafe", "横移左移 50 mm", 0.0, 0.050, 0.0, 1.0, 3),
    Experiment(8, "strafe", "横移右移 50 mm", 0.0, -0.050, 0.0, 1.0, 3),
    # ---- 斜行：四个对角方向，各 120 mm（上限 120）----
    Experiment(9, "diagonal", "斜行前左 45° 120 mm", _DIAG_V, _DIAG_V, 0.0, 1.0, 3),
    Experiment(10, "diagonal", "斜行前右 45° 120 mm", _DIAG_V, -_DIAG_V, 0.0, 1.0, 3),
    Experiment(11, "diagonal", "斜行后左 45° 120 mm", -_DIAG_V, _DIAG_V, 0.0, 1.0, 3),
    Experiment(12, "diagonal", "斜行后右 45° 120 mm", -_DIAG_V, -_DIAG_V, 0.0, 1.0, 3),
    # ---- 原地旋转：±30° / ±45°（上限 45°，45°/s）----
    Experiment(13, "rotate_in_place", "原地逆时针 30°", 0.0, 0.0, _OM, 2.0 / 3.0, 3),
    Experiment(14, "rotate_in_place", "原地顺时针 30°", 0.0, 0.0, -_OM, 2.0 / 3.0, 3),
    Experiment(15, "rotate_in_place", "原地逆时针 45°", 0.0, 0.0, _OM, 1.0, 3),
    Experiment(16, "rotate_in_place", "原地顺时针 45°", 0.0, 0.0, -_OM, 1.0, 3),
]


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
    """
    _require(exp.duration_s > 0,
             f"实验 #{exp.id}（{exp.desc}）：duration_s={exp.duration_s} 必须为正。")

    dx_mm = exp.vx * exp.duration_s * 1000.0
    dy_mm = exp.vy * exp.duration_s * 1000.0
    dpsi_deg = exp.om * exp.duration_s * 180.0 / math.pi
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
