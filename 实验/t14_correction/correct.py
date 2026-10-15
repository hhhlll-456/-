"""Task 14：小车**修正公式**的可执行实现。

配套说明：`小车修正公式.md`（仓库根）。
常数由 `实验/t14_correction/analyze.py` 从**两次真机会话**的数据拟合而来
（2026-10-15 补实验 87 条 + 更早会话 62 条汇总行），**没有一个是估计值**。

**本模块只算数、只生成命令行字符串，不连车、不发命令。**

## 公式

固件在 `RUN` 段把四个轮子全部钳到占空比 110（实测 RUN 段 4848/4848 个采样
`|pwm| = 110`）⇒ **命令速度的"大小"是惰性的，行程只由"时长"决定**：

    正向：行程 s = V·(T − t_d)          # T = 下发的毫秒数
    反解：T = s / V + t_d              # 这就是修正公式

`V` 与 `t_d` **按通道（纵向/横向/旋转/斜行）× 方向分别取**。

## 三项"非主项"因素（都不是噪声，都是可算的）

1. **方向锁定的航向偏差** —— 见 `YAW_BIAS_DEG`。前进恒往一边歪、后退恒往另一边，
   符号 100% 一致，是确定性偏差。
2. **`om` 的死区与悬崖** —— 见 `om_limit_straight()`。想让直行时用 `om` 主动掰方向
   **做不到**：`om` 在 [0, 1.48] 内对结果毫无影响，过了 1.48 车直接散架（见下）。
3. **会话间漂移** —— 同一命令、同一时长，两次会话能差 15%（横移最明显）。
   本模块取"时长档数最多的那次会话"；两次数值都在 `analyze.py` 的表 1 里。

## 能承诺的精度（开环天花板 = 同一条命令跑两遍的散布）

| 动作 | 典型（同会话 σ） | 最差（观测极差） |
|---|---|---|
| 直行前进 / 后退 | ±1.4 / **±3.4 mm** | 2.8 / 8.6 mm |
| 横移左 / 右 | **±0.3–1.2 / ±0.4–2.4 mm** | 1.7 / 3.3 mm |
| 原地旋转 | ±1.1 / ±1.8 ° | 6.5 / 4.6 ° |
| 斜行（45°） | ±2.6–4.6 mm | — |

**"绝对不歪"超出开环能力** —— 直行每组内部仍有 0.7–0.9° 的随机散布。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# --------------------------------------------------------------------------
# 1) 标定常数（来源：实验/t14_correction/analyze.py 的输出）
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Channel:
    """一个「通道 × 方向」的实测常数。单位：mm/s（平移/斜行）或 °/s（旋转）。"""
    V: float             # 速度（地板 duty=110 下）
    t_d_ms: float        # 起转死区
    n: int               # 样本数
    r2: float            # 拟合优度
    sigma: float         # 拟合残差标准差（与该通道同单位）
    source: str          # 取自哪次会话（口径见 analyze.py）
    unit: str = "mm"

    @property
    def unit_speed(self) -> str:
        return f"{self.unit}/s"


# `source` 的两个取值：
#   "合并"        —— 两次会话都在用（纵向：两次的时长档互补且互相吻合，差 <2%）
#   "2026-10-15" —— 只用本次补实验（这次有 3–7 个时长档；更早那次只有 2 档，
#                   分不开 V 和 t_d，其值在 analyze.py 表 1 里作对照）
CHANNELS: dict[tuple[str, str], Channel] = {
    ("纵向", "前进"): Channel(574.9, 135.9, 22, 0.9949, 4.80, "合并"),
    ("纵向", "后退"): Channel(600.9, 140.5, 26, 0.9926, 5.77, "合并"),
    ("横向", "左移"): Channel(233.4, 100.7, 9, 0.9677, 1.93, "2026-10-15"),
    ("横向", "右移"): Channel(207.0, 93.2, 10, 0.9863, 1.45, "2026-10-15"),
    ("旋转", "逆时针"): Channel(217.4, 131.5, 8, 0.9953, 1.10, "2026-10-15", unit="°"),
    ("旋转", "顺时针"): Channel(205.6, 122.8, 10, 0.9865, 1.78, "2026-10-15", unit="°"),
    # 斜行：`vx`、`vy` 同时非零 ⇒ 四个轮子只有**一对**出力，走的是另一套运动学，
    # 速度既不是纵向常数也不是横向常数，必须单独标定。
    # 两"轴"对应不同的轮子对，实测速度不同（轴 A 快 ~8%）。
    ("斜行", "轴A"): Channel(209.4, 127.3, 9, 0.8269, 4.55, "2026-10-15"),
    ("斜行", "轴B"): Channel(181.9, 147.8, 6, 0.9450, 2.63, "2026-10-15"),
}

# 斜行的四个方向名 → 轴。轴 A 驱动 FR+RL 一对，轴 B 驱动 FL+RR 一对。
DIAGONAL_AXIS = {"前左": "轴A", "后右": "轴A", "前右": "轴B", "后左": "轴B"}

# 更早那次会话（只有 2 个时长档）的对照值，**不要拿去做定量修正**，只用来量"会话间漂移"。
# 格式：(通道, 方向) -> (V, t_d_ms, n, R²)
EARLIER_SESSION = {
    ("纵向", "前进"): (582.9, 140.4, 11, 0.9528),
    ("纵向", "后退"): (653.7, 165.1, 10, 0.9538),
    ("横向", "左移"): (196.6, 97.1, 8, 0.5100),
    ("横向", "右移"): (267.7, 131.3, 6, 0.9374),
    ("旋转", "逆时针"): (189.5, 121.1, 9, 0.9433),
    ("旋转", "顺时针"): (102.5, -14.3, 6, 0.7029),
}


# --------------------------------------------------------------------------
# 2) 走歪：方向锁定的确定性偏差
# --------------------------------------------------------------------------

# 每次「纯平移/直行」跑完，车头实际转过的角度（有符号，真机中位）。
# 符号在四组里 100% 一致 ⇒ 确定性偏差，不是随机摇晃。
YAW_BIAS_DEG: dict[tuple[str, str], float] = {
    ("纵向", "前进"): -1.60,   # n=22，0 正 / 22 负
    ("纵向", "后退"): +1.97,   # n=26，26 正 / 0 负
    ("横向", "左移"): +6.47,   # n=17，17 正 / 0 负
    ("横向", "右移"): -6.40,   # n=16，0 正 / 16 负
}

# 组内残差标准差（去掉该组中位之后剩下的散布）—— 这是"补掉系统偏差之后还剩多少"。
YAW_BIAS_SPREAD_DEG: dict[tuple[str, str], float] = {
    ("纵向", "前进"): 0.94,
    ("纵向", "后退"): 0.70,
    ("横向", "左移"): 0.98,
    ("横向", "右移"): 1.48,
}


# --------------------------------------------------------------------------
# 3) `om` 的死区与悬崖 —— 从固件源码推出来的，并经真机验证到 ±0.02
# --------------------------------------------------------------------------
#
# 固件 `bodyToPWM`（`固件/exp/src/main.cpp`）：
#     w[0] = (vx − vy − LXY·om)/R ... 四个轮子
# 然后 `wheelOmegaToPWM`：
#   * `if (abs_w > 0.3f)` 才给 duty —— 即 **|ω| ≤ 0.3 rad/s 的轮子直接停转**；
#   * 低于查表首行（duty=110 对应 ω≈12.9–15.0）时 `interpDuty` 线性外推，
#     算出的 duty 必然 < 110，于是被 `MIN_DUTY` 钳回 110。
#
# 两条合起来 ⇒ 四个轮子的 duty **全等于 110**，只要每个轮的 |ω| 落在 (0.3, 表首行]。
# 在这个区间里改 `vx`/`vy`/`om` 的**大小**对车的行为毫无影响。
R_WHEEL_M = 0.0335          # 固件 R_WHEEL
LXY_M = 0.12825             # 固件 LXY
OM_DEAD_OMEGA = 0.3         # 固件里 |ω| ≤ 0.3 的轮子不转
TABLE_FIRST_OMEGA = 12.98   # 最严的一列（FR_FWD 表首行 ω）

# 真机验证（`vx=0.2` 固定，改 `om`，240 ms）：
#   om=0.00 → 57.83 mm   om=1.00 → 59.62 mm   om=1.45 → 59.64 mm   ← 全是同一条直行
#   om=1.49 → 19.25 mm + 转 8.68°   om=1.90 → 14.42 mm + 转 23.84° ← 已经散架
# 理论拐点 (0.2 − 0.3·0.0335)/0.12825 = 1.4811，实测落在 (1.45, 1.49] 之间。
OM_KNEE_MEASURED = dict(lo=1.45, hi=1.49, vx=0.2)


def om_limit_straight(vx: float) -> float:
    """直行（`vy=0`）时，`om` 最大只能给到多少**而不破坏四个轮子同时出力**。

    两头的约束取更严的那个：

    * 内侧轮 ω 掉到 0.3 以下 → 那个轮子直接停转（车从"四轮直行"掉成"两轮斜行"）；
    * 外侧轮 ω 顶过查表首行 → 那一轮开始加速（车开始带角速度）。

    返回值 <= 0 表示这条命令**任何** `om` 都会破坏直行。

    ⚠ 注意：在 (0, 该值] 区间内给 `om` **不是"给一点能掰一点点"** —— 是**毫无影响**。
    所以想用 `om` 做"走歪补偿"在本固件上是行不通的：死区里没作用，出了死区就是悬崖。
    """
    v = abs(float(vx))
    lim_rev = (v - OM_DEAD_OMEGA * R_WHEEL_M) / LXY_M          # 内侧轮失速
    lim_acc = (TABLE_FIRST_OMEGA * R_WHEEL_M - v) / LXY_M      # 外侧轮越过地板
    return min(lim_rev, lim_acc)


# --------------------------------------------------------------------------
# 4) 场地/车体边界（这个场地里车的活动余量非常小）
# --------------------------------------------------------------------------
FIELD_HALF_LENGTH_MM = 252.5
FIELD_HALF_WIDTH_MM = 125.9
CAR_HALF_LENGTH_MM = 135.0
CAR_HALF_WIDTH_MM = 74.5
X_LIMIT_MM = FIELD_HALF_LENGTH_MM - CAR_HALF_LENGTH_MM     # ±117.5
Y_LIMIT_MM = FIELD_HALF_WIDTH_MM - CAR_HALF_WIDTH_MM       # ±51.4


# --------------------------------------------------------------------------
# 5) 修正公式本体
# --------------------------------------------------------------------------


def _resolve(channel: str, direction: str) -> tuple[str, str]:
    """把「斜行 前左」这类方向名映射到常数表的键。"""
    if channel == "斜行" and direction in DIAGONAL_AXIS:
        return "斜行", DIAGONAL_AXIS[direction]
    return channel, direction


def duration_ms(channel: str, direction: str, amount: float) -> int:
    """**修正公式**：要走 `amount`（mm 或 度），下发多少毫秒。

        T = |amount| / V + t_d      （再向上取整到 1 ms）

    斜行的 `amount` 是**沿 45° 的总位移**（不是某一轴的分量）。
    """
    ch = CHANNELS[_resolve(channel, direction)]
    return int(math.ceil(abs(float(amount)) / ch.V * 1000.0 + ch.t_d_ms))


def travel(channel: str, direction: str, T_ms: float) -> float:
    """**正向模型**：下发 `T_ms` 会走多少（mm 或 度）。`T < t_d` 时取 0。"""
    ch = CHANNELS[_resolve(channel, direction)]
    return ch.V * max(0.0, (float(T_ms) - ch.t_d_ms)) / 1000.0


def _split(dx_mm: float, dy_mm: float, dpsi_deg: float):
    """把车体系目标 (前后, 左右, 转角) 拆成 (通道, 方向, 量) 三元组。"""
    parts = []
    if abs(dx_mm) > 1e-9:
        parts.append(("纵向", "前进" if dx_mm > 0 else "后退", abs(dx_mm)))
    if abs(dy_mm) > 1e-9:
        parts.append(("横向", "左移" if dy_mm > 0 else "右移", abs(dy_mm)))
    if abs(dpsi_deg) > 1e-9:
        parts.append(("旋转", "逆时针" if dpsi_deg > 0 else "顺时针", abs(dpsi_deg)))
    return parts


def plan(dx_mm: float = 0.0, dy_mm: float = 0.0, dpsi_deg: float = 0.0) -> dict:
    """**给出修正后的执行方案**（不连车、不发命令）。

    参数是**车体系**目标：`dx_mm` 前为正、`dy_mm` 左为正、`dpsi_deg` 逆时针为正。

    ⚠ 平移与旋转**不要塞进同一条 `RUN`**：一次命令里 `vx`/`vy`/`om` 全非零时，
    四个轮子的合成占空比怎么落到地板上是未知的。分开做。
    """
    parts = _split(dx_mm, dy_mm, dpsi_deg)
    if not parts:
        return {"steps": [], "warnings": [], "note": "目标全为 0，无需动作。"}

    steps = []
    for channel, direction, amount in parts:
        ch = CHANNELS[_resolve(channel, direction)]
        T = duration_ms(channel, direction, amount)
        steps.append(dict(
            channel=channel, direction=direction, requested=round(amount, 2),
            T_ms=T, predicted=round(travel(channel, direction, T), 2), unit=ch.unit,
            sigma=ch.sigma, r2=ch.r2, n=ch.n, source=ch.source,
            yaw_bias_deg=YAW_BIAS_DEG.get((channel, direction)),
            yaw_spread_deg=YAW_BIAS_SPREAD_DEG.get((channel, direction)),
        ))

    warn = []
    for s in steps:
        if s["yaw_bias_deg"] is not None:
            warn.append(
                f"{s['channel']}{s['direction']}：这条跑完车头会偏 "
                f"{s['yaw_bias_deg']:+.2f}°（组内散布 ±{s['yaw_spread_deg']:.2f}°）"
                f" —— 开环补不掉，见 YAW_BIAS_DEG。")
    y_step = [s for s in steps if s["channel"] == "横向"]
    if y_step and abs(y_step[0]["requested"]) > Y_LIMIT_MM:
        warn.append(f"横移 {y_step[0]['requested']} mm 超过车中心可用半宽 ±{Y_LIMIT_MM:.1f} mm"
                    f"（y 方向全场只有 {2*Y_LIMIT_MM:.0f} mm 可用）—— 一趟做不到，得分几步。")
    x_step = [s for s in steps if s["channel"] == "纵向"]
    if x_step and abs(x_step[0]["requested"]) > X_LIMIT_MM:
        warn.append(f"直行 {x_step[0]['requested']} mm 超过车中心可用半长 ±{X_LIMIT_MM:.1f} mm"
                    f"（x 方向全场只有 {2*X_LIMIT_MM:.0f} mm 可用）—— 一趟做不到。")

    return dict(steps=steps, warnings=warn,
                x_limit_mm=round(X_LIMIT_MM, 1), y_limit_mm=round(Y_LIMIT_MM, 1))


def command_line(vx: float = 0.0, vy: float = 0.0, om: float = 0.0, T_ms: int = 0) -> str:
    """拼一条 `RUN` 命令（**只拼字符串，不发**）。"""
    return f"RUN {vx:g} {vy:g} {om:g} {int(T_ms)}"


def diagonal_commands(direction: str, total_mm: float) -> dict:
    """斜行：给一个 45° 总位移，返回 (vx, vy, T_ms)。"""
    if direction not in DIAGONAL_AXIS:
        raise KeyError(f"斜行方向只能是 {sorted(DIAGONAL_AXIS)}，收到 {direction!r}")
    T = duration_ms("斜行", direction, total_mm)
    sgn_x = 1.0 if direction.startswith("前") else -1.0
    sgn_y = 1.0 if direction.endswith("左") else -1.0
    # 命令幅值本身是惰性的（占空比钳位），取一个能让四个轮子里"该转的那一对"
    # 稳稳高于 0.3 rad/s 的值即可；0.2 与其它通道一致，也经实测。
    return dict(direction=direction, T_ms=T, vx=0.2 * sgn_x, vy=0.2 * sgn_y,
                predicted_total_mm=round(travel("斜行", direction, T), 2),
                axis=DIAGONAL_AXIS[direction])


# --------------------------------------------------------------------------
# 6) 自检：拿真机行对拍（不连车）
# --------------------------------------------------------------------------


def _selftest() -> int:
    import json
    import statistics as stx
    from pathlib import Path
    base = Path(__file__).resolve().parent
    camp = base / "data" / "campaign.jsonl"
    if not camp.exists():
        print(f"{camp} 不在，跳过对拍（先跑 campaign.py 补实验）")
        return 0
    rows = [json.loads(l) for l in camp.read_text().splitlines() if l.strip()]

    def sel(pred):
        return [r for r in rows if pred(r) and r["dist_mm"] is not None]

    jobs = [
        ("直行前进", lambda r: r["vx"] > 0 and r["vy"] == 0 and r["om"] == 0,
         "纵向", "前进", False),
        ("直行后退", lambda r: r["vx"] < 0 and r["vy"] == 0 and r["om"] == 0,
         "纵向", "后退", False),
        ("横移左移", lambda r: r["vy"] > 0 and r["vx"] == 0 and r["om"] == 0,
         "横向", "左移", False),
        ("横移右移", lambda r: r["vy"] < 0 and r["vx"] == 0 and r["om"] == 0,
         "横向", "右移", False),
        ("旋转逆时针", lambda r: r["om"] > 0 and r["vx"] == 0 and r["vy"] == 0,
         "旋转", "逆时针", True),
        ("旋转顺时针", lambda r: r["om"] < 0 and r["vx"] == 0 and r["vy"] == 0,
         "旋转", "顺时针", True),
    ]
    print(f"{'组':12s}{'n':>3s}{'残差均值':>10s}{'残差RMS':>10s}{'最大残差':>10s}{'σ(拟合)':>10s}")
    bad = 0
    for name, pred, ch, dr, use_yaw in jobs:
        rs = sel(pred)
        # **逐行**残差：同一条记录上「公式预测」减「实测」。跨时长取平均会和实测的
        # 时长结构对不上，得出假偏差（第一版就踩了这个坑）。
        res = []
        for r in rs:
            want = abs(r["yaw_deg"]) if use_yaw else r["dist_mm"]
            res.append(travel(ch, dr, r["ms"]) - want)
        unit = "°" if use_yaw else "mm"
        b = stx.fmean(res)
        rms = math.sqrt(stx.fmean([x * x for x in res]))
        mx = max(abs(x) for x in res)
        sig = CHANNELS[(_resolve(ch, dr))[0], (_resolve(ch, dr))[1]].sigma
        # 判据：偏置（系统项）不超过该通道的 σ。RMS 里含随机散布，不该拿 σ 去卡。
        flag = "" if abs(b) <= sig else f"  ← 偏置超过 σ={sig:.2f}{unit}"
        bad += 0 if abs(b) <= sig else 1
        print(f"{name:12s}{len(rs):3d}{b:+9.2f}{rms:10.2f}{mx:10.2f}{sig:10.2f} {unit}{flag}")

    print(f"\n公式 vs 真机（逐行残差）：{len(jobs)-bad}/{len(jobs)} 组无系统偏置。")
    print("注：常数就是在这批数据上拟合的，此处分不到 σ 以内属正常；真正可信度看各组的 R²。")
    print("\nom 死区自检（vx=0.2）：")
    lim = om_limit_straight(0.2)
    print(f"  理论安全上限 om ≤ {lim:.4f}（实测拐点落在 "
          f"({OM_KNEE_MEASURED['lo']}, {OM_KNEE_MEASURED['hi']}]）")
    return 0


if __name__ == "__main__":
    import sys
    if "--plan" in sys.argv:
        i = sys.argv.index("--plan")
        a = [float(x) for x in sys.argv[i + 1:i + 4]]
        while len(a) < 3:
            a.append(0.0)
        p = plan(*a)
        for s in p["steps"]:
            print(f"  {s['channel']}{s['direction']}: 目标 {s['requested']} {s['unit']} "
                  f"⇒ 下发 {s['T_ms']} ms（预测 {s['predicted']} {s['unit']}，"
                  f"σ=±{s['sigma']}，n={s['n']}，R²={s['r2']}，源={s['source']}）")
        for w in p["warnings"]:
            print(f"  ⚠ {w}")
    else:
        raise SystemExit(_selftest())
