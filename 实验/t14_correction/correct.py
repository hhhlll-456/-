"""Task 14：小车**修正公式**的可执行实现。

配套说明：`小车修正公式.md`（仓库根）。常数由 `实验/t14_correction/fit.py` 从
`实验/实验数据记录.xlsx` 的 62 条真机汇总行拟合而来，**不是估计值**。

**本模块只算数、只生成命令行字符串，不连车、不发命令。** 要用请配合
`实验/car_link.py`，并且先确认车在开阔处。

## 公式（一句话）

固件在 `RUN` 段把**四个轮子全部钳到占空比 110**（独立复算：RUN 段 4848/4848 个采样
`|pwm| = 110`）⇒ **命令速度的"大小"是惰性的，行程只由"时长"决定**。所以：

    正向：行程 s = V·(T − t_d)          # T = 下发的毫秒数
    反解：T = s / V + t_d              # 这就是修正公式

`V` 与 `t_d` **按通道（纵向/横向/旋转）× 方向（前/后、左/右、逆/顺）分别取**，
因为实测方向不对称：后退比前进快 12%、右移比左移快 36%、逆时针是顺时针的 1.85 倍。

## 能承诺的精度（模型自身的重复性，σ 与 |误差| 均来自真机）

| 动作 | 典型（中位） | 最差（max） |
|---|---|---|
| 直行前进 / 后退 | ±5 mm | ±11 mm |
| 横移左 / 右 | ±3 / ±1 mm | ±9 / ±3 mm |
| 原地旋转 | ±0.8° | ±3.7° |

**开环到此为止。** "绝对不歪"超出开环能力 —— 见 `YAW_BIAS` 的说明。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# 1) 标定常数（来源：实验/t14_correction/fit.py 的输出，口径写在 fit.py 顶部）
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Channel:
    """一个「通道 × 方向」的实测常数。单位：mm/s（平移）或 °/s（旋转）。"""
    V: float             # 速度（地板 duty=110 下）
    t_d_ms: float        # 起转死区
    n: int               # 样本数
    r2: float            # 拟合优度（只有两个时长档 ⇒ R² 是唯一可信度指标）
    resid_med: float     # 模型 |误差| 中位
    resid_max: float     # 模型 |误差| max
    unit: str = "mm"

    @property
    def unit_speed(self) -> str:
        return f"{self.unit}/s"


CHANNELS: dict[tuple[str, str], Channel] = {
    ("纵向", "前进"): Channel(582.88, 140.4, 11, 0.9528, 5.17, 10.78),
    ("纵向", "后退"): Channel(653.66, 165.1, 10, 0.9538, 4.99, 10.90),
    ("横向", "左移"): Channel(196.57, 97.1, 8, 0.5100, 3.02, 8.69),
    ("横向", "右移"): Channel(267.70, 131.3, 6, 0.9374, 1.09, 3.41),
    ("旋转", "逆时针"): Channel(189.47, 121.1, 9, 0.9433, 0.83, 2.56, unit="°"),
    # ⚠ 顺时针 R²=0.70 且 t_d 为负（−14 ms）⇒ 该组只能当「同向不等速」用，
    #   别拿它做定量修正。它的存在本身是结论：顺/逆不是一回事（102 vs 189 °/s）。
    ("旋转", "顺时针"): Channel(102.50, -14.3, 6, 0.7029, 0.79, 3.71, unit="°"),
}

# 斜行：raw 数据只有**单一时长档**（357 ms），V 与 t_d 分离不出来 ⇒ 只能给单档常数。
# 想按公式修正斜行，需要补一次多档时长的斜行实验。
DIAGONAL_AT_357MS = {
    "前左": dict(dist_mm=53.45, rng=(52.41, 55.01), yaw_deg=-0.12, n=3),
    "前右": dict(dist_mm=37.74, rng=(31.89, 41.88), yaw_deg=+0.99, n=3),
    "后左": dict(dist_mm=43.05, rng=(40.18, 50.45), yaw_deg=+0.75, n=3),
    "后右": dict(dist_mm=44.33, rng=(41.75, 48.25), yaw_deg=+0.58, n=3),
}

# --------------------------------------------------------------------------
# 2) 走歪（偏航偏差）—— 方向锁定的确定性偏差
# --------------------------------------------------------------------------

# 每组「该走多直的时候实际歪了多少度」（有符号，真机中位）。
# **符号在四组里 100% 一致**（前进 0 正 11 负、后退 10 正 0 负、左移 8 正 0 负、
# 右移 0 正 6 负）⇒ 这是**确定性偏差**，不是随机噪声。混前进后退一起统计会
# 把它平均成 ≈0（E8/E15 就是这么写的），按方向分开才看得见。
YAW_BIAS_DEG: dict[tuple[str, str], float] = {
    ("纵向", "前进"): -1.47,
    ("纵向", "后退"): +1.84,
    ("横向", "左移"): +6.37,
    ("横向", "右移"): -6.36,
}

# ⚠⚠ 下面这个开关**故意关着**，别随手打开。
#
# 想「补掉」这个偏差，得在下发 `RUN` 时带一个非零的 `om`（例如直行前进时给 om>0
# 反向掰）。可是**本批 84 条里没有一条是「平移 + om≠0」** —— 命令矩阵的 16 条
# 要么纯平移（om=0）、要么纯旋转（vx=vy=0）。所以"给多少 om 能抵消多少度歪"
# 这个**增益完全没有实测**。此外 RUN 段四个轮子都被钳在 110，给 om 是让两个轮
# 子顶到地板以上、另两个轮子仍在地板 —— 增益是否线性、是否可加，都未验证。
#
# 打开它就得先补一次「直行 + 若干个 om 档」的标定实验（约 10–15 次跑车）。
YAW_COMP_GAIN_CALIBRATED = False


# --------------------------------------------------------------------------
# 3) 场地/车体边界（提醒：这个场地里车的活动余量非常小）
# --------------------------------------------------------------------------
FIELD_HALF_LENGTH_MM = 252.5
FIELD_HALF_WIDTH_MM = 125.9
CAR_HALF_LENGTH_MM = 135.0
CAR_HALF_WIDTH_MM = 74.5
# 车中心能落在的范围
X_LIMIT_MM = FIELD_HALF_LENGTH_MM - CAR_HALF_LENGTH_MM     # ±117.5
Y_LIMIT_MM = FIELD_HALF_WIDTH_MM - CAR_HALF_WIDTH_MM       # ±51.4


# --------------------------------------------------------------------------
# 4) 修正公式本体
# --------------------------------------------------------------------------


def duration_ms(channel: str, direction: str, amount: float) -> int:
    """**修正公式**：要走 `amount`（mm 或 度），下发多少毫秒。

        T = |amount| / V + t_d      （再向上取整到 1 ms）

    向上取整（不是向下）：1 ms ≈ 0.58 mm，向上只会略微走过头，而"走不够"
    在闭环里要再补一步 —— 这里取"宁多勿少"。
    """
    ch = CHANNELS[(channel, direction)]
    amt = abs(float(amount))
    return int(math.ceil(amt / ch.V * 1000.0 + ch.t_d_ms))


def travel(channel: str, direction: str, T_ms: float) -> float:
    """**正向模型**：下发 `T_ms` 会走多少（mm 或 度）。负的 T-t_d 取 0。"""
    ch = CHANNELS[(channel, direction)]
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

    参数是**车体系**目标：`dx_mm` 前为正、`dy_mm` 左为正、`dpsi_deg` 逆时针为正
    （与固件 `RUN vx vy om` 的符号一致，见 `实验/vgt/matrix.py` 的 R69 注解）。

    返回每个通道的时长、预测到达量、以及**预测误差**（= 该通道的实测重复性）。

    注意：平移与旋转在 `RUN` 里是**同一条命令**（`RUN vx vy om ms`），
    而实测常数的单位不同（平移 mm、旋转度）。**混着发没有实测支撑** ——
    一次 `RUN` 里 `vx`/`vy`/`om` 全非零时，四个轮子的合成占空比怎么落到地板上
    是未知的。本函数给出各通道**分开做**的方案；要合并请在开阔处先试一条。
    """
    parts = _split(dx_mm, dy_mm, dpsi_deg)
    if not parts:
        return {"steps": [], "note": "目标全为 0，无需动作。"}

    steps = []
    for channel, direction, amount in parts:
        ch = CHANNELS[(channel, direction)]
        T = duration_ms(channel, direction, amount)
        pred = travel(channel, direction, T)
        steps.append(dict(
            channel=channel, direction=direction, requested=round(amount, 2),
            T_ms=T, predicted=round(pred, 2), unit=ch.unit,
            err_median=ch.resid_med, err_max=ch.resid_max,
            r2=ch.r2, n=ch.n,
            yaw_bias_deg=YAW_BIAS_DEG.get((channel, direction)),
        ))

    warn = []
    total_x = sum(s["requested"] * (1 if s["direction"] == "前进" else -1)
                  for s in steps if s["channel"] == "纵向")
    total_y = sum(s["requested"] * (1 if s["direction"] == "左移" else -1)
                  for s in steps if s["channel"] == "横向")
    if abs(total_x) > 2 * X_LIMIT_MM:
        warn.append(f"x 方向 {total_x:.1f} mm 超过车中心可用范围 ±{X_LIMIT_MM:.1f} mm "
                    f"（即 {2*X_LIMIT_MM:.0f} mm 是场地里 x 的**物理上限**）")
    if abs(total_y) > 2 * Y_LIMIT_MM:
        warn.append(f"y 方向 {total_y:.1f} mm 超过车中心可用范围 ±{Y_LIMIT_MM:.1f} mm "
                    f"（{2*Y_LIMIT_MM:.0f} mm 是场地里 y 的物理上限）")
    if not YAW_COMP_GAIN_CALIBRATED:
        warn.append("偏航偏差（走歪）**未做补偿**：补偿增益没有实测数据，见模块说明。")

    return dict(steps=steps, warnings=warn,
                x_limit_mm=round(X_LIMIT_MM, 1), y_limit_mm=round(Y_LIMIT_MM, 1))


def command_line(dx_mm: float = 0.0, dy_mm: float = 0.0, om: float = 0.0,
                 T_ms: int = 0) -> str:
    """拼一条 `RUN` 命令（**只拼字符串，不发**）。"""
    return f"RUN {dx_mm:g} {dy_mm:g} {om:g} {int(T_ms)}"


# --------------------------------------------------------------------------
# 5) 自检：把本模块的常数与真机行对拍（不连车）
# --------------------------------------------------------------------------


def _selftest() -> int:
    import json
    from pathlib import Path
    ds = Path(__file__).resolve().parents[2] / "实验" / "t12_stats" / "dataset.json"
    if not ds.exists():
        print("dataset.json 不在，跳过对拍（先跑 实验/t12_stats/build.py）")
        return 0
    D = json.load(open(ds))
    OK = [d for d in D if str(d["trial_ok"]).strip().upper() in ("TRUE", "1", "OK")
          and d["vision_dist_mm"] is not None]

    check = [
        ("直行前进", "纵向", "前进", "vision_dist_mm", 1.0, "mm"),
        ("直行后退", "纵向", "后退", "vision_dist_mm", 1.0, "mm"),
        ("横移左移", "横向", "左移", "vision_dist_mm", 1.0, "mm"),
        ("横移右移", "横向", "右移", "vision_dist_mm", 1.0, "mm"),
        ("旋转逆时针", "旋转", "逆时针", "vision_yaw_deg", -1.0, "°"),
        ("旋转顺时针", "旋转", "顺时针", "vision_yaw_deg", -1.0, "°"),
    ]
    print(f"{'组':10s}{'n':>3s}{'用公式从实际 T 预测':>20s}{'实测中位':>12s}{'偏差':>10s}")
    bad = 0
    for name, ch, dr, col, sign, unit in check:
        sel = [d for d in OK if name[2:] in str(d["command_description"])]
        got = [abs(d[col]) if col.endswith("yaw_deg") else d[col] for d in sel]
        pred = [travel(ch, dr, d["duration_planned"] * 1000.0) for d in sel]
        mp, mg = sum(pred) / len(pred), sum(got) / len(got)
        d = mp - mg
        flag = "" if abs(d) <= 1.0 else "  ← 偏"
        if abs(d) > 1.0:
            bad += 1
        print(f"{name:10s}{len(sel):3d}{mp:20.2f}{mg:12.2f}{d:+10.2f} {unit}{flag}")
    print(f"\n公式 vs 真机：{len(check)-bad}/{len(check)} 组的均值偏差 ≤1.0（单位见列）。")
    print("注：常数是**在这批数据上**拟合的，这里的均值必然吻合；真正的可信度看")
    print("    各组的 R² 与档内 σ（见 实验/t14_correction/fit.py 的输出）。")
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
                  f"重复性 ±{s['err_median']}，最差 ±{s['err_max']}，n={s['n']} R²={s['r2']}）")
        for w in p["warnings"]:
            print(f"  ⚠ {w}")
    else:
        raise SystemExit(_selftest())
