"""Task 14：从 Task 12 的 84 条真机行拟合「修正公式」所需的全部常数，并打印成表。

> ⚠ **本脚本已被 `analyze.py` 取代（2026-10-15）。** 它只吃更早会话那 62 条，而那一批
> **每组只有 2 个时长档** ⇒ V 与 t_d 分不开（横移左 R²=0.510、顺时针 R²=0.703 算出
> `t_d = −14 ms` 这种负数）。`analyze.py` 把 2026-10-15 补实验的 87 条一并纳入，
> 每组 3–7 档时长，**定量修正请用 `analyze.py` / `correct.py`**。
>
> 本文件保留，作为 §8.1 那组 2 档常数与 `error-mece.md` E8/E15 的历史复算依据。

**本脚本只读数据、只算数，不连车、不开相机。**

## 口径（与 `小车档案.md` §8.1 / `error-mece.md` 逐字一致）

- 数据源：`实验/实验数据记录.xlsx` sheet `data` 的**汇总行**（`trial_ok` 非空的那 84 行），
  经 `实验/t12_stats/build.py` 抽成 `dataset.json`；
- 样本：`trial_ok ∈ {TRUE,1,OK}` 且 `vision_dist_mm` 有值 → **62 条**；
- 分组：按「动作 + 方向」（不是按实验号）；
- 模型：`d = V·(T − t_d)`，最小二乘拟合 `d = a·T + b` ⇒ `V = a`、`t_d = −b/a`；
- 距离取 `vision_dist_mm`，角度取 `|vision_yaw_deg|`；`T` 取 `duration_planned`。

## 为什么要按方向分组（本轮复算的新发现）

把直行前进与后退**混在一起**统计时，偏航的有符号中位 ≈ 0，看起来"左右各半"。
按方向分开后：**前进 11/11 全为负、后退 10/10 全为正、左移 8/8 全为正、右移 6/6 全为负**
—— 偏航是**方向锁定的确定性偏差**，不是随机噪声。混统计会把它平均掉。

复算：`python3 实验/t14_correction/fit.py`
"""
from __future__ import annotations

import json
import math
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "实验" / "t12_stats" / "dataset.json"

# 组名 → (action_type, 方向关键词, 是否角度量)
GROUPS = {
    ("纵向", "前进"): ("straight", "前进", False),
    ("纵向", "后退"): ("straight", "后退", False),
    ("横向", "左移"): ("strafe", "左移", False),
    ("横向", "右移"): ("strafe", "右移", False),
    ("旋转", "逆时针"): ("rotate_in_place", "逆时针", True),
    ("旋转", "顺时针"): ("rotate_in_place", "顺时针", True),
}
# 斜行只有单一档时长（357 ms）⇒ V 与 t_d 分离不出来，单独按「单档常数」处理
DIAGONAL = {
    "前左": ("diagonal", "前左"), "前右": ("diagonal", "前右"),
    "后左": ("diagonal", "后左"), "后右": ("diagonal", "后右"),
}


def load_ok() -> list[dict]:
    D = json.load(open(DATASET))
    return [d for d in D
            if str(d["trial_ok"]).strip().upper() in ("TRUE", "1", "OK")
            and d["vision_dist_mm"] is not None]


def select(ok: list[dict], action: str, key: str) -> list[dict]:
    return [d for d in ok if d["action_type"] == action
            and key in str(d["command_description"])]


def _lsq(pairs: list[tuple[float, float]]) -> tuple[float, float, float, int]:
    """最小二乘直线 d = a·t + b；返回 (a, b, R², n)。"""
    n = len(pairs)
    mx = sum(p[0] for p in pairs) / n
    my = sum(p[1] for p in pairs) / n
    sxx = sum((p[0] - mx) ** 2 for p in pairs)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pairs)
    a = sxy / sxx
    b = my - a * mx
    syy = sum((p[1] - my) ** 2 for p in pairs)
    r2 = 1 - sum((p[1] - (a * p[0] + b)) ** 2 for p in pairs) / syy if syy else float("nan")
    return a, b, r2, n


def _value(d: dict, ang: bool) -> float:
    return abs(d["vision_yaw_deg"]) if ang else d["vision_dist_mm"]


def fit_channel(ok: list[dict], action: str, key: str, ang: bool) -> dict:
    """一个通道-方向组：拟合 (V, t_d) + 档内重复性 + 偏航偏差。"""
    sel = select(ok, action, key)
    pairs = [(d["duration_planned"], _value(d, ang)) for d in sel]
    a, b, r2, n = _lsq(pairs)

    # 同一时长档内的散布 = **重复性**（模型只拟合档均值，残差就是它）
    per_level: dict[float, list[float]] = {}
    for d in sel:
        per_level.setdefault(d["duration_planned"], []).append(_value(d, ang))
    sigma = {int(t * 1000): round(st.pstdev(v), 2)
             for t, v in sorted(per_level.items()) if len(v) > 1}
    med = {int(t * 1000): round(st.median(v), 2) for t, v in sorted(per_level.items())}

    # 模型预测残差（从 T 预测 s，与实测比）
    err = sorted(abs(_value(d, ang) - (a * d["duration_planned"] + b)) for d in sel)

    yaws = [d["vision_yaw_deg"] for d in sel]
    yaw_level: dict[float, list[float]] = {}
    for d in sel:
        yaw_level.setdefault(d["duration_planned"], []).append(d["vision_yaw_deg"])
    return dict(
        channel=key, n=n,
        V=a, t_d_ms=-b / a * 1000.0, r2=r2,
        level_median=med, level_sigma=sigma,
        resid_med=st.median(err), resid_max=err[-1],
        yaw_median=st.median(yaws),
        yaw_by_level={int(t * 1000): round(st.median(v), 2)
                      for t, v in sorted(yaw_level.items())},
        yaw_pos=sum(1 for y in yaws if y > 0),
        yaw_neg=sum(1 for y in yaws if y < 0),
        yaw_values=list(yaws),
    )


def fit_diagonal(ok: list[dict], key: str) -> dict:
    """斜行单档常数（357 ms）：只给实测中位与极差，不给 V/t_d。"""
    action, kw = DIAGONAL[key]
    sel = select(ok, action, kw)
    ds = [d["vision_dist_mm"] for d in sel]
    ys = [d["vision_yaw_deg"] for d in sel]
    return dict(key=key, n=len(sel), T_ms=int(sel[0]["duration_planned"] * 1000),
                dist_median=st.median(ds), dist_min=min(ds), dist_max=max(ds),
                yaw_median=st.median(ys))


def main() -> int:
    ok = load_ok()
    print(f"样本：trial_ok 且 vision_dist_mm 有值 = {len(ok)} 条"
          f"（汇总行共 84 条，失败 22 条无对比列）\n")

    print("== 通道 × 方向：距离/角度模型 d = V·(T − t_d) ==")
    print(f"{'通道':6s}{'方向':8s}{'n':>3s}{'V':>12s}{'t_d(ms)':>9s}{'R²':>8s}"
          f"{'档内σ(ms:值)':>26s}{'模型|误差| 中位/max':>22s}")
    fits = {}
    for (ch, key), (action, kw, ang) in GROUPS.items():
        f = fit_channel(ok, action, kw, ang)
        fits[(ch, key)] = f
        unit = "°/s" if ang else "mm/s"
        sig = " ".join(f"{t}:{s}" for t, s in f["level_sigma"].items())
        print(f"{ch:6s}{key:8s}{f['n']:3d}{f['V']:12.2f}{f['t_d_ms']:9.1f}{f['r2']:8.4f}"
              f"{sig:>26s}{f['resid_med']:10.2f}/{f['resid_max']:<10.2f}  {unit}")

    print("\n== 偏航偏差（走歪）—— 看符号是否方向锁定 ==")
    print(f"{'通道':6s}{'方向':8s}{'n':>3s}{'yaw 中位°':>10s}{'正/负':>9s}"
          f"{'各时长档 yaw 中位°':>28s}")
    for (ch, key), f in fits.items():
        print(f"{ch:6s}{key:8s}{f['n']:3d}{f['yaw_median']:+10.2f}"
              f"{f['yaw_pos']:5d}/{f['yaw_neg']:<4d}"
              f"{str(f['yaw_by_level']):>28s}")

    print("\n== 斜行（单一档 357 ms，V/t_d 无法分离）==")
    for key in DIAGONAL:
        g = fit_diagonal(ok, key)
        print(f"  斜行{key}: n={g['n']} T={g['T_ms']} ms  距离中位 {g['dist_median']:6.2f} mm "
              f"(min {g['dist_min']:.2f} / max {g['dist_max']:.2f})  yaw 中位 {g['yaw_median']:+.2f}°")

    print("\n== 与既有常数的对照 ==")
    print(f"  纵向前进 V={fits[('纵向','前进')]['V']/1000:.3f} m/s "
          f"(matrix.V_STEADY_MPS=0.560, +{(fits[('纵向','前进')]['V']/1000/0.56-1)*100:.0f}%)")
    print(f"  横向右移 V={fits[('横向','右移')]['V']/1000:.3f} m/s "
          f"(matrix.V_LATERAL_MPS=0.223, +{(fits[('横向','右移')]['V']/1000/0.223-1)*100:.0f}%)")
    print(f"  横向左移 V={fits[('横向','左移')]['V']/1000:.3f} m/s "
          f"(matrix.V_LATERAL_MPS=0.223, {(fits[('横向','左移')]['V']/1000/0.223-1)*100:+.0f}%)")
    print(f"  旋转逆时针 V={fits[('旋转','逆时针')]['V']:.1f} °/s "
          f"(matrix.YAW_RATE_DEG_S=243.0, {(fits[('旋转','逆时针')]['V']/243-1)*100:+.0f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
