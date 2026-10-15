"""Task B 系统辨识：从 Task 12 真机行拟合「距离(或角度) vs 指令时长」直线，
反推有效速度 V 与起转死区 t_d，并与 R72 的既有模型常数对照。

模型：车体位移 d(t) = V·(t − t_d)  ⇒ 线性回归 d = a·t + b，V = a，t_d = −b/a。
口径：trial_ok=TRUE 且有视觉对比列；按「动作 + 方向 + 指令距离档」分组，各组样本数 n 标注。
不预置任何修正项 —— 拟合只用真机行本身。
"""
import pathlib as _pl, json, math
ROOT = _pl.Path(__file__).resolve().parents[2]
D = json.load(open(str(_pl.Path(__file__).with_name("dataset.json"))))


def fit(pairs):
    """过最小二乘直线。pairs = [(t, d)]；返回 a, b, r2, n。"""
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


def report(title, sel, col, unit):
    pairs = [(d["duration_planned"], abs(d[col]) if col.startswith("vision_yaw") else d[col])
             for d in sel]
    a, b, r2, n = fit(pairs)
    td = -b / a
    print(f"\n{title}")
    print(f"  直线 d = {a:8.2f}·t {b:+8.2f} {unit}   R²={r2:.4f}  n={n}")
    print(f"  ⇒ V = {a:.3f} {unit}/s   t_d = {td*1000:.1f} ms")
    for t, d in sorted(pairs):
        print(f"       t={t:.3f}s  d={d:8.2f} {unit}   (模型 {a*t+b:8.2f})")
    return a, td, r2, n


OK = [d for d in D if str(d["trial_ok"]).strip().upper() in ("TRUE", "1", "OK")
      and d["vision_dist_mm"] is not None]

print("=" * 78)
print("【纵向】直行 —— vision_dist_mm vs duration_planned")
fwd = [d for d in OK if d["action_type"] == "straight" and "前进" in str(d["command_description"])]
bwd = [d for d in OK if d["action_type"] == "straight" and "后退" in str(d["command_description"])]
V, T, _, n = report("直行前进（150/200 mm 两档，n=11）", fwd, "vision_dist_mm", "mm")
V2, T2, _, n2 = report("直行后退（150/200 mm 两档，n=10）", bwd, "vision_dist_mm", "mm")

print("\n" + "=" * 78)
print("【横向】横移 —— vision_dist_mm vs duration_planned")
sl = [d for d in OK if d["action_type"] == "strafe" and "左移" in str(d["command_description"])]
sr = [d for d in OK if d["action_type"] == "strafe" and "右移" in str(d["command_description"])]
VL, TL, _, _ = report("横移左移（50/80 mm 两档，n=8）", sl, "vision_dist_mm", "mm")
VR, TR, _, _ = report("横移右移（50/80 mm 两档，n=6）", sr, "vision_dist_mm", "mm")

print("\n" + "=" * 78)
print("【旋转】原地 —— |vision_yaw_deg| vs duration_planned")
for tag, key in (("逆时针", "逆时针"), ("顺时针", "顺时针")):
    sel = [d for d in OK if d["action_type"] == "rotate_in_place" and key in str(d["command_description"])]
    report(f"原地{tag}（30/45° 两档，n={len(sel)}）", sel, "vision_yaw_deg", "°")

print("\n" + "=" * 78)
print("【与 R72 既有模型常数对照】")
import sys
sys.path.insert(0, str(ROOT / "实验"))
from vgt import matrix, harness
print(f"  R72 纵向: V_STEADY={matrix.V_STEADY_MPS} m/s, DEAD={matrix.DEAD_MS} ms   "
      f"本批拟合 {V/1000:.3f} m/s, {T*1000:.0f} ms")
print(f"  R72 横向: V_LATERAL={matrix.V_LATERAL_MPS} m/s, DEAD_LATERAL={matrix.DEAD_LATERAL_MS} ms   "
      f"本批拟合左移 {VL/1000:.3f} m/s/{TL*1000:.0f} ms、右移 {VR/1000:.3f} m/s/{TR*1000:.0f} ms")
