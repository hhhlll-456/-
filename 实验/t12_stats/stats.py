"""Task 12 真机行的统计（口径在每条打印里写清）。"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import json, math, statistics as st

D = json.load(open(str(_pl.Path(__file__).with_name("dataset.json"))))
OK = [d for d in D if str(d["trial_ok"]).strip().upper() in ("TRUE", "1", "OK")]
NUM = [d for d in OK if d["dist_diff_pct"] is not None]


def q(xs, p):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] if lo == hi else xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def dist(label, xs, unit=""):
    xs = [x for x in xs if x is not None]
    if not xs:
        print(f"  {label:34s} (无数据)")
        return
    print(f"  {label:34s} n={len(xs):3d} min={min(xs):+8.2f} p25={q(xs,.25):+8.2f} "
          f"med={q(xs,.5):+8.2f} p75={q(xs,.75):+8.2f} max={max(xs):+8.2f} "
          f"mean={st.mean(xs):+8.2f} {unit}")


print("=" * 100)
print("【A/E5】dist_diff_pct / yaw_diff_deg —— 口径：trial_ok 真且 dist_diff_pct 非空的行")
dist("dist_diff_pct (%)", [d["dist_diff_pct"] for d in NUM])
dist("yaw_diff_deg (deg)", [d["yaw_diff_deg"] for d in NUM])
for a in ("straight", "strafe", "diagonal", "rotate_in_place"):
    s = [d for d in NUM if d["action_type"] == a]
    print(f"  -- {a} (n={len(s)})")
    dist("     dist_diff_pct", [d["dist_diff_pct"] for d in s])
    dist("     yaw_diff_deg", [d["yaw_diff_deg"] for d in s])

print("=" * 100)
print("【B/E6】odo vs vision 过原点斜率  → R_eff = 33.5 / 斜率")
def slope(rows):
    num = sum(d["odo_dist_mm"] * d["vision_dist_mm"] for d in rows)
    den = sum(d["vision_dist_mm"] ** 2 for d in rows)
    return num / den, len(rows)

for name, sel in (
    ("全部平移（straight+strafe+diagonal）", [d for d in NUM if d["action_type"] != "rotate_in_place"]),
    ("仅 straight", [d for d in NUM if d["action_type"] == "straight"]),
    ("仅 strafe", [d for d in NUM if d["action_type"] == "strafe"]),
    ("仅 diagonal", [d for d in NUM if d["action_type"] == "diagonal"]),
):
    k, n = slope(sel)
    print(f"  {name:36s} n={n:3d} 斜率={k:6.4f}  R_eff={33.5/k:6.2f} mm  ({(33.5/k)/33.5-1:+.2%})")

print("  逐动作 odo/vision 比值分布（只取 vision>10mm 的行，避免小位移放大）")
for a in ("straight", "strafe", "diagonal"):
    rs = [d["odo_dist_mm"] / d["vision_dist_mm"] for d in NUM
          if d["action_type"] == a and d["vision_dist_mm"] > 10]
    dist(f"     {a} odo/vision", rs)

print("=" * 100)
print("【C/E9】横移交叉耦合 ε_xy —— 口径：trial_ok 真且 action=strafe；把场地系 Δ(x,y) 旋回车体系")
def body_delta(d):
    dx = d["vision_end_x"] - d["vision_start_x"]
    dy = d["vision_end_y"] - d["vision_start_y"]
    psi = math.radians(d["vision_start_yaw"])
    # 场地系 == 车体系 旋转 psi（body→field），反变换得车体系分量
    bx = dx * math.cos(psi) + dy * math.sin(psi)
    by = -dx * math.sin(psi) + dy * math.cos(psi)
    return bx, by, math.hypot(dx, dy)

left, right = [], []
for d in NUM:
    if d["action_type"] != "strafe":
        continue
    bx, by, r = body_delta(d)
    eps = abs(bx) / abs(by) if by else float("inf")
    # 车体系 +y = 车左；vy>0 指令 = 左移
    tgt = "左" if by > 0 else "右"
    (left if by > 0 else right).append((d["experiment_id"], bx, by, r, eps, d["yaw_diff_deg"]))
    print(f"   #{d['experiment_id']:>3} {tgt}移 Δbody=({bx:+7.2f},{by:+7.2f}) |Δ|={r:6.1f} "
          f"ε_xy={eps:6.1%} yawΔ={d['yaw_diff_deg']:+5.2f}°")
print(f"  左移 n={len(left)}: ε_xy 中位={q([x[4] for x in left],.5):.1%}  右移 n={len(right)}: ε_xy 中位={q([x[4] for x in right],.5):.1%}")
dist("  左移 ε_xy", [x[4] for x in left]); dist("  右移 ε_xy", [x[4] for x in right])

print("=" * 100)
print("【D/E8+E15】直行：航向漂移 °/m + 左右轮计数差")
for d in NUM:
    if d["action_type"] != "straight" or d["vision_dist_mm"] < 10:
        continue
    drift = abs(d["vision_yaw_deg"]) / (d["vision_dist_mm"] / 1000.0)
    print(f"   #{d['experiment_id']:>3} dist={d['vision_dist_mm']:7.1f}mm yawΔ={d['vision_yaw_deg']:+6.2f}° "
          f"→ 漂移 {drift:6.2f} °/m   yaw_diff_deg={d['yaw_diff_deg']:+6.2f}")
drifts = [abs(d["vision_yaw_deg"]) / (d["vision_dist_mm"] / 1000.0) for d in NUM
          if d["action_type"] == "straight" and d["vision_dist_mm"] > 10]
dist("  直行 |yawΔ|/m (°/m)", drifts)

print("=" * 100)
print("【E/E10】原地旋转：视觉 vs 编码器航向差")
dist("旋转 yaw_diff_deg", [d["yaw_diff_deg"] for d in NUM if d["action_type"] == "rotate_in_place"])
dist("旋转 dist_diff_pct", [d["dist_diff_pct"] for d in NUM if d["action_type"] == "rotate_in_place"])
for d in NUM:
    if d["action_type"] != "rotate_in_place":
        continue
    print(f"   #{d['experiment_id']:>3} vision={d['vision_yaw_deg']:+7.2f}° odo={d['odo_yaw_deg']:+7.2f}° "
          f"Δ={d['yaw_diff_deg']:+6.2f}°  ({d['yaw_diff_deg']/d['vision_yaw_deg']-1:+.1%} 相对)")
