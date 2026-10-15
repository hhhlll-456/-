"""E2 稳态轮速 vs 新表 / E8 直行左右轮计数差 / E15 直行航向漂移 vs IMU。

数据源：`实验/实验数据记录.xlsx` 的原始行（每 trial 一块，100 Hz，编码器计数块内累加、
每 trial 归零；`timestamp` = 块内秒数；`pwm_*` = 该块下发的板级 duty，块内常数）。
"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import math, re, json, statistics as st
import openpyxl

XLSX = str(ROOT / str(ROOT / "实验/实验数据记录.xlsx"))
WMAP = str(ROOT / "固件/exp/src/wheel_map.h")
C1REV = [1276.0, 1270.0, 1276.0, 1274.0]


# ---------- 1. 解析 wheel_map.h 的八条曲线 ----------
src = open(WMAP).read()
tables = {}
for name, body in re.findall(r"const DutyOmega\s+(\w+)\[\]\s*=\s*\{(.*?)\};", src, re.S):
    pts = [(int(d), float(o)) for d, o in re.findall(r"\{\s*(\d+)\s*,\s*([\d.]+)\s*\}", body)]
    tables[name] = dict(pts)
# 板级极性与 invert[4]={true,false,true,false}：FL/RL 前进=板级负，FR/RR 前进=板级正
Pos = ["FL_BWD", "FR_FWD", "RL_BWD", "RR_FWD"]   # ω_des>0 查这几张
Neg = ["FL_FWD", "FR_BWD", "RL_FWD", "RR_BWD"]   # ω_des<0
print("wheel_map 曲线:", {k: len(v) for k, v in tables.items()})


def omega_map(duty, wheel_i):
    """按板级 duty 查表得 |ω|：板级负 → FL/RL 视为前进(ω_des>0)、FR/RR 视为后退。"""
    board_neg = duty < 0
    fwd = (not board_neg) if wheel_i in (1, 3) else board_neg
    name = Pos[wheel_i] if fwd else Neg[wheel_i]
    return tables[name].get(abs(int(duty)))


# ---------- 2. 分块 ----------
wb = openpyxl.load_workbook(XLSX, read_only=True, data_only=True)
ws = wb["data"]
rows = list(ws.iter_rows(values_only=True))
h = rows[0]; ix = {k: i for i, k in enumerate(h)}
blocks = []
cur = []
for r in rows[1:]:
    if r[ix["trial_ok"]] not in (None, ""):
        blocks.append((cur, r)); cur = []
    else:
        cur.append(r)
print(f"trial 块数={len(blocks)}")


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def steady_omega(block):
    """稳态窗 = 后 40% 采样；逐轮 ω = 2π·Δcnt/(C_1rev·Δt)（编码器单相，只有幅值）。"""
    ts = [num(r[ix["timestamp"]]) for r in block]
    ok = [i for i, t in enumerate(ts) if t is not None]
    if len(ok) < 5:
        return None
    t0, t1 = ts[ok[0]], ts[ok[-1]]
    lo = t0 + 0.6 * (t1 - t0)
    win = [i for i in ok if ts[i] >= lo]
    if len(win) < 3:
        return None
    a, b = win[0], win[-1]
    dt = ts[b] - ts[a]
    if dt <= 0:
        return None
    out = []
    for i, col in enumerate(("encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr")):
        d = num(block[b][ix[col]]) - num(block[a][ix[col]])
        if d is None:
            return None
        out.append(2 * math.pi * d / C1REV[i] / dt)
    return out


def q(xs, p):
    xs = sorted(xs); k = (len(xs) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] if lo == hi else xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def show(label, xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        print(f"  {label:30s} (无)")
        return
    print(f"  {label:30s} n={len(xs):3d} min={min(xs):+8.2f} p25={q(xs,.25):+8.2f} "
          f"med={q(xs,.5):+8.2f} p75={q(xs,.75):+8.2f} max={max(xs):+8.2f} mean={st.mean(xs):+8.2f}")


# ---------- 3. E2：稳态实测 |ω| vs 新表预测 ----------
print("\n" + "=" * 96)
print("【E2】稳态实测 |ω| vs 新表(wheel_map.h 落地套)对同 duty 的预测")
errs = {0: [], 1: [], 2: [], 3: []}
allerr = []
used = 0
for block, summ in blocks:
    if str(summ[ix["trial_ok"]]).strip().upper() not in ("TRUE", "1", "OK"):
        continue
    om = steady_omega(block)
    if om is None:
        continue
    duties = [num(block[len(block)//2][ix[c]]) for c in ("pwm_fl", "pwm_fr", "pwm_rl", "pwm_rr")]
    if any(d is None for d in duties):
        continue
    used += 1
    for i in range(4):
        pred = omega_map(duties[i], i)
        if pred:
            e = om[i] / pred - 1
            errs[i].append(e); allerr.append(e)
print(f"  用了 {used} 条 trial（trial_ok 真且有原始行）")
for i, nm in enumerate(("FL", "FR", "RL", "RR")):
    show(f"  {nm} 实测/表 −1 (%)", [e * 100 for e in errs[i]])
show("  全部轮 (%)", [e * 100 for e in allerr])

# ---------- 4. E8：直行左右轮计数差 ----------
print("\n" + "=" * 96)
print("【E8】直行：左右轮计数差（整块累计计数；左=FL+RL，右=FR+RR）")
diffs = []
for block, summ in blocks:
    if summ[ix["action_type"]] != "straight" or str(summ[ix["trial_ok"]]).strip().upper() not in ("TRUE", "1", "OK"):
        continue
    if not block:
        continue
    last = block[-1]
    c = [num(last[ix[k]]) for k in ("encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr")]
    if any(v is None for v in c):
        continue
    L, R = c[0] + c[2], c[1] + c[3]
    d = (L - R) / ((L + R) / 2)
    diffs.append(d)
    print(f"   #{summ[ix['experiment_id']]:>3} cnt L={L:6.0f} R={R:6.0f}  Δ左−右={L-R:+6.0f} ({d:+.2%})  "
          f"vision_dist={num(summ[ix['vision_dist_mm']]):7.1f} yawΔ={num(summ[ix['vision_yaw_deg']]):+6.2f}°")
show("  左右计数差 (%)", [d * 100 for d in diffs])

# ---------- 5. E15：直行视觉航向漂移 vs IMU ----------
print("\n" + "=" * 96)
print("【E15】直行：视觉航向漂移率 vs IMU 航向漂移率")
for block, summ in blocks:
    if summ[ix["action_type"]] != "straight" or str(summ[ix["trial_ok"]]).strip().upper() not in ("TRUE", "1", "OK"):
        continue
    vd = num(summ[ix["vision_dist_mm"]]); vy = num(summ[ix["vision_yaw_deg"]])
    if not block or vd is None or vd < 10:
        continue
    imu = [num(r[ix["imu_yaw"]]) for r in block]
    imu = [v for v in imu if v is not None]
    if len(imu) < 2:
        continue
    d = imu[-1] - imu[0]
    d = (d + 180) % 360 - 180
    print(f"   #{summ[ix['experiment_id']]:>3} vision yawΔ={vy:+6.2f}° → {abs(vy)/(vd/1000):6.2f} °/m | "
          f"IMU Δ={d:+7.2f}° → {abs(d)/(vd/1000):6.2f} °/m   (差 {abs(vy)-abs(d):+6.2f}°)")
