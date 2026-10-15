"""逐条复核 error-mece.md 中已回填的数字（不复用任何 opencode 产物）。"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import math, re, json, sys, statistics as st
import openpyxl

XLSX = str(ROOT / str(ROOT / "实验/实验数据记录.xlsx"))
sys.path.insert(0, str(ROOT / "实验"))
from vgt.harness import body_delta_to_field

def q(xs, p):
    xs = sorted(xs); k = (len(xs) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] if lo == hi else xs[lo] + (xs[hi] - xs[lo]) * (k - lo)

def show(label, xs, u=""):
    xs = [x for x in xs if x is not None]
    if not xs: return print(f"  {label:42s} (无)")
    print(f"  {label:42s} n={len(xs):3d} min={min(xs):+8.2f} p25={q(xs,.25):+8.2f} "
          f"med={q(xs,.5):+8.2f} p75={q(xs,.75):+8.2f} max={max(xs):+8.2f} {u}")

wb = openpyxl.load_workbook(XLSX, read_only=True, data_only=True)
rows = list(wb["data"].iter_rows(values_only=True))
h = rows[0]; ix = {k: i for i, k in enumerate(h)}
ENC = ("encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr")
PWM = ("pwm_fl", "pwm_fr", "pwm_rl", "pwm_rr")
def num(v):
    try: return float(v)
    except (TypeError, ValueError): return None

blocks = []; cur = []
for r in rows[1:]:
    if r[ix["trial_ok"]] not in (None, ""): blocks.append((cur, r)); cur = []
    else: cur.append(r)
OKb = [(b, s) for b, s in blocks if str(s[ix["trial_ok"]]).strip().upper() in ("TRUE","1","OK")]
print(f"块={len(blocks)}  trial_ok 真={len(OKb)}")

def run_span(block):
    """RUN 段 = pwm 非零的行；返回 (首行 idx, 末行 idx)。"""
    idx = [i for i, r in enumerate(block) if any(num(r[ix[c]]) not in (None, 0.0) for c in PWM)]
    return (idx[0], idx[-1]) if idx else None

# ---------- E2 ----------
print("\n【E2】稳态 duty + 实测轮速")
C1 = [1276.0,1270.0,1276.0,1274.0]
allw = []; byact = {}
dutyvals = []
for block, summ in OKb:
    sp = run_span(block)
    if not sp: continue
    a, b = sp
    ts = [num(block[i][ix["timestamp"]]) for i in range(a, b+1)]
    ok = [i for i, t in enumerate(ts) if t is not None]
    if len(ok) < 5: continue
    t0, t1 = ts[ok[0]], ts[ok[-1]]
    for i in ok: dutyvals.extend([num(block[a+i][ix[c]]) for c in PWM])
    lo = t0 + 0.6*(t1-t0)
    w = [a+i for i in ok if ts[i] >= lo]
    if len(w) < 3: continue
    p, z = w[0], w[-1]; dt = ts[z-a] - ts[p-a]
    if dt <= 0: continue
    act = summ[ix["action_type"]]
    for j, c in enumerate(ENC):
        d = num(block[z][ix[c]]) - num(block[p][ix[c]])
        if d is None: continue
        om = 2*math.pi*d/C1[j]/dt
        allw.append(om); byact.setdefault(act, []).append(om)
print(f"  稳态采样数 |pwm|==110 的比例: {sum(1 for v in dutyvals if v is not None and abs(v)==110)}/{len(dutyvals)}"
      f"  distinct={sorted(set(v for v in dutyvals if v is not None))[:6]}")
show("  全轮次 实测|ω| (rad/s)", allw)
for a in ("straight","rotate_in_place","strafe","diagonal"):
    show(f"    {a} 中位", [q(byact[a],.5)] if a in byact else [], "")

# ---------- E5 ----------
OKs = [s for _, s in OKb]
print("\n【E5】dist_diff_pct / yaw_diff_deg")
for a in (None,"straight","strafe","diagonal","rotate_in_place"):
    sel = OKs if a is None else [s for s in OKs if s[ix["action_type"]] == a]
    show(f"  {(a or '全部')} dist_diff_pct", [num(s[ix["dist_diff_pct"]]) for s in sel], "%")
for a in (None,):
    show("  全部 yaw_diff_deg", [num(s[ix["yaw_diff_deg"]]) for s in OKs], "deg")
print(f"  判据 |dist_diff_pct|<20 通过 "
      f"{sum(1 for s in OKs if abs(num(s[ix['dist_diff_pct']])) < 20)}/{len(OKs)}")

# ---------- E8 ----------
print("\n【E8】直行左右计数差（两种口径）")
sel = [s for _, s in OKb if _ and s[ix["action_type"]] == "straight"]
A = []; B = []
for block, summ in OKb:
    if summ[ix["action_type"]] != "straight": continue
    sp = run_span(block)
    if not sp: continue
    a, b = sp
    def cnt(r):
        v = [num(r[ix[k]]) for k in ENC]
        return None if any(x is None for x in v) else v
    cs, ce = cnt(block[a]), cnt(block[b])
    if cs is None or ce is None: continue
    d = [ce[i]-cs[i] for i in range(4)]
    L, R = d[0]+d[2], d[1]+d[3]
    A.append((L-R)/abs((L+R)/2)*100)
    cl, cr = cnt(block[-1]), cnt(block[0])
    if cl and cr:
        dl = [cl[i]-cr[i] for i in range(4)]
        L2, R2 = dl[0]+dl[2], dl[1]+dl[3]
        B.append((L2-R2)/abs((L2+R2)/2)*100)
show("  (L−R) RUN段首末 %", A)
show("  (L−R) 全块首末   %", B)
print(f"  左多(>0) 条数: RUN段 {sum(1 for x in A if x>0)}/{len(A)}   全块 {sum(1 for x in B if x>0)}/{len(B)}")
show("  直行 |yawΔ|/m (deg/m)", [abs(num(s[ix["vision_yaw_deg"]]))/(num(s[ix["vision_dist_mm"]])/1000)
     for _, s in OKb if _ and s[ix["action_type"]]=="straight" and (num(s[ix["vision_dist_mm"]]) or 0) > 10])
show("  直行 有符号 yawΔ/m", [num(s[ix["vision_yaw_deg"]])/(num(s[ix["vision_dist_mm"]])/1000)
     for _, s in OKb if _ and s[ix["action_type"]]=="straight" and (num(s[ix["vision_dist_mm"]]) or 0) > 10])
