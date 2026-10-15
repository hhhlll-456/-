import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）
import sys
"""E2 窗口敏感性 + RUN 时长分布 + E8 相关 + E12 方向一致性。"""
import math, re, json, statistics as st
import openpyxl

D = json.load(open(str(_pl.Path(__file__).with_name("dataset.json"))))
OK = [d for d in D if str(d["trial_ok"]).strip().upper() in ("TRUE", "1", "OK")]
NUM = [d for d in OK if d["dist_diff_pct"] is not None]


def q(xs, p):
    xs = sorted(xs); k = (len(xs) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] if lo == hi else xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def show(label, xs, u=""):
    xs = [x for x in xs if x is not None]
    print(f"  {label:32s} n={len(xs):3d} min={min(xs):+8.2f} p25={q(xs,.25):+8.2f} "
          f"med={q(xs,.5):+8.2f} p75={q(xs,.75):+8.2f} max={max(xs):+8.2f} {u}")


print("【RUN 时长分布（duration_planned, s）】")
show("全部 trial_ok 真", [d["duration_planned"] for d in OK], "s")
for a in ("straight", "strafe", "diagonal", "rotate_in_place"):
    show(f"  {a}", [d["duration_planned"] for d in OK if d["action_type"] == a], "s")

print("\n【E2 窗口敏感性】稳态/晚窗/全程 三种窗口下「实测|ω|/表预测−1」的中位")
WMAP = str(ROOT / "固件/exp/src/wheel_map.h")
src = open(WMAP).read()
tables = {n: dict((int(d), float(o)) for d, o in re.findall(r"\{\s*(\d+)\s*,\s*([\d.]+)\s*\}", b))
          for n, b in re.findall(r"const DutyOmega\s+(\w+)\[\]\s*=\s*\{(.*?)\};", src, re.S)}
Pos = ["FL_BWD", "FR_FWD", "RL_BWD", "RR_FWD"]; Neg = ["FL_FWD", "FR_BWD", "RL_FWD", "RR_BWD"]
C1 = [1276.0, 1270.0, 1276.0, 1274.0]


def omap(duty, i):
    bn = duty < 0
    fwd = (not bn) if i in (1, 3) else bn
    return tables[(Pos if fwd else Neg)[i]].get(abs(int(duty)))


wb = openpyxl.load_workbook(str(ROOT / "实验/实验数据记录.xlsx"), read_only=True, data_only=True)
rows = list(wb["data"].iter_rows(values_only=True))
h = rows[0]; ix = {k: i for i, k in enumerate(h)}
blocks = []; cur = []
for r in rows[1:]:
    if r[ix["trial_ok"]] not in (None, ""):
        blocks.append((cur, r)); cur = []
    else:
        cur.append(r)


def n_(v):
    try: return float(v)
    except (TypeError, ValueError): return None


def omega(block, frac_lo, frac_hi):
    ts = [n_(r[ix["timestamp"]]) for r in block]
    ok = [i for i, t in enumerate(ts) if t is not None]
    if len(ok) < 5: return None
    t0, t1 = ts[ok[0]], ts[ok[-1]]
    w = [i for i in ok if frac_lo * (t1 - t0) + t0 <= ts[i] <= frac_hi * (t1 - t0) + t0]
    if len(w) < 2: return None
    a, b = w[0], w[-1]; dt = ts[b] - ts[a]
    if dt <= 0: return None
    return [2 * math.pi * (n_(block[b][ix[c]]) - n_(block[a][ix[c]])) / C1[i] / dt
            for i, c in enumerate(("encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr"))]


for lo, hi, nm in ((0.85, 1.0, "末 15%"), (0.6, 1.0, "后 40%"), (0.0, 1.0, "全程")):
    errs = []
    for block, summ in blocks:
        if str(summ[ix["trial_ok"]]).strip().upper() not in ("TRUE", "1", "OK") or len(block) < 6:
            continue
        om = omega(block, lo, hi)
        if om is None: continue
        mid = len(block) // 2
        dts = [n_(block[mid][ix[c]]) for c in ("pwm_fl", "pwm_fr", "pwm_rl", "pwm_rr")]
        for i in range(4):
            p = omap(dts[i], i) if dts[i] is not None else None
            if p: errs.append(om[i] / p - 1)
    show(f"  {nm} 实测/表−1 (%)", [e * 100 for e in errs])

print("\n【E8】左右计数差 (%) 与视觉航向漂移 (°) 的相关")
import statistics as s2
pairs = []
for block, summ in blocks:
    if summ[ix["action_type"]] != "straight" or str(summ[ix["trial_ok"]]).strip().upper() not in ("TRUE", "1", "OK") or not block:
        continue
    c = [n_(block[-1][ix[k]]) for k in ("encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr")]
    vy = n_(summ[ix["vision_yaw_deg"]])
    if any(v is None for v in c) or vy is None: continue
    L, R = c[0] + c[2], c[1] + c[3]
    pairs.append(((L - R) / ((L + R) / 2) * 100, vy))
xs = [p[0] for p in pairs]; ys = [p[1] for p in pairs]
mx, my = s2.mean(xs), s2.mean(ys)
num = sum((a - mx) * (b - my) for a, b in pairs)
den = math.sqrt(sum((a - mx) ** 2 for a in xs) * sum((b - my) ** 2 for b in ys))
print(f"  n={len(pairs)}  Pearson r = {num/den:+.3f}   （若为负：左轮多转 ↔ 航向负向漂）")

print("\n【E12】方向一致性（视觉位移方向 vs 指令方向）")
sys.path.insert(0, str(ROOT / "实验"))
from vgt.harness import body_delta_to_field
bad = 0; tot = 0
for d in NUM:
    fx = d["vision_end_x"] - d["vision_start_x"]; fy = d["vision_end_y"] - d["vision_start_y"]
    c = math.cos(math.radians(d["vision_start_yaw"])); s = math.sin(math.radians(d["vision_start_yaw"]))
    bx, by = c * fx - s * fy, -s * fx - c * fy
    p1, p2 = d["command_param_1"], d["command_param_2"]
    if d["action_type"] == "rotate_in_place":
        continue
    tot += 1
    if d["action_type"] == "straight":
        ok = (bx > 0) == ("前进" in str(d["command_description"]))
    elif d["action_type"] == "strafe":
        ok = (by > 0) == ("左移" in str(d["command_description"]))
    else:   # diagonal
        ok = (bx > 0) == ("前进" in str(d["command_description"])) and (by > 0) == ("左" in str(d["command_description"]))
    if not ok:
        bad += 1
        print(f"   ⚠ #{d['experiment_id']} {d['command_description']} Δbody=({bx:+.1f},{by:+.1f})")
print(f"  平移行 n={tot}，方向一致 {tot-bad}，不一致 {bad}")
rotd = [d for d in NUM if d["action_type"] == "rotate_in_place"]
c = sum(1 for d in rotd if (d["vision_yaw_deg"] > 0) == ("顺时针" not in str(d["command_description"])))
print(f"  旋转行 n={len(rotd)}，yaw 符号与描述一致 {c}")
