"""E2 窗口敏感性：整段 RUN vs 末 40% RUN —— 判定 doc 里 n=248 那组数是哪个窗口。"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import math, openpyxl
def q(xs,p):
    xs=sorted(xs); k=(len(xs)-1)*p; lo,hi=math.floor(k),math.ceil(k)
    return xs[lo] if lo==hi else xs[lo]+(xs[hi]-xs[lo])*(k-lo)
def show(l,xs):
    xs=[x for x in xs if x is not None]
    print(f"  {l:34s} n={len(xs):3d} min={min(xs):+6.2f} p25={q(xs,.25):+6.2f} med={q(xs,.5):+6.2f} p75={q(xs,.75):+6.2f} max={max(xs):+6.2f}")
wb=openpyxl.load_workbook(str(ROOT / str(ROOT / "实验/实验数据记录.xlsx")),read_only=True,data_only=True)
rows=list(wb["data"].iter_rows(values_only=True)); h=rows[0]; ix={k:i for i,k in enumerate(h)}
ENC=("encoder_fl","encoder_fr","encoder_rl","encoder_rr"); PWM=("pwm_fl","pwm_fr","pwm_rl","pwm_rr")
def num(v):
    try: return float(v)
    except (TypeError,ValueError): return None
blocks=[];cur=[]
for r in rows[1:]:
    if r[ix["trial_ok"]] not in (None,""): blocks.append((cur,r)); cur=[]
    else: cur.append(r)
OKb=[(b,s) for b,s in blocks if str(s[ix["trial_ok"]]).strip().upper() in ("TRUE","1","OK")]
def last_run(block):
    nz=[i for i,r in enumerate(block) if all(num(r[ix[c]]) not in (None,0.0) for c in PWM)]
    if not nz: return None
    e=nz[-1]; s=e
    while s-1 in nz: s-=1
    return s,e
C1=[1276.,1270.,1276.,1274.]
def collect(frac_lo, skip_ramp_s=None):
    allw=[]; byact={}
    for block,summ in OKb:
        sp=last_run(block)
        if not sp: continue
        a,b=sp
        ts=[num(block[i][ix["timestamp"]]) for i in range(a,b+1)]
        if any(t is None for t in ts) or len(ts)<2: continue
        t0,t1=ts[0],ts[-1]
        lo=t0+frac_lo*(t1-t0)
        if skip_ramp_s: lo=max(lo, t0+skip_ramp_s)
        w=[i for i in range(len(ts)) if ts[i]>=lo]
        if len(w)<2: continue
        p,z=w[0],w[-1]; dt=ts[z]-ts[p]
        if dt<=0: continue
        for j,c in enumerate(ENC):
            d=num(block[a+z][ix[c]])-num(block[a+p][ix[c]])
            if d is None: continue
            om=2*math.pi*d/C1[j]/dt; allw.append(om); byact.setdefault(summ[ix["action_type"]],[]).append(om)
    return allw,byact
print("\n[A] 整段 RUN（首末样点差）")
A,AA=collect(0.0); show("  全轮次 |ω|",A)
for k in ("straight","rotate_in_place","strafe","diagonal"):
    if k in AA: print(f"    {k:18s} 中位={q(AA[k],.5):6.2f} (n={len(AA[k])})")
print("\n[B] 后 40% RUN")
B,BB=collect(0.6); show("  全轮次 |ω|",B)
for k in ("straight","rotate_in_place","strafe","diagonal"):
    if k in BB: print(f"    {k:18s} 中位={q(BB[k],.5):6.2f} (n={len(BB[k])})")
print("\n[C] 跳过前 0.15 s（死区）后")
Cf,CC=collect(0.0,0.15); show("  全轮次 |ω|",Cf)
