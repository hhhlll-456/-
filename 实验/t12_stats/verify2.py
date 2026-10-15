"""E2（修正 RUN 段定义）+ E10 + E12 + E15 + E16 复核。"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import math, json, sys
import openpyxl
XLSX = str(ROOT / str(ROOT / "实验/实验数据记录.xlsx"))
def q(xs,p):
    xs=sorted(xs); k=(len(xs)-1)*p; lo,hi=math.floor(k),math.ceil(k)
    return xs[lo] if lo==hi else xs[lo]+(xs[hi]-xs[lo])*(k-lo)
def show(l,xs,u=""):
    xs=[x for x in xs if x is not None]
    if not xs: return print(f"  {l:40s} (无)")
    print(f"  {l:40s} n={len(xs):3d} min={min(xs):+8.2f} p25={q(xs,.25):+8.2f} med={q(xs,.5):+8.2f} "
          f"p75={q(xs,.75):+8.2f} max={max(xs):+8.2f} {u}")
wb=openpyxl.load_workbook(XLSX,read_only=True,data_only=True)
rows=list(wb["data"].iter_rows(values_only=True))
h=rows[0]; ix={k:i for i,k in enumerate(h)}
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
    """RUN 段 = 末段连续 pwm 全非零区域。返回 (a,b) 或 None。"""
    nz=[i for i,r in enumerate(block) if all(num(r[ix[c]]) not in (None,0.0) for c in PWM)]
    if not nz: return None
    end=nz[-1]; start=end
    while start-1 in nz: start-=1
    return start,end

print("【E2 修正】以「末段连续 pwm≠0」为 RUN 段")
C1=[1276.,1270.,1276.,1274.]; allw=[]; byact={}; dv=[]; durs={}; pers=[]
for block,summ in OKb:
    sp=last_run(block)
    if not sp: continue
    a,b=sp
    durs[summ[ix["action_type"]]]=durs.get(summ[ix["action_type"]],[])+[b-a+1]
    for i in range(a,b+1): dv.extend([num(block[i][ix[c]]) for c in PWM])
    t0,t1=num(block[a][ix["timestamp"]]),num(block[b][ix["timestamp"]]); T=t1-t0
    lo=t0+0.6*T; w=[i for i in range(a,b+1) if (num(block[i][ix["timestamp"]]) or -1)>=lo]
    if len(w)<3: continue
    p,z=w[0],w[-1]; dt=num(block[z][ix["timestamp"]])-num(block[p][ix["timestamp"]])
    if dt<=0: continue
    act=summ[ix["action_type"]]
    for j,c in enumerate(ENC):
        d=num(block[z][ix[c]])-num(block[p][ix[c]])
        if d is None: continue
        om=2*math.pi*d/C1[j]/dt; allw.append(om); byact.setdefault(act,[]).append(om)
nz=[v for v in dv if v is not None]
print(f"  RUN 段采样 |pwm|: distinct={sorted(set(abs(v) for v in nz))}  |pwm|==110: {sum(1 for v in nz if abs(v)==110)}/{len(nz)}")
show("  全轮次 实测|ω| (rad/s)", allw)
for a in ("straight","rotate_in_place","strafe","diagonal"):
    if a in byact: show(f"    {a} 中位",[q(byact[a],.5)],f"(n={len(byact[a])} 轮次)")
print("  RUN 段采样行数（时间窗长）:", {k:sorted(set(v)) for k,v in durs.items()})

print("\n【E12】odo−vision / imu−vision")
trs=[s for _,s in OKb if s[ix["action_type"]]!="rotate_in_place"]
show("  平移 odo−vision (mm)",[num(s[ix["odo_dist_mm"]])-num(s[ix["vision_dist_mm"]]) for s in trs])
show("  odo/vision 比值",[num(s[ix["odo_dist_mm"]])/num(s[ix["vision_dist_mm"]]) for s in trs])
im=[]
for block,summ in OKb:
    sp=last_run(block)
    if not sp: continue
    a,b=sp
    y0,y1=num(block[a][ix["imu_yaw"]]),num(block[b][ix["imu_yaw"]])
    vy=num(summ[ix["vision_yaw_deg"]])
    if None in (y0,y1,vy): continue
    d=(y1-y0+180)%360-180
    im.append((summ[ix["action_type"]],d-vy))
show("  全部 imu−vision (deg)",[v for _,v in im])
show("  直行 imu−vision (deg)",[v for a,v in im if a=="straight"])

print("\n【E10】原地旋转")
rot=[s for _,s in OKb if s[ix["action_type"]]=="rotate_in_place"]
show("  yaw_diff_deg",[num(s[ix["yaw_diff_deg"]]) for s in rot],"deg")
show("  |vision|/|odo|",[abs(num(s[ix["vision_yaw_deg"]]))/abs(num(s[ix["odo_yaw_deg"]])) for s in rot])

print("\n【E15】直行 IMU 漂移率")
sel=[]
for block,summ in OKb:
    if summ[ix["action_type"]]!="straight": continue
    vd=num(summ[ix["vision_dist_mm"]]); sp=last_run(block)
    if not sp or not vd or vd<10: continue
    a,b=sp
    y0,y1=num(block[a][ix["imu_yaw"]]),num(block[b][ix["imu_yaw"]])
    if None in (y0,y1): continue
    d=(y1-y0+180)%360-180
    sel.append(d/(vd/1000))
show("  IMU 有符号 漂移率 (deg/m)",sel)
show("  IMU |漂移率| (deg/m)",[abs(v) for v in sel])
