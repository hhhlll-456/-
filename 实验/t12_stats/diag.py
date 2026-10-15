"""斜行 trial 的 RUN 段 pwm 结构 —— 为什么「四轮全非零」判据会漏掉全部 12 条斜行。"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import openpyxl
wb=openpyxl.load_workbook(str(ROOT / str(ROOT / "实验/实验数据记录.xlsx")),read_only=True,data_only=True)
rows=list(wb["data"].iter_rows(values_only=True)); h=rows[0]; ix={k:i for i,k in enumerate(h)}
PWM=("pwm_fl","pwm_fr","pwm_rl","pwm_rr")
def num(v):
    try: return float(v)
    except (TypeError,ValueError): return None
blocks=[];cur=[]
for r in rows[1:]:
    if r[ix["trial_ok"]] not in (None,""): blocks.append((cur,r)); cur=[]
    else: cur.append(r)
n=0
for block,summ in blocks:
    if summ[ix["action_type"]]!="diagonal" or str(summ[ix["trial_ok"]]).strip().upper() not in ("TRUE","1","OK"): continue
    n+=1
    if n>3: continue
    print(f"\n#{summ[ix['experiment_id']]} {summ[ix['command_description']]} 行数={len(block)}")
    for i,r in enumerate(block):
        p=[num(r[ix[c]]) for c in PWM]
        t=num(r[ix["timestamp"]])
        mark="" if all(v not in (None,0.0) for v in p) else "   <-- 有 0"
        if i<4 or i>len(block)-4 or mark: print(f"  [{i:3d}] t={t} pwm={p}{mark}")
        elif i==4: print("   ...")
