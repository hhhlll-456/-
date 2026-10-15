"""E9 横移交叉耦合 ε_xy —— 用 harness 的权威变换反解，并自检「实测方向 == 指令方向」。

口径：trial_ok 真、action=strafe；场地位移 (vision_end − vision_start) 用**起点 yaw**
经 `harness.body_delta_to_field` 的反变换转到**起点车体系**（与里程计落点同系）。
ε_xy := |Δx_body| / |Δy_body|（纵向泄漏 / 横向主运动）。
自检：body Δy > 0 ⇒ 车左 ⇒ 指令必须是「横移左移」。
"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import json, math, sys
sys.path.insert(0, str(ROOT / "实验"))
from vgt.harness import body_delta_to_field

D = json.load(open(str(_pl.Path(__file__).with_name("dataset.json"))))
S = [d for d in D if d["dist_diff_pct"] is not None and d["action_type"] == "strafe"]


def field_to_body(fx, fy, psi_deg):
    """反解 body，使 body_delta_to_field(bx,by,psi) == (fx,fy)（数值验证，不手推）。"""
    c, s = math.cos(math.radians(psi_deg)), math.sin(math.radians(psi_deg))
    bx, by = c * fx - s * fy, -s * fx - c * fy
    assert abs(body_delta_to_field(bx, by, psi_deg)[0] - fx) < 1e-9
    assert abs(body_delta_to_field(bx, by, psi_deg)[1] - fy) < 1e-9
    return bx, by


def med(v):
    v = sorted(v); n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2


rows = []
for d in S:
    fx = d["vision_end_x"] - d["vision_start_x"]
    fy = d["vision_end_y"] - d["vision_start_y"]
    bx, by = field_to_body(fx, fy, d["vision_start_yaw"])
    eps = abs(bx) / abs(by)
    want_left = "左移" in str(d["command_description"])
    rows.append(dict(eid=d["experiment_id"], desc=d["command_description"],
                     fx=fx, fy=fy, bx=bx, by=by, eps=eps,
                     meas="左移" if by > 0 else "右移",
                     cmd="左移" if want_left else "右移",
                     yaw=d["vision_start_yaw"], yawd=d["yaw_diff_deg"],
                     dist=d["vision_dist_mm"]))

bad = [r for r in rows if r["meas"] != r["cmd"]]
print(f"自检：{len(rows)} 条横移，实测方向 == 指令方向的 {len(rows)-len(bad)} 条；不一致 {len(bad)} 条")
for r in bad:
    print("   ⚠ 不一致:", r)
print()
print("分组明细：")
for tag in ("左移", "右移"):
    g = [r for r in rows if r["cmd"] == tag]
    print(f" {tag} n={len(g)}  ε_xy 中位={med([r['eps'] for r in g]):.1%}"
          f"  分位(min/p25/med/p75/max)={'/'.join(f'{v:.1%}' for v in sorted([r['eps'] for r in g]))}")
    for r in g:
        print(f"    #{r['eid']:>3} {str(r['desc'])[:14]:14s} ψ0={r['yaw']:+5.1f}° |Δ|={r['dist']:5.1f} "
              f"Δbody=({r['bx']:+6.2f},{r['by']:+7.2f}) ε_xy={r['eps']:6.1%} yawΔ={r['yawd']:+5.2f}°")
epsL = [r["eps"] for r in rows if r["cmd"] == "左移"]
epsR = [r["eps"] for r in rows if r["cmd"] == "右移"]
print(f"\n汇总：左移 n={len(epsL)} 中位={med(epsL):.1%}  右移 n={len(epsR)} 中位={med(epsR):.1%}")
print(f"      合并 n={len(rows)} 中位={med([r['eps'] for r in rows]):.1%} 均值={sum(r['eps'] for r in rows)/len(rows):.1%}")
print(f"      横移航向漂移 |yawΔ| 中位={med([abs(r['yawd']) for r in rows]):.2f}° max={max(abs(r['yawd']) for r in rows):.2f}°")
