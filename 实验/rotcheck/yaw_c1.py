#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""偏航腿的 c1 —— 用 rot.jsonl 里已有的腿做离线解算（不连车、不发命令）。

## 为什么偏航腿能独立判 c1

仓库自己的正运动学（`odometry.integrate_encoders`）：

    d_i  = q_i · 2πR / c1_i              （带符号滚动距离，mm）
    dψ   = (−d0 + d1 − d2 + d3) / (4·LXY)

**偏航和平移吃的是同一个 c1/R**（d_i 只有一处定义）。所以"平移尺度差 2.7×"
若成立，偏航必然也差 2.7×。

原地旋转时四轮线速度满足 `vFL = −vFR = −vRL = vRR = LXY·ω`
（由 vx = vy = 0、ω ≠ 0 解出来，见 `verify_formula()`），于是**四项带符号距离
同号**，dψ 化简成

    dψ = (2πR / (4·LXY)) · Σ_i (Δc_i / c1_i)

即"总计数 → 偏航角"是一条**只被 1/c1 缩放**的直线：c1 大 ⇒ 每计数走的距离小 ⇒
偏航小。所以拿视觉偏航当真值就能反解 c1：

    c1 = (2πR / (4·LXY)) · Σ(Δc_i / c1_nom_i) / Δψ_true   （以标称 c1 为单位）

写成标称的**尺度因子** s（c1_true_i = s · c1_nom_i）最干净：

    s = (2πR / (4·LXY)) · Σ(Δc_i / c1_nom_i) / Δψ_true

s = 1 表示标称 c1 刚好对上；s = 2.7 才会支持"尺度少报 2.7×"。

## 窗口对齐

`rot_leg` 的编码器窗口 = [RUN 前 ZENC 读数, DONE+STOP+settle 0.8s 之后]，
视觉窗口 = [start 帧, end 帧]。**两端的余量都落在车静止的时候**，所以多出来
的时间既不加计数也不加偏航 —— 这是与"平移腿"的 D 行窗口错配不同的：
这里 `d_enc` 是停车后再读的，已经把自由滑行包进去了。
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path("/home/hanli/麦轮小车pid")
sys.path.insert(0, str(ROOT / "实验"))
from vgt.odometry import integrate_encoders, parse_telemetry   # noqa: E402

C1_NOM = (1276.0, 1270.0, 1276.0, 1274.0)
SIGN = (-1.0, 1.0, -1.0, 1.0)
R_EFF = 33.5
LXY_MM = 0.12825 * 1000.0        # m → mm（固件值 R47：不是轴距）
K = 2.0 * math.pi * R_EFF / (4.0 * LXY_MM)      # (2πR/4LXY)，单位 mm/计数


def verify_formula():
    """闭式 dψ = K·Σ(Δc/c1) 必须与仓库自己的 `integrate_encoders` 逐位一致。

    用**合成的原地旋转样本**：四轮同速、pwm 交替方向。合成不了"真实"麦轮
    运动学，所以这里只验闭式化简那一步（把 D 行积分出来的偏航拿回来比）。
    """
    # 构造：四轮同速递增。关键在 pwm 图案 —— 仓库的 forward 符号 sign=[−1,+1,−1,+1]
    # 已经把麦轮的"镜像接法"编进去了，所以**原地旋转时四轮 pwm 同号**（不是交替）：
    #   运动学要求 d = (−a,+a,−a,+a)；而 sign(d_i)=sign_i·sgn(pwm_i)，
    #   要它等于 (−,+,−,+)，只能 sgn(pwm) 全为 +。
    #   （这里若写成交替 pwm，积分出来是纯平移、偏航≈0 —— 那是合成数据错了，
    #     不是闭式错了。下面的数字就是那次写错的样子：−0.001 rad vs 0.644 rad。）
    n = 500
    samples = [
        {"t_ms": 0, "pwm": [110, 110, 110, 110], "enc": [0, 0, 0, 0], "rpy": [0, 0, 0]},
        {"t_ms": 100, "pwm": [110, 110, 110, 110], "enc": [n, n, n, n], "rpy": [0, 0, 0]},
    ]
    traj = integrate_encoders(samples, c_1rev=C1_NOM, sign=SIGN,
                              l_xy_over_r=0.12825 / 0.0335, r_eff_mm=R_EFF)
    dpsi_repo = math.radians(traj[-1][2])
    dpsi_closed = K * sum(n / c for n, c in zip(samples[-1]["enc"], C1_NOM))
    # 同时验"没有平移"：原地旋转 dx_body/dy_body 必须是 0
    print(f"[verify] integrate_encoders dψ = {dpsi_repo:.9f} rad "
          f"（dx={traj[-1][0]:.6f} dy={traj[-1][1]:.6f}，都该≈0）")
    print(f"[verify] 闭式 K·Σ(Δc/c1)  = {dpsi_closed:.9f} rad")
    ok = abs(dpsi_repo - dpsi_closed) < 1e-9
    print(f"[verify] {'✅ 一致' if ok else '❌ 不一致 —— 闭式别用'}\n")
    return ok


def leg_rows(path):
    rows = []
    for ln in Path(path).read_text(encoding="utf-8").splitlines():
        if not ln.strip():
            continue
        r = json.loads(ln)
        if r.get("vision_dyaw_deg") is None or not r.get("d_enc"):
            continue
        rows.append(r)
    return rows


def analyze(path):
    rows = leg_rows(path)
    print(f"== {path}：{len(rows)} 条偏航腿 ==")
    scales, flats = [], []
    for r in rows:
        dc = [float(x) for x in r["d_enc"]]
        dpsi_vis = math.radians(r["vision_dyaw_deg"])
        # 用标称 c1 算出来的偏航幅值（应当 ≈ 视觉偏航 —— 差得不多就说明标称 c1 对）。
        # 闭式吃的是**无符号**计数（固件只做 encCount[i]++），所以算出来的只有幅值；
        # 方向要靠指令 om 的符号补回去（运动学 d_i 的符号 = sign(w_des_i) = sign(om)·(−,+,−,+)）。
        dpsi_mag = K * sum(c / cn for c, cn in zip(dc, C1_NOM))
        s = dpsi_mag / abs(dpsi_vis)
        dir_ok = (r["om"] > 0) == (r["vision_dyaw_deg"] > 0)
        scales.append(s)
        flats.append(s * 1274.0)                # 等效"标称那一个数"的 c1
        print(f"  {r['tag']:>5} {r['cmd']:<24} "
              f"视觉Δψ={r['vision_dyaw_deg']:+8.2f}°  "
              f"里程计(标称c1)|Δψ|={math.degrees(dpsi_mag):7.2f}°  "
              f"比={s:6.4f}  方向{'✓' if dir_ok else '✗反了'}  "
              f"⇒ c1尺度 s={s:6.4f}  ⇒ c1_flat={s * 1274.0:7.1f}  "
              f"平移={r['shift_mm']:5.1f}mm")
    if scales:
        mean = statistics.fmean(scales)
        sd = statistics.stdev(scales) if len(scales) > 1 else 0.0
        print(f"  → 尺度因子 s: mean={mean:.4f}  sd={sd:.4f}  "
              f"峰峰={max(scales) - min(scales):.4f} "
              f"({(max(scales) - min(scales)) / mean * 100:.2f}% of mean)  "
              f"min={min(scales):.4f} max={max(scales):.4f}  n={len(scales)}")
        print(f"  → 等效 c1_flat: mean={statistics.fmean(flats):.1f}  "
              f"min={min(flats):.1f} max={max(flats):.1f}  "
              f"(标称 1274，偏差 {(statistics.fmean(flats) - 1274) / 1274 * 100:+.1f}%)")
        print(f"  → 若'尺度少报 2.7×'成立，s 应当 ≈ {1 / 2.7:.3f} 或 2.7 —— "
              f"实测 {mean:.3f}"
              f"{'，❌ 不支持（离得太远）' if min(abs(mean - 1 / 2.7), abs(mean - 2.7)) > 0.3 else ''}")
    print()
    return scales


def main():
    if not verify_formula():
        return 2
    here = Path(__file__).resolve().parent / "data"
    paths = sys.argv[1:] or [str(here / "rot_prev.jsonl"), str(here / "rot.jsonl")]
    all_s = []
    for p in paths:
        all_s += analyze(p)
    if all_s:
        print("=" * 78)
        print(f"全部偏航腿合并：n={len(all_s)}  s mean={statistics.fmean(all_s):.4f}  "
              f"sd={statistics.stdev(all_s) if len(all_s) > 1 else 0.0:.4f}  "
              f"峰峰={max(all_s) - min(all_s):.4f}")
        print(f"           等效 c1_flat mean={statistics.fmean(all_s) * 1274:.1f}  "
              f"（标称 1274）")
    return 0


if __name__ == "__main__":
    sys.exit(main())