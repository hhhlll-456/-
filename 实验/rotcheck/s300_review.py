#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R66 收尾判定实验：**一轮**单发 S300 式脉冲（300 ms 直行），复核 `run_trial` 的修复。

## 判据（用户给定，不许为了通过而放宽）

1. `odo_dist_mm ≈ vision_dist_mm`，**±10%** 内；
2. `dist_diff_pct` 从修复前的 **−62%** 塌到**个位数**；
3. **`c1` 一个数不动**（`ODOM_C_1REV = (1276, 1270, 1276, 1274)`，本脚本不碰）。

**一轮就够**，结果如何都如实报。

## 这一轮为什么是"一次实验"

它同时是对 `run_trial` 修复的端到端复核和对**同一个 c1** 的又一次独立测量：
同一个 `ODOM_C_1REV` 既是里程计积分用的标定常数，也是这次判据第 3 条的检查对象。
若修复只是把数字凑好看，c1 会被改动；这里比对提交前后的 `ODOM_C_1REV` 即可。

## 沉降时间为什么用 1.0 s（不是默认 0.4 s）

判据比的是**两个窗口**。`run_trial` 的里程计补读在终点帧**之后**，所以只有
"终点帧那一刻车已经滑停"两个窗口才对齐。默认 0.4 s 对 300 ms 脉冲**不够**
（停车是自由滑行，滑行量是通电量的 1.5~1.7 倍）。这是**测量有效性**的选择，
不是"调到通过"—— 沉降给短了只会让 odo 偏大（终点帧早于滑行结束），
判据第 1 条反而更难达成。两个方向都记下来。

## 安全

- 绝不发裸 `M`：`run_trial` 内部只发 `RUN <vx> <vy> <om> <ms>`（**带时长**）。
- 本轮选**后退**：起手 x≈+90 mm（PnP），后退 ~85 mm 落到 x≈0，两侧余量都很大。
  目标位移 ≈85 mm ≤ 200 mm 上限。
- `finally` 里永远 `STOP`，收尾再补一次 `PING` 确认板子活着。
"""
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path("/home/hanli/麦轮小车pid")
sys.path.insert(0, str(ROOT / "实验"))

from car_link import CarLink                              # noqa: E402
from vgt.camera import CameraSession                      # noqa: E402
from vgt.harness import Harness, ODOM_C_1REV, ODOM_SIGN, ODOM_R_EFF_MM  # noqa: E402
from vgt import compare, matrix                           # noqa: E402
from vision_core import load_calib                        # noqa: E402

OUT = Path("/tmp/opencode/c1cal/s300_review.json")
SETTLE_BEFORE = 0.5
SETTLE_AFTER = 1.0
MS = 300
VX = -0.200                      # 后退：起手在 x≈+90，后退落到场地中央


def locate_x5(h):
    xs, ys, yaws = [], [], []
    for _ in range(5):
        p = h.locate()
        if p:
            xs.append(p[0]); ys.append(p[1]); yaws.append(p[2])
    if not xs:
        return None
    return (sum(xs) / len(xs), sum(ys) / len(ys), sum(yaws) / len(yaws))


def main():
    rec = {"c1rev_used": list(ODOM_C_1REV), "sign": list(ODOM_SIGN),
           "r_eff_mm": ODOM_R_EFF_MM, "settle_before_s": SETTLE_BEFORE,
           "settle_after_s": SETTLE_AFTER, "ms": MS, "vx": VX}
    with CameraSession() as cam, CarLink(ip="192.168.1.115", auto_discover=False) as car:
        h = Harness(car, cam, load_calib(), home_xy=(0.0, 0.0))
        try:
            before = locate_x5(h)
            print(f"[before] {before}", flush=True)
            rec["loc_before"] = before
            if before is None:
                print("[abort] 视觉定位失败，不发 RUN。")
                return 2
            if abs(before[0]) > 130 or abs(before[1]) > 55:
                print("[abort] 起点离边界太近，不发 RUN。")
                return 2

            exp = matrix.Experiment(101, "straight", f"S300 后退 {MS}ms",
                                    VX, 0.0, 0.0, MS / 1000.0, 1)
            t0 = time.time()
            tr = h.run_trial(exp, settle_before_start_s=SETTLE_BEFORE,
                             settle_after_done_s=SETTLE_AFTER)
            wall = round(time.time() - t0, 1)
            after = locate_x5(h)
            rec["loc_after"] = after
            print(f"[after ] {after}   wall={wall}s", flush=True)

            summ = compare.trial_summary(tr)
            rec.update({"trial": {k: tr[k] for k in
                                  ("ok", "reason", "run_line", "run_ms",
                                   "vision", "vision_delta", "odometry_delta",
                                   "n_samples", "sign_ok", "sign_dot_mm2",
                                   "yaw_sign_ok", "coast_read_failed",
                                   "coast_counts", "stop_confirmed")},
                        "warnings": tr["warnings"],
                        "summary": summ, "samples": tr["samples"],
                        "loc_before": before, "loc_after": after, "wall_s": wall})

            v, o = summ["vision_dist_mm"], summ["odo_dist_mm"]
            print("\n" + "=" * 72)
            print(f"run_line          : {tr['run_line']}")
            print(f"ok / reason       : {tr['ok']} / {tr['reason']}")
            print(f"D 行              : {tr['n_samples']} 条")
            print(f"coast_read_failed : {tr['coast_read_failed']}   "
                  f"coast_counts={tr.get('coast_counts')}")
            print(f"warnings          : {tr['warnings']}")
            print(f"vision  Δ         : dist={summ['vision_dist_mm']} mm  "
                  f"yaw={summ['vision_yaw_deg']}°")
            print(f"odometry Δ        : dist={summ['odo_dist_mm']} mm  "
                  f"yaw={summ['odo_yaw_deg']}°")
            print(f"dist_diff_pct     : {summ['dist_diff_pct']:+.2f} %")
            print(f"sign_ok / dot     : {tr['sign_ok']} / {tr['sign_dot_mm2']}")
            print(f"c_1rev            : {list(ODOM_C_1REV)}")
            print("=" * 72)

            print("\n判据逐项：")
            vd, od, dd = summ["vision_dist_mm"], summ["odo_dist_mm"], summ["dist_diff_pct"]
            c1 = (abs(dd) <= 10.0) if dd is not None else False
            print(f"  ① |odo−vision|/vision ≤ 10%   → 实测 {abs(dd):.2f}%  "
                  f"{'✅' if c1 else '❌'}")
            c2 = dd is not None and abs(dd) < 10.0
            print(f"  ② dist_diff_pct 塌到个位数     → 实测 {dd:+.2f}%  "
                  f"{'✅' if c2 else '❌'}")
            c3 = list(ODOM_C_1REV) == [1276.0, 1270.0, 1276.0, 1274.0]
            print(f"  ③ c1 一个数不动               → {list(ODOM_C_1REV)}  "
                  f"{'✅' if c3 else '❌'}")
            rec["verdict"] = {"within_10pct": bool(c1), "single_digit": bool(c2),
                              "c1_unchanged": bool(c3)}
        finally:
            try:
                h._try_stop(car)
            except Exception as exc:
                print(f"[stop] {type(exc).__name__}: {exc}")
        print("[link] PING ->", car.cmd("PING", wait=True, timeout=3.0))

    OUT.write_text(json.dumps(rec, ensure_ascii=False, indent=2, default=str),
                   encoding="utf-8")
    print("[out]", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())