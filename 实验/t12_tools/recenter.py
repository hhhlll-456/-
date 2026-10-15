#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把车归位到场地中心并把车头摆正 —— chunk 之间 / 巡检发现越界时的安全兜底。

    cd 实验
    python3 t12_tools/recenter.py <tag> [x_mm] [y_mm]

只用 `vgt.harness` 的 `drive_to` / `align_yaw`：两条都只发**带时长**的
`RUN vx vy om <ms>`（固件到时自停），**绝不发裸 `M`**（`M` 没有超时，
今天因此把车跑飞过几分钟）。全部动作过视觉闭环；闭环失败就**停手**，
不猜位置、不盲开 —— 车在哪不知道时开电机等于盲开。

## ⚠ 实测到的两个坑（2026-10-15 Task 12 Step 4）

1. **车在 yaw ≈ −80° 时被 `drive_to` 拉回中心，运动方向与误差方向相反。**
   实测：起点 (−5.4, +17.5, −79.7°) → 目标 (0,0)，第 1 脉冲 `gained=+4.5 mm`
   （朝目标），第 2 脉冲 `gained=−35.6 mm`（**离目标更远**），3 次脉冲后车到
   (+6.8, +48.1) 并继续滑出相机视野（ID29 出画 → `locate()` 死锁 → 需人工搬回）。

   **根因未查明**（本轮车已失控、无法再复测）。候选：① yaw 口径在大幅度下 180° 翻转；
   ② P 律命令在这个方向上落进 MIN_DUTY 钳位、实际轮向与命令不一致；
   ③ 视觉位姿在车体大角度时解错。**不要把这个脚本的顺序当成已验证的修法。**

   本脚本因此把顺序改成**先摆头、再平移** —— 理由是**预防性**的：本轮所有
   收敛的 `drive_to` 都发生在 yaw≈0 附近（±19° 以内），大角度下只有那一次
   且失败了。让平移永远落在已验证过的区间里，是目前唯一不需要复测就能降低风险的做法。
2. `drive_to` 不判断"车停稳了没有"。上一次的 RUN 还在滑行时就发下一条脉冲，
   位移会叠加。`align_yaw` 每步都 STOP + 沉降 + 重新 `locate()`，比 `drive_to`
   的沉降长；这里在两者之间额外睡 `settle_s`。

参数与 Task 12 Step 4 全量跑批用的那组一致（`--pulse-ms 120 --coarse-ms 320
--tol-mm 4 --max-iter 24`），归位容差放宽到 6 mm：中心附近余量充足，
不必为 2 mm 的差别多烧脉冲。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_link import CarLink                                   # noqa: E402
from vgt.camera import CameraSession                           # noqa: E402
from vgt.harness import Harness                                # noqa: E402
from vision_core import load_calib                             # noqa: E402

LOG = Path(__file__).resolve().parent / "recenter.jsonl"

PULSE_MS = 120.0
COARSE_MS = 320.0
TOL_MM = 6.0
MAX_ITER = 24
YAW_TOL_DEG = 6.0
EXTRA_SETTLE_S = 1.0     # 上一次 RUN 的自由滑行必须滑停，否则位移会叠加


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    tag = argv[0] if argv else "recenter"
    x = float(argv[1]) if len(argv) > 1 else 0.0
    y = float(argv[2]) if len(argv) > 2 else 0.0
    out = {"tag": tag, "target": [x, y]}
    with CameraSession() as cam, CarLink() as car:
        h = Harness(car, cam, load_calib())

        # ① 先摆头 —— 必须在平移之前（见模块 docstring 的坑 1）
        yaw = h.align_yaw(0.0, tol_deg=YAW_TOL_DEG)
        out["yaw"] = {k: yaw.get(k) for k in
                      ("converged", "steps", "reason", "final_yaw", "stop_confirmed")}
        out["yaw_trace"] = [{k: t.get(k) for k in ("step", "yaw", "err_deg", "cmd", "outcome")}
                            for t in (yaw.get("trace") or [])]
        time.sleep(EXTRA_SETTLE_S)
        out["yaw"]["ok"] = bool(yaw.get("converged"))
        if not out["yaw"]["ok"]:
            # 摆头都定位不了 = 车已经看不见了；平移只会盲开，直接收手。
            out["skipped_drive_to"] = "align_yaw 未收敛，车已定位不了，不盲开"
            print(json.dumps(out, ensure_ascii=False, indent=2))
            LOG.open("a").write(json.dumps(out, ensure_ascii=False) + "\n")
            return 1

        # ② 再平移
        home = h.drive_to(x, y, pulse_ms=PULSE_MS, coarse_pulse_ms=COARSE_MS,
                          tol_mm=TOL_MM, max_iter=MAX_ITER)
        out["home"] = {k: home.get(k) for k in
                       ("reached", "at_target", "moved", "iters", "reason",
                        "start_dist_mm", "final", "stop_confirmed")}
        # trace 落全字段：R72 之后 `d`/`ms` 不是独立键，在 `dist_mm` 与 `cmd` 里。
        out["home_trace"] = [
            {"iter": t.get("iter"), "pos": t.get("pos"),
             "err_mm": t.get("err_mm"), "dist_mm": t.get("dist_mm"),
             "cmd": t.get("cmd"), "gained_mm": t.get("gained_mm"),
             "outcome": t.get("outcome"), "stuck_run": t.get("stuck_run")}
            for t in (home.get("trace") or [])]
        out["home"]["ok"] = bool(home.get("reached") and home.get("moved"))
        car.stop()
    print(json.dumps(out, ensure_ascii=False, indent=2))
    LOG.open("a").write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0 if out["home"]["ok"] and out["yaw"]["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())