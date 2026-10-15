#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 10 真机验收：跑一次 `Harness.drive_to`，把过程全量留证（不碰仓库代码）。

做三件 harness 本身不做、但真机验收必须有的事：

1. **命令审计**：包住 `CarLink.cmd`，把本会话发出的**每一条**命令记下来，
   事后校验"没有任何一条是无时长的裸 `M`"（brief 的硬约束）。
2. **每条脉冲的遥测**：`drive_to` 调 `_wait_done` 时不收样本，所以 IMU yaw 与
   编码器轮速在这里补收（不改变 harness 行为，只是把 `samples=` 传进去）。
   这是坑 2（"轮子在转、车没转"）唯一能查的证据。
3. **视觉 vs 里程计**：每条脉冲前后各一次 `locate()`（harness 自己就在做，
   trace 里有），把视觉位移与编码器位移并排 —— 两者背离即判该次作废。

用法（从 `实验/` 下跑）：
    cd 实验
    python3 task10_live/run_drive_to.py --tag H1 --target 0 0 --max-iter 3
    python3 task10_live/run_drive_to.py --tag S1 --max-iter 1 --pulse-ms 150 --target -100 -20

输出：stdout 一行 JSON（人读摘要另见 run 的终端打印）；追加到 `out/drive_log.jsonl`。
"""
from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
import time
import warnings
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPERIMENT_DIR = HERE.parent
ROOT = EXPERIMENT_DIR.parent
for _p in (str(EXPERIMENT_DIR), str(ROOT / "视觉测量" / "脚本")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from task10_live.live_link import open_link                 # noqa: E402
from vgt import odometry                                   # noqa: E402
from vgt.camera import CameraSession                        # noqa: E402
from vgt.harness import (                                   # noqa: E402
    DEFAULT_MAX_ITER, DEFAULT_PULSE_MS, DEFAULT_TOL_MM, Harness)
from vision_core import calib_store                         # noqa: E402

OUT_DIR = HERE / "out"

# 无时长命令的正则 —— 裸 M（`M f f f f`）在验收里必须为空。
_BARE_M = re.compile(r"^M\s")


def _telemetry_summary(samples):
    """一段 `D` 行 → 轮速/IMU yaw/里程计 位移。空列表返回 None（不假装有数据）。"""
    if not samples:
        return None
    enc0 = samples[0]["enc"]
    enc1 = samples[-1]["enc"]
    dt_ms = max(1, int(samples[-1]["t_ms"]) - int(samples[0]["t_ms"]))
    d_enc = [int(e1) - int(e0) for e0, e1 in zip(enc0, enc1)]
    # 轮速（rad/s）：d_enc 计数 / c_1rev 每圈计数 → 圈数 → rad / dt。
    omega = [d * 2.0 * math.pi / c * 1000.0 / dt_ms
             for d, c in zip(d_enc, (1276, 1270, 1276, 1274))]
    pwms = [tuple(s["pwm"]) for s in samples]
    pwm_mode = max(set(pwms), key=pwms.count) if pwms else None
    mags = [abs(w) for w in omega]
    mean_mag = statistics.fmean(mags) if mags else 0.0
    spread = ((max(mags) - min(mags)) / mean_mag * 100.0) if mean_mag > 1e-6 else None
    yaws = [float(s["rpy"][2]) for s in samples]
    imu_yaw_delta = yaws[-1] - yaws[0] if len(yaws) >= 2 else None
    out = {
        "n_d": len(samples), "dt_ms": dt_ms,
        "pwm_mode": list(pwm_mode) if pwm_mode else None,
        "omega_rad_s": [round(w, 2) for w in omega],
        "omega_abs_mean": round(mean_mag, 2),
        "omega_dispersion_pct": round(spread, 1) if spread is not None else None,
        "imu_yaw_delta_deg": round(imu_yaw_delta, 2) if imu_yaw_delta is not None else None,
    }
    try:
        traj = odometry.integrate_encoders(
            samples, c_1rev=(1276.0, 1270.0, 1276.0, 1274.0),
            sign=(-1.0, 1.0, -1.0, 1.0), l_xy_over_r=0.12825 / 0.0335, r_eff_mm=33.5)
        if len(traj):
            out["odo_dx_mm"] = round(float(traj[-1][0]), 1)
            out["odo_dy_mm"] = round(float(traj[-1][1]), 1)
            out["odo_dist_mm"] = round(math.hypot(float(traj[-1][0]), float(traj[-1][1])), 1)
            out["odo_yaw_deg"] = round(float(traj[-1][2]), 2)
    except ValueError as exc:
        out["odo_error"] = str(exc)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="跑一次 drive_to 并全量留证")
    ap.add_argument("--tag", default="run")
    ap.add_argument("--target", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--tol", type=float, default=DEFAULT_TOL_MM)
    ap.add_argument("--max-iter", type=int, default=DEFAULT_MAX_ITER)
    ap.add_argument("--pulse-ms", type=float, default=DEFAULT_PULSE_MS)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    sent: list[dict] = []
    telemetry: list[list] = []
    result: dict = {}

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam, open_link() as car:
            raw_cmd = car.cmd

            def audited_cmd(line, wait=False, timeout=2.0):
                sent.append({"t": round(time.monotonic(), 3), "line": line})
                return raw_cmd(line, wait=wait, timeout=timeout)

            car.cmd = audited_cmd

            h = Harness(car, cam, calib, home_xy=(0.0, 0.0))
            orig_wait = h._wait_done

            def capturing_wait(link, *, samples=None, on_sample=None):
                s: list = []
                out = orig_wait(link, samples=s, on_sample=on_sample)
                telemetry.append(s)
                return out

            h._wait_done = capturing_wait

            t0 = time.monotonic()
            result = h.drive_to(args.target[0], args.target[1], tol_mm=args.tol,
                                max_iter=args.max_iter, pulse_ms=args.pulse_ms,
                                settle_s=args.settle_s)
            result["wall_s"] = round(time.monotonic() - t0, 1)
        warns = [str(w.message) for w in caught]

    trace = result.get("trace", [])
    iters = []
    for i, rec in enumerate(trace):
        tel = _telemetry_summary(telemetry[i] if i < len(telemetry) else [])
        pos_before = rec.get("pos")
        pos_after = trace[i + 1]["pos"] if i + 1 < len(trace) else result.get("final")
        vis = None
        if pos_before is not None and pos_after is not None:
            vis = {"dx_mm": round(pos_after[0] - pos_before[0], 1),
                   "dy_mm": round(pos_after[1] - pos_before[1], 1),
                   "dist_mm": round(math.hypot(pos_after[0] - pos_before[0],
                                               pos_after[1] - pos_before[1]), 1),
                   "dyaw_deg": round(pos_after[2] - pos_before[2], 1)}
        iters.append({
            "iter": rec.get("iter"), "pos_before": pos_before, "pos_after": pos_after,
            "err_mm": [round(v, 1) for v in rec.get("err_mm", ())],
            "dist_mm": round(rec.get("dist_mm", 0.0), 1),
            "cmd": [round(v, 4) for v in rec.get("cmd", ())],
            "run_line": rec.get("run_line"), "run_reply": rec.get("run_reply"),
            "n_d": rec.get("n_d"), "outcome": rec.get("outcome"),
            "vision_step": vis, "telemetry": tel,
        })

    bare_m = [s["line"] for s in sent if _BARE_M.match(s["line"])]
    final = result.get("final")
    out = {
        "tag": args.tag, "note": args.note,
        "target_mm": list(args.target), "tol_mm": args.tol,
        "max_iter": args.max_iter, "pulse_ms": args.pulse_ms,
        "settle_s": args.settle_s,
        "reached": result.get("reached"), "reason": result.get("reason"),
        "iters_used": result.get("iters"), "final": final,
        "final_dist_mm": (round(math.hypot(final[0] - args.target[0],
                                           final[1] - args.target[1]), 1)
                          if final else None),
        "events": result.get("events"), "reset_lines": result.get("reset_lines"),
        "stop_confirmed": result.get("stop_confirmed"),
        "wall_s": result.get("wall_s"),
        "commands": [s["line"] for s in sent],
        "bare_M_commands": bare_m,
        "run_detail": iters,
        "warnings": warns,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / "drive_log.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())