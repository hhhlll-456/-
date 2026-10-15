#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单条 `RUN <vx> <vy> <om> <ms>` 脉冲的三方对照：视觉 / 编码器 / IMU。

**所有命令都带时长**（RUN 的第 4 个字段是必填的 ms），不发裸 `M`。
每条命令经 `CarLink.cmd` 审计后落盘，事后可查。

三方对照的用途（Task 10 真机验收）：
- `drive_to` 的收敛与否取决于"一条脉冲走多远"（步长）与 `tol_mm` 的关系；
- 坑 2（轮子在转、车没转）只有把 **ID29 位姿变化** 和 **IMU yaw 变化** 放在一起
  看才能发现 —— 单看编码器永远正常。

用法（从 `实验/` 下跑）：
    cd 实验
    python3 task10_live/pulse_probe.py --tag P1 --vx 0.1149 --vy 0 --om 0 --ms 400
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
from vgt.harness import Harness                            # noqa: E402
from vision_core import calib_store                        # noqa: E402

OUT_DIR = HERE / "out"
_BARE_M = re.compile(r"^M\s")


def main() -> int:
    ap = argparse.ArgumentParser(description="单条带时长 RUN 脉冲 + 三方对照")
    ap.add_argument("--tag", default="pulse")
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--om", type=float, default=0.0)
    ap.add_argument("--ms", type=int, required=True, help="脉冲时长（ms）—— 必填，禁止裸 M")
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    sent, samples = [], []
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam, open_link() as car:
            raw_cmd = car.cmd

            def audited_cmd(line, wait=False, timeout=2.0):
                sent.append(line)
                return raw_cmd(line, wait=wait, timeout=timeout)

            car.cmd = audited_cmd
            h = Harness(car, cam, calib)

            before = h.locate()
            run_line = "RUN {:.4f} {:.4f} {:.4f} {}".format(args.vx, args.vy, args.om, args.ms)
            t0 = time.monotonic()
            car.cmd(run_line)
            outcome, stats = h._wait_done(car, samples=samples)
            wall = time.monotonic() - t0
            time.sleep(args.settle_s)
            after = h.locate()
            stopped = h._try_stop(car)
        warns = [str(w.message) for w in caught]

    rec = {
        "tag": args.tag, "note": args.note, "run_line": run_line,
        "outcome": outcome, "run_reply": stats.get("run_reply"),
        "n_d": stats.get("n_d"), "wall_s": round(wall, 2), "ms": args.ms,
        "cmd_mps_or_rads": [args.vx, args.vy, args.om],
        "before": before, "after": after, "stop_confirmed": stopped,
        "commands": sent, "bare_M_commands": [s for s in sent if _BARE_M.match(s)],
        "warnings": warns,
        "locate_reason_after": h.last_locate_reason,
    }
    if before and after:
        dx, dy = after[0] - before[0], after[1] - before[1]
        rec["vision"] = {
            "dx_mm": round(dx, 1), "dy_mm": round(dy, 1),
            "dist_mm": round(math.hypot(dx, dy), 1), "dyaw_deg": round(after[2] - before[2], 2),
        }
    if samples:
        enc_d = [int(b) - int(a) for a, b in zip(samples[0]["enc"], samples[-1]["enc"])]
        dt_ms = max(1, int(samples[-1]["t_ms"]) - int(samples[0]["t_ms"]))
        omega = [d * 2.0 * math.pi / c * 1000.0 / dt_ms
                 for d, c in zip(enc_d, (1276, 1270, 1276, 1274))]
        yaws = [float(s["rpy"][2]) for s in samples]
        mags = [abs(w) for w in omega]
        rec["telemetry"] = {
            "n_d": len(samples), "dt_ms": dt_ms, "enc_delta": enc_d,
            "pwm_first": samples[0]["pwm"], "pwm_last": samples[-1]["pwm"],
            "omega_rad_s": [round(w, 2) for w in omega],
            "omega_abs_mean": round(statistics.fmean(mags), 2),
            "omega_dispersion_pct": (round((max(mags) - min(mags)) / statistics.fmean(mags) * 100, 1)
                                     if statistics.fmean(mags) > 1e-6 else None),
            "imu_yaw_delta_deg": round(yaws[-1] - yaws[0], 2) if len(yaws) >= 2 else None,
        }
        try:
            traj = odometry.integrate_encoders(
                samples, c_1rev=(1276.0, 1270.0, 1276.0, 1274.0),
                sign=(-1.0, 1.0, -1.0, 1.0), l_xy_over_r=0.12825 / 0.0335, r_eff_mm=33.5)
            rec["odometry"] = {
                "dx_mm": round(float(traj[-1][0]), 1), "dy_mm": round(float(traj[-1][1]), 1),
                "dist_mm": round(math.hypot(float(traj[-1][0]), float(traj[-1][1])), 1),
                "yaw_deg": round(float(traj[-1][2]), 2),
            }
        except ValueError as exc:
            rec["odometry_error"] = str(exc)
        if "vision" in rec and "odometry" in rec:
            rec["vision_over_odometry"] = (
                round(rec["vision"]["dist_mm"] / rec["odometry"]["dist_mm"], 2)
                if rec["odometry"]["dist_mm"] > 0.5 else None)

    print(json.dumps(rec, ensure_ascii=False, indent=1))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / "pulse_log.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())