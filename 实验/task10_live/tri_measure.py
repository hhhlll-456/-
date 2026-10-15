#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三方对照实弹：一条带时长的 RUN 前后各拍 N 帧，同时用**像素尺子**独立量位移。

三方 = ① 视觉管线（`build_field_frame`+`body_pose_in_field`，`drive_to` 用的就是它）
        ② 像素尺子（ID20→ID21 = 505mm 当尺子，只用像素，不经 PnP/内参）
        ③ 编码器里程计（`vgt.odometry`）

现场出现过 ①说 137mm、③说 61mm 的矛盾（差 2.25 倍）。本脚本的作用就是
在**同一次运动**上把三者摆在一起，判定谁对 —— 因为 `drive_to` 的收敛判据
只信 ①，① 的尺度错了，"收敛到 11mm" 就是假的。

用法（从 `实验/` 下跑）：
    cd 实验
    python3 task10_live/tri_measure.py --ms 400 --vx 0.1 --n 4
"""
from __future__ import annotations

import argparse
import json
import math
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
from vision_core import calib_store                        # noqa: E402
from vision_core.field import body_pose_in_field, build_field_frame   # noqa: E402
from vision_core.markers import detect_markers              # noqa: E402

OUT_DIR = HERE / "out"
FIELD_LEN_MM = 505.0


def _centers(poses):
    return {mid: (float(p.corners_px[:, 0].mean()), float(p.corners_px[:, 1].mean()))
            for mid, p in poses.items()}


def _sample(cam, calib, n):
    """连抓 n 帧，返回 (视觉位姿中位数, ID29 像素中位数, 尺子px/mm中位数, 失败计数)。"""
    vis, px, scale, miss = [], [], [], 0
    for _ in range(max(1, n)):
        poses = detect_markers(cam.grab(fresh=True), calib)
        c = _centers(poses)
        if 20 in c and 21 in c:
            d = math.hypot(c[21][0] - c[20][0], c[21][1] - c[20][1])
            if d > 1.0:
                scale.append(d / FIELD_LEN_MM)
        if all(b in poses for b in (20, 21, 22, 23)) and 29 in poses:
            ff = build_field_frame(poses)
            x, y, yaw = body_pose_in_field(ff, poses[29], calib["bias"])
            vis.append((x, y, yaw))
        if 29 in c:
            px.append(c[29])
        else:
            miss += 1
    med = lambda seq, i: (round(statistics.median([q[i] for q in seq]), 2) if seq else None)
    return ((med(vis, 0), med(vis, 1), med(vis, 2)) if vis else None,
            (med(px, 0), med(px, 1)) if px else None,
            round(statistics.median(scale), 5) if scale else None, miss)


def main() -> int:
    ap = argparse.ArgumentParser(description="视觉/像素尺子/编码器 三方对照")
    ap.add_argument("--ms", type=int, required=True)
    ap.add_argument("--vx", type=float, default=0.1)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--om", type=float, default=0.0)
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--tag", default="tri")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    sent = []
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam, open_link() as car:
            raw = car.cmd

            def audited(line, wait=False, timeout=2.0):
                sent.append(line)
                return raw(line, wait=wait, timeout=timeout)

            car.cmd = audited
            before = _sample(cam, calib, args.n)
            samples: list = []
            line = "RUN {:.4f} {:.4f} {:.4f} {}".format(args.vx, args.vy, args.om, args.ms)
            car.cmd(line)
            _outcome, stats = _wait_done(car, samples)
            time.sleep(args.settle_s)
            after = _sample(cam, calib, args.n)
            car.cmd("STOP")
        warns = [str(w.message) for w in caught]

    out = {
        "tag": args.tag, "run_line": line, "ms": args.ms,
        "commands": sent, "bare_M": [s for s in sent if s.startswith("M ")],
        "run_reply": stats.get("run_reply"), "outcome": _outcome, "n_d": stats.get("n_d"),
        "before": {"vision": before[0], "px": before[1], "px_per_mm": before[2],
                   "miss": before[3]},
        "after": {"vision": after[0], "px": after[1], "px_per_mm": after[2],
                  "miss": after[3]},
    }
    if before[0] and after[0]:
        out["vision_step_mm"] = round(math.hypot(after[0][0] - before[0][0],
                                                 after[0][1] - before[0][1]), 2)
        out["vision_dyaw_deg"] = round(after[0][2] - before[0][2], 2)
    if before[1] and after[1]:
        ppmm = statistics.median([before[2], after[2]])
        out["ruler_step_mm"] = round(math.hypot(after[1][0] - before[1][0],
                                                after[1][1] - before[1][1]) / ppmm, 2)
    if samples:
        try:
            traj = odometry.integrate_encoders(
                samples, c_1rev=(1276.0, 1270.0, 1276.0, 1274.0),
                sign=(-1.0, 1.0, -1.0, 1.0), l_xy_over_r=0.12825 / 0.0335, r_eff_mm=33.5)
            out["odometry_step_mm"] = round(math.hypot(float(traj[-1][0]),
                                                      float(traj[-1][1])), 2)
            out["odometry_dyaw_deg"] = round(float(traj[-1][2]), 2)
        except ValueError as exc:
            out["odometry_error"] = str(exc)
        yaws = [float(s["rpy"][2]) for s in samples]
        if len(yaws) >= 2:
            out["imu_dyaw_deg"] = round(yaws[-1] - yaws[0], 2)
    v, r = out.get("vision_step_mm"), out.get("ruler_step_mm")
    if v and r:
        out["vision_over_ruler"] = round(v / r, 3)
    e = out.get("odometry_step_mm")
    if r and e:
        out["ruler_over_odometry"] = round(r / e, 3)
    out["warnings"] = warns

    print(json.dumps(out, ensure_ascii=False, indent=1))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / "tri_log.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0


def _wait_done(link, samples):
    from vgt.harness import Harness
    h = Harness.__new__(Harness)          # 只借它的 _wait_done，不做 I/O 构造
    h.done_timeout_s, h.silence_timeout_s = 15.0, 4.0
    h.reset_lines = []
    return h._wait_done(link, samples=samples)


if __name__ == "__main__":
    sys.exit(main())
