#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判"视觉和编码器谁对"：用**边界码自己当尺子**，量一条脉冲的真实位移。

现场出现过的矛盾：一条 400ms 脉冲，视觉说车走了 137mm，编码器说 61mm
（差 2.25 倍）。两者必有一个错，而 `drive_to` 的收敛判据**只看视觉** ——
所以必须先确认视觉的尺度是可信的，否则"收敛到 11mm"这个结论本身可疑。

## 尺子为什么可靠

四张边界码 ID20–23 贴在同一个平面上、位置固定，两张图里它们的**像素**间距
就是这把尺子的刻度。物理上 ID20→ID21 = 505mm（`matrix.FIELD_HALF_LENGTH_MM`
×2，已知量），于是

    位移(mm) = ID29 的像素位移 / (ID20→ID21 的像素距离) × 505

这条式子**只用像素**，不经过 PnP、不经过内参、不经过场地图 —— 尺度错误
（内参错、码尺寸错）**不会**影响它。所以它是独立于视觉管线的第三方尺子。

⚠ 车体是**架高**在场地平面之上的，透视会让它的像素位移与"贴地"物体不同
（离画面中心越远、架得越高，偏差越大）。所以本脚本同时报出车在画面里的
位置（离画面中心多远），供判断这个偏差能不能忽略。

用法（从 `实验/` 下跑）：
    cd 实验
    python3 task10_live/ruler_check.py --n 5
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import warnings
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPERIMENT_DIR = HERE.parent
ROOT = EXPERIMENT_DIR.parent
for _p in (str(EXPERIMENT_DIR), str(ROOT / "视觉测量" / "脚本")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cv2                                                   # noqa: E402
import numpy as np                                          # noqa: E402

from vgt.camera import CameraSession                        # noqa: E402
from vision_core import calib_store                         # noqa: E402
from vision_core.field import body_pose_in_field, build_field_frame   # noqa: E402
from vision_core.markers import detect_markers              # noqa: E402

OUT_DIR = HERE / "out"

FIELD_LEN_MM = 505.0        # ID20→ID21 的物理距离（matrix: 2×252.5）
FRAME_CENTER = (960.0, 540.0)   # 1920×1080 的画面中心


def _centers(poses):
    return {mid: (float(p.corners_px[:, 0].mean()), float(p.corners_px[:, 1].mean()))
            for mid, p in poses.items()}


def _shot(cam, calib):
    frame = cam.grab(fresh=True)
    poses = detect_markers(frame, calib)
    return frame, poses


def main() -> int:
    ap = argparse.ArgumentParser(description="像素尺子：视觉位移 vs 编码器位移")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--tag", default="ruler")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam:
            A, posesA = _shot(cam, calib)
            frames_A = [(A, posesA)]
            for _ in range(max(1, args.n) - 1):
                frames_A.append(_shot(cam, calib))
        warns = [str(w.message) for w in caught]

    # 尺子刻度：每帧量一次 ID20→ID21 的像素距离，取中位数
    scales = []
    for _f, p in frames_A:
        c = _centers(p)
        if 20 in c and 21 in c:
            d = math.hypot(c[21][0] - c[20][0], c[21][1] - c[20][1])
            if d > 1.0:
                scales.append(d)
    if not scales:
        print(json.dumps({"tag": args.tag, "ok": False,
                          "why": "量不到 ID20/ID21 的像素距离"}, ensure_ascii=False))
        return 2
    px_per_mm = statistics.median(scales) / FIELD_LEN_MM

    # 视觉管线自己报的位姿（build_field_frame + body_pose_in_field）
    field_poses = []
    for _f, p in frames_A:
        if all(b in p for b in (20, 21, 22, 23)) and 29 in p:
            ff = build_field_frame(p)
            x, y, yaw = body_pose_in_field(ff, p[29], calib["bias"])
            field_poses.append((x, y, yaw))

    out = {
        "tag": args.tag, "n_frames": len(frames_A),
        "px_per_mm": round(px_per_mm, 5),
        "ruler_px_id20_id21": [round(s, 1) for s in scales],
        "ruler_spread_px": round(max(scales) - min(scales), 2),
        # 车在画面里的位置（离画面中心越远，架高透视误差越大）
        "vehicle_px": [[round(v, 1) for v in _centers(p)[29]] for _f, p in frames_A if 29 in p],
        "vehicle_dist_from_frame_center_px": [
            round(math.hypot(_centers(p)[29][0] - FRAME_CENTER[0],
                             _centers(p)[29][1] - FRAME_CENTER[1]), 1)
            for _f, p in frames_A if 29 in p],
        "field_pose_mm": [[round(v, 1) for v in q] for q in field_poses],
        "field_pose_spread_mm": (
            {"x": round(max(q[0] for q in field_poses) - min(q[0] for q in field_poses), 2),
             "y": round(max(q[1] for q in field_poses) - min(q[1] for q in field_poses), 2)}
            if len(field_poses) >= 2 else None),
        "warnings": warns,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / "ruler_log.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
