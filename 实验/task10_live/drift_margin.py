#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量 R18 的 15 px 漂移阈值离"自然抖动"有多远（Task 10 清单 5.5 的子问题①）。

**纯视觉：只开相机，不连车、不发任何命令。** 车可以完全静止。

为什么必须单独量：`Harness._check_boundary_drift` 拿**每一帧**的
`FieldFrame.marker_px`（= 4 个角点图像坐标的均值）与会话基线比欧氏距离，
**任一码 > 15 px 就抛 `BoundaryMarkersMoved`**。可是这个量本身带相机噪声
（自动对焦呼吸、反光、传感器噪声）—— 若自然抖动本来就接近 15 px，
那 5.5 的"漂移门禁"就会**误报**，判据得重定。本脚本给出抖动上限。

口径与 harness 逐字对齐：
- 取量：`poses[mid].corners_px[:, 0].mean()` / `[:, 1].mean()`（四角点均值）；
- 基线：**第一个**"四码齐 + 建得出场地系 + 车码解得出"的帧（同 `locate()`）；
- 比较：与基线同量的欧氏距离（`math.hypot`），严格 `> 15` 才算超限。

用法（从 `实验/` 下跑，`vgt` 才在 sys.path 上）：
    cd 实验
    python3 task10_live/drift_margin.py --n 20
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

from vgt.camera import CameraSession                      # noqa: E402
from vgt.harness import BOUNDARY_DRIFT_TOL_PX             # noqa: E402
from vision_core import calib_store                       # noqa: E402
from vision_core.field import build_field_frame           # noqa: E402
from vision_core.markers import detect_markers            # noqa: E402

BOUNDARY = tuple(calib_store.BOUNDARY_IDS)
VEHICLE = int(calib_store.VEHICLE_ID)


def _marker_px(pose) -> tuple[float, float]:
    """与 `FieldFrame.marker_px` 同一口径：4 个角点图像坐标的均值。"""
    return (float(pose.corners_px[:, 0].mean()), float(pose.corners_px[:, 1].mean()))


def main() -> int:
    ap = argparse.ArgumentParser(description="量 15 px 漂移阈值的自然抖动余量")
    ap.add_argument("--n", type=int, default=20, help="抓帧数（默认 20）")
    ap.add_argument("--tol", type=float, default=None, help="覆盖阈值（默认用 harness 的常量）")
    args = ap.parse_args()

    tol = BOUNDARY_DRIFT_TOL_PX if args.tol is None else float(args.tol)
    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)

    baseline: dict[int, tuple[float, float]] | None = None
    devs: dict[int, list[float]] = {mid: [] for mid in BOUNDARY}
    missing: dict[int, int] = {mid: 0 for mid in BOUNDARY}
    n_frames = 0          # 四码齐 + 建得出场地系的帧
    n_baseline_frame = 0  # 基线取自第几帧（1-based，0 = 没抓到）
    warns: list[str] = []

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam:
            for i in range(max(1, args.n)):
                frame = cam.grab(fresh=True)
                poses = detect_markers(frame, calib)
                for mid in BOUNDARY:
                    if mid not in poses:
                        missing[mid] += 1
                if any(mid not in poses for mid in BOUNDARY):
                    continue
                try:
                    ff = build_field_frame(poses)
                except ValueError:
                    continue
                n_frames += 1
                if baseline is None:
                    if VEHICLE not in poses:
                        # 同 locate()：车码没解出来就不抓基线，等下一帧
                        continue
                    baseline = {mid: (float(px[0]), float(px[1]))
                                for mid, px in ff.marker_px.items()}
                    n_baseline_frame = i + 1
                    continue
                for mid, px in ff.marker_px.items():
                    dx = float(px[0]) - baseline[mid][0]
                    dy = float(px[1]) - baseline[mid][1]
                    devs[mid].append(math.hypot(dx, dy))
        warns = [str(w.message) for w in caught]

    per_id: dict[str, dict[str, float]] = {}
    worst = 0.0
    for mid in BOUNDARY:
        d = devs[mid]
        if not d:
            per_id[str(mid)] = {"n": 0}
            continue
        m = max(d)
        worst = max(worst, m)
        per_id[str(mid)] = {
            "n": len(d),
            "mean_px": round(statistics.fmean(d), 2),
            "median_px": round(statistics.median(d), 2),
            "std_px": round(statistics.pstdev(d), 2),
            "max_px": round(m, 2),
        }

    out = {
        "tol_px": tol,
        "n_requested": args.n,
        "n_frames_four_codes": n_frames,
        "baseline_frame": n_baseline_frame,
        "n_compared": (len(devs[BOUNDARY[0]]) if devs[BOUNDARY[0]] else 0),
        "worst_dev_px": round(worst, 2),
        "margin_px": round(tol - worst, 2),
        "margin_x": (round(tol / worst, 2) if worst > 0 else None),
        "per_id": per_id,
        "missing_frames": {str(k): v for k, v in missing.items()},
        "passes_tol": bool(worst <= tol),
        "warnings": warns,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
