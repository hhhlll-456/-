#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只拍照的诊断：列出这一帧检到哪些 ID、车载码 ID29 在图像里的朝向。

**不发任何命令、不连车**（只开相机）。用于：
- `locate` 报 `boundary_moved` 时分诊"是掉码了还是码被车挡住了"；
- 车跑偏到视野边缘、闭环已经没法用时，人工接管前先量一下车头朝哪。

输出里 `id29_px_dir_field` 是把 ID29 的车头方向换算到**场地系**（+x = 图像右、
+y = 图像下，heading 逆时针为正 ⇒ 车头 = (cosθ, −sinθ)）的估计值。
"""
from __future__ import annotations

import argparse
import json
import math
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
from vision_core import calib_store                       # noqa: E402
from vision_core.markers import detect_markers            # noqa: E402

BOUNDARY = (20, 21, 22, 23)
VEHICLE = 29


def main() -> int:
    ap = argparse.ArgumentParser(description="只拍照：检到哪些码 + 车头朝向")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--save", default="")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    found_sets, vehicles = [], []
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam:
            for _ in range(max(1, args.n)):
                frame = cam.grab(fresh=True)
                poses = detect_markers(frame, calib)
                found_sets.append(sorted(poses))
                p = poses.get(VEHICLE)
                if p is not None:
                    c = p.corners_px
                    cx = sum(float(pt[0]) for pt in c) / 4.0
                    cy = sum(float(pt[1]) for pt in c) / 4.0
                    others = sorted(i for i in poses if i in BOUNDARY)
                    # 规范 +X = 角点0→角点1；规范 +Y = 角点3→角点0（markers.py 口径，
                    # 注意是 3→0，**不是** 0→3 —— 写反会让车头方向整个翻掉）
                    ax = (float(c[1][0]) - float(c[0][0]), float(c[1][1]) - float(c[0][1]))
                    ay = (float(c[0][0]) - float(c[3][0]), float(c[0][1]) - float(c[3][1]))
                    norm = lambda v: math.hypot(*v) or 1.0
                    ax = (ax[0] / norm(ax), ax[1] / norm(ax))
                    ay = (ay[0] / norm(ay), ay[1] / norm(ay))
                    # 图像坐标 y 向下；场地系 +y 也向下，+x 向右。
                    # 车体系 X_body=(cosθ,−sinθ)、Y_body=(−sinθ,−cosθ)，
                    # 由车头在图像里的方向反解 θ：图像向量 (px,py) → 场地 (px, py)
                    # 车头 = a·X_body + b·Y_body。
                    # 直接用 a,b 组成车体系速度指令即可（见 body_command）。
                    # 车头 = (sin d, cos d)，d = dtheta（calib bias），
                    # 表达在**码的规范系**里；再把码轴旋到场地系（图像 x 右 / y 下
                    # 即场地 +x / +y，于是图像向量可直接当场地方向用）。
                    dth = math.radians(float(calib["bias"]["dtheta_deg"]))
                    fwd = (math.sin(dth) * ax[0] + math.cos(dth) * ay[0],
                           math.sin(dth) * ax[1] + math.cos(dth) * ay[1])
                    nf = math.hypot(*fwd) or 1.0
                    fwd = (fwd[0] / nf, fwd[1] / nf)
                    vehicles.append({"cx": round(cx, 1), "cy": round(cy, 1),
                                     "x_axis_px": [round(v, 4) for v in ax],
                                     "y_axis_px": [round(v, 4) for v in ay],
                                     "car_forward_field": [round(v, 4) for v in fwd],
                                     "car_forward_deg_below_horiz":
                                         round(math.degrees(math.atan2(fwd[1], fwd[0])), 1),
                                     "dtheta_deg": calib["bias"]["dtheta_deg"],
                                     "boundary_px": {str(i): [
                                         round(float(poses[i].corners_px[:, 0].mean()), 1),
                                         round(float(poses[i].corners_px[:, 1].mean()), 1)]
                                         for i in others},
                                     "rms_px": round(p.reproj_rms_px, 2)})
            if args.save:
                cam.save(args.save)
        warns = [str(w.message) for w in caught]

    out = {
        "n": len(found_sets), "found_ids_per_frame": found_sets,
        "boundary_always_found": all(all(b in s for b in BOUNDARY) for s in found_sets),
        "missing_boundary": sorted({b for s in found_sets for b in BOUNDARY if b not in s}),
        "vehicle_found": len(vehicles), "vehicle": vehicles,
        "warnings": warns,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
