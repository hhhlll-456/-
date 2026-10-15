#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检：量车在场地上"哪儿"（**车体原点**口径）+ 各码检出率 + 离边界余量。

    cd 实验
    python3 t12_tools/carpos.py <tag> [帧数]

## 为什么必须用车体原点口径

安全判据（离边界余量、车体包络）算的是**车体原点**，不是 ID29 码的位置。
两者差一个"码 → 车体原点"的固定偏置（本车实测约 83 mm，且跟着车头转）——
用 ID29 的 `tvec` 当车位置会**系统性偏掉 80 mm 以上**，据此判"车快出界了"是假的，
据此判"车在场中央"也是假的。本脚本因此走 `body_pose_in_field`，与
`vgt.harness.Harness.locate()` **同一个函数**。

## 两条余量，含义不同，别混

- `margin_point_mm`：车心到场地边的距离 —— **简报给的"离边界 < 80 mm 就暂停"就是这条**。
- `margin_envelope_mm`：车体矩形四角到场地边的最小距离（yaw 感知，半包络 135 × 74.5 mm）。
  车 y 向半包络就有 85.5 mm > 场地半宽 125.9 mm，即使死位在中心也只有 ~39 mm 余量，
  所以 80 mm 那条规则**不可能**是量包络。包络只用来判"整车有没有越线"（< 0）。

只拍照，不连车、不发任何命令。
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "视觉测量" / "脚本"))
sys.path.insert(0, str(ROOT / "实验"))

import cv2                                                    # noqa: E402
from vision_core import calib_store                           # noqa: E402
from vision_core.field import body_pose_in_field, build_field_frame   # noqa: E402
from vision_core.markers import detect_markers               # noqa: E402
from vgt.camera import CameraSession                          # noqa: E402

VEHICLE_ID = 29
BOUNDARY_IDS = (20, 21, 22, 23)
# 实测场地边界（场地系 mm，Task 12 简报给定）：x∈[−252.5,252.5]、y∈[−125.9,125.9]
FIELD_X = 252.5
FIELD_Y = 125.9
# 车体半包络（与 vgt/matrix.py 的 CAR_HALF_LENGTH_MM / CAR_HALF_WIDTH_MM 同值）
BODY_HALF_L = 135.0
BODY_HALF_W = 74.5
# 简报的暂停阈值：车心到边界 < 80 mm
POINT_MARGIN_MIN_MM = 80.0
LOG = Path(__file__).resolve().parent / "carpos.jsonl"


def envelope_margin_mm(x, y, yaw):
    """车体矩形四角到场地边的最小距离（mm；< 0 = 整车越线）。"""
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    m = math.inf
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        px = x + sx * BODY_HALF_L * c - sy * BODY_HALF_W * s
        py = y + sx * BODY_HALF_L * s + sy * BODY_HALF_W * c
        m = min(m, FIELD_X - abs(px), FIELD_Y - abs(py))
    return m


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    tag = argv[0] if argv else "pos"
    n = int(argv[1]) if len(argv) > 1 else 9
    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    xs, ys, angs = [], [], []
    hits = {i: 0 for i in BOUNDARY_IDS}
    hits[VEHICLE_ID] = 0
    n_id29 = 0
    last = None
    with CameraSession() as cam:
        for i in range(n):
            frame = cam.grab(fresh=True)
            last = frame
            poses = detect_markers(frame, calib)
            for mid in hits:
                if mid in poses:
                    hits[mid] += 1
            missing = [b for b in BOUNDARY_IDS if b not in poses]
            if missing or VEHICLE_ID not in poses:
                why = f"缺边界码 {missing}" if missing else "缺 ID29"
                print(f"  [{tag}] #{i} {why}（该帧作废）")
                continue
            try:
                ff = build_field_frame(poses)
                x, y, yaw = body_pose_in_field(ff, poses[VEHICLE_ID], calib["bias"])
            except ValueError as exc:
                print(f"  [{tag}] #{i} 位姿解算失败：{exc}")
                continue
            n_id29 += 1
            xs.append(float(x))
            ys.append(float(y))
            angs.append(float(yaw))
    if last is not None:
        out_jpg = ROOT / "视觉测量" / "结果" / "captures" / f"carpos_{tag}.jpg"
        try:
            out_jpg.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_jpg), last)
        except Exception as exc:                       # noqa: BLE001 - 存图失败不阻断巡检
            print(f"  ⚠ 存图失败：{exc}")
    out = {"tag": tag, "n_frames": n, "n_id29": n_id29,
           "hits": {str(k): v for k, v in hits.items()}}
    if not xs:
        out["verdict"] = "ID29_DEADLOCK"
        print(json.dumps(out, ensure_ascii=False, indent=2))
        LOG.open("a").write(json.dumps(out, ensure_ascii=False) + "\n")
        return 2
    x, y, yaw = (statistics.fmean(xs), statistics.fmean(ys), statistics.fmean(angs))
    margin_pt = min(FIELD_X - abs(x), FIELD_Y - abs(y))
    margin_env = envelope_margin_mm(x, y, yaw)
    out.update({
        "body_x_mm": round(x, 2), "body_y_mm": round(y, 2),
        "body_yaw_deg": round(yaw, 2),
        "body_spread_mm": [round(max(xs) - min(xs), 2), round(max(ys) - min(ys), 2)],
        "yaw_spread_deg": round(max(angs) - min(angs), 2),
        "margin_point_mm": round(margin_pt, 1),
        "margin_envelope_mm": round(margin_env, 1),
        "verdict": ("OUT_OF_FIELD" if margin_env < 0 else
                    "DEADLOCK" if n_id29 < 0.75 * n else
                    "OK" if margin_pt >= POINT_MARGIN_MIN_MM else "CHECK"),
    })
    print(json.dumps(out, ensure_ascii=False, indent=2))
    LOG.open("a").write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())