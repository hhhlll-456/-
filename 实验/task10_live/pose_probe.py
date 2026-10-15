#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量车在场地上"哪儿" —— 纯视觉，不连车、不发任何命令（Task 10 真机验收的安全闸）。

用 `Harness.locate()` 的**同一条代码路径**（`vgt.harness.Harness`）解算，所以
报出来的数与 `drive_to` 闭环里用的是同一个口径（同一套 bias、同一套场地系、
同一套边界码漂移基线）。只连相机、不建 TCP 连接。

用法（从 `实验/` 下跑，`vgt` 才在 sys.path 上）：
    cd 实验
    python3 task10_live/pose_probe.py --tag before_homing1 --n 5

输出：stdout 一行 JSON；追加到 `实验/task10_live/out/pose_log.jsonl`。
判"出界/贴边"用 `edge_clear_x_mm` / `edge_clear_y_mm`：车中心到 x / y 场地边界的
距离（边界在 ±252.5 / ±125.9 mm）。车体半包络 135 / 74.5 mm ⇒ 车**完整**在场地内
需要 clear_x ≥ 135 且 clear_y ≥ 74.5。
"""
from __future__ import annotations

import argparse
import json
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

from vgt import matrix                                    # noqa: E402
from vgt.camera import CameraSession                      # noqa: E402
from vgt.harness import Harness                           # noqa: E402
from vision_core import calib_store                       # noqa: E402

OUT_DIR = HERE / "out"

# 车中心到场地边界 < 该值就当"贴边了"，先归位再继续（brief 给的 80 mm）。
NEAR_EDGE_MM = 80.0


def main() -> int:
    ap = argparse.ArgumentParser(description="纯视觉量车位置（不连车）")
    ap.add_argument("--tag", default="probe")
    ap.add_argument("--n", type=int, default=5, help="抓几帧取中位数")
    ap.add_argument("--save", default="", help="顺便存一张 jpg（相对仓库根）")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    rows, missing = [], {}
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam:
            # 同一个 Harness 实例 ⇒ 同一个边界码基线；只做一次 locate 也行，
            # 这里 n 帧用来量化重复性（R49 说单帧重复性 ~1.2 mm）。
            h = Harness(None, cam, calib)
            for _ in range(max(1, args.n)):
                pos = h.locate()
                if pos is None:
                    missing[h.last_locate_reason or "unknown"] = \
                        missing.get(h.last_locate_reason or "unknown", 0) + 1
                else:
                    rows.append({"x": pos[0], "y": pos[1], "yaw": pos[2]})
            if args.save:
                cam.save(args.save)
        warns = [str(w.message) for w in caught]

    if not rows:
        print(json.dumps({"tag": args.tag, "ok": False, "missing": missing,
                          "warnings": warns}, ensure_ascii=False))
        return 2

    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    yaws = [r["yaw"] for r in rows]
    x, y = statistics.median(xs), statistics.median(ys)
    clear_x = matrix.FIELD_HALF_LENGTH_MM - abs(x)
    clear_y = matrix.FIELD_HALF_WIDTH_MM - abs(y)
    out = {
        "tag": args.tag, "ok": True, "n": len(rows), "n_missing": missing,
        "x_mm": round(x, 2), "y_mm": round(y, 2),
        "yaw_deg": round(statistics.median(yaws), 2),
        "x_spread_mm": round(max(xs) - min(xs), 2),
        "y_spread_mm": round(max(ys) - min(ys), 2),
        "edge_clear_x_mm": round(clear_x, 1), "edge_clear_y_mm": round(clear_y, 1),
        "near_edge": bool(clear_x < NEAR_EDGE_MM or clear_y < NEAR_EDGE_MM),
        "fully_inside": bool(clear_x >= matrix.CAR_HALF_LENGTH_MM
                             and clear_y >= matrix.CAR_HALF_WIDTH_MM),
        "warnings": warns,
    }
    print(json.dumps(out, ensure_ascii=False))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / "pose_log.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())