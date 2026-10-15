#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测**真实稳态误差**：归位停下来之后，绕着车连抓 N 帧看它到底离目标多远。

为什么必须单独测：`drive_to` 的 `reached` 是"误差 ≤ tol"就算过，所以
**收敛到什么位置完全由 tol 决定，而不是由控制律决定**。只要步长 < tol，
它就会一直走到"刚好进窗口"为止 —— 实测稳态误差会紧贴 tol 的上沿，
看起来"收敛得很好"，其实是判据在替控制律兜底。

这个脚本在归位之后**不再发任何命令**，纯视觉连抓 N 帧：
- `settle_dist_mm` = 各帧到目标的距离（中位数 + 极差）；
- 与 `tol_mm` 一起看，得到"余量倍数 = tol / 实测稳态误差"。

用法（从 `实验/` 下跑）：
    cd 实验
    python3 task10_live/settle_error.py --target 0 0 --n 12
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

from vgt.camera import CameraSession                        # noqa: E402
from vgt.harness import Harness                            # noqa: E402
from vision_core import calib_store                        # noqa: E402

OUT_DIR = HERE / "out"


def main() -> int:
    ap = argparse.ArgumentParser(description="纯视觉测稳态误差（不发命令）")
    ap.add_argument("--target", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--tol", type=float, default=12.0)
    ap.add_argument("--tag", default="settle")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    dists, poses, reasons = [], [], {}
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam:
            h = Harness(None, cam, calib)
            for _ in range(args.n):
                p = h.locate()
                if p is None:
                    r = h.last_locate_reason or "unknown"
                    reasons[r] = reasons.get(r, 0) + 1
                    continue
                poses.append([round(v, 2) for v in p])
                dists.append(math.hypot(p[0] - args.target[0], p[1] - args.target[1]))
        warns = [str(w.message) for w in caught]

    out = {
        "tag": args.tag, "target_mm": list(args.target), "tol_mm": args.tol,
        "n_requested": args.n, "n_ok": len(dists), "locate_failures": reasons,
        "settle_dist_mm": [round(d, 2) for d in dists],
        "settle_median_mm": (round(statistics.median(dists), 2) if dists else None),
        "settle_mean_mm": (round(statistics.fmean(dists), 2) if dists else None),
        "settle_max_mm": (round(max(dists), 2) if dists else None),
        "settle_spread_mm": (round(max(dists) - min(dists), 2) if dists else None),
        "margin_tol_over_median": (round(args.tol / statistics.median(dists), 2)
                                   if dists and statistics.median(dists) > 0 else None),
        "inside_tol_all": bool(dists) and all(d <= args.tol for d in dists),
        "poses": poses, "warnings": warns,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / "settle_log.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(out, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
