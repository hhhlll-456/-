#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 10 收敛性重复验收：每轮先"打散"再归位，量收敛误差的分布。

为什么不能直接连跑 `drive_to(0,0)`：车已经在原点时它 0 次迭代就
`converged`（那不是收敛证据，只是"已经在窗口里"）。每轮必须先把车开到
**已知偏离**的位置再归位，才拿得到一条真的闭环轨迹。

每轮流程：
1. `pose_probe` 量当前位置（纯视觉，不连车）—— 出界/贴边就在这里看见；
2. 打散：`drive_to(scatter_xy, pulse_ms=P)`（同一条控制律，只换目标）；
3. 归位：`drive_to(home, pulse_ms=P)` → 记 `final_dist_mm` / `iters`；
4. 每轮之间量边界余量（`edge_clear_*`），<80 mm 就先归位再继续。

用法（从 `实验/` 下跑）：
    cd 实验
    python3 task10_live/convergence_trials.py --n 4 --pulse-ms 400 \
        --scatter "-95 0" --home "0 0"
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

from task10_live.live_link import open_link                 # noqa: E402
from vgt import matrix                                      # noqa: E402
from vgt.camera import CameraSession                        # noqa: E402
from vgt.harness import Harness                            # noqa: E402
from vision_core import calib_store                        # noqa: E402

OUT_DIR = HERE / "out"
NEAR_EDGE_MM = 80.0


def _pose(h, tries=3):
    for _ in range(tries):
        p = h.locate()
        if p is not None:
            return p
    return None


def _edge(pos):
    if pos is None:
        return None
    return (round(matrix.FIELD_HALF_LENGTH_MM - abs(pos[0]), 1),
            round(matrix.FIELD_HALF_WIDTH_MM - abs(pos[1]), 1))


def main() -> int:
    ap = argparse.ArgumentParser(description="归位收敛性重复验收")
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--pulse-ms", type=float, default=400.0)
    ap.add_argument("--scatter", type=float, nargs=2, default=(-95.0, 0.0),
                    metavar=("X", "Y"), help="打散目标（场地系 mm）")
    ap.add_argument("--home", type=float, nargs=2, default=(0.0, 0.0),
                    metavar=("X", "Y"), help="归位目标（场地系 mm）")
    ap.add_argument("--max-iter", type=int, default=12)
    ap.add_argument("--tol", type=float, default=12.0)
    ap.add_argument("--tag", default="conv")
    args = ap.parse_args()

    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    trials = []
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam, open_link() as car:
            h = Harness(car, cam, calib, home_xy=tuple(args.home))
            for k in range(1, args.n + 1):
                print(f"\n===== {args.tag} 第 {k}/{args.n} 轮 =====", flush=True)
                start = _pose(h)
                edge0 = _edge(start)
                print(f"  起点 {None if start is None else [round(v,1) for v in start]}"
                      f"  边界余量(x,y)={edge0}", flush=True)
                if start is None:
                    trials.append({"round": k, "error": "起点解不出位姿"})
                    break
                if edge0 and (edge0[0] < NEAR_EDGE_MM or edge0[1] < NEAR_EDGE_MM):
                    print(f"  ⚠ 起点贴边（<{NEAR_EDGE_MM:.0f}mm），先归位再继续", flush=True)
                    h.drive_to(args.home[0], args.home[1], tol_mm=args.tol,
                               max_iter=args.max_iter, pulse_ms=args.pulse_ms)
                    start = _pose(h)
                    edge0 = _edge(start)
                    print(f"  归位后 {None if start is None else [round(v,1) for v in start]}"
                          f"  边界余量={edge0}", flush=True)

                # 打散
                if math.hypot(start[0] - args.scatter[0], start[1] - args.scatter[1]) > 5.0:
                    sc = h.drive_to(args.scatter[0], args.scatter[1], tol_mm=args.tol,
                                    max_iter=args.max_iter, pulse_ms=args.pulse_ms)
                    print(f"  打散 → ({args.scatter[0]:.0f},{args.scatter[1]:.0f}):"
                          f" reached={sc['reached']} reason={sc['reason']}"
                          f" iters={sc['iters']}"
                          f" final={[round(v,1) for v in (sc['final'] or [])]}", flush=True)
                scattered = _pose(h)
                edge1 = _edge(scattered)
                print(f"  打散后 {None if scattered is None else [round(v,1) for v in scattered]}"
                      f"  边界余量={edge1}", flush=True)
                if scattered is None:
                    trials.append({"round": k, "error": "打散后解不出位姿",
                                   "locate_reason": h.last_locate_reason,
                                   "scatter_result": {kk: sc[kk] for kk in
                                                      ("reached", "reason", "iters", "final")}})
                    print(f"  ✗ 打散后 locate 连续失败：last_locate_reason="
                          f"{h.last_locate_reason}", flush=True)
                    break

                # 归位（真验收）
                res = h.drive_to(args.home[0], args.home[1], tol_mm=args.tol,
                                 max_iter=args.max_iter, pulse_ms=args.pulse_ms)
                end = _pose(h)
                edge2 = _edge(end)
                rec = {
                    "round": k, "scatter_target": list(args.scatter),
                    "home_target": list(args.home),
                    "start": [round(v, 1) for v in scattered],
                    "start_edge_clear": edge1,
                    "start_err_mm": round(math.hypot(scattered[0] - args.home[0],
                                                     scattered[1] - args.home[1]), 1),
                    "reached": res["reached"], "reason": res["reason"],
                    "iters": res["iters"], "stop_confirmed": res["stop_confirmed"],
                    "final": [round(v, 1) for v in res["final"]] if res["final"] else None,
                    "final_dist_mm": res["final_dist_mm"] if "final_dist_mm" in res else (
                        round(math.hypot(res["final"][0] - args.home[0],
                                         res["final"][1] - args.home[1]), 1)
                        if res["final"] else None),
                    "verify_pos": [round(v, 1) for v in end] if end else None,
                    "verify_dist_mm": (round(math.hypot(end[0] - args.home[0],
                                                        end[1] - args.home[1]), 1)
                                       if end else None),
                    "end_edge_clear": edge2,
                    "trace_dist_series": [round(r["dist_mm"], 1) for r in res["trace"]],
                    "trace_outcomes": [r["outcome"] for r in res["trace"]],
                    "trace_cmds": [[round(c, 4) for c in r["cmd"]] for r in res["trace"]],
                    "warnings": [],
                }
                trials.append(rec)
                print(f"  归位 → ({args.home[0]:.0f},{args.home[1]:.0f}):"
                      f" reached={rec['reached']} reason={rec['reason']}"
                      f" iters={rec['iters']} final={rec['final']}"
                      f" 独立复核={rec['verify_pos']} dist={rec['verify_dist_mm']}mm"
                      f" stop={rec['stop_confirmed']}", flush=True)
                print(f"  trace dist 序列: {rec['trace_dist_series']}", flush=True)
                print(f"  trace outcome: {rec['trace_outcomes']}", flush=True)
        warns = [str(w.message) for w in caught]

    ok = [t for t in trials if t.get("verify_dist_mm") is not None]
    summary = {
        "tag": args.tag, "pulse_ms": args.pulse_ms, "tol_mm": args.tol,
        "n_rounds": len(trials), "n_measured": len(ok),
        "reached_count": sum(1 for t in ok if t.get("reached")),
        "final_dist_mm": [t["verify_dist_mm"] for t in ok],
        "final_dist_mean_mm": (round(statistics.fmean(t["verify_dist_mm"] for t in ok), 2)
                               if ok else None),
        "final_dist_max_mm": (round(max(t["verify_dist_mm"] for t in ok), 2) if ok else None),
        "iters": [t["iters"] for t in ok],
        "stop_confirmed_all": all(t.get("stop_confirmed") for t in ok) if ok else None,
        "any_out_of_field": any(
            t.get("end_edge_clear") and (t["end_edge_clear"][0] < 0 or t["end_edge_clear"][1] < 0)
            for t in trials),
        "min_edge_clear_seen": min(
            [c for t in trials for c in (t.get("end_edge_clear") or []) if c is not None],
            default=None),
        "warnings": warns,
        "trials": trials,
    }
    print("\n===== 汇总 =====")
    print(json.dumps({kk: vv for kk, vv in summary.items() if kk != "trials"},
                     ensure_ascii=False, indent=1))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with (OUT_DIR / f"{args.tag}_trials.json").open("w", encoding="utf-8") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())