#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查"静默假数据"：`drive_to` 在几种**失败**情形下，返回值长得像不像成功。

这是 Task 10 真机验收里最重要的一条 —— 一个"失败不报错、只返回看起来
正常的结果"的闭环，比不收敛危险得多（Task 12 会把每一行都写进表，
假绿会一路流到结论里）。

## 手法

复用 `vgt/tests/test_harness.py` 里现成的合成场景 `scene()` 与假链路
（同一条 `build_field_frame` / `body_pose_in_field` 真管线，只把
`detect_markers` 换成脚本化替身），所以这里验的是**仓库里的那份代码**，
不是副本。

## 查的几种"看起来该成功"的情形

| # | 注入的情形 | 期望 |
|---|---|---|
| 1 | 目标一开始就满足（不挪车直接归位） | `reached=True` 且 **`iters==0`、一条 RUN 都不发** |
| 2 | 视觉中途整段解不出（车压住边界码） | `reached=False`、`reason="locate_failed"`、无 `final` |
| 3 | 边界码被碰（R18 漂移超限） | 抛 `BoundaryMarkersMoved` |
| 4 | 归位目标越界（车会撞码/撞桌沿） | 抛 `ValueError`，**一条命令都不许发** |
| 5 | ⚠ 车**根本没动**、但本来就在窗口里 | `reached=True` —— 见下面的"真陷阱" |

## 第 5 条是真陷阱，不是 bug

`drive_to` 的 `reached` **只由 `|final − target| ≤ tol` 决定**，不看车有没有动过。
所以「车被卡住/轮子空转，但它本来就在容差窗口里」会报 `reached=True`。
调用方（Task 12）必须自己看 `iters`/`trace` 有没有实际位移，否则会把
"没动过"记成"归位成功"。真机上这条已被观测到：连跑三次
`drive_to(0,0)` 都返回 `reached=True, iters=0` —— 那不是收敛证据，
只是"车本来就在窗口里"。
"""
from __future__ import annotations

import json
import sys
import warnings
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPERIMENT_DIR = HERE.parent
ROOT = EXPERIMENT_DIR.parent
for _p in (str(EXPERIMENT_DIR), str(ROOT / "视觉测量" / "脚本"),
           str(EXPERIMENT_DIR / "vgt" / "tests")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from vgt import harness as H                                     # noqa: E402

import test_harness as T                                         # noqa: E402

CALIB = dict(T.CALIB)


def _run_replies(n_runs=12):
    """给 N 条 RUN 各配一组正常回复（OK RUN + D 行 + DONE），STOP 回 OK STOP。"""
    run = [[b"OK RUN pwm=-110,110,-110,110 ms=200 kick=150ms/105\n",
            b"D 0 -110 110 -110 110 0 0 0 0 0.00 0.00 90.00\n",
            b"DONE\n"] for _ in range(n_runs)]
    return {"RUN": run, "STOP": [[b"OK STOP\n"]] * (n_runs + 4),
            "PING": [[b"PONG\n"]] * 4}


def _make(poses_list, replies=None):
    """假设备：`detect_markers` 脚本化，其余走真管线。返回 (harness, sock)。"""
    sock = T.FakeSock(replies if replies is not None else _run_replies())
    camera = T.FakeCamera([])
    h = H.Harness(T.make_car_link(sock), camera, dict(CALIB),
                  done_timeout_s=1.0, silence_timeout_s=0.05,
                  reply_timeout_s=0.05, handshake_timeout_s=0.5, locate_tries=1)
    script = list(poses_list)

    def detect(frame, calib):
        return script.pop(0) if script else {}

    H.detect_markers = detect          # 脚本化（进程内，退出即还原）
    return h, sock


def _runs(sock):
    return [c for c in sock.sent if c.startswith("RUN")]


def case1_already_at_target():
    h, sock = _make([T.scene(vehicle_xy=(3.0, 4.0))])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r = h.drive_to(0.0, 0.0, pulse_ms=200)
    return {"case": "已在容差窗口内直接归位",
            "reached": r["reached"], "reason": r["reason"], "iters": r["iters"],
            "final": [round(v, 1) for v in r["final"]] if r["final"] else None,
            "PASS": bool(r["reached"] and r["reason"] == "converged"
                         and r["iters"] == 0 and not _runs(sock))}


def case2_vision_blind_midway():
    """视觉中途整段失明：起点能看见，之后连续失败。"""
    h, sock = _make([T.scene(vehicle_xy=(-300.0, 0.0)), {}, {}, {}, {}, {}])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r = h.drive_to(0.0, 0.0, pulse_ms=200, max_iter=6)
    return {"case": "中途视觉失明",
            "reached": r["reached"], "reason": r["reason"], "iters": r["iters"],
            "final": r["final"], "last_locate_reason": h.last_locate_reason,
            "契约": ("`final` 的定义是**最后一次成功 locate** 的位姿（模块 docstring），"
                   "所以它停在**脉冲之前**的 (-300,0)，而不是脉冲之后的真实位置 —— "
                   "这是诚实的'我不知道'，不是编造；但调用方若拿 final 算误差，"
                   "算到的是**上一轮**的误差。"),
            "PASS": bool(r["reached"] is False and r["reason"] == "locate_failed"
                         and r["final"] is not None
                         and abs(r["final"][0] + 300.0) < 1e-6)}


def case3_boundary_drift():
    """边界码像素漂移 > 15px ⇒ 必须抛 `BoundaryMarkersMoved`。"""
    h, sock = _make([T.scene(vehicle_xy=(-300.0, 0.0))])
    h.locate()                                  # 第一帧成功 ⇒ 抓基线
    h2, _ = _make([T.scene(vehicle_xy=(-300.0, 0.0), px_shift=(40.0, 40.0))])
    h2._boundary_px_baseline = h._boundary_px_baseline
    thrown = None
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            h2.locate()
        except H.BoundaryMarkersMoved as exc:
            thrown = type(exc).__name__
    return {"case": "边界码漂移超限（40px > 15px）", "raised": thrown,
            "PASS": thrown == "BoundaryMarkersMoved"}


def case4_target_out_of_field():
    h, sock = _make([T.scene(vehicle_xy=(0.0, 0.0))], replies=_run_replies(0))
    raised = None
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            h.drive_to(9000.0, 0.0, pulse_ms=200)
        except ValueError:
            raised = "ValueError"
    return {"case": "目标越出场地", "raised": raised,
            "commands_sent": list(getattr(sock, "sent", [])),
            "PASS": raised == "ValueError" and not getattr(sock, "sent", [])}


def case5_never_moves_but_in_window():
    """真陷阱：误差已在窗口内 ⇒ 0 次迭代就 reached=True（车根本没动）。"""
    h, sock = _make([T.scene(vehicle_xy=(3.0, 4.0))])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r = h.drive_to(0.0, 0.0, pulse_ms=200)
    return {"case": "车没动过但已在窗口内",
            "reached": r["reached"], "reason": r["reason"], "iters": r["iters"],
            "n_RUN": len(_runs(sock)),
            "语义": ("reached 只表示'几何上在 tol 窗口内'，不表示'车动过'。"
                     "调用方必须自己看 iters/trace。"),
            "PASS": None}


def main() -> int:
    cases = [case1_already_at_target(), case2_vision_blind_midway(),
             case3_boundary_drift(), case4_target_out_of_field(),
             case5_never_moves_but_in_window()]
    hard = [c for c in cases if c["PASS"] is not None]
    out = {"cases": cases, "n_hard": len(hard),
           "all_hard_cases_pass": all(c["PASS"] for c in hard)}
    print(json.dumps(out, ensure_ascii=False, indent=1))
    (HERE / "out").mkdir(parents=True, exist_ok=True)
    (HERE / "out" / "silent_data_audit.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0 if out["all_hard_cases_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
