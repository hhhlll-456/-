#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 14 补实验的统一跑车器：**一条一条跑、每条都验位置、不安全就停手**。

为什么不用 `run_all_experiments.py`：那个绑定 `vgt.matrix.EXPERIMENTS` 的 16 条固定矩阵
（时长是写死的），本任务要跑的是**自选的 (vx, vy, om, ms) 组合**。安全机制照抄
`Harness.run_trial`（等 `DONE`、失控保护 STOP、边界码漂移门禁、起止帧 settle）。

## 安全闸（每一层都不许跳过）

1. **开场**：`locate()` 解不出车 → 一条都不跑；
2. **每条之前**：用 `correct.py` 的模型预测终点，车中心必须落在
   **|x| ≤ 117.5−MARGIN、|y| ≤ 51.4−MARGIN** 内（`MARGIN` 默认 15 mm）——
   车体半包络 135×74.5，越线就是压边界码；
3. **每条之后**：`locate()` 实测位置，`fully_inside=False` 或 `near_edge=True` → **停批**；
4. 任何异常 → 收尾 STOP（`run_trial` 的 finally 保证）+ 停批。

## 用法（从 `实验/` 下跑）

    cd 实验
    python3 t14_correction/campaign.py --tag probe --trial "0.2 0 0 320"

每条记录追加到 `实验/t14_correction/data/campaign.jsonl`。
**本脚本会真的驱动电机** —— 跑之前确认车在场地里、电量够、没别人在用它。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import warnings
from collections import namedtuple
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPERIMENT_DIR = HERE.parent
ROOT = EXPERIMENT_DIR.parent
for _p in (str(EXPERIMENT_DIR), str(ROOT / "视觉测量" / "脚本"), str(HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from car_link import CarLink                                   # noqa: E402
from vgt.camera import CameraSession                           # noqa: E402
from vgt.harness import Harness, BoundaryMarkersMoved          # noqa: E402
from vision_core import calib_store                            # noqa: E402

BOARD_IP = "192.168.1.115"
BOARD_PORT = 8266

FIELD_HALF_X = 252.5
FIELD_HALF_Y = 125.9
CAR_HALF_X = 135.0
CAR_HALF_Y = 74.5
LIMIT_X = FIELD_HALF_X - CAR_HALF_X            # 117.5
LIMIT_Y = FIELD_HALF_Y - CAR_HALF_Y            # 51.4
MARGIN_MM = 15.0                               # 预测终点留的安全余量

DATA = HERE / "data"
OUT = DATA / "campaign.jsonl"

Experiment = namedtuple("Experiment", "id action desc vx vy om duration_s repeats")


def open_link(ip=BOARD_IP, port=BOARD_PORT, tries=5, pause_s=3.0, handshake_s=8.0):
    """连板子 + `PING`/`PONG` 握手（口径同 `task10_live/live_link.py`）。"""
    last = None
    for k in range(1, tries + 1):
        link = None
        try:
            link = CarLink(ip=ip, port=port)
            t0 = time.time()
            while time.time() - t0 < handshake_s:
                r = link.cmd("PING", wait=True, timeout=2.0)
                if r and "PONG" in str(r):
                    return link
            last = "PING 没等到 PONG"
        except Exception as exc:                            # noqa: BLE001
            last = f"{type(exc).__name__}: {exc}"
        if link is not None:
            try:
                link.close()
            except Exception:                              # noqa: BLE001
                pass
        print(f"  [link] 第 {k}/{tries} 次失败：{last}；{pause_s:g}s 后重试",
              file=sys.stderr, flush=True)
        time.sleep(pause_s)
    raise RuntimeError(f"连不上板子 {ip}:{port}（最后一次：{last}）")


def _predict_end(x, y, yaw, vx, vy, om, ms):
    """用 correct.py 的修正常数预测终点（保守：平移按最大速度估）。

    斜行（vx、vy 都非零）走的是**另一套运动学** —— 四个轮子只出一对，速度既不是
    纵向也不是横向的常数。闸门里**故意用两轴中较快的那个（轴 A）**，宁可高估；
    用纵向/横向常数去估会高估 2～3 倍，把闸门卡死（2026-10-15 实测过）。
    """
    import correct
    dx = dy = 0.0
    diag = abs(vx) > 1e-12 and abs(vy) > 1e-12
    if diag:
        axs = [correct.CHANNELS[("斜行", k)] for k in ("轴A", "轴B")]
        fast = max(c.V for c in axs)          # 宁可高估：取快的轴、取小的死区
        td = min(c.t_d_ms for c in axs)
        comp = fast * max(0.0, ms - td) / 1000.0 / math.sqrt(2.0)
        dx = comp if vx > 0 else -comp
        dy = comp if vy > 0 else -comp
    else:
        if abs(vx) > 1e-12:
            dx = correct.travel("纵向", "前进" if vx > 0 else "后退", ms)
            dx *= 1.0 if vx > 0 else -1.0
        if abs(vy) > 1e-12:
            dy = correct.travel("横向", "左移" if vy > 0 else "右移", ms)
            dy *= 1.0 if vy > 0 else -1.0
    th = math.radians(yaw)
    # 车体系 (dx, dy) → 场地系（同 harness.body_delta_to_field）
    fx = dx * math.cos(th) - dy * math.sin(th)
    fy = -dx * math.sin(th) - dy * math.cos(th)
    return x + fx, y + fy


def _safe(x, y, margin=MARGIN_MM):
    return abs(x) <= LIMIT_X - margin and abs(y) <= LIMIT_Y - margin


def main() -> int:
    ap = argparse.ArgumentParser(description="Task 14 补实验跑车器（会驱动电机）")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--trial", action="append", required=True,
                    metavar="VX VY OM MS", help='例：--trial "0.2 0 0 320"')
    ap.add_argument("--ip", default=BOARD_IP,
                    help=f"板子 IP（缺省 {BOARD_IP} = A 车；B 车是 192.168.1.111）")
    ap.add_argument("--out", default=None,
                    help="落盘文件名（相对 data/；缺省 campaign.jsonl = A 车那份）")
    ap.add_argument("--margin", type=float, default=MARGIN_MM)
    ap.add_argument("--max-seconds", type=float, default=600.0)
    ap.add_argument("--yes", action="store_true", help="跳过回车确认")
    args = ap.parse_args()

    trials = []
    for k, raw in enumerate(args.trial, 1):
        p = [float(x) for x in raw.split()]
        if len(p) != 4:
            print(f"--trial {raw!r} 需要 4 个数（vx vy om ms）", file=sys.stderr)
            return 2
        trials.append((k, *p))

    DATA.mkdir(parents=True, exist_ok=True)
    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    records: list[dict] = []
    t_batch = time.time()

    print(f"批次 {args.tag}：{len(trials)} 条")
    for k, vx, vy, om, ms in trials:
        print(f"  #{k}: RUN {vx} {vy} {om} {int(ms)}")
    if not args.yes:
        try:
            if input("回车开始（Ctrl-C 取消）：").strip():
                pass
        except EOFError:
            pass

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with CameraSession() as cam, open_link(ip=args.ip) as car:
            h = Harness(car, cam, calib, home_xy=(0.0, 0.0))

            pose = h.locate()
            if pose is None:
                print(f"❌ 开场 locate 失败（{h.last_locate_reason}）—— 一条都不跑",
                      file=sys.stderr)
                return 3
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
            print(f"开场位姿 x={x:8.2f} y={y:8.2f} yaw={yaw:+7.2f}°")

            for k, vx, vy, om, ms in trials:
                if time.time() - t_batch > args.max_seconds:
                    print(f"⏱ 超过 --max-seconds={args.max_seconds:g}s，停批")
                    break
                px, py = _predict_end(x, y, yaw, vx, vy, om, ms)
                if not _safe(px, py, args.margin):
                    print(f"⛔ #{k} 预测终点 x={px:.1f} y={py:.1f} 超出安全区"
                          f"（|x|≤{LIMIT_X - args.margin:.1f}、"
                          f"|y|≤{LIMIT_Y - args.margin:.1f}）—— **停批**")
                    break

                exp = Experiment(k, "probe", f"t14 {args.tag} #{k}",
                                 vx, vy, om, ms / 1000.0, 1)
                print(f"\n▶ #{k} RUN {vx} {vy} {om} {int(ms)}"
                      f"（预测终点 {px:.1f}, {py:.1f}）", flush=True)
                try:
                    r = h.run_trial(exp)
                except BoundaryMarkersMoved as exc:
                    print(f"⛔ 边界码漂移：{exc} —— 停批")
                    break
                except Exception as exc:                   # noqa: BLE001
                    print(f"❌ #{k} 抛 {type(exc).__name__}: {exc} —— 停批")
                    break

                vd = r.get("vision_delta") or {}
                vs = r.get("vision", {}).get("start")
                ve = r.get("vision", {}).get("end")
                rec = dict(
                    tag=args.tag, idx=k, vx=vx, vy=vy, om=om, ms=int(ms),
                    ok=r["ok"], reason=r["reason"],
                    run_line=r["run_line"], n_samples=r["n_samples"],
                    start=vs, end=ve,
                    dist_mm=vd.get("dist_mm"), yaw_deg=vd.get("yaw_deg"),
                    dx_mm=vd.get("dx_mm"), dy_mm=vd.get("dy_mm"),
                    odo=r.get("odometry_delta"),
                    warnings=list(r.get("warnings") or []),
                )
                records.append(rec)
                with open(OUT, "a") as fh:
                    fh.write(json.dumps(rec, ensure_ascii=False) + "\n")

                if vd:
                    print(f"  → 走 {vd['dist_mm']:7.2f} mm  偏航 {vd['yaw_deg']:+6.2f}°"
                          f"  Δ({vd['dx_mm']:+7.2f}, {vd['dy_mm']:+7.2f})"
                          f"  ok={r['ok']}")
                else:
                    print(f"  → ❌ 没解出位移（reason={r['reason']}）")

                if ve is None:
                    print("  ⚠ 终点帧没解出车码 —— 停批（位置未知）")
                    break
                x, y, yaw = float(ve[0]), float(ve[1]), float(ve[2])
                ins = abs(x) + CAR_HALF_X <= FIELD_HALF_X and abs(y) + CAR_HALF_Y <= FIELD_HALF_Y
                near = min(FIELD_HALF_X - abs(x), FIELD_HALF_Y - abs(y)) < 80.0
                print(f"  实测位置 x={x:8.2f} y={y:8.2f} yaw={yaw:+7.2f}°"
                      f"  整车在内={ins} 贴边={near}")
                if not ins or near:
                    print("  ⛔ 位置不安全 —— 停批")
                    break

        warns = [str(w.message) for w in caught]

    print(f"\n批次结束：{len(records)}/{len(trials)} 条落盘 → {OUT}")
    for w in warns:
        print(f"  ⚠ {w}")
    return 0 if len(records) == len(trials) else 1


if __name__ == "__main__":
    raise SystemExit(main())
