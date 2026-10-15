#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机标定编码器尺度常数 c1（counts/rev）—— 平移法。

## 方法

车在场地内做**已知的平移**（不原地转，避开麦轮旋转的系统性平移）：

1. `STOP` → 等停 → 视觉量起点（PnP + 像素尺子，各 N 帧）
2. `ZENC` 清零 → 立刻读编码器（记下清零后的读数 e0）
3. `RUN vx vy 0 <ms>`（**带时长**）→ 收 `D` 行到 `DONE` → 立刻 `STOP`
4. 等停 → 读编码器 e1 → 视觉量终点
5. `c1_i = Δc_i · 2π·R_eff / s`，其中 `s` = 三条口径各自解出的沿车头位移

## 安全

- 绝不发裸 `M`（无超时）：只用 `RUN vx vy 0 <ms>`。
- 每条腿前后都量位置；`|x| > 117` 或 `|y| > 51`（车体包络出场地）立刻停并退出。
- 每条腿前检查"起点在安全半区"，并按实测速度把目标位移截到安全范围内。
- `finally` 里永远 `STOP`。
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent        # 实验/rotcheck/
_ROOT = _HERE.parents[1]                      # 仓库根
sys.path.insert(0, str(_HERE))                # measure.py 与本文件同目录
sys.path.insert(0, str(_ROOT / "实验"))
sys.path.insert(0, str(_ROOT / "视觉测量" / "脚本"))

import measure as M                                           # noqa: E402
from vgt.camera import CameraSession                           # noqa: E402
from vgt.odometry import parse_telemetry                       # noqa: E402
from vision_core import calib_store                            # noqa: E402

sys.path.insert(0, str(_ROOT / "实验" / "task10_live"))
from live_link import open_link                                # noqa: E402

OUT = _HERE / "data" / "c1_legs.jsonl"    # 原来写 /tmp，重启就没了

R_EFF_MM = 33.5
CIRC_MM = 2.0 * math.pi * R_EFF_MM
SAFE_X_MM = 117.0
SAFE_Y_MM = 51.0
BAND_MM = 95.0          # 正式腿的终点必须落在 |x| <= 95（车体包络才不出标记矩形）
V_STEADY = 0.56        # m/s 稳态（实测拟合）
DEAD_MS = 143.0        # 起步死时间（错峰 120ms + kick）
WHEELS = ("FL", "FR", "RL", "RR")


# ---- 链路底层 ---------------------------------------------------------------

def drain(link, budget=0.3):
    link.sock.settimeout(0.1)
    end = time.time() + budget
    got = []
    while time.time() < end:
        try:
            chunk = link.sock.recv(4096)
        except Exception:
            continue
        if not chunk:
            break
        link.buf += chunk
        end = time.time() + budget          # 有数据就再等一会儿
    lines = split_lines(link)
    return lines


def split_lines(link):
    out = []
    while b"\n" in link.buf:
        raw, link.buf = link.buf.split(b"\n", 1)
        s = raw.decode("utf-8", "replace").strip("\r").strip()
        if s:
            out.append(s)
    return out


def read_until(link, pred, timeout_s=3.0, keep=False):
    """读到满足 `pred(line)` 的行为止。返回 (命中行, 期间的其它行)。"""
    others, end = [], time.time() + timeout_s
    link.sock.settimeout(0.2)
    while time.time() < end:
        for ln in split_lines(link):
            if pred(ln):
                return ln, (others if keep else [])
            others.append(ln)
        try:
            chunk = link.sock.recv(4096)
        except Exception:
            continue
        if not chunk:
            break
        link.buf += chunk
    return None, others


def stop(link):
    r = link.cmd("STOP", wait=True, timeout=3.0)
    return r


def read_enc(link, n_lines=3):
    """用 `STREAM 1` 的 `I` 行读编码器（RUN 之外唯一的读数途径）。"""
    drain(link, 0.25)
    r = link.cmd("STREAM 1", wait=True, timeout=3.0)
    if r is None or not r.startswith("OK STREAM"):
        raise RuntimeError(f"STREAM 1 没被接受：{r!r}")
    got = []
    end = time.time() + 2.0
    link.sock.settimeout(0.2)
    while len(got) < n_lines and time.time() < end:
        for ln in split_lines(link):
            if ln.startswith("I "):
                got.append(ln)
        try:
            chunk = link.sock.recv(4096)
        except Exception:
            continue
        if not chunk:
            break
        link.buf += chunk
    link.cmd("STREAM 0", wait=True, timeout=3.0)
    drain(link, 0.3)
    if not got:
        raise RuntimeError("读不到 `I` 行（空闲样本）—— 编码器读数失败。")
    parts = got[-1].split()
    return [int(v) for v in parts[6:10]], got[-1]


def do_leg(link, cam, calib, *, vx, ms, tag, n_frames=7, vy=0.0):
    """一条带时长的平移腿。返回一条完整记录。"""
    rec = {"tag": tag, "cmd": f"RUN {vx:.4f} {vy:.4f} 0 {ms}", "ms": ms,
           "vx": vx, "vy": vy}

    # ---- 起点
    print(f"  [{tag}] STOP 收尾 + 等停 …", flush=True)
    stop(link)
    time.sleep(0.6)
    drain(link, 0.2)
    m0 = M.measure(cam, calib, n=n_frames, tag=tag + "/start", min_frames=4)
    rec["start"] = m0
    print(f"  [{tag}] 起点 pnp=({m0['pnp'][0]:.1f}, {m0['pnp'][1]:.1f}) "
          f"yaw={m0['pnp'][2]:.2f}deg", flush=True)

    # ---- 清零 + 记清零后读数
    zr = link.cmd("ZENC", wait=True, timeout=3.0)
    if zr is None or not zr.startswith("OK ZENC"):
        raise RuntimeError(f"ZENC 未确认：{zr!r}")
    e0, raw0 = read_enc(link)
    rec["enc_after_zenc"] = e0
    rec["enc_after_zenc_raw"] = raw0
    print(f"  [{tag}] ZENC 后编码器 = {e0}", flush=True)

    # ---- 跑
    print(f"  [{tag}] {rec['cmd']}", flush=True)
    run_line = rec["cmd"]
    link.sock.sendall((run_line + "\n").encode())
    ok, extra = read_until(link, lambda l: l.startswith("OK RUN") or l.startswith("ERR"),
                           timeout_s=3.0, keep=True)
    if ok is None or not ok.startswith("OK RUN"):
        raise RuntimeError(f"RUN 没被接受：ok={ok!r} extra={extra[-3:]}")
    rec["run_reply"] = ok
    done, dlines = read_until(link, lambda l: l == "DONE", timeout_s=20.0, keep=True)
    rec["n_d"] = len(dlines)
    if done is None:
        stop(link)
        raise RuntimeError("等不到 DONE（已发 STOP）")
    stop(link)

    # ---- 终点：先读编码器再量视觉（车已由 STOP 锁死）
    time.sleep(0.8)
    e1, raw1 = read_enc(link)
    rec["enc_end"] = e1
    rec["enc_end_raw"] = raw1
    rec["d_enc"] = [e1[i] - e0[i] for i in range(4)]
    print(f"  [{tag}] 编码器增量 = {rec['d_enc']}", flush=True)

    m1 = M.measure(cam, calib, n=n_frames, tag=tag + "/end", min_frames=4)
    rec["end"] = m1
    print(f"  [{tag}] 终点 pnp=({m1['pnp'][0]:.1f}, {m1['pnp'][1]:.1f}) "
          f"yaw={m1['pnp'][2]:.2f}deg", flush=True)

    # ---- 位移（先算，遥测摘要失败也不能丢）----
    d_pnp = M.disp(m0, m1)
    yaw0 = m0["pnp"][2]
    s_pnp = M.along(d_pnp["pnp"][0], d_pnp["pnp"][1], yaw0)
    lat_pnp = M.lateral(d_pnp["pnp"][0], d_pnp["pnp"][1], yaw0)

    s_h = M.along(d_pnp["ruler_h"][0], d_pnp["ruler_h"][1], yaw0)
    lat_h = M.lateral(d_pnp["ruler_h"][0], d_pnp["ruler_h"][1], yaw0)

    # 线性像素尺子：把 ID29 的像素位移投影到运动轴，再乘 505mm / d_px
    dx_px = m1["car_px"][0] - m0["car_px"][0]
    dy_px = m1["car_px"][1] - m0["car_px"][1]
    d2021 = 0.5 * (m0["d2021_px"] + m1["d2021_px"])
    s_l = M.along(dx_px, dy_px, yaw0) * M.FIELD_LEN_MM / d2021
    lat_l = M.lateral(dx_px, dy_px, yaw0) * M.FIELD_LEN_MM / d2021

    rec["disp"] = {
        "pnp": {"s_mm": s_pnp, "lat_mm": lat_pnp,
                "dx": d_pnp["pnp"][0], "dy": d_pnp["pnp"][1]},
        "ruler_h": {"s_mm": s_h, "lat_mm": lat_h,
                    "dx": d_pnp["ruler_h"][0], "dy": d_pnp["ruler_h"][1]},
        "ruler_l": {"s_mm": s_l, "lat_mm": lat_l},
        "yaw_deg": d_pnp["yaw_deg"],
    }
    rec["c1"] = {}
    for key in ("pnp", "ruler_h", "ruler_l"):
        s = rec["disp"][key]["s_mm"]
        rec["c1"][key] = ([round(rec["d_enc"][i] * CIRC_MM / s, 1) for i in range(4)]
                          if abs(s) > 5.0 else None)
    print(f"  [{tag}] 位移 pnp={s_pnp:.1f}mm 尺H={s_h:.1f}mm 尺L={s_l:.1f}mm "
          f"横移={lat_pnp:.1f}mm yawΔ={d_pnp['yaw_deg']:.2f}deg", flush=True)
    if rec["c1"]["pnp"]:
        print(f"  [{tag}] c1(pnp)={rec['c1']['pnp']} c1(尺H)={rec['c1']['ruler_h']} "
              f"c1(尺L)={rec['c1']['ruler_l']}", flush=True)

    # ---- 遥测摘要（只用来看 pwm / 轮速一致性，不参与 c1 解算）
    try:
        samples = parse_telemetry(dlines)
        if len(samples) >= 2:
            dt = max(1, samples[-1]["t_ms"] - samples[0]["t_ms"])
            modes = {}
            for s in samples:
                modes[tuple(s["pwm"])] = modes.get(tuple(s["pwm"]), 0) + 1
            rec["telemetry"] = {
                "n_d": len(samples), "dt_ms": dt,
                "omega_1276": [round((samples[-1]["enc"][i] - samples[0]["enc"][i])
                                      * 2 * math.pi / c * 1000 / dt, 2)
                                for i, c in enumerate((1276, 1270, 1276, 1274))],
                "pwm_mode": list(max(modes, key=modes.get)),
                "imu_yaw_delta": round(samples[-1]["rpy"][2] - samples[0]["rpy"][2], 2),
            }
    except Exception as exc:                       # 遥测摘要坏了也不能毁掉位移数据
        rec["telemetry_error"] = f"{type(exc).__name__}: {exc}"
    return rec


def in_field(m):
    return abs(m["pnp"][0]) <= SAFE_X_MM and abs(m["pnp"][1]) <= SAFE_Y_MM


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["pilot", "legs", "full"], default="full")
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--pilot-ms", type=int, default=600)
    ap.add_argument("--target-mm", type=float, default=180.0)
    ap.add_argument("--frames", type=int, default=7)
    ap.add_argument("--nlegs", type=int, default=6)
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--allow-outside", action="store_true",
                    help="允许起点在场外（只用于把车挪回安全区）")
    args = ap.parse_args()

    out = Path(args.out)
    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    legs = []

    # 速度模型：s(mm) = V_STEADY * (T_ms - DEAD_MS)。实测拟合（600ms -> 256.3mm、
    # 350ms -> 110.3mm、300ms -> 88.6mm）给出 V~0.56 m/s、DEAD~143ms，与固件错峰窗口
    # (3x40=120ms) + kick 对得上。时长由模型算出来，不靠试。
    def ms_for(mm):
        return int(round(mm / V_STEADY + DEAD_MS))   # mm / (m/s) 本身就是 ms

    def save(rec):
        legs.append(rec)
        with out.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")

    link = open_link()
    try:
        stop(link)
        drain(link, 0.3)
        with CameraSession() as cam:
            m = M.measure(cam, calib, n=args.frames, tag="init", min_frames=3)
            print(f"[init] 车在 ({m['pnp'][0]:.1f}, {m['pnp'][1]:.1f}) "
                  f"yaw={m['pnp'][2]:.2f}", flush=True)
            if not in_field(m):
                if not args.allow_outside:
                    print("[init] 起始位置就在场地外，中止。", flush=True)
                    return 2
                print("[init] WARN 起点在场外（放行模式）", flush=True)

            # ---- 归位到安全带（|x| <= 95），否则正式腿没有回旋余地 ----
            print("== 归位到安全带 ==", flush=True)
            for r in range(6):
                pos = M.measure(cam, calib, n=7, tag=f"pre{r}", min_frames=3)
                x_now = pos["pnp"][0]
                if -95.0 <= x_now <= 95.0:
                    print(f"[home] x={x_now:.1f} 已在安全带内", flush=True)
                    break
                if x_now < -95.0:
                    step, dirn = (-95.0 - x_now), +1.0
                else:
                    step, dirn = (x_now - 95.0), -1.0
                step = min(120.0, step)
                ms = max(150, min(900, ms_for(step)))
                print(f"[home] x={x_now:.1f} -> {'前进' if dirn>0 else '后退'} "
                      f"{step:.0f}mm（{ms}ms）", flush=True)
                try:
                    rec = do_leg(link, cam, calib, vx=dirn * args.vx, ms=ms,
                                 tag=f"H{r}", n_frames=args.frames)
                except Exception as exc:
                    print(f"[H{r}] 失败：{type(exc).__name__}: {exc}", flush=True)
                    stop(link)
                    return 2
                save(rec)
                if not in_field(rec["end"]):
                    print(f"[H{r}] 跑出场地，中止。", flush=True)
                    return 2

            # ---- y 回中（横移；每步都量，|y|<=12 就停）----
            for r in range(4):
                pos = M.measure(cam, calib, n=7, tag=f"prey{r}", min_frames=3)
                y_now = pos["pnp"][1]
                if abs(y_now) <= 12.0:
                    print(f"[y] y={y_now:.1f} 已居中", flush=True)
                    break
                dy = min(30.0, abs(y_now) - 8.0) * (1.0 if y_now > 0 else -1.0)
                ms = max(300, min(900, ms_for(abs(dy))))   # 横移别用 <300ms 的短脉冲
                print(f"[y] y={y_now:.1f} -> 横移 {dy:+.0f}mm（{ms}ms）", flush=True)
                try:
                    rec = do_leg(link, cam, calib, vx=0.0, vy=0.10 if dy > 0 else -0.10,
                                 ms=ms, tag=f"Y{r}", n_frames=args.frames)
                except Exception as exc:
                    print(f"[Y{r}] 失败：{type(exc).__name__}: {exc}", flush=True)
                    stop(link)
                    return 2
                save(rec)
                if abs(rec["end"]["pnp"][1]) > 48.0:
                    print(f"[Y{r}] y 出界，中止。", flush=True)
                    return 2

            # ---- 正式腿：正向/反向交替，起点永远落在安全半区 ----
            print(f"== 正式腿（目标 {args.target_mm:.0f} mm ≈ "
                  f"{ms_for(args.target_mm)} ms） ==", flush=True)
            dirs = ([1, -1] * 4)[:args.nlegs]
            for k, direction in enumerate(dirs):
                pos = M.measure(cam, calib, n=7, tag=f"pre{k}", min_frames=3)
                x_now, y_now = pos["pnp"][0], pos["pnp"][1]
                if abs(y_now) > 35.0:
                    print(f"[leg{k}] 起点 y={y_now:.1f} 离边界太近，跳过", flush=True)
                    continue
                # 终点必须留在 ±BAND 内；截断目标位移，绝不"跑出去再看看"
                end_ok = BAND_MM if direction > 0 else -BAND_MM
                room = (end_ok - x_now) * direction
                if room < 30.0:
                    print(f"[leg{k}] 起点 x={x_now:.1f} 方向 {direction:+d} 没有回旋余地，"
                          f"跳过", flush=True)
                    continue
                tgt = min(args.target_mm, room)
                ms = max(150, min(2500, ms_for(tgt)))
                tag = f"L{k+1}{'F' if direction > 0 else 'B'}"
                try:
                    rec = do_leg(link, cam, calib, vx=direction * args.vx, ms=ms,
                                 tag=tag, n_frames=args.frames)
                except Exception as exc:
                    print(f"[{tag}] 失败：{type(exc).__name__}: {exc}", flush=True)
                    stop(link)
                    break
                save(rec)
                if not in_field(rec["end"]):
                    print(f"[{tag}] 跑出场地，中止。", flush=True)
                    break
    finally:
        try:
            stop(link)
        except Exception:
            pass
        try:
            link.close()
        except Exception:
            pass
    with out.open("a", encoding="utf-8") as fh:
        for rec in legs:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")

    print("\n== 汇总 ==")
    for rec in legs:
        print(f"{rec['tag']:>5}  dEnc={rec['d_enc']}  "
              f"s_pnp={rec['disp']['pnp']['s_mm']:.1f}  "
              f"s_rh={rec['disp']['ruler_h']['s_mm']:.1f}  "
              f"s_rl={rec['disp']['ruler_l']['s_mm']:.1f}  "
              f"c1_pnp={rec['c1']['pnp']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

def ms_for(mm):
    """位移 mm → 脉冲时长 ms（速度模型 s = V_STEADY·(T − DEAD_MS)）。"""
    return int(round(mm / V_STEADY + DEAD_MS))
