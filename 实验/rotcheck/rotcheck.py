#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""偏航通道交叉校验（rotcheck 的修复版）—— 把上一轮没跑完的那条腿跑完。

## 依赖（2026-10-15 深夜已自包含）

本文件原先 `sys.path` 第一条写死 `/tmp/opencode/c1cal`，**那两个模块不在仓库里**
⇒ 直接跑会 `ModuleNotFoundError: measure`。现已把两个模块**收进本目录**：

  - `measure.py` —— 视觉测量层（一次抓 N 帧，给出 `pnp` / `ruler_h` / `ruler_l`
    三条彼此独立的位移口径 + 帧间离散度）；
  - `calib_c1.py` —— 链路底层（`stop` / `drain` / `read_until` / `read_enc` /
    `do_leg`）与速度模型 `ms_for(mm) = mm/0.56 + 143`。

三者的 `sys.path` 与输出路径已改为**按 `__file__` 定位**，输出落到本目录
`data/`（原来写 `/tmp`，重启就没了）⇒ 现在整目录自包含。其余两个脚本
（`yaw_c1.py`、`s300_review.py`）本来就只依赖仓库内模块，也已把写死的输出路径
一并改为 `data/`。

## 与上一轮 `rotcheck.py` 的唯一差别（真 bug，不是"改进"）

上一轮 `rot_leg` 收 D 行时写的是

    done, dlines = C.read_until(link, lambda l: l == "DONE", timeout_s=15.0)

`read_until` 的 `keep` 默认 **False**，命中即 `return ln, (others if keep else [])`
—— 于是 `dlines` 恒为 `[]`，`parse_telemetry([])` → 空，`odo_dyaw_deg_nominal_c1`
三条腿全是 `NaN`。车跑了、尺子拍了，就是没算数。修复就是加 `keep=True`。

## 为什么这个偏航腿还值得再跑一遍

上一轮那 3 条腿的 `d_enc`（ZENC→停车后）其实**够**解 c1（余量都落在车静止的
时段，窗口天然对齐），所以离线已经算出了 c1。但那条路走的是我自己写的闭式
`dψ = (2πR/4LXY)·Σ(Δc/c1)`。这里补一次**用仓库自己的 `integrate_encoders`**
算的记录，并且**同时**给出"只积 D 行"与"并入滑行"两个数 ——
偏航腿和 S300 是两种完全不同的运动，纯旋转照样能独立验证 R66 那条修复。

## 安全

- 绝不发裸 `M`：只用 `RUN 0 0 ±0.20 200`（自带时长）。
- 原地旋转的系统性平移 ≈4 mm/rad；一条腿 ~0.23 rad ⇒ 实测 6.5~9.0 mm，可接受。
- 每跑一条腿就量位置；`|x| > 105` 或 `|y| > 46` 立刻中止（车体包络不该出标记矩形）。
- 正反交替，净平移≈0。`finally` 里永远 `STOP`。
"""
import json
import math
import statistics
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent        # 实验/rotcheck/
_ROOT = _HERE.parents[1]                      # 仓库根
sys.path.insert(0, str(_HERE))                # calib_c1.py / measure.py 同目录
sys.path.insert(0, str(_ROOT / "实验"))
sys.path.insert(0, str(_ROOT / "实验" / "vgt"))
sys.path.insert(0, str(_ROOT / "视觉测量" / "脚本"))
sys.path.insert(0, str(_ROOT / "实验" / "task10_live"))

import calib_c1 as C                                             # noqa: E402
import measure as M                                              # noqa: E402
from vgt.camera import CameraSession                              # noqa: E402
from vgt.harness import coasted_samples                           # noqa: E402
from vgt.odometry import integrate_encoders, parse_telemetry      # noqa: E402
from live_link import open_link                                   # noqa: E402
from vision_core import calib_store                               # noqa: E402

C1_NOM = (1276.0, 1270.0, 1276.0, 1274.0)
SIGN = (-1.0, 1.0, -1.0, 1.0)
LXY_OVER_R = 0.12825 / 0.0335
R_EFF = 33.5
LXY_MM = 128.25
K = 2.0 * math.pi * R_EFF / (4.0 * LXY_MM)
OUT = _HERE / "data" / "rot2.jsonl"       # 原来写 /tmp，重启就没了
SAFE_X, SAFE_Y = 105.0, 46.0


def rot_leg(link, cam, calib, om, ms, tag, out, frames=9):
    C.stop(link)
    time.sleep(0.8)
    C.drain(link, 0.2)
    m0 = M.measure(cam, calib, n=frames, tag=tag + "/start", min_frames=4)
    yaw0 = m0["pnp"][2]
    print(f"[{tag}] 起点 ({m0['pnp'][0]:.1f},{m0['pnp'][1]:.1f}) yaw={yaw0:.2f}",
          flush=True)

    link.sock.sendall(b"ZENC\n")
    C.read_until(link, lambda l: l.startswith("OK ZENC"), timeout_s=3.0)
    e0, _ = C.read_enc(link)

    line = f"RUN 0 0 {om:.4f} {ms}"
    print(f"[{tag}] {line}", flush=True)
    link.sock.sendall((line + "\n").encode())
    ok, _ = C.read_until(link, lambda l: l.startswith("OK RUN"), timeout_s=3.0)
    if ok is None:
        C.stop(link)
        raise RuntimeError("RUN 没被接受")
    # ★ 上一轮的 bug：`keep` 默认 False ⇒ dlines 恒空 ⇒ 里程计偏航恒 NaN
    done, dlines = C.read_until(link, lambda l: l == "DONE",
                                 timeout_s=15.0, keep=True)
    C.stop(link)
    if done is None:
        raise RuntimeError("等不到 DONE（已 STOP）")
    time.sleep(0.8)                      # 与视觉终点帧同一时刻：车已滑停
    e1, _ = C.read_enc(link)             # 停车后的累计计数（含自由滑行）

    m1 = M.measure(cam, calib, n=frames, tag=tag + "/end", min_frames=4)
    dyaw = m1["pnp"][2] - yaw0
    dxy = (m1["pnp"][0] - m0["pnp"][0], m1["pnp"][1] - m0["pnp"][1])

    samples = parse_telemetry(dlines)
    odo_raw = integrate_encoders(samples, c_1rev=C1_NOM, sign=SIGN,
                                 l_xy_over_r=LXY_OVER_R, r_eff_mm=R_EFF) \
        if len(samples) >= 2 else None

    # 并入停车后的滑行（用仓库自己的 R66 修复），让里程计窗口包住视觉终点帧
    merged = coasted_samples(samples, {"enc": list(e1), "rpy": None})
    coast_ok = merged is not samples
    odo_coast = integrate_encoders(merged, c_1rev=C1_NOM, sign=SIGN,
                                   l_xy_over_r=LXY_OVER_R, r_eff_mm=R_EFF) \
        if len(merged) >= 2 else None

    rec = {"tag": tag, "cmd": line, "ms": ms, "om": om,
           "d_enc": [e1[i] - e0[i] for i in range(4)],
           "start": m0, "end": m1,
           "vision_dyaw_deg": dyaw,
           "n_d_lines": len(samples),
           "last_d_pwm": list(samples[-1]["pwm"]) if samples else None,
           "coast_merged": coast_ok,
           "coast_counts": ([b - a for a, b in zip(samples[-1]["enc"], e1)]
                            if coast_ok else None),
           "odo_dyaw_deg_donly": (odo_raw[-1][2] if odo_raw is not None else None),
           "odo_dyaw_deg_nominal_c1": (odo_coast[-1][2]
                                       if odo_coast is not None else None),
           "shift_mm": math.hypot(*dxy)}
    print(f"[{tag}] D行={len(samples)} 条  视觉Δ={dyaw:+.2f}deg  "
          f"里程计(只积D行,标称c1)Δ={rec['odo_dyaw_deg_donly']:+.2f}deg  "
          f"里程计(并入滑行)Δ={rec['odo_dyaw_deg_nominal_c1']:+.2f}deg  "
          f"比={rec['odo_dyaw_deg_nominal_c1'] / dyaw:.4f}  "
          f"平移={rec['shift_mm']:.1f}mm  dEnc={rec['d_enc']}", flush=True)
    with out.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return rec


def main():
    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    link = open_link()
    recs = []
    try:
        C.stop(link)
        C.drain(link, 0.3)
        with CameraSession() as cam:
            pos = M.measure(cam, calib, n=7, tag="init", min_frames=3)
            print(f"[init] ({pos['pnp'][0]:.1f}, {pos['pnp'][1]:.1f}) "
                  f"yaw={pos['pnp'][2]:.2f}", flush=True)
            if abs(pos["pnp"][0]) > SAFE_X or abs(pos["pnp"][1]) > SAFE_Y:
                print("[abort] 起点不在安全区", flush=True)
                return 2
            for k, om in enumerate([0.20, -0.20, 0.20]):
                tag = f"R{k + 1}{'+' if om > 0 else '-'}"
                try:
                    recs.append(rot_leg(link, cam, calib, om, 200, tag, OUT))
                except Exception as exc:
                    print(f"[{tag}] 失败：{type(exc).__name__}: {exc}", flush=True)
                    break
                p = recs[-1]["end"]["pnp"]
                print(f"[{tag}] 结束后 ({p[0]:.1f}, {p[1]:.1f})", flush=True)
                if abs(p[0]) > SAFE_X or abs(p[1]) > SAFE_Y:
                    print(f"[{tag}] 位置不安全，中止。", flush=True)
                    break
    finally:
        try:
            C.stop(link)
            link.cmd("STREAM 0", wait=True, timeout=3.0)
        except Exception:
            pass
        link.close()
        print("[link] 已 STOP + STREAM 0，连接关闭", flush=True)

    if not recs:
        return 2
    print("\n== 偏航交叉校验（标称 c1 = 1276/1270/1276/1274，未动） ==")
    for r in recs:
        v = r["vision_dyaw_deg"]
        d = r["odo_dyaw_deg_donly"]
        c = r["odo_dyaw_deg_nominal_c1"]
        s_c = abs(c) / abs(v)
        print(f"{r['tag']:>4}: 视觉 {v:+7.2f}°  只积D行 {d:+7.2f}° ({d / v:5.3f})  "
              f"并入滑行 {c:+7.2f}° ({s_c:5.3f})  ⇒ c1尺度 s={s_c:.4f}  "
              f"c1_flat={s_c * 1274:.1f}  平移 {r['shift_mm']:.1f}mm")
    ss = [abs(r["odo_dyaw_deg_nominal_c1"]) / abs(r["vision_dyaw_deg"]) for r in recs]
    dd = [r["odo_dyaw_deg_donly"] / r["vision_dyaw_deg"] for r in recs]
    print(f"\n里程计/视觉（只积D行）: mean={statistics.fmean(dd):.4f}")
    print(f"里程计/视觉（并入滑行）: mean={statistics.fmean(ss):.4f}  "
          f"sd={statistics.stdev(ss) if len(ss) > 1 else 0.0:.4f}  "
          f"峰峰={max(ss) - min(ss):.4f} "
          f"({(max(ss) - min(ss)) / statistics.fmean(ss) * 100:.2f}% of mean)")
    print(f"⇒ 等效 c1_flat mean={statistics.fmean(ss) * 1274:.1f}  "
          f"(标称 1274，偏差 {(statistics.fmean(ss) * 1274 - 1274) / 1274 * 100:+.1f}%)")
    return 0


if __name__ == "__main__":
    sys.exit(main())