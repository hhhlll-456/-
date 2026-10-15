#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""c1 标定的视觉测量层：一次抓 N 帧，同时给出三条彼此独立的位移口径。

三条口径（**都不共用同一个尺度常数**）：

1. `pnp`      —— PnP + 内参 + 建系（harness 主口径）。绝对值可能带系统偏差，
                但**帧间差值**是 PnP 自己的一致量。
2. `ruler_h`  —— **像素尺子（单应）**：四角码图像形心 → 场地理想矩形 → 单应 H，
                把 ID29 的图像形心投到场地平面。**不经过 PnP、不读内参。**
3. `ruler_l`  —— **像素尺子（线性 1D）**：只量 ID20→ID21 的像素距离 d_px，
                物理长度取场地实测长边 FIELD_LEN_MM；车在运动轴上的像素位移
                乘 505/d_px 就是 mm。这是"最朴素"的像素尺子。

⚠ `ruler_h`/`ruler_l` 把**架高**的车载码当成贴地物：离画面中心越远、架得越高，
误差越大。每帧都报出车在画面里的位置，供判断。
"""
from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # 仓库根（本文件在 实验/rotcheck/ 下）
for _p in (str(ROOT / "实验"), str(ROOT / "实验" / "vgt"), str(ROOT / "视觉测量" / "脚本")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cv2                                                   # noqa: E402
import numpy as np                                          # noqa: E402

from vision_core.field import body_pose_in_field, build_field_frame   # noqa: E402
from vision_core.markers import detect_markers              # noqa: E402

# 场地理想矩形（matrix.FIELD_HALF_*，不是码位实测值 —— 实测四码非理想矩形，
# 这里要用"标称矩形"才能让单应只承担透视，不偷带码位误差）。
FIELD_HALF_L = 252.5
FIELD_HALF_W = 125.9
CORNER_FIELD = np.array([[-FIELD_HALF_L, -FIELD_HALF_W],
                         [FIELD_HALF_L, -FIELD_HALF_W],
                         [FIELD_HALF_L, FIELD_HALF_W],
                         [-FIELD_HALF_L, FIELD_HALF_W]], dtype=np.float64)  # 20,21,22,23

CORNER_IDS = (20, 21, 22, 23)
FIELD_LEN_MM = 2 * FIELD_HALF_L        # ID20→ID21 = 505 mm
FRAME_CENTER = (960.0, 540.0)


def _centroid(pose):
    c = pose.corners_px
    return np.array([float(c[:, 0].mean()), float(c[:, 1].mean())])


def measure_once(cam, calib):
    """抓一帧 → 三条口径的车位置（mm，场地系）+ 诊断量。解不出就返回 None。"""
    frame = cam.grab(fresh=True)
    poses = detect_markers(frame, calib)
    if 29 not in poses:
        return None
    px = {mid: _centroid(poses[mid]) for mid in CORNER_IDS if mid in poses}
    if len(px) != 4:
        return None

    src = np.array([px[20], px[21], px[22], px[23]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, CORNER_FIELD.astype(np.float32))
    c29 = _centroid(poses[29])
    hxy = cv2.perspectiveTransform(c29.reshape(1, 1, 2).astype(np.float32), H).reshape(2)

    # PnP 口径（需要 ID29 姿态 + 四角建系）
    ff = build_field_frame(poses)
    x, y, yaw = body_pose_in_field(ff, poses[29], calib["bias"])

    d2021 = float(math.hypot(*(px[21] - px[20])))
    d2023 = float(math.hypot(*(px[23] - px[20])))
    return {
        "pnp": (float(x), float(y), float(yaw)),
        "ruler_h": (float(hxy[0]), float(hxy[1])),
        "d2021_px": d2021,
        "d2023_px": d2023,
        "px_per_mm_2021": d2021 / FIELD_LEN_MM,
        "car_px": (float(c29[0]), float(c29[1])),
        "car_from_center_px": float(math.hypot(c29[0] - FRAME_CENTER[0],
                                               c29[1] - FRAME_CENTER[1])),
        "rms_px": float(poses[29].reproj_rms_px),
    }


def measure(cam, calib, n=7, tag="", min_frames=2):
    """连抓 n 帧求均值。返回聚合后的 dict（各口径的帧间离散度一并报出）。

    `min_frames`：低于这个可用帧数就报错。掉码多时（ID23 常掉）可以用 1 放宽。
    """
    samples = []
    for k in range(n):
        m = measure_once(cam, calib)
        if m is not None:
            m["ok"] = True
            samples.append(m)
        print(f"    [{tag}] frame {k+1}/{n} "
              f"{'ok' if m else 'MISS'}", flush=True)
    if len(samples) < min_frames:
        raise RuntimeError(f"只拿到 {len(samples)} 帧可用测量（要 {min_frames} 帧）。")

    def agg(key):
        vals = [s[key] for s in samples]
        return [float(statistics.fmean([v[i] for v in vals])) for i in range(len(vals[0]))]

    return {
        "n_frames": len(samples),
        "pnp": agg("pnp"),
        "ruler_h": agg("ruler_h"),
        "px_per_mm_2021": float(statistics.fmean(s["px_per_mm_2021"] for s in samples)),
        "px_per_mm_2023": d2023_to_mm(samples),
        "d2021_px": float(statistics.fmean(s["d2021_px"] for s in samples)),
        "d2023_px": float(statistics.fmean(s["d2023_px"] for s in samples)),
        "car_px": agg("car_px"),
        "car_from_center_px": float(statistics.fmean(s["car_from_center_px"] for s in samples)),
        "rms_px": float(statistics.fmean(s["rms_px"] for s in samples)),
        "spread_pnp": spread_mm([s["pnp"] for s in samples]),
        "spread_ruler_h": spread_mm([s["ruler_h"] for s in samples]),
    }


def d2023_to_mm(samples):
    """ID20→ID23 的短边像素距离 → px/mm（供诊断，不参与 c1）。"""
    return float(statistics.fmean(s["d2023_px"] for s in samples)) / (2 * FIELD_HALF_W)


def spread_mm(pts):
    if len(pts) < 2:
        return None
    return {
        "x": float(max(p[0] for p in pts) - min(p[0] for p in pts)),
        "y": float(max(p[1] for p in pts) - min(p[1] for p in pts)),
    }


def disp(m0, m1):
    """两次测量之间的位移向量（mm，场地系）。三条口径各一份。"""
    return {
        "pnp": (m1["pnp"][0] - m0["pnp"][0], m1["pnp"][1] - m0["pnp"][1]),
        "ruler_h": (m1["ruler_h"][0] - m0["ruler_h"][0], m1["ruler_h"][1] - m0["ruler_h"][1]),
        "yaw_deg": m1["pnp"][2] - m0["pnp"][2],
    }


def along(dx, dy, yaw_deg):
    """把位移投影到"车头方向"上：正 = 沿车头前进。"""
    return dx * math.cos(math.radians(yaw_deg)) + dy * math.sin(math.radians(yaw_deg))


def lateral(dx, dy, yaw_deg):
    """垂直车头方向的分量（直行判据）。"""
    return -dx * math.sin(math.radians(yaw_deg)) + dy * math.cos(math.radians(yaw_deg))