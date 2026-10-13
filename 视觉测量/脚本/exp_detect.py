#!/usr/bin/env python3
"""实验: 为什么 ID=23 未被检测 —— 尝试 尺度/阈值窗/锐化/CLAHE 各种组合"""
import os

import cv2
import numpy as np

img = cv2.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "照片", "photo5.jpg"))
crop = cv2.cvtColor(img[600:930, 170:500], cv2.COLOR_BGR2GRAY)  # 左中部码 + 边距 (灰度)
target = (335 - 170, 765 - 600)  # 码在crop内的大致中心

d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


def run(tag, g, **kw):
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    kw.pop("_scale", None)
    for k, v in kw.items():
        setattr(p, k, v)
    c, i, rej = cv2.aruco.detectMarkers(g, d, parameters=p)
    ids = [] if i is None else [int(x) for x in i.flatten()]
    hit = 23 in ids
    near_rej = len(rej) if rej else 0
    print(f"  {tag:60s} ids={ids} 拒绝候选总数={near_rej}" + ("   <<< 检测到ID23!" if hit else ""))
    return hit


print("== 不同尺度, 默认参数 ==")
for fx in [1, 2, 3, 4]:
    g = cv2.resize(crop, None, fx=fx, fy=fx, interpolation=cv2.INTER_CUBIC)
    run(f"scale={fx}x", g, _scale=fx)

print("\n== 2x + 自适应阈值窗扫参 ==")
g2 = cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
for wmin, wmax, step, const in [(3, 23, 10, 7), (5, 35, 10, 7), (7, 53, 10, 7), (11, 71, 10, 7),
                                 (5, 45, 5, 5), (9, 65, 8, 10), (15, 85, 10, 12), (3, 11, 4, 5)]:
    run(f"win={wmin}..{wmax}/{step} c={const}", g2, _scale=2,
        adaptiveThreshWinSizeMin=wmin, adaptiveThreshWinSizeMax=wmax,
        adaptiveThreshWinSizeStep=step, adaptiveThreshConstant=const)

print("\n== 2x + 预处理 (锐化/CLAHE/对比度) ==")
def unsharp(g, a=1.5, r=3):
    blur = cv2.GaussianBlur(g, (0, 0), r)
    return cv2.addWeighted(g, a, blur, 1 - a, 0)

g2u = unsharp(g2)
run("unsharp 1.5/3", g2u, _scale=2)
g2u2 = unsharp(g2, 2.0, 1.5)
run("unsharp 2.0/1.5", g2u2, _scale=2)
clahe = cv2.createCLAHE(3.0, (8, 8))
run("CLAHE 3.0", clahe.apply(g2), _scale=2)
print("\n== 原图(非crop) full image 2x + 最佳候选参数 ==")
full = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
f2 = cv2.resize(full, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
for wmin, wmax, step, const in [(3, 23, 10, 7), (5, 35, 10, 7), (5, 45, 5, 5), (7, 53, 10, 7)]:
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.adaptiveThreshWinSizeMin = wmin
    p.adaptiveThreshWinSizeMax = wmax
    p.adaptiveThreshWinSizeStep = step
    p.adaptiveThreshConstant = const
    c, i, rej = cv2.aruco.detectMarkers(f2, d, parameters=p)
    ids = [] if i is None else sorted(int(x) for x in i.flatten())
    print(f"  win={wmin}..{wmax}/{step} c={const} -> {len(ids)}个: {ids}")
