#!/usr/bin/env python3
"""检测 4 个边界角码 + 1 个车载码, 解析含义: 建立场地坐标系并计算车载码归一化位置。
多参数融合: 原图默认 -> 2x默认 -> 2x小窗 -> 2x大窗, 直到凑齐5个。
用法: python3 detect5.py <图片>
"""
import os
import sys
import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_HERE, "..", "照片", "photo5.jpg")
img = cv2.imread(path)
if img is None:
    sys.exit(f"无法读取 {path}")
H, W = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ups = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
print(f"照片 {path}  {W}x{H}")

adict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


def detect_once(g, **kw):
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    for k, v in kw.items():
        setattr(p, k, v)
    c, i, _ = cv2.aruco.detectMarkers(g, adict, parameters=p)
    return c, i


passes = [
    ("原图/默认", gray, 1.0, {}),
    ("原图/中窗", gray, 1.0, dict(adaptiveThreshWinSizeMin=7, adaptiveThreshWinSizeMax=23,
                                 adaptiveThreshWinSizeStep=8, adaptiveThreshConstant=5)),
    ("2x/默认", ups, 2.0, {}),
    ("2x/小窗", ups, 2.0, dict(adaptiveThreshWinSizeMin=3, adaptiveThreshWinSizeMax=11,
                              adaptiveThreshWinSizeStep=4, adaptiveThreshConstant=5)),
    ("2x/大窗", ups, 2.0, dict(adaptiveThreshWinSizeMin=15, adaptiveThreshWinSizeMax=85,
                              adaptiveThreshWinSizeStep=10, adaptiveThreshConstant=12)),
]

best = {}
for tag, g, s, kw in passes:
    c, i = detect_once(g, **kw)
    if i is not None:
        for cc, ii in zip(c, i.flatten()):
            ii = int(ii)
            if ii not in best:
                best[ii] = (tag, np.asarray(cc).reshape(-1, 2) / s)
    if len(best) >= 5:
        break

ids = np.array(sorted(best))
corners = [best[k][1] for k in ids]
centers = np.array([c.mean(axis=0) for c in corners])
sides = np.array([np.linalg.norm(c[1] - c[0]) for c in corners])

print(f"\n检测到 {len(ids)} 个码 (多参数融合), DICT_4X4_50:")
for k in range(len(ids)):
    print(f"  ID={ids[k]:3d}  中心=({centers[k][0]:7.1f},{centers[k][1]:7.1f})  边长={sides[k]:5.1f}px  "
          f"(首次检出: {best[ids[k]][0]})")

if len(ids) != 5:
    print(f"\n警告: 只检测到 {len(ids)} 个, 不等于 5。输出当前可视化。")
    vis = img.copy()
    for k in range(len(ids)):
        cv2.aruco.drawDetectedMarkers(vis, [corners[k].reshape(1, 4, 2)], np.array([[ids[k]]]))
    out_path = os.path.join(_HERE, "..", "结果", "detect5_result.jpg")
    cv2.imwrite(out_path, vis, [cv2.IMWRITE_JPEG_QUALITY, 95])
    sys.exit(0)

# --- 分组: 凸包顶点 = 边界码; 内部 = 车载码 ---
hull = cv2.convexHull(centers.astype(np.float32)).reshape(-1, 2)
is_boundary = [any(np.linalg.norm(ct - hp) < 5 for hp in hull) for ct in centers]
b_idx = [k for k in range(5) if is_boundary[k]]
in_idx = [k for k in range(5) if not is_boundary[k]]
if len(b_idx) != 4:
    img_corners = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)
    d = np.array([[np.linalg.norm(ct - ic) for ic in img_corners] for ct in centers])
    b_idx = list(np.argsort(d.min(axis=1))[:4])
    in_idx = [k for k in range(5) if k not in b_idx]

s = centers[:, 0] + centers[:, 1]
df = centers[:, 0] - centers[:, 1]
def pick(fn):
    return min((k for k in b_idx), key=lambda k: fn(k))
TL = pick(lambda k: s[k]); BR = pick(lambda k: -s[k])
TR = pick(lambda k: -df[k]); BL = pick(lambda k: df[k])
named = {TL: "TL(左上)", TR: "TR(右上)", BR: "BR(右下)", BL: "BL(左下)"}

print("\n== 分组与命名 (按画面方位) ==")
for k in (TL, TR, BR, BL):
    print(f"  边界码 ID={ids[k]:3d} -> {named[k]}  中心=({centers[k][0]:.0f},{centers[k][1]:.0f})  边长={sides[k]:.0f}px")
for k in in_idx:
    print(f"  内部码 ID={ids[k]:3d} -> 车载/物体目标  中心=({centers[k][0]:.0f},{centers[k][1]:.0f})  边长={sides[k]:.0f}px (最大: {sides[k] == sides.max()})")

# --- 单应变换: 4角中心 -> 单位方形 ---
src = np.array([centers[TL], centers[TR], centers[BR], centers[BL]], np.float32)
dst = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
Hm = cv2.getPerspectiveTransform(src, dst)
print("\n== 场地坐标系 (由4个边界码确定) ==")
print("  归一化单位: (0,0)=左上角码, (1,0)=右上, (1,1)=右下, (0,1)=左下")
for k in in_idx:
    p = Hm @ np.array([*centers[k], 1.0]); p = p[:2] / p[2]
    print(f"  车载码 ID={ids[k]} 场地坐标 = ({p[0]:.3f}, {p[1]:.3f})  "
          f"= 场地宽度的 {p[0]*100:.1f}%, 高度的 {p[1]*100:.1f}%")

inv = np.linalg.inv(Hm)
for k in (TL, TR, BR, BL):
    un = dst[{TL: 0, TR: 1, BR: 2, BL: 3}[k]]
    q = inv @ np.array([un[0], un[1], 1.0]); q = q[:2] / q[2]
    print(f"  反投影校验 {named[k]}: 误差 {np.linalg.norm(q - centers[k]):.3f}px")

# --- 可视化 ---
vis = img.copy()
poly = np.array([centers[TL], centers[TR], centers[BR], centers[BL]], np.int32)
cv2.polylines(vis, [poly], True, (0, 255, 0), 3)
for k, name in named.items():
    x, y = centers[k]
    cv2.aruco.drawDetectedMarkers(vis, [corners[k].reshape(1, 4, 2)], np.array([[ids[k]]]))
    cv2.putText(vis, f"{name} ID={ids[k]}", (int(x) - 130, int(y) - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3)
for k in in_idx:
    x, y = centers[k]
    cv2.aruco.drawDetectedMarkers(vis, [corners[k].reshape(1, 4, 2)], np.array([[ids[k]]]))
    p = Hm @ np.array([x, y, 1.0]); p = p[:2] / p[2]
    cv2.circle(vis, (int(x), int(y)), 8, (0, 0, 255), -1)
    cv2.putText(vis, f"CAR ID={ids[k]} ({p[0]:.2f},{p[1]:.2f})", (int(x) - 160, int(y) - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
out_path = os.path.join(_HERE, "..", "结果", "detect5_result.jpg")
cv2.imwrite(out_path, vis, [cv2.IMWRITE_JPEG_QUALITY, 95])
print(f"\n可视化已存 {out_path}")
