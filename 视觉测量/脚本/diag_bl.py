#!/usr/bin/env python3
"""诊断左侧未识别码: 1) 宽松参数重检测 2) 透视矫正+比特级解析, 直接匹配所有字典ID"""
import os

import cv2
import numpy as np

img = cv2.imread(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "照片", "photo5.jpg"))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# ---------- 1) 宽松参数重检测 ----------
print("== 宽松参数检测 ==")
params = cv2.aruco.DetectorParameters_create()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.minMarkerPerimeterRate = 0.005
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 71
params.adaptiveThreshWinSizeStep = 8
params.adaptiveThreshConstant = 7
params.polygonalApproxAccuracyRate = 0.08
params.maxErroneousBitsInBorderRate = 0.8
params.errorCorrectionRate = 1.0
params.minDistanceToBorder = 0
d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
ups = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
for tag, g in (("原图", gray), ("2x", ups)):
    c, i, _ = cv2.aruco.detectMarkers(g, d, parameters=params)
    if i is not None:
        s = 2.0 if tag == "2x" else 1.0
        print(f"  [{tag}] 检测到 {len(i)} 个:", [(int(x), (int(np.asarray(cc).reshape(-1,2).mean(0)[0]/s), int(np.asarray(cc).reshape(-1,2).mean(0)[1]/s))) for cc, x in zip(c, i.flatten())])
    else:
        print(f"  [{tag}] 无")

# ---------- 2) 比特级解析左侧码 ----------
print("\n== 比特级解析 (左中部码) ==")
crop = img[660:870, 230:440]
g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
g = cv2.resize(g, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)

blur = cv2.GaussianBlur(g, (5, 5), 0)
th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 61, 12)
cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
c = max(cnts, key=cv2.contourArea)
approx = cv2.approxPolyDP(c, 0.03 * cv2.arcLength(c, True), True)
print(f"  外框角点数: {len(approx)}")

pts = approx.reshape(-1, 2).astype(np.float32)
s = pts.sum(1); df = pts[:, 0] - pts[:, 1]
tl = pts[np.argmin(s)]; br = pts[np.argmax(s)]
tr = pts[np.argmax(df)]; bl = pts[np.argmin(df)]
src = np.array([tl, tr, br, bl], np.float32)
dst = np.array([[0, 0], [239, 0], [239, 239], [0, 239]], np.float32)
M = cv2.getPerspectiveTransform(src, dst)
warp = cv2.warpPerspective(g, M, (240, 240))
_, bw = cv2.threshold(warp, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
cv2.imwrite("marker_rectified.png", bw)

# 采样 6x6 单元 (1=白, 0=黑)
cells = np.zeros((6, 6), np.uint8)
for r in range(6):
    for cc in range(6):
        y = int((r + 0.5) * 40); x = int((cc + 0.5) * 40)
        cells[r, cc] = 1 if bw[y - 8:y + 8, x - 8:x + 8].mean() > 127 else 0
print("  6x6 单元矩阵 (1=白):")
for r in range(6):
    print("   ", "".join(str(v) for v in cells[r]))
border_ok = cells[0].sum() == 0 and cells[5].sum() == 0 and cells[:, 0].sum() == 0 and cells[:, 5].sum() == 0
print(f"  黑色边框完整: {border_ok}")

obs = cells[1:5, 1:5]


def sample_marker_img(m, ncell):
    side = ncell * 10
    _, b = cv2.threshold(m, 127, 255, cv2.THRESH_BINARY)
    out = np.zeros((ncell, ncell), np.uint8)
    for r in range(ncell):
        for cc in range(ncell):
            y = int((r + 0.5) * 10); x = int((cc + 0.5) * 10)
            out[r, cc] = 1 if b[y - 2:y + 3, x - 2:x + 3].mean() > 127 else 0
    return out


print("\n  与各字典ID比对 (汉明距离, 含4旋转+镜像):")
cand = []
for dname, nid in [("DICT_4X4_50", 50), ("DICT_4X4_100", 100), ("DICT_4X4_250", 250), ("DICT_4X4_1000", 1000)]:
    dd = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dname))
    for mid in range(nid):
        m = cv2.aruco.drawMarker(dd, mid, 60)
        gm = sample_marker_img(m, 6)
        gb = gm[1:5, 1:5]
        for rot in range(4):
            h = int(np.sum(obs != np.rot90(gb, rot)))
            cand.append((h, dname, mid, rot, False))
        for rot in range(4):
            h = int(np.sum(obs != np.rot90(np.fliplr(gb), rot)))
            cand.append((h, dname, mid, rot, True))
cand.sort()
for h, dname, mid, rot, mir in cand[:8]:
    print(f"    距离={h:2d}/16  {dname} ID={mid:3d} 旋转={rot*90}° {'(镜像!)' if mir else ''}")

best = cand[0]
print(f"\n  结论: 最匹配 {best[1]} ID={best[2]} (旋转{best[3]*90}°, 镜像={best[4]}), 汉明距离 {best[0]}/16")
