#!/usr/bin/env python3
"""ArUco 码检测: 遍历所有预定义字典, 对比原图/2x放大 的检测能力。
用法: python3 aruco_test.py <图片> [--qr-check]
"""
import sys
import time
import cv2
import numpy as np

img_path = sys.argv[1] if len(sys.argv) > 1 else "new_photo.jpg"
qr_check = "--qr-check" in sys.argv

img = cv2.imread(img_path)
if img is None:
    sys.exit(f"无法读取 {img_path}")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ups = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
print(f"图像: {img_path}  {img.shape[1]}x{img.shape[0]}")

params = cv2.aruco.DetectorParameters_create()
try:
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
except Exception:
    pass

dict_names = sorted(n for n in dir(cv2.aruco) if n.startswith("DICT_"))
print(f"预定义字典 {len(dict_names)} 个\n")

found = []
for name in dict_names:
    try:
        d = cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))
    except Exception:
        continue
    for tag, g in (("原图", gray), ("2x放大", ups)):
        t0 = time.perf_counter()
        corners, ids, _ = cv2.aruco.detectMarkers(g, d, parameters=params)
        dt = (time.perf_counter() - t0) * 1000
        if ids is not None and len(ids) > 0:
            found.append((name, tag, corners, ids, dt))
            break

if not found:
    print("未检测到任何 ArUco 码 (所有字典, 原图+2x)")
else:
    print("== 检测结果 ==")
    for name, tag, corners, ids, dt in found:
        print(f"字典 {name}: 在[{tag}]检测到 {len(ids)} 个码, 耗时 {dt:.0f}ms")
        for c, i in zip(corners, ids.flatten()):
            c = np.asarray(c).reshape(-1, 2)
            side = np.linalg.norm(c[1] - c[0])
            print(f"   ID={i}  中心=({c[:,0].mean():.0f},{c[:,1].mean():.0f})  边长≈{side:.0f}px")

    # 可视化 (坐标映射回原图)
    vis = img.copy()
    for name, tag, corners, ids, dt in found:
        s = 2.0 if tag == "2x放大" else 1.0
        for c, i in zip(corners, ids.flatten()):
            p = (np.asarray(c).reshape(-1, 2) / s).astype(np.int32)
            cv2.polylines(vis, [p], True, (0, 255, 0), 3)
            x, y = p[0]
            cv2.putText(vis, f"ID={i}", (int(x), int(y) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
            cv2.circle(vis, (int(p[:, 0].mean()), int(p[:, 1].mean())), 6, (255, 0, 0), -1)
    cv2.imwrite("aruco_result.jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print("可视化已存 aruco_result.jpg")

if qr_check:
    print("\n== 对照: 同一张照片的 QR 检测 ==")
    det = cv2.QRCodeDetector()
    ok, dec, _, _ = det.detectAndDecodeMulti(img)
    if not ok:
        ok, dec, _, _ = det.detectAndDecodeMulti(ups)
        if ok:
            print(f"  QRCodeDetector(2x放大): {list(dec)}")
    else:
        print(f"  QRCodeDetector(原图): {list(dec)}")
    if not ok:
        print("  QRCodeDetector: 未检测到 QR")
    try:
        wq = cv2.wechat_qrcode.WeChatQRCode()
        res, _ = wq.detectAndDecode(img)
        print(f"  WeChatQRCode(原图): {list(res) if res else '未检测到'}")
    except Exception as e:
        print(f"  WeChatQRCode 不可用: {e}")
