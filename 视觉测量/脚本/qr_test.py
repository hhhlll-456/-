#!/usr/bin/env python3
"""从照片中检测并提取 QR 码: QRCodeDetector + WeChat QR 两种算法对比。"""
import sys
import cv2
import numpy as np

img_path = sys.argv[1] if len(sys.argv) > 1 else "camera_photo.jpg"
img = cv2.imread(img_path)
if img is None:
    sys.exit(f"无法读取 {img_path}")
print(f"图像: {img_path}  尺寸={img.shape[1]}x{img.shape[0]}")

results = {}

# --- 方法1: cv2.QRCodeDetector ---
det = cv2.QRCodeDetector()
ok, decoded, pts, _ = det.detectAndDecodeMulti(img)
print(f"\n[QRCodeDetector] detectAndDecodeMulti -> {ok}")
if ok:
    for i, s in enumerate(decoded):
        print(f"  码{i}: {repr(s)}")
        results[f"qrdecoder_{i}"] = s
    print("  角点:", np.round(pts, 0).tolist() if pts is not None else None)
else:
    # 尝试图像增强后重试
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ups = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    ok2, dec2, pts2, _ = det.detectAndDecodeMulti(ups)
    print(f"  灰度2x放大重试 -> {ok2}")
    if ok2:
        for i, s in enumerate(dec2):
            print(f"  码{i}: {repr(s)}")
            results[f"qrdecoder_up_{i}"] = s
    # 单码接口
    data, pts1, _ = det.detectAndDecode(img)
    print(f"  单码接口 detectAndDecode -> {repr(data)}")
    if data:
        results["qrdecoder_single"] = data

# --- 方法2: WeChat QRCode (更强) ---
try:
    wdet = cv2.wechat_qrcode_WeChatQRCode if False else None
except Exception:
    pass
try:
    # 需要模型文件; 若无则跳过
    wq = cv2.wechat_qrcode.WeChatQRCode()
except Exception as e:
    wq = None
    print(f"\n[WeChatQRCode] 无模型文件, 跳过 ({type(e).__name__})")

if wq is not None:
    res, points = wq.detectAndDecode(img)
    print(f"\n[WeChatQRCode] 检测到 {len(res)} 个码")
    for i, s in enumerate(res):
        print(f"  码{i}: {repr(s)}")
        results[f"wechat_{i}"] = s

# --- 画框可视化 (原图失败时用2x放大的坐标映射回原图) ---
vis = img.copy()
ok_v, dec_v, pts_v, _ = det.detectAndDecodeMulti(img)
scale = 1.0
if not ok_v:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ups = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    ok_v, dec_v, pts_v, _ = det.detectAndDecodeMulti(ups)
    scale = 2.0
if ok_v and pts_v is not None:
    for p, s in zip(pts_v, dec_v):
        p = (p / scale).astype(np.int32)
        cv2.polylines(vis, [p], True, (0, 255, 0), 3)
        x, y = p[0]
        cv2.putText(vis, s, (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
cv2.imwrite("qr_result.jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 95])

print("\n== 结论 ==")
print(f"识别到 {len(results)} 条内容, 可视化已存 qr_result.jpg")
for k, v in results.items():
    print(f"  {k}: {v!r}")
