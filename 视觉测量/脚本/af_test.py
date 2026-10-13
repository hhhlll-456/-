#!/usr/bin/env python3
"""自动对焦实证实验:
对比 AF开启 vs 手动对焦行程两端(focus=0 近焦 / focus=1023 远焦)的图像清晰度。
若手动两端明显更糊、AF 恢复后变清晰 -> 证明对焦执行器有效且 AF 在工作。
"""
import subprocess
import time
import cv2
import numpy as np

DEV = "/dev/video0"
CTRL = "./set_ctrl"


def ctrl(op, cid, val=None):
    args = [CTRL, DEV, op, cid] + ([str(val)] if val is not None else [])
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{args} failed: {r.stderr.strip()}")
    return r.stdout.strip()


def sharpness(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 全图 + 中心ROI
    full = cv2.Laplacian(gray, cv2.CV_64F).var()
    h, w = gray.shape
    roi = gray[h // 4:3 * h // 4, w // 4:3 * w // 4]
    mid = cv2.Laplacian(roi, cv2.CV_64F).var()
    return full, mid


def grab(cap, name, n=10):
    for _ in range(n):
        cap.read()
    ok, frame = cap.read()
    assert ok, name
    f, m = sharpness(frame)
    cv2.imwrite(f"af_{name}.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    return f, m, frame


def main():
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    assert cap.isOpened()

    print("初始状态:")
    print("  focus_abs  =", ctrl("get", "focus"))
    print("  focus_auto =", ctrl("get", "af"))

    results = {}

    # 1) AF 开启
    ctrl("set", "af", 1)
    time.sleep(2.5)
    results["1_af_on"] = grab(cap, "af_on")
    print(f"[AF on ] 全图清晰度={results['1_af_on'][0]:.0f}  中心ROI={results['1_af_on'][1]:.0f}")

    # 2) 关 AF, 手动近焦 focus=0
    ctrl("set", "af", 0)
    time.sleep(0.3)
    print("  focus_auto ->", ctrl("get", "af"))
    ctrl("set", "focus", 0)
    time.sleep(1.0)
    print("  focus ->", ctrl("get", "focus"))
    results["2_focus_0_near"] = grab(cap, "focus_0")
    print(f"[focus=0] 全图={results['2_focus_0_near'][0]:.0f}  中心ROI={results['2_focus_0_near'][1]:.0f}")

    # 3) 手动远焦 focus=1023
    ctrl("set", "focus", 1023)
    time.sleep(1.0)
    print("  focus ->", ctrl("get", "focus"))
    results["3_focus_1023_far"] = grab(cap, "focus_1023")
    print(f"[focus=1023] 全图={results['3_focus_1023_far'][0]:.0f}  中心ROI={results['3_focus_1023_far'][1]:.0f}")

    # 4) 恢复 AF
    ctrl("set", "af", 1)
    time.sleep(2.5)
    results["4_af_restore"] = grab(cap, "af_restore")
    print(f"[AF恢复] 全图={results['4_af_restore'][0]:.0f}  中心ROI={results['4_af_restore'][1]:.0f}")

    print("\n== 汇总 (Laplacian 方差, 越大越清晰) ==")
    a = results["1_af_on"][1]
    for k, (f, m, _) in results.items():
        bar = "#" * int(m / max(x[1] for x in results.values()) * 50)
        print(f"  {k:22s} 全图={f:8.0f}  中心={m:8.0f}  相对AF={m / a * 100:5.1f}%  {bar}")

    cap.release()


if __name__ == "__main__":
    main()
