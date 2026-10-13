#!/usr/bin/env python3
"""用 USB 摄像头拍照。
用法:
  python3 capture_photo.py [输出文件]              # 自动对焦, 预热2秒等 AF 收敛
  python3 capture_photo.py [输出文件] --focus 275  # 关闭AF并固定焦点(桌面场景推荐)
"""
import os
import subprocess
import sys
import time
import cv2

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "new_photo.jpg"
fixed_focus = None
if "--focus" in sys.argv:
    fixed_focus = int(sys.argv[sys.argv.index("--focus") + 1])

DEV = "/dev/video0"


def ctrl(op, cid, val=None):
    ctrl_bin = os.path.join(os.path.dirname(os.path.abspath(__file__)), "set_ctrl")
    cmd = [ctrl_bin, DEV, op, cid] + ([str(val)] if val is not None else [])
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return r.stdout.strip()


cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
assert cap.isOpened()

if fixed_focus is not None:
    ctrl("set", "af", 0)
    ctrl("set", "focus", fixed_focus)
    print(f"固定焦点: af=0, focus={ctrl('get', 'focus')}")
    warm = 10
else:
    ctrl("set", "af", 1)
    warm = 60
    print("自动对焦: 预热60帧等待AF收敛...")

for _ in range(warm):
    cap.read()
ok, frame = cap.read()
assert ok, "读帧失败"
cv2.imwrite(out, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
print(f"已拍照: {out}  {frame.shape[1]}x{frame.shape[0]}")
print(f"AF={cap.get(cv2.CAP_PROP_AUTOFOCUS)}, FOCUS={cap.get(cv2.CAP_PROP_FOCUS)}")
cap.release()
