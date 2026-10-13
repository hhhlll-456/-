#!/usr/bin/env python3
"""实时 ArUco 检测预览窗口 (DICT_4X4_50)
按键: q/ESC 退出 | g 网格开关 | s 保存快照
"""
import time
import cv2
import numpy as np

DEV = "/dev/video0"
DICT = cv2.aruco.DICT_4X4_50
SCALE = 0.75

cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
assert cap.isOpened(), "摄像头打开失败"

aruco_dict = cv2.aruco.Dictionary_get(DICT)
params = cv2.aruco.DetectorParameters_create()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.adaptiveThreshWinSizeMin = 7
params.adaptiveThreshWinSizeMax = 23
params.adaptiveThreshWinSizeStep = 8
params.adaptiveThreshConstant = 5

win = "ArUco Live 1920x1080 (q/ESC quit | g grid | s snapshot)"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, int(1920 * SCALE), int(1080 * SCALE))
cv2.moveWindow(win, 0, 0)

show_grid = True
fps = 0.0
t_prev = time.perf_counter()
last_msg = None
print("窗口已启动 (DISPLAY=%s)" % __import__("os").environ.get("DISPLAY"), flush=True)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败, 退出", flush=True)
            break
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if show_grid:
            for x in range(0, w + 1, 240):
                cv2.line(frame, (x, 0), (x, h), (0, 170, 0), 1)
                cv2.putText(frame, str(x), (x + 4, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 170, 0), 1)
            for y in range(240, h, 240):
                cv2.line(frame, (0, y), (w, y), (0, 170, 0), 1)
                cv2.putText(frame, str(y), (4, y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 170, 0), 1)

        cx, cy = w // 2, h // 2
        cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 44, 2)

        now = time.perf_counter()
        fps = fps * 0.9 + (1.0 / max(now - t_prev, 1e-6)) * 0.1
        t_prev = now

        cv2.rectangle(frame, (0, 0), (w, 44), (0, 0, 0), -1)
        cv2.putText(frame, f"MJPG {w}x{h}  {fps:4.1f} FPS  DICT_4X4_50",
                    (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 2)

        if ids is not None:
            parts = []
            for c, i in zip(corners, ids.flatten()):
                m = np.asarray(c).reshape(-1, 2).mean(axis=0)
                parts.append(f"ID={int(i)} @({int(m[0])},{int(m[1])})")
            msg = f"检测到 {len(ids)} 个: " + "  ".join(parts)
            cv2.putText(frame, msg, (10, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
        else:
            msg = "未检测到 ArUco 码"
            cv2.putText(frame, msg, (10, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 80, 255), 2)

        if msg != last_msg:
            print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)
            last_msg = msg

        disp = cv2.resize(frame, None, fx=SCALE, fy=SCALE) if SCALE != 1.0 else frame
        cv2.imshow(win, disp)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("g"):
            show_grid = not show_grid
        if k == ord("s"):
            fn = f"snapshot_{int(time.time())}.jpg"
            cv2.imwrite(fn, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            print("已保存快照", fn, flush=True)
finally:
    cap.release()
    cv2.destroyAllWindows()
    print("窗口已关闭", flush=True)
