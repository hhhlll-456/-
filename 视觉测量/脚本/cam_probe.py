#!/usr/bin/env python3
"""探测 USB 摄像头：连接、枚举能力、拍照、读取参数、检测自动对焦支持。"""
import cv2
import json
import sys

DEV = "/dev/video0"

PROP_NAMES = {
    "POS_MSEC": cv2.CAP_PROP_POS_MSEC,
    "FRAME_WIDTH": cv2.CAP_PROP_FRAME_WIDTH,
    "FRAME_HEIGHT": cv2.CAP_PROP_FRAME_HEIGHT,
    "FPS": cv2.CAP_PROP_FPS,
    "FOURCC": cv2.CAP_PROP_FOURCC,
    "FRAME_COUNT": cv2.CAP_PROP_FRAME_COUNT,
    "FORMAT": cv2.CAP_PROP_FORMAT,
    "MODE": cv2.CAP_PROP_MODE,
    "BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
    "CONTRAST": cv2.CAP_PROP_CONTRAST,
    "SATURATION": cv2.CAP_PROP_SATURATION,
    "HUE": cv2.CAP_PROP_HUE,
    "GAIN": cv2.CAP_PROP_GAIN,
    "EXPOSURE": cv2.CAP_PROP_EXPOSURE,
    "AUTO_EXPOSURE": cv2.CAP_PROP_AUTO_EXPOSURE,
    "WHITE_BALANCE": cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,
    "BACKLIGHT": cv2.CAP_PROP_BACKLIGHT,
    "GAMMA": cv2.CAP_PROP_GAMMA,
    "TEMPERATURE": cv2.CAP_PROP_TEMPERATURE,
    "AUTOFOCUS": cv2.CAP_PROP_AUTOFOCUS,
    "FOCUS": cv2.CAP_PROP_FOCUS,
    "ISO_SPEED": cv2.CAP_PROP_ISO_SPEED,
    "BUFFERSIZE": cv2.CAP_PROP_BUFFERSIZE,
    "CONVERT_RGB": cv2.CAP_PROP_CONVERT_RGB,
    "RECTIFICATION": cv2.CAP_PROP_RECTIFICATION,
    "MONOCHROME": cv2.CAP_PROP_MONOCHROME,
    "SHARPNESS": cv2.CAP_PROP_SHARPNESS,
}


def fourcc_str(v):
    try:
        i = int(v)
        return "".join(chr((i >> 8 * k) & 0xFF) for k in range(4))
    except Exception:
        return "?"


def main():
    print("=" * 60)
    print("1) 连接摄像头:", DEV)
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("   打开失败!")
        sys.exit(1)
    print("   打开成功, backend = V4L2")

    # 默认参数
    print("\n" + "=" * 60)
    print("2) 默认打开时的当前参数:")
    for k, p in PROP_NAMES.items():
        v = cap.get(p)
        extra = ""
        if k == "FOURCC":
            extra = f" -> '{fourcc_str(v)}'"
        if v == -1 and k in ("AUTOFOCUS", "FOCUS"):
            print(f"   {k:16s} = {v}  (不支持/未实现){extra}")
        else:
            print(f"   {k:16s} = {v}{extra}")

    # 枚举能力
    print("\n" + "=" * 60)
    print("3) 摄像头支持的模式 (OpenCV backend enumeration):")
    backends = []
    # OpenCV 的 enumeration 在 VideoCapture 构造时带参数比较麻烦，用 backend API
    try:
        modes = cv2.videoio_registry.getStreamBackend if False else None
    except Exception:
        pass
    # 用 getBackendName + 逐一分辨率试探
    for fourcc in ["MJPG", "YUYV", "H264"]:
        for w, h in [(3840, 2160), (2560, 1440), (1920, 1080), (1600, 1200),
                     (1280, 720), (960, 540), (800, 600), (640, 480)]:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            aw, ah, af = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FPS)
            if (int(aw), int(ah)) == (w, h):
                print(f"   OK  {fourcc} {int(aw)}x{int(ah)} @ {af:.1f} fps")

    # 设置为 1080p MJPG 拍照
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # 自动对焦测试：读/写 AUTOFOCUS 与 FOCUS
    print("\n" + "=" * 60)
    print("4) 自动对焦属性测试:")
    af0 = cap.get(cv2.CAP_PROP_AUTOFOCUS)
    print(f"   读取 CAP_PROP_AUTOFOCUS = {af0}")
    ok_on = cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    print(f"   设置 AUTOFOCUS=1 -> {ok_on}, 现值 {cap.get(cv2.CAP_PROP_AUTOFOCUS)}")
    ok_off = cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    print(f"   设置 AUTOFOCUS=0 -> {ok_off}, 现值 {cap.get(cv2.CAP_PROP_AUTOFOCUS)}")
    # 恢复连续对焦
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    f0 = cap.get(cv2.CAP_PROP_FOCUS)
    print(f"   读取 CAP_PROP_FOCUS (绝对焦距) = {f0}")
    print(f"   设置 FOCUS=30 -> {cap.set(cv2.CAP_PROP_FOCUS, 30)}, 现值 {cap.get(cv2.CAP_PROP_FOCUS)}")
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # 恢复 AF

    # 预热 + 拍照
    print("\n" + "=" * 60)
    print("5) 预热并拍照:")
    for i in range(15):
        cap.read()
    ok, frame = cap.read()
    if not ok:
        print("   读帧失败!")
        sys.exit(1)
    print(f"   实际帧尺寸 = {frame.shape[1]}x{frame.shape[0]}, dtype={frame.dtype}")
    print(f"   实际 FPS 属性 = {cap.get(cv2.CAP_PROP_FPS)}")
    out = "camera_photo.jpg"
    cv2.imwrite(out, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f"   已保存: {out}")

    # 拍照后的最终参数快照
    print("\n" + "=" * 60)
    print("6) 拍照时的参数快照:")
    snap = {}
    for k, p in PROP_NAMES.items():
        v = cap.get(p)
        snap[k] = float(v)
        if k == "FOURCC":
            print(f"   {k:16s} = {v} -> '{fourcc_str(v)}'")
        else:
            print(f"   {k:16s} = {v}")
    with open("camera_props.json", "w") as f:
        json.dump(snap, f, indent=2)

    cap.release()
    print("\n完成。")


if __name__ == "__main__":
    main()
