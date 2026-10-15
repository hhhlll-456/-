"""判定：车在做原地旋转时到底转没转 —— 用视觉，不用编码器、不用 IMU。

原理：`detect_markers` 返回的 `corners_px` 是**跟着码走的规范次序**，
所以 (c1 - c0) 是码面的一个固定方向。图像里这个向量的角度只随车体姿态变。
先用两张静帧量噪声，再跑一次自带超时的 RUN，再拍一张量信号。

为什么必须做这一步：编码器测的是"轮子转了多少"，不是"车动了多少"。
轮子空转 / 打滑时两者完全脱钩（R63）。今天已经被架空状态误导过一次。
"""
import math
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # 仓库根（按脚本位置定位，换机器也能跑）
sys.path.insert(0, str(ROOT / "视觉测量" / "脚本"))
sys.path.insert(0, str(ROOT / "实验" / "vgt"))

from vision_core import calib_store          # noqa: E402
from vision_core.markers import detect_markers  # noqa: E402
from camera import CameraSession             # noqa: E402

MARKER_ID = 29
HOST, PORT = "192.168.1.115", 8266


def marker_angle(calib, session, tag):
    """返回 (角度deg, 位置mm, 重投影RMS)。角度 = 图像系下 (c1-c0) 的方向角。"""
    frame = session.grab(fresh=True)
    poses = detect_markers(frame, calib)
    p = poses.get(MARKER_ID)
    if p is None:
        print(f"  [{tag}] ❌ 没检到 ID{MARKER_ID}（可见的：{sorted(poses)}）")
        return None, None, None
    c = p.corners_px
    dx = float(c[1][0] - c[0][0])
    dy = float(c[1][1] - c[0][1])
    ang = math.degrees(math.atan2(dy, dx))
    t = p.tvec.ravel()
    print(f"  [{tag}] 方向角 {ang:8.2f}°   tvec=({t[0]:7.1f},{t[1]:7.1f},{t[2]:7.1f}) mm  "
          f"RMS {p.reproj_rms_px:.2f}px")
    return ang, t, p.reproj_rms_px


def angdiff(a, b):
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def run_pivot(om, ms):
    s = socket.create_connection((HOST, PORT), timeout=6)
    s.settimeout(0.5)
    t0 = time.time()
    try:
        while time.time() - t0 < 1.0:          # 排空
            if not s.recv(65536):
                break
    except Exception:
        pass
    s.sendall(f"RUN 0 0 {om} {ms}\n".encode())
    time.sleep(ms / 1000.0 + 2.0)
    s.sendall(b"STOP\n")
    time.sleep(0.4)
    s.close()


def main() -> int:
    """跑一次：静帧量噪声 → 发一条 1.6 s 原地旋转 → 再量一次。

    ⚠ 会**真的驱动电机**（`RUN 0 0 -4.676 1600`）。原先这段写在模块顶层，
    于是 `import vision_rotate_check` 就等于开车（2026-10-15 已因此误驱动一次），
    现在收进 `main()` 并由下面的 `__main__` 守卫把住。
    """
    calib_path = calib_store.DEFAULT_CALIB_PATH
    calib = calib_store.load_calib(calib_path)
    print(f"标定: {calib_path}")

    with CameraSession() as cam:
        a1, t1, r1 = marker_angle(calib, cam, "静帧A")
        time.sleep(1.0)
        a2, t2, r2 = marker_angle(calib, cam, "静帧B（噪声基线）")
        if a1 is None or a2 is None:
            return 2

        noise = abs(angdiff(a2, a1))
        print(f"\n静帧噪声 = {noise:.2f}°  （车绝对没动，这就是测量噪声底）")

        print("\n发 RUN 0 0 -4.676 1600（请求 268°/s 的偏航，应转约 430°）…")
        run_pivot(-4.676, 1600)
        time.sleep(1.5)

        a3, t3, r3 = marker_angle(calib, cam, "转后")
        if a3 is None:
            return 2

    sig = abs(angdiff(a3, a1))
    print(f"\n信号（转后 vs 静帧A）= {sig:.2f}°    噪声底 = {noise:.2f}°")
    print(f"位置变化 = ({t3[0]-t1[0]:+.1f}, {t3[1]-t1[1]:+.1f}) mm")

    if sig > max(5 * noise, 20.0):
        print(f"\n⇒ ✅ 车确实转了（信号 {sig:.1f}° 远超噪声 {noise:.1f}°）")
        return 0
    print(f"\n⇒ ❌ 车没转（信号 {sig:.1f}° 与噪声 {noise:.1f}° 同级）—— 轮子在空转/打滑")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
