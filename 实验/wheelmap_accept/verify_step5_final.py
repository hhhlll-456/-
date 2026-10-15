"""Task 7 Step 5 验收（v2：单连接 + 接地门禁）。

v1 的教训：每次测试新建 TCP 连接会踩到"僵尸客户端"缺陷（重连后板子往死
socket 写 → 环形缓冲填满 → 遥测只吐几十行就静默，R62）。sweep 全程一个长
连接，所以它一直正常。**本脚本也全程只用一个连接。**

其余设计不变：
  - 接地门禁（视觉判定车真的在转）——不过门禁就不做速度判定（R63）；
  - 判据用 IMU 偏航率（车体直接观测量），不是轮速；
  - 只用自带超时的 RUN（R60）；
  - 检查 DONE / D 行数以识别 R62。
"""
import math
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # 仓库根（按脚本位置定位，换机器也能跑）
sys.path.insert(0, str(ROOT / "视觉测量" / "脚本"))
sys.path.insert(0, str(ROOT / "实验" / "vgt"))

from vision_core import calib_store              # noqa: E402
from vision_core.markers import detect_markers   # noqa: E402
from camera import CameraSession                 # noqa: E402

HOST, PORT = "192.168.1.115", 8266
MARKER_ID = 29
C1REV = [1276.0, 1270.0, 1276.0, 1274.0]
LXY, R = 0.12825, 0.0335
K = LXY / R
TOL_PCT = 15.0
OM_LIST = [-4.676, -6.0, -3.0]


def connect():
    s = socket.create_connection((HOST, PORT), timeout=6)
    s.settimeout(0.5)
    t0 = time.time()
    try:
        while time.time() - t0 < 1.5:
            if not s.recv(65536):
                break
    except Exception:
        pass
    return s


def drain(s, budget=1.2):
    s.settimeout(0.35)
    t0 = time.time()
    while time.time() - t0 < budget:
        try:
            if not s.recv(65536):
                break
        except Exception:
            break
    s.settimeout(6.0)


def run_on(s, om, ms):
    """同一个连接上跑一次 RUN 并收完整段遥测。"""
    drain(s, 1.0)
    s.sendall(f"RUN 0 0 {om} {ms}\n".encode())
    t0, buf = time.time(), b""
    deadline = ms / 1000.0 + 16.0
    while time.time() - t0 < deadline:
        try:
            ch = s.recv(65536)
        except Exception:
            break
        if not ch:
            break
        buf += ch
        if b"DONE" in buf:
            break
    s.sendall(b"STOP\n")
    return buf.decode("utf-8", "replace")


def parse_D(txt):
    rows, bad = [], 0
    for ln in txt.splitlines():
        if ln.startswith("D "):
            p = ln.split()
            if len(p) < 13:
                bad += 1
                continue
            try:
                rows.append((int(p[1]), [int(x) for x in p[2:6]],
                             [int(x) for x in p[6:10]], [float(x) for x in p[10:13]]))
            except Exception:
                bad += 1
    return rows, bad


def window(rows, lo=400, hi=1400):
    return [r for r in rows if lo <= r[0] <= hi]


def yaw_rate(st):
    if len(st) < 5:
        return None
    dt = (st[-1][0] - st[0][0]) / 1000.0
    if dt <= 0:
        return None
    yaw = 0.0
    for u, v in zip(st, st[1:]):
        d = v[3][2] - u[3][2]
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        yaw += d
    return abs(yaw / dt) * 3.14159 / 180.0


def wheels(st):
    if len(st) < 5:
        return None
    dt = (st[-1][0] - st[0][0]) / 1000.0
    if dt <= 0:
        return None
    return [(st[-1][2][i] - st[0][2][i]) / dt * (6.2832 / C1REV[i]) for i in range(4)]


def marker_angle(calib, cam):
    poses = detect_markers(cam.grab(fresh=True), calib)
    p = poses.get(MARKER_ID)
    if p is None:
        return None
    c = p.corners_px
    return math.degrees(math.atan2(float(c[1][1] - c[0][1]),
                                   float(c[1][0] - c[0][0])))


def angdiff(a, b):
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)

def main() -> int:
    """Task 7 Step 5 验收主体。

    ⚠ 会**真的连车并驱动电机**。原先这段写在模块顶层，`import` 即等于开车；
    2026-10-15 已因此专门加 `__main__` 守卫（同目录脚本一律照此办理）。
    """
    s = connect()
    try:
        # ---------------- 第 0 步：接地门禁 ----------------
        print("=== 接地门禁：车必须真的转动 ===")
        with CameraSession() as cam:
            a1 = marker_angle(calib, cam)
            if a1 is None:
                print("❌ 视觉没检到 ID29")
                return 2
            time.sleep(1.0)
            a2 = marker_angle(calib, cam)
            noise = abs(angdiff(a2, a1))
            print(f"  静帧方向角噪声 = {noise:.2f}°")
            run_on(s, -4.676, 1200)
            time.sleep(1.2)
            a3 = marker_angle(calib, cam)
            if a3 is None:
                print("❌ 转后检不到 ID29")
                return 2
            sig = abs(angdiff(a3, a1))
            print(f"  旋转后方向角变化 = {sig:.2f}°")
            if sig <= max(10 * noise, 20.0):
                print(f"\n❌ 门禁不过：车没有真的转（信号 {sig:.1f}° vs 噪声 {noise:.2f}°）。")
                print("   **不做速度判定** —— 悬空时轮速与车体速率完全脱钩（R63）。")
                return 1
            print(f"  ✅ 门禁过：车确实在转（{sig:.1f}°）\n")

        # ---------------- 第 1 步：速度跟踪 ----------------
        print("=== 速度跟踪（判据：IMU 偏航率）===")
        results = []
        for om in OM_LIST:
            txt = run_on(s, om, 1600)
            rows, bad = parse_D(txt)
            st = window(rows)
            pwm = next((l.split("pwm=")[1].split(" ms=")[0]
                        for l in txt.splitlines() if l.startswith("OK RUN") and "pwm=" in l), None)
            done, ok_run = "DONE" in txt, "OK RUN" in txt
            yr, ws = yaw_rate(st), wheels(st)

            print(f"\nRUN 0 0 {om} 1600   请求 |偏航率| = {abs(om):.2f} rad/s")
            print(f"  OK RUN={'有' if ok_run else '无'}  DONE={'有' if done else '无'}  "
                  f"D 行 {len(rows)}（坏行 {bad}）  稳态样本 {len(st)}")
            if pwm:
                print(f"  固件下发 pwm = {pwm}")
            if ws:
                print(f"  实测轮速 = {[round(x, 1) for x in ws]}  "
                      f"→ 折算车体 {sum(ws) / 4 / K:.2f} rad/s")
            if yr is None:
                print("  ⇒ 样本不足，本次无效（R62）")
                results.append((om, None))
                continue
            dev = 100 * (yr - abs(om)) / abs(om)
            print(f"  实测偏航率(IMU) = {yr:.2f} rad/s   偏差 {dev:+.1f}%   "
                  f"{'✅ 过' if abs(dev) <= TOL_PCT else '❌ 不过'}（判据 ±{TOL_PCT:.0f}%）")
            results.append((om, dev))
            time.sleep(0.8)

        print("\n———— 汇总 ————")
        for om, dev in results:
            print(f"  om={om:<8} {'无效' if dev is None else f'{dev:+.1f}%'}")
        valid = [d for _, d in results if d is not None]
        ok = len(valid) == len(OM_LIST) and all(abs(d) <= TOL_PCT for d in valid)
        print("⇒ Step 5 " + ("验收通过" if ok else "未通过"))
    finally:
        try:
            s.sendall(b"STOP\n")
        except Exception:
            pass
        s.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
