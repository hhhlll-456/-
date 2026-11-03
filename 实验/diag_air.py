#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""架空诊断（车已抬起、四轮悬空）：验证固件 / IMU / 编码器 / 电机。

流程（全程串口监听抓崩溃，命令走 Wi-Fi TCP——固件命令只从 TCP 进）：
  1. 链路：PING / ID / CHIP / HELP（确认烧的是 exp 固件）
  2. SNIFF：GPIO37/38 边沿计数（验证 IMU TX 接线）
  3. 静置 12s：yaw 漂移率、roll/pitch 零位、编码器假计数、IMU 实际更新率
  4. 逐轮 M 测试：四轮同占空比，比较空载转速一致性（直行跑偏的执行器证据）
  5. RUN 直行 + RUN 原地旋转：遥测里的 yaw/roll/pitch 对指令的响应
  6. 结果文本 + 原始行日志存档

用法：python3 diag_air.py
"""
import os
import sys
import socket
import time
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from car_link import discover, TCP_PORT

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "原始日志")
SERIAL_PORT = "/dev/ttyACM0"
TEST_PWM = 130          # 架空逐轮测试占空比
WHEEL_FWD_PWM = [-TEST_PWM, TEST_PWM, -TEST_PWM, TEST_PWM]  # 正向 omega 对应的 PWM 符号（invert 表）
C_1REV = [1276, 1270, 1276, 1274]   # 标定值 FL/FR/RL/RR
TABLE_OMEGA_AT_130 = [26.1, 26.6, 25.1, 28.0]  # wheel_map 表 130duty 插值近似 rad/s
WHEEL_NAMES = ["FL", "FR", "RL", "RR"]


# ---------------- 串口监听（抓复位/Guru Meditation） ----------------
class SerialMonitor(threading.Thread):
    def __init__(self, port, out_path):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.out = open(out_path, "a", encoding="utf-8")
        self.stop_flag = False
        self.events = []          # 崩溃/复位关键词行

    def run(self):
        while not self.stop_flag:
            try:
                line = self.ser.readline().decode("utf-8", "replace").strip()
            except Exception:
                break
            if line:
                self.out.write(f"[{time.time():.3f}] {line}\n")
                self.out.flush()
                low = line.lower()
                if any(k in low for k in ("guru", "abort", "panic", "rst:0x", "brownout", "wifi failed")):
                    self.events.append((time.time(), line))

    def close(self):
        self.stop_flag = True
        time.sleep(0.4)
        try:
            self.ser.close()
        except Exception:
            pass
        self.out.close()


# ---------------- TCP 原始链路（能分离遥测行与命令回复） ----------------
class RawLink:
    def __init__(self, ip, port=TCP_PORT):
        self.s = socket.create_connection((ip, port), timeout=5)
        self.s.settimeout(0.2)
        self.buf = b""

    def send(self, line):
        self.s.sendall((line + "\n").encode())

    def _read_once(self):
        try:
            d = self.s.recv(8192)
            if d:
                self.buf += d
        except socket.timeout:
            pass
        lines = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            lines.append(raw.decode("utf-8", "replace").strip())
        return lines

    def collect(self, dur, stop_words=("DONE",), extra_stop=None):
        """收 dur 秒内的所有行；遇到 stop_words 或 非遥测行也收集到 replies。"""
        t0 = time.time()
        data, replies = [], []
        while time.time() - t0 < dur:
            for ln in self._read_once():
                if ln.startswith(("I ", "D ")):
                    data.append(ln)
                else:
                    replies.append(ln)
                    if extra_stop and ln in extra_stop:
                        return data, replies, time.time() - t0
                    if ln in stop_words:
                        return data, replies, time.time() - t0
        return data, replies, time.time() - t0

    def cmd(self, line, timeout=4.0):
        self.send(line)
        t0 = time.time()
        replies = []
        while time.time() - t0 < timeout:
            for ln in self._read_once():
                if ln.startswith(("I ", "D ")):
                    continue
                replies.append(ln)
                return ln, replies
        return None, replies

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


# ---------------- 解析 ----------------
def parse_line(ln):
    p = ln.split()
    tag = p[0]
    t = int(p[1])
    pwm = list(map(int, p[2:6]))
    enc = list(map(int, p[6:10]))
    rpy = list(map(float, p[10:13]))
    return tag, t, pwm, enc, rpy


def yaw_drift_rate(samples):
    """线性拟合 yaw(t) -> deg/min。"""
    pts = [(t, y) for _, t, _, _, (r, p_, y) in samples]
    n = len(pts)
    if n < 10:
        return None
    mt = sum(t for t, _ in pts) / n
    my = sum(y for _, y in pts) / n
    st = sum((t - mt) ** 2 for t, _ in pts)
    if st == 0:
        return None
    k = sum((t - mt) * (y - my) for t, y in pts) / st       # deg/ms
    resid = [y - (my + k * (t - mt)) for t, y in pts]
    rms = (sum(r * r for r in resid) / n) ** 0.5
    return k * 1000 * 60, rms                                # deg/min, deg


def imu_update_rate(samples):
    """telemetry 100Hz，但 IMU 串口 9600 波特带宽有限——测 rpy 实际变化频率。"""
    changes = 0
    prev = None
    for _, _, _, _, rpy in samples:
        if prev is not None and rpy != prev:
            changes += 1
        prev = rpy
    t0, t1 = samples[0][1], samples[-1][1]
    span = max(t1 - t0, 1)
    return changes / (span / 1000.0)


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    raw_log = os.path.join(LOG_DIR, f"diag_air_{stamp}.txt")
    report = []
    def log(msg):
        print(msg)
        report.append(msg)

    # 0. 串口监听（打开会复位 ESP32 —— 正好从启动日志开始抓）
    mon = SerialMonitor(SERIAL_PORT, raw_log)
    mon.start()
    log(f"[0] 串口监听 {SERIAL_PORT}（打开会复位板子，等 Wi-Fi 重连）")
    time.sleep(8)   # 启动 + Wi-Fi 连接（实测约 3~6s）

    car = None
    try:
        ip = discover() or ("192.168.1.115", TCP_PORT)
        if isinstance(ip, tuple):
            ip = ip[0]
        log(f"[1] TCP 连接 {ip}:{TCP_PORT}")
        car = RawLink(ip)

        # ---- 1. 链路/固件身份 ----
        for c in ("PING", "ID", "CHIP", "HELP"):
            r, _ = car.cmd(c)
            log(f"    {c:5s} -> {r}")
        log(f"    SNIFF -> {car.cmd('SNIFF')[0]}")

        # ---- 2. 静置基线 ----
        car.cmd("STREAM 1")
        data, _, _ = car.collect(12.0, stop_words=())
        car.cmd("STREAM 0")
        samples = [parse_line(ln) for ln in data]
        log(f"[2] 静置 {len(samples)} 样本 ({samples[-1][1]-samples[0][1]} ms)")
        if samples:
            roll = [s[4][0] for s in samples]
            pitch = [s[4][1] for s in samples]
            yaw = [s[4][2] for s in samples]
            log(f"    roll  {min(roll):.2f}..{max(roll):.2f} deg   (均值 {sum(roll)/len(roll):+.2f})")
            log(f"    pitch {min(pitch):.2f}..{max(pitch):.2f} deg   (均值 {sum(pitch)/len(pitch):+.2f})")
            log(f"    yaw   {min(yaw):.2f}..{max(yaw):.2f} deg")
            drift = yaw_drift_rate(samples)
            if drift:
                log(f"    yaw 漂移率 {drift[0]:+.2f} deg/min (拟合残差 RMS {drift[1]:.3f} deg)")
            rate = imu_update_rate(samples)
            log(f"    IMU 实际更新率约 {rate:.0f} Hz (遥测 100Hz)")
            enc0 = samples[0][3]
            enc1 = samples[-1][3]
            delta = [b - a for a, b in zip(enc0, enc1)]
            log(f"    静置编码器增量 {delta} （应全为 0，非 0 = 电气噪声假计数）")

        # ---- 3. 逐轮空载转速 ----
        log(f"[3] 逐轮测试 PWM={WHEEL_FWD_PWM} (正向)")
        wheel_rates = []
        for w in range(4):
            car.cmd("ZENC")
            m = [0, 0, 0, 0]
            m[w] = WHEEL_FWD_PWM[w]
            car.cmd("M %d %d %d %d" % tuple(m))
            data, _, _ = car.collect(2.0, stop_words=())
            car.cmd("STOP")
            samples = [parse_line(ln) for ln in data]
            if len(samples) < 5:
                log(f"    {WHEEL_NAMES[w]}: 样本太少 ({len(samples)}) —— 检查")
                wheel_rates.append(None)
                continue
            denc = samples[-1][3][w] - samples[0][3][w]
            dt_s = (samples[-1][1] - samples[0][1]) / 1000.0
            cps = denc / dt_s if dt_s > 0 else 0
            om = cps * 2 * 3.14159265 / C_1REV[w]
            wheel_rates.append(om)
            log(f"    {WHEEL_NAMES[w]}: Δenc={denc:6d} / {dt_s:.2f}s -> {cps:7.0f} cnt/s = {om:5.2f} rad/s "
                f"(wheel_map 表值 {TABLE_OMEGA_AT_130[w]:.1f})")
            time.sleep(0.5)   # 停稳再测下一轮
        valid = [r for r in wheel_rates if r]
        if len(valid) >= 2:
            spread = (max(valid) - min(valid)) / (sum(valid) / len(valid)) * 100
            log(f"    四轮离散度 (max-min)/mean = {spread:.1f}%  "
                f"(>10% 时开环直行必然跑偏)")

        # ---- 4. RUN 直行（轮子悬空，车身不动） ----
        log("[4] RUN 0.15 0 0 1500 (直行指令，架空)")
        car.send("RUN 0.15 0 0 1500")
        data, replies, _ = car.collect(6.0, stop_words=("DONE",))
        log(f"    回复: {replies}")
        runs = [parse_line(ln) for ln in data if ln.startswith("D ")]
        if runs:
            e0, e1 = runs[0][3], runs[-1][3]
            dt_s = (runs[-1][1] - runs[0][1]) / 1000.0
            for w in range(4):
                cps = (e1[w] - e0[w]) / dt_s
                om = cps * 2 * 3.14159265 / C_1REV[w]
                log(f"    {WHEEL_NAMES[w]} pwm={runs[len(runs)//2][2+w]:4d} -> {om:5.2f} rad/s")
            ys = [s[4][2] for s in runs]
            rs = [s[4][0] for s in runs]
            log(f"    yaw {ys[0]:+.2f} -> {ys[-1]:+.2f} deg (Δ={ys[-1]-ys[0]:+.2f}; 车身不动,理想≈0)")
            log(f"    roll 抖动范围 {min(rs):+.2f}..{max(rs):+.2f} deg (电机振动是否污染 IMU)")

        # ---- 5. RUN 原地旋转（车身不转，看 IMU 是否被振动骗） ----
        log("[5] RUN 0 0 1.0 1200 (旋转指令 ω=1rad/s, 架空车身不动)")
        car.send("RUN 0 0 1.0 1200")
        data, replies, _ = car.collect(6.0, stop_words=("DONE",))
        log(f"    回复: {replies}")
        runs = [parse_line(ln) for ln in data if ln.startswith("D ")]
        if runs:
            ys = [s[4][2] for s in runs]
            log(f"    yaw {ys[0]:+.2f} -> {ys[-1]:+.2f} deg (Δ={ys[-1]-ys[0]:+.2f}; "
                f"车身不转,理想≈0; |Δ|>3° = 振动致 IMU 假旋转)")

        car.cmd("STOP")
        car.cmd("STREAM 0")

    finally:
        if mon.events:
            log(f"[!] 串口异常事件 {len(mon.events)} 条:")
            for t, line in mon.events:
                log(f"    {line}")
        else:
            log("[6] 串口全程无崩溃/复位事件")
        if car:
            try:
                car.cmd("STOP")
            except Exception:
                pass
            car.close()
        mon.close()
        with open(raw_log.replace(".txt", "_report.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(report) + "\n")
        print(f"\n原始日志: {raw_log}\n报告:     {raw_log.replace('.txt', '_report.txt')}")


if __name__ == "__main__":
    main()
