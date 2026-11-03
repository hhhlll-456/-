#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""架空诊断 v2（修复：全程 STREAM 在线 + 断线重连 + 温和占空比）。

重点：
  A. 单轮空载转速一致性（直行跑偏的执行器证据）
  B. RUN 直行/旋转时 IMU yaw 是否被电机振动/磁干扰污染（车身架空转不了，
     yaw 应变 0 —— 不为 0 就是假读数）
  C. brownout 是否复现
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
TEST_PWM = 120
WHEEL_FWD_PWM = [-TEST_PWM, TEST_PWM, -TEST_PWM, TEST_PWM]
C_1REV = [1276, 1270, 1276, 1274]
WHEEL_NAMES = ["FL", "FR", "RL", "RR"]


class SerialMonitor(threading.Thread):
    def __init__(self, port, out_path):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.out = open(out_path, "a", encoding="utf-8")
        self.stop_flag = False
        self.events = []

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


class RawLink:
    def __init__(self, ip, port=TCP_PORT):
        self.ip, self.port = ip, port
        self._connect()

    def _connect(self):
        self.s = socket.create_connection((self.ip, self.port), timeout=5)
        self.s.settimeout(0.2)
        self.buf = b""

    def reconnect(self):
        try:
            self.s.close()
        except Exception:
            pass
        for _ in range(20):          # 等 Wi-Fi 重连（brownout 复位后约 2~5s）
            try:
                self._connect()
                return True
            except OSError:
                time.sleep(0.5)
        return False

    def send(self, line):
        self.s.sendall((line + "\n").encode())

    def _read_once(self):
        try:
            d = self.s.recv(8192)
            if not d:
                raise ConnectionError("closed")
            self.buf += d
        except socket.timeout:
            pass
        lines = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            lines.append(raw.decode("utf-8", "replace").strip())
        return lines

    def collect(self, dur, stop_words=("DONE",), quiet=False):
        t0 = time.time()
        data, replies = [], []
        while time.time() - t0 < dur:
            try:
                for ln in self._read_once():
                    if ln.startswith(("I ", "D ")):
                        data.append(ln)
                    else:
                        replies.append(ln)
                        if ln in stop_words:
                            return data, replies
            except (ConnectionError, ConnectionResetError, OSError):
                if not quiet:
                    raise
                return data, replies
        return data, replies

    def cmd(self, line, timeout=4.0):
        self.send(line)
        t0 = time.time()
        while time.time() - t0 < timeout:
            for ln in self._read_once():
                if ln.startswith(("I ", "D ")):
                    continue
                return ln
        return None

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def parse_line(ln):
    p = ln.split()
    return p[0], int(p[1]), list(map(int, p[2:6])), list(map(int, p[6:10])), list(map(float, p[10:13]))


def yaw_drift_rate(samples):
    pts = [(t, rpy[2]) for _, t, _, _, rpy in samples]
    n = len(pts)
    if n < 10:
        return None
    mt = sum(t for t, _ in pts) / n
    my = sum(y for _, y in pts) / n
    st = sum((t - mt) ** 2 for t, _ in pts)
    if st == 0:
        return None
    k = sum((t - mt) * (y - my) for t, y in pts) / st
    resid = [y - (my + k * (t - mt)) for t, y in pts]
    rms = (sum(r * r for r in resid) / n) ** 0.5
    return k * 1000 * 60, rms


def imu_update_rate(samples):
    changes = 0
    prev = None
    for _, _, _, _, rpy in samples:
        if prev is not None and rpy != prev:
            changes += 1
        prev = rpy
    span = max(samples[-1][1] - samples[0][1], 1)
    return changes / (span / 1000.0)


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    raw_log = os.path.join(LOG_DIR, f"diag_air2_{stamp}.txt")
    lines_out = []

    def log(msg):
        print(msg)
        lines_out.append(msg)

    mon = SerialMonitor(SERIAL_PORT, raw_log)
    mon.start()
    log(f"[0] 串口监听中（{SERIAL_PORT}）")
    time.sleep(1.5)

    car = None
    try:
        ip = discover() or ("192.168.1.115", TCP_PORT)
        ip = ip[0] if isinstance(ip, tuple) else ip
        car = RawLink(ip)
        log(f"[1] TCP {ip}:{TCP_PORT}")
        for c in ("PING", "ID"):
            log(f"    {c} -> {car.cmd(c)}")
        log(f"    SNIFF -> {car.cmd('SNIFF', timeout=6)}")
        car.cmd("STREAM 1")
        log("    STREAM 1 已开（全程在线）")

        # ---- A. 静置基线 ----
        data, _ = car.collect(10.0, stop_words=(), quiet=True)
        samples = [parse_line(ln) for ln in data]
        log(f"[A] 静置 {len(samples)} 样本")
        if samples:
            roll = [s[4][0] for s in samples]
            pitch = [s[4][1] for s in samples]
            yaw = [s[4][2] for s in samples]
            log(f"    roll {min(roll):+.2f}..{max(roll):+.2f}  pitch {min(pitch):+.2f}..{max(pitch):+.2f}  yaw {min(yaw):+.2f}..{max(yaw):+.2f}")
            d = yaw_drift_rate(samples)
            if d:
                log(f"    yaw 漂移 {d[0]:+.3f} deg/min (RMS {d[1]:.3f})")
            log(f"    IMU 更新率 ~{imu_update_rate(samples):.0f} Hz")
            delta = [b - a for a, b in zip(samples[0][3], samples[-1][3])]
            log(f"    静置编码器增量 {delta}")

        # ---- B. 逐轮空载转速 ----
        log(f"[B] 逐轮 PWM={WHEEL_FWD_PWM}")
        rates = []
        for w in range(4):
            car.cmd("ZENC")
            m = [0, 0, 0, 0]
            m[w] = WHEEL_FWD_PWM[w]
            car.cmd("M %d %d %d %d" % tuple(m))
            data, _ = car.collect(1.5, stop_words=(), quiet=True)
            car.cmd("STOP")
            samples = [parse_line(ln) for ln in data]
            if len(samples) < 5:
                log(f"    {WHEEL_NAMES[w]}: 样本不足 ({len(samples)})")
                rates.append(None)
            else:
                denc = samples[-1][3][w] - samples[0][3][w]
                dt = (samples[-1][1] - samples[0][1]) / 1000.0
                om = (denc / dt) * 2 * 3.14159265 / C_1REV[w] if dt > 0 else 0
                rates.append(om)
                log(f"    {WHEEL_NAMES[w]}: Δenc={denc:5d}/{dt:.2f}s -> {om:5.2f} rad/s")
            time.sleep(0.4)
        valid = [r for r in rates if r]
        if len(valid) >= 2:
            mean = sum(valid) / len(valid)
            spread = (max(valid) - min(valid)) / mean * 100
            log(f"    离散度 {spread:.1f}%  (mean {mean:.2f})")

        # ---- C. RUN 直行 ----
        log("[C] RUN 0.10 0 0 1200")
        car.send("RUN 0.10 0 0 1200")
        data, replies = car.collect(8.0)
        log(f"    回复 {replies[:2]}")
        runs = [parse_line(ln) for ln in data if ln.startswith("D ")]
        if runs:
            mid = runs[len(runs) // 2]
            log(f"    中段 pwm={mid[2]}")
            e0, e1 = runs[0][3], runs[-1][3]
            dt = (runs[-1][1] - runs[0][1]) / 1000.0
            for w in range(4):
                om = ((e1[w] - e0[w]) / dt) * 2 * 3.14159265 / C_1REV[w] if dt > 0 else 0
                log(f"    {WHEEL_NAMES[w]}: {om:5.2f} rad/s")
            ys = [s[4][2] for s in runs]
            log(f"    yaw {ys[0]:+.2f}->{ys[-1]:+.2f} (Δ={ys[-1]-ys[0]:+.2f} deg, 架空应≈0)")

        # ---- D. RUN 旋转（车身架空转不了，yaw 应≈0） ----
        log("[D] RUN 0 0 0.5 1200")
        car.send("RUN 0 0 0.5 1200")
        data, replies = car.collect(8.0)
        log(f"    回复 {replies[:2]}")
        runs = [parse_line(ln) for ln in data if ln.startswith("D ")]
        if runs:
            mid = runs[len(runs) // 2]
            log(f"    中段 pwm={mid[2]}")
            e0, e1 = runs[0][3], runs[-1][3]
            dt = (runs[-1][1] - runs[0][1]) / 1000.0
            for w in range(4):
                om = ((e1[w] - e0[w]) / dt) * 2 * 3.14159265 / C_1REV[w] if dt > 0 else 0
                log(f"    {WHEEL_NAMES[w]}: {om:5.2f} rad/s")
            ys = [s[4][2] for s in runs]
            log(f"    yaw {ys[0]:+.2f}->{ys[-1]:+.2f} (Δ={ys[-1]-ys[0]:+.2f} deg, 电机全转时应≈0)")

    except (ConnectionResetError, ConnectionError, OSError) as e:
        log(f"[!] TCP 断连: {e} —— 尝试重连")
        if car and car.reconnect():
            log("    重连成功")
            try:
                car.cmd("STOP"); car.cmd("STREAM 0")
            except Exception:
                pass
        else:
            log("    重连失败（板子可能还没起来）")
    finally:
        if mon.events:
            log(f"[!] 串口异常 {len(mon.events)} 条:")
            for _, line in mon.events:
                log(f"    {line}")
        else:
            log("[E] 串口无崩溃/复位事件")
        if car:
            try:
                car.cmd("STOP"); car.cmd("STREAM 0")
            except Exception:
                pass
            car.close()
        mon.close()
        rep = raw_log.replace(".txt", "_report.txt")
        with open(rep, "w", encoding="utf-8") as f:
            f.write("\n".join(lines_out) + "\n")
        print(f"\n报告: {rep}")


if __name__ == "__main__":
    main()
