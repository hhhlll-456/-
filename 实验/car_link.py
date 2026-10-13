#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""麦轮小车 Wi-Fi 链路工具（PC 端）。

用法：
    from car_link import CarLink
    car = CarLink()            # 自动 UDP 发现
    car.cmd("PING")            # 发一条命令，返回回复
    car.motor(60, 60, 60, 60)  # 四轮 PWM
    car.stop()
    car.close()

命令行自测：
    python3 car_link.py        # 发现 + PING/ID 冒烟测试（不驱动电机）
"""
import socket
import time

TCP_PORT = 8266
UDP_PORT = 8267
DISCOVER_TIMEOUT = 3.0


def discover(timeout=DISCOVER_TIMEOUT):
    """UDP 广播找小车，返回 (ip, port) 或 None。"""
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    u.settimeout(timeout)
    try:
        u.sendto(b"WHO?", ("255.255.255.255", UDP_PORT))
        data, addr = u.recvfrom(128)
        parts = data.decode("utf-8", "replace").strip().split()
        if parts and parts[0] == "MECANUM":
            ip = parts[1] if len(parts) > 1 else addr[0]
            port = int(parts[2]) if len(parts) > 2 else TCP_PORT
            return ip, port
    except socket.timeout:
        return None
    finally:
        u.close()
    return None


class CarLink:
    def __init__(self, ip=None, port=TCP_PORT, timeout=4.0, auto_discover=True):
        if ip is None and auto_discover:
            found = discover()
            if not found:
                raise RuntimeError("未发现小车：确认小车已上电、与电脑同一 Wi-Fi(B1222)")
            ip, port = found
        self.addr = (ip or "192.168.1.115", port)
        self.sock = socket.create_connection(self.addr, timeout=timeout)
        self.sock.settimeout(timeout)
        self.buf = b""

    def cmd(self, line, wait=False, timeout=2.0):
        """发一条文本命令。wait=True 时读取一行回复（按行缓冲，抗 TCP 粘包）。"""
        self.sock.sendall((line + "\n").encode())
        if not wait:
            return None
        self.sock.settimeout(timeout)
        while b"\n" not in self.buf:
            try:
                chunk = self.sock.recv(512)
            except socket.timeout:
                break
            if not chunk:
                break
            self.buf += chunk
        if b"\n" in self.buf:
            line_bytes, self.buf = self.buf.split(b"\n", 1)
            return line_bytes.decode("utf-8", "replace").strip("\r").strip()
        out, self.buf = self.buf, b""
        return out.decode("utf-8", "replace").strip() or None

    def motor(self, fl, fr, rl, rr):
        return self.cmd(f"M {int(fl)} {int(fr)} {int(rl)} {int(rr)}", wait=True)

    def stop(self):
        return self.cmd("STOP", wait=True)

    def ping(self):
        return self.cmd("PING", wait=True)

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    print("UDP 发现中 ...")
    found = discover()
    print("  发现:", found)
    with CarLink() as car:
        print("  地址:", car.addr)
        print("  PING ->", car.ping())
        print("  ID   ->", car.cmd("ID", wait=True))
        print("  STOP ->", car.stop())
    print("链路正常。")
