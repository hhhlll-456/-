#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机脚本统一的连板子方式：显式 IP 优先 + 重试，绕开 UDP 发现的不稳定。

现场观察（2026-10-15 Task 10 验收）：
- 固件是**单客户端** TCP（`main.cpp` 的 `server.available()`），上一个会话刚
  断开时新连接可能连上但收不到任何回复（固件对陈旧 client 的接受有滞后，R56）。
  ⇒ 建连后必须做 `PING`/`PONG` 握手，**握手失败要等一下重试**，不能直接跑。
- UDP 广播发现（`car_link.discover`）在现场**间歇性失败**（同一分钟内一次失败、
  一次成功）⇒ 一律显式传 IP，发现只当兜底。

握手失败时的动作是「等 + 重连 + 立刻 STOP」，不是「继续跑」。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPERIMENT_DIR = HERE.parent
ROOT = EXPERIMENT_DIR.parent
for _p in (str(EXPERIMENT_DIR), str(ROOT / "视觉测量" / "脚本")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from car_link import CarLink                                # noqa: E402

BOARD_IP = "192.168.1.115"
BOARD_PORT = 8266


def open_link(*, ip=BOARD_IP, port=BOARD_PORT, tries=5, pause_s=3.0,
              handshake_s=8.0):
    """连板子并做 `PING`/`PONG` 握手；握手过了才交回去。

    返回一个 `CarLink`。**每一次失败重试之间都会先等**，让固件释放上一个
    client —— 立刻重连会连上但没人应。
    """
    last = None
    for k in range(1, tries + 1):
        link = None
        try:
            link = CarLink(ip=ip, port=port)
            reply, _ = _ping(link, handshake_s)
            if reply is not None:
                return link
            last = "PING 未收到 PONG（固件还没释放上一个 client？）"
        except Exception as exc:                          # 连不上 / 超时
            last = f"{type(exc).__name__}: {exc}"
        finally:
            if link is not None and not getattr(link, "_keep", False):
                pass
        if link is not None:
            try:
                link.close()
            except Exception:
                pass
        print(f"  [link] 第 {k}/{tries} 次连板失败：{last}；{pause_s:g}s 后重试",
              file=sys.stderr, flush=True)
        time.sleep(pause_s)
    raise RuntimeError(f"连不上板子（试了 {tries} 次）：{last}")


def _ping(link, timeout_s):
    link.cmd("PING")
    buf, deadline = b"", time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        link.sock.settimeout(0.2)
        try:
            chunk = link.sock.recv(4096)
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
        if b"PONG" in buf:
            return "PONG", buf
    return None, buf
