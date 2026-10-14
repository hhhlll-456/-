#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R62 复现 / 验收脚本：反复「新建 TCP 连接 → PING → RUN → 关闭」压测。

背景（R62，证据见 .superpowers/sdd/2026-10-13-vision-ground-truth-experiments/progress.md）：
    固件只在「当前 client 自认断开」时才 accept 新连接，而 WiFiClient::connected()
    只探测 RX（recv(fd,&dummy,0)）、完全不看 TX。一旦这条 socket 的发送路径卡死
    （send() 持续失败）而 connected() 仍为真，固件就会：
      - 把遥测/控制行继续往环形缓冲里堆、发不出去 → D 行中途彻底静默、DONE 丢失；
      - 永不 accept 新连接 → 整块板子的 TCP 控制通道废掉（断电重启前不自恢复）。
    历史观察：RUN 的 D 行「先正常吐一段、然后彻底静默」，且跨新连接不复原。

本脚本每次迭代（模拟「短会话 + 重连」的真实用法）：
    1) 新建连接，发 PING —— 必须收到 PONG（超时 = 失败）；
    2) 发一次自带超时的 RUN（**只原地旋转**：vx=vy=0、om<0），
       等待 D 行 + DONE。**DONE 是「跑完」的唯一判据**（R56/R62：环形缓冲
       停滞会推迟交付，超时必须给足 RUN 时长 + 15 s）；
    3) 关闭连接，进入下一轮。每隔 --long-every 轮插一次较长的 RUN。

失败时会额外做两个诊断探针（对照报告里的 R62 签名）：
    - 同连接再 PING（旧 socket 仍开着）：应超时 —— "TCP 还连着、板子不回"；
    - 另开一条新连接 PING：若回 PONG，说明固件还活着，只是旧连接废了。

判据：
    修复前（R62 未修）——几十次重连后必须出现失败（否则说明复现方式没打中）；
    修复后 —— 全程 0 失败。

安全：绝不发裸 M（无超时、会把车跑飞，R60）；RUN 全程 vx=vy=0 原地旋转。
      运行前确认车四轮着地、在场地内。

用法：
    python3 实验/tcp_reconnect_stress.py                     # 36 轮，每 6 轮一次长 RUN
    python3 实验/tcp_reconnect_stress.py --iters 60 --long-every 10
"""
from __future__ import annotations

import argparse
import socket
import sys
import time

HOST = "192.168.1.115"
PORT = 8266

PING_TIMEOUT_S = 4.0        # PING → PONG 的等待上限
DONE_GRACE_S = 15.0         # DONE 的墙钟宽限（RUN 时长之外再加这么多秒）

SHORT_RUN_MS = 300          # 短抽查：RUN 0 0 -1 300
LONG_RUN_MS = 1500          # 长抽查：RUN 0 0 -2 1500（D 行 + DONE 完整性）

LATE_WINDOW_S = 6.0         # 失败后再多等这么久，看 DONE/残余 D 行会不会迟到


def recv_until(sock: socket.socket, needle: bytes, deadline: float, buf: bytes):
    """把数据读进 buf，直到出现 needle 或超过 deadline。返回 (buf, 是否命中)。"""
    while needle not in buf:
        remain = deadline - time.time()
        if remain <= 0:
            break
        sock.settimeout(min(0.5, remain))
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
    return buf, needle in buf


def same_conn_ping(sock: socket.socket, buf: bytes):
    """在（已静默的）旧连接上再发一次 PING，返回 (是否回 PONG, 新增文本)。"""
    try:
        sock.sendall(b"PING\n")
    except OSError:
        return False, ""
    before = len(buf)
    buf, hit = recv_until(sock, b"PONG", time.time() + PING_TIMEOUT_S, buf)
    return hit, buf[before:].decode("utf-8", "replace")


def fresh_conn_ping(host: str, port: int):
    """另开一条全新连接 PING（旧连接保持不动），返回 (是否回 PONG, 说明)。"""
    try:
        s = socket.create_connection((host, port), timeout=6.0)
    except OSError as exc:
        return False, f"connect 失败：{exc!r}"
    try:
        s.settimeout(0.5)
        s.sendall(b"PING\n")
        buf, hit = recv_until(s, b"PONG", time.time() + PING_TIMEOUT_S, b"")
        tail = buf.decode("utf-8", "replace").strip().replace("\n", " / ")[-120:]
        return hit, tail
    finally:
        s.close()


def one_round(host: str, port: int, om: float, run_ms: int):
    """一轮 = 一条新连接：PING → RUN → 关闭。返回 (ok, info dict)。"""
    info = {
        "ping": False,          # 新连接上收到 PONG
        "ok_run": False,        # 收到 OK RUN 回执
        "d_lines": 0,           # D 行条数
        "done": False,          # 收到 DONE（唯一判据）
        "error": "",
        "tail": "",             # 失败时的非 D 行尾巴（诊断用）
        "same_conn_pong": None,     # 失败后同连接 PING 是否回 PONG
        "fresh_conn_pong": None,    # 失败后新连接 PING 是否回 PONG
        "late_done": False,         # 失败后额外窗口内是否收到迟到的 DONE
        "late_d_lines": 0,          # 额外窗口内新增的 D 行
    }
    sock = None
    buf = b""
    try:
        sock = socket.create_connection((host, port), timeout=6.0)
        sock.settimeout(0.5)

        # ---- 1) 新连接上 PING 必须回 PONG ----
        sock.sendall(b"PING\n")
        buf, hit = recv_until(sock, b"PONG", time.time() + PING_TIMEOUT_S, buf)
        info["ping"] = hit
        if not hit:
            info["error"] = f"PING 在 {PING_TIMEOUT_S:.0f}s 内没有 PONG（新连接无响应）"
            info["tail"] = buf.decode("utf-8", "replace").strip().replace("\n", " / ")[-200:]
            return False, info

        # ---- 2) 原地旋转的 RUN，等 D 行 + DONE ----
        sock.sendall(f"RUN 0 0 {om} {run_ms}\n".encode())
        deadline = time.time() + run_ms / 1000.0 + DONE_GRACE_S
        buf, hit = recv_until(sock, b"DONE", deadline, buf)
        text = buf.decode("utf-8", "replace")
        lines = text.splitlines()
        info["done"] = hit
        info["d_lines"] = sum(1 for ln in lines if ln.startswith("D "))
        info["ok_run"] = any(ln.startswith("OK RUN") for ln in lines)
        if not hit:
            info["error"] = (f"RUN {om} {run_ms} 在 {run_ms / 1000.0 + DONE_GRACE_S:.0f}s 内"
                             f"没有 DONE（只收到 {info['d_lines']} 条 D 行"
                             f"{'，也没有 OK RUN' if not info['ok_run'] else ''}）")
            info["tail"] = " | ".join(ln for ln in lines if not ln.startswith("D "))[-300:]
            # ---- 失败诊断：R62 签名 ----
            # (a) 旧连接还开着，同连接再 PING
            pong, extra = same_conn_ping(sock, buf)
            info["same_conn_pong"] = pong
            buf += extra.encode()
            # (b) 旧连接不动，另开新连接 PING（测 accept 路径是否被僵尸占住）
            info["fresh_conn_pong"], fresh_tail = fresh_conn_ping(host, port)
            if fresh_tail and not info["tail"]:
                info["tail"] = f"[新连接] {fresh_tail}"
            # (c) 额外窗口：DONE / D 行会不会迟到
            n_before = info["d_lines"]
            buf, late_done = recv_until(sock, b"DONE", time.time() + LATE_WINDOW_S, buf)
            info["late_done"] = late_done and not hit
            info["late_d_lines"] = sum(1 for ln in buf.decode("utf-8", "replace").splitlines()
                                       if ln.startswith("D ")) - n_before
        elif not info["ok_run"]:
            info["error"] = "收到了 DONE 但没有 OK RUN 回执（回复错位？）"
        return (hit and info["ok_run"]), info
    except OSError as exc:
        info["error"] = f"socket 错误：{exc!r}"
        return False, info
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def main() -> int:
    ap = argparse.ArgumentParser(description="R62 重连压测（只原地旋转，安全）")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--iters", type=int, default=36, help="重连轮数（判据要求 ≥30）")
    ap.add_argument("--long-every", type=int, default=6,
                    help="每隔几轮插一次长 RUN（0 = 全部用短 RUN）")
    ap.add_argument("--pause", type=float, default=0.15, help="轮间暂停秒数")
    ap.add_argument("--fail-fast", action="store_true", help="首个失败即退出")
    args = ap.parse_args()

    print(f"R62 重连压测：{args.iters} 轮 · 目标 {args.host}:{args.port} · "
          f"长 RUN 每 {args.long_every} 轮一次")
    print(f"每轮判据：新连接 PING→PONG；RUN 收到 D 行 + DONE（宽限 {DONE_GRACE_S:.0f}s）")
    print("-" * 72)

    failures = []           # (轮号, info)
    consecutive = 0
    worst_streak = 0
    for i in range(1, args.iters + 1):
        long_round = args.long_every > 0 and i % args.long_every == 0
        if long_round:
            om, run_ms = -2.0, LONG_RUN_MS
        else:
            om, run_ms = -1.0, SHORT_RUN_MS
        ok, info = one_round(args.host, args.port, om, run_ms)
        tag = "长" if long_round else "短"
        if ok:
            consecutive = 0
            print(f"[{i:3d}/{args.iters}] ✅ {tag} RUN {om} {run_ms}  "
                  f"PING✓ D行{info['d_lines']:3d} DONE✓")
        else:
            consecutive += 1
            worst_streak = max(worst_streak, consecutive)
            failures.append((i, info))
            print(f"[{i:3d}/{args.iters}] ❌ {tag} RUN {om} {run_ms}  PING"
                  f"{'✓' if info['ping'] else '✗'} D行{info['d_lines']:3d} "
                  f"DONE{'✓' if info['done'] else '✗'}")
            print(f"        原因：{info['error']}")
            if info["same_conn_pong"] is not None:
                print(f"        诊断：同连接再 PING = "
                      f"{'PONG✓' if info['same_conn_pong'] else '超时✗'}；"
                      f"新连接 PING = "
                      f"{'PONG✓' if info['fresh_conn_pong'] else '超时✗'}"
                      f"（旧连接保持不动时测）")
                if info["late_done"] or info["late_d_lines"]:
                    print(f"        迟到：额外 {LATE_WINDOW_S:.0f}s 内又收到 "
                          f"DONE={info['late_done']}，D 行 +{info['late_d_lines']}")
            if info["tail"]:
                print(f"        非 D 行尾巴：{info['tail']}")
            if args.fail_fast:
                break
        time.sleep(args.pause)

    print("-" * 72)
    if not failures:
        print(f"✅ 全部 {args.iters} 轮通过：没有出现 R62（PING 全回、RUN 全部收到 DONE）")
        return 0
    print(f"❌ {args.iters} 轮中失败 {len(failures)} 轮；首次失败在第 {failures[0][0]} 轮；"
          f"最长连续失败 {worst_streak} 轮")
    same_dead = sum(1 for _, f in failures if f["same_conn_pong"] is False)
    accept_blocked = sum(1 for _, f in failures if f["fresh_conn_pong"] is False)
    print(f"   失败签名计数：同连接 PING 超时 {same_dead}/{len(failures)}；"
          f"新连接 PING 超时 {accept_blocked}/{len(failures)}")
    if worst_streak >= 3:
        print("   ⚠ 连续失败 ≥3：板子已进入「TCP 僵尸」态，后续新连接也大概率无响应"
              "（修复前只能靠断电/复位恢复）—— 这正是 R62。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
