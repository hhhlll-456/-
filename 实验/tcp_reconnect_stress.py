#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R62 复现 / 验收脚本：反复「新建 TCP 连接 → PING → RUN → 关闭」压测。

背景（R62，证据见 .superpowers/sdd/2026-10-13-vision-ground-truth-experiments/progress.md）：
    固件只在「当前 client 自认断开」时才 accept 新连接，而 WiFiClient::connected()
    只探测 RX（recv(fd,&dummy,0)）、完全不看 TX。一旦这条 socket 的发送路径卡死
    （send() 持续失败）而 connected() 仍为真，固件就会：
      - 把遥测/控制行继续往环形缓冲里堆、发不出去 → D 行中途彻底静默、DONE 丢失；
      - 永不 accept 新连接 → 整块板子的 TCP 控制通道废掉（断电重启前不自恢复）。
    修复（9045e70）加了 TX 停摆看门狗：环里有积压且 >2s 一个字节都没发出去 →
    client.stop() + 清环 + 停车。修复后的正确表现：停摆 = 板子主动把连接踢断
    （PC 侧收到 EOF/RST），而不是「TCP 还连着但装死」；新连接立刻可用。

本脚本每次迭代（模拟「短会话 + 重连」的真实用法）：
    1) 新建连接，发 PING —— 必须收到 PONG（超时 = 失败）；
    2) 掉电/复位检测（两道，缺一不可 —— 评审 I3）：
       a) 非正常复位（brownout/panic/看门狗…）后，PING 回复之后会多一行
          'RESET <原因> count=<n>'；发现 count 比基线/此前更大 ⇒ 本轮**无效**；
       b) 而整板掉电（POWERON）**不带** RESET 行 —— 用 millis 探针补盲：
          STREAM 1 取一行空闲样本 'I <millis> …'，跨轮比较，millis 回退即重启过。
       无效轮 = 既不是通过、也不是失败（复位会清僵尸态，让失败假绿）。
    3) 发一次自带超时的 RUN（**只原地旋转**：vx=vy=0、om<0），等待 D 行 + DONE。
       **DONE 是「跑完」的唯一判据**（R56/R62：环形缓冲停滞会推迟交付，超时必须给足
       RUN 时长 + 15 s）；
    4) 关闭连接，进入下一轮。每隔 --long-every 轮插一次较长的 RUN。
    5) 记录每轮数据到达间隔；若板子在本轮**主动断连**（EOF/RST，而不仅是超时）
       —— 这是看门狗/复位的签名（评审 I1 要求统计看门狗触发次数）。

失败时的诊断探针（对照报告里的 R62 签名）：
    - 同连接再 PING（旧 socket 仍开着）：应超时 —— "TCP 还连着、板子不回"；
    - 另开一条新连接 PING：若回 PONG，说明固件还活着，只是旧连接废了。

判据：
    修复前（R62 未修）——几十次重连后必须出现失败（否则说明复现方式没打中）；
    修复后 —— 全程 0 真失败；无效轮次（复位/掉电）单独报告并补跑。
    退出码：0 = 全部有效且通过；1 = 有真失败；2 = 无失败但有无效轮次。

安全：绝不发裸 M（无超时、会把车跑飞，R60）；RUN 全程 vx=vy=0 原地旋转。
      每轮结尾补发 STOP（命令序列收尾）。运行前确认车四轮着地/悬空、场地安全。

用法：
    python3 实验/tcp_reconnect_stress.py                     # 36 轮，每 6 轮一次长 RUN
    python3 实验/tcp_reconnect_stress.py --iters 60 --long-every 10
    python3 实验/tcp_reconnect_stress.py --json-out /tmp/r62_rounds.json
"""
from __future__ import annotations

import argparse
import json
import math
import re
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
RESET_DRAIN_S = 0.3         # PONG 之后再读一小段，等 RESET 行（PONG 与 RESET 是两次 send）
UPTIME_TIMEOUT_S = 3.0      # STREAM 1 之后等第一行 I 样本的上限

RESET_RE = re.compile(r"RESET\s+(\S+)\s+count=(\d+)")
I_MS_RE = re.compile(r"^I\s+(\d+)\b")


# ---------------- 基础 IO ----------------

def recv_until(sock: socket.socket, needle: bytes, deadline: float, buf: bytes,
               arrivals: list | None = None):
    """把数据读进 buf，直到出现 needle / EOF / 超时。返回 (buf, hit, reason)。

    reason ∈ {'hit','eof','timeout','error:<repr>'}。arrivals（可选）记录每个
    非空 recv 的时刻，用于到达间隔统计。
    """
    reason = "timeout"
    while needle not in buf:
        remain = deadline - time.time()
        if remain <= 0:
            break
        sock.settimeout(min(0.5, remain))
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        except OSError as exc:
            reason = f"error:{exc!r}"
            break
        if not chunk:
            reason = "eof"
            break
        buf += chunk
        if arrivals is not None:
            arrivals.append(time.time())
    if needle in buf:
        reason = "hit"
    return buf, reason == "hit", reason


def read_for(sock: socket.socket, seconds: float, buf: bytes,
             arrivals: list | None = None):
    """尽力读 seconds 秒（读到 EOF/错误即提前停）。返回 (buf, reason)。"""
    deadline = time.time() + seconds
    reason = "timeout"
    while True:
        remain = deadline - time.time()
        if remain <= 0:
            break
        sock.settimeout(min(0.3, remain))
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        except OSError as exc:
            reason = f"error:{exc!r}"
            break
        if not chunk:
            reason = "eof"
            break
        buf += chunk
        if arrivals is not None:
            arrivals.append(time.time())
    return buf, reason


def gap_stats(arrivals: list):
    """相邻到达时刻的间隔统计：n / max / p95 / >1s 次数 / >2s 次数。"""
    if len(arrivals) < 2:
        return {"n": len(arrivals), "max": None, "p95": None, "gt1": 0, "gt2": 0}
    diffs = sorted(b - a for a, b in zip(arrivals, arrivals[1:]))
    k = max(0, min(len(diffs) - 1, math.ceil(0.95 * len(diffs)) - 1))
    return {"n": len(arrivals), "max": diffs[-1], "p95": diffs[k],
            "gt1": sum(1 for d in diffs if d > 1.0),
            "gt2": sum(1 for d in diffs if d > 2.0)}


def parse_resets(text: str):
    """提取所有 'RESET <原因> count=<n>'。返回 [(原因, n)]（按出现顺序）。"""
    return [(reason, int(cnt)) for reason, cnt in RESET_RE.findall(text)]


def find_idle_ms(buf: bytes):
    """在累计缓冲里找第一行完整的 'I <millis> …'，返回 millis 或 None（半行不算）。"""
    idx = 0
    while True:
        j = buf.find(b"\nI ", idx)
        if j < 0:
            return None
        k = buf.find(b"\n", j + 1)
        if k < 0:
            return None
        m = I_MS_RE.match(buf[j + 1:k].decode("utf-8", "replace").strip())
        if m:
            return int(m.group(1))
        idx = k + 1


# ---------------- 探针 ----------------

def same_conn_ping(sock: socket.socket, buf: bytes, arrivals: list):
    """在（已静默的/已断开的）旧连接上再发一次 PING。返回 (是否回 PONG, 新增文本, 说明)。

    注意：只能看**这次 send 之后**收到的字节。buf 里已有的 PONG（本轮开头的那个）
    不算数 —— 否则诊断永远是 "PONG✓"，僵尸永远测不出来（2026-10-15 用伪板子抓到的坑）。
    """
    try:
        sock.sendall(b"PING\n")
        arrivals.append(time.time())
    except OSError as exc:
        return False, "", f"send 失败：{exc!r}"
    new = b""
    reason = "timeout"
    deadline = time.time() + PING_TIMEOUT_S
    while b"PONG" not in new:
        remain = deadline - time.time()
        if remain <= 0:
            break
        sock.settimeout(min(0.5, remain))
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        except OSError as exc:
            reason = f"error:{exc!r}"
            break
        if not chunk:
            reason = "eof"
            break
        new += chunk
        arrivals.append(time.time())
    hit = b"PONG" in new
    err = "" if hit else f"未回 PONG（{reason}）"
    return hit, new.decode("utf-8", "replace"), err


def fresh_conn_ping(host: str, port: int):
    """另开一条全新连接 PING（旧连接保持不动）。返回 (是否回 PONG, 文本尾巴, RESET 列表)。"""
    try:
        s = socket.create_connection((host, port), timeout=6.0)
    except OSError as exc:
        return False, f"connect 失败：{exc!r}", []
    try:
        s.settimeout(0.5)
        s.sendall(b"PING\n")
        buf, hit, reason = recv_until(s, b"PONG", time.time() + PING_TIMEOUT_S, b"")
        if hit:
            buf, _ = read_for(s, RESET_DRAIN_S, buf)
        text = buf.decode("utf-8", "replace")
        tail = text.strip().replace("\n", " / ")[-160:]
        if not hit:
            tail = f"（{reason}）{tail}".strip()
        return hit, tail, parse_resets(text)
    finally:
        s.close()


def baseline_probe(host: str, port: int, uptime_probe: bool = True):
    """会话开始前的干净 PING：给 RESET 状态与板子 millis 建立基线。"""
    out = {"ping": False, "resets": [], "board_ms": None, "board_ms_wall": None, "tail": ""}
    try:
        s = socket.create_connection((host, port), timeout=6.0)
    except OSError as exc:
        out["tail"] = f"connect 失败：{exc!r}"
        return out
    try:
        s.settimeout(0.5)
        s.sendall(b"PING\n")
        buf, hit, reason = recv_until(s, b"PONG", time.time() + PING_TIMEOUT_S, b"")
        out["ping"] = hit
        if not hit:
            out["tail"] = f"{reason}: " + buf.decode("utf-8", "replace").strip()[:120]
            return out
        buf, _ = read_for(s, RESET_DRAIN_S, buf)
        if uptime_probe:
            s.sendall(b"STREAM 1\n")
            deadline = time.time() + UPTIME_TIMEOUT_S
            while out["board_ms"] is None and time.time() < deadline:
                buf, reason = read_for(s, 0.25, buf)
                out["board_ms"] = find_idle_ms(buf)
                if out["board_ms"] is not None:
                    out["board_ms_wall"] = time.time()
                if reason == "eof" or reason.startswith("error"):
                    break
            try:
                s.sendall(b"STREAM 0\n")
            except OSError:
                pass
            buf, _ = read_for(s, RESET_DRAIN_S, buf)
        out["resets"] = parse_resets(buf.decode("utf-8", "replace"))
        return out
    except OSError as exc:
        out["tail"] = f"socket 错误：{exc!r}"
        return out
    finally:
        s.close()


# ---------------- 一轮压测 ----------------

def one_round(host: str, port: int, om: float, run_ms: int, uptime_probe: bool = True):
    """一轮 = 一条新连接：PING(+RESET/uptime 探针) → RUN → 关闭。

    返回 (ok, info)。ok 只表示「本轮流程全部走通」；复位/重启判定在 main() 里做
    （info['resets'] / info['board_ms'] 供调用方比较）。
    """
    info = {
        "ping": False,          # 新连接上收到 PONG
        "ok_run": False,        # 收到 OK RUN 回执
        "d_lines": 0,           # D 行条数
        "done": False,          # 收到 DONE（唯一判据）
        "error": "",
        "tail": "",             # 失败时的非 D 行尾巴（诊断用）
        "peer_closed": False,   # 板子主动断开（EOF），而不是超时/我们主动关
        "close_phase": "",      # 断开发生在哪个阶段：ping / run
        "close_reason": "",     # 'eof' / 'error:…' / ''
        "resets": [],           # 本轮缓冲区里见到的 [(RESET 原因, count)]
        "board_ms": None,       # millis 探针（板子 uptime；None=没取到）
        "board_ms_wall": None,  # 取到 board_ms 的墙钟时刻（重启判定用）
        "same_conn_pong": None,     # 失败后同连接 PING 是否回 PONG
        "same_conn_err": "",
        "fresh_conn_pong": None,    # 失败后新连接 PING 是否回 PONG
        "fresh_conn_tail": "",
        "fresh_conn_resets": [],
        "late_done": False,         # 失败后额外窗口内是否收到迟到的 DONE
        "late_d_lines": 0,          # 额外窗口内新增的 D 行
        "gap": None,                # 本轮到达间隔统计
    }
    arrivals: list = []
    sock = None
    buf = b""
    try:
        t_connect = time.time()
        sock = socket.create_connection((host, port), timeout=6.0)
        sock.settimeout(0.5)
        arrivals.append(t_connect)

        # ---- 1) 新连接上 PING 必须回 PONG ----
        sock.sendall(b"PING\n")
        arrivals.append(time.time())
        buf, hit, reason = recv_until(sock, b"PONG", time.time() + PING_TIMEOUT_S, buf, arrivals)
        info["ping"] = hit
        if not hit:
            info["peer_closed"] = reason == "eof"
            info["close_phase"], info["close_reason"] = "ping", reason
            info["resets"] = parse_resets(buf.decode("utf-8", "replace"))
            info["gap"] = gap_stats(arrivals)
            info["error"] = f"PING 在 {PING_TIMEOUT_S:.0f}s 内没有 PONG（{reason}，新连接无响应）"
            info["tail"] = buf.decode("utf-8", "replace").strip().replace("\n", " / ")[-200:]
            return False, info
        buf, _ = read_for(sock, RESET_DRAIN_S, buf, arrivals)   # 等 RESET 行

        # ---- 1b) uptime 探针：STREAM 1 → 取一行 I <millis> → STREAM 0 ----
        # （整板掉电重启不带 RESET 行，这是唯一的外部可观测）
        if uptime_probe:
            sock.sendall(b"STREAM 1\n")
            arrivals.append(time.time())
            deadline = time.time() + UPTIME_TIMEOUT_S
            while info["board_ms"] is None and time.time() < deadline:
                buf, reason = read_for(sock, 0.25, buf, arrivals)
                info["board_ms"] = find_idle_ms(buf)
                if info["board_ms"] is not None:
                    info["board_ms_wall"] = time.time()
                if reason == "eof" or reason.startswith("error"):
                    break
            try:
                sock.sendall(b"STREAM 0\n")
                arrivals.append(time.time())
            except OSError:
                pass
            buf, _ = read_for(sock, RESET_DRAIN_S, buf, arrivals)

        # ---- 2) 原地旋转的 RUN，等 D 行 + DONE ----
        sock.sendall(f"RUN 0 0 {om} {run_ms}\n".encode())
        arrivals.append(time.time())
        deadline = time.time() + run_ms / 1000.0 + DONE_GRACE_S
        buf, hit, reason = recv_until(sock, b"DONE", deadline, buf, arrivals)
        text = buf.decode("utf-8", "replace")
        lines = text.splitlines()
        info["done"] = hit
        info["d_lines"] = sum(1 for ln in lines if ln.startswith("D "))
        info["ok_run"] = any(ln.startswith("OK RUN") for ln in lines)
        if not hit:
            info["peer_closed"] = reason == "eof"
            info["close_phase"] = "run"
            info["close_reason"] = reason
            if reason == "eof":
                info["error"] = (f"RUN {om} {run_ms} 进行中**板子主动断开连接**（EOF）——"
                                 f"看门狗/复位签名（读到 {info['d_lines']} 条 D 行）")
            elif reason.startswith("error"):
                info["error"] = (f"RUN {om} {run_ms} 进行中连接出错（{reason}）——"
                                 f"读到 {info['d_lines']} 条 D 行")
            else:
                info["error"] = (f"RUN {om} {run_ms} 在 {run_ms / 1000.0 + DONE_GRACE_S:.0f}s 内"
                                 f"没有 DONE（只收到 {info['d_lines']} 条 D 行"
                                 f"{'，也没有 OK RUN' if not info['ok_run'] else ''}）")
            info["tail"] = " | ".join(ln for ln in lines if not ln.startswith("D "))[-300:]
            # ---- 失败诊断：R62 签名 ----
            # (a) 旧连接还开着，同连接再 PING
            pong, extra, err = same_conn_ping(sock, buf, arrivals)
            info["same_conn_pong"], info["same_conn_err"] = pong, err
            buf += extra.encode()
            # (b) 旧连接不动，另开新连接 PING（测 accept 路径是否被僵尸占住）
            info["fresh_conn_pong"], info["fresh_conn_tail"], info["fresh_conn_resets"] = \
                fresh_conn_ping(host, port)
            if info["fresh_conn_tail"] and not info["tail"]:
                info["tail"] = f"[新连接] {info['fresh_conn_tail']}"
            # (c) 额外窗口：DONE / D 行会不会迟到
            n_before = info["d_lines"]
            before = len(buf)
            buf, late_done, reason2 = recv_until(sock, b"DONE", time.time() + LATE_WINDOW_S,
                                                 buf, arrivals)
            info["late_done"] = late_done and not hit
            info["late_d_lines"] = (sum(1 for ln in buf.decode("utf-8", "replace").splitlines()
                                        if ln.startswith("D ")) - n_before)
            if not info["peer_closed"] and reason2 == "eof":
                # 迟到窗口里才看到 EOF：板子在停摆后主动踢了我们（看门狗典型时序）
                info["peer_closed"] = True
                info["close_phase"] = "run"
                info["close_reason"] = "eof(late)"
        elif not info["ok_run"]:
            info["error"] = "收到了 DONE 但没有 OK RUN 回执（回复错位？）"

        info["resets"] = parse_resets(buf.decode("utf-8", "replace"))
        info["gap"] = gap_stats(arrivals)
        # 命令序列收尾：无论成功与否都补发一次 STOP（绝不发裸 M）
        try:
            sock.sendall(b"STOP\n")
        except OSError:
            pass
        return (hit and info["ok_run"]), info
    except OSError as exc:
        info["error"] = f"socket 错误：{exc!r}"
        info["resets"] = parse_resets(buf.decode("utf-8", "replace"))
        info["gap"] = gap_stats(arrivals)
        return False, info
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


# ---------------- 主流程 ----------------

def main() -> int:
    ap = argparse.ArgumentParser(description="R62 重连压测（只原地旋转，安全）")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--iters", type=int, default=36, help="重连轮数（判据要求 ≥30）")
    ap.add_argument("--long-every", type=int, default=6,
                    help="每隔几轮插一次长 RUN（0 = 全部用短 RUN）")
    ap.add_argument("--pause", type=float, default=0.15, help="轮间暂停秒数")
    ap.add_argument("--fail-fast", action="store_true", help="首个真失败即退出")
    ap.add_argument("--no-uptime-probe", action="store_true",
                    help="关闭 millis 探针（掉电重启检测；默认开启）")
    ap.add_argument("--json-out", default="", help="把每轮原始记录写到该 JSON 文件")
    args = ap.parse_args()

    uptime_probe = not args.no_uptime_probe
    print(f"R62 重连压测：{args.iters} 轮 · 目标 {args.host}:{args.port} · "
          f"长 RUN 每 {args.long_every} 轮一次")
    print(f"每轮判据：新连接 PING→PONG；RUN 收到 D 行 + DONE（宽限 {DONE_GRACE_S:.0f}s）；"
          f"复位/重启 ⇒ 本轮无效")

    # ---- 基线：会话前的 RESET 状态与板子 millis ----
    base = baseline_probe(args.host, args.port, uptime_probe=uptime_probe)
    if not base["ping"]:
        print(f"❌ 基线 PING 失败：{base['tail']}")
        return 1
    known_resets: dict = {}
    for res_reason, cnt in base["resets"]:
        known_resets[res_reason] = max(known_resets.get(res_reason, -1), cnt)
    prev_board_ms = base["board_ms"]
    prev_board_wall = base.get("board_ms_wall")
    print(f"基线：PONG✓；板子 millis={prev_board_ms}；"
          f"已有复位行={base['resets'] if base['resets'] else '无'}")
    print("-" * 72)

    records = []
    rounds_run = 0
    for i in range(1, args.iters + 1):
        long_round = args.long_every > 0 and i % args.long_every == 0
        if long_round:
            om, run_ms = -2.0, LONG_RUN_MS
        else:
            om, run_ms = -1.0, SHORT_RUN_MS
        rounds_run = i
        ok, info = one_round(args.host, args.port, om, run_ms, uptime_probe=uptime_probe)

        # ---- 复位/掉电判定 ----
        invalids = []
        for res_reason, cnt in info["resets"]:
            if cnt > known_resets.get(res_reason, -1):
                invalids.append(f"发现新复位 RESET {res_reason} count={cnt}"
                                f"（此前 {known_resets.get(res_reason, '无')}）")
                known_resets[res_reason] = cnt
        # millis 探针：板子 millis 至少应随墙钟增长（留 5s 余量）。
        # 只比"回退"不够 —— 重启后如果新 uptime 已超过上次观测值（例如上一轮花了很久），
        # 回退判定会漏（伪板子场景）；按墙钟推算 expected 下限才算完整。
        # 注：>4e9 ms（49.7 天）临近 32 位回绕，跳过以免误报。
        if info["board_ms"] is not None:
            if (prev_board_ms is not None and prev_board_wall is not None
                    and prev_board_ms < 4_000_000_000):
                expected_min = (prev_board_ms
                                + (info["board_ms_wall"] - prev_board_wall) * 1000.0 - 5000.0)
                if info["board_ms"] < expected_min:
                    short = (expected_min - info["board_ms"]) / 1000.0
                    invalids.append(f"板子 millis={info['board_ms']} 少于墙钟推算下限 "
                                    f"{int(expected_min)}（短 {short:.1f}s）⇒ 掉电/重启过")
            prev_board_ms, prev_board_wall = info["board_ms"], info["board_ms_wall"]
        status = "pass" if ok else "fail"
        if invalids:
            status = "invalid"
            # 紧随其后的复位/重启会把前面连续失败的轮次解释掉 → 改判无效
            j = len(records) - 1
            while j >= 0 and records[j]["status"] == "fail":
                records[j]["status"] = "invalid"
                records[j]["note"] = (records[j]["note"] +
                                      "；被紧随其后的复位/重启改判无效").lstrip("；")
                j -= 1

        gap = info["gap"] or {}
        gap_max = f"{gap.get('max'):.2f}s" if gap.get("max") is not None else "n/a"
        tag = "长" if long_round else "短"
        if ok:
            print(f"[{i:3d}/{args.iters}] ✅ {tag} RUN {om} {run_ms}  "
                  f"PING✓ D行{info['d_lines']:3d} DONE✓  max_gap={gap_max}")
        else:
            print(f"[{i:3d}/{args.iters}] ❌ {tag} RUN {om} {run_ms}  PING"
                  f"{'✓' if info['ping'] else '✗'} D行{info['d_lines']:3d} "
                  f"DONE{'✓' if info['done'] else '✗'}  原因：{info['error']}")
            if info["same_conn_pong"] is not None:
                print(f"        诊断：同连接再 PING = "
                      f"{'PONG✓' if info['same_conn_pong'] else '超时✗'}"
                      f"{'（' + info['same_conn_err'] + '）' if info['same_conn_err'] else ''}；"
                      f"新连接 PING = "
                      f"{'PONG✓' if info['fresh_conn_pong'] else '超时✗'}"
                      f"（旧连接保持不动时测）")
                if info["late_done"] or info["late_d_lines"]:
                    print(f"        迟到：额外 {LATE_WINDOW_S:.0f}s 内又收到 "
                          f"DONE={info['late_done']}，D 行 +{info['late_d_lines']}")
            if info["peer_closed"]:
                print(f"        ⚡ 板子主动断开（{info['close_phase']}/{info['close_reason']}）"
                      f"—— 看门狗踢僵尸或板子复位的签名")
            if info["tail"]:
                print(f"        非 D 行尾巴：{info['tail']}")
        if invalids:
            print(f"        ⚠ 本轮无效（不计通过也不计失败）：{'；'.join(invalids)}")
            print(f"          —— 若要满足「干净 N 轮」，本轮需补跑")
        records.append({"round": i, "status": status, "om": om, "run_ms": run_ms,
                        "note": "；".join(invalids), "info": info})
        time.sleep(args.pause)
        if args.fail_fast and status == "fail":
            break

    # ---- 汇总 ----
    passed = [r for r in records if r["status"] == "pass"]
    failed = [r for r in records if r["status"] == "fail"]
    invalid = [r for r in records if r["status"] == "invalid"]
    watchdog_suspect = [r for r in records if r["info"]["peer_closed"]]
    zombie_suspect = [r for r in records
                      if (not r["info"]["done"]) and (not r["info"]["peer_closed"])
                      and r["info"]["same_conn_pong"] is False]
    print("-" * 72)
    print(f"轮次统计：通过 {len(passed)} · 真失败 {len(failed)} · 无效（复位/掉电）{len(invalid)}"
          f"  （共 {len(records)} 轮）")
    if failed:
        print("真失败：")
        for r in failed:
            print(f"  第 {r['round']} 轮（RUN {r['om']} {r['run_ms']}ms）：{r['info']['error']}")
    if invalid:
        print("无效轮次：")
        for r in invalid:
            print(f"  第 {r['round']} 轮：{r['note'] or '（跟随复位/重启改判）'}")
    print(f"看门狗/复位疑似触发（板子主动断连 EOF，共 {len(watchdog_suspect)} 轮）："
          + (", ".join(str(r['round']) for r in watchdog_suspect) if watchdog_suspect else "无"))
    print(f"僵尸签名（TCP 活着但同连接 PING 无响应，共 {len(zombie_suspect)} 轮）："
          + (", ".join(str(r['round']) for r in zombie_suspect) if zombie_suspect else "无"))

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump({"host": args.host, "port": args.port, "iters": args.iters,
                       "rounds_run": rounds_run, "baseline": base,
                       "counts": {"pass": len(passed), "fail": len(failed),
                                  "invalid": len(invalid)},
                       "records": records}, f, ensure_ascii=False, indent=1)
        print(f"原始记录已写入 {args.json_out}")

    if failed:
        print(f"❌ {len(failed)} 轮真失败"
              + (f"；首次失败在第 {failed[0]['round']} 轮" if failed else ""))
        return 1
    if invalid:
        print(f"⚠ 无真失败，但有 {len(invalid)} 轮无效（复位/掉电）—— 需要补跑后再下结论")
        return 2
    print(f"✅ 全部 {len(passed)} 轮有效且通过：没有出现 R62"
          f"（PING 全回、RUN 全部收到 DONE、无复位/掉电、无看门狗踢连接）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
