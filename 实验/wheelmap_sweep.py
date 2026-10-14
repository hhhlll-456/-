#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wheel_map 占空比→轮速采集（Task 7：架空套 + 落地套）。

======================================================================
一、为什么"逐轮逐占空比"用 M 命令而不是 RUN（路子选择的现场考证）
======================================================================
- RUN 是整车运动指令：`RUN vx vy om ms` 先过运动学解成四轮 ω_des，再过
  `wheelOmegaToPWM` 查表 → 实际 pwm。它有两处**不适合建表**：
  1) 请求的是 ω 不是 duty：低位目标 ω 会被 interpDuty 折到小于 MIN_DUTY=110 的
     duty，再被 `if (duty>0 && duty<MIN_DUTY) duty=MIN_DUTY` **钳到 110**
     （main.cpp 经 wheel_map.h）。50/60/80/100 这些档**根本下发不出去**
     —— 这就是裁决 R10 说的钳位陷阱。
  2) 运动学上无法只转一个轮：w=(vx±vy±L·om)/R，任何 (vx,vy,om) 组合都至少
     同时驱动两个轮（对角或同侧），拿不到"单轮稳态转速"。
- M 是直接 PWM：`M fl fr rl rr` 原样 setMotor（不经映射、无 MIN_DUTY 钳位），
  请求 50 就是 50。缺点：M 不产生流式 `D` 行。但固件在 `STREAM 1` 下空闲时也会
  以 100 Hz 回传 `I` 行，字段与 `D` 行完全相同：
      `I <t_ms> <pwm0..3> <enc0..3> <roll> <pitch> <yaw>`
  `pwm` 列就是每个样本时刻**实际下发**的 lastPwm —— 单轮转速与"实际占空比"
  都从这里取（旧架空诊断 `实验/diag_air2.py` 走的同一条路，已上机验证可用）。
  所以：**采集走 M + STREAM 1 的 I 行；R10 要求的"记录实际下发 pwm、丢弃
  实际≠请求的行"逐行执行**（低位档如实记录实际值，且这里根本没有钳位）。

二、为什么测量窗口取"每档后 0.6 s"（不是随便取的）
======================================================================
- 固件四轮按轮序**错峰抬升**（`STAGGER_ENABLED=true`，Task 6 commit 99b1040）：
  第 i 轮在 t0 + i×40 ms 才被放行（`stagger.h`，共 120 ms），放行后还有
  KICK_MS=150 ms 的 kick 窗口（M 直驱时 kick 幅值=目标值，无过冲；
  但 RUN 路线 kick 也在）。
- 上机实测的起步瞬态：`D3=-110 0 0 0` → `D42=-110 110 0 0` →
  `D82=-110 110 -110 0`（Task 6 架空复现两次）——即前 ~120 ms 四轮不完全在转。
- 之后再叠加电机自身的起转（约 100~300 ms）。合计起步瞬态 ≈ 0.4 s。
- 1.2 s 的档期、取**后 0.6 s**：窗口起点 ≈ 档期一半（0.6 s），把上述
  错峰 + kick + 起转全部留在窗口之外。窗口内还要求逐行 pwm 与请求一致，
  双保险（实际≠请求的行会让该档标 invalid，绝不静默写表 —— R10）。

三、判"跑完"与连接管理（R56 三条 + 2026-10-14 现场加深的两条）
======================================================================
- 只认 DONE：本脚本用 M 不需要 DONE；`--run-check` 走 RUN 时以 `DONE` 行判完，
  超时默认 15 s（实测 DONE 墙钟延迟最多 10.87 s，短超时会误判）。
- **绝不打开 /dev/ttyACM0**（DTR/RTS 边沿会复位板子，3/3 复现；另有 ModemManager
  偷探该口 —— 本机已装 ID_MM_DEVICE_IGNORE=1 的 udev 规则，但脚本依旧不碰它）。
- 长连接 + 重试：几十条 M 不重连；只有断线才重连，重连后等 PONG 再继续
  （固件对"陈旧 TCP client"的接受有滞后），并在**新会话第一件事发 STOP** ——
  上一会话若被中断，固件环形缓冲里可能积压未发出的旧字节，会在新连接上先冲
  出来把回复往后挤最多 ~3.5 s（实测首条命令回复延迟 1.4~3.4 s）；STOP 的
  handler 会 resetRunBuf()，直接清干净。
- 【2026-10-14 采集现场新发现，见 `_split_glue`】M/RUN/STOP 的 handler 调
  resetRunBuf() 时，如果某行 I 样本正好**只发出去一半**（非阻塞 send 部分写出），
  剩下字节（含行尾 '\n'）会被清环丢掉 ⇒ 下一条回复（如 "OK M"）**粘**在那半行
  后面（现场原始：`I 2410259 0 0 0 0 0 0 0OK M`）。PC 侧必须拆粘连行，否则
  命令回复超时、整步误判失败。
- 因此**不拿 "OK M" 回复当成功门**：控制回复可能丢/迟到（同族原因），而"M 到底
  有没有生效"有更硬的证据 —— 测量窗口里逐行的实际 pwm 列（R10）。缺回复只记
  `meta['missing']`，由窗口分析定罪；不可用单元才走重测阶梯。

四、落地套的两种驱动图形（本脚本只实现工具，落地采集是人工闸门）
======================================================================
- `--pattern pivot`（默认，推荐先试）：`M +d +d +d +d` / `M -d -d -d -d`。
  由运动学 w=(vx±vy±L·om)/R 反推：四轮板级同号 = 纯原地旋转（vx=vy=0，
  om≠0），车绕着自身中心转、**不跑出场地**，四轮同 duty 一次测完两个方向
  （σ=+d 时 FL/RL 是反向、FR/RR 是正向；σ=−d 反之）。代价：麦轮横刮，
  负载偏大（偏保守）。
- `--pattern straight`：四轮同时"前进"（板级符号 FWD_SIGN）→ 车真的往前走，
  必须有人在场地边看着，且高档位 1.2 s 就可能撞边界（0.5 m/s × 1.2 s ≈ 0.6 m）。
  脚本会在开始前打印每档的预估位移。往返成对下发可抵消净位移。

用法：
    # 架空（车抬起，四轮悬空）：逐轮单轮 M 扫
    python3 wheelmap_sweep.py --mode air
    # 落地（车放场地里）：原地旋转扫（默认）
    python3 wheelmap_sweep.py --mode ground
    # 落地直线往返扫（需人看着车，防撞边界）
    python3 wheelmap_sweep.py --mode ground --pattern straight
    # 架空跑一次 RUN 速度跟踪检查（旧/新表都可用来测；只认 DONE）
    python3 wheelmap_sweep.py --run-check "0.15 0 0 2000"

输出：`固件/exp/data/wheelmap_airborne.json` / `wheelmap_grounded.json`，
结构（裁决 R9）：
    {"c_1rev":[fl,fr,rl,rr], "duty":[50,...], "omega_fwd":[[fl,fr,rl,rr],...],
     "omega_bwd":[...]}
其中 `omega_fwd` = 该轮按**行进正向**（正 ω_des，即 firmware 的
`wheelOmegaToPWM` 里 w≥0 的方向）驱动时的稳态轮速 rad/s；`omega_bwd` 反向。
对 FL/RL，正向 = 板级负 PWM；对 FR/RR，正向 = 板级正 PWM
（与 wheel_map.h invert[4]={T,F,T,F}、固件/历史/kpwm 的 FWD_SIGN={-1,+1,-1,+1} 一致）。
ω < --min-omega（默认 0.5）记 0（死区内，与 brief 一致）；原始测值在网上
用 --raw-out 存一份（报告用），表里只放 brief 规定的值。
若出现任何 invalid 档位（实际 pwm≠请求 / 窗口样本不足 / 不稳定），JSON 会带上
"invalid" 列表且退出码=2 —— 生成器拒绝消费，防止静默写错表（R10）。
"""

import argparse
import json
import math
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from car_link import discover, TCP_PORT  # noqa: E402  复用项目的 UDP 发现 + 端口常量

WHEEL_NAMES = ["FL", "FR", "RL", "RR"]

# 各轮"前进方向"对应的板级 PWM 符号。来源：wheel_map.h invert[4]={true,false,true,false}
# （正 ω_des 需求 → 左轮 −d / 右轮 +d），与 固件/历史/{kpwm,encoder}/src/main.cpp 的
# FWD_SIGN={-1,+1,-1,+1}（"由方向检查得出：左− 右+"）完全一致。
FWD_SIGN = [-1, +1, -1, +1]

# 编码器 A 相 CHANGE 双边沿，每转计数（固件/历史/kpwm/src/main.cpp:32 标定值；
# HANDOFF.md 同值）。脚本可用 --c1rev 覆盖。
DEFAULT_C1REV = [1276.0, 1270.0, 1276.0, 1274.0]

# 与旧表同档位（便于对比），brief Step 1 指定。
DEFAULT_DUTIES = [50, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240]

MODE_OUT = {"air": "wheelmap_airborne.json", "ground": "wheelmap_grounded.json"}

# 轮径（main.cpp:212 R_WHEEL = 0.0335 m），--run-check 估算车体 vx 用。
R_WHEEL = 0.0335

DATA_DIR = os.path.join(REPO, "固件", "exp", "data")


# ---------------------------------------------------------------- 链路
class Sample:
    __slots__ = ("tag", "t", "pwm", "enc", "rpy")

    def __init__(self, tag, t, pwm, enc, rpy):
        self.tag = tag
        self.t = t            # 'D' = 相对 RUN 起点 ms；'I' = millis() ms
        self.pwm = pwm        # 实际下发 pwm（lastPwm）
        self.enc = enc        # 四轮累计计数（单相，只增不减）
        self.rpy = rpy


class Telemetry:
    """TCP 长连接 + 行流式读取（R56：别每条命令重连，断线才重连）。"""

    def __init__(self, ip, port, log=print):
        self.ip, self.port = ip, port
        self.log = log
        self.sock = None
        self.buf = b""
        self.reset_lines = []      # 收到的 RESET 行（非正常复位的证据）
        self.conn_events = []      # 断线/重连事件时间线（报告用）
        self.last_rx = time.time() # 最近一次收到字节的墙钟（静默看门狗用）
        self.silence_timeout = 4.0 # poll 期间超过这么久没任何字节 = 链路死，主动断开

    # ---- 连接 ----
    def connect(self, retries=6, retry_delay=0.5):
        last_err = None
        for _ in range(retries):
            try:
                self.sock = socket.create_connection((self.ip, self.port), timeout=3.0)
                self.sock.settimeout(0.05)
                self.buf = b""
                self.last_rx = time.time()
                if self._handshake(timeout=8.0):
                    # 新会话第一件事：STOP 清环 + 停车。
                    # 上一会话若被中断，固件的环形缓冲里可能积压着未发出的旧字节
                    # （会在新连接上先冲出来，把回复往后挤最多 ~3.5 s）；STOP 的
                    # handler 会 resetRunBuf()，直接清干净。
                    self.cmd("STOP", expect="OK STOP", timeout=8.0)
                    return True
                self._close_sock()
            except OSError as e:
                last_err = e
            time.sleep(retry_delay)
        self.log(f"[link] 连接失败: {last_err}")
        return False

    def _handshake(self, timeout=6.0):
        """重连后要等 PONG：固件对陈旧 client 的接受有滞后（R56）。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                self.sock.sendall(b"PING\n")
            except OSError:
                return False
            deadline = time.time() + 1.0
            while time.time() < deadline:
                _, replies = self._read_once()
                if any(r.text.startswith("PONG") for r in replies):
                    return True
        return False

    def _close_sock(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def close(self):
        self._close_sock()

    # ---- 读取 ----
    def _read_once(self, budget=0.05):
        """在 budget 秒内把 socket 里能读到的字节尽量读干净，解析成完整行。

        注意：`STREAM 1` 下 I 行每 ~10 ms 来一行，"等超时才退出"的写法会一直
        有新数据、永远退不出来 —— 所以用时间预算而非"读到超时"来收敛。
        """
        if self.sock is None:
            raise ConnectionError("socket closed")
        deadline = time.time() + budget
        samples, replies = [], []
        while time.time() < deadline:
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                raise ConnectionError("peer closed")
            self.last_rx = time.time()
            self.buf += chunk
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            ln = raw.decode("utf-8", "replace").strip()
            if not ln:
                continue
            for part in _split_glue(ln):     # 拆粘连的控制行（见 _split_glue 注释）
                if part[0] in ("I", "D") and len(part) > 2 and part[1] == " ":
                    p = part.split()
                    try:
                        samples.append(Sample(p[0], int(p[1]),
                                              [int(x) for x in p[2:6]],
                                              [int(x) for x in p[6:10]],
                                              [float(x) for x in p[10:13]]))
                    except (ValueError, IndexError):
                        replies.append(_Reply(part, time.time()))   # 半行样本/垃圾行
                else:
                    if part.startswith("RESET"):
                        self.reset_lines.append((time.time(), part))
                    replies.append(_Reply(part, time.time()))
        return samples, replies

    def poll(self, duration):
        """持续收 duration 秒；期间"静默看门狗"防僵尸链接。

        背景：实测固件单客户端 TCP 会在网络打嗝后变成僵尸（它不 accept、
        不发数据，但我们的 socket 层还 ESTABLISHED）。傻等 4 s 就能判定，
        主动 raise → 上层重连（重连能不能成就看对面是不是真僵尸；真僵尸
        要 OTA 复位，那是会话外的处置）。
        """
        t0 = time.time()
        samples, replies = [], []
        while time.time() - t0 < duration:
            s, r = self._read_once()
            samples.extend(s)
            replies.extend(r)
            if duration >= 1.0 and time.time() - self.last_rx > self.silence_timeout:
                raise ConnectionError(
                    "遥测静默 %.1fs（链路失联）" % (time.time() - self.last_rx))
        return samples, replies

    def cmd(self, line, expect=None, timeout=8.0):
        """发一条命令并等回复（默认 8 s：新会话的陈旧缓冲最多挤 ~3.5 s 的队）。

        返回 (reply|None, samples, extra_replies)。收到的样本一并带回，
        避免"命令回复期间读走的遥测"被丢掉。
        """
        if self.sock is None:
            raise ConnectionError("socket closed")
        self.sock.sendall((line + "\n").encode())
        t0 = time.time()
        samples, extra = [], []
        while time.time() - t0 < timeout:
            s, r = self._read_once()
            samples.extend(s)
            for rep in r:
                if expect is None or rep.text.startswith(expect):
                    return rep.text, samples, extra
                extra.append(rep.text)
            time.sleep(0.005)
        return None, samples, extra


class _Reply:
    __slots__ = ("text", "wall")

    def __init__(self, text, wall):
        self.text = text
        self.wall = wall


# 控制行词表（用于拆"粘连行"，见 Telemetry._split_glue）。
_CTRL_TOKENS = ("OK ZENC", "OK M", "OK STOP", "OK STREAM", "OK RUN",
                "PONG", "DONE", "RESET", "ERR ")


def _split_glue(ln):
    """把粘连的控制行拆出来（现场实测的真实固件行为，别删）。

    根因（固件侧，本任务不许改）：M/RUN/STOP 的 handler 会调 resetRunBuf() 清环，
    若此刻恰好有一行 I 样本**只发出去一半**（非阻塞 send 部分写出，剩下的还留在环里），
    清环会把剩下的字节（含行尾 '\\n'）丢掉 —— 于是紧接着 push 的回复（如 "OK M"）
    就会**粘在这半行样本后面**。现场原始样本：
        'I 2410259 0 0 0 0 0 0 0OK M'
    PC 侧不拆就会把 "OK M" 连同半行一起当垃圾丢掉，命令回复超时 → 整步误判失败。
    """
    out, rest = [], ln
    while True:
        pos, tok_hit = None, None
        for tok in _CTRL_TOKENS:
            p = rest.find(tok)
            if p > 0 and (pos is None or p < pos):
                pos, tok_hit = p, tok
        if pos is None:
            out.append(rest)
            break
        out.append(rest[:pos])
        rest = rest[pos:]
        # 控制行后面还可能粘着别的控制行（少见）：再看一次
        nxt = None
        for tok in _CTRL_TOKENS:
            p = rest.find(tok, 1)
            if p > 0 and (nxt is None or p < nxt):
                nxt = p
        if nxt is None:
            out.append(rest)
            break
    return [s for s in out if s]


# ---------------------------------------------------------------- 单步采集
def measure_step(link, expect, dwell, first_match_timeout=1.5):
    """一次 M 直驱：ZENC → M → 等实际 pwm 到位 → 采 dwell 秒 → STOP。

    返回 (samples, meta)。meta 记录时间线（t_ref = 发 STOP 前最后一个样本的
    t_ms，测量窗口以它为右端点）。

    容错原则：**回复丢失不算失败** —— 实测这条 TCP 链路会间歇卡顿（R56），
    控制行回复偶尔被吞/迟到；但 "M 到底有没有生效" 有更硬的证据：窗口里每行
    的实际 pwm 列（R10）。所以缺回复只记进 meta['missing']，由窗口分析定罪。
    """
    meta = {"status": "ok", "missing": []}

    link.poll(0.15)   # 丢掉上一步 STOP 后残留的 I 行，避免混进本步统计

    r, samples, _ = link.cmd("ZENC", expect="OK ZENC", timeout=3.0)
    if r is None:
        meta["missing"].append("OK ZENC")

    t_m0 = samples[-1].t if samples else None      # 发 M 前最后一个样本的板载时刻
    r, s2, _ = link.cmd("M %d %d %d %d" % tuple(expect), expect="OK M", timeout=3.0)
    samples += s2
    if r is None:
        meta["missing"].append("OK M")

    # 等第一个"实际 pwm == 请求"的样本（错峰放行：第 i 轮 i*40 ms 才上电）。
    t0 = time.time()
    first_match_ms = None
    while time.time() - t0 < first_match_timeout:
        s, _ = link._read_once()
        samples.extend(s)
        for smp in s:
            if smp.pwm == expect:
                first_match_ms = smp.t
                break
        if first_match_ms is not None:
            break
        time.sleep(0.005)
    meta["t_first_match_ms"] = first_match_ms
    if first_match_ms is not None and t_m0 is not None:
        meta["release_delay_ms"] = first_match_ms - t_m0   # ≈错峰放行延迟（FL 0 / FR 40 / RL 80 / RR 120 ms）

    # 采集 dwell 秒（从"到位"起算，含错峰+起转瞬态；窗口只取后 0.6 s）。
    s, _ = link.poll(dwell)
    samples.extend(s)
    t_ref = samples[-1].t if samples else None
    meta["t_ref"] = t_ref

    r, s2, _ = link.cmd("STOP", expect="OK STOP", timeout=3.0)
    samples += s2
    if r is None:
        meta["missing"].append("OK STOP")

    if not samples or t_ref is None:
        meta["status"] = "no_samples"
    return samples, meta


def analyze(samples, wheel, expect, c1rev, t_ref, window_ms, min_omega):
    """从一步样本里取窗口并算该轮稳态 ω（rad/s，无方向）。

    - 窗口 = (t_ref − window_ms, t_ref] 的样本；
    - 窗口内每一行都必须 pwm == expect（R10：实际≠请求 → 该档 invalid，
      不做任何"跳过坏行接着算"的桥接）；
    - 半窗转速差 >25% → unstable（还没稳就别测）；
    - ω < min_omega → 记 0（死区内，实测原始值仍留在 omega_raw）。
    """
    res = {"status": "ok", "n_samples": len(samples), "n_win": 0,
           "win_span_ms": None, "n_mismatch_before_window": 0,
           "n_mismatch_in_window": 0, "omega_raw": None, "omega": None,
           "in_deadzone": False, "dcnt": None, "half1": None, "half2": None}
    if t_ref is None:
        res["status"] = "no_samples"
        return res
    win = [s for s in samples if t_ref - window_ms < s.t <= t_ref]
    res["n_win"] = len(win)
    res["n_mismatch_before_window"] = sum(
        1 for s in samples if s.t <= t_ref - window_ms and s.pwm != expect)
    bad = [s for s in win if s.pwm != expect]
    res["n_mismatch_in_window"] = len(bad)
    if len(win) < 20:
        res["status"] = "short_window"
        return res
    span_ms = win[-1].t - win[0].t
    res["win_span_ms"] = span_ms
    if span_ms < 0.8 * window_ms:
        res["status"] = "short_window"
        return res
    if bad:
        res["status"] = "pwm_mismatch"
        return res

    def rate(lo, hi):
        a = [s for s in win if lo < s.t <= hi]
        if len(a) < 2:
            return None, 0
        d = a[-1].enc[wheel] - a[0].enc[wheel]
        dt = (a[-1].t - a[0].t) / 1000.0
        if dt <= 0:
            return None, 0
        return d * 2.0 * math.pi / c1rev[wheel] / dt, d

    omega, dcnt = rate(t_ref - window_ms, t_ref)
    if omega is None:
        res["status"] = "short_window"
        return res
    res["omega_raw"] = omega
    res["dcnt"] = dcnt

    # 半窗稳定性：两半都至少有 15 个计数才判（避免低速量化噪声误报）。
    mid = (t_ref - window_ms + t_ref) / 2.0
    w1, d1 = rate(t_ref - window_ms, mid)
    w2, d2 = rate(mid, t_ref)
    res["half1"], res["half2"] = w1, w2
    if w1 is not None and w2 is not None and d1 >= 15 and d2 >= 15:
        denom = max(abs(w1), abs(w2), 1e-9)
        if abs(w1 - w2) / denom > 0.25:
            res["status"] = "unstable"
            return res

    if omega < min_omega:
        res["omega"] = 0.0
        res["in_deadzone"] = True
    else:
        res["omega"] = omega
    return res


# ---------------------------------------------------------------- 采集流程
class SweepAbort(Exception):
    pass


class Sweeper:
    def __init__(self, link, cfg, log):
        self.link = link
        self.cfg = cfg
        self.log = log
        self.raw = []          # 每步的原始记录（--raw-out）
        self.invalid = {}      # (wheel, duty, dir) -> reason；写进 JSON，生成器拒绝消费
        self.data = {"omega_fwd": [[0.0] * 4 for _ in cfg.duties],
                     "omega_bwd": [[0.0] * 4 for _ in cfg.duties]}

    # -- 冷却/换轮等待：边等边清管道（防积压冲环，见 _split_glue） ------
    def nap(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                self.link._read_once(budget=0.05)
            except (OSError, ConnectionError):
                return

    # -- 单次尝试（断线才重连；回复异常交上层重测） ------------------
    def _once(self, expect, tag, dwell):
        for _ in range(3):
            try:
                samples, meta = measure_step(self.link, expect, dwell)
            except (ConnectionError, ConnectionResetError, OSError) as e:
                self.log(f"  [!] 断线: {e} —— 尝试重连")
                self.link.conn_events.append((time.time(), str(e)))
                if not self.link.connect():
                    raise SweepAbort("重连失败，中止采集（已保存部分数据）")
                continue
            if meta.get("missing"):
                self.log(f"  [i] {tag}: 缺回复 {meta['missing']}（不算失败，以窗口 pwm 列为准）")
            if meta["status"] != "ok":
                self.log(f"  [!] 步骤异常 {meta['status']}（{tag}）")
                return None, meta
            return samples, meta
        return None, {"status": "conn_failed"}

    def exec_step(self, expect, tag, results_map):
        """跑一步、落数；失败/不可用按"降强度"阶梯重测（R56 精神）。

        results_map = {(wheel, direction): (k, expect)}。
        阶梯：① 原参数 → ② 原参数重试（先 STOP 清环 + PING 探活）→
              ③ 延长窗口重测（dwell×2.5 —— 对付低速 stick-slip 型 unstable）。
        """
        plans = [(self.cfg.dwell, self.cfg.window),
                 (self.cfg.dwell, self.cfg.window),
                 (self.cfg.dwell * 2.5, self.cfg.window * 2.5)]
        samples, meta, res = None, {"status": "step_failed"}, {}
        got_samples = False
        for attempt, (dwell, window) in enumerate(plans):
            samples, meta = self._once(expect, tag, dwell)
            if samples:
                got_samples = True
            if samples is None:
                if attempt < len(plans) - 1:
                    self.log("      → STOP 清环 + PING 探活后重测")
                    _safe_cmd(self.link, "STOP", "OK STOP")
                    time.sleep(0.6)
                    _safe_cmd(self.link, "PING", "PONG")
                    continue
                break
            res = {kk: (k, exp, analyze(samples, kk[0], exp, self.cfg.c1rev,
                                        meta.get("t_ref"), window * 1000.0, self.cfg.min_omega))
                   for kk, (k, exp) in results_map.items()}
            bad = [kk for kk, (_, _, r) in res.items() if r["status"] != "ok"]
            if not bad:
                break
            if attempt < len(plans) - 1:
                self.log(f"      → 有 {len(bad)} 个单元不可用（"
                         f"{sorted(set(res[kk][2]['status'] for kk in bad))}）"
                         f"，下一步 dwell={plans[attempt + 1][0]:.1f}s 重测")
                time.sleep(0.5)

        if not got_samples:
            # 三次尝试一条样本都没收到 = 不是"这一档测坏了"，是链路死了。
            # 实测固件的单客户端 TCP 会进入僵尸态：socket 层 established，
            # 但固件只认旧 client（connected() 恒真）→ 永远不再 accept，
            # 也不回任何数据。这种状态**只能复位板子**（本机走 OTA 重刷，
            # 不动 USB 串口）。立刻中止会话，别在死链上空转（R56：不硬轰）。
            raise SweepAbort("连续 3 次没收到任何样本 —— TCP 链路失联"
                             "（固件僵尸 client？需复位板子；本工具用 OTA 重刷恢复）")

        rec = {"tag": tag, "n_samples": len(samples) if samples else 0,
               "meta_status": meta.get("status"),
               "missing_replies": meta.get("missing"),
               "release_delay_ms": meta.get("release_delay_ms")}
        for (wheel, direction), (k, expect, r) in res.items():
            ok = r["status"] == "ok"
            key = (wheel, self.cfg.duties[k], direction)
            if ok:
                self.invalid.pop(key, None)
                value = r["omega"]
            else:
                self.invalid[key] = r["status"]
                value = 0.0
            self.data["omega_fwd" if direction == "fwd" else "omega_bwd"][k][wheel] = value
            rec[f"{WHEEL_NAMES[wheel]}_{direction}"] = r
            self.log("  %s d=%3d %s: %s" % (
                WHEEL_NAMES[wheel], self.cfg.duties[k], direction,
                ("invalid(%s)" % r["status"]) if not ok else
                ("ω=%.2f rad/s%s" % (value, "（死区记0，实测 %.2f）" % r["omega_raw"]
                                     if r["in_deadzone"] else
                                     "  (Δ%d cnt, 半窗 %.2f/%.2f)" % (r["dcnt"], r["half1"], r["half2"])))))
        self.raw.append(rec)

    # -- JSON 保存（每步都存，断线不丢已完成部分） ------------------
    def save(self):
        os.makedirs(os.path.dirname(self.cfg.out), exist_ok=True)
        obj = {"c_1rev": self.cfg.c1rev, "duty": self.cfg.duties,
               "omega_fwd": _round3(self.data["omega_fwd"]),
               "omega_bwd": _round3(self.data["omega_bwd"])}
        if self.invalid:
            obj["invalid"] = [{"wheel": WHEEL_NAMES[w], "duty": d, "dir": dd, "reason": rr}
                              for (w, d, dd), rr in sorted(self.invalid.items())]
        tmp = self.cfg.out + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self.cfg.out)
        if self.cfg.raw_out:
            os.makedirs(os.path.dirname(self.cfg.raw_out), exist_ok=True)
            with open(self.cfg.raw_out, "w", encoding="utf-8") as f:
                json.dump({"mode": self.cfg.mode, "pattern": self.cfg.pattern,
                           "c1rev": self.cfg.c1rev, "duties": self.cfg.duties,
                           "steps": self.raw,
                           "conn_events": self.link.conn_events,
                           "reset_lines": self.link.reset_lines},
                          f, ensure_ascii=False, indent=1)
                f.write("\n")


def _safe_cmd(link, line, expect=None):
    """收尾用：连接可能已死，任何 OSError 都吞掉。"""
    try:
        return link.cmd(line, expect=expect, timeout=8.0)
    except (OSError, AttributeError):
        return None, [], []


def _round3(table):
    return [[round(v, 3) for v in row] for row in table]


def sweep_air(sw):
    """架空：逐轮、逐档、正反各 1.2 s，单轮 M 直驱。"""
    log = sw.log
    for wi in range(4):
        if wi not in sw.cfg.wheels:
            continue
        log(f"[轮 {WHEEL_NAMES[wi]}] 单轮 M 扫（架空，其余三轮不驱动）")
        for k, duty in enumerate(sw.cfg.duties):
            for direction, dname in ((+1, "fwd"), (-1, "bwd")):
                expect = [0, 0, 0, 0]
                expect[wi] = FWD_SIGN[wi] * direction * duty
                tag = f"air {WHEEL_NAMES[wi]} d={duty} {dname}"
                sw.exec_step(expect, tag, {(wi, dname): (k, expect)})
                sw.save()
                sw.nap(sw.cfg.cooldown)
        sw.nap(sw.cfg.wheel_pause)


def sweep_ground(sw):
    """落地：pivot（默认，原地旋转）= M ±d 全轮；straight = 全轮前进/后退。"""
    log = sw.log
    pattern = sw.cfg.pattern
    if pattern == "straight":
        # 预估位移（用旧架空表的粗估量级给操作者一个感觉：高档很危险）。
        log("[!] straight 模式：车会真的行走。请确认场地够长、有人看着、随时能断电。")
        log("    参考：轮速 ~15–35 rad/s（对照旧数据）时车体 ~0.5–1.1 m/s，")
        log("    1.2 s 档期位移可达 0.6–1.3 m —— 场地只有 0.5 m 长，高档位请勿用 straight。")
    for k, duty in enumerate(sw.cfg.duties):
        for sigma in (+1, -1):
            if pattern == "pivot":
                expect = [sigma * duty] * 4
                tag = f"ground pivot d={duty} sigma={'+' if sigma > 0 else '-'}"
                results = {}
                for wi in range(4):
                    direction = "fwd" if expect[wi] == FWD_SIGN[wi] * duty else "bwd"
                    results[(wi, direction)] = (k, expect)
            else:  # straight：四轮 σ 方向同驱
                expect = [FWD_SIGN[wi] * sigma * duty for wi in range(4)]
                tag = f"ground straight d={duty} sigma={'+' if sigma > 0 else '-'}"
                direction = "fwd" if sigma > 0 else "bwd"
                results = {(wi, direction): (k, expect) for wi in range(4)}
            if not any(wi in sw.cfg.wheels for wi in range(4)):
                continue
            # 只保留被选轮的结果
            results = {kk: vv for kk, vv in results.items() if kk[0] in sw.cfg.wheels}
            sw.exec_step(expect, tag, results)
            sw.save()
            sw.nap(sw.cfg.cooldown)


# ---------------------------------------------------------------- RUN 检查
def run_check(link, cfg, args_str):
    """`--run-check "vx vy om ms"`：跑一次 RUN，只认 DONE 判完（R56，超时 ≥15 s），
    用 D 行遥测算实际轮速→车体 vx，与指令比较。落地/架空都能用。"""
    log = cfg.log
    parts = args_str.split()
    if len(parts) != 4:
        raise SystemExit("--run-check 需要 'vx vy om ms' 四个数")
    vx, vy, om, ms = (float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3]))
    link.cmd("STREAM 0")           # RUN 期间是 D 行，别混 I 行
    reply, samples, extra = link.cmd(f"RUN {vx} {vy} {om} {ms}", expect="OK RUN", timeout=8.0)
    log(f"[run-check] 命令: RUN {vx} {vy} {om} {ms}")
    log(f"[run-check] OK RUN 回复: {reply}")      # 固件算出的 pwm（R10 现场证据）
    if reply is None:
        log("[run-check] [!] 没收到 OK RUN（会话卡顿，命令可能已生效）—— 改看 D 行判活")
    wall0 = time.time()
    done_wall = None
    while time.time() - wall0 < cfg.done_timeout:
        s, r = link._read_once()
        samples.extend(s)
        if any(x.text.startswith("DONE") for x in r):
            done_wall = time.time() - wall0
            break
        time.sleep(0.01)
    if done_wall is None:
        log(f"[run-check] [!] {cfg.done_timeout}s 内没等到 DONE —— 判为异常（复位/掉线？）")
        return 1
    d_lines = [s for s in samples if s.tag == "D"]
    log(f"[run-check] DONE（判完只认 DONE）: 收到 {len(d_lines)} 行 D 样本, "
        f"墙钟 {done_wall:.2f}s, 标称 {ms}ms")
    if not d_lines:
        log("[run-check] 没有 D 行，无法算轮速")
        return 1
    t_ref = d_lines[-1].t
    win = [s for s in d_lines if s.t > t_ref - 600]
    span = (win[-1].t - win[0].t) / 1000.0
    omega = []
    for wi in range(4):
        d = win[-1].enc[wi] - win[0].enc[wi]
        w = d * 2.0 * math.pi / cfg.c1rev[wi] / span
        # 单相编码器不能辨向：用该轮 D 行的 pwm 符号定方向（0 = 该轮没动）
        pwm_mid = win[len(win) // 2].pwm[wi]
        sign = 1 if pwm_mid == 0 else (1 if pwm_mid == FWD_SIGN[wi] * abs(pwm_mid) else -1)
        omega.append(sign * w)
        log(f"[run-check]   {WHEEL_NAMES[wi]}: pwm={pwm_mid:+4d} → ω={sign * w:+.2f} rad/s "
            f"(Δ{d} cnt / {span:.2f}s)")
    vx_est = R_WHEEL / 4.0 * sum(omega)
    log(f"[run-check] 实测车体 vx ≈ {vx_est:+.3f} m/s（指令 {vx:+.3f}），"
        f"偏差 {abs(vx_est - vx) / max(abs(vx), 1e-9) * 100:.0f}%")
    log("[run-check] 注：这是开环估法（编码器无方向，按 pwm 符号定轮子方向）。")
    return 0


# ---------------------------------------------------------------- 离散度打印
def print_tables(cfg, data, log):
    log("")
    log("=" * 72)
    log("采集结果（rad/s；* = 死区记 0 的原始实测值见 --raw-out）")
    hdr = "duty | " + " | ".join(f"{n:^17s}" for n in WHEEL_NAMES)
    for tag, key in (("正向 fwd", "omega_fwd"), ("反向 bwd", "omega_bwd")):
        log(f"--- {tag} ---")
        log(hdr)
        for k, duty in enumerate(cfg.duties):
            row = data[key][k]
            log(f"{duty:4d} | " + " | ".join(f"{v:17.3f}" for v in row))
    log("")
    log("[四轮离散度] (max−min)/mean ×100%，只统计 ω>0 的轮")
    all_sp = []
    for k, duty in enumerate(cfg.duties):
        for tag, key in (("fwd", "omega_fwd"), ("bwd", "omega_bwd")):
            vals = data[key][k]
            pos = [v for v in vals if v > 0]
            if len(pos) >= 2:
                sp = (max(pos) - min(pos)) / (sum(pos) / len(pos)) * 100.0
                all_sp.append(sp)
                log(f"  duty={duty:3d} {tag}: {['%.2f' % v for v in vals]}  spread={sp:5.1f}%")
            else:
                log(f"  duty={duty:3d} {tag}: {['%.2f' % v for v in vals]}  (起转轮 <2，跳过)")
    if all_sp:
        log(f"  合计平均离散度 = {sum(all_sp) / len(all_sp):.1f}%   （旧架空实测 13.5%）")


# ---------------------------------------------------------------- main
def parse_args():
    ap = argparse.ArgumentParser(
        description="wheel_map 占空比→轮速采集（Task 7）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["air", "ground"],
                    help="air=架空逐轮单轮扫；ground=落地扫（人工闸门，确认车已着地再做）")
    ap.add_argument("--pattern", choices=["pivot", "straight"], default="pivot",
                    help="ground 模式的驱动图形：pivot=原地旋转（默认，不跑出场地）；"
                         "straight=全轮前进/后退（车会走，需人看着）")
    ap.add_argument("--dwell", type=float, default=1.2, help="每档驱动时长 s（默认 1.2）")
    ap.add_argument("--window", type=float, default=0.6, help="测量窗口取档期最后多少秒（默认 0.6）")
    ap.add_argument("--cooldown", type=float, default=0.8, help="档间冷却 s（默认 0.8，防空转过热）")
    ap.add_argument("--wheel-pause", type=float, default=2.0, help="换轮暂停 s（默认 2.0）")
    ap.add_argument("--c1rev", default="1276,1270,1276,1274",
                    help="每转计数 FL,FR,RL,RR（默认 固件/历史/kpwm 标定值）")
    ap.add_argument("--duty", default=",".join(str(d) for d in DEFAULT_DUTIES),
                    help="占空比序列（默认与旧表同档位 50..240）")
    ap.add_argument("--wheels", default="0,1,2,3", help="只测哪些轮（默认全部）")
    ap.add_argument("--host", default=None, help="小车 IP（默认 UDP 发现，失败回落 192.168.1.115）")
    ap.add_argument("--out", default=None,
                    help="输出 JSON（默认 固件/exp/data/wheelmap_{airborne|grounded}.json）")
    ap.add_argument("--raw-out", default=None, help="原始逐步记录 JSON（可选，报告用）")
    ap.add_argument("--min-omega", type=float, default=0.5, help="低于此值记 0（默认 0.5 rad/s）")
    ap.add_argument("--done-timeout", type=float, default=15.0,
                    help="--run-check 等 DONE 的超时 s（默认 15；实测 DONE 延迟最多 10.87 s）")
    ap.add_argument("--run-check", default=None, metavar='"vx vy om ms"',
                    help="不扫表，跑一次 RUN 并算实际 vx（落地/架空都可）")
    return ap.parse_args()


def main():
    args = parse_args()
    log = print
    cfg = argparse.Namespace(**vars(args))
    cfg.c1rev = [float(x) for x in args.c1rev.replace(",", " ").split()]
    if len(cfg.c1rev) != 4:
        raise SystemExit("--c1rev 需要 4 个数")
    cfg.duties = [int(x) for x in args.duty.replace(",", " ").split()]
    cfg.wheels = {int(x) for x in args.wheels.replace(",", " ").split()}
    if cfg.window > cfg.dwell - 0.3:
        raise SystemExit("--window 必须 ≤ --dwell − 0.3（保证窗口在错峰/起转之后）")
    cfg.log = log
    if cfg.done_timeout < 15.0 and args.run_check:
        log("[!] --done-timeout < 15 s：R56 实测 DONE 墙钟延迟最多 10.87 s，短超时会误判")
    if not args.mode and not args.run_check:
        raise SystemExit("要么给 --mode {air,ground}，要么给 --run-check")

    # 连接（UDP 发现失败回落固定 IP；长连接全程复用）
    ip = args.host
    if ip is None:
        found = discover(timeout=3.0)
        ip = found[0] if found else "192.168.1.115"
    log(f"[link] 连接 {ip}:{TCP_PORT}（只走 TCP；绝不打开串口，避免 DTR/RTS 复位）")
    link = Telemetry(ip, TCP_PORT, log=log)
    if not link.connect():
        raise SystemExit("连不上小车：确认上电、同 Wi-Fi(B1222)")
    log("[link] 握手 OK（PONG）")
    rc = 0
    try:
        if args.run_check:
            return run_check(link, cfg, args.run_check)

        cfg.mode = args.mode
        cfg.out = args.out or os.path.join(DATA_DIR, MODE_OUT[args.mode])
        cfg.raw_out = args.raw_out
        sw = Sweeper(link, cfg, log)

        log(f"[cfg] mode={cfg.mode} pattern={cfg.pattern} dwell={cfg.dwell}s "
            f"window={cfg.window}s cooldown={cfg.cooldown}s")
        log(f"[cfg] duty={cfg.duties}")
        log(f"[cfg] c_1rev={cfg.c1rev}（固件/历史/kpwm 标定值；--c1rev 可覆盖）")

        # 会话开始：STOP 清环 + 停车，STREAM 1 开流。
        # 回复可能丢/迟到（实测这条 TCP 会间歇卡顿）—— 不拿 OK 回复当门，
        # 用"静置 1 s 收到 ~100 行 I 线"来验证流真的开了（比 OK STREAM 硬）。
        _safe_cmd(link, "STOP", "OK STOP")
        _safe_cmd(link, "STREAM 1", "OK STREAM")
        stream_ok = False
        for attempt in range(4):
            base, _ = link.poll(1.0)
            b = [s for s in base if s.tag == "I"]
            if len(b) >= 50:
                stream_ok = True
                break
            log(f"[i] 流没起来（1s 只有 {len(b)} 行 I），PING/STREAM 后重试")
            _safe_cmd(link, "PING", "PONG")
            _safe_cmd(link, "STREAM 1", "OK STREAM")
        if not stream_ok:
            raise SweepAbort("静置 1 s 收不到 I 行（流没开或链路卡死）")
        if link.reset_lines:
            # RESET 行表示板子**本次启动**是非正常复位（RTC 记录会一直报到下次复位）。
            # 不一定和本次采集有关 —— 如实记录、继续采；采集中若再断线会走重连并标记。
            log(f"[!] 板子本次启动曾非正常复位（继续采，但报告里要写清）: "
                f"{link.reset_lines[-1][1]}")

        # 静置基线：确认编码器不假计数（旧架空诊断判据）
        b = [s for s in base if s.tag == "I"]
        delta = [b[-1].enc[i] - b[0].enc[i] for i in range(4)]
        log(f"[基线] 静置 1 s 编码器增量 {delta}（应全 0），I 行 {len(b)} 行/1s（≈100 Hz）")
        if any(delta):
            raise SweepAbort("静置编码器有假计数，先查接线/上拉")

        try:
            if cfg.mode == "air":
                sweep_air(sw)
            else:
                sweep_ground(sw)
        except SweepAbort as e:
            log(f"[!] 采集中止: {e}")
            rc = 2
        finally:
            _safe_cmd(link, "STOP", "OK STOP")
            _safe_cmd(link, "STREAM 0", "OK STREAM")
            sw.save()
            log(f"[out] {cfg.out}（invalid={len(sw.invalid)}）")
            if cfg.raw_out:
                log(f"[out] {cfg.raw_out}")

        if sw.invalid:
            log(f"[!] 有 {len(sw.invalid)} 个不可用档位（已写进 JSON 的 invalid，"
                f"生成器会拒绝消费）:")
            for (w, d, dd), rr in sorted(sw.invalid.items()):
                log(f"    {WHEEL_NAMES[w]} duty={d} {dd}: {rr}")
            rc = 2
        print_tables(cfg, sw.data, log)
        if link.reset_lines:
            # RESET 行 = 板子本次启动有非正常复位（RTC 会一直报）。可能发生在采集
            # 之前（比如昨天烧录/上电），不定罪、只如实上报；采集期间真复位一般会
            # 先表现为断线（见 conn_events），那会把受影响档位标 invalid。
            log("[!] 观察到 RESET 行（板子本次启动曾非正常复位）:")
            for t, l in link.reset_lines:
                log(f"    {l}")
        if link.conn_events:
            log(f"[!] 断线事件 {len(link.conn_events)} 次（原因与恢复见 --raw-out）")
    finally:
        _safe_cmd(link, "STOP", "OK STOP")
        link.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
