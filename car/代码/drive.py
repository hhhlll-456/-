#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""麦轮小车 —— 一步到位：目标 → 该发多少毫秒 → 发给车。

    python3 drive.py --find                     # UDP 发现小车（不知道 IP 时先跑这个）
    python3 drive.py --ping                     # PING / ID / CHIP（不驱动电机）
    python3 drive.py --plan 前进 200            # 只算不发，**不连车**
    python3 drive.py --go   前进 200            # 算 + 连车 + 跑
    python3 drive.py --go   左移 50 --ip 192.168.1.115

动作：前进 后退 左移 右移 逆时针 顺时针 前左 前右 后左 后右
单位：平移类 = **毫米**；旋转类 = **度**。

本文件只做两件事：查 `correct.py` 的常数表、把 `RUN` 发出去。
行程标定的来龙去脉见同目录 `../对接.md`。
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import correct                                    # noqa: E402
from car_link import CarLink, discover            # noqa: E402

# 动作 → (通道, 方向, RUN 的 vx/vy/om, 单位)
# 命令幅值本身是惰性的（固件把占空比钳到 ±110），所以 vx/vy/om 只要"够大、方向对"就行，
# 统一用 ±0.2 —— 和标定时用的值一致。**方向符号已用真机数据核对过**（见 对接.md §4）。
ACTIONS: dict[str, dict] = {
    "前进":   dict(channel="纵向", direction="前进",   vx=0.2,  vy=0.0,  om=0.0,  unit="mm"),
    "后退":   dict(channel="纵向", direction="后退",   vx=-0.2, vy=0.0,  om=0.0,  unit="mm"),
    "左移":   dict(channel="横向", direction="左移",   vx=0.0,  vy=0.2,  om=0.0,  unit="mm"),
    "右移":   dict(channel="横向", direction="右移",   vx=0.0,  vy=-0.2, om=0.0,  unit="mm"),
    "逆时针": dict(channel="旋转", direction="逆时针", vx=0.0,  vy=0.0,  om=0.2,  unit="度"),
    "顺时针": dict(channel="旋转", direction="顺时针", vx=0.0,  vy=0.0,  om=-0.2, unit="度"),
    "前左":   dict(channel="斜行", direction="前左",   vx=0.2,  vy=0.2,  om=0.0,  unit="mm"),
    "前右":   dict(channel="斜行", direction="前右",   vx=0.2,  vy=-0.2, om=0.0,  unit="mm"),
    "后左":   dict(channel="斜行", direction="后左",   vx=-0.2, vy=0.2,  om=0.0,  unit="mm"),
    "后右":   dict(channel="斜行", direction="后右",   vx=-0.2, vy=-0.2, om=0.0,  unit="mm"),
}


def make_plan(action: str, amount: float) -> dict:
    """查表算方案（不连车）。返回 dict，含 T_ms / 预测 / RUN 行 / 警告。"""
    if action not in ACTIONS:
        raise KeyError(f"未知动作 {action!r}；可选：{' '.join(ACTIONS)}")
    a = ACTIONS[action]
    ch, dr = a["channel"], a["direction"]
    T = correct.duration_ms(ch, dr, amount)
    c = correct.CHANNELS[correct._resolve(ch, dr)]
    run = correct.command_line(a["vx"], a["vy"], a["om"], T)
    warn = []
    bias = correct.YAW_BIAS_DEG.get((ch, dr))
    if bias is not None:
        warn.append(f"跑完车头会偏 {bias:+.2f}°（组内散布 ±{correct.YAW_BIAS_SPREAD_DEG[(ch, dr)]:.2f}°）"
                    f" —— 开环补不掉")
    if ch == "横向" and abs(amount) > correct.STRAFE_CAL_MAX_MM:
        warn.append(f"横移 {amount} mm 超出标定区间（上限 {correct.STRAFE_CAL_MAX_MM:.1f} mm）："
                    f"只能保证送出去，**不能保证走直**。长距离请改用直行；拆小步会更糟")
    return dict(action=action, amount=amount, unit=a["unit"], channel=ch, direction=dr,
                T_ms=T, predicted=round(correct.travel(ch, dr, T), 2),
                sigma=c.sigma, n=c.n, r2=c.r2, run=run, warnings=warn)


# --------------------------------------------------------------------------
# 连车
# --------------------------------------------------------------------------

def read_lines(sock: socket.socket, seconds: float) -> list[str]:
    """按给定秒数尽量排空链路，返回整行列表（超时不算错误）。"""
    end = time.time() + seconds
    buf, out = b"", []
    while time.time() < end:
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            out.append(line.decode("utf-8", "replace").strip())
    return out


def sample_track(lines: list[str]) -> list[tuple[int, list[int], float]]:
    """从 D 行里抽出 (t_ms, enc[4], yaw)。"""
    rows = []
    for ln in lines:
        p = ln.split()
        if ln.startswith("D ") and len(p) == 13:
            try:
                rows.append((int(p[1]), [int(x) for x in p[6:10]], float(p[12])))
            except ValueError:
                pass
    return rows


def run_on_car(plan: dict, ip: str | None, settle: float = 0.4) -> int:
    car = CarLink(ip=ip)
    sock = car.sock
    sock.settimeout(0.2)
    try:
        sock.sendall(b"PING\n")
        time.sleep(0.4)
        banner = read_lines(sock, 0.8)
        print(f"链路 OK（{car.addr[0]}:{car.addr[1]}）: {' '.join(banner[:2])}")
        for ln in banner:
            if ln.startswith("RESET "):
                print(f"  ⚠ 板子上次是异常复位：{ln}（可能是 brownout / 掉电）")
        sock.sendall(b"STREAM 0\n")          # 跑之前先关掉空闲流，免得淹了 D 行
        read_lines(sock, 0.6)

        print(f"\n▶ {plan['run']}   （{plan['action']} {plan['amount']}{plan['unit']} → "
              f"预测 {plan['predicted']}{plan['unit']}）")
        sock.sendall((plan["run"] + "\n").encode())
        lines = read_lines(sock, plan["T_ms"] / 1000.0 + 3.0)
        done = any(ln == "DONE" for ln in lines)
        rows = sample_track(lines)
        print(f"   D 样本 {len(rows)} 条   DONE={done}")
        if rows:
            enc = [abs(rows[-1][1][i] - rows[0][1][i]) for i in range(4)]
            print(f"   编码器增量 {enc}")
            print(f"   行程内 yaw 变化 {rows[-1][2] - rows[0][2]:+.2f}°")
        if not done:
            print("   ⚠ 没等到 DONE —— 链路可能断了；车已 DR 停机（固件有失联停车）")
        return 0 if done else 1
    finally:
        try:
            sock.sendall(b"STOP\n")
            sock.sendall(b"STREAM 0\n")
            time.sleep(0.3)
        except OSError:
            pass
        car.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="麦轮小车：目标 → 毫秒 → 发给车")
    ap.add_argument("--find", action="store_true", help="UDP 广播发现小车")
    ap.add_argument("--ping", action="store_true", help="PING/ID/CHIP，不驱动电机")
    ap.add_argument("--plan", nargs=2, metavar=("动作", "量"), help="只算不发，不连车")
    ap.add_argument("--go", nargs=2, metavar=("动作", "量"), help="算 + 连车 + 跑")
    ap.add_argument("--ip", default=None, help="小车 IP（缺省自动 UDP 发现）")
    ap.add_argument("--list", action="store_true", help="列出所有动作与常数")
    args = ap.parse_args()

    if args.list:
        print(f"{'动作':6s}{'通道':5s}{'方向':8s}{'V':>9s}{'t_d(ms)':>9s}{'n':>4s}{'R²':>8s}{'车头偏':>8s}")
        for name, a in ACTIONS.items():
            c = correct.CHANNELS[correct._resolve(a["channel"], a["direction"])]
            bias = correct.YAW_BIAS_DEG.get((a["channel"], a["direction"]))
            print(f"{name:6s}{a['channel']:5s}{a['direction']:8s}{c.V:9.1f}{c.t_d_ms:9.1f}"
                  f"{c.n:4d}{c.r2:8.4f}{(f'{bias:+.2f}°' if bias is not None else '—'):>8s}"
                  f"   {c.unit}/s")
        return 0

    if args.find:
        found = discover()
        print("发现:", found or "没有（确认车已上电、和电脑同一个 Wi-Fi）")
        return 0 if found else 3

    if args.ping:
        car = CarLink(ip=args.ip)
        try:
            print("地址:", car.addr)
            print("PING ->", car.ping())
            print("ID   ->", car.cmd("ID", wait=True))
            print("CHIP ->", car.cmd("CHIP", wait=True))
        finally:
            car.close()
        return 0

    spec = args.go or args.plan
    if not spec:
        ap.print_help()
        return 0

    action, raw = spec
    try:
        amount = float(raw)
    except ValueError:
        print(f"量必须是个数：{raw!r}", file=sys.stderr)
        return 2
    try:
        p = make_plan(action, amount)
    except KeyError as e:
        print(e.args[0], file=sys.stderr)
        return 2

    print(f"{p['action']} {p['amount']}{p['unit']}  →  {p['run']}")
    print(f"  预测 {p['predicted']}{p['unit']}   该组 σ=±{p['sigma']}、n={p['n']}、R²={p['r2']:.4f}")
    for w in p["warnings"]:
        print(f"  ⚠ {w}")
    if args.plan:
        return 0
    return run_on_car(p, args.ip)


if __name__ == "__main__":
    raise SystemExit(main())
