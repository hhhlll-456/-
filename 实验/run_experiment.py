#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""执行一次实验并把 100Hz 数据写入 实验数据记录.xlsx。

用法:
    python3 run_experiment.py 1          # 跑实验 1
    python3 run_experiment.py 1 --dry    # 只读清单不驱动
    python3 run_experiment.py --fill 1 --distance 0.27 --angle 3.5
                                         # 事后把人工测量值填入表

流程: 发现小车 -> RUN <vx> <vy> <om> <ms> -> 收 100Hz 'D' 样本直到 DONE -> 写表
"""
import argparse
import math
import socket
import sys
import time

import openpyxl

import experiments
from car_link import CarLink

XLSX = "/home/hanli/麦轮小车pid/实验/实验数据记录.xlsx"
SHEET = "data"

# data 表列序（1-based），与 实验数据记录_build.py 的 COLS 一致
COL = {
    "experiment_id": 1, "action_type": 2, "command_description": 3,
    "command_param_1": 4, "command_param_2": 5, "duration_planned": 6,
    "t1": 7, "timestamp": 8,
    "pwm_fl": 9, "pwm_fr": 10, "pwm_rl": 11, "pwm_rr": 12,
    "encoder_fl": 13, "encoder_fr": 14, "encoder_rl": 15, "encoder_rr": 16,
    "imu_roll": 23, "imu_pitch": 24, "imu_yaw": 25,
    "manual_distance": 26, "manual_angle": 27,
    "manual_final_x": 28, "manual_final_y": 29, "manual_final_yaw": 30,
    "notes": 31,
}


def params_for(e):
    _id, action, desc, dur, vx, vy, om, _purpose = e[:8]
    if action == "rotate_in_place":
        p1 = om * dur * 180.0 / math.pi      # 角度(deg)
        p2 = om                               # rad/s
    else:
        p1 = math.hypot(vx, vy) * dur         # 距离(m)
        p2 = math.hypot(vx, vy)               # m/s
    return p1, p2


def write_rows(exp_id, samples, e):
    _id, action, desc, dur, vx, vy, om, _p = e[:8]
    p1, p2 = params_for(e)
    t1 = samples[-1]["t"] if samples else 0.0

    wb = openpyxl.load_workbook(XLSX)
    ws = wb[SHEET]
    # 删掉该实验已存在的旧行
    for r in range(ws.max_row, 1, -1):
        if ws.cell(row=r, column=COL["experiment_id"]).value == exp_id:
            ws.delete_rows(r)

    for s in samples:
        r = ws.max_row + 1
        ws.cell(row=r, column=COL["experiment_id"], value=exp_id)
        ws.cell(row=r, column=COL["action_type"], value=action)
        ws.cell(row=r, column=COL["command_description"], value=desc)
        ws.cell(row=r, column=COL["command_param_1"], value=round(p1, 4))
        ws.cell(row=r, column=COL["command_param_2"], value=round(p2, 4))
        ws.cell(row=r, column=COL["duration_planned"], value=dur)
        ws.cell(row=r, column=COL["t1"], value=round(t1, 3))
        ws.cell(row=r, column=COL["timestamp"], value=s["t"])
        ws.cell(row=r, column=COL["pwm_fl"], value=s["pwm"][0])
        ws.cell(row=r, column=COL["pwm_fr"], value=s["pwm"][1])
        ws.cell(row=r, column=COL["pwm_rl"], value=s["pwm"][2])
        ws.cell(row=r, column=COL["pwm_rr"], value=s["pwm"][3])
        ws.cell(row=r, column=COL["encoder_fl"], value=s["enc"][0])
        ws.cell(row=r, column=COL["encoder_fr"], value=s["enc"][1])
        ws.cell(row=r, column=COL["encoder_rl"], value=s["enc"][2])
        ws.cell(row=r, column=COL["encoder_rr"], value=s["enc"][3])
        ws.cell(row=r, column=COL["imu_roll"], value=s["rpy"][0])
        ws.cell(row=r, column=COL["imu_pitch"], value=s["rpy"][1])
        ws.cell(row=r, column=COL["imu_yaw"], value=s["rpy"][2])
    wb.save(XLSX)
    return t1


def parse_D(line):
    # D t p0 p1 p2 p3 e0 e1 e2 e3 roll pitch yaw
    p = line.split()
    if len(p) < 13 or p[0] != "D":
        return None
    return {
        "t": int(p[1]) / 1000.0,
        "pwm": [int(p[2]), int(p[3]), int(p[4]), int(p[5])],
        "enc": [int(p[6]), int(p[7]), int(p[8]), int(p[9])],
        "rpy": [float(p[10]), float(p[11]), float(p[12])],
    }


def run(exp_id, tail_s=0.4):
    e = experiments.get(exp_id)
    _id, action, desc, dur, vx, vy, om, purpose = e[:8]
    ms = int(round(dur * 1000))
    print(f"实验 {exp_id}: {desc} | {action} | dur={dur}s | vx={vx} vy={vy} om={om}")
    print(f"目的: {purpose}")

    with CarLink() as car:
        car.cmd("STREAM 0")
        # 清空积压
        car.sock.settimeout(0.4)
        try:
            while car.sock.recv(8192):
                pass
        except socket.timeout:
            pass

        car.cmd(f"RUN {vx} {vy} {om} {ms}")
        car.sock.settimeout(dur + 3)
        buf = b""
        t_end = time.time() + dur + 3
        samples = []
        ok_line = None
        while time.time() < t_end:
            try:
                chunk = car.sock.recv(8192)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                s = ln.decode("utf-8", "replace").strip()
                if s.startswith("OK RUN"):
                    ok_line = s
                elif s == "DONE":
                    t_end = 0
                elif s.startswith("D "):
                    d = parse_D(s)
                    if d and d["t"] <= dur + 0.3:   # 防呆：只收本次运行时间窗内
                        samples.append(d)
            if t_end == 0:
                break
        print("小车回复:", ok_line)
        car.cmd("STOP")

    if not samples:
        print("没收到样本，检查链路/固件")
        return None

    t1 = write_rows(exp_id, samples, e)
    print(f"已写入 {len(samples)} 行 (t1={t1}s) -> {XLSX}")
    last = samples[-1]
    print(f"末样本: pwm={last['pwm']} enc={last['enc']} rpy={last['rpy']}")
    print()
    print(">>> 请手动测量并告诉我（我填表）：")
    print("    - 实际位移距离 manual_distance (m)")
    print("    - 实际转角 manual_angle (deg)")
    print("    - 可选：final_x, final_y, final_yaw")
    return {"samples": len(samples), "t1": t1, "last": last}


def fill_manual(exp_id, distance=None, angle=None, fx=None, fy=None, fyaw=None, notes=None):
    wb = openpyxl.load_workbook(XLSX)
    ws = wb[SHEET]
    n = 0
    for r in range(2, ws.max_row + 1):
        if ws.cell(row=r, column=COL["experiment_id"]).value == exp_id:
            if distance is not None:
                ws.cell(row=r, column=COL["manual_distance"], value=distance)
            if angle is not None:
                ws.cell(row=r, column=COL["manual_angle"], value=angle)
            if fx is not None:
                ws.cell(row=r, column=COL["manual_final_x"], value=fx)
            if fy is not None:
                ws.cell(row=r, column=COL["manual_final_y"], value=fy)
            if fyaw is not None:
                ws.cell(row=r, column=COL["manual_final_yaw"], value=fyaw)
            if notes is not None:
                ws.cell(row=r, column=COL["notes"], value=notes)
            n += 1
    wb.save(XLSX)
    print(f"实验 {exp_id}: 已给 {n} 行填入人工测量值")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("exp_id", nargs="?", type=int)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--fill", type=int)
    ap.add_argument("--distance", type=float)
    ap.add_argument("--angle", type=float)
    ap.add_argument("--fx", type=float); ap.add_argument("--fy", type=float)
    ap.add_argument("--fyaw", type=float); ap.add_argument("--notes")
    a = ap.parse_args()

    if a.fill is not None:
        fill_manual(a.fill, a.distance, a.angle, a.fx, a.fy, a.fyaw, a.notes)
        return
    if a.exp_id is None:
        print(__doc__)
        return
    e = experiments.get(a.exp_id)
    if a.dry:
        print("清单:", e)
        return
    run(a.exp_id)


if __name__ == "__main__":
    main()
