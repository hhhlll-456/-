#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成实验数据记录 Excel（表头 + 字段说明）。数据表保持干净，便于后续逐行追加。"""
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side

# (列名, 类型, 单位, 说明)
COLS = [
    ("experiment_id",      "int",    "-",      "实验编号 1～50"),
    ("action_type",        "string", "-",      "straight / rotate_in_place / strafe / diagonal / composite"),
    ("command_description","string", "-",      "高层指令原文，例如「前进0.40m」「逆时针90度」"),
    ("command_param_1",    "float",  "视情况",  "主要参数1（距离或角度）"),
    ("command_param_2",    "float",  "视情况",  "主要参数2（速度等，可选）"),
    ("duration_planned",   "float",  "s",      "计划执行时长"),
    ("t1",                 "float",  "s",      "实际结束时间（相对0）"),
    ("timestamp",          "float",  "s",      "当前采样点相对时间（0.00, 0.01, 0.02…）"),
    ("pwm_fl",             "int",    "-",      "前左轮实际下发的 PWM"),
    ("pwm_fr",             "int",    "-",      "前右轮实际下发的 PWM"),
    ("pwm_rl",             "int",    "-",      "后左轮实际下发的 PWM"),
    ("pwm_rr",             "int",    "-",      "后右轮实际下发的 PWM"),
    ("encoder_fl",         "int",    "pulse",  "前左轮累计原始脉冲"),
    ("encoder_fr",         "int",    "pulse",  "前右轮累计原始脉冲"),
    ("encoder_rl",         "int",    "pulse",  "后左轮累计原始脉冲"),
    ("encoder_rr",         "int",    "pulse",  "后右轮累计原始脉冲"),
    ("imu_ax",             "float",  "m/s²",   "IMU 加速度 X（GY-25T 不提供，留空）"),
    ("imu_ay",             "float",  "m/s²",   "IMU 加速度 Y（GY-25T 不提供，留空）"),
    ("imu_az",             "float",  "m/s²",   "IMU 加速度 Z（GY-25T 不提供，留空）"),
    ("imu_gx",             "float",  "rad/s",  "IMU 角速度 X（GY-25T 不提供，留空）"),
    ("imu_gy",             "float",  "rad/s",  "IMU 角速度 Y（GY-25T 不提供，留空）"),
    ("imu_gz",             "float",  "rad/s",  "IMU 角速度 Z（GY-25T 不提供，留空）"),
    ("imu_roll",           "float",  "deg",    "IMU 横滚角 roll（GY-25T）"),
    ("imu_pitch",          "float",  "deg",    "IMU 俯仰角 pitch（GY-25T）"),
    ("imu_yaw",            "float",  "deg",    "IMU 航向角 yaw（GY-25T）"),
    ("manual_distance",    "float",  "m",      "尺子测得的实际位移距离（实验后填写）"),
    ("manual_angle",       "float",  "deg",    "量角器测得的实际转角（实验后填写）"),
    ("manual_final_x",     "float",  "m",      "最终 X 位置（可选）"),
    ("manual_final_y",     "float",  "m",      "最终 Y 位置（可选）"),
    ("manual_final_yaw",   "float",  "deg",    "最终航向角（可选）"),
    ("notes",              "string", "-",      "备注（打滑、异常等）"),
]

wb = openpyxl.Workbook()

# ---------- Sheet 1: data ----------
ws = wb.active
ws.title = "data"

hdr_fill = PatternFill("solid", fgColor="1F4E78")
hdr_font = Font(bold=True, color="FFFFFF")
thin = Side(style="thin", color="BFBFBF")
border = Border(left=thin, right=thin, top=thin, bottom=thin)
center = Alignment(horizontal="center", vertical="center")

for c, (name, _t, _u, _d) in enumerate(COLS, start=1):
    cell = ws.cell(row=1, column=c, value=name)
    cell.fill = hdr_fill
    cell.font = hdr_font
    cell.alignment = center
    cell.border = border

# 手动测量列（后 5 列前 1 列 manual_distance + notes）用不同底色提示
manual_fill = PatternFill("solid", fgColor="C6E0B4")
for c in range(26, 32):  # manual_distance ... notes（实验后填写列）
    ws.cell(row=1, column=c).fill = manual_fill

ws.freeze_panes = "A2"          # 冻结表头
ws.auto_filter.ref = f"A1:{openpyxl.utils.get_column_letter(len(COLS))}1"

# 列宽
widths = {
    "experiment_id": 14, "action_type": 16, "command_description": 26,
    "command_param_1": 15, "command_param_2": 15, "duration_planned": 15,
    "t1": 10, "timestamp": 12,
    "pwm_fl": 9, "pwm_fr": 9, "pwm_rl": 9, "pwm_rr": 9,
    "encoder_fl": 12, "encoder_fr": 12, "encoder_rl": 12, "encoder_rr": 12,
    "imu_ax": 10, "imu_ay": 10, "imu_az": 10,
    "imu_gx": 10, "imu_gy": 10, "imu_gz": 10,
    "imu_roll": 10, "imu_pitch": 10, "imu_yaw": 10,
    "manual_distance": 15, "manual_angle": 15,
    "manual_final_x": 14, "manual_final_y": 14, "manual_final_yaw": 15,
    "notes": 30,
}
for c, (name, *_r) in enumerate(COLS, start=1):
    ws.column_dimensions[openpyxl.utils.get_column_letter(c)].width = widths.get(name, 14)

# ---------- Sheet 2: 字段说明 ----------
ws2 = wb.create_sheet("字段说明")
for c, title in enumerate(["列名", "类型", "单位", "说明"], start=1):
    cell = ws2.cell(row=1, column=c, value=title)
    cell.fill = hdr_fill
    cell.font = hdr_font
    cell.alignment = center
    cell.border = border
for r, (name, t, u, d) in enumerate(COLS, start=2):
    ws2.cell(row=r, column=1, value=name).border = border
    ws2.cell(row=r, column=2, value=t).border = border
    ws2.cell(row=r, column=3, value=u).border = border
    ws2.cell(row=r, column=4, value=d).border = border
for col, w in zip("ABCD", [26, 10, 10, 52]):
    ws2.column_dimensions[col].width = w
ws2.freeze_panes = "A2"

# ---------- Sheet 3: 实验清单 (新 22 条方案) ----------
import experiments
ws3 = wb.create_sheet("实验清单")
hdr3 = ["ID", "原参考", "action_type", "指令描述", "duration_s",
        "vx(m/s)", "vy(m/s)", "om(rad/s)", "重点观测"]
for c, t in enumerate(hdr3, start=1):
    cell = ws3.cell(row=1, column=c, value=t)
    cell.fill = hdr_fill; cell.font = hdr_font; cell.alignment = center; cell.border = border
for r, e in enumerate(experiments.EXPERIMENTS, start=2):
    _id, action, desc, dur, vx, vy, om, purpose, ref = e
    for c, v in enumerate([_id, ref, action, desc, dur, vx, vy, om, purpose], start=1):
        cell = ws3.cell(row=r, column=c, value=v)
        cell.border = border
        if c in (5, 6, 7, 8):
            cell.number_format = "0.000"
for col, w in zip("ABCDEFGHI", [6, 9, 16, 20, 11, 10, 10, 11, 24]):
    ws3.column_dimensions[col].width = w
ws3.freeze_panes = "A2"

out = "/home/hanli/麦轮小车pid/实验/实验数据记录.xlsx"

# 防误覆盖：已有采集数据时拒绝重建（重建会清空 data 表）
import os
if os.path.exists(out):
    try:
        _wb = openpyxl.load_workbook(out)
        if "data" in _wb.sheetnames and _wb["data"].max_row > 1:
            print(f"拒绝覆盖：{out} 已有 {_wb['data'].max_row - 1} 行数据。"
                  "如确需重建，请先备份并手动删除该文件。")
            raise SystemExit(1)
    except SystemExit:
        raise
    except Exception:
        pass

wb.save(out)
print("saved:", out, "| columns:", len(COLS))
