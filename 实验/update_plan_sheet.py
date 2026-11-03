#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只刷新 实验数据记录.xlsx 的「实验清单」表为 experiments.py 当前方案。

不动 data / 历史数据 / 字段说明，不会丢失已采集数据。
"""
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side

import experiments

XLSX = "/home/hanli/麦轮小车pid/实验/实验数据记录.xlsx"

wb = openpyxl.load_workbook(XLSX)

hdr_fill = PatternFill("solid", fgColor="1F4E78")
hdr_font = Font(bold=True, color="FFFFFF")
thin = Side(style="thin", color="BFBFBF")
border = Border(left=thin, right=thin, top=thin, bottom=thin)
center = Alignment(horizontal="center", vertical="center")

ws3 = wb["实验清单"]
ws3.delete_rows(1, ws3.max_row)
hdr3 = ["ID", "原参考", "action_type", "指令描述", "duration_s",
        "vx(m/s)", "vy(m/s)", "om(rad/s)", "重点观测"]
ws3.append(hdr3)
for c in range(1, len(hdr3) + 1):
    cell = ws3.cell(row=1, column=c)
    cell.fill = hdr_fill; cell.font = hdr_font
    cell.alignment = center; cell.border = border
for e in experiments.EXPERIMENTS:
    _id, action, desc, dur, vx, vy, om, purpose, ref = e
    r = ws3.max_row + 1
    for c, v in enumerate([_id, ref, action, desc, dur, vx, vy, om, purpose], start=1):
        cell = ws3.cell(row=r, column=c, value=v)
        cell.border = border
        if c in (5, 6, 7, 8):
            cell.number_format = "0.000"
for col, w in zip("ABCDEFGHI", [6, 9, 16, 20, 11, 10, 10, 11, 24]):
    ws3.column_dimensions[col].width = w
ws3.freeze_panes = "A2"

wb.save(XLSX)
print("实验清单已刷新:", len(experiments.EXPERIMENTS), "条 ->", XLSX)
