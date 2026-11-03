#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次性迁移：把现有 实验数据记录.xlsx 对齐到新的 22 条实验方案。

- data 表：已测数据重编号对齐新ID（原1..4→1..4, 原6→6, 原7→7, 原9→8）
- 新方案未引用的已测数据（原5/8/10/11/12）移到「历史数据」表
- 「实验清单」表重建为新 22 条方案
数据（PWM/编码器/IMU/manual_*）原样保留，不丢失。
"""
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side

import experiments

XLSX = "/home/hanli/麦轮小车pid/实验/实验数据记录.xlsx"

REMAP = {1: 1, 2: 2, 3: 3, 4: 4, 6: 6, 7: 7, 9: 8}   # 原ID -> 新ID

wb = openpyxl.load_workbook(XLSX)

# ---------- data 表：重编号 + 分流 ----------
ws = wb["data"]
hdr = [c.value for c in ws[1]]
maxr = ws.max_row
rows = [list(r) for r in ws.iter_rows(min_row=2, values_only=True)]

kept, hist = [], []
for r in rows:
    eid = r[0]
    if eid is None:
        continue
    if eid in REMAP:
        rr = list(r)
        rr[0] = REMAP[eid]
        kept.append(rr)
    else:
        hist.append(list(r))

if maxr >= 2:
    ws.delete_rows(2, maxr - 1)
for rr in kept:
    ws.append(rr)

# ---------- 历史数据 表 ----------
if "历史数据" in wb.sheetnames:
    del wb["历史数据"]
wh = wb.create_sheet("历史数据")
wh.append(hdr)
for rr in hist:
    wh.append(rr)
wh.freeze_panes = "A2"

# ---------- 实验清单 表：重建为新方案 ----------
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

print("data 保留行:", len(kept), "| 新ID:", sorted(set(r[0] for r in kept)))
print("历史数据行:", len(hist), "| 原ID:", sorted(set(r[0] for r in hist)))
print("实验清单条数:", len(experiments.EXPERIMENTS), "->", XLSX)
