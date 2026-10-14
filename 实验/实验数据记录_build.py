#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 / 就地升级「实验数据记录.xlsx」（表头 + 字段说明 + 实验清单）。

    python3 实验数据记录_build.py [输出路径.xlsx]

两种模式自动选择：

- **新建**：目标文件不存在 → 从零生成。`data` 表只有表头（45 列 = 既有 31 列，
  Task 12 在末尾追加 14 个视觉/对比列），数据交给 `vgt.compare.write_trial` 追加。
- **就地升级**：目标文件已存在 → **保留全部数据行**（含「历史数据」表，一行不丢），
  把 `data` 表头补齐到 `COLS`，重建「字段说明」与「实验清单」。

为什么是"升级"而不是"重建"：本脚本以前的做法是"已有数据就拒绝重建"（防误覆盖），
而 Task 12 加列时若直接重建会把既有数据行整体清空（SDD 台账 R16 记过这个静默毁表
风险）。就地升级保持"xlsx 只能由生成器产出、不直接改表"的既有约定，同时把重建的
破坏性换成只加表头。

安全闸门：升级前逐字校验既有表头必须是 `COLS` 的前缀（只许在末尾加列，不许改已有
列名/列序）——对不上就报错退出，防"把新行贴上错位表头"这种静默毁表。

表内**只有值、没有公式**（本机没有 LibreOffice 的既有约束；公式不会有人算）。
"""
from __future__ import annotations

import sys
from pathlib import Path

import openpyxl
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "实验数据记录.xlsx"

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
    # ---- Task 12：视觉 vs 里程计对比（14 列，接在既有 31 列之后）--------------------
    ("vision_start_x",   "float",  "mm",  "视觉起点 X（场地系；原点 = 四边界码质心，R4）"),
    ("vision_start_y",   "float",  "mm",  "视觉起点 Y（场地系）"),
    ("vision_start_yaw", "float",  "deg", "视觉起点航向（场地系 +X=0°，俯视逆时针为正，R34）"),
    ("vision_end_x",     "float",  "mm",  "视觉终点 X（场地系）"),
    ("vision_end_y",     "float",  "mm",  "视觉终点 Y（场地系）"),
    ("vision_end_yaw",   "float",  "deg", "视觉终点航向（同上口径）"),
    ("vision_dist_mm",   "float",  "mm",  "视觉净位移 = |终点−起点|"),
    ("vision_yaw_deg",   "float",  "deg", "视觉净转角 = 终点yaw−起点yaw（归一到 (−180,180]）"),
    ("odo_dist_mm",      "float",  "mm",  "编码器里程计净位移（pwm 定方向，R13；同一 trial 积分）"),
    ("odo_yaw_deg",      "float",  "deg", "编码器里程计净转角（俯视逆时针为正）"),
    ("dist_diff_pct",    "float",  "%",   "位移相对差 =（里程计−视觉）/ 视觉 ×100（视觉≈0 时留空）"),
    ("yaw_diff_deg",     "float",  "deg", "转角差 = 视觉−里程计（归一到 (−180,180]）"),
    ("trial_ok",         "bool",   "-",   "本条是否判通过（含 dot(视觉,里程计)>0 有符号判据；仅汇总行有值）"),
    ("trial_reason",     "string", "-",   "trial_ok=False 的原因（与 harness.TRIAL_REASONS 同词表；仅汇总行有值）"),
]

# Task 12 的 14 个新列名（与 `实验/vgt/compare.py` 的 `VISION_COLS` 逐字一致；
# 生成器不 import vgt，两份名单由 test_compare.py 钉死）。
VISION_COLS = [name for name, *_r in COLS][-14:]

HDR_FILL = PatternFill("solid", fgColor="1F4E78")
HDR_FONT = Font(bold=True, color="FFFFFF")
MANUAL_FILL = PatternFill("solid", fgColor="C6E0B4")     # 既有"实验后填写"列
VISION_FILL = PatternFill("solid", fgColor="DDEBF7")     # Task 12 的对比列
THIN = Side(style="thin", color="BFBFBF")
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)
CENTER = Alignment(horizontal="center", vertical="center")

WIDTHS = {
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
    "vision_start_x": 14, "vision_start_y": 14, "vision_start_yaw": 15,
    "vision_end_x": 14, "vision_end_y": 14, "vision_end_yaw": 15,
    "vision_dist_mm": 14, "vision_yaw_deg": 14,
    "odo_dist_mm": 13, "odo_yaw_deg": 13,
    "dist_diff_pct": 13, "yaw_diff_deg": 13,
    "trial_ok": 10, "trial_reason": 24,
}


def _col_names():
    return [name for name, *_r in COLS]


def _style_header_cell(cell, fill):
    cell.fill = fill
    cell.font = HDR_FONT
    cell.alignment = CENTER
    cell.border = BORDER


def _write_data_header(ws):
    """`data` 表头 = COLS（45 列）；第 26–31 列绿底（人工填写）、32+ 列蓝底（Task 12）。"""
    for c, (name, *_r) in enumerate(COLS, start=1):
        if 26 <= c <= 31:
            fill = MANUAL_FILL
        elif c > 31:
            fill = VISION_FILL
        else:
            fill = HDR_FILL
        _style_header_cell(ws.cell(row=1, column=c, value=name), fill)
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = f"A1:{openpyxl.utils.get_column_letter(len(COLS))}1"
    for c, (name, *_r) in enumerate(COLS, start=1):
        ws.column_dimensions[openpyxl.utils.get_column_letter(c)].width = WIDTHS.get(name, 14)


def _reset_sheet(wb, title):
    """取出同名表并清空全部内容（重建派生表用）；没有就新建。"""
    if title in wb.sheetnames:
        ws = wb[title]
        ws.delete_rows(1, max(ws.max_row, 1))
        return ws
    return wb.create_sheet(title)


def _write_field_help(wb):
    ws2 = _reset_sheet(wb, "字段说明")
    for c, title in enumerate(["列名", "类型", "单位", "说明"], start=1):
        _style_header_cell(ws2.cell(row=1, column=c, value=title), HDR_FILL)
    for r, (name, t, u, d) in enumerate(COLS, start=2):
        ws2.cell(row=r, column=1, value=name).border = BORDER
        ws2.cell(row=r, column=2, value=t).border = BORDER
        ws2.cell(row=r, column=3, value=u).border = BORDER
        ws2.cell(row=r, column=4, value=d).border = BORDER
    for col, w in zip("ABCD", [26, 10, 10, 52]):
        ws2.column_dimensions[col].width = w
    ws2.freeze_panes = "A2"


def _write_experiment_list(wb):
    """「实验清单」= 旧 22 条方案（`实验/experiments.py`，历史遗留，原样重建）。"""
    import experiments                      # 同目录；执行脚本时 HERE 在 sys.path 上

    ws3 = _reset_sheet(wb, "实验清单")
    hdr3 = ["ID", "原参考", "action_type", "指令描述", "duration_s",
            "vx(m/s)", "vy(m/s)", "om(rad/s)", "重点观测"]
    for c, t in enumerate(hdr3, start=1):
        _style_header_cell(ws3.cell(row=1, column=c, value=t), HDR_FILL)
    for r, e in enumerate(experiments.EXPERIMENTS, start=2):
        _id, action, desc, dur, vx, vy, om, purpose, ref = e
        for c, v in enumerate([_id, ref, action, desc, dur, vx, vy, om, purpose], start=1):
            cell = ws3.cell(row=r, column=c, value=v)
            cell.border = BORDER
            if c in (5, 6, 7, 8):
                cell.number_format = "0.000"
    for col, w in zip("ABCDEFGHI", [6, 9, 16, 20, 11, 10, 10, 11, 24]):
        ws3.column_dimensions[col].width = w
    ws3.freeze_panes = "A2"


def _validate_prefix(ws):
    """既有表头必须是 COLS 的前缀：只加列、绝不改已有列名/列序。"""
    existing = [cell.value for cell in ws[1] if cell.value is not None]
    expected = _col_names()[:len(existing)]
    if existing != expected:
        raise RuntimeError(
            "拒绝升级 data 表头：既有列名不是 COLS 的前缀（本脚本只允许在末尾加列）。\n"
            f"  既有：{existing}\n"
            f"  预期：{expected}")


def build_workbook():
    """新建工作簿（data / 字段说明 / 实验清单，无数据行）。"""
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "data"
    _write_data_header(ws)
    _write_field_help(wb)
    _write_experiment_list(wb)
    return wb


def upgrade_workbook(wb):
    """就地升级：保留 data / 历史数据 的数据行，只补表头并重建派生表。"""
    if "data" not in wb.sheetnames:
        raise RuntimeError("工作簿里没有 data 表 —— 不是本项目的实验数据表，拒绝升级。")
    _validate_prefix(wb["data"])
    _write_data_header(wb["data"])
    _write_field_help(wb)
    _write_experiment_list(wb)
    return wb


def write_workbook(out_path=DEFAULT_OUT) -> str:
    """存在 → 就地升级（保留数据行）；不存在 → 新建。返回输出路径字符串。"""
    out = Path(out_path)
    if out.exists():
        wb = openpyxl.load_workbook(out)
        if "data" not in wb.sheetnames:
            raise RuntimeError(f"{out} 不是本项目的实验数据表（没有 data 表）。")
        n_before = wb["data"].max_row - 1
        upgrade_workbook(wb)
        wb.save(out)
        print(f"upgraded: {out} | data 保留 {n_before} 行数据 | columns: {len(COLS)}"
              f"（末尾 14 列 = Task 12 对比列）")
    else:
        wb = build_workbook()
        wb.save(out)
        print(f"saved: {out} | columns: {len(COLS)}（末尾 14 列 = Task 12 对比列）")
    return str(out)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    out = Path(argv[0]) if argv else DEFAULT_OUT
    write_workbook(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
