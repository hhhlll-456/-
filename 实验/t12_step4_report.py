#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 12 Step 4 真机全量 48 条的**离线汇总**（只读 xlsx，不写表、不连车）。

    cd 实验
    python3 t12_step4_report.py                # 全部汇总行
    python3 t12_step4_report.py --from-row 24  # 只统计第 24 条汇总行起（= 本轮全量）

## 口径（写清楚，别让"统计"变成"挑数"）

- 只看 `data` 表的**汇总行**（`notes` 含「汇总行」），样本行一律不进统计。
- 行号是 `data` 表的物理行号；`--from-row` 指的是**第几条汇总行**（1 起），不是表行号。
- 上一轮的真机行与本轮的行**在表里混着**：上一轮共 23 条汇总行（1–17 预检、
  18–20 冒烟、21–23 死锁，见 commit d9b888c），本轮从第 24 条起。
- 失败条目（`trial_ok=False`）也统计，但**不进 dist/yaw 分布**（没有有效对比值），
  单独按 `trial_reason` 分类计数。
- 不做任何筛除：p25/p75/中位/极值全部按实际拿到的值算，缺就写缺。
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
import sys
from pathlib import Path

import openpyxl

from vgt import matrix

DEFAULT_XLSX = Path(__file__).resolve().parent / "实验数据记录.xlsx"
SHEET = "data"

# 上一轮留下的汇总行条数（见 commit d9b888c 的说明），本轮从第 24 条起。
PREV_ROUND_SUMMARY_ROWS = 23

# 三条轴的"低速区"分组：命令量小到被固件 MIN_DUTY=110 钳位的那些腿。
LOW_SPEED_GROUPS = {
    "straight": "直行（150/200 mm）",
    "strafe": "横移（50/80 mm）",
    "diagonal": "斜行（120 mm）",
    "rotate_in_place": "旋转（30/45°）",
}


def load_summary_rows(xlsx_path=DEFAULT_XLSX):
    """`data` 表的全部汇总行 → `[{"n": 第几条汇总行, "excel_row": 表行号, ...列}]`。"""
    wb = openpyxl.load_workbook(xlsx_path, read_only=True, data_only=True)
    ws = wb[SHEET]
    rows = list(ws.iter_rows(values_only=True))
    wb.close()
    header = [str(c) for c in rows[0]]
    idx = {name: i for i, name in enumerate(header)}
    out = []
    for excel_row, raw in enumerate(rows[1:], start=2):
        notes = raw[idx["notes"]]
        if not notes or "汇总行" not in str(notes):
            continue
        rec = {"n": len(out) + 1, "excel_row": excel_row, "notes": str(notes)}
        for key in ("experiment_id", "action_type", "command_description",
                    "vision_dist_mm", "vision_yaw_deg", "odo_dist_mm", "odo_yaw_deg",
                    "dist_diff_pct", "yaw_diff_deg", "trial_ok", "trial_reason",
                    "vision_start_x", "vision_start_y", "vision_start_yaw",
                    "vision_end_x", "vision_end_y", "vision_end_yaw"):
            rec[key] = raw[idx[key]]
        out.append(rec)
    return out


def describe(values):
    """一组样本 → 中位 / p25 / p75 / 极值 / n（四分位用线性插值）。"""
    vals = sorted(float(v) for v in values if v is not None)
    if not vals:
        return None

    def pct(p):
        if len(vals) == 1:
            return vals[0]
        pos = (len(vals) - 1) * p / 100.0
        lo = int(math.floor(pos))
        hi = min(lo + 1, len(vals) - 1)
        return vals[lo] + (vals[hi] - vals[lo]) * (pos - lo)

    return {"n": len(vals), "median": statistics.median(vals),
            "p25": pct(25), "p75": pct(75),
            "min": vals[0], "max": vals[-1]}


def fmt(stat, unit="", digits=2):
    if stat is None:
        return "（无有效样本）"
    return (f"中位 {stat['median']:+.{digits}f}{unit} / "
            f"p25 {stat['p25']:+.{digits}f} / p75 {stat['p75']:+.{digits}f} / "
            f"极值 {stat['min']:+.{digits}f}…{stat['max']:+.{digits}f}（n={stat['n']}）")


def nominal_from_label(desc):
    """从腿的标签文本里取**标称**行程/角度（`"横移左移 80 mm"` → 80.0 mm）。

    ⚠ 刻意**不用** `matrix.expected_delta_mm`：那是 R68/R72 按实测速度模型反解的
    "真机会走多少"（横移腿已经被压到 32.8 mm），拿它当分母算出来的"达成率"
    恒等于 1 附近，把"命令量没兑现"这件事正好抹掉。上一轮的基线口径也是**标签**
    （"#05 标称 80 mm 实测 28–32 mm"），两边才对得上。
    ⚠ 标签里有两个数时（`"斜行前左 45° 120 mm"` = 角度 + 行程）取 **mm** 那个；
      旋转腿（`"原地逆时针 30°"`）没有 mm，取 °。
    """
    found = re.findall(r"(\d+(?:\.\d+)?)\s*(mm|°)", str(desc))
    for value, unit in found:
        if unit == "mm":
            return float(value), unit
    return (float(found[0][0]), found[0][1]) if found else (None, None)


def nominal_travel_mm(exp_id):
    """矩阵里这条腿的标称行程（取自标签文本）。"""
    for exp in matrix.EXPERIMENTS:
        if int(exp.id) == int(exp_id):
            value, unit = nominal_from_label(exp.desc)
            return (value if unit == "mm" else None), str(exp.action), exp.desc
    return None, None, None


def nominal_yaw_deg(exp_id):
    for exp in matrix.EXPERIMENTS:
        if int(exp.id) == int(exp_id):
            value, unit = nominal_from_label(exp.desc)
            return (value if unit == "°" else None), exp.desc
    return None, None


def main(argv=None):
    ap = argparse.ArgumentParser(description="Task 12 Step 4 汇总（只读）")
    ap.add_argument("--from-row", type=int, default=PREV_ROUND_SUMMARY_ROWS + 1,
                    help=f"从第几条汇总行开始统计（默认 {PREV_ROUND_SUMMARY_ROWS + 1} = 本轮）")
    ap.add_argument("--xlsx", default=str(DEFAULT_XLSX))
    args = ap.parse_args(argv)

    rows = load_summary_rows(args.xlsx)
    if not rows:
        print(f"{args.xlsx} 的 {SHEET} 表里没有汇总行。")
        return 2
    scope = [r for r in rows if r["n"] >= args.from_row]
    prev = [r for r in rows if r["n"] < args.from_row]
    out = print

    out(f"表：{args.xlsx}")
    out(f"汇总行总数 {len(rows)}（上一轮 {len(prev)} 条 + 本轮 {len(scope)} 条，"
        f"本轮口径 = 第 {args.from_row} 条起）")

    ok = [r for r in scope if r["trial_ok"] is True]
    bad = [r for r in scope if r["trial_ok"] is not True]

    # ---- 1. 跑通 / 失败，按 trial_reason 分类 ----
    out("\n=== 1. 跑通 / 失败（按 trial_reason）===")
    out(f"本轮共 {len(scope)} 条：跑通 {len(ok)} 条，失败 {len(bad)} 条。")
    if prev:
        pre_ok = [r for r in prev if r["trial_ok"] is True]
        out(f"（上一轮同口径 {len(prev)} 条：跑通 {len(pre_ok)} 条，失败 "
            f"{len(prev) - len(pre_ok)} 条）")
    by_reason = {}
    for r in bad:
        by_reason.setdefault(str(r["trial_reason"] or "unknown"), []).append(r)
    if by_reason:
        out("  失败原因分布：")
        for reason, group in sorted(by_reason.items(), key=lambda kv: -len(kv[1])):
            ids = ",".join(f"#{int(r['experiment_id'])}(行{r['n']})" for r in group)
            out(f"    {reason:26s} × {len(group):2d}   {ids}")
    else:
        out("  （无失败条目）")

    # ---- 2. dist_diff_pct / yaw_diff_deg 分布 ----
    out("\n=== 2. 误差分布（只统计跑通的条目）===")
    dd = describe([r["dist_diff_pct"] for r in ok])
    yd = describe([r["yaw_diff_deg"] for r in ok])
    out(f"  dist_diff_pct : {fmt(dd, '%')}")
    out(f"  yaw_diff_deg  : {fmt(yd, '°')}")
    over = [r for r in ok if r["dist_diff_pct"] is not None
            and abs(float(r["dist_diff_pct"])) > 20.0]
    out(f"  超 ±20% 的条数：{len(over)}/{len(ok)}"
        + ("　→ " + ",".join(f"#{int(r['experiment_id'])}:{r['dist_diff_pct']:+.1f}%"
                             for r in over) if over else ""))
    if prev:
        prev_ok = [r for r in prev if r["trial_ok"] is True]
        out(f"  上一轮基线（{len(prev_ok)} 条有效样本）：")
        out(f"    dist_diff_pct : {fmt(describe([r['dist_diff_pct'] for r in prev_ok]), '%')}")
        out(f"    yaw_diff_deg  : {fmt(describe([r['yaw_diff_deg'] for r in prev_ok]), '°')}")
        p_over = [r for r in prev_ok if r["dist_diff_pct"] is not None
                  and abs(float(r["dist_diff_pct"])) > 20.0]
        out(f"    超 ±20%：{len(p_over)}/{len(prev_ok)}")

    # ---- 3. 有符号判据 ----
    out("\n=== 3. 有符号判据（sign_mismatch）===")
    sm = [r for r in scope if r["trial_reason"] == "sign_mismatch"]
    out(f"  判为 sign_mismatch 的条数：{len(sm)}/{len(scope)}"
        + ("　→ " + ",".join(f"#{int(r['experiment_id'])}" for r in sm) if sm else ""))
    pre_sm = [r for r in prev if r["trial_reason"] == "sign_mismatch"]
    out(f"  上一轮：{len(pre_sm)}/{len(prev)}")

    # ---- 4. 低速区（MIN_DUTY 钳位）：三条轴分开 ----
    out("\n=== 4. 低速区：分轴达成率（**不进总体结论**）===")
    out("  达成率 = vision_dist_mm / 矩阵标称行程；<100% 即命令量没兑现。")
    groups = {}
    for r in ok:
        groups.setdefault(str(r["action_type"]), []).append(r)
    for action, label in LOW_SPEED_GROUPS.items():
        group = groups.get(action)
        if not group:
            out(f"  {label:20s} 本轮无有效样本")
            continue
        if action == "rotate_in_place":
            ratios = []
            for r in group:
                nominal, _ = nominal_yaw_deg(r["experiment_id"])
                if nominal and r["vision_yaw_deg"] is not None:
                    ratios.append(abs(float(r["vision_yaw_deg"])) / nominal * 100.0)
            stat = describe(ratios)
            if stat is None:
                out(f"  {label:20s} n= 0（无可用转角样本）")
            else:
                out(f"  {label:20s} n={stat['n']:2d}  转角达成率 "
                    f"{fmt(stat, '%', digits=1)}（|Δyaw| / 标称角）")
            continue
        ratios = []
        for r in group:
            nominal, _, _ = nominal_travel_mm(r["experiment_id"])
            if nominal and r["vision_dist_mm"] is not None and nominal > 0:
                ratios.append(float(r["vision_dist_mm"]) / nominal * 100.0)
        stat = describe(ratios)
        if stat is None:
            out(f"  {label:20s} n= 0（无可用位移样本）")
            continue
        out(f"  {label:20s} n={stat['n']:2d}  位移达成率 {fmt(stat, '%', digits=1)}")
        for r in group:
            nominal, _, desc = nominal_travel_mm(r["experiment_id"])
            if nominal and r["vision_dist_mm"] is not None:
                out(f"      #{int(r['experiment_id']):02d} {desc:16s} "
                    f"标称 {nominal:6.1f} → 实测 {float(r['vision_dist_mm']):6.1f} mm "
                    f"= {float(r['vision_dist_mm']) / nominal * 100.0:5.1f}%")

    # 旋转腿另给转角达成率（标称是角，不是位移）
    rot = groups.get("rotate_in_place") or []
    for r in rot:
        nominal, desc = nominal_yaw_deg(r["experiment_id"])
        v = r["vision_yaw_deg"]
        if nominal and v is not None:
            out(f"      #{int(r['experiment_id']):02d} {desc:16s} "
                f"标称 {nominal:5.1f}° → 实测 {float(v):+6.2f}° "
                f"= {abs(float(v)) / nominal * 100.0:5.1f}%")

    # ---- 5. 逐条明细 ----
    out("\n=== 5. 本轮逐条明细 ===")
    out("  行  #  描述                 标称     vision    odo     Δ%      Δyaw°   判定")
    for r in scope:
        exp_id = int(r["experiment_id"])
        nominal, action, desc = nominal_travel_mm(exp_id)
        if action == "rotate_in_place":
            nominal, desc = nominal_yaw_deg(exp_id)
            nominal_txt = f"{nominal:5.1f}°" if nominal else "   -  "
            v = r["vision_yaw_deg"]
            vtxt = f"{float(v):6.2f}°" if v is not None else "     - "
        else:
            nominal_txt = f"{nominal:6.1f}" if nominal else "     -"
            v = r["vision_dist_mm"]
            vtxt = f"{float(v):7.1f}" if v is not None else "      -"
        o = r["odo_dist_mm"]
        otxt = f"{float(o):7.1f}" if o is not None else "      -"
        d = r["dist_diff_pct"]
        dtxt = f"{float(d):+6.2f}" if d is not None else "     -"
        y = r["yaw_diff_deg"]
        ytxt = f"{float(y):+7.2f}" if y is not None else "      -"
        verdict = "OK" if r["trial_ok"] else f"FAIL {r['trial_reason']}"
        out(f"  {r['n']:3d} #{exp_id:02d} {desc:18s} {nominal_txt} {vtxt} {otxt} "
            f"{dtxt} {ytxt}  {verdict}")

    # ---- 6. 静默假数据自查 ----
    out("\n=== 6. 静默假数据自查 ===")
    suspicious = []
    for r in ok:
        v, o = r["vision_dist_mm"], r["odo_dist_mm"]
        if v is None or o is None:
            suspicious.append((r, "缺 vision/odo 对比值却判 OK"))
            continue
        if float(v) == 0.0 and float(o) == 0.0:
            suspicious.append((r, "vision 与 odo 同时为 0（车没动却判通过）"))
        notes = r["notes"]
        if "零脉冲" in notes:
            suspicious.append((r, "notes 里有 homing_zero_pulse 留痕"))
    zero_pulse = [r for r in scope if "homing_zero_pulse" in r["notes"]]
    no_mid = [r for r in ok if "mid_capture_failed" in r["notes"]]
    out(f"  判 OK 但有可疑特征的条目：{len(suspicious)}")
    for r, why in suspicious:
        out(f"    行{r['n']} #{int(r['experiment_id'])}：{why}")
    out(f"  notes 带 homing_zero_pulse（零脉冲归位，R67）的条目：{len(zero_pulse)}")
    out(f"  notes 带 mid_capture_failed（降级为起/终两点）的条目：{len(no_mid)}/{len(ok)}")
    out("  说明：dist_diff_pct 为空 + trial_ok=False 的失败条目**不算**假数据 —— "
        "它们本来就该没有对比值。")
    return 0


if __name__ == "__main__":
    sys.exit(main())