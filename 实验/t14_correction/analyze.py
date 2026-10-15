#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 14 的**唯一复算入口**：把两次会话的跑车数据合成一张修正常数表。

数据来源
--------
1. `实验/t14_correction/data/campaign.jsonl` —— 2026-10-15 本任务补的实验（`campaign.py` 逐条追加）；
2. `实验/t12_stats/dataset.json` —— 更早会话的 84 条汇总行（`t12_stats/build.py` 产出），
   只取 `trial_ok` 且 `vision_dist_mm` 非空的行。

口径（每一张表都按这个来，不另设）
----------------------------------
* 行程 `s` 取**视觉场系位移**（`vision_dist_mm` / 汇总行的同名列）；
* 时长 `T` 取**下发的毫秒数**（`duration_planned`），不是实测的 `t1`；
* 拟合 `s = V·(T − t_d)`，等价 `d = a·T + b` 最小二乘，`V = a`、`t_d = −b/a`；
* 分组键 = **通道 × 方向**（不是实验编号）。
* `σ` = 拟合残差的标准误（自由度为 n−2），单位与 `s` 同。
* `se_V` / `se_td` 是斜率、截距传播过来的标准误 —— **它才是「这个常数有多准」的度量**，
  不是 R²。R² 只说明「线性像不像」。

跑法：`python3 实验/t14_correction/analyze.py`
"""
from __future__ import annotations

import json
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
CAMPAIGN = HERE / "data" / "campaign.jsonl"
ORIGINAL = HERE.parent / "t12_stats" / "dataset.json"

TODAY = "2026-10-15"      # 本次补实验的会话标签
EARLIER = "上次会话"       # 原 84 条


# --------------------------------------------------------------------------
# 1. 读两端数据，统一成 (session, channel, direction, T_ms, s, yaw, note)
# --------------------------------------------------------------------------
def classify(vx: float, vy: float, om: float):
    """命令 → (通道, 方向)。返回 None 表示不是本表要的单通道动作。"""
    eps = 1e-9
    if abs(om) > eps and abs(vx) < eps and abs(vy) < eps:
        return ("旋转", "逆时针" if om > 0 else "顺时针")
    if abs(om) > eps:
        return None                       # 「平移 + om」单独报告，不进常数表
    if abs(vy) > eps and abs(vx) < eps:
        return ("横向", "左移" if vy > 0 else "右移")
    if abs(vx) > eps and abs(vy) < eps:
        return ("纵向", "前进" if vx > 0 else "后退")
    if abs(vx) > eps and abs(vy) > eps:
        return ("斜行", ("前" if vx > 0 else "后") + ("左" if vy > 0 else "右"))
    return None


def load_campaign():
    if not CAMPAIGN.exists():
        return []
    out = []
    for line in CAMPAIGN.read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        ch = classify(r["vx"], r["vy"], r["om"])
        out.append(dict(
            session=TODAY, tag=r["tag"], idx=r["idx"],
            channel=ch[0] if ch else None, direction=ch[1] if ch else None,
            T=float(r["ms"]), s=float(r["dist_mm"]) if r["dist_mm"] is not None else None,
            yaw=float(r["yaw_deg"]) if r["yaw_deg"] is not None else None,
            vx=r["vx"], vy=r["vy"], om=r["om"],
        ))
    return out


_DESC = {"前进": ("纵向", "前进"), "后退": ("纵向", "后退"),
         "左移": ("横向", "左移"), "右移": ("横向", "右移"),
         "逆时针": ("旋转", "逆时针"), "顺时针": ("旋转", "顺时针")}


def load_original():
    if not ORIGINAL.exists():
        return []
    out = []
    for r in json.load(open(ORIGINAL)):
        if not r.get("trial_ok") or r.get("vision_dist_mm") is None:
            continue
        at, desc = r["action_type"], r["command_description"]
        ch = None
        if at == "straight":
            ch = ("纵向", "前进" if r["command_param_1"] > 0 else "后退")
        elif at == "strafe":
            ch = ("横向", "左移" if "左移" in desc else "右移")
        elif at == "rotate_in_place":
            ch = ("旋转", "逆时针" if "逆时针" in desc else "顺时针")
        elif at == "diagonal":
            ch = ("斜行", ("前" if "前" in desc else "后") + ("左" if "左" in desc else "右"))
        out.append(dict(
            session=EARLIER, tag="orig", idx=r["xlsx_row"],
            channel=ch[0] if ch else None, direction=ch[1] if ch else None,
            T=r["duration_planned"] * 1000.0, s=r["vision_dist_mm"],
            yaw=r["vision_yaw_deg"],
            vx=r["command_param_1"], vy=0.0, om=0.0,
        ))
    return out


# --------------------------------------------------------------------------
# 2. 最小二乘 + 不确定度
# --------------------------------------------------------------------------
def lsq(pts):
    """pts: [(T, s)]  →  V, t_d, R², σ, se_V, se_td（角度通道单位自动跟随 s）"""
    n = len(pts)
    if n < 2:
        return None
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    a = (n * sxy - sx * sy) / den            # 单位/毫秒
    b = (sy - a * sx) / n
    yb = sy / n
    sst = sum((p[1] - yb) ** 2 for p in pts)
    ssr = sum((p[1] - (a * p[0] + b)) ** 2 for p in pts)
    if n == 2:
        se_a = 0.0                                   # 两点必然穿过去，残差恒为 0
        sig = float("nan")
    else:
        se_a = math.sqrt((ssr / (n - 2)) / (sxx - sx * sx / n))
        sig = math.sqrt(ssr / (n - 2))
    V = a * 1000.0                                   # 每秒
    td = -b / a                                      # 毫秒
    se_td = abs(b / a) * math.sqrt((se_a / a) ** 2) if n > 2 else float("nan")
    return dict(n=n, V=V, td=td, r2=1 - ssr / sst if sst > 0 else float("nan"),
                sigma=sig, se_V=se_a * 1000.0, se_td=se_td, a=a, b=b)


def fit_group(recs, use_abs_yaw=False):
    pts = [(r["T"], abs(r["yaw"]) if use_abs_yaw else r["s"]) for r in recs
           if r["s"] is not None and (r["yaw"] is not None or not use_abs_yaw)]
    return lsq(pts) if len(pts) >= 2 else None


# --------------------------------------------------------------------------
# 3. 报告
# --------------------------------------------------------------------------
def show_fits(recs):
    groups = defaultdict(list)
    for r in recs:
        if r["channel"]:
            groups[(r["channel"], r["direction"])].append(r)
    print("=" * 104)
    print("表 1 —— 各通道 × 方向的常数（按会话分开 + 合并）")
    print("=" * 104)
    print(f"{'会话':<10}{'通道':<6}{'方向':<8}{'n':>3}  {'V':>9}{'±se':>7}  "
          f"{'t_d(ms)':>8}{'±se':>7}  {'R²':>7}  {'σ':>6}")
    order = [("纵向", "前进"), ("纵向", "后退"), ("横向", "左移"), ("横向", "右移"),
             ("旋转", "逆时针"), ("旋转", "顺时针"),
             ("斜行", "前左"), ("斜行", "前右"), ("斜行", "后左"), ("斜行", "后右")]
    for ch, dr in order:
        for sess in (TODAY, EARLIER, "合并"):
            key = (ch, dr)
            if sess == "合并":
                rs = [r for r in recs if r["channel"] == ch and r["direction"] == dr]
            else:
                rs = [r for r in recs if r["channel"] == ch and r["direction"] == dr
                      and r["session"] == sess]
            if not rs:
                continue
            f = fit_group(rs, use_abs_yaw=(ch == "旋转"))
            if not f:
                continue
            unit = "°/s" if ch == "旋转" else "mm/s"
            V = f"{f['V']:.1f}{unit}" if ch != "旋转" else f"{f['V']:.1f}{unit}"
            print(f"{sess:<10}{ch:<6}{dr:<8}{f['n']:>3}  {f['V']:>9.1f}{f['se_V']:>7.1f}  "
                  f"{f['td']:>8.1f}{f['se_td']:>7.1f}  {f['r2']:>7.4f}  "
                  f"{('%.2f' % f['sigma']) if f['sigma'] == f['sigma'] else '  --':>6}"
                  f"   ({unit})")
        print("-" * 104)


def show_yaw_bias(recs):
    print("\n" + "=" * 104)
    print("表 2 —— 方向锁定的航向偏差（只有纯平移/直行有定义）")
    print("=" * 104)
    print(f"{'通道':<6}{'方向':<8}{'n':>3}  {'yaw 中位':>10}  {'正/负':>8}  {'组内残差 σ':>11}"
          f"  {'末端横向偏移(中位)':>18}")
    for ch in ("纵向", "横向"):
        for dr in ("前进", "后退", "左移", "右移"):
            rs = [r for r in recs if r["channel"] == ch and r["direction"] == dr
                  and r["yaw"] is not None and r["s"] is not None]
            if not rs:
                continue
            ys = [r["yaw"] for r in rs]
            med = st.median(ys)
            pos = sum(1 for y in ys if y > 0)
            resid = [y - med for y in ys]
            sd = st.stdev(resid) if len(resid) > 1 else float("nan")
            offs = [abs(r["s"] * math.sin(math.radians(r["yaw"]))) for r in rs]
            print(f"{ch:<6}{dr:<8}{len(rs):>3}  {med:>+9.2f}°  {pos:>4}/{len(ys)-pos:<3}  "
                  f"{sd:>10.2f}°  {st.median(offs):>15.1f} mm")


def show_om(recs):
    print("\n" + "=" * 104)
    print("表 3 —— om 死区（vx 固定、om 变）：om 的**唯一**作用点")
    print("=" * 104)
    rs = [r for r in recs if abs(r["om"]) > 1e-9]
    if not rs:
        print("  （本批没有 om≠0 的样本）")
        return
    rs.sort(key=lambda r: (abs(r["vx"]) > 1e-9, abs(r["om"])))
    print(f"{'vx':>6}{'om':>7}{'T(ms)':>7}  {'行程':>8}  {'yaw':>8}  {'dy':>8}   判定")
    for r in rs:
        dy = r["s"] * math.sin(math.radians(r["yaw"])) if r["yaw"] is not None else float("nan")
        verdict = ("纯旋转" if abs(r["vx"]) < 1e-9 else
                   "平移为主" if r["s"] and r["s"] > 25 else "**被改写成旋转**")
        print(f"{r['vx']:>6.2f}{r['om']:>7.2f}{r['T']:>7.0f}  {r['s']:>8.2f}"
              f"  {r['yaw']:>+8.2f}  {dy:>8.2f}   {verdict}")


def show_repeat(recs):
    print("\n" + "=" * 104)
    print("表 4 —— 同一命令重复跑：这是**开环精度的真实上限**（同一会话内）")
    print("=" * 104)
    bucket = defaultdict(list)
    for r in recs:
        if r["session"] != TODAY or not r["channel"] or r["channel"] == "斜行":
            continue
        if abs(r["om"]) > 1e-9 and abs(r["vx"]) > 1e-9:
            continue
        bucket[(r["channel"], r["direction"], r["T"])].append(r)
    print(f"{'通道':<6}{'方向':<8}{'T(ms)':>7}{'n':>3}  {'均值':>8}{'σ':>7}{'极差':>8}"
          f"  {'yaw 均值':>9}{'yaw σ':>7}")
    for k in sorted(bucket):
        rs = bucket[k]
        if len(rs) < 2:
            continue
        vals = [r["s"] for r in rs]
        ys = [r["yaw"] for r in rs if r["yaw"] is not None]
        sd = st.stdev(vals) if len(vals) > 1 else float("nan")
        ysd = st.stdev(ys) if len(ys) > 1 else float("nan")
        print(f"{k[0]:<6}{k[1]:<8}{k[2]:>7.0f}{len(rs):>3}  {st.mean(vals):>8.2f}{sd:>7.2f}"
              f"{max(vals)-min(vals):>8.2f}  {st.mean(ys):>+9.2f}{ysd:>7.2f}")


def show_chrono(recs):
    """同一条命令按时间顺序排开 —— 看电池/发热有没有让速度漂。"""
    print("\n" + "=" * 104)
    print("表 5 —— 时间漂移检查：同一条命令在批次里的先后（今天，按落盘顺序）")
    print("=" * 104)
    rs = [r for r in recs if r["session"] == TODAY and r["channel"] == "纵向"
          and r["T"] == 240.0 and abs(r["om"]) < 1e-9]
    if not rs:
        print("  （没有重复的锚点）")
        return
    print(f"{'序':>3}  {'标签':>14}{'方向':<8}{'行程':>8}{'yaw':>8}")
    for i, r in enumerate(rs, 1):
        print(f"{i:>3}  {r['tag']:>14}{r['direction']:<8}{r['s']:>8.2f}{r['yaw']:>+8.2f}")
    f = [r["s"] for r in rs if r["direction"] == "前进"]
    b = [r["s"] for r in rs if r["direction"] == "后退"]
    if f:
        print(f"  前进 240ms：均值 {st.mean(f):.2f}  极差 {max(f)-min(f):.2f} mm  n={len(f)}")
    if b:
        print(f"  后退 240ms：均值 {st.mean(b):.2f}  极差 {max(b)-min(b):.2f} mm  n={len(b)}")


def cold_start_fit(recs):
    """把「斜行」的四个方向按单档时长列出来（斜行目前档位太少，先看数）。"""
    print("\n" + "=" * 104)
    print("表 6 —— 斜行：目前有哪些时长档")
    print("=" * 104)
    bucket = defaultdict(list)
    for r in recs:
        if r["channel"] == "斜行":
            bucket[(r["direction"], r["T"])].append(r)
    if not bucket:
        print("  （还没有斜行数据）")
        return
    axis = {"前左": "A", "后右": "A", "前右": "B", "后左": "B"}
    for k in sorted(bucket, key=lambda k: (axis[k[0]], k[1])):
        rs = [r for r in bucket[k] if r["s"] is not None]
        if not rs:
            print(f"  {k[0]:<5} T={k[1]:>5.0f} ms  n=0（该条没解出位移）")
            continue
        vals = [r["s"] for r in rs]
        ys = [r["yaw"] for r in rs if r["yaw"] is not None]
        print(f"  [{axis[k[0]]}] {k[0]:<5} T={k[1]:>5.0f} ms  n={len(rs):>2}"
              f"  行程均值={st.mean(vals):>7.2f} mm  极差 {min(vals):.2f}–{max(vals):.2f}"
              f"  yaw 均值={st.mean(ys):>+6.2f}°" if ys else "")
    print("  轴 A = 前左 / 后右（驱动 FR+RL 一对）；轴 B = 前右 / 后左（驱动 FL+RR 一对）。")
    print("  两轴不是同一件事 —— 固件里它们走的是不同的轮子对，速度也不同。")

    print(f"\n{'':2}{'轴':<4}{'会话':<10}{'n':>3}  {'V(mm/s)':>9}{'±se':>7}{'t_d(ms)':>9}"
          f"{'±se':>7}{'R²':>8}{'σ(mm)':>7}")
    for ax, dirs in (("A", ("前左", "后右")), ("B", ("前右", "后左"))):
        for sess in (TODAY, EARLIER, "合并"):
            rs = [r for r in recs if r["channel"] == "斜行" and r["direction"] in dirs
                  and (sess == "合并" or r["session"] == sess) and r["s"] is not None]
            if len(rs) < 3:
                continue
            f = fit_group(rs)
            if not f:
                continue
            print(f"{'':2}[{ax}]{'':<2}{sess:<10}{f['n']:>3}  {f['V']:>9.1f}{f['se_V']:>7.1f}"
                  f"{f['td']:>9.1f}{f['se_td']:>7.1f}{f['r2']:>8.4f}"
                  f"{('%.2f' % f['sigma']) if f['sigma'] == f['sigma'] else '  --':>7}")


def main() -> int:
    recs = load_original() + load_campaign()
    print(f"载入：{EARLIER} {sum(1 for r in recs if r['session']==EARLIER)} 条，"
          f"{TODAY} {sum(1 for r in recs if r['session']==TODAY)} 条\n")
    show_fits(recs)
    show_yaw_bias(recs)
    show_om(recs)
    show_repeat(recs)
    show_chrono(recs)
    cold_start_fit(recs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
