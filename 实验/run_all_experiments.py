#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 12 批量入口：矩阵 × repeats，逐条 归位 → 单次实验 → 对比入表 → 停止。

    cd 实验
    python3 run_all_experiments.py              # 全部 16 条 × 各自 repeats（真机！）
    python3 run_all_experiments.py --only 1     # 只跑实验 #1（× 它的 repeats）
    python3 run_all_experiments.py --dry        # 离线：只做场地校验、打印执行计划
                                                #（不开相机、不连车、不写表）

真机流程（每条）：`plan_start_xy(exp)` 算出执行起点 → `check_experiment_feasible`
（**显式**起点，见下）→ `harness.drive_to(起点)` → 归位失败就不跑实验、只记一行
`homing_*` 失败 → `harness.run_trial(exp)` → `compare.write_trial` 入表（失败也写）
→ 打印一行"视觉 vs 里程计"对比 → 收尾 `car.stop()`。

## Important 3（R67）：归位结果必须分三类，不能只读 `reached`

真机现场：`drive_to(0,0)` 连跑三次全 `reached=True, iters=0` —— **车根本没动**。
本文件原来只看 `reached`，于是把"没动过"记成"归位成功"（静默假数据）。
`harness.drive_to` 现在把"成功"与"车已在位"拆成两个字段（`reached` vs `at_target` +
`reason="already_at_target"`），本文件用 `classify_homing` 把归位结果分成三类：

- `"ok"` —— 闭环真的动过并收敛（`reached and moved`）：照常跑实验；
- `"zero_pulse"` —— 车**实测**在容差内但一条 RUN 都没发：实验**照跑**（车在位，数据
  是真的），但打印行带 `⚠零脉冲` 标记、计入 `n_homing_zero_pulse`、并往 trial 的
  `warnings` 里塞一条 `homing_zero_pulse`（`write_trial` 会写进汇总行的 `notes` 列 ——
  表里筛得出来，不留无声痕迹）；
- `"failed"` —— 真没归位：不跑实验（车在哪不知道，开电机等于盲开），只写一行
  `trial_reason="homing_*"` 的失败记录，继续下一条。

## Important 1：为什么起点必须显式传

`matrix.assert_within_field(exp, start_xy=(0.0, 0.0))` 的默认值会**静默走宽松
分支**（"全场可行性"），抓不住"这条实验在指定起点下其实放不下"。Task 9 复核给的
两条路：① Task 12 传目标执行起点；② 把 matrix 的默认改成 `None` 哨兵。**本文件
选①，且把"缺起点"封成显式错误**（`check_experiment_feasible(exp, None)` 直接
`ValueError`），理由：

- `实验/vgt/matrix.py` 的默认宽松分支是 brief 四条测试（默认起点、逐条不许抛）
  的行为锁，改哨兵会当场全红；且该文件正被别的任务修改，Task 12 不去碰。
- 宽松分支的合法用途（Task 9 的"换一个偏心起点一定放得下"）保留给 matrix 自己的
  测试；**执行预约**只许走严格分支，起点从 `plan_start_xy` 来（位移的对称中点，
  终点 = 起点 + 位移），旋转类起点就是中心 `(0, 0)`。

## 安全语义（别把这两类失败混进"标注原因"）

- `StopFailed` / `BoundaryMarkersMoved` 等**安全异常原样冒泡、中止整批** ——
  "失败条目标注原因而非崩溃"说的是 trial 级失败（`run_trial` 返回 `ok=False`）。
- 归位失败（`drive_to` 没到 `reached`）**不跑实验**：车在哪不知道，开电机等于盲开；
  只写一行 `trial_reason="homing_*"` 的失败记录，继续下一条。
"""
from __future__ import annotations

import argparse
import contextlib
import math
import sys
from pathlib import Path

from vgt import compare, matrix
from vgt.harness import DEFAULT_PULSE_MS, REASON_ALREADY_AT_TARGET, body_delta_to_field

# 规划起点假定的车头朝向（度）：`matrix` 的几何前提是"归位后车头已对齐"（R15），
# 场地系 +X 就是车头方向。用 `body_delta_to_field` 转，不自己再推一遍符号。
PLANNED_START_YAW_DEG = 0.0


def classify_homing(home: dict) -> tuple[str, str | None]:
    """`drive_to` 的结果 → `("ok" | "zero_pulse" | "failed", reason)`（R67）。

    一个布尔值说不清这件事，必须分三类（词表见模块 docstring）：

    - `"ok"`：闭环**真的动过**并且收敛（`reached and moved`）；
    - `"zero_pulse"`：车**实测**在容差内，但这次一条 RUN 都没发（`iters == 0`）——
      `harness` 报 `reached=False, reason="already_at_target", at_target=True`；
      这是"起点本来就达标"的合法情形，但**绝不是归位成功**（R67 的静默假成功）；
    - `"failed"`：真没归到位（`locate_failed` / `max_iter` / `run_rejected` …）。

    兜底（老结果字典、手造字典）：`moved` 缺失时用 `iters` 判；`reached=True` 且明确
    "没动过"的一律降级成 `zero_pulse` —— **宁可标出来，也不许静默当成功**。
    """
    reason = home.get("reason") or None
    moved = home.get("moved")
    if moved is None:
        iters = home.get("iters")
        moved = None if iters is None else int(iters) > 0
    reached = bool(home.get("reached"))
    at_target = home.get("at_target")
    if at_target is None:
        at_target = reached        # 老字典的 `reached` 原意就是"车落在容差内"

    if reached and moved is True:
        return "ok", None
    if reason == REASON_ALREADY_AT_TARGET or (reached and moved is False):
        # 零脉冲：只有"车确实在容差内"才允许照常跑实验，否则按没归位处理。
        return ("zero_pulse", REASON_ALREADY_AT_TARGET) if at_target \
            else ("failed", REASON_ALREADY_AT_TARGET)
    if not reached:
        return "failed", reason or "unknown"
    return "ok", None            # 退化兼容：既没 moved 也没 iters 的老字典


def _zero_pulse_note(home: dict) -> str:
    """零脉冲归位的留痕文字（进打印行与 trial 的 warnings → xlsx 的 `notes` 列）。"""
    start = home.get("start_dist_mm")
    start_txt = "" if start is None else f"，起点实测距目标 {float(start):.1f} mm"
    return (f"homing_zero_pulse（零脉冲归位）：起点已在容差内、零次迭代"
            f"（drive_to reason={REASON_ALREADY_AT_TARGET}{start_txt}）—— "
            "车确实在位，但本次归位没有执行过任何动作，不计为归位成功")


def plan_start_xy(exp) -> tuple[float, float]:
    """一条实验的**执行起点**（场地系 mm）：让行程以场地原点对称（R15）。

    终点 = 起点 + 位移，取位移的对称中点：直行 200 → 起点 (−100, 0)；
    横移左 80 → (0, +40)；斜行 → (∓42.4, ±42.4)；原地旋转 → (0, 0)。

    ⚠ 位移取**期望行程**（`matrix.expected_delta_mm`，真机会走多少），不是命令量
    `v·T`：R68 起 `duration_s` 是按实测速度模型反解的，命令量只有真机的 ~1/2
    （#01：`v·T` = 100 mm，真机 ≈200 mm）。拿命令量算起点会把车摆到
    126 mm 外（x 中心上限 117.5）⇒ 第一条就出界。

    ⚠⚠ 命令方向**不等于**场地系方向（R69，真机实测钉死）：`(vx, vy)` 是**车体系**
    速度，矩阵的前提是"归位后车头已对齐"（yaw ≈ 0）。按 `harness.body_delta_to_field`
    的同一条符号链，车体系 +y（车左）在场地系里是 **−Y** —— 真机实测：#05「横移左移
    80 mm」起点 y = −34.9、终点 y = −66.3（**往 −Y 走**）。拿命令方向当场地方向会把
    横移/斜行的起点摆到错的一侧：#05 因此在 −Y 侧多走 31 mm，车体包络越出场地 14.9 mm
    所以这里一律先用 `body_delta_to_field(..., yaw=0)` 转到场地系；
    R69 起"对中"的唯一实现是 `matrix.expected_start_xy`（内部走 `field_delta_mm`），
    本函数直接调它 —— 规划起点与场地校验共用同一个真源，不再各自推一遍符号。
    """
    return tuple(matrix.expected_start_xy(exp, PLANNED_START_YAW_DEG))


def check_experiment_feasible(exp, start_xy) -> None:
    """严格场地校验：起点**显式**给出，缺起点直接报错（不落回默认宽松分支）。

    `start_xy=None` 是文档化错误：调用方不许拿"没归位"冒充"名义中心"。
    """
    if start_xy is None:
        raise ValueError(
            f"实验 #{exp.id}（{exp.desc}）：assert_within_field 必须显式传执行起点 —— "
            "默认 (0.0, 0.0) 会静默走宽松分支（Task 9 复核 Important 1）。")
    matrix.assert_within_field(exp, start_xy=start_xy)


def iterate_trials(experiments=matrix.EXPERIMENTS, only=None):
    """`(exp, repeat_index)` 生成器：矩阵 × 每条自己的 repeats；`only` 按 id 过滤。"""
    for exp in experiments:
        if only is not None and int(exp.id) != int(only):
            continue
        for rep in range(int(exp.repeats)):
            yield exp, rep


def _homing_failed_trial(reason: str) -> dict:
    """归位失败时合成的"没跑过"的 trial 结果（形状与 `run_trial` 一致，全空）。"""
    return {
        "ok": False, "reason": reason,
        "vision": {"start": None, "mid": None, "end": None},
        "vision_delta": None, "odometry_delta": None,
        "samples": [], "warnings": [],
    }


def format_trial_line(index: int, total: int, exp, rep: int, summary: dict,
                      homing_note: str = "") -> str:
    """一行实时对比：`#id rep 描述：vision / odo / Δ% / yawΔ / 判定`（+ 归位备注）。"""
    def num(value, spec):
        return "-" if value is None else format(value, spec)

    verdict = "OK" if summary["trial_ok"] else f"FAIL {summary['trial_reason'] or '?'}"
    return (
        f"[{index:2d}/{total}] #{int(exp.id):02d} rep{rep + 1}/{int(exp.repeats)} {exp.desc}: "
        f"vision {num(summary['vision_dist_mm'], '.1f')}mm / "
        f"odo {num(summary['odo_dist_mm'], '.1f')}mm / "
        f"Δ {num(summary['dist_diff_pct'], '+.1f')}% / "
        f"yawΔ {num(summary['yaw_diff_deg'], '+.2f')}°  {verdict}"
        + (f"  ⚠{homing_note}" if homing_note else ""))


def _final_stop(harness, printer) -> None:
    """每条结束 `car.stop()`（计划要求）；停了没停已由 harness 的收尾 STOP 兜底，
    这里失败只大声提示，不覆盖 `StopFailed` 语义。"""
    link = getattr(harness, "car_link", None)
    stop = getattr(link, "stop", None)
    if stop is None:
        return
    try:
        stop()
    except Exception as exc:                              # noqa: BLE001 - 提示性兜底
        printer(f"⚠ 收尾 car.stop() 未成功：{type(exc).__name__}: {exc}"
                "（run_trial 自身的收尾 STOP 已发；若两次都没确认，harness 会抛 StopFailed）")


def run_batch(harness, *, experiments, only=None, xlsx_path,
              printer=print, write=compare.write_trial, archive=True,
              pulse_ms=DEFAULT_PULSE_MS) -> dict:
    """核心批量循环（硬件只从 `harness` 进来 —— 测试注入假对象即可离线跑通）。

    返回统计：`n_trials / n_ok / n_failed / n_homing_failed / n_homing_zero_pulse /
    archive_path`。安全异常（`StopFailed`/`BoundaryMarkersMoved`）原样冒泡中止整批。

    `pulse_ms`：传给 `drive_to` 的**归位脉冲宽度**（默认 = `harness` 默认 400 ms）。
    ⚠ 真机实测（Task 10 清单 5.3 / SDD 台账）：400 ms 单脉冲步长 131.8–148.2 mm
    ≫ `tol_mm=12`，12 次迭代走成周期 2 极限环、**不收敛**，"有时收敛"比稳定不收敛更
    危险；而 >400 ms 的脉冲会把车开出场地（实测出界 1 次）。只有 **150–175 ms** 窗口
    收敛（3 轮 6.7/11.0/11.4 mm）。所以真机跑全量必须显式 `--pulse-ms 150`。
    """
    trials = list(iterate_trials(experiments, only=only))
    stats = {"n_trials": len(trials), "n_ok": 0, "n_failed": 0,
             "n_homing_failed": 0, "n_homing_zero_pulse": 0, "archive_path": None}

    for index, (exp, rep) in enumerate(trials, start=1):
        start_xy = plan_start_xy(exp)
        check_experiment_feasible(exp, start_xy)          # 缺起点/越界：当场报错
        homing_note = ""
        try:
            home = harness.drive_to(start_xy[0], start_xy[1], pulse_ms=float(pulse_ms))
            kind, homing_reason = classify_homing(home)
            if kind == "failed":
                stats["n_homing_failed"] += 1
                trial = _homing_failed_trial(f"homing_{homing_reason or 'unknown'}")
            else:
                if kind == "zero_pulse":
                    # R67：车在位但归位没动过 —— 照跑实验，但绝不当成功，且留痕。
                    stats["n_homing_zero_pulse"] += 1
                    homing_note = _zero_pulse_note(home)
                trial = harness.run_trial(exp)
                if homing_note:
                    trial.setdefault("warnings", []).append(homing_note)
            summary = compare.trial_summary(trial)
            write(trial, exp, xlsx_path=xlsx_path)        # 失败也写（留痕）
            printer(format_trial_line(index, len(trials), exp, rep, summary,
                                      homing_note=homing_note))
        finally:
            _final_stop(harness, printer)                 # 每条结束 car.stop()
        if summary["trial_ok"]:
            stats["n_ok"] += 1
        else:
            stats["n_failed"] += 1

    if archive and trials:
        try:
            stats["archive_path"] = str(compare.archive_workbook(xlsx_path))
        except Exception as exc:                          # noqa: BLE001 - 归档失败不毁数据
            printer(f"⚠ 归档失败（数据已入表）：{type(exc).__name__}: {exc}")
    return stats


def dry_run(*, experiments=matrix.EXPERIMENTS, only=None, printer=print) -> int:
    """`--dry`：不开相机、不连车 —— 只做场地校验并打印每条的执行计划。"""
    trials = list(iterate_trials(experiments, only=only))
    if not trials:
        printer(f"没有匹配 --only {only} 的实验。")
        return 2
    for index, (exp, rep) in enumerate(trials, start=1):
        start_xy = plan_start_xy(exp)
        check_experiment_feasible(exp, start_xy)
        # 终点取**期望行程**（R68：命令量 v·T 只有真机的一半，见 matrix docstring），
        # 且必须是**场地系**位移（R69）：`plan_start_xy` 的起点已经用
        # `body_delta_to_field` 转过了，打印机再拿车体系 `expected_delta_mm` 相加
        # 会把横移的终点印到起点同侧（#05 印成 +119.3，实际是 −39.7）——显示与
        # `check_experiment_feasible` 的校验各说一套，`--dry` 就成了误导操作员的那一份。
        dx, dy = matrix.field_delta_mm(exp)
        dpsi = matrix.expected_yaw_deg(exp)
        ms = int(round(float(exp.duration_s) * 1000.0))
        printer(
            f"[{index:2d}/{len(trials)}] #{int(exp.id):02d} rep{rep + 1}/{int(exp.repeats)} "
            f"{exp.desc}: 起点 ({start_xy[0]:+.1f}, {start_xy[1]:+.1f}) → "
            f"终点 ({start_xy[0] + dx:+.1f}, {start_xy[1] + dy:+.1f}) mm；"
            f"净转 {dpsi:+.1f}°；RUN {float(exp.vx):.4f} {float(exp.vy):.4f} "
            f"{float(exp.om):.4f} {ms}")
    printer(f"DRY：{len(trials)} 条计划全部通过场地校验（未开相机、未连车、未写表）。")
    return 0


@contextlib.contextmanager
def _open_real_session():
    """真机会话（只有非 `--dry` 路径才会走到这里）：相机 + 链路 + harness。

    依赖全部延迟 import —— 本模块在被测试导入时不许碰硬件入口。
    """
    from car_link import CarLink
    from vision_core import load_calib

    from vgt.camera import CameraSession
    from vgt.harness import Harness

    with CameraSession() as cam, CarLink() as car:
        yield Harness(car, cam, load_calib())


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Task 12 批量：矩阵 × repeats，归位 → 实验 → 对比入表（--dry 离线预演）")
    parser.add_argument("--only", type=int, default=None, metavar="ID",
                        help="只跑指定 experiment_id（默认全部）")
    parser.add_argument("--dry", action="store_true",
                        help="不开相机/不连车：只做场地校验并打印执行计划")
    parser.add_argument("--pulse-ms", type=float, default=DEFAULT_PULSE_MS, metavar="MS",
                        help=f"归位脉冲宽度 ms（默认 {DEFAULT_PULSE_MS:g} = harness 默认；"
                             "真机实测 400 ms 不收敛，必须显式给 150）")
    return parser.parse_args(argv)


def main(argv=None, *, session_factory=None, printer=print, xlsx_path=None) -> int:
    """CLI 入口。返回 0 = 全部通过；1 = 有条目失败（已入表标注）；2 = 用法错误。

    `--pulse-ms` 只改**归位脉冲宽度**（默认 400 ms = `harness.DEFAULT_PULSE_MS`，行为不变）；
    实验本身的 `RUN` 参数来自 `matrix.EXPERIMENTS`，不受它影响。真机跑全量必须显式
    给 `--pulse-ms 150`（理由见 `run_batch` 的 docstring）。
    """
    args = parse_args(argv)
    known = {int(e.id) for e in matrix.EXPERIMENTS}
    if args.only is not None and args.only not in known:
        printer(f"没有实验 #{args.only}：矩阵只有 {sorted(known)}。")
        return 2
    if args.dry:
        return dry_run(experiments=matrix.EXPERIMENTS, only=args.only, printer=printer)

    xlsx = Path(xlsx_path) if xlsx_path is not None else compare.DEFAULT_XLSX
    factory = session_factory if session_factory is not None else _open_real_session
    n_trials = sum(1 for _ in iterate_trials(matrix.EXPERIMENTS, only=args.only))
    printer(f"=== Task 12 批量：{n_trials} 次（{len(matrix.EXPERIMENTS)} 条矩阵 × repeats"
            f"{'，--only ' + str(args.only) if args.only is not None else ''}"
            f"，归位脉冲 {args.pulse_ms:g} ms）→ {xlsx} ===")
    with factory() as harness:
        stats = run_batch(harness, experiments=matrix.EXPERIMENTS, only=args.only,
                          xlsx_path=xlsx, printer=printer, pulse_ms=args.pulse_ms)
    if stats["archive_path"]:
        printer(f"已归档：{stats['archive_path']}")
    printer(f"完成：{stats['n_ok']}/{stats['n_trials']} 通过；{stats['n_failed']} 条失败"
            f"（其中归位失败 {stats['n_homing_failed']} 条）"
            f" —— 失败条目已入表并标注 trial_reason，可筛除后重跑。")
    if stats["n_homing_zero_pulse"]:
        printer(f"⚠ 另有 {stats['n_homing_zero_pulse']} 条的归位是**零脉冲**"
                f"（起点已在容差内、没执行过任何动作，R67）：实验照跑，数据有效，"
                "但这些条目的起点只由单次测量担保，汇总行 notes 里有 homing_zero_pulse 留痕。")
    return 0 if stats["n_failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
