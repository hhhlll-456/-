#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Task 12 Step 4 **补跑**：只补上一轮失败 / 从没跑过的那几条，每条之前先归位。

    cd 实验
    python3 t12_tools/rerun_failed.py --write-plan          # 离线：算出补跑清单并冻结
    python3 t12_tools/rerun_failed.py --plan                 # 离线：看清单
    python3 t12_tools/rerun_failed.py --exp 7                # 真机：跑 #7 名下的补跑条目
    python3 t12_tools/rerun_failed.py --archive              # 真机：给当前表打一份时间戳归档

## 为什么不是 `--only <id>` 重跑整条

`--only` 只能筛到**实验 id** 粒度，一条实验的 3 个 repeats 是一个整体。上一轮已经跑通
的条目重跑一遍等于**用新数据盖掉旧数据**，还多烧 5 条真机行程。所以本脚本按
`(experiment_id, repeat_index)` 精确补：上一轮 45 条里 10 条失败 + #16 一条没跑 = **13 条**。

## rep 序号是怎么从表里认出来的（别当成理所当然）

同一条实验的 repeats 在表里按 `rep1 → repN` 的写入顺序出现，所以**组内位置 = rep 序号**
（0 起）。本脚本 `plan_missing()` 就是按这个把汇总行映射回 `(id, rep)` 的。
一旦某条实验的汇总行数 **超过** `exp.repeats`（= 这条实验被重复跑过，表里出现两组
rep1..repN），位置推断就不可靠了 —— 此时**直接报错退出**，不许猜，宁可一条不补。

## 安全顺序（每条之间，不是一条里面）

`recenter.py`（**先摆头、再平移**）→ 跑这一条 → `carpos.py` 巡检。上一轮 10 条失败里
**7 条是 `homing_locate_failed`、`iters=0`** —— 上一条把车开到相机定位不到的地方，
下一条还没开电机就死了。所以：

- 归位**失败就不跑这一条**（直接跳过并记账），不是"开着电机试试"；
- 巡检分两档（见 `HARD_STOP_VERDICTS` / `RECOVER_VERDICTS`）：
  - `DEADLOCK`（视觉定位不了）—— 没有任何自救手段，**立刻停、要人工搬**；
  - `OUT_OF_FIELD` / `CHECK`（包络越出 marker 矩形 / 车心离边界 < 80 mm）——
    **停本 chunk → 强制归位 → 复检**，复检不 OK 就不许再跑。纯包络越线（车心余量还很大、
    车还在桌面上、9/9 帧能定位）也走这一档，上一轮出界 2 次都是这个形态；
- 归位失败时 `recenter.py` 自己也会停手（`align_yaw` 不收敛就不平移），不盲开。

## 根因未查明：反向脉冲

上一轮 #15 归位时车头 −79.7°，第 2 个脉冲 `gained=−35.6 mm`（离目标更远），车滑出视野。
`recenter.py` 的「先摆头再平移」是**预防性措施、不是已验证的修法**。本脚本因此在每条
补跑条目之间强制走一遍 `recenter.py`，让平移永远落在 yaw≈0 的已验证区间。
本脚本**额外**记录每条跑完的 `carpos` 车体 yaw —— 一旦看到车头角很大，会在日志里出声。
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "实验"))

from vgt import compare, matrix                                   # noqa: E402

PLAN_PATH = HERE / "rerun_plan.json"
XLSX = compare.DEFAULT_XLSX
RECENTER = HERE / "recenter.py"
CARPOS = HERE / "carpos.py"
# 与 Task 12 Step 4 全量跑批那一组完全一致（run_chunk.sh 同款），别单独调参。
BATCH_ARGS = ["--pulse-ms", "120", "--coarse-ms", "320",
              "--yaw-tol-deg", "6", "--tol-mm", "4", "--max-iter", "24"]
# 归位失败 / 巡检不 OK 时给子进程的时间上限（s）。
RECENTER_TIMEOUT_S = 900
TRIAL_TIMEOUT_S = 1800
CARPOS_TIMEOUT_S = 300
# 巡检判定（`carpos.py` 的 verdict 词表）分两档，别混：
#   DEADLOCK —— 视觉定位不了（ID29/边界码掉码）。**没有任何自救手段**，立刻停、要人工搬。
#   OUT_OF_FIELD / CHECK —— 车体越出 marker 矩形，或车心离边界 < 80 mm。
#       这两档按简报的口径**先停本 chunk，再强制归位 + 复检**；复检回到场地内且视觉可
#       定位才继续（简报："离边界 < 80 mm 就暂停、归位再继续"）。
#       纯包络越线（车心余量还很大、车还在桌面上、9/9 帧能定位）也走这一档 —— 上一轮
#       出界 2 次都是这个形态。每一次都在日志里出声，不静默吞掉。
HARD_STOP_VERDICTS = ("DEADLOCK",)
RECOVER_VERDICTS = ("OUT_OF_FIELD", "CHECK")
OK_VERDICTS = ("OK",)


def exp_by_id(exp_id):
    for exp in matrix.EXPERIMENTS:
        if int(exp.id) == int(exp_id):
            return exp
    raise KeyError(f"矩阵里没有实验 #{exp_id}")


def plan_missing(xlsx=XLSX, from_row=24):
    """汇总行 → 还缺哪些 `(experiment_id, repeat_index)`（按实验 id、rep 升序）。

    `from_row` = **第几条汇总行**（1 起），24 = 上一轮全量的起点（见
    `t12_step4_report.PREV_ROUND_SUMMARY_ROWS`）。复用的判据：任一实验的汇总行数
    超过它自己的 `repeats` ⇒ 这条实验在表里不止一组 rep，位置推断不可信 ⇒ 报错。
    """
    from t12_step4_report import load_summary_rows          # 只读 xlsx，不开相机

    rows = [r for r in load_summary_rows(xlsx) if r["n"] >= int(from_row)]
    groups: dict[int, list] = {}
    for r in rows:                                          # 表里本来就是写入顺序
        groups.setdefault(int(r["experiment_id"]), []).append(r)

    missing = []
    for exp in matrix.EXPERIMENTS:
        exp_id = int(exp.id)
        group = groups.get(exp_id, [])
        if len(group) > int(exp.repeats):
            raise SystemExit(
                f"实验 #{exp_id:02d} 在表里有 {len(group)} 条汇总行，超过它的 "
                f"repeats={exp.repeats} —— 这条实验被重复跑过，组内位置不再等于 "
                "rep 序号，不许猜。请用 --trials 显式指定补跑清单。")
        for rep in range(int(exp.repeats)):
            if rep >= len(group) or group[rep]["trial_ok"] is not True:
                missing.append({"experiment_id": exp_id, "repeat": rep,
                                "previous_reason": None if rep >= len(group) else
                                str(group[rep]["trial_reason"] or "?")})
    return missing


def write_plan(path=PLAN_PATH, xlsx=XLSX, from_row=24):
    plan = plan_missing(xlsx=xlsx, from_row=from_row)
    payload = {"xlsx": str(xlsx), "from_row": int(from_row), "missing": plan}
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return plan


def load_plan(path=PLAN_PATH):
    if not path.exists():
        raise SystemExit(f"没有补跑清单：先跑 `python3 {Path(__file__).name} --write-plan`")
    return json.loads(path.read_text(encoding="utf-8"))["missing"]


def parse_trials(spec):
    """`--trials "7:0,7:1"` → `[{"experiment_id", "repeat"}]`，`repeat` 是**0 起下标**。

    ⚠ 与 `run_all_experiments.py --trials` 的口径**相反**：那个是给人敲的，用 1 起；
    这里是内部计划文件，统一用 0 起下标。转换在拼命令行时做。
    """
    out = []
    for item in str(spec).split(","):
        item = item.strip()
        if not item:
            continue
        exp_id, _, rep = item.partition(":")
        out.append({"experiment_id": int(exp_id), "repeat": int(rep)})
    if not out:
        raise SystemExit("--trials 解析不出任何条目")
    return out


def _run_cmd(argv, timeout, printer):
    printer(f"$ {' '.join(str(a) for a in argv)}")
    proc = subprocess.run([str(a) for a in argv], cwd=str(ROOT / "实验"),
                          timeout=timeout, capture_output=True, text=True)
    tail = "\n".join((proc.stdout or "").strip().splitlines()[-14:])
    if tail:
        printer(tail)
    if proc.stderr.strip():
        printer("  [stderr] " + "\n".join(proc.stderr.strip().splitlines()[-6:]))
    return proc.returncode


def _carpos(printer, tag):
    """跑一次巡检，返回 `(verdict, body_yaw_deg, margin_point, margin_env)`。

    读 `carpos.jsonl` 最后一行（`carpos.py` 每次都会追加一条）。跑不动 → `RUN_FAILED`。
    """
    rc = _run_cmd([sys.executable, CARPOS, tag, "9"], CARPOS_TIMEOUT_S, printer)
    log = HERE / "carpos.jsonl"
    if rc != 0 or not log.exists():
        return "RUN_FAILED", None, None, None
    try:
        last = json.loads(log.read_text(encoding="utf-8").strip().splitlines()[-1])
    except (ValueError, IndexError):
        return "RUN_FAILED", None, None, None
    return (str(last.get("verdict")), last.get("body_yaw_deg"),
            last.get("margin_point_mm"), last.get("margin_envelope_mm"))


def recover(printer, tag):
    """越界/靠边后的固定恢复动作：归位到场地中心 → 复检。返回复检的 verdict。

    `recenter.py` 失败（连车都定位不了）就**不再往下走** —— 那已经是 `DEADLOCK`，
    只能人工搬。本函数只走 `recenter` + `carpos`，不发任何实验命令。
    """
    printer(f"—— 恢复：归位到 (0,0) ——")
    rc = _run_cmd([sys.executable, RECENTER, f"rec_{tag}", "0", "0"],
                  RECENTER_TIMEOUT_S, printer)
    if rc != 0:
        printer(f"⚠ 归位失败（rc={rc}）—— 车已定位不了，请人工搬回场地中央。")
        return "RECOVER_FAILED"
    verdict, yaw, mp, me = _carpos(printer, f"recpost_{tag}")
    printer(f"  复检：verdict={verdict} body_yaw={yaw}° "
            f"margin_point={mp} mm margin_env={me} mm")
    return verdict


def run_exp(exp_id, plan, *, printer=print, do_recenter=True, do_carpos=True,
            extra_args=()):
    """跑 `exp_id` 名下的所有补跑条目。返回 `{"ran", "skipped", "stopped"}` 记账。

    每一条都是独立进程：**相机与链路同一时刻只开一路**（简报要求"你一个人用板子"）。
    """
    todo = [p for p in plan if int(p["experiment_id"]) == int(exp_id)]
    if not todo:
        printer(f"实验 #{exp_id:02d} 没有补跑条目。")
        return {"ran": 0, "skipped": 0, "stopped": None, "n_out_of_field": 0}
    exp = exp_by_id(exp_id)
    stats = {"ran": 0, "skipped": 0, "stopped": None, "n_out_of_field": 0}

    for i, item in enumerate(todo, start=1):
        rep = int(item["repeat"])
        tag = f"r{exp_id:02d}_{rep}"
        printer("")
        printer(f"================ 补跑 #{exp_id:02d} rep{rep + 1}/{exp.repeats} "
                f"{exp.desc} ================")

        if do_recenter:
            rc = _run_cmd([sys.executable, RECENTER, f"pre_{tag}", "0", "0"],
                          RECENTER_TIMEOUT_S, printer)
            if rc != 0:
                # 归位失败 = 车现在在哪不知道 ⇒ 开电机就是盲开。这条不跑。
                printer(f"⚠ 归位未收敛（rc={rc}）—— #{exp_id:02d} rep{rep + 1} 不跑。")
                stats["skipped"] += 1
                continue

        want = {rep}
        argv = [sys.executable, "run_all_experiments.py", "--only", str(exp_id),
                *BATCH_ARGS, *extra_args,
                # `--trials` 的 rep 是 **1 起**（表里/命令里都 1 起），内部下标 0 起。
                "--trials", ",".join(f"{exp_id}:{r + 1}" for r in sorted(want))]
        rc = _run_cmd(argv, TRIAL_TIMEOUT_S, printer)
        stats["ran"] += 1
        if rc not in (0, 1):
            printer(f"⚠ 跑批进程 rc={rc}（非 0/1）—— 中止本 chunk。")
            stats["stopped"] = f"exit_{rc}"
            break

        if do_carpos:
            verdict, yaw, mp, me = _carpos(printer, f"post_{tag}")
            printer(f"  巡检：verdict={verdict} body_yaw={yaw}° "
                    f"margin_point={mp} mm margin_env={me} mm")
            if verdict in HARD_STOP_VERDICTS or verdict == "RUN_FAILED":
                printer(f"⚠ 巡检 {verdict} —— 中止 #{exp_id:02d} 剩余 {len(todo) - i} 条，"
                        "视觉死锁没有自救手段，请人工把车搬回场地中央。")
                stats["stopped"] = verdict
                break
            if verdict in RECOVER_VERDICTS:
                # 越线/靠边：停本 chunk → 归位 → 复检。复检不 OK 就不许再跑。
                stats["n_out_of_field"] += 1
                printer(f"⚠ 巡检 {verdict} —— 中止 #{exp_id:02d} 剩余 {len(todo) - i} 条，"
                        "先归位复检。")
                after = recover(printer, tag)
                if after not in OK_VERDICTS:
                    printer(f"⚠ 复检 {after} —— 不许再跑，请人工把车搬回场地中央。")
                    stats["stopped"] = f"recover_{after}"
                break
            if yaw is not None and abs(float(yaw)) > 45.0:
                printer(f"⚠ 车体 yaw {float(yaw):+.1f}° 很大 —— 下一条之前 "
                        "recenter 会先摆头；若归位时出现 gained 为负的脉冲，立刻停手报告。")
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(description="Task 12 Step 4 补跑（只补失败/没跑的条目）")
    ap.add_argument("--write-plan", action="store_true", help="离线：算出补跑清单并冻结到 rerun_plan.json")
    ap.add_argument("--plan", action="store_true", help="离线：打印当前补跑清单")
    ap.add_argument("--exp", type=int, default=None, metavar="ID", help="真机：跑这条实验名下的补跑条目")
    ap.add_argument("--trials", default=None, metavar="ID:REP,...", help="覆盖冻结清单（不走 --exp）")
    ap.add_argument("--no-recenter", action="store_true", help="⚠ 调试用：跳过每条之前的归位")
    ap.add_argument("--no-carpos", action="store_true", help="⚠ 调试用：跳过每条之后的巡检")
    ap.add_argument("--archive", action="store_true", help="给当前 xlsx 打一份时间戳归档")
    ap.add_argument("--from-row", type=int, default=24, help="口径：第几条汇总行起算本轮（默认 24）")
    args = ap.parse_args(argv)

    if args.archive:
        print(f"已归档：{compare.archive_workbook(XLSX)}")
        return 0
    if args.write_plan:
        plan = write_plan(from_row=args.from_row)
        print(f"补跑清单 → {PLAN_PATH}（{len(plan)} 条）")
        for p in plan:
            why = p["previous_reason"] or "上一轮没跑到"
            print(f"  #{p['experiment_id']:02d} rep{p['repeat'] + 1}：{why}")
        return 0

    plan = parse_trials(args.trials) if args.trials else load_plan()
    if args.plan or args.exp is None:
        print(f"补跑清单（{len(plan)} 条）：")
        for p in plan:
            print(f"  #{p['experiment_id']:02d} rep{p['repeat'] + 1}")
        if args.exp is None:
            return 0

    exp_ids = sorted({int(p["experiment_id"]) for p in plan
                      if int(p["experiment_id"]) == int(args.exp)})
    if not exp_ids:
        print(f"实验 #{args.exp:02d} 不在补跑清单里。")
        return 0
    total = {"ran": 0, "skipped": 0, "n_out_of_field": 0}
    for exp_id in exp_ids:
        s = run_exp(exp_id, plan, do_recenter=not args.no_recenter,
                    do_carpos=not args.no_carpos)
        total["ran"] += s["ran"]
        total["skipped"] += s["skipped"]
        total["n_out_of_field"] += s["n_out_of_field"]
        if s["stopped"]:
            print(f"⚠ #{exp_id:02d} 中止：{s['stopped']}")
            return 1
    print(f"\n#0{args.exp} 补跑结束：跑 {total['ran']} 条、跳过 {total['skipped']} 条"
          f"（归位未收敛）、包络越线 {total['n_out_of_field']} 次。")
    return 0


if __name__ == "__main__":
    sys.exit(main())