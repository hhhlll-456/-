"""独立重建 Task 12 数据集（不复用 opencode 的 dataset.json，用于交叉核对）。

汇总行 = `trial_ok` 非空的行（每个 trial 的最后一行）。
"""
import pathlib as _pl
ROOT = _pl.Path(__file__).resolve().parents[2]   # 仓库根（麦轮小车pid/）

import json
import openpyxl

XLSX = str(ROOT / str(ROOT / "实验/实验数据记录.xlsx"))
OUT = str(_pl.Path(__file__).with_name("dataset.json"))
import pathlib  # noqa: E402

wb = openpyxl.load_workbook(XLSX, read_only=True, data_only=True)
ws = wb["data"]
rows = list(ws.iter_rows(values_only=True))
hdr = list(rows[0])
idx = {h: i for i, h in enumerate(hdr)}

COLS = [
    "experiment_id", "action_type", "command_description", "command_param_1",
    "command_param_2", "duration_planned", "t1", "pwm_fl", "pwm_fr", "pwm_rl",
    "pwm_rr", "encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr",
    "imu_yaw", "vision_start_x", "vision_start_y", "vision_start_yaw",
    "vision_end_x", "vision_end_y", "vision_end_yaw", "vision_dist_mm",
    "vision_yaw_deg", "odo_dist_mm", "odo_yaw_deg", "dist_diff_pct",
    "yaw_diff_deg", "trial_ok", "trial_reason", "notes",
]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


out = []
for ri, r in enumerate(rows[1:], start=2):     # xlsx 行号从 1 开始（表头=1）
    if r[idx["trial_ok"]] in (None, ""):
        continue
    d = {"xlsx_row": ri}
    for c in COLS:
        v = r[idx[c]]
        if c in ("pwm_fl", "pwm_fr", "pwm_rl", "pwm_rr", "encoder_fl",
                 "encoder_fr", "encoder_rl", "encoder_rr", "imu_yaw",
                 "t1", "duration_planned", "command_param_1", "command_param_2"):
            v = f(v)
        elif c in ("vision_dist_mm", "vision_yaw_deg", "odo_dist_mm",
                   "odo_yaw_deg", "dist_diff_pct", "yaw_diff_deg",
                   "vision_start_x", "vision_start_y", "vision_start_yaw",
                   "vision_end_x", "vision_end_y", "vision_end_yaw"):
            v = f(v)
        d[c] = v
    out.append(d)

json.dump(out, open(OUT, "w"), ensure_ascii=False, indent=1)

n = len(out)
ok = [d for d in out if str(d["trial_ok"]).strip().upper() in ("TRUE", "1", "OK")]
print(f"汇总行 n={n}  trial_ok 真={len(ok)}  失败={n - len(ok)}")
from collections import Counter
print("action:", dict(Counter(d["action_type"] for d in out)))
print("action(ok):", dict(Counter(d["action_type"] for d in ok)))
print("reason:", dict(Counter(str(d["trial_reason"]) for d in out if str(d["trial_reason"]).strip() not in ("", "None"))))
num = [d for d in ok if d["dist_diff_pct"] is not None]
print(f"有对比列的 OK 行 n={len(num)}")
