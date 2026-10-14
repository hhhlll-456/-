"""对比入表（Task 12）：`run_trial` 的结果 → `实验数据记录.xlsx` 的 14 个对比列。

## 一层图

    trial 结果(dict) ──trial_summary()──► 14 列汇总值 ──write_trial()──► data 表追加
                                            ▲
                             （有符号判据在这里再钉一遍）

- **样本行**：`trial["samples"]` 逐条追加（时间序列表：timestamp / pwm / 编码器 / IMU）。
- **汇总行**：每条约一行；14 个新列只在这里有值（样本行为空），事后筛"汇总行"
  就看这 14 列非空（同时 notes 写「汇总行」）。
- `trial_ok=False` 的条目**照样写行**，`trial_reason` 标注原因（与
  `harness.TRIAL_REASONS` 同词表，外加归位失败等 Task 12 侧原因），便于事后筛除。
- 表里只有值、没有公式；xlsx 由 `实验/实验数据记录_build.py` 产出/升级，
  本模块只 append 不重建（R16：重建会清空数据；生成器已改成保留数据的就地升级）。
- 追加前先验表头（14 个新列在尾段）——31 列旧表直接报错，防"45 列数据贴上旧表头"。

## 有符号判据（Task 9 复核 Important 3 的硬要求，Task 12 侧再钉一遍）

只比 `dist_mm`（标量）抓不住"平移符号整体翻转"：量级对、方向反时
`dist_diff_pct ≈ 0`，会静默通过。所以 `trial_summary()` 独立复核
`dot(vision_Δxy, odo_Δxy) > 0`（`harness.translation_sign_ok`，两边模长都 ≥ 5 mm
才判）：`trial["sign_ok"]` 缺失时自己重算；结果为 False 时强制
`trial_ok=False, trial_reason="sign_mismatch"`，**优先于 trial 自报的 ok**。
Task 11 的 harness 已在 `run_trial` 里判过一次；这里是第二道闸，防手工构造的
trial 字典（或将来别的生产者）绕过。

## 列布局（45 列）

既有 31 列不动，末尾接 14 列；`VISION_COLS` 与 `实验数据记录_build.py` 的
`COLS` 尾段**逐字一致**（生成器不 import 本包，两份名单由
`test_compare.py::test_builder_appends_exactly_the_14_vision_columns` 钉死）。
"""
from __future__ import annotations

import math
import shutil
from datetime import datetime
from pathlib import Path

import openpyxl

from .harness import body_delta_to_field, translation_sign_ok

SHEET_DATA = "data"

# Task 12 的 14 个新列（顺序 = 表头尾段顺序 = 汇总行的写入顺序）。
VISION_COLS = [
    "vision_start_x", "vision_start_y", "vision_start_yaw",
    "vision_end_x", "vision_end_y", "vision_end_yaw",
    "vision_dist_mm", "vision_yaw_deg",
    "odo_dist_mm", "odo_yaw_deg",
    "dist_diff_pct", "yaw_diff_deg",
    "trial_ok", "trial_reason",
]

_SUMMARY_NOTE = "汇总行"

# 第一列 = 首列名；本模块只按表头名核对，不负责生成（生成器的事）。
DEFAULT_XLSX = Path(__file__).resolve().parents[1] / "实验数据记录.xlsx"


def _wrap180_deg(deg: float) -> float:
    """归一到 (−180, 180]（与 `harness._wrap180_deg` / `vision_core.field._wrap180` 同口径）。"""
    wrapped = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def trial_summary(trial: dict) -> dict:
    """`Harness.run_trial` 的结果 → 14 个对比列的值（纯计算，不碰 I/O）。

    - 视觉位移优先用 `trial["vision_delta"]`；缺失时从 `vision.start/end` 现算
      （手工构造的 trial 字典也走得通）；
    - 里程计位移是**车体系**口径（`odometry.integrate_encoders`），有符号判据前
      按起点 yaw 转到场地系（`harness.body_delta_to_field`，与 harness 同一条符号链）；
    - `dist_diff_pct =（里程计−视觉）/ 视觉 × 100`（视觉 ≈ 0 时留空）；
      `yaw_diff_deg = 视觉 − 里程计`（归一到 (−180,180]）。
    """
    vision = trial.get("vision") or {}
    start = vision.get("start")
    end = vision.get("end")

    vdelta = trial.get("vision_delta")
    if vdelta is None and start is not None and end is not None:
        vdx = float(end[0]) - float(start[0])
        vdy = float(end[1]) - float(start[1])
        vdelta = {"dist_mm": math.hypot(vdx, vdy),
                  "yaw_deg": _wrap180_deg(float(end[2]) - float(start[2])),
                  "dx_mm": vdx, "dy_mm": vdy}

    odelta = trial.get("odometry_delta")

    # ---- 有符号判据：sign_ok 不是明确布尔值就自己重算（Important 3） -------------
    sign_ok = trial.get("sign_ok")
    if sign_ok in (True, False):
        sign_ok = bool(sign_ok)
    else:
        sign_ok = None
        if vdelta is not None and odelta is not None and start is not None:
            fdx, fdy = body_delta_to_field(odelta["dx_mm"], odelta["dy_mm"], start[2])
            sign_ok = translation_sign_ok((vdelta["dx_mm"], vdelta["dy_mm"]), (fdx, fdy))

    trial_ok = bool(trial.get("ok", False))
    reason = trial.get("reason")
    if trial_ok and sign_ok is False:
        trial_ok = False
        reason = "sign_mismatch"
    if trial_ok:
        reason = None                      # 通过的条目在表里理由留空
    elif reason is None:
        reason = "unknown_failure"         # 兜底：失败必须有名有姓，便于筛除

    vdist = None if vdelta is None else float(vdelta["dist_mm"])
    vyaw = None if vdelta is None else float(vdelta["yaw_deg"])
    odist = None if odelta is None else float(odelta["dist_mm"])
    oyaw = None if odelta is None else float(odelta["yaw_deg"])
    if vdist is None or odist is None or vdist == 0.0:
        dist_diff_pct = None
    else:
        dist_diff_pct = (odist - vdist) / vdist * 100.0
    if vyaw is None or oyaw is None:
        yaw_diff_deg = None
    else:
        yaw_diff_deg = _wrap180_deg(vyaw - oyaw)

    return {
        "vision_start_x": None if start is None else float(start[0]),
        "vision_start_y": None if start is None else float(start[1]),
        "vision_start_yaw": None if start is None else float(start[2]),
        "vision_end_x": None if end is None else float(end[0]),
        "vision_end_y": None if end is None else float(end[1]),
        "vision_end_yaw": None if end is None else float(end[2]),
        "vision_dist_mm": vdist,
        "vision_yaw_deg": vyaw,
        "odo_dist_mm": odist,
        "odo_yaw_deg": oyaw,
        "dist_diff_pct": dist_diff_pct,
        "yaw_diff_deg": yaw_diff_deg,
        "trial_ok": trial_ok,
        "trial_reason": reason,
    }


# ---- 行构造（列布局见 `实验数据记录_build.py` 的 COLS） ----------------------------


def _command_param_1(exp):
    """主要参数：直行/横移 = 带符号位移 mm；斜行 = 位移模长 mm；旋转 = 转角 deg。"""
    dur = float(exp.duration_s)
    if exp.action == "straight":
        return float(exp.vx) * dur * 1000.0
    if exp.action == "strafe":
        return float(exp.vy) * dur * 1000.0
    if exp.action == "diagonal":
        return math.hypot(float(exp.vx), float(exp.vy)) * dur * 1000.0
    if exp.action == "rotate_in_place":
        return float(exp.om) * dur * 180.0 / math.pi
    return None


def _command_param_2(exp):
    """次要参数：平移 = 速度模长 m/s；旋转 = |om| rad/s。"""
    if exp.action == "rotate_in_place":
        return abs(float(exp.om))
    return math.hypot(float(exp.vx), float(exp.vy))


def _sample_row(exp, sample: dict, t_end):
    """一个遥测样本 → 一行（31 列；14 个新列为空 —— 只有汇总行有值）。"""
    pwm = [int(v) for v in sample["pwm"]]
    enc = [int(v) for v in sample["enc"]]
    rpy = [float(v) for v in sample["rpy"]]
    return (
        [int(exp.id), str(exp.action), str(exp.desc),
         _command_param_1(exp), _command_param_2(exp), float(exp.duration_s), t_end,
         float(sample["t_ms"]) / 1000.0]
        + pwm + enc
        + [None] * 6                       # imu_ax..gz（GY-25T 不提供）
        + rpy                              # imu_roll/pitch/yaw
        + [None] * 5                       # manual_*
        + [None]                           # notes
    )


def _summary_row(exp, t_end, summary: dict, notes: str):
    """一条试验的汇总行：31 列（样本列留空）+ 14 个对比列。"""
    return (
        [int(exp.id), str(exp.action), str(exp.desc),
         _command_param_1(exp), _command_param_2(exp), float(exp.duration_s), t_end,
         None]                             # timestamp：汇总行不占样本时间
        + [None] * 4                       # pwm
        + [None] * 4                       # encoder
        + [None] * 6                       # imu_ax..gz
        + [None] * 3                       # imu_roll/pitch/yaw
        + [None] * 5                       # manual_*
        + [notes]
        + [summary[name] for name in VISION_COLS]
    )


def _validate_header(path: Path, ws) -> None:
    """表头必须是"既有 31 列 + 14 个新列在尾段"；不是就直接报错（不静默错位贴行）。"""
    header = [cell.value for cell in ws[1]]
    if header[31:31 + len(VISION_COLS)] != VISION_COLS:
        raise RuntimeError(
            f"{path} 的 data 表头缺 Task 12 的 14 个对比列（现有 {len(header)} 列）—— "
            "先跑 实验/实验数据记录_build.py 升级表头，再入表。")


def write_trial(trial: dict, exp, *, xlsx_path=DEFAULT_XLSX) -> None:
    """把一条 trial 追加进 `data` 表：样本行 ×N + 汇总行 ×1（就地 append，绝不覆盖）。

    失败条目（`ok=False`）照样写：汇总行 `trial_ok=False`、`trial_reason` 标注原因。
    表头不是 45 列（14 个新列缺位）时抛 `RuntimeError` —— 防"新行 45 列数据贴上
    旧 31 列表头"的静默错位（生成器升级后再入表）。
    """
    path = Path(xlsx_path)
    if not path.exists():
        raise FileNotFoundError(
            f"找不到数据表 {path} —— 先跑 实验/实验数据记录_build.py 生成。")
    wb = openpyxl.load_workbook(path)
    if SHEET_DATA not in wb.sheetnames:
        raise RuntimeError(f"{path} 里没有 {SHEET_DATA!r} 表 —— 不是本项目的实验数据表。")
    ws = wb[SHEET_DATA]
    _validate_header(path, ws)

    summary = trial_summary(trial)
    samples = list(trial.get("samples") or [])
    t_end = None
    if samples:
        t_ms = samples[-1].get("t_ms")
        if t_ms is not None:
            t_end = float(t_ms) / 1000.0       # t1 = 实际结束（末样本时刻）

    for sample in samples:
        ws.append(_sample_row(exp, sample, t_end))

    notes = _SUMMARY_NOTE
    warnings_ = [str(w) for w in (trial.get("warnings") or [])]
    if warnings_:
        notes += "：" + "；".join(warnings_)
    ws.append(_summary_row(exp, t_end, summary, notes))
    wb.save(path)


def archive_workbook(xlsx_path=DEFAULT_XLSX, *, dst=None, now=None) -> Path:
    """把当前工作簿拷成带日期时间戳的归档文件（R16：批跑完先留底，防误重建）。

    返回归档文件路径。`now` 可注入（测试用）；默认取本机时间。
    """
    src = Path(xlsx_path)
    if dst is None:
        stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
        dst = src.with_name(f"{src.stem}_{stamp}{src.suffix}")
    dst = Path(dst)
    shutil.copy2(src, dst)
    return dst
