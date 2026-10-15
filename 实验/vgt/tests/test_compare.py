"""`vgt.compare`（Task 12）的行为锁：14 列对比入表 + 有符号判据 + 归档。

不碰硬件、不碰真表：工作簿一律由生成器 `实验/实验数据记录_build.py` 在 `tmp_path`
里产出（"xlsx 只能由生成器脚本产出"的既有约定），真
`实验/实验数据记录.xlsx` 永远不被这些测试写。
"""
from __future__ import annotations

import importlib.util
import math
from datetime import datetime
from pathlib import Path

import openpyxl
import pytest

from vgt import compare, matrix

REPO = Path(__file__).resolve().parents[3]          # <repo>/实验/vgt/tests → <repo>
BUILDER_PATH = REPO / "实验" / "实验数据记录_build.py"

# 既有 31 列：一个不许多、一个不许少、顺序不许变（Task 12 只许在末尾加列）。
ORIGINAL_COLS = [
    "experiment_id", "action_type", "command_description", "command_param_1",
    "command_param_2", "duration_planned", "t1", "timestamp",
    "pwm_fl", "pwm_fr", "pwm_rl", "pwm_rr",
    "encoder_fl", "encoder_fr", "encoder_rl", "encoder_rr",
    "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz",
    "imu_roll", "imu_pitch", "imu_yaw",
    "manual_distance", "manual_angle",
    "manual_final_x", "manual_final_y", "manual_final_yaw",
    "notes",
]

# Task 12 的 14 个新列（计划原文的顺序，接在 31 之后）。
VISION_COLS = [
    "vision_start_x", "vision_start_y", "vision_start_yaw",
    "vision_end_x", "vision_end_y", "vision_end_yaw",
    "vision_dist_mm", "vision_yaw_deg",
    "odo_dist_mm", "odo_yaw_deg",
    "dist_diff_pct", "yaw_diff_deg",
    "trial_ok", "trial_reason",
]


def _load_builder():
    """按路径加载生成器；它已重构成 `main()` 收口的模块（import 无副作用）。"""
    spec = importlib.util.spec_from_file_location("实验数据记录_build", BUILDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def builder():
    return _load_builder()


@pytest.fixture()
def xlsx(tmp_path, builder):
    """本测试专用的新表（由生成器产出，45 列表头、无数据行）。"""
    path = tmp_path / "实验数据记录.xlsx"
    builder.write_workbook(path)
    return path


def _count_formulas(wb) -> int:
    """整本工作簿里以 `=` 开头的字符串单元格数（"表里绝对不能有公式"）。"""
    n = 0
    for ws in wb.worksheets:
        for row in ws.iter_rows():
            for cell in row:
                if isinstance(cell.value, str) and cell.value.startswith("="):
                    n += 1
    return n


def _rows_as_dicts(ws):
    hdr = [cell.value for cell in ws[1]]
    return [dict(zip(hdr, [cell.value for cell in ws[r]]))
            for r in range(2, ws.max_row + 1)]


def _ok_trial(dist_mm=200.0, samples=None, warnings=None, sign_ok_missing=True):
    """一条成功的 trial 结果（形状与 `Harness.run_trial` 的返回一致）。"""
    trial = {
        "ok": True, "reason": None,
        "vision": {"start": (0.0, 0.0, 0.0), "mid": None,
                   "end": (dist_mm, 0.0, 0.0)},
        "vision_delta": {"dist_mm": dist_mm, "yaw_deg": 0.0,
                         "dx_mm": dist_mm, "dy_mm": 0.0},
        "odometry_delta": {"dist_mm": dist_mm, "yaw_deg": 0.0,
                           "dx_mm": dist_mm, "dy_mm": 0.0},
        "samples": list(samples or []),
        "warnings": list(warnings or []),
    }
    if not sign_ok_missing:
        trial["sign_ok"] = True
    return trial


def _samples_3():
    pwm = [-110, 110, -110, 110]
    return [
        {"t_ms": 10, "pwm": list(pwm), "enc": [0, 0, 0, 0], "rpy": [0.0, 0.0, 0.0]},
        {"t_ms": 510, "pwm": list(pwm), "enc": [9, 9, 9, 9], "rpy": [0.0, 0.0, 0.0]},
        {"t_ms": 990, "pwm": list(pwm), "enc": [19, 19, 19, 19], "rpy": [0.1, 0.2, 0.3]},
    ]


# ------------------------------------------------------ 生成器：14 个新列（Step 1）

def test_builder_appends_exactly_the_14_vision_columns(builder):
    """新增列的顺序/数量/表头正确：45 列 = 既有 31（逐字不动）+ 14（顺序固定）。"""
    names = [c[0] for c in builder.COLS]
    assert len(names) == 45
    assert names[:31] == ORIGINAL_COLS
    assert names[31:] == VISION_COLS
    # compare 与生成器的列名单不许漂移（两个模块各写一份是故意的：
    # 生成器不 import vgt，这里用测试把两份钉在一起）。
    assert list(compare.VISION_COLS) == VISION_COLS


def test_generated_workbook_has_the_new_header_and_no_formulas(xlsx, builder):
    wb = openpyxl.load_workbook(xlsx)
    assert wb.sheetnames[:3] == ["data", "字段说明", "实验清单"]
    ws = wb["data"]
    assert [cell.value for cell in ws[1]] == [c[0] for c in builder.COLS]
    assert ws.max_row == 1                       # 新建：只有表头
    assert wb["字段说明"].max_row == 1 + len(builder.COLS)
    assert _count_formulas(wb) == 0


def test_upgrade_keeps_data_rows_and_extends_the_header(tmp_path, builder):
    """重跑生成器 = 就地升级：既有数据行一行不丢，表头补齐到 45 列。"""
    path = tmp_path / "实验数据记录.xlsx"
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "data"
    ws.append(ORIGINAL_COLS)
    ws.append([5, "straight", "前进 0.30 m"] + [None] * 28)
    ws.append([5, "straight", "前进 0.30 m"] + [None] * 28)
    wb.save(path)

    builder.write_workbook(path)                 # 生成器必须能安全重跑

    wb2 = openpyxl.load_workbook(path)
    ws2 = wb2["data"]
    assert [cell.value for cell in ws2[1]] == [c[0] for c in builder.COLS]
    assert ws2.max_row == 3                      # 两行旧数据一行不少
    assert ws2.cell(row=2, column=1).value == 5
    assert ws2.cell(row=2, column=3).value == "前进 0.30 m"
    assert "字段说明" in wb2.sheetnames and "实验清单" in wb2.sheetnames
    assert _count_formulas(wb2) == 0

    builder.write_workbook(path)                 # 幂等：再跑一次还是 3 行
    assert openpyxl.load_workbook(path)["data"].max_row == 3


def test_upgrade_refuses_a_header_that_is_not_the_known_prefix(tmp_path, builder):
    """安全闸门：既有表头不是 COLS 前缀（列名被人改过）时拒绝升级，不许静默贴错。"""
    path = tmp_path / "坏表.xlsx"
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "data"
    ws.append(["experiment_id", "被改过的第二列"] + ORIGINAL_COLS[2:])
    wb.save(path)

    with pytest.raises(RuntimeError, match="表头"):
        builder.write_workbook(path)


# -------------------------------------------------------------- trial_summary（纯计算）

def test_default_xlsx_points_at_the_repo_workbook():
    assert compare.DEFAULT_XLSX == REPO / "实验" / "实验数据记录.xlsx"


def test_trial_summary_computes_all_14_fields_in_column_order():
    trial = {
        "ok": True, "reason": None,
        "vision": {"start": (0.0, 0.0, 10.0), "mid": None, "end": (100.0, 5.0, 12.0)},
        "odometry_delta": {"dist_mm": 98.0, "yaw_deg": 2.2,
                           "dx_mm": 97.8, "dy_mm": -6.3},
        "sign_ok": True,
    }
    s = compare.trial_summary(trial)

    assert list(s) == VISION_COLS                # 顺序 = 新列顺序（表头怎么排，值怎么排）
    assert s["vision_start_x"] == 0.0
    assert s["vision_start_y"] == 0.0
    assert s["vision_start_yaw"] == 10.0
    assert s["vision_end_x"] == 100.0
    assert s["vision_end_y"] == 5.0
    assert s["vision_end_yaw"] == 12.0
    vd = math.hypot(100.0, 5.0)
    assert s["vision_dist_mm"] == pytest.approx(vd)
    assert s["vision_yaw_deg"] == pytest.approx(2.0)          # 12 − 10
    assert s["odo_dist_mm"] == 98.0
    assert s["odo_yaw_deg"] == 2.2
    assert s["dist_diff_pct"] == pytest.approx((98.0 - vd) / vd * 100.0)
    assert s["yaw_diff_deg"] == pytest.approx(2.0 - 2.2)
    assert s["trial_ok"] is True
    assert s["trial_reason"] is None


def test_signed_check_catches_translation_flip_when_distances_match():
    """复核 Important 3 的硬要求：量级相同、方向相反 —— 只比 dist_mm 会静默通过。

    视觉说 +X 200 mm、编码器说 −X 200 mm：`dist_diff_pct ≈ 0`；
    没有 `dot(vision, odo) > 0` 的有符号判据时这条会被当成成功写进表。
    """
    trial = {
        "ok": True, "reason": None,              # 生产方只比了标量距离，放行了
        "vision": {"start": (0.0, 0.0, 0.0), "mid": None, "end": (200.0, 0.0, 0.0)},
        "vision_delta": {"dist_mm": 200.0, "yaw_deg": 0.0,
                         "dx_mm": 200.0, "dy_mm": 0.0},
        "odometry_delta": {"dist_mm": 200.0, "yaw_deg": 0.0,
                           "dx_mm": -200.0, "dy_mm": 0.0},
        # 特意不带 sign_ok：只有本模块自己重算才抓得住
    }
    s = compare.trial_summary(trial)

    assert s["dist_diff_pct"] == pytest.approx(0.0)       # 标量距离：一模一样
    assert s["trial_ok"] is False
    assert s["trial_reason"] == "sign_mismatch"


def test_harness_sign_ok_false_cannot_be_overridden_by_ok_true():
    trial = _ok_trial(sign_ok_missing=False)
    trial["sign_ok"] = False                              # harness 判过：方向反
    s = compare.trial_summary(trial)
    assert s["trial_ok"] is False
    assert s["trial_reason"] == "sign_mismatch"


def test_short_vectors_are_not_judged_by_the_signed_check():
    """门槛以下（< 5 mm）方向无意义：不判、不冤枉（与 harness.translation_sign_ok 同口径）。"""
    trial = {
        "ok": True, "reason": None,
        "vision": {"start": (0.0, 0.0, 0.0), "mid": None, "end": (2.0, 0.0, 0.0)},
        "odometry_delta": {"dist_mm": 2.0, "yaw_deg": 0.0,
                           "dx_mm": -2.0, "dy_mm": 0.0},
    }
    s = compare.trial_summary(trial)
    assert s["trial_ok"] is True
    assert s["trial_reason"] is None


# --------------------------------------------------------------------- write_trial

def test_write_trial_appends_sample_rows_plus_one_summary_row(xlsx):
    exp = matrix.EXPERIMENTS[0]
    trial = _ok_trial(samples=_samples_3())
    ws_before = openpyxl.load_workbook(xlsx)["data"]
    n_before = ws_before.max_row

    assert compare.write_trial(trial, exp, xlsx_path=xlsx) is None

    ws = openpyxl.load_workbook(xlsx)["data"]
    assert ws.max_row == n_before + 4            # 3 个样本行 + 1 个汇总行

    r0, r2, rsum = (_rows_as_dicts(ws)[i] for i in (0, 2, 3))
    assert r0["experiment_id"] == 1
    assert r0["action_type"] == "straight"
    assert r0["command_description"] == exp.desc
    assert r0["duration_planned"] == pytest.approx(float(exp.duration_s))   # R68：0.5
    assert r0["timestamp"] == pytest.approx(0.010)
    assert [r0["pwm_fl"], r0["pwm_fr"], r0["pwm_rl"], r0["pwm_rr"]] == [-110, 110, -110, 110]
    assert [r0["encoder_fl"], r0["encoder_fr"],
            r0["encoder_rl"], r0["encoder_rr"]] == [0, 0, 0, 0]
    assert r0["notes"] is None                   # 样本行不是汇总行
    assert r2["timestamp"] == pytest.approx(0.990)
    assert [r2["imu_roll"], r2["imu_pitch"], r2["imu_yaw"]] == pytest.approx([0.1, 0.2, 0.3])
    assert r2["t1"] == pytest.approx(0.990)      # t1 = 实际结束（末样本时刻）

    assert rsum["timestamp"] is None             # 汇总行不占样本时间
    assert rsum["notes"] == "汇总行"
    assert rsum["vision_start_x"] == 0.0
    assert rsum["vision_end_x"] == 200.0
    assert rsum["vision_dist_mm"] == pytest.approx(200.0)
    assert rsum["odo_dist_mm"] == pytest.approx(200.0)
    assert rsum["dist_diff_pct"] == pytest.approx(0.0)
    assert rsum["trial_ok"] is True              # 真布尔值，不是字符串
    assert rsum["trial_reason"] is None
    assert _count_formulas(openpyxl.load_workbook(xlsx)) == 0


def test_write_trial_keeps_failed_trials_and_never_overwrites(xlsx):
    """`trial_ok=False` 不许丢：照样写行、`trial_reason` 标注原因；既有行不被覆盖。"""
    exp = matrix.EXPERIMENTS[0]
    compare.write_trial(_ok_trial(samples=_samples_3()), exp, xlsx_path=xlsx)
    ws1 = openpyxl.load_workbook(xlsx)["data"]
    n1 = ws1.max_row

    failed = {
        "ok": False, "reason": "board_reset",
        "vision": {"start": None, "mid": None, "end": None},
        "vision_delta": None, "odometry_delta": None,
        "samples": [], "warnings": [],
    }
    compare.write_trial(failed, exp, xlsx_path=xlsx)

    ws = openpyxl.load_workbook(xlsx)["data"]
    assert ws.max_row == n1 + 1                  # 失败也写一行（汇总是唯一一行）
    rows = _rows_as_dicts(ws)
    assert rows[-1]["trial_ok"] is False
    assert rows[-1]["trial_reason"] == "board_reset"
    assert rows[-1]["vision_dist_mm"] is None
    assert rows[-1]["odo_dist_mm"] is None
    # 前一条试验的汇总行原样还在（append 不覆盖）：
    assert rows[-2]["trial_ok"] is True
    assert rows[-2]["vision_dist_mm"] == pytest.approx(200.0)
    assert rows[-2]["t1"] == pytest.approx(0.990)


def test_write_trial_refuses_a_31_column_workbook(tmp_path):
    """防静默错位：旧 31 列表直接报错（先跑生成器升级表头），不许把 45 列数据贴上旧表。"""
    path = tmp_path / "旧表.xlsx"
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "data"
    ws.append(ORIGINAL_COLS)
    wb.save(path)

    with pytest.raises(RuntimeError, match="表头"):
        compare.write_trial(_ok_trial(), matrix.EXPERIMENTS[0], xlsx_path=path)


def test_write_trial_refuses_a_missing_workbook(tmp_path):
    with pytest.raises(FileNotFoundError):
        compare.write_trial(_ok_trial(), matrix.EXPERIMENTS[0],
                            xlsx_path=tmp_path / "没有这个文件.xlsx")


# --------------------------------------------------------------------- archive

def test_archive_workbook_copies_with_a_timestamped_name(xlsx):
    dst = compare.archive_workbook(xlsx, now=datetime(2026, 10, 15, 21, 30, 0))
    assert dst.name == "实验数据记录_20261015_213000.xlsx"
    assert dst.parent == xlsx.parent
    assert dst.read_bytes() == xlsx.read_bytes()
