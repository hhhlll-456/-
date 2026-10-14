"""`vgt.matrix` 的行为锁：动作上限 + 场地几何（R15）+ 矩阵覆盖。

**前五个用例与 brief 的 Step 1 逐字一致**（R15 要求它们原样跑通）：
`Experiment` 保持 8 字段 namedtuple、默认起点 = 名义场地中心（0, 0）。

R15 的几何校验分两个分支（完整理由见 `vgt/matrix.py` 的模块 docstring）：

- 默认 `start_xy=(0.0, 0.0)`（名义中心、起点未归位）：只查**全场可行性**
  —— 位移 ≤ 235 mm (x) / 103 mm (y)，也就是"换一个合适的偏心起点就一定放得下"。
  直行 200 从**正中心**出发物理上放不下（单侧净余量只有 117.5 / 51.4 mm），
  所以"绝对扫掠校验"留给 Task 12 传真实归位起点的分支；默认分支若也做绝对校验，
  这几个用例会当场全红。
- 显式 `start_xy`（Task 12 归位后传入）：车体半包络 135 × 74.5 mm 在**起点与终点**
  都必须落在 ±252.5 × ±125.9 mm 内。
"""
from __future__ import annotations

import math

import pytest

from vgt import matrix


# --- brief Step 1 的四条 + repeats 用例（逐字，不许改） ----------------------------


def test_no_experiment_exceeds_field_clearance():
    for e in matrix.EXPERIMENTS:
        matrix.assert_within_field(e)          # 不抛异常即通过


def test_all_rotations_are_at_most_45_deg():
    for e in matrix.EXPERIMENTS:
        if e.action == "rotate_in_place":
            assert abs(e.om) * e.duration_s * 180/math.pi <= 45.0 + 1e-9


def test_strafe_step_is_at_most_80mm():
    for e in matrix.EXPERIMENTS:
        if e.action == "strafe":
            assert abs(e.vy) * e.duration_s * 1000 <= 80.0 + 1e-9


def test_straight_distance_is_at_most_200mm():
    for e in matrix.EXPERIMENTS:
        if e.action == "straight":
            assert abs(e.vx) * e.duration_s * 1000 <= 200.0 + 1e-9


def test_every_experiment_repeats_at_least_three_times():
    assert all(e.repeats >= 3 for e in matrix.EXPERIMENTS)


# --- 辅助 -----------------------------------------------------------------------


def _dx_mm(e):
    return e.vx * e.duration_s * 1000.0


def _dy_mm(e):
    return e.vy * e.duration_s * 1000.0


def _dpsi_deg(e):
    return e.om * e.duration_s * 180.0 / math.pi


def _probe(action, vx, vy, om, duration_s=1.0, exp_id=99, repeats=3):
    """造一条测试用的实验（desc 只是占位符）。"""
    return matrix.Experiment(exp_id, action, "探针", vx, vy, om, duration_s, repeats)


def _sign(x):
    return 1 if x > 0 else -1


# --- 矩阵内容：覆盖与唯一性 -------------------------------------------------------


def test_experiment_namedtuple_has_exactly_the_eight_fields():
    """R15：namedtuple 字段与 brief 一字不差（Task 12 按字段名解包）。"""
    assert matrix.Experiment._fields == (
        "id", "action", "desc", "vx", "vy", "om", "duration_s", "repeats")


def test_matrix_covers_the_mandated_manoeuvres():
    """brief 的覆盖要求：直行 ±200/±150、横移 ±80/±50、斜行 ±120、旋转 ±30°/±45°。"""
    straight = sorted(round(_dx_mm(e), 6) for e in matrix.EXPERIMENTS
                      if e.action == "straight")
    assert straight == pytest.approx([-200.0, -150.0, 150.0, 200.0])

    strafe = sorted(round(_dy_mm(e), 6) for e in matrix.EXPERIMENTS
                    if e.action == "strafe")
    assert strafe == pytest.approx([-80.0, -50.0, 50.0, 80.0])

    rotate = sorted(round(_dpsi_deg(e), 6) for e in matrix.EXPERIMENTS
                    if e.action == "rotate_in_place")
    assert rotate == pytest.approx([-45.0, -30.0, 30.0, 45.0])

    # 斜行：四个对角方向都覆盖，位移都是 120 mm（45°）
    diagonal = [e for e in matrix.EXPERIMENTS if e.action == "diagonal"]
    assert {(_sign(_dx_mm(e)), _sign(_dy_mm(e))) for e in diagonal} == {
        (1, 1), (1, -1), (-1, 1), (-1, -1)}
    for e in diagonal:
        assert math.hypot(_dx_mm(e), _dy_mm(e)) == pytest.approx(120.0, abs=1e-9)


def test_experiment_ids_are_unique_and_actions_are_known():
    ids = [e.id for e in matrix.EXPERIMENTS]
    assert len(ids) == len(set(ids)), f"id 有重复：{ids}"
    assert all(e.action in {"straight", "strafe", "diagonal", "rotate_in_place"}
               for e in matrix.EXPERIMENTS)
    assert all(e.desc for e in matrix.EXPERIMENTS), "每条都要有一句人话 desc"


# --- R15 的两个分支 ---------------------------------------------------------------


def test_default_start_checks_full_field_slack_not_the_centre_envelope():
    """默认 (0,0) = 名义场地中心：只查"全场可行性"（位移 ≤ 235 / 103），不查绝对扫掠。

    直行 200 从正中心出发物理上放不下（单侧净余量 117.5 mm）—— 这正是 R15
    把起点做成参数的原因；默认分支若做绝对校验，brief 的测试 1 立刻红。
    """
    e200 = _probe("straight", 0.200, 0.0, 0.0)
    matrix.assert_within_field(e200)                       # 默认起点：不抛
    matrix.assert_within_field(e200, start_xy=(-100.0, 0.0))   # 偏心起点可执行
    with pytest.raises(ValueError):
        matrix.assert_within_field(e200, start_xy=(50.0, 0.0))    # 终点包络越 x 界
    with pytest.raises(ValueError):
        matrix.assert_within_field(e200, start_xy=(0.0, 60.0))    # 起点包络越 y 界


def test_reference_start_uses_the_car_half_envelope_at_both_ends():
    """显式起点的硬校验：半包络 135 × 74.5 在起点与终点都必须在场。"""
    e100 = _probe("straight", 0.100, 0.0, 0.0)
    matrix.assert_within_field(e100, start_xy=(-117.5, 0.0))   # 左边缘贴边：可以
    with pytest.raises(ValueError):
        matrix.assert_within_field(e100, start_xy=(-117.51, 0.0))  # 差 0.01 mm：不行


def test_mandated_strafe_is_reachable_from_a_real_start():
    """R15 的理由句：横移 80 从正中心放不下（净余量 51.4），但偏心起点可以。"""
    e80 = _probe("strafe", 0.0, 0.080, 0.0)
    matrix.assert_within_field(e80, start_xy=(0.0, -40.0))     # y 包络 ±114.5 < ±125.9
    with pytest.raises(ValueError):
        matrix.assert_within_field(e80, start_xy=(0.0, -20.0))     # 终点 y 包络 134.5 越界
    with pytest.raises(ValueError):
        matrix.assert_within_field(e80, start_xy=(0.0, -60.0))     # 起点 y 包络越界


def test_mandated_diagonal_is_reachable_from_a_real_start():
    e = _probe("diagonal", 0.12 / math.sqrt(2), 0.12 / math.sqrt(2), 0.0)
    matrix.assert_within_field(e, start_xy=(-100.0, -40.0))


# --- 上限与非法输入 ---------------------------------------------------------------


def test_assert_within_field_rejects_over_cap_motions():
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("straight", 0.250, 0.0, 0.0))      # 250 > 200
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("strafe", 0.0, 0.090, 0.0))        # 90 > 80
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("diagonal", 0.09, 0.09, 0.0))      # 127.3 > 120
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("rotate_in_place", 0.0, 0.0, 1.0))  # 57.3° > 45°
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("hover", 0.0, 0.0, 0.0))           # 未知动作


def test_assert_within_field_rejects_impure_actions_and_bad_duration():
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("straight", 0.1, 0.05, 0.0))
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("strafe", 0.1, 0.05, 0.0))
    with pytest.raises(ValueError):
        matrix.assert_within_field(_probe("rotate_in_place", 0.0, 0.0, 0.5, duration_s=0.0))
