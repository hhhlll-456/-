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
    return matrix.expected_delta_mm(e)[0]


def _dy_mm(e):
    return matrix.expected_delta_mm(e)[1]


def _dpsi_deg(e):
    return matrix.expected_yaw_deg(e)


def _probe(action, vx, vy, om, duration_s=None, exp_id=99, repeats=3):
    """造一条测试用的实验（desc 只是占位符）。

    `duration_s=None`（默认）按**目标行程**反解时长（R68：命令速度惰性，真机行程只由
    时长决定）：平移取 `|v|·1 s` mm、旋转取 `|om|·1 s` 度，与老口径的"命令量"等值。
    """
    if duration_s is None:
        if action == "rotate_in_place":
            duration_s = matrix.ms_for_deg(abs(om) * 180.0 / math.pi) / 1000.0
        else:
            duration_s = matrix.ms_for_mm(math.hypot(vx, vy) * 1000.0) / 1000.0
    return matrix.Experiment(exp_id, action, "探针", vx, vy, om, duration_s, repeats)


def _sign(x):
    return 1 if x > 0 else -1


# --- 矩阵内容：覆盖与唯一性 -------------------------------------------------------


def test_experiment_namedtuple_has_exactly_the_eight_fields():
    """R15：namedtuple 字段与 brief 一字不差（Task 12 按字段名解包）。"""
    assert matrix.Experiment._fields == (
        "id", "action", "desc", "vx", "vy", "om", "duration_s", "repeats")


def test_matrix_covers_the_mandated_manoeuvres():
    """brief 的覆盖要求：直行 ±200/±150、横移 ±80/±50、斜行 ±120、旋转 ±30°/±45°。

    位移取**期望行程**（R68：命令量惰性，真机行程由 `duration_s` 决定），
    取整数毫米比对：1 ms 的量化误差 < 1 mm。
    """
    straight = sorted(round(_dx_mm(e)) for e in matrix.EXPERIMENTS
                      if e.action == "straight")
    assert straight == [-200, -150, 150, 200]

    strafe = sorted(round(_dy_mm(e)) for e in matrix.EXPERIMENTS
                    if e.action == "strafe")
    assert strafe == [-80, -50, 50, 80]

    rotate = sorted(round(_dpsi_deg(e)) for e in matrix.EXPERIMENTS
                    if e.action == "rotate_in_place")
    assert rotate == [-45, -30, 30, 45]

    # 斜行：四个对角方向都覆盖，位移都是 120 mm（45°）
    diagonal = [e for e in matrix.EXPERIMENTS if e.action == "diagonal"]
    assert {(_sign(_dx_mm(e)), _sign(_dy_mm(e))) for e in diagonal} == {
        (1, 1), (1, -1), (-1, 1), (-1, -1)}
    for e in diagonal:
        assert round(math.hypot(_dx_mm(e), _dy_mm(e))) == 120


def test_durations_are_the_measured_speed_model_inverted():
    """R68：`duration_s` 全部由 `s = 0.56·(T−0.143)` 反解（向下取整）。

    时长是**唯一**能改变真机行程的量（命令速度被 MIN_DUTY 抬平），所以这张表
    必须锁住：改矩阵时长 = 改实验项目，不能顺手四舍五入。
    """
    assert matrix.ms_for_mm(200) == 500
    assert matrix.ms_for_mm(150) == 410
    assert matrix.ms_for_mm(80) == 285
    assert matrix.ms_for_mm(50) == 232
    assert matrix.ms_for_mm(120) == 357
    assert matrix.ms_for_deg(30) == 266
    assert matrix.ms_for_deg(45) == 328
    for e in matrix.EXPERIMENTS:
        if e.action == "rotate_in_place":
            want = matrix.ms_for_deg(round(abs(_dpsi_deg(e))))
        else:
            want = matrix.ms_for_mm(round(math.hypot(_dx_mm(e), _dy_mm(e))))
        assert e.duration_s * 1000.0 == pytest.approx(want, abs=1e-6)


def test_expected_travel_rounds_down_under_the_target_and_the_cap():
    """向下取整 1 ms ⇒ 期望行程 ≤ 目标 ≤ 上限（差 < 1 mm，横移 80 才不顶穿）。"""
    for e in matrix.EXPERIMENTS:
        if e.action == "rotate_in_place":
            got = abs(_dpsi_deg(e))
            cap = matrix.MAX_ROTATION_DEG
        else:
            got = math.hypot(_dx_mm(e), _dy_mm(e))
            cap = {"straight": matrix.MAX_STRAIGHT_MM,
                   "strafe": matrix.MAX_STRAFE_MM,
                   "diagonal": matrix.MAX_DIAGONAL_MM}[e.action]
        target = round(got)
        assert target - 1.0 < got <= target + 1e-9, f"#{e.id} 期望 {got} vs 目标 {target}"
        assert got <= cap + 1e-9, f"#{e.id} 期望 {got} 超过上限 {cap}"


def test_expected_start_xy_is_the_mirror_midpoint_of_the_expected_leg():
    """`expected_start_xy` = −**场地系**期望位移/2（行程对中）；旋转原地不动。

    ⚠ R69：对中必须用场地系那一侧。车体系 +y（车左）在场地系里是 −Y —— 真机实测
    #05「横移左移 80 mm」起点 y = −34.9 → 终点 y = −66.3（往 −Y 走）。
    """
    for e in matrix.EXPERIMENTS:
        sx, sy = matrix.expected_start_xy(e)
        dx, dy = matrix.field_delta_mm(e)
        assert sx == pytest.approx(-dx / 2.0, abs=1e-9)
        assert sy == pytest.approx(-dy / 2.0, abs=1e-9)
        if e.action == "rotate_in_place":
            assert (sx, sy) == (0.0, 0.0)
        else:
            assert (sx, sy) != (0.0, 0.0), f"#{e.id} 平移类起点不许是 (0, 0)"


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
    """R15 的理由句：横移 80 从正中心放不下（净余量 51.4），但偏心起点可以。

    ⚠ 起点要取**车会走的那一侧的对面**（R69：`vy>0` = 车左 = 场地 −Y）：横移左移
    80 mm 从 `y = +40` 出发往 −Y 走到 `y = −40`，y 包络 ±114.5 < ±125.9。
    """
    e80 = _probe("strafe", 0.0, 0.080, 0.0)
    matrix.assert_within_field(e80, start_xy=(0.0, 40.0))      # y 包络 ±114.5 < ±125.9
    with pytest.raises(ValueError):
        matrix.assert_within_field(e80, start_xy=(0.0, 20.0))  # 终点 y 包络 134.0 越界
    with pytest.raises(ValueError):
        matrix.assert_within_field(e80, start_xy=(0.0, 60.0))  # 起点 y 包络越界


def test_mandated_diagonal_is_reachable_from_a_real_start():
    e = _probe("diagonal", 0.12 / math.sqrt(2), 0.12 / math.sqrt(2), 0.0)
    matrix.assert_within_field(e, start_xy=(-100.0, 40.0))     # 斜行前左：vx>0、vy>0


def test_body_to_field_delta_matches_the_harness_sign_chain():
    """R69：车体系 → 场地系的符号链必须与 `harness.body_delta_to_field` 逐字一致。

    真机锚点：#05「横移左移 80 mm」（vy>0）起点 y = −34.9 → 终点 y = −66.3，**往 −Y 走**。
    """
    from vgt.harness import body_delta_to_field
    for dx, dy in ((0.0, 79.5), (79.5, 0.0), (84.7, 84.7), (-50.0, 25.0)):
        for yaw in (0.0, 17.0, -42.0, 90.0):
            assert matrix.body_to_field_delta(dx, dy, yaw) == pytest.approx(
                body_delta_to_field(dx, dy, yaw))
    # 车左 = 场地 −Y（yaw=0）；车头 = 场地 +X
    assert matrix.body_to_field_delta(0.0, 10.0) == pytest.approx((0.0, -10.0))
    assert matrix.body_to_field_delta(10.0, 0.0) == pytest.approx((10.0, 0.0))


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
