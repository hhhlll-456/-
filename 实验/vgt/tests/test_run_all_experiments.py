"""`run_all_experiments`（Task 12 批量入口）：计划起点 + 严格场地校验 + mock 批量循环。

批量循环全部走假 harness（不连车、不开相机、不写真表）：
- `FakeHarness` 记录 `drive_to` 的目标、`run_trial` 的 id 与完整事件顺序；
- 工作簿一律由生成器在 `tmp_path` 里产出；
- `--dry` 路径断言**任何**硬件会话工厂都不会被调用。

需要真相机的项（一条真 RUN、真归位）不在这里 —— 计划的 Step 4 由人执行。
"""
from __future__ import annotations

import importlib.util
import math
from contextlib import contextmanager
from pathlib import Path

import openpyxl
import pytest

import run_all_experiments as RA
from vgt import matrix
from vgt.harness import DEFAULT_PULSE_MS as H_DEFAULT_PULSE_MS
from vgt.harness import StopFailed

REPO = Path(__file__).resolve().parents[3]
BUILDER_PATH = REPO / "实验" / "实验数据记录_build.py"


@pytest.fixture(scope="module")
def builder():
    spec = importlib.util.spec_from_file_location("实验数据记录_build", BUILDER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture()
def xlsx(tmp_path, builder):
    path = tmp_path / "实验数据记录.xlsx"
    builder.write_workbook(path)
    return path


# ------------------------------------------------------------------ 假设备（无网络）

class FakeCarLink:
    """只记录 `stop()`（批量循环"每条结束 car.stop()"的验收点）。"""

    def __init__(self, events):
        self.events = events
        self.stops = 0

    def stop(self):
        self.stops += 1
        self.events.append(("stop",))
        return "OK STOP"


class FakeHarness:
    def __init__(self, *, events=None, home_results=(), trial_results=(),
                 raise_stop_failed=False):
        self.events = [] if events is None else events
        self.car_link = FakeCarLink(self.events)
        self.home_results = list(home_results)
        self.trial_results = list(trial_results)
        self.raise_stop_failed = bool(raise_stop_failed)
        self.drive_targets = []
        self.drive_kwargs = []
        self.trial_ids = []

    def drive_to(self, x, y, **kw):
        self.drive_targets.append((x, y))
        self.drive_kwargs.append(dict(kw))
        self.events.append(("drive_to", (x, y)))
        if self.home_results:
            return dict(self.home_results.pop(0))
        return {"reached": True, "reason": "converged", "at_target": True,
                "moved": True, "iters": 1}

    def run_trial(self, exp, **kw):
        self.trial_ids.append(exp.id)
        self.events.append(("run_trial", exp.id))
        if self.raise_stop_failed:
            raise StopFailed("fake：STOP 未确认，立即给车断电")
        return self.trial_results.pop(0)


def _probe(exp_id=1, action="straight", vx=0.2, vy=0.0, om=0.0,
           duration_s=None, repeats=1):
    """造一条测试用的实验；时长默认按目标行程反解（R68：命令量惰性，见 matrix）。"""
    if duration_s is None:
        if action == "rotate_in_place":
            duration_s = matrix.ms_for_deg(abs(om) * 180.0 / math.pi) / 1000.0
        else:
            duration_s = matrix.ms_for_mm(math.hypot(vx, vy) * 1000.0) / 1000.0
    return matrix.Experiment(exp_id, action, f"探针 #{exp_id}",
                             vx, vy, om, duration_s, repeats)


def _ok_trial(dist_mm=200.0):
    return {
        "ok": True, "reason": None,
        "vision": {"start": (0.0, 0.0, 0.0), "mid": None,
                   "end": (dist_mm, 0.0, 0.0)},
        "vision_delta": {"dist_mm": dist_mm, "yaw_deg": 0.0,
                         "dx_mm": dist_mm, "dy_mm": 0.0},
        "odometry_delta": {"dist_mm": dist_mm, "yaw_deg": 0.0,
                           "dx_mm": dist_mm, "dy_mm": 0.0},
        "samples": [], "warnings": [],
    }


def _failed_trial(reason):
    return {
        "ok": False, "reason": reason,
        "vision": {"start": None, "mid": None, "end": None},
        "vision_delta": None, "odometry_delta": None,
        "samples": [], "warnings": [],
    }


def _summary_rows(ws):
    hdr = [cell.value for cell in ws[1]]
    return [dict(zip(hdr, [cell.value for cell in ws[r]]))
            for r in range(2, ws.max_row + 1)]


# ------------------------------------------------------------------ 计划起点（R15）

def test_plan_start_puts_the_sweep_midpoint_at_the_field_origin():
    # 容差 = 1 ms 的量化尺（R68：时长按实测速度模型向下取整到 1 ms ⇒ 行程差 < 0.6 mm）
    tol = 0.5
    by_id = {e.id: e for e in matrix.EXPERIMENTS}
    assert RA.plan_start_xy(by_id[1]) == pytest.approx((-100.0, 0.0), abs=tol)  # 前进 200
    assert RA.plan_start_xy(by_id[2]) == pytest.approx((100.0, 0.0), abs=tol)   # 后退 200
    assert RA.plan_start_xy(by_id[3]) == pytest.approx((-75.0, 0.0), abs=tol)   # 前进 150
    assert RA.plan_start_xy(by_id[4]) == pytest.approx((75.0, 0.0), abs=tol)
    assert RA.plan_start_xy(by_id[5]) == pytest.approx((0.0, -40.0), abs=tol)   # 左移 80
    assert RA.plan_start_xy(by_id[6]) == pytest.approx((0.0, 40.0), abs=tol)    # 右移 80
    e9 = by_id[9]                                                       # 斜行 120（45°）
    dx9, dy9 = matrix.expected_delta_mm(e9)
    assert RA.plan_start_xy(e9) == pytest.approx((-dx9 / 2.0, -dy9 / 2.0))
    assert RA.plan_start_xy(e9) == pytest.approx((-42.37, -42.37), abs=0.05)
    assert RA.plan_start_xy(by_id[15]) == (0.0, 0.0)                    # 原地旋转
    assert RA.plan_start_xy(by_id[16]) == (0.0, 0.0)


def test_planned_starts_engage_the_strict_branch_and_are_all_feasible():
    """规划起点必须可执行，而且是真的走**严格分支**（不是静默落回宽松）。

    两个断言缺一不可：
    1. 平移类起点不许是 (0,0) —— 即使显式传 (0,0)，`assert_within_field` 也会
       走宽松分支（它只看"换一个偏心起点放得下"），绝对扫掠校验根本不执行；
    2. 把规划起点整体推向场外 200 mm 必须被拒 —— 证明这个显式起点真的吃进了
       绝对扫掠校验（直行 200 的终点包络 200+135 > 252.5）。
    """
    for exp in matrix.EXPERIMENTS:
        start_xy = RA.plan_start_xy(exp)
        RA.check_experiment_feasible(exp, start_xy)                  # 不抛 = 通过
        if exp.action != "rotate_in_place":
            assert start_xy != (0.0, 0.0), (
                f"#{exp.id} 的规划起点不许是名义中心：assert_within_field 对 (0,0) "
                "走宽松分支，平移类会静默跳过绝对扫掠校验（Important 1）")
        with pytest.raises(ValueError):
            matrix.assert_within_field(
                exp, start_xy=(start_xy[0] + 200.0, start_xy[1]))


def test_missing_start_is_an_error_and_the_default_branch_is_a_real_trap():
    """Important 1：`assert_within_field` 的默认 (0.0,0.0) 会静默走宽松分支。

    本任务选"Task 12 侧显式传执行起点 + 缺起点报错"（不动 matrix.py，理由见
    `run_all_experiments` 模块 docstring 与交付报告）。
    """
    e5 = next(e for e in matrix.EXPERIMENTS if e.id == 5)            # 左移 80

    # 陷阱本身是真的：不传起点 → 宽松分支 → 下面这个越界的偏心起点"静默通过"
    matrix.assert_within_field(e5)
    # 同一个实验、显式起点走严格分支 → 立刻抓住"起点就贴在场外"
    with pytest.raises(ValueError):
        matrix.assert_within_field(e5, start_xy=(0.0, -60.0))
    # Task 12 的堵法：缺起点是显式错误，绝不落回默认
    with pytest.raises(ValueError, match="起点"):
        RA.check_experiment_feasible(e5, None)
    # 规划起点本体必须过
    RA.check_experiment_feasible(e5, RA.plan_start_xy(e5))


def test_iterate_trials_yields_every_repeat_and_only_filters_by_id():
    trials = list(RA.iterate_trials(matrix.EXPERIMENTS))
    assert len(trials) == sum(e.repeats for e in matrix.EXPERIMENTS) == 48
    assert trials[0][0].id == 1 and trials[0][1] == 0

    only3 = list(RA.iterate_trials(matrix.EXPERIMENTS, only=3))
    assert [e.id for e, _rep in only3] == [3, 3, 3]
    assert [rep for _e, rep in only3] == [0, 1, 2]
    assert list(RA.iterate_trials(matrix.EXPERIMENTS, only=999)) == []


# ------------------------------------------------------------------ 批量循环（mock）

def test_batch_loop_homes_runs_writes_prints_and_stops_per_trial(tmp_path, builder, xlsx):
    events = []
    exp1 = _probe(1, repeats=2)
    exp5 = _probe(5, action="strafe", vx=0.0, vy=0.08)
    h = FakeHarness(events=events,
                    trial_results=[_ok_trial(200.0), _ok_trial(200.0), _ok_trial(80.0)])
    lines = []

    def write(trial, exp, **kw):
        events.append(("write", exp.id))

    stats = RA.run_batch(h, experiments=[exp1, exp5], only=None, xlsx_path=xlsx,
                         printer=lines.append, write=write, archive=False)

    def _near(ev):                                        # drive_to 目标取整毫米
        kind, *rest = ev                                  # （R68：时长 1 ms 量化）
        if kind == "drive_to":
            return (kind, (round(rest[0][0]), round(rest[0][1])))
        return tuple(ev)

    assert [(round(x), round(y)) for x, y in h.drive_targets] == [
        (-100, 0), (-100, 0), (0, -40)]
    assert h.trial_ids == [1, 1, 5]
    assert [_near(e) for e in events] == [                # 计划要求的每条顺序
        ("drive_to", (-100, 0)), ("run_trial", 1), ("write", 1), ("stop",),
        ("drive_to", (-100, 0)), ("run_trial", 1), ("write", 1), ("stop",),
        ("drive_to", (0, -40)), ("run_trial", 5), ("write", 5), ("stop",),
    ]
    assert h.car_link.stops == 3                          # 每条结束 car.stop()
    assert stats == {"n_trials": 3, "n_ok": 3, "n_failed": 0,
                     "n_homing_failed": 0, "n_homing_zero_pulse": 0,
                     "archive_path": None}
    assert len(lines) == 3 and all("OK" in ln for ln in lines)
    assert any("200.0mm" in ln or "200.0 mm" in ln or "200.0" in ln for ln in lines)


def test_batch_loop_records_a_failed_trial_and_keeps_going(tmp_path, builder, xlsx):
    exp1 = _probe(1)
    exp5 = _probe(5, action="strafe", vx=0.0, vy=0.08)
    h = FakeHarness(trial_results=[_ok_trial(200.0), _failed_trial("boundary_moved")])
    lines = []

    stats = RA.run_batch(h, experiments=[exp1, exp5], xlsx_path=xlsx,
                         printer=lines.append, archive=False)

    assert h.trial_ids == [1, 5]                  # 失败不中断后续
    assert stats["n_ok"] == 1 and stats["n_failed"] == 1
    assert any("FAIL" in ln and "boundary_moved" in ln for ln in lines)

    rows = _summary_rows(openpyxl.load_workbook(xlsx)["data"])
    assert rows[0]["trial_ok"] is True
    assert rows[1]["trial_ok"] is False
    assert rows[1]["trial_reason"] == "boundary_moved"
    assert rows[1]["vision_dist_mm"] is None


def test_homing_failure_skips_the_trial_but_records_the_row(tmp_path, builder, xlsx):
    """归位失败 = 车在哪不知道：不许开电机跑实验，但失败要留痕（trial_reason=homing_*）。"""
    exp1 = _probe(1)
    h = FakeHarness(home_results=[{"reached": False, "reason": "locate_failed"}])
    lines = []

    stats = RA.run_batch(h, experiments=[exp1], xlsx_path=xlsx,
                         printer=lines.append, archive=False)

    assert h.trial_ids == []                      # 没跑实验
    assert h.car_link.stops == 1                  # 但照常收尾停车
    assert stats["n_failed"] == 1 and stats["n_homing_failed"] == 1
    assert "homing_locate_failed" in lines[0]

    rows = _summary_rows(openpyxl.load_workbook(xlsx)["data"])
    assert len(rows) == 1
    assert rows[0]["trial_ok"] is False
    assert rows[0]["trial_reason"] == "homing_locate_failed"


# ------------------------------------------- 归位脉冲宽度（真机：400 ms 必振荡）

def test_batch_passes_the_homing_pulse_width_through_to_drive_to(tmp_path, builder, xlsx):
    """归位脉冲宽度必须能传下去 —— 默认 400 ms 在真机上**不收敛**（Task 10 实测）。

    `drive_to` 默认 400 ms：单脉冲步长 131.8–148.2 mm ≫ `tol_mm=12`，12 次迭代走成
    周期 2 极限环（"有时收敛"比稳定不收敛更危险）；只有 150–175 ms 窗口收敛。
    批量入口原来把这个参数写死在默认档上，没法用真机验证过的值。
    """
    exp1 = _probe(1, repeats=2)
    h = FakeHarness(trial_results=[_ok_trial(200.0), _ok_trial(200.0)])

    RA.run_batch(h, experiments=[exp1], xlsx_path=xlsx, pulse_ms=150,
                 printer=lambda _s: None, archive=False)

    assert h.drive_kwargs == [{"pulse_ms": 150.0}] * 2


def test_batch_default_pulse_matches_the_harness_default(tmp_path, builder, xlsx):
    """不给 `--pulse-ms` 时不许偷偷换档：与 `harness.DEFAULT_PULSE_MS` 同值。"""
    exp1 = _probe(1)
    h = FakeHarness(trial_results=[_ok_trial(200.0)])

    RA.run_batch(h, experiments=[exp1], xlsx_path=xlsx,
                 printer=lambda _s: None, archive=False)

    from vgt import harness as H
    assert h.drive_kwargs == [{"pulse_ms": H.DEFAULT_PULSE_MS}]


def test_cli_pulse_ms_is_optional_and_parsed(xlsx):
    assert RA.parse_args([]).pulse_ms == H_DEFAULT_PULSE_MS
    assert RA.parse_args(["--pulse-ms", "150"]).pulse_ms == 150.0
    h = FakeHarness(trial_results=[_ok_trial(200.0)] * 3)

    @contextmanager
    def factory():
        yield h

    rc = RA.main(["--only", "1", "--pulse-ms", "150"], session_factory=factory,
                 printer=lambda _s: None, xlsx_path=xlsx)

    assert rc == 0
    assert h.drive_kwargs == [{"pulse_ms": 150.0}] * 3
    with pytest.raises(SystemExit):
        RA.parse_args(["--pulse-ms", "abc"])


# ------------------------------------------- 零脉冲归位（R67：静默假成功的堵法）

def test_classify_homing_separates_real_success_from_zero_pulse():
    """`drive_to` 的结果不是一个布尔值能说清的：三类必须分得开。"""
    # 闭环真的动过并收敛
    assert RA.classify_homing({"reached": True, "reason": "converged", "moved": True,
                               "iters": 2}) == ("ok", None)
    # 零迭代（新版 drive_to：reached=False + already_at_target）
    assert RA.classify_homing({"reached": False, "reason": "already_at_target",
                               "at_target": True, "moved": False, "iters": 0}) \
        == ("zero_pulse", "already_at_target")
    # 老字典/手造结果：只有 reached、明确说没动过 ⇒ 一律降级，绝不当成功
    assert RA.classify_homing({"reached": True, "reason": "converged",
                               "moved": False}) == ("zero_pulse", "already_at_target")
    assert RA.classify_homing({"reached": True, "iters": 0}) == ("zero_pulse", "already_at_target")
    # 真没归到位
    assert RA.classify_homing({"reached": False, "reason": "max_iter", "iters": 12}) \
        == ("failed", "max_iter")
    assert RA.classify_homing({"reached": False, "reason": None}) == ("failed", "unknown")
    # 退化兼容：老结果既没有 moved 也没有 iters，但 reached=True ⇒ 只能当成功
    assert RA.classify_homing({"reached": True, "reason": "converged"}) == ("ok", None)


def test_zero_pulse_homing_runs_the_trial_but_is_never_counted_as_success(tmp_path, builder, xlsx):
    """零脉冲归位：**跑实验**（车实测在容差内，照常安全），但不许记成"归位成功"。

    静默假数据（R67）的堵法：既不跳条（车确实在位，数据是真的），也不当成功 ——
    打印行带醒目标记、计数单独统计、汇总行的 notes 里留痕（表里筛得出来）。
    """
    exp1 = _probe(1)
    h = FakeHarness(trial_results=[_ok_trial(200.0)],
                    home_results=[{"reached": False, "reason": "already_at_target",
                                  "at_target": True, "moved": False, "iters": 0,
                                  "start_dist_mm": 4.1}])
    lines = []

    stats = RA.run_batch(h, experiments=[exp1], xlsx_path=xlsx,
                         printer=lines.append, archive=False)

    assert h.trial_ids == [1]                    # 实验照跑（车在容差内）
    assert stats["n_homing_zero_pulse"] == 1
    assert stats["n_homing_failed"] == 0         # 不是"归位失败"：车确实在位
    assert stats["n_ok"] == 1                    # 数据本身有效
    assert "零脉冲" in lines[0] and "already_at_target" in lines[0]

    rows = _summary_rows(openpyxl.load_workbook(xlsx)["data"])
    assert rows[0]["trial_ok"] is True           # trial 本身合格
    assert "homing_zero_pulse" in str(rows[0]["notes"])     # 但归位零脉冲留了痕


def test_zero_pulse_that_is_not_at_target_is_a_homing_failure(tmp_path, builder, xlsx):
    """零迭代 + 没到容差内（`at_target` 假）= 真没归位 ⇒ 跳条、留痕。"""
    exp1 = _probe(1)
    h = FakeHarness(home_results=[{"reached": False, "reason": "already_at_target",
                                   "at_target": False, "moved": False, "iters": 0}])

    stats = RA.run_batch(h, experiments=[exp1], xlsx_path=xlsx,
                         printer=lambda _s: None, archive=False)

    assert h.trial_ids == []
    assert stats["n_homing_failed"] == 1 and stats["n_homing_zero_pulse"] == 0
    rows = _summary_rows(openpyxl.load_workbook(xlsx)["data"])
    assert rows[0]["trial_reason"] == "homing_already_at_target"


def test_stop_failed_aborts_the_whole_batch(tmp_path, builder, xlsx):
    """StopFailed = 车可能还在开：中止整批（不许当 trial 级失败吞掉），收尾照样 stop。"""
    exp1 = _probe(1)
    exp5 = _probe(5, action="strafe", vx=0.0, vy=0.08)
    h = FakeHarness(raise_stop_failed=True)

    with pytest.raises(StopFailed):
        RA.run_batch(h, experiments=[exp1, exp5], xlsx_path=xlsx,
                     printer=lambda _s: None, archive=False)

    assert h.trial_ids == [1]                     # 第二条不跑
    assert h.car_link.stops == 1                  # finally 里停过
    assert openpyxl.load_workbook(xlsx)["data"].max_row == 1     # 没写任何行


def test_batch_archives_the_workbook_when_asked(tmp_path, builder, xlsx):
    h = FakeHarness(trial_results=[_ok_trial(200.0)])
    stats = RA.run_batch(h, experiments=[_probe(1)], xlsx_path=xlsx,
                         printer=lambda _s: None, archive=True)
    assert stats["archive_path"] is not None
    archive = Path(stats["archive_path"])
    assert archive.exists() and archive.parent == xlsx.parent
    assert archive.name.startswith("实验数据记录_")


# ------------------------------------------------------------------ CLI（--dry / --only）

def test_dry_run_prints_every_trial_and_touches_no_hardware(monkeypatch):
    def boom():
        raise AssertionError("--dry 不许打开任何硬件会话")

    monkeypatch.setattr(RA, "_open_real_session", boom)
    lines = []
    rc = RA.main(["--dry", "--only", "1"], printer=lines.append)

    assert rc == 0
    assert len(lines) == 4                        # #1 的 3 次 + 收尾一行
    assert all("#01" in ln for ln in lines[:3])
    assert all("起点" in ln for ln in lines[:3])
    assert "DRY" in lines[-1]


def test_unknown_only_id_is_rejected_without_side_effects():
    lines = []
    rc = RA.main(["--dry", "--only", "999"], printer=lines.append)
    assert rc == 2
    assert any("999" in ln for ln in lines)


def test_main_runs_the_batch_through_the_injected_session(xlsx):
    exp1 = _probe(1)
    h = FakeHarness(trial_results=[_ok_trial(200.0)] * 3)

    @contextmanager
    def factory():
        yield h

    lines = []
    rc = RA.main(["--only", "1"], session_factory=factory,
                 printer=lines.append, xlsx_path=xlsx)

    assert rc == 0
    assert h.trial_ids == [exp1.id] * 3           # 真矩阵里的 #1 是 repeats=3
    assert h.car_link.stops == 3
    assert openpyxl.load_workbook(xlsx)["data"].max_row == 4      # 3 个汇总行
    assert any("完成" in ln for ln in lines)


def test_main_returns_nonzero_when_a_trial_failed(xlsx):
    h = FakeHarness(trial_results=[_ok_trial(200.0), _failed_trial("no_telemetry"),
                                   _ok_trial(200.0)])

    @contextmanager
    def factory():
        yield h

    rc = RA.main(["--only", "1"], session_factory=factory,
                 printer=lambda _s: None, xlsx_path=xlsx)
    assert rc == 1                                # 有失败条目 → 退出码 1（表里留痕）
