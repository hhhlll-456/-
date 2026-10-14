#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R63 护栏（`wheelmap_sweep.body_rotation_check` / `analyze`）的主机侧行为锁。

全合成样本、零硬件：直接用 `Sample` 构造 I 行序列（`I t pwm0..3 enc0..3 rpy`），
不连小车、不碰串口/TCP。

测试要能因实现错误而失败 —— 每个判据都配了会给出**相反结论**的对照样本：
  - 正常旋转 → 通过；悬空空转（编码器在涨、yaw 不动）→ 必须 invalid；
  - ±180 回绕 → 正确累加（把累加改成"首尾相减"或不 wrap，本测试必红）；
  - 噪声级 yaw 抖动 → 不得被当成"在转"；车轮也不转的真死区档 → 不误杀；
  - straight / air / 不传 mode 的旧调用 → 不启用（不被误杀）；
    同一批"悬空"样本按 pivot 判必须是 invalid —— 证明按 pattern/mode 选判据真的生效。

运行：
    python3 -m pytest 实验/tests/test_wheelmap_sweep_motion.py -q
"""
import math
import types

import pytest

import wheelmap_sweep as ws

DT_MS = 10                        # 固件 I 行 ~100 Hz（与现场一致）
T0 = 1_000_000                    # 板载 millis 起点（大数，贴近现场）
PWM_PIVOT = [110, 110, 110, 110]  # 落地 pivot 的请求（duty=110，落地表最低档）
C1REV = ws.DEFAULT_C1REV


def make_samples(n=61, *, yaw0=0.0, yaw_rate=0.0, yaws=None, enc_rate=13.0,
                 pwm=None, t0=T0, dt_ms=DT_MS):
    """造 n 行 I 样本（默认 61 行 × 10 ms = 0.6 s 窗）。

    - `yaws` 给了就用逐样本 yaw 序列（测回绕），否则 yaw = yaw0 + yaw_rate·t；
    - `enc_rate` > 0 时四轮计数按该轮速（rad/s）匀速增长（"轮子在转"）；
      = 0 时计数整段不动（真死区/静置）。
    """
    pwm = list(PWM_PIVOT if pwm is None else pwm)
    out = []
    for i in range(n):
        t = t0 + i * dt_ms
        y = yaws[i] if yaws is not None else yaw0 + yaw_rate * (t - t0) / 1000.0
        enc = [round(enc_rate * (t - t0) / 1000.0 * C1REV[w] / (2.0 * math.pi))
               for w in range(4)]
        out.append(ws.Sample("I", t, list(pwm), enc, [0.0, 0.0, y]))
    return out


def check(win, *, enforced=True, min_turn_deg=ws.DEFAULT_MIN_TURN_DEG, min_omega=0.5):
    return ws.body_rotation_check(win, enforced=enforced, c1rev=C1REV,
                                  min_omega=min_omega, min_turn_deg=min_turn_deg)


# ============================================================ 判据（纯函数）
def test_threshold_is_the_documented_3deg():
    # 阈值是文档化契约（来源见 wheelmap_sweep.DEFAULT_MIN_TURN_DEG 注释）；
    # 调它就必须同步解释/测试，不许悄悄漂。
    assert ws.DEFAULT_MIN_TURN_DEG == pytest.approx(3.0)


def test_car_really_rotating_passes():
    # 落地 duty=110 量级：车体 ~195°/s（0.6 s 转 ~117°）而轮速 13 rad/s
    win = make_samples(61, yaw_rate=195.0, enc_rate=13.0)
    r = check(win)
    assert r["enforced"] is True and r["wheels_turned"] is True
    assert r["ok"] is True and r["reason"] is None
    assert r["turn_deg"] == pytest.approx(117.0, abs=0.5)
    assert r["turn_rate_deg_s"] == pytest.approx(195.0, abs=2.0)


def test_lifted_car_encoder_spins_yaw_flat_is_invalid():
    # 今天踩的坑：四轮悬空空转（16 rad/s），编码器涨、yaw 一动不动
    win = make_samples(61, yaw_rate=0.0, enc_rate=16.0)
    r = check(win)
    assert r["wheels_turned"] is True          # 编码器确实在转……
    assert r["ok"] is False                    # ……所以必须判"车没动"
    assert "no_body_rotation" in r["reason"]
    assert "车没转" in r["reason"] and "悬空" in r["reason"]   # 人话原因（R63 立据）


def test_deadzone_wheels_still_no_invalid():
    # 真死区：车轮也不转（Δenc≈0）→ ω 记 0 是真实值，不许判 invalid
    win = make_samples(61, yaw_rate=0.0, enc_rate=0.0)
    r = check(win)
    assert r["wheels_turned"] is False
    assert r["ok"] is True and r["reason"] is None


def test_yaw_wrap_positive_crossing():
    # 从 +150° 起以 2.5°/样本顺时针转，中段跨过 +180 → 应正确累计到 +150°
    yaws = [((150.0 + 2.5 * i + 180.0) % 360.0) - 180.0 for i in range(61)]
    r = check(make_samples(61, yaws=yaws, enc_rate=13.0))
    assert r["turn_deg"] == pytest.approx(150.0, abs=1e-6)   # 首尾相减会得到 -212.5°
    assert r["ok"] is True


def test_yaw_wrap_negative_crossing():
    # 从 −150° 起以 −3°/样本转，中段跨过 −180 → 应正确累计到 −180°
    yaws = [((-150.0 - 3.0 * i + 180.0) % 360.0) - 180.0 for i in range(61)]
    r = check(make_samples(61, yaws=yaws, enc_rate=13.0))
    assert r["turn_deg"] == pytest.approx(-180.0, abs=1e-6)
    assert r["ok"] is True


def test_noise_level_yaw_jitter_is_not_rotation():
    # 真实静帧那种抖动（±0.05°，IMU 量化 0.01°）+ 悬空轮子空转：
    # 抖动不得被当成"在转" —— 必须 invalid，且累计转角是噪声量级
    yaws = [0.05 * math.sin(i * 1.7) for i in range(61)]
    r = check(make_samples(61, yaws=yaws, enc_rate=16.0))
    assert abs(r["turn_deg"]) < 1.0            # ≪ 3.0° 阈值
    assert r["ok"] is False
    assert "no_body_rotation" in r["reason"]


def test_noise_level_yaw_jitter_with_still_wheels_ok():
    # 静置 + 抖动（车轮不转）：正常死亡区档，不是 R63 要管的事
    yaws = [0.05 * math.sin(i * 1.7) for i in range(61)]
    r = check(make_samples(61, yaws=yaws, enc_rate=0.0))
    assert r["ok"] is True


def test_threshold_boundary_both_sides():
    # 同一构造只差转角：2.5° < 3.0° → invalid；3.5° > 3.0° → ok
    lo = [i * (2.5 / 60.0) for i in range(61)]
    hi = [i * (3.5 / 60.0) for i in range(61)]
    assert check(make_samples(61, yaws=lo, enc_rate=13.0))["ok"] is False
    assert check(make_samples(61, yaws=hi, enc_rate=13.0))["ok"] is True


def test_guard_enables_if_any_wheel_spins():
    # 只要有一轮在转（哪怕其余三轮不动），就不许把空转当"车在动"
    win = make_samples(61, yaw_rate=0.0, enc_rate=16.0)
    for s in win:
        for w in (1, 2, 3):
            s.enc[w] = 0
    r = check(win)
    assert r["wheels_turned"] is True and r["ok"] is False


def test_not_enforced_returns_ok_even_if_lifted_like():
    # enforced=False（air/straight）必须恒 ok —— 护栏不能误杀那些形态
    r = check(make_samples(61, yaw_rate=0.0, enc_rate=16.0), enforced=False)
    assert r["ok"] is True and r["reason"] is None
    assert r["turn_deg"] is not None           # 仍记录观测值（进 --raw-out）


# ============================================================ analyze() 集成
def analyze_samples(*, yaw_rate, enc_rate, n=121):
    """1.21 s 样本；t_ref=最后一个样本 ⇒ 测量窗 = 最后 0.6 s（60 行）。"""
    s = make_samples(n, yaw_rate=yaw_rate, enc_rate=enc_rate)
    return s, s[-1].t


def run_analyze(samples, t_ref, *, mode="ground", pattern="pivot", **kw):
    return ws.analyze(samples, 0, PWM_PIVOT, C1REV, t_ref, 600.0, 0.5,
                      mode=mode, pattern=pattern, **kw)


def test_analyze_ok_and_omega_unchanged_when_rotating():
    samples, t_ref = analyze_samples(yaw_rate=195.0, enc_rate=13.0)
    r = run_analyze(samples, t_ref)
    assert r["status"] == "ok"
    assert r["omega"] == pytest.approx(13.0, abs=0.15)   # ω 测量数学原样（Δcnt·2π/C/dt）
    assert r["rotation"]["enforced"] is True and r["rotation"]["ok"] is True


def test_analyze_invalid_when_lifted():
    samples, t_ref = analyze_samples(yaw_rate=0.0, enc_rate=16.0)
    r = run_analyze(samples, t_ref)
    assert r["status"] == "no_body_rotation"
    assert "车没转" in r["reason"] and "悬空" in r["reason"]
    assert r["omega_raw"] == pytest.approx(16.0, abs=0.2)  # 原始轮速留档（诊断用）
    assert r["rotation"]["wheels_turned"] is True


def test_analyze_straight_pattern_not_killed():
    # straight：车平移、yaw 基本不变 → 护栏退化、不得判死
    samples, t_ref = analyze_samples(yaw_rate=0.0, enc_rate=13.0)
    r = run_analyze(samples, t_ref, pattern="straight")
    assert r["status"] == "ok"
    assert r["rotation"]["enforced"] is False
    # 同一批样本按 pivot 判必须 invalid —— pattern 选判据必须真的生效
    assert run_analyze(samples, t_ref, pattern="pivot")["status"] == "no_body_rotation"


def test_analyze_air_mode_not_enforced():
    # 架空套：车本来就该悬空，逐轮空转是预期行为 → 不许启用护栏
    samples, t_ref = analyze_samples(yaw_rate=0.0, enc_rate=16.0)
    r = run_analyze(samples, t_ref, mode="air")
    assert r["status"] == "ok"
    assert r["rotation"]["enforced"] is False


def test_analyze_deadzone_still_ok():
    samples, t_ref = analyze_samples(yaw_rate=0.0, enc_rate=0.0)
    r = run_analyze(samples, t_ref)
    assert r["status"] == "ok"
    assert r["in_deadzone"] is True and r["omega"] == 0.0


def test_analyze_without_mode_kwargs_is_backward_compatible():
    # 旧调用形态（不传 mode/pattern）= 不启用护栏（默认参数必须向后兼容）
    samples, t_ref = analyze_samples(yaw_rate=0.0, enc_rate=16.0)
    r = ws.analyze(samples, 0, PWM_PIVOT, C1REV, t_ref, 600.0, 0.5)
    assert r["status"] == "ok"
    assert r["rotation"]["enforced"] is False


def test_exec_step_records_human_readable_invalid_reason(monkeypatch):
    """护栏不通过时，Sweeper 必须走既有 invalid 机制（生成器据此拒收整表）。"""
    samples, t_ref = analyze_samples(yaw_rate=0.0, enc_rate=16.0)

    def fake_measure(link, expect, dwell, first_match_timeout=1.5):
        return samples, {"status": "ok", "missing": [], "t_ref": t_ref}

    monkeypatch.setattr(ws, "measure_step", fake_measure)
    cfg = types.SimpleNamespace(
        mode="ground", pattern="pivot", duties=[110], dwell=1.2, window=0.6,
        c1rev=C1REV, min_omega=0.5, min_turn_deg=ws.DEFAULT_MIN_TURN_DEG)
    sw = ws.Sweeper(link=None, cfg=cfg, log=lambda *a, **k: None)
    expect = [110, 110, 110, 110]
    sw.exec_step(expect, "ground pivot d=110 sigma=+", {(0, "fwd"): (0, expect)})

    reason = sw.invalid[(0, 110, "fwd")]
    assert "no_body_rotation" in reason and "悬空" in reason
    assert sw.data["omega_fwd"][0][0] == 0.0          # 无效条目不许悄悄写表
    assert sw.raw[-1]["FL_fwd"]["status"] == "no_body_rotation"
