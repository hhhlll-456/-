"""`vgt.odometry` 的行为锁：D 行解析 + 单相编码器里程计 + IMU yaw 取出。

## 编码器方向约定（R12/R13 的语义，这两条测试在这里把它钉死）

固件编码器**只有 A 相**（`main.cpp:74-87` 只有 `encCount[i]++`）：计数不辨向 ——
轮子往哪边转、转多少，计数都只涨。所以喂给 `integrate_encoders` 的 `enc`
一律是**原始无符号、单调递增**的计数（`_ramp()` 造）。

方向从哪来：同一行的 `pwm` 极性 × `sign`（固件 `invert={true,false,true,false}`，
spec §2.3 的 forward 符号 FL=−1 FR=+1 RL=−1 RR=+1）。固件的"前进"命令下发的
pwm 是 `sign·150`（FL/RL 反相）—— 与实车 exp1 的 `[-57,+53,-73,+49]` 同一形状，
`test_integrate_pure_forward_uses_both_sides` 的激励就照这个构造。"纯前进"的语义是
**四轮各自沿 sign 定义的正方向转**：对下的 pwm 符号就是 sign，计数等量增长。

**sign 只在 `integrate_encoders` 内部应用一次**：

    q_i = sign_i · sgn(pwm_i) · Δc_i        # 带符号计数增量

调用方**不许**再把 sign 乘进 enc（那是双重应用，R13：固件发的就是原始计数）。
把方向预乘进计数的旧写法会造出递减序列，函数按契约用 `ValueError` 拒绝
（`test_integrate_rejects_direction_baked_into_counts`）。

## 数值关系（独立于实现，用于钉结果）

- 一个计数 = 轮子滚过 `2πR / c_1rev` mm；四轮各滚 n 计数 → 直行 `n · 2πR / c_1rev`。
- 原地旋转：四轮等量反向滚 `d` → `yaw = d / LXY`（每个轮子都在半径 LXY 上）。
- LXY = (轴距 + 轮距)/2 = 128.25 mm（**不是轴距**，R47）。
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from vgt import odometry

# 实车标定值（spec §2.3 / 固件/历史）
C_1REV = 1276.0
SIGN = [-1.0, 1.0, -1.0, 1.0]        # forward 符号 FL=-1 FR=+1 RL=-1 RR=+1
LXY_R = 0.12825 / 0.0335             # LXY / R；LXY=(轴距+轮距)/2=128.25 mm
R_EFF = 33.5                         # mm
PWM_P = 150.0

ROLL_PER_COUNT = 2 * math.pi * R_EFF / C_1REV        # ≈0.16496 mm/计数
LXY_MM = LXY_R * R_EFF                               # = 128.25 mm

# 固件映射出的 pwm 模式（各自对应一种运动；符号来自 invert 表）
PWM_FWD = [s * PWM_P for s in SIGN]                  # [-150,+150,-150,+150]：前进
PWM_ROT_CCW = [PWM_P] * 4                            # [+150]*4：原地逆时针
PWM_STRAFE_L = [PWM_P, PWM_P, -PWM_P, -PWM_P]        # [+150,+150,-150,-150]：横移左


def _ramp(total, steps=40):
    """每轮原始计数：0, total/steps, ..., total —— 单调递增、四轮同量。"""
    assert total % steps == 0
    per = total // steps
    return [[per * k] * 4 for k in range(steps + 1)]


def _samples(enc_rows, pwm, t_step_ms=10):
    """按 parse_telemetry 的字段造样本（t_ms 只给消费方看，积分只用 enc/pwm）。"""
    return [{"t_ms": i * t_step_ms, "pwm": list(pwm), "enc": list(row),
             "rpy": [0.0, 0.0, 0.0]}
            for i, row in enumerate(enc_rows)]


def _integrate(enc_rows, pwm, sign=SIGN):
    return odometry.integrate_encoders(
        _samples(enc_rows, pwm), c_1rev=C_1REV, sign=sign,
        l_xy_over_r=LXY_R, r_eff_mm=R_EFF)


# --- parse_telemetry：brief 的两条逐字 + 加严 -------------------------------------


def test_parse_telemetry_roundtrip():
    s = odometry.parse_telemetry(["D 10 -57 53 -73 49 100 200 300 400 1.0 2.0 3.0"])
    assert s[0]["t_ms"] == 10 and s[0]["enc"] == [100,200,300,400] and s[0]["rpy"] == [1.0,2.0,3.0]


def test_parse_telemetry_skips_junk():
    s = odometry.parse_telemetry(["OK RUN pwm=110,110,110,110", "", "D 0 0 0 0 0 1 2 3 4 0 0 0"])
    assert len(s) == 1


def test_parse_telemetry_skips_malformed_and_idle_lines():
    s = odometry.parse_telemetry([
        "D 1 2 3",                                    # 字段不够
        "D a b c d e f g h i j k l m",                # 全非数字
        "D 5 0 0 0 0 1 2 3 4 x 0 0",                  # rpy 里混了非数字
        "I 123 0 0 0 0 1 2 3 4 0 0 0",                # 空闲样本（不是 D）
        "DONE",                                       # 结束标志
        "D 7 0 0 0 0 1 2 3 4 0.5 -0.5 1.25 ",         # 合法（带尾空格）
    ])
    assert len(s) == 1
    assert s[0]["t_ms"] == 7 and s[0]["rpy"] == [0.5, -0.5, 1.25]


def test_parse_telemetry_accepts_idle_lines_when_asked():
    """`tags=("I",)`：R66 读"停车后的惯性滑行"要靠空闲行（固件 `streamSample("I",
    millis())`，`main.cpp:698`），它与 `D` 行同形、只有行头不同。

    默认 `("D",)` 必须**照旧不认** `I` 行（否则 RUN 期间淌进来的空闲行会混进
    里程计序列），所以要显式传 tags。
    """
    lines = [
        "D 10 -57 53 -73 49 100 200 300 400 1.0 2.0 3.0",
        "I 123456 0 0 0 0 700 640 721 655 0.00 0.00 0.00",
    ]
    assert [s["t_ms"] for s in odometry.parse_telemetry(lines)] == [10]
    idle = odometry.parse_telemetry(lines, tags=("I",))
    assert len(idle) == 1
    assert idle[0]["t_ms"] == 123456
    assert idle[0]["enc"] == [700, 640, 721, 655]
    assert idle[0]["pwm"] == [0, 0, 0, 0]
    # 行头白名单可以同时给两个
    assert len(odometry.parse_telemetry(lines, tags=("D", "I"))) == 2


def test_parse_telemetry_preserves_types_and_order():
    s = odometry.parse_telemetry([
        "D 10 -57 53 -73 49 100 200 300 400 1.0 2.0 3.0",
        "D 20 -57 53 -73 49 110 210 310 410 1.0 2.0 3.0",
    ])
    assert [x["t_ms"] for x in s] == [10, 20]
    assert all(isinstance(v, int)
               for v in s[0]["pwm"] + s[0]["enc"] + [s[0]["t_ms"]])
    assert all(isinstance(v, float) for v in s[0]["rpy"])


# --- integrate_encoders：两条 brief 用例（按 R12/R13 重写语义） --------------------


def test_integrate_pure_forward_uses_both_sides():
    """纯前进：四轮按 sign 定义的正方向滚，原始计数等量增长，pwm = sign·150。

    sign 只由函数内部应用一次（enc 里不含方向，也没有第二个 sign 因子）。
    """
    out = _integrate(_ramp(400), PWM_FWD)

    assert out.shape == (41, 3)
    assert np.array_equal(out[0], np.zeros(3))          # 第一个样本是原点
    expected_x = 400 * ROLL_PER_COUNT                   # 独立关系：n·2πR/c_1rev
    assert out[-1][0] > 0
    assert abs(out[-1][0] - expected_x) < 1e-6
    assert abs(out[-1][1]) < 0.5
    assert abs(out[-1][2]) < 0.5


def test_equal_left_right_gives_no_yaw():
    out = _integrate(_ramp(1000), PWM_FWD)
    assert abs(out[-1][2]) < 0.5
    assert abs(out[-1][1]) < 0.5
    assert out[-1][0] > 0


# --- 数值关系：与实车标定对得上 ----------------------------------------------------


def test_one_full_wheel_revolution_moves_one_circumference():
    enc = [[0] * 4, [319] * 4, [638] * 4, [957] * 4, [1276] * 4]
    out = _integrate(enc, PWM_FWD)
    assert abs(out[-1][0] - 2 * math.pi * R_EFF) < 1e-6


def test_pure_rotation_pattern_gives_yaw_only():
    """pwm=[+150]*4 是固件的原地逆时针模式（FL/RL 反相后全为正）。

    四轮计数同样在涨（单相不辨向），只有 pwm 极性把"旋转"与"直行"区分开：
    旋转时左右轮实际反向，`yaw = 轮子滚过的距离 / LXY`。
    """
    out = _integrate(_ramp(1000), PWM_ROT_CCW)
    expected_yaw = math.degrees((1000 * ROLL_PER_COUNT) / LXY_MM)

    assert abs(out[-1][0]) < 0.5
    assert abs(out[-1][1]) < 0.5
    assert out[-1][2] > 0                               # 逆时针为正
    assert abs(out[-1][2] - expected_yaw) < 1e-6


def test_pure_strafe_pattern_gives_lateral_only():
    out = _integrate(_ramp(400), PWM_STRAFE_L)
    expected_y = 400 * ROLL_PER_COUNT

    assert abs(out[-1][0]) < 0.5
    assert abs(out[-1][2]) < 0.5
    assert out[-1][1] > 0                               # +Y = 左
    assert abs(out[-1][1] - expected_y) < 1e-6


def test_yaw_and_translation_scale_with_the_same_c1():
    """**偏航与平移吃同一个 c1/R** —— R66 能靠偏航腿独立判死"c1 标错了"的前提。

    原地旋转时运动学要求 `d = (−a,+a,−a,+a)`，于是偏航里的四项带符号距离**同号**，
    `dψ = (−d0+d1−d2+d3)/(4·LXY)` 化简成

        dψ = (2πR / (4·LXY)) · Σ_i (Δc_i / c1_i)

    即"总计数 → 偏航角"是一条**只被 1/c1 缩放**的直线。所以 c1 若真错了 k 倍，
    偏航也必然差 k 倍 —— 平移腿和偏航腿互相独立，却指向同一个 c1。

    真机依据（`实验/rotcheck/`）：偏航腿解出的 `c1_flat` 均值 1275.2（标称 1274，
    偏差 +0.09%），尺度因子 `s = 1.0009`；直行腿独立解出的 `c1` 同样围着 1274。
    两者都不是 1274/2.7 或 1274×2.7 ⇒ "少报 2.7×"不是尺度问题。
    """
    counts = 1000
    out = _integrate(_ramp(counts), PWM_ROT_CCW)
    closed = (2 * math.pi * R_EFF / (4 * LXY_MM)) * (4 * counts / C_1REV)

    assert abs(math.radians(out[-1][2]) - closed) < 1e-9      # 闭式 == 积分器
    assert abs(out[-1][0]) < 0.5 and abs(out[-1][1]) < 0.5   # 纯旋转：不平移

    # c1 翻倍 ⇒ 同样的计数只转出一半的偏航。两条腿因此共用一个尺度常数。
    half = odometry.integrate_encoders(
        _samples(_ramp(counts), PWM_ROT_CCW), c_1rev=2 * C_1REV, sign=SIGN,
        l_xy_over_r=LXY_R, r_eff_mm=R_EFF)
    assert abs(half[-1][2] - out[-1][2] / 2) < 1e-9


def test_reversed_pwm_drives_backward():
    out = _integrate(_ramp(400), [-v for v in PWM_FWD])
    assert out[-1][0] < 0


# --- sign 只应用一次：契约与拒绝 ---------------------------------------------------


def test_sign_and_pwm_carry_the_direction_exactly_once():
    """同一物理前进的两种等价写法必须给出同一个答案：

    (sign = 实车四轮极性, pwm = sign·150) ≡ (sign = 全 +1, pwm = 全 +150)。
    方向信息由 (sign, pwm) 这一对携带一次，不重不漏。
    """
    a = _integrate(_ramp(400), PWM_FWD, sign=SIGN)
    b = _integrate(_ramp(400), [PWM_P] * 4, sign=[1.0, 1.0, 1.0, 1.0])
    assert np.array_equal(a, b)


def test_integrate_rejects_direction_baked_into_counts():
    """把 sign 预乘进 enc（"有符号计数"的旧写法）＝双重应用。

    固件发的是原始单调计数（R13），所以这种序列对 sign=−1 的轮子必然递减，
    函数用 ValueError 拒绝而不是静默算出一个错的角度。
    """
    enc = [[1000] * 4] + [[1000 + s * 10 * k for s in SIGN] for k in range(1, 5)]
    with pytest.raises(ValueError, match="单调"):
        _integrate(enc, PWM_FWD)


def test_integrate_requires_pwm_and_enc_keys():
    s = [{"t_ms": 0, "enc": [0, 0, 0, 0]},
         {"t_ms": 10, "enc": [10, 10, 10, 10]}]
    with pytest.raises(ValueError, match="pwm"):
        odometry.integrate_encoders(s, c_1rev=C_1REV, sign=SIGN,
                                    l_xy_over_r=LXY_R, r_eff_mm=R_EFF)


def test_integrate_rejects_bad_sign_and_calibration():
    s = _samples(_ramp(400), PWM_FWD)
    with pytest.raises(ValueError):
        odometry.integrate_encoders(s, c_1rev=C_1REV, sign=[0.0, 1.0, -1.0, 1.0],
                                    l_xy_over_r=LXY_R, r_eff_mm=R_EFF)
    with pytest.raises(ValueError):
        odometry.integrate_encoders(s, c_1rev=C_1REV, sign=[-1.0, 1.0],
                                    l_xy_over_r=LXY_R, r_eff_mm=R_EFF)
    with pytest.raises(ValueError):
        odometry.integrate_encoders(s, c_1rev=C_1REV, sign=SIGN,
                                    l_xy_over_r=0.0, r_eff_mm=R_EFF)


# --- 边界：空输入 / 单样本 / IMU --------------------------------------------------


def test_integrate_empty_and_single_sample():
    empty = odometry.integrate_encoders([], c_1rev=C_1REV, sign=SIGN,
                                        l_xy_over_r=LXY_R, r_eff_mm=R_EFF)
    assert empty.shape == (0, 3)
    one = _integrate([[5, 5, 5, 5]], PWM_FWD)
    assert np.array_equal(one, np.zeros((1, 3)))


# --- 端到端符号链：逆运动学造数据 → 正运动学重建 -----------------------------------


def _firmware_run(vx, vy, om, duration_s, dt=0.01):
    """按固件行为造一段"真实"采集（测试侧独立实现，不用被测模块的公式）。

    - pwm：`pwm_i = sign_i · sgn(w_i) · 幅值`（`invert` 表 + 死区后的符号形状）；
    - 计数：单相编码器只数边沿，每步新增 |Δθ|·c_1rev/(2π) 个计数，**小数累积、
      整边沿入账**（与真实编码器一致：总计数 ∝ 总转角，抽样时刻只决定何时看到）。
    """
    lxy_m = LXY_MM / 1000.0
    r_m = R_EFF / 1000.0
    w = np.array([
        (vx - vy - lxy_m * om) / r_m,
        (vx + vy + lxy_m * om) / r_m,
        (vx + vy - lxy_m * om) / r_m,
        (vx - vy + lxy_m * om) / r_m,
    ])
    pwm = [s * math.copysign(PWM_P, wi) for s, wi in zip(SIGN, w)]

    steps = int(round(duration_s / dt))
    frac = np.zeros(4)
    counts = np.zeros(4)
    enc_rows = [counts.astype(int).tolist()]
    for _ in range(steps):
        frac += np.abs(w) * dt * C_1REV / (2 * math.pi)
        whole = np.floor(frac)
        frac -= whole
        counts += whole
        enc_rows.append(counts.astype(int).tolist())

    samples = _samples(enc_rows, pwm, t_step_ms=int(dt * 1000))
    truth = (vx * duration_s * 1000.0, vy * duration_s * 1000.0,
             math.degrees(om * duration_s))
    return samples, truth


@pytest.mark.parametrize("vx, vy, om", [
    (0.20, 0.0, 0.0),          # 直行前进
    (-0.15, 0.0, 0.0),         # 直行后退
    (0.0, 0.08, 0.0),          # 横移左
    (0.0, -0.08, 0.0),         # 横移右
    (0.05, -0.05, 0.0),        # 斜行（前右）
    (0.0, 0.0, math.pi / 4),   # 原地逆时针
    (0.0, 0.0, -math.pi / 4),  # 原地顺时针
])
def test_inverse_then_forward_round_trip_recovers_commanded_motion(vx, vy, om):
    """端到端符号链：测试侧用固件逆运动学造计数/pwm，函数侧必须重建回指令位移。

    任何一处 ± 写反（pwm 极性、sign、正运动学的 vy/ω 行符号），
    重建的方向或比例就对不上 —— 这是对整套约定的独立交叉校验。
    """
    samples, (dx, dy, dpsi) = _firmware_run(vx, vy, om, 1.0)
    out = odometry.integrate_encoders(samples, c_1rev=C_1REV, sign=SIGN,
                                      l_xy_over_r=LXY_R, r_eff_mm=R_EFF)
    assert abs(out[-1][0] - dx) < 0.005 * abs(dx) + 0.2
    assert abs(out[-1][1] - dy) < 0.005 * abs(dy) + 0.2
    assert abs(out[-1][2] - dpsi) < 0.005 * abs(dpsi) + 0.05


def test_imu_yaw_series_extracts_the_yaw_column_only():
    s = _samples(_ramp(20, steps=2), PWM_FWD)
    for i, sample in enumerate(s):
        sample["rpy"] = [i + 0.5, -i, 10.0 * i]
    ys = odometry.imu_yaw_series(s)
    assert ys.shape == (3,)
    assert ys.dtype == float
    assert np.allclose(ys, [0.0, 10.0, 20.0])
    assert odometry.imu_yaw_series([]).shape == (0,)
