"""编码器 / IMU 解算（纯数学，无硬件依赖）：`parse_telemetry` → `integrate_encoders`。

## 数据来源：固件 `D` 行

    D <t_ms> <pwm0..3> <enc0..3> <roll> <pitch> <yaw>

一行 = 一个样本，100 Hz 采样；运行期间每攒 5 行 flush 一批（**5 行 = 50 ms**，
R7 已修正：不是 250 ms）。PC 侧一律按 `t_ms` 自行判断样本间隔，
**不要**假设"收到 N 行 = 过了 M 毫秒"（R7 修正的裁决）。

## 车体系约定

X 前、Y 左、ω 俯视逆时针为正（spec §2.3）；原点 = 四轮轴心俯视投影矩形的中心。
轮序 `0=FL 1=FR 2=RL 3=RR`。固件 `bodyToPWM`（`main.cpp:149-156`）的逆运动学：

    w0 = (vx - vy - LXY*om) / R      # FL
    w1 = (vx + vy + LXY*om) / R      # FR
    w2 = (vx + vy - LXY*om) / R      # RL
    w3 = (vx - vy + LXY*om) / R      # RR

⚠ **`LXY = (轴距 + 轮距)/2`，不是轴距**（R47：实测轴距 140、轮距 117 ⇒ (140+117)/2
= 128.5，固件取 128.25，差 0.25 mm）。`l_xy_over_r = LXY / R` 传参时分子必须用
这个"半轴距 + 半轮距"；误当轴距用会让力臂错 ~8%，旋转解算直接错。

## 正运动学（本模块是它的位移形式）

解得（与固件逆运动学互逆）：

    vx = R/4 * ( w0 + w1 + w2 + w3)
    vy = R/4 * (-w0 + w1 + w2 - w3)
    om = R/(4*LXY) * (-w0 + w1 - w2 + w3)

把 `w_i·dt` 记成轮子滚过的**带符号距离** `d_i`（mm；`d_i = q_i · 2πR / c_1rev`，
`q_i` 为带符号计数增量），一步的位移就是

    dx   = ( d0 + d1 + d2 + d3) / 4
    dy   = (-d0 + d1 + d2 - d3) / 4
    dpsi = (-d0 + d1 - d2 + d3) / (4 * LXY)          [rad]

## 单相编码器：唯一可行的方向约定（E13，SDD 台账 R13）

编码器只有 A 相（`main.cpp:74-87` 只有 `encCount[i]++`），**计数不辨向**：
不管轮子往哪边转、转多少，计数都只增不减。所以

- `enc` 的帧间增量 `Δc_i ≥ 0` 只是"滚过多远"（总路程），方向必须另外来；
- 方向来自**同一行的 pwm 极性**：固件 `invert={true,false,true,false}`（FL/RL 反相），
  命令逻辑轮速 `w_i > 0` 时实际下发的 pwm 符号就是 `sign_i`
  （spec §2.3 的 forward 符号 `[-1,+1,-1,+1]`，与实车 exp1 的
  `[-57,+53,-73,+49]` 一致）。于是该区间轮子的运动学方向为
  `sign_i * sgn(pwm_i)`；
- **`sign` 只在这里应用一次**：

      q_i = sign_i · sgn(pwm_i) · Δc_i

  ⚠ 调用方**不许**再把 sign（或 pwm 的符号）乘进 enc —— 固件发的是原始单调
  计数（R13），预乘一遍就是双重应用。方向被预乘进计数的序列必然出现递减，
  本函数按契约抛 `ValueError` 而不是静默算一个错角度。

为什么不能只用 `sign·Δc`：直行时四轮同向、横移/原地旋转时左右轮**反向**，
但两种情况下单相计数都在涨 —— 只有 pwm 极性（固件前进 `[-,+,-,+]`、
横移 `[+,+,-,-]`、原地旋转 `[+,+,+,+]`）能把它们区分开。
`pwm_i = 0` 的区间方向未知，该轮的计数不计入（计数不静默造假）。

## 其他约定与已知限制

- 固件在 `RUN` 开始时清零编码器（`zeroEncoders()`），本函数以**第一个样本**
  为原点：`out[0] = (0, 0, 0)`，`out[k]` 是第 k 个样本时刻相对起点的位姿。
- 位姿积分用**中点航向**（`psi + dpsi/2` 旋转本步的体坐标位移）；
  10 ms 步长下与精确解无可见差。
- `out[:, 2]` 是**俯视逆时针为正**的 yaw（度）。
- `imu_yaw_series` 只把 `rpy[2]` 原样取出：不 unwrap、不重新归零 —— 消费方取差值
  （R32 记过"视觉与里程计 yaw 符号必须一致"的账，Task 11 对表时先验符号）。
- IMU 只有 ~6 Hz（spec §3），只作粗略交叉校验，主真值是视觉。
"""
from __future__ import annotations

import math

import numpy as np


def parse_telemetry(lines, tags=("D",)):
    """把固件 `D` 行解析成样本字典列表（顺序保持；非法行跳过）。

    返回 `{"t_ms": int, "pwm": [4 int], "enc": [4 int], "rpy": [3 float]}`。
    只认 `tags` 里的行头（默认 `D`）、恰好 13 个字段、数字都能转的行：
    `DONE`、`OK RUN ...`、别的行头、空行、字段缺失或数字坏掉的行一律跳过
    （live 流里这些都是噪声，不该让整批数据报废）。

    `tags`：行头白名单。**空闲样本**要用 `("I",)` —— 固件在非 RUN 期间发的
    `I` 行字段与 `D` 行完全同形（`main.cpp` 的 `streamSample("I", millis())`），
    只有 `t_ms` 是 `millis()` 而不是 RUN 相对时间、`pwm` 恒为停车后的 0。
    R66 读停车后的惯性滑行计数就靠它（见 `harness.coasted_samples`）。
    """
    wanted = frozenset(tags)
    samples = []
    for line in lines:
        parts = line.split()
        if not parts or parts[0] not in wanted or len(parts) != 13:
            continue
        try:
            t_ms = int(parts[1])
            pwm = [int(v) for v in parts[2:6]]
            enc = [int(v) for v in parts[6:10]]
            rpy = [float(v) for v in parts[10:13]]
        except ValueError:
            continue
        samples.append({"t_ms": t_ms, "pwm": pwm, "enc": enc, "rpy": rpy})
    return samples


def _vec4(value, name: str) -> np.ndarray:
    """转成形状 (4,) 的 float 向量（标量广播到 4 轮）；不合形状抛一句人话。"""
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 0:
        return np.full(4, float(arr))
    if arr.shape != (4,):
        raise ValueError(f"{name} 必须是 4 个元素（轮序 FL/FR/RL/RR），实际 {arr.shape}。")
    return arr


def integrate_encoders(samples, *, c_1rev, sign, l_xy_over_r, r_eff_mm):
    """把编码器样本序列积分成位姿序列，返回 `(N, 3)` 的 `(x_mm, y_mm, yaw_deg)`。

    参数
    ----
    - `samples`：`parse_telemetry` 的输出（每个样本要有 `enc` 与 `pwm`；
      `pwm` 用来定方向，见模块 docstring —— 单相计数不辨向，E13）。
    - `c_1rev`：每轮转一圈的计数（标定值 1276/1270/1276/1274，标量或 4 元素皆可）。
    - `sign`：四轮 forward 符号（spec §2.3，`[-1,+1,-1,+1]`），取值 ±1。
    - `l_xy_over_r`：`LXY / R`，其中 `LXY = (轴距 + 轮距)/2`（**不是轴距**，R47）。
    - `r_eff_mm`：有效轮半径（mm）。

    约定
    ----
    - `enc` 必须是**原始无符号、单调不减**的计数；出现递减（或 `enc`/`pwm`
      缺失、形状不对）立即 `ValueError` —— 静默算出一个错角度比报错危险得多。
    - `q_i = sign_i · sgn(pwm_i) · Δc_i`（sign 只应用一次）；一步的
      `d_i = q_i · 2πR / c_1rev`，再按正运动学拆成 `(dx, dy, dpsi)`，中点航向积分。
    - 第一个样本是原点：`out[0] == (0, 0, 0)`；`N = 0` 返回 `(0, 3)` 空数组。
    """
    c1 = _vec4(c_1rev, "c_1rev")
    sign_arr = _vec4(sign, "sign")
    if not np.all(np.abs(sign_arr) == 1.0):
        raise ValueError(f"sign 只能取 ±1（forward 符号），实际 {list(sign_arr)}。")
    if not np.all(c1 > 0):
        raise ValueError(f"c_1rev 必须为正，实际 {list(c1)}。")
    if not (l_xy_over_r > 0) or not (r_eff_mm > 0):
        raise ValueError(
            f"l_xy_over_r 与 r_eff_mm 必须为正（LXY=(轴距+轮距)/2，R47），"
            f"实际 l_xy_over_r={l_xy_over_r} r_eff_mm={r_eff_mm}。")

    n = len(samples)
    out = np.zeros((n, 3), dtype=float)
    if n == 0:
        return out

    enc = np.empty((n, 4), dtype=float)
    pwm = np.empty((n, 4), dtype=float)
    for k, sample in enumerate(samples):
        if "enc" not in sample or "pwm" not in sample:
            raise ValueError(
                f"第 {k} 个样本缺 'enc'/'pwm' 字段：integrate_encoders 需要 pwm 定方向"
                "（单相编码器不辨向，E13）—— 请传 parse_telemetry 的输出。")
        enc_row = np.asarray(sample["enc"], dtype=float)
        pwm_row = np.asarray(sample["pwm"], dtype=float)
        if enc_row.shape != (4,) or pwm_row.shape != (4,):
            raise ValueError(
                f"第 {k} 个样本的 enc/pwm 必须是 4 个元素（轮序 FL/FR/RL/RR），"
                f"实际 enc={enc_row.shape} pwm={pwm_row.shape}。")
        enc[k], pwm[k] = enc_row, pwm_row

    roll_per_count = 2.0 * math.pi * r_eff_mm / c1          # (4,) mm / 计数
    lxy_mm = l_xy_over_r * r_eff_mm                         # LXY，mm

    x = y = psi = 0.0
    for k in range(1, n):
        dc = enc[k] - enc[k - 1]
        if np.any(dc < 0):
            raise ValueError(
                f"第 {k} 个样本的编码器计数递减（{list(enc[k - 1])} → {list(enc[k])}）："
                "固件只做 encCount[i]++，计数必须单调不减（R13）。递减 = ZENC/截断/"
                "把方向预乘进计数的双重应用（sign 只能由本函数应用一次）。")
        q = sign_arr * np.sign(pwm[k - 1]) * dc             # 带符号计数增量
        d = q * roll_per_count                              # 带符号滚动距离 mm
        dx_body = (d[0] + d[1] + d[2] + d[3]) / 4.0
        dy_body = (-d[0] + d[1] + d[2] - d[3]) / 4.0
        dpsi = (-d[0] + d[1] - d[2] + d[3]) / (4.0 * lxy_mm)
        mid = psi + dpsi / 2.0                              # 中点航向
        x += dx_body * math.cos(mid) - dy_body * math.sin(mid)
        y += dx_body * math.sin(mid) + dy_body * math.cos(mid)
        psi += dpsi
        out[k] = (x, y, math.degrees(psi))
    return out


def imu_yaw_series(samples):
    """把样本的 IMU yaw（`rpy[2]`，度）取成 `(N,)` 数组：不 unwrap、不重新归零。

    GY-25T 只有 ~6 Hz（spec §3），它只配做"粗略交叉校验"：消费方应当用
    **差值**（`yaw_end - yaw_start`），而不是把它当绝对航向。
    """
    yaw = []
    for k, sample in enumerate(samples):
        if "rpy" not in sample:
            raise ValueError(f"第 {k} 个样本缺 'rpy' 字段（IMU roll/pitch/yaw 三元组）。")
        rpy = sample["rpy"]
        if len(rpy) != 3:
            raise ValueError(f"第 {k} 个样本的 'rpy' 必须是 3 个元素，实际 {len(rpy)}。")
        yaw.append(float(rpy[2]))
    return np.asarray(yaw, dtype=float)
