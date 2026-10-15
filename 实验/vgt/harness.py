"""视觉闭环归位 harness（Task 10）：`locate()` 单帧真值 + `drive_to()` 闭环开进目标点。

    # 真机用法（`vgt` 只在 `实验/` 于 sys.path 上时可导入，R11/R17）
    cd 实验
    python3 -c "
    from vgt.harness import Harness
    from vgt.camera import CameraSession
    from vision_core import load_calib
    from car_link import CarLink
    with CameraSession() as cam, CarLink() as car:
        h = Harness(car, cam, load_calib())
        print(h.locate())          # (x_mm, y_mm, yaw_deg) 或 None
        print(h.drive_to(0.0, 0.0))
    "

## 接口

- `Harness(car_link, camera, calib, home_xy=None)`：
  - `car_link`：`car_link.CarLink` 兼容壳（`cmd(line, wait=False)` 发送、`sock`、
    `buf`、`addr`）。失控保护会重连，新链接放进 `harness.car_link`。
  - `camera`：`vgt.camera.CameraSession`（`grab()` **默认 `fresh=True`**，R54）。
  - `calib`：`vision_core.load_calib()` 的输出（用到 `vehicle_side_mm` / `bias`）。
  - `home_xy`：归位点。**默认 `None` 是哨兵** = 名义场地中心（与 Task 9
    `assert_within_field` 的 `(0.0, 0.0)` 默认分支同一个"宽松口径"）；显式给坐标
    （哪怕就是 `(0.0, 0.0)`）时 `home_xy_explicit=True`。见文末一节。
- `locate() -> (x_mm, y_mm, yaw_deg) | None`：单帧解算；失败原因在
  `last_locate_reason`（`"boundary_moved"` / `"no_vehicle_marker"`，与
  `verify_vision` 同一词表）。
- `drive_to(x_mm, y_mm, tol_mm=12.0, max_iter=12) -> dict`：P 控制归位。返回
  `{"reached", "iters", "final", "reason", "trace", "events", "reset_lines",
  "stop_confirmed"}`；`iters` = 实际下发的 RUN 脉冲数，`final` = 最后一次成功
  `locate()` 的位姿（一次都没成功就是 `None`）。

## 控制律与脉冲（brief 原文 + 已知物理坑）

每个控制周期：`locate()` → 误差 `(dx, dy)`（场地系 mm）→ 换算到车体系 `(vx, vy)`
→ `RUN vx vy 0 <pulse_ms>`（默认 400）→ 等 `DONE` → 等 `settle_s`（默认 0.5 s）
→ 重新 `locate()`，直到 `|err| ≤ tol_mm` 或脉冲数到 `max_iter`。

车体/场地坐标换算（`body_command`，符号链是整条管线最容易写反的一条）：θ 是
`heading_deg` 口径（俯视逆时针为正、零点 +X，且**场地 +Y 在画面上朝下**）：

    X_body（车头）= ( cosθ, −sinθ)      Y_body（车左）= (−sinθ, −cosθ)
    vx = v · dot(err_unit, X_body)     vy = v · dot(err_unit, Y_body)

两个锚点：θ=0（车头朝图像右）时 `err=+Y（图像下）` 给出 `vy<0`（右侧）；
θ=+90°（车头朝图像上）时 `err=−Y` 给出 `vx>0`（正前）。由
`test_body_command_direction_convention` 的八个 90° 朝向钉死。

⚠ **`MIN_DUTY=110` 地板（Task 7 实锤）**：`RUN` 的目标轮速经 `wheelOmegaToPWM`
查表后，任何需要转动的轮子都会被抬到 duty ≥ 110；落地实测该档轮速 ~11.5–13.3
rad/s（换算车速 ≈0.4 m/s）。也就是说 400 ms 脉冲的最小位移量级是**几十到一百多
毫米**，而 `tol_mm` 默认 12 mm —— 真机上很可能在目标两侧来回跳、不收敛。本模块
**不自动改策略**（没有实测数据前不猜）：`pulse_ms` 是显式参数（可以先试 150 ms
更短的脉冲）；要根治得动固件的 `MIN_DUTY`（Task 7 报告的结论，改前需用户确认）。
待真机验证清单里有这一条的判据与处置。

## 边界码漂移复核（R18，Review Focus #2）

`locate()` 每次都会把 4 个边界码的角点均值像素（`FieldFrame.marker_px` 口径）与
**本次会话第一次成功解出车体位姿那一帧**的基准对比；任一码移动严格 > 15 px 就抛
`BoundaryMarkersMoved`。基线**每个 harness 实例只抓一次**：

- 相机被挪动同样会触发（基线是像素量，相机一动四码全"漂移"）——挪回去之后要
  **重开一个 `Harness`**（没有重置基线的接口，故意的）；
- `drive_to` 里 `locate()` 抛出的异常立即向上冒泡 = "立即中止"。

## run_trial（Task 11）：单次实验编排

    # 一条实验的完整时序（真机用法）：
    cd 实验
    python3 -c "
    from vgt.harness import Harness
    from vgt.camera import CameraSession
    from vgt import matrix
    from vision_core import load_calib
    from car_link import CarLink
    with CameraSession() as cam, CarLink() as car:
        h = Harness(car, cam, load_calib(), home_xy=(0.0, 0.0))
        res = h.run_trial(matrix.EXPERIMENTS[0])
        print(res['ok'], res['reason'], res['vision_delta'], res['odometry_delta'])
    "

时序（每步的"为什么"都在代码注释里）：

1. 开场 PING（链路探活 + RESET 基线）→ STOP（清环 + 兜底停车）→ 等 0.3 s；
2. `locate()` 抓**起点**帧（缺边界码/漂移 → `boundary_moved`；解不出 ID29 →
   `no_vehicle_marker`）；
3. 下发 `RUN vx vy om ms`，同一时刻开始读流式 `D` 行；
4. 第一条 `t_ms ≥ ms/2` 的样本到达时抓**中点**帧 —— **抓失败不致命**，降级为
   起/终两点（Review Focus #1），失败原因进 `warnings`；
5. 收到 `DONE`（**只认 DONE**，R56：实测墙钟延迟最多 10.87 s，所以判完超时 ≥15 s）
   → 等 0.4 s → 抓**终点**帧；
6. 没有 `DONE`（超时/静默/断链）→ 失控保护（STOP；不确认就重连 + STOP）→
   **强制重连 + 查 RESET 行**再定性（R56）：查到**新的** RESET 文本 = `board_reset`，
   没有 = `deadline_timeout`。不能"超时即判 brownout"（环满时 `DONE` 本身会被丢）；
7. 收尾**无条件** `STOP`（异常/中断路径同样执行，见 `run_trial` 的 finally）。

**抓帧新鲜度**：三个关键帧都走 `locate()` 里的 `camera.grab()` **默认 `fresh=True`**
（R54/C1）："等 0.3/0.4 s 沉降再抓帧"本身**不保鲜** —— 驱动队列会在这段静置里囤下
运动中的老帧，`vision_delta` 会静默算错。测试用 `camera.calls == [True, True, True]`
钉死这条。

**有符号判据（Task 9 复核 Important 3 的硬要求）**：`vision_delta` 与
`odometry_delta` 只比 `dist_mm`（标量）抓不住"平移符号整体翻转" —— 量级对、方向反时
相对差仍 ≈ 0，会静默通过第一次上机验收。所以 `run_trial` 把里程计的（车体系）位移用
起点 yaw 转到场地系（`body_delta_to_field`），算 `dot(vision_Δxy, odometry_Δxy)`：

- 两向量模长都 ≥ `SIGN_MIN_NORM_MM`（5 mm）且点积 ≤ 0 → `ok=False,
  reason="sign_mismatch"`，`sign_ok=False` 落进结果；
- 任一向量太短（原地旋转的漂移、没动的实验）→ `sign_ok=None` = "判不出方向"，
  既不算通过也不算失败（不冤枉合法数据）。

**`ok=False` 的 `reason` 词表**（`TRIAL_REASONS`；比 plan 的 brief 多几个，都是实现里
真的会遇到、且必须有名有姓才能让 Task 12 标注行 / 事后分诊的失败）：

| reason | 含义 |
|---|---|
| `boundary_moved` | 起点/终点帧缺边界码、建不出场地系，或 R18 漂移超限 |
| `no_vehicle_marker` | 起点/终点帧解不出 ID29 |
| `deadline_timeout` | `done_timeout_s` 内没等到 `DONE`，且重连后没有**新的** RESET 证据 |
| `board_reset` | 同上，但查到了新的 `RESET` 行（brownout/panic/看门狗） |
| `run_rejected` | `RUN` 被固件回 `ERR` 拒了（没有在跑） |
| `no_telemetry` | `DONE` 到了但 `D` 样本 < 2，里程计无法积分（不能拿 0 当"没动"） |
| `camera_failed` | 关键帧抓帧失败（`CameraUnavailable`，R39 同句柄不重试） |
| `sign_mismatch` | 有符号判据不过（见上） |
| `trial_error` | 未预期异常（原文进 `warnings`） |

`mid_capture_failed` **不在** `reason` 里 —— brief 的 Step 2 明确要求中点失败
`ok=True`；它只作为 `result["mid_capture_failed"]` 与 `warnings` 的标注。

**停止安全性（R58 未修缺陷的兜底）**：任何一次必要的 `STOP` 无法确认、且本次实验
已经发过 `RUN`、又没有"DONE 已停车"或已确认 `OK STOP` 的证据时，抛 `StopFailed`
（消息含"立即给车断电"）。收到 `DONE` 本身是停车证据（固件在发 `DONE` 前执行
`stopAll()`），此时收尾 STOP 未确认只记 `stop_confirmed=False` + 警告，不虚报
`StopFailed`。

## 链路协议侧（全是踩过的坑）

- **RESET-aware（R58 复核 Important 4）**：板子非正常复位后，每条 `PING`/`ID`
  回复后面都会多一行 `RESET <原因> count=<n>`。裸"只读一行"的读取器会把这条旧行
  当命令回复，之后**所有回复整体错位一行**。本模块的读取器把 `RESET` 行单独收集
  到 `reset_lines`，**永不**作为命令回复匹配。
- **粘连行**：`RUN`/`STOP` 的 handler 清环形缓冲时，可能把回复粘在一条样本半行
  屁股上（现场原文 `I 2410259 0 0 0 0 0 0 0OK M`）；读进来的整行先过
  `_split_glue`（与 `实验/wheelmap_sweep.py` 的实现同表，测试对拍）。
- **判"跑完"只认 `DONE`**（R56）：超时 `done_timeout_s`（默认 15 s；实测 DONE
  墙钟延迟最多 10.87 s，短超时会误判）。等待期间连续 `silence_timeout_s`
  （默认 4 s）没有任何字节 = 链路死了，不再傻等到超时。
- **失控保护**（`StopFailed`）：`DONE` 超时 / 链路死 / 发送失败 → 先在当前链路
  `STOP` 并等 `OK STOP`。**确认了就不重连**（链路活着 + 车已停；缺的 DONE 可能
  只是固件环形缓冲满时被丢了，R58 已知现象）；没确认（链路死/固件卡）就**关掉旧
  链路 → 重连 → 第一件事 `STOP`**；两边都失败抛 `StopFailed` —— 固件在 WiFi
  掉线时不会执行 RUN 的到时停车（`loop()` 在采样段前 `return`），车可能还在开。
- `drive_to` 开场与收尾各 `STOP` 一次（清环 + 兜底停车）；开场前还会 drain 一次
  积压字节，防上一会话的残帧冒充本会话回复。

## 与 Task 9 的 `(0.0, 0.0)` 宽松分支的关系（复核 Important 1）

`matrix.assert_within_field(exp, start_xy=(0.0, 0.0))` 的默认值会走近中心分支
（"全场可行性"），只有显式起点才走绝对扫掠校验。所以 `home_xy` 用 **None 哨兵**：
`None` = 还没归过位（名义中心，宽松口径），显式给坐标时 `home_xy_explicit=True`。
**Task 12 传 `start_xy` 时不得直接拿默认的 `(0.0, 0.0)` 去撞宽松分支** —— 要么传
实验真正的执行起点，要么按 `home_xy_explicit` 自行决定判据。本模块只区分两种口径，
不替 Task 12 裁决。

## 不做什么

- 构造无 I/O：不开相机、不建链接、不发命令（`CameraSession` 的口径）。
- 不动相机属性（R53），不改固件，不改标定文件，不用 `M` 直驱绕 `MIN_DUTY`
  （brief 指定 `RUN`）。
"""
from __future__ import annotations

import math
import socket
import time
import warnings

from vision_core import calib_store
from vision_core.field import body_pose_in_field, build_field_frame
from vision_core.markers import detect_markers
from verify_vision import REASON_BOUNDARY_MOVED, REASON_NO_VEHICLE

from . import matrix
from . import odometry
# `CameraUnavailable` 只在 run_trial 的收尾归类里用到；import 它不会开相机
# （打开设备只发生在 CameraSession.__enter__）。
from .camera import CameraUnavailable

# ---- 常量（每个都不是随手取的，出处见模块 docstring 与各自注释） --------------------

# R18：边界码角点均值相对会话基线的位移上限（px）。"超过 15 px"是**严格大于**。
BOUNDARY_DRIFT_TOL_PX = 15.0

# brief 的默认接口参数。
DEFAULT_TOL_MM = 12.0
DEFAULT_MAX_ITER = 12
DEFAULT_PULSE_MS = 400.0
DEFAULT_SETTLE_S = 0.5

# R56：判"跑完"只认 DONE，超时 ≥15 s（实测 DONE 墙钟延迟最多 10.87 s）。
DEFAULT_DONE_TIMEOUT_S = 15.0
# 等待 RUN 期间连续这么久没有任何字节 = 链路死（与 wheelmap_sweep 的静默看门狗同值）。
DEFAULT_SILENCE_TIMEOUT_S = 4.0
# 等一条命令回复（OK STOP / PONG）的超时。重连后的新会话第一件事是 PING 握手
# （积压最多能把**首条**回复挤后 ~3.5 s，那由 8 s 的握手窗口兜住），握手过了之后
# 环形缓冲是清的，STOP/PONG 的回复不该再排队，3 s 很宽裕。
DEFAULT_REPLY_TIMEOUT_S = 3.0
# 重连后的 PING/PONG 握手总超时（固件对陈旧 client 的接受有滞后，R56）。
DEFAULT_HANDSHAKE_TIMEOUT_S = 8.0
# 单次 locate() 解不出（掉码/反光）时的重试次数（每次重抓一帧）。
DEFAULT_LOCATE_TRIES = 3
_LOCATE_RETRY_PAUSE_S = 0.05

# ---- run_trial（Task 11）的时序与判据常量 ------------------------------------------

# 起点/终点抓帧前的沉降等待（plan 的 0.3 s / 0.4 s）。⚠ 它**不负责**帧新鲜度 ——
# 驱动队列会在这段等待里囤运动中的老帧（R54），新鲜度由 grab(fresh=True) 负责。
DEFAULT_START_SETTLE_S = 0.3
DEFAULT_AFTER_DONE_SETTLE_S = 0.4
# 中点抓帧的触发线：`D` 行的 `t_ms` 首次 ≥ `ms · 该系数`（固件 D 行的时间就是
# 相对 RUN 起点的 ms，main.cpp:373-374）。
DEFAULT_MID_FRACTION = 0.5
# PONG 之后补收 RESET 行的短暂 drain：固件把 `RESET …` 单独一行发在 PONG **后面**
# （main.cpp:400-401），同一 recv 批次可能读不到，漏了就会把上一轮的证据弄脏。
_PROBE_TAIL_DRAIN_S = 0.2

# R66：RUN 到时停车后车会**自由滑行** —— 固件 `stopAll()` 是把两个 IN 都写 0
# （main.cpp:132-138），那是"惯性滑行"不是"短接刹车"，固件里没有刹车命令。
# 而 `D` 行随停车一起停（main.cpp:693-696），滑行期间编码器还在计数却没有人报；
# 视觉的终点帧却在 `settle_after_done_s` 之后才抓，把滑行量进去了。两个窗口不一样长
# （实测滑行量是通电量的 1.5~1.7 倍），`odometry_delta` 就系统性少报 —— 看着像
# "c1 标错了 2.7×"，其实 c1 是对的（真机复标：1295 vs 标定 1274，差 1.6%）。
# 所以停车后要补读一次空闲（`STREAM 1` 的 `I` 行）编码器累计值，把滑行并进里程计。
_COAST_ENC_TIMEOUT_S = 0.5        # 读完这几行就够；读不到是非致命的（但要出声）
_COAST_ENC_LINES = 3

# 里程计标定常量（Task 9 的 `integrate_encoders` 要求显式传参；真机默认值在这）。
# 出处：`c_1rev` = `固件/exp/data/wheelmap_{airborne,grounded}.json` 的实测值；
# `sign` = spec §2.3 的 forward 符号；`LXY=(轴距+轮距)/2=128.25 mm` 是**固件值**
# （R47；实测 128.5，差 0.25 mm，别当轴距用）；`r_eff` 见 Task 9 的标定口径。
ODOM_C_1REV = (1276.0, 1270.0, 1276.0, 1274.0)
ODOM_SIGN = (-1.0, 1.0, -1.0, 1.0)
ODOM_R_EFF_MM = 33.5
ODOM_LXY_OVER_R = 0.12825 / 0.0335

# 有符号判据的门槛：两个位移向量的模长都 ≥ 5 mm 才谈方向（矩阵里最短的横移是
# 50 mm，5 mm 以下是噪声/漂移，不判）；yaw 同理，1° 以下不判。
SIGN_MIN_NORM_MM = 5.0
YAW_SIGN_MIN_DEG = 1.0

# run_trial 的 ok=False 词表（模块 docstring 有逐条说明）。
TRIAL_REASONS = (
    "boundary_moved",
    "no_vehicle_marker",
    "deadline_timeout",
    "board_reset",
    "run_rejected",
    "no_telemetry",
    "camera_failed",
    "sign_mismatch",
    "trial_error",
)

# P 控制参数：v_cmd = clamp(KP · |err|, v_min, v_max)，单位 m/s。
# KP=0.001 的含义：100 mm 误差 → 0.1 m/s（与实验矩阵 0.05–0.2 m/s 同量级）。
# v_max 取 0.12：归位要稳不要快；v_min 只是"明确给一个非零指令"（真地板的成因是
# 固件侧的 MIN_DUTY=110，见模块 docstring，别拿这里的小数字当"最低速度"）。
KP_MPS_PER_MM = 0.001
MIN_CMD_MPS = 0.03
MAX_CMD_MPS = 0.12

# 每次 recv 的阻塞上限（s）。读取循环用"时间预算 + 短 recv"收敛，不用"读到超时"。
_RECV_POLL_S = 0.05

# 粘连行拆分用的控制行词表。与 `实验/wheelmap_sweep.py::_CTRL_TOKENS` **逐字相同**
# （同一段固件行为、同一份现场样本），测试 `test_split_glue_matches_wheelmap` 对拍。
_CTRL_TOKENS = ("OK ZENC", "OK M", "OK STOP", "OK STREAM", "OK RUN",
                "PONG", "DONE", "RESET", "ERR ")


# ---- 异常 -------------------------------------------------------------------------


class HarnessError(RuntimeError):
    """harness 层面的失败基类（区别于 vision_core 的 ValueError / 相机的 CameraUnavailable）。"""


class BoundaryMarkersMoved(HarnessError):
    """边界码相对本次会话基线移动超过 `BOUNDARY_DRIFT_TOL_PX`（R18）。

    多数情况是码被撞了/没贴平，也可能是**相机被挪了**（基线是像素量，相机一动
    四码全"漂移"）—— 两种都会让 `locate()` 的数字失真，必须停下来人工看一眼。
    """


class StopFailed(HarnessError):
    """RUN 等待失败后，STOP 与重连都未能确认停车 —— **车可能还在开**。

    这是本项目最危险的失败模式：WiFi 掉线时固件在采样段前 `return`，RUN 的
    "到时停车"不执行，电机会保持开环输出直到 WiFi 恢复。收到本异常时唯一安全
    的处置是**给车断电**（异常对象上的 `.events` 带完整时间线，供报告用）。
    """


# ---- 无硬件依赖的纯函数 ------------------------------------------------------------


def body_command(err_x_mm, err_y_mm, yaw_deg, *,
                 kp=KP_MPS_PER_MM, v_min=MIN_CMD_MPS, v_max=MAX_CMD_MPS):
    """场地系误差向量 → 车体系速度指令 `(vx, vy)`（m/s；vx 车头为正、vy 车左为正）。

    `err = target − current`（场地系 mm），`yaw_deg` = 车体航向（`locate()` 的第三个
    数、`heading_deg` 口径）。换算与本模块 docstring 的符号链逐字一致：

        vx = v · dot(err_unit, X_body)，  X_body = ( cosθ, −sinθ)
        vy = v · dot(err_unit, Y_body)，  Y_body = (−sinθ, −cosθ)
        v = clamp(kp · |err|, v_min, v_max)

    `err` 为零向量时方向无定义 → `ValueError`（`drive_to` 只在 `|err| > tol` 时调用，
    真走到这里是调用方的 bug —— 静默给 (0,0) 会让闭环每一步都假装下发、实际一步
    不挪，最后只是把 `max_iter` 次空转用完）。
    """
    kp, v_min, v_max = float(kp), float(v_min), float(v_max)
    if not (kp > 0.0) or not (v_min > 0.0) or not (v_max > 0.0):
        raise ValueError(f"kp/v_min/v_max 必须为正，实际 kp={kp} v_min={v_min} v_max={v_max}")
    if v_max < v_min:
        raise ValueError(f"v_max({v_max}) 不能小于 v_min({v_min})")
    ex, ey = float(err_x_mm), float(err_y_mm)
    d = math.hypot(ex, ey)
    if not math.isfinite(d) or d <= 0.0:
        raise ValueError(f"误差 ({ex}, {ey}) 的模长是 {d}，方向无定义")

    speed = min(kp * d, v_max)
    speed = max(speed, v_min)

    th = math.radians(float(yaw_deg))
    c, s = math.cos(th), math.sin(th)
    ux, uy = ex / d, ey / d
    return (speed * (ux * c - uy * s), speed * (-ux * s - uy * c))


def coasted_samples(samples, coast):
    """把「DONE 之后的惯性滑行」并进**积分用**的样本序列（R66）。

    固件在 RUN 到时的那一刻 `stopAll()`（main.cpp:693），补发最后一条 `D` 样本，
    再发 `DONE` —— **`D` 行到此为止**，可车还在滑、编码器还在计数，只是没人报。
    视觉的终点帧却在 `settle_after_done_s` 之后才抓，把滑行量进去了。两个窗口不一样
    长，`odometry_delta` 就系统性少报（实测滑行量是通电量的 1.5~1.7 倍）——
    看着像"c1 标错了 2.7×"，其实 c1 是对的。把停车后读到的累计计数补成最后一个样本，
    两个窗口就一样长了（真机同一发脉冲：补前 111.9/110.3 = 1.01）。

    两处必须做对，少一个就是**静默空操作**：

    1. `integrate_encoders` 的方向取自 `sgn(pwm[k-1])`，而固件停车补发的那条样本
       `pwm` 全 0（`pwm_last=[0,0,0,0]`）⇒ `sgn(0)=0` ⇒ 滑行段的位移会被整段判成 0。
       所以最后一条 `D` 样本在**积分用的副本**里要带上本段 RUN 的行进方向
       （滑行方向 = 停车前的运动方向）。
    2. 单调性：编码器只增不减。补进来的读数必须**逐轮 ≥** 最后一条 `D` 样本，否则
       宁可原样返回 —— 递减会让 `integrate_encoders` 抛 `ValueError`，把整条实验判死。
       不满足最常见的原因是读到了一条 RUN 之前残留的旧 `I` 行（R62 那类陈旧读数的
       亲戚），丢掉它比让整条实验报废好。

    `coast`：`{"enc": [4 int], "rpy": [3 float] | None}`（`_read_idle_enc` 的输出）。
    返回**新列表**；`samples` 不足两条、`coast` 为 None 或读数不可信时，返回 `samples`
    本身（调用方可以用 `is` 判有没有并进去）。
    """
    if coast is None or len(samples) < 2:
        return samples
    enc = [int(v) for v in (coast.get("enc") or [])]
    last = samples[-1]
    if len(enc) != 4 or any(b < a for a, b in zip(last["enc"], enc)):
        return samples
    # 行进方向：往前找最后一条有非零 pwm 的 D 样本（固件停车补发的那条 pwm 全 0）。
    direction = None
    for s in reversed(samples):
        if any(int(v) != 0 for v in s["pwm"]):
            direction = [int(v) for v in s["pwm"]]
            break
    if direction is None:                       # 整段 RUN 都没转过：没有方向可言
        return samples
    if not any(int(v) != 0 for v in last["pwm"]):
        last = dict(last, pwm=list(direction))  # 只改积分用的副本，不改调用方的 samples
    coast_rpy = coast.get("rpy") or last["rpy"]
    return (list(samples[:-1]) + [last]
            + [{"t_ms": int(last["t_ms"]), "pwm": list(direction), "enc": enc,
                "rpy": [float(v) for v in coast_rpy], "synthetic": "coast"}])


def body_delta_to_field(dx_mm, dy_mm, yaw_deg):
    """车体系位移 `(前, 左)` → 场地系位移 `(x, y)`（mm），用起点 yaw 旋转。

    与 `body_command` 同一条符号链（见模块 docstring）：`X_body=(cosθ,−sinθ)`、
    `Y_body=(−sinθ,−cosθ)`，于是

        field = dx·X_body + dy·Y_body = (dx·cosθ − dy·sinθ, −dx·sinθ − dy·cosθ)

    `odometry.integrate_encoders` 的轨迹就落在**起点的车体系**里；要和视觉的
    场地系位移比方向（有符号判据），必须先转过来。
    """
    th = math.radians(float(yaw_deg))
    c, s = math.cos(th), math.sin(th)
    dx, dy = float(dx_mm), float(dy_mm)
    return (dx * c - dy * s, -dx * s - dy * c)


def translation_sign_ok(vision_dxy, odometry_field_dxy, *, min_norm_mm=SIGN_MIN_NORM_MM):
    """有符号判据（Task 9 复核 Important 3）：两个位移向量同向 → True，反向 → False。

    只比 `dist_mm`（标量）抓不住"平移符号整体翻转"：量级对、方向反时相对差 ≈ 0，
    第一次上机验收（`dist_diff_pct < 20%`）会**静默通过**。这里额外钉住方向：
    两个向量都在场地系里，点积 `dot > 0` 才叫同向。

    任一向量模长 < `min_norm_mm` 时方向无意义（原地旋转的漂移、没动的实验），
    返回 None = "判不出"：既不算通过也不算失败，不许拿噪声冤枉合法数据。
    """
    vx, vy = float(vision_dxy[0]), float(vision_dxy[1])
    ox, oy = float(odometry_field_dxy[0]), float(odometry_field_dxy[1])
    if math.hypot(vx, vy) < min_norm_mm or math.hypot(ox, oy) < min_norm_mm:
        return None
    return (vx * ox + vy * oy) > 0.0


def _yaw_sign_ok(vision_yaw_deg, odometry_yaw_deg, *, min_deg=YAW_SIGN_MIN_DEG):
    """yaw 净转角的方向一致性（信息性字段，**不**参与 `ok`）。

    转角太小（任一 < `min_deg`）时返回 None。平移有专门的判据；yaw 的翻转会以
    `yaw_diff_deg ≈ ±90°` 的形式在 Task 12 的表里大声暴露，不需要在这里升级成
    失败，但并排记下来对分诊有用。
    """
    v, o = float(vision_yaw_deg), float(odometry_yaw_deg)
    if abs(v) < min_deg or abs(o) < min_deg:
        return None
    return (v > 0.0) == (o > 0.0)


def _wrap180_deg(deg):
    """归一到 (−180, 180]（与 `vision_core.field._wrap180` 同口径；不跨包 import 私有名）。"""
    wrapped = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def _split_glue(line: str) -> list[str]:
    """把粘在样本半行后面的控制行拆出来（固件缺陷，PC 侧绕过）。

    与 `实验/wheelmap_sweep.py::_split_glue` 同一个现场现象：`RUN`/`STOP` 的 handler
    调 `resetRunBuf()` 清环时，若某行 `I`/`D` 样本只发出去一半，剩下的字节（含行尾
    `\\n`）被丢在环外 ⇒ 下一条回复（如 `OK STOP`）**粘在那半行屁股上**。现场原文：

        I 2410259 0 0 0 0 0 0 0OK M

    这里只认控制行词表 `_CTRL_TOKENS`；样本半行原样保留（上层按 `"I "`/`"D "`
    前缀归为噪声）。测试与 wheelmap_sweep 的版本对拍，两个实现漂移时当场变红。
    """
    out, rest = [], line
    while True:
        pos = None
        for tok in _CTRL_TOKENS:
            p = rest.find(tok)
            if p > 0 and (pos is None or p < pos):
                pos = p
        if pos is None:
            out.append(rest)
            break
        out.append(rest[:pos])
        rest = rest[pos:]
        nxt = None
        for tok in _CTRL_TOKENS:
            p = rest.find(tok, 1)
            if p > 0 and (nxt is None or p < nxt):
                nxt = p
        if nxt is None:
            out.append(rest)
            break
    return [s for s in out if s]


def _empty_stats(detail=None) -> dict:
    """`_wait_done` 的统计骨架（发送失败时也要有一个同形状的空壳）。"""
    stats = {"run_reply": None, "n_d": 0, "errors": [], "n_other": 0, "n_reset": 0}
    if detail is not None:
        stats["detail"] = detail
    return stats


def _default_link_factory(addr):
    """默认重连工厂：延迟 import `实验/car_link.py`，用同一地址重建一条 TCP。

    **只在真的要重连时才 import**：`vgt` 的模块级依赖里不放硬件入口；没有车的机器
    （跑单测）永远走不到这里。`addr` 未知时退回 UDP 自动发现（约 3 s）。
    """
    from car_link import CarLink

    if addr is None:
        return CarLink()
    return CarLink(ip=addr[0], port=addr[1], auto_discover=False)


# ---- harness ----------------------------------------------------------------------


class Harness:
    """视觉闭环归位的全部状态：一条链路、一台相机、一份标定。

    一个实例对应"一次会话"：边界码像素基线（R18）在这一个实例里只抓一次。
    重连（失控保护）不会重建实例，也不会动基线 —— 相机没动，基线就该继续有效。
    """

    def __init__(self, car_link, camera, calib, home_xy=None, *,
                 link_factory=None,
                 done_timeout_s=DEFAULT_DONE_TIMEOUT_S,
                 silence_timeout_s=DEFAULT_SILENCE_TIMEOUT_S,
                 reply_timeout_s=DEFAULT_REPLY_TIMEOUT_S,
                 handshake_timeout_s=DEFAULT_HANDSHAKE_TIMEOUT_S,
                 locate_tries=DEFAULT_LOCATE_TRIES):
        """`home_xy` 的哨兵语义见类 docstring 与模块 docstring 的专门一节。

        `link_factory`：失控保护重连时用来造新链路（无参可调用，返回 CarLink
        兼容对象）。默认 None 时用 `_default_link_factory`（延迟 import car_link，
        按传入链路的 `.addr` 重连）。单测注入假工厂即可全程不碰网络。

        构造**不做任何 I/O**：不开相机、不发命令、不建连接。
        """
        self.car_link = car_link
        self.camera = camera
        self.calib = calib

        if home_xy is None:
            self.home_xy = (0.0, 0.0)
            self.home_xy_explicit = False
        else:
            try:
                hx, hy = float(home_xy[0]), float(home_xy[1])
            except (TypeError, IndexError, ValueError):
                raise ValueError(
                    f"home_xy 要 (x_mm, y_mm) 两个数，实际 {home_xy!r}。"
                    "（默认 None 是哨兵：名义场地中心、宽松口径）") from None
            if not (math.isfinite(hx) and math.isfinite(hy)):
                raise ValueError(f"home_xy 必须是有限值，实际 {home_xy!r}")
            self.home_xy = (hx, hy)
            self.home_xy_explicit = True

        # 会话状态。
        self.reset_lines: list[tuple[float, str]] = []   # (wall, "RESET ...")，R58
        self.last_locate_reason: str | None = None
        self._boundary_px_baseline: dict[int, tuple[float, float]] | None = None

        # 时序参数（测试传小值即可全速跑）。
        self.done_timeout_s = float(done_timeout_s)
        self.silence_timeout_s = float(silence_timeout_s)
        self.reply_timeout_s = float(reply_timeout_s)
        self.handshake_timeout_s = float(handshake_timeout_s)
        self.locate_tries = int(locate_tries)

        self._addr = getattr(car_link, "addr", None)
        if link_factory is None:
            self._link_factory = lambda: _default_link_factory(self._addr)
        else:
            self._link_factory = link_factory

    # ---- 视觉：单帧真值 ------------------------------------------------------------

    def locate(self):
        """抓一帧、解出车体在场地系里的 `(x_mm, y_mm, yaw_deg)`；解不出返回 `None`。

        失败原因写进 `last_locate_reason`（每次调用先清空）：
        `"boundary_moved"`（缺任一边界码 / 建不出场地系）/ `"no_vehicle_marker"`
        （缺 ID29 / 位姿不可用），与 `verify_vision.solve_image` 同一词表。

        - 抓帧走 `camera.grab()` 的**默认路径** `fresh=True`（R54/C1）：读前先丢掉
          驱动队列里排队的老帧，否则"等 0.5 s 再 locate"拿到的是运动中的旧画面。
        - `detect_markers` 契约上不抛（除 MemoryError）；本函数不吞 MemoryError。
        - 边界码漂移复核对**每一次** `locate()` 生效（R18），超限抛
          `BoundaryMarkersMoved`；基线在第一次成功解出车体位姿时抓取。
        - 相机读帧失败（`CameraUnavailable`）不在这里兜底 —— 同一根句柄重试没有
          意义（R39），让上层带着异常结束本轮。
        """
        self.last_locate_reason = None
        frame = self.camera.grab()          # 默认 fresh=True（R54/C1），别改成 False
        poses = detect_markers(frame, self.calib)

        missing = [mid for mid in calib_store.BOUNDARY_IDS if mid not in poses]
        if missing:
            self.last_locate_reason = REASON_BOUNDARY_MOVED
            return None
        try:
            ff = build_field_frame(poses)
        except ValueError:
            # 四点退化到构不成平面（数学上≈不可能，检测成功后只剩理论空间）：
            # 与 verify_vision 同一口径，算"这一帧没有场地系"。
            self.last_locate_reason = REASON_BOUNDARY_MOVED
            return None

        self._check_boundary_drift(ff)

        vehicle_id = int(calib_store.VEHICLE_ID)
        if vehicle_id not in poses:
            self.last_locate_reason = REASON_NO_VEHICLE
            return None
        try:
            x, y, yaw = body_pose_in_field(ff, poses[vehicle_id], self.calib["bias"])
        except ValueError:
            self.last_locate_reason = REASON_NO_VEHICLE
            return None

        if self._boundary_px_baseline is None:
            # R18：每个实例只在**第一次成功解出车体位姿**这一帧抓一次基线。
            # 用 FieldFrame.marker_px 口径（4 个角点的图像坐标均值），与后续帧
            # 的同一个量对比；不是角点之间的距离、不是重投影 RMS。
            self._boundary_px_baseline = {
                mid: (float(px[0]), float(px[1])) for mid, px in ff.marker_px.items()
            }
        return (float(x), float(y), float(yaw))

    def _check_boundary_drift(self, ff) -> None:
        """与基线比 4 个边界码的角点均值像素；任一码 > `BOUNDARY_DRIFT_TOL_PX` 抛。"""
        base = self._boundary_px_baseline
        if base is None:
            return
        for mid, px in ff.marker_px.items():
            dx = float(px[0]) - base[mid][0]
            dy = float(px[1]) - base[mid][1]
            dist_px = math.hypot(dx, dy)
            if dist_px > BOUNDARY_DRIFT_TOL_PX:
                raise BoundaryMarkersMoved(
                    f"边界码 ID{mid} 相对本次会话基线移动了 {dist_px:.1f} px"
                    f"（阈值 {BOUNDARY_DRIFT_TOL_PX:.0f} px，R18）："
                    f"基线 ({base[mid][0]:.1f}, {base[mid][1]:.1f}) → "
                    f"本帧 ({float(px[0]):.1f}, {float(px[1]):.1f})。"
                    "码被撞/没贴平，或者**相机被挪了** —— 两种都会让场地系失真，"
                    "必须人工看一眼；恢复原状后重开一个 Harness（基线每个实例只抓一次）。"
                )

    # ---- 控制：闭环归位 ------------------------------------------------------------

    def drive_to(self, x_mm, y_mm, tol_mm=DEFAULT_TOL_MM, max_iter=DEFAULT_MAX_ITER, *,
                 pulse_ms=DEFAULT_PULSE_MS, settle_s=DEFAULT_SETTLE_S,
                 v_max_mps=MAX_CMD_MPS):
        """P 控制把车开到场地系 `(x_mm, y_mm)`，返回结果字典（键见模块 docstring）。

        每个周期：`locate()`（失败重试 `locate_tries` 次）→ 算误差 → 够近就收工 →
        `RUN vx vy 0 <ms>` → 等 `DONE` → 睡 `settle_s` → 下一圈。`max_iter` 是
        **脉冲上限**；收敛判断发生在每次 `locate()` 之后（所以最后一次脉冲的效果
        一定会被再看一眼，`final` 永远是最新测量）。

        `pulse_ms`：单条 RUN 的时长（brief 默认 400；MIN_DUTY 地板下这是步长的
        唯一旋钮，见模块 docstring）。`settle_s`：DONE 之后等车停稳再抓帧。
        `v_max_mps`：速度指令上限（P 律的 clamp 上界）。

        失败语义：

        - `locate()` 抛 `BoundaryMarkersMoved` → **立即向上冒泡**（"立即中止"，R18）；
        - RUN 等待失败（超时/静默/断链）→ 失控保护（STOP；STOP 不确认就重连+STOP），
          恢复成功则继续下一圈（这一圈的位移已经算数），失败抛 `StopFailed`；
        - 固件回 `ERR`（RUN 被拒）→ 收工，`reason="run_rejected"`；
        - `locate()` 连续失败 → 收工，`reason="locate_failed"`；
        - 脉冲用满还没进 `tol_mm` → 收工，`reason="max_iter"`。
        """
        x_t, y_t = float(x_mm), float(y_mm)
        self._check_target_in_field(x_t, y_t)
        tol = float(tol_mm)
        if not (tol > 0.0):
            raise ValueError(f"tol_mm 必须是正数，实际 {tol_mm!r}")
        iter_cap = int(max_iter)
        if iter_cap < 1:
            raise ValueError(f"max_iter 至少 1，实际 {max_iter!r}")
        ms = int(round(float(pulse_ms)))
        if ms < 1:
            raise ValueError(f"pulse_ms 至少 1 整数毫秒（固件 RUN 的 ms），实际 {pulse_ms!r}")
        settle = float(settle_s)
        if settle < 0.0:
            raise ValueError(f"settle_s 不能为负，实际 {settle_s!r}")
        vmax = float(v_max_mps)
        if not (vmax >= MIN_CMD_MPS):
            raise ValueError(
                f"v_max_mps 必须 ≥ {MIN_CMD_MPS}（P 律的下限；比它低的话固件侧也"
                f"会被 MIN_DUTY 抬起来），实际 {v_max_mps!r}")

        events: list[dict] = []
        trace: list[dict] = []

        # 开场红灯：上一会话若留下没停的 RUN（失控保护漏网的场景），先兜底停一次。
        open_stop_ok = self._try_stop(self.car_link)
        events.append({"event": "open_stop", "ok": open_stop_ok})
        if not open_stop_ok:
            warnings.warn(
                "drive_to 开场 STOP 未确认：车若本来就在跑，这里没拦住。"
                "继续跑闭环（下一次 locate 能看到真实位置），但先人工确认一下车是停的。",
                RuntimeWarning, stacklevel=2)
        self._drain_pending(self.car_link)

        final = None
        iters = 0
        reason = "locate_failed"
        pos = self._locate_retry()
        while pos is not None:
            final = pos
            ex, ey = x_t - pos[0], y_t - pos[1]
            dist = math.hypot(ex, ey)
            if dist <= tol:
                reason = "converged"
                break
            if iters >= iter_cap:
                reason = "max_iter"
                break

            vx, vy = body_command(ex, ey, pos[2], v_max=vmax)
            iters += 1
            run_line = "RUN {:.4f} {:.4f} 0 {}".format(vx, vy, ms)
            rec = {"iter": iters, "pos": pos, "err_mm": (ex, ey), "dist_mm": dist,
                   "cmd": (vx, vy, ms), "run_line": run_line}
            try:
                self._send_line(self.car_link, run_line)
            except Exception as exc:                       # 发不出去 = 链路有问题
                outcome = "closed"
                stats = _empty_stats(f"发送失败：{type(exc).__name__}: {exc}")
                rec["send_error"] = stats["detail"]
            else:
                outcome, stats = self._wait_done(self.car_link)
            rec["run_reply"] = stats.get("run_reply")
            rec["n_d"] = stats.get("n_d", 0)
            rec["outcome"] = outcome

            if outcome == "done":
                trace.append(rec)
                time.sleep(settle)
                pos = self._locate_retry()
                if pos is None:
                    reason = "locate_failed"
                continue
            if outcome == "error":
                rec["errors"] = list(stats.get("errors", []))
                trace.append(rec)
                reason = "run_rejected"                        # RUN 被拒 = 没在跑
                break

            # timeout / silent / closed → 失控保护（内部可能抛 StopFailed）。
            rec["detail"] = stats.get("detail")
            self._recover_after_failed_run(outcome, stats, events, rec)
            trace.append(rec)
            time.sleep(settle)
            pos = self._locate_retry()
            if pos is None:
                reason = "locate_failed"

        stop_confirmed = self._try_stop(self.car_link)
        if not stop_confirmed:
            warnings.warn(
                "drive_to 收尾 STOP 未确认：RUN 结束时固件已自停（或失控保护里停过），"
                "这里只是清环+报平安；稳妥起见人工确认车是停的。",
                RuntimeWarning, stacklevel=2)
        return {
            "reached": reason == "converged",
            "iters": iters,
            "final": final,
            "reason": reason,
            "trace": trace,
            "events": events,
            "reset_lines": list(self.reset_lines),
            "stop_confirmed": stop_confirmed,
        }

    def _check_target_in_field(self, x_mm, y_mm) -> None:
        """归位目标必须让车体半包络落在场地内（与 matrix 的严格分支同口径）。

        车中心可达域：|x| ≤ 252.5−135 = 117.5、|y| ≤ 125.9−74.5 = 51.4 mm。
        越界的目标意味着"开着电机去撞边界码/撞桌沿"，宁可当场报错。
        """
        x_lim = matrix.FIELD_HALF_LENGTH_MM - matrix.CAR_HALF_LENGTH_MM
        y_lim = matrix.FIELD_HALF_WIDTH_MM - matrix.CAR_HALF_WIDTH_MM
        if abs(x_mm) > x_lim + 1e-9:
            raise ValueError(
                f"归位目标 x={x_mm:.1f} mm 越出场地：车体半长 {matrix.CAR_HALF_LENGTH_MM:.1f} mm，"
                f"车中心 |x| 最多 {x_lim:.1f} mm（场地半长 {matrix.FIELD_HALF_LENGTH_MM:.1f}）。")
        if abs(y_mm) > y_lim + 1e-9:
            raise ValueError(
                f"归位目标 y={y_mm:.1f} mm 越出场地：车体半宽 {matrix.CAR_HALF_WIDTH_MM:.1f} mm，"
                f"车中心 |y| 最多 {y_lim:.1f} mm（场地半宽 {matrix.FIELD_HALF_WIDTH_MM:.1f}）。")

    def _locate_retry(self):
        """`locate()` 失败（掉码/反光）重试几次；`BoundaryMarkersMoved` 不重试。"""
        tries = max(1, self.locate_tries)
        for k in range(tries):
            pos = self.locate()
            if pos is not None:
                return pos
            if k + 1 < tries:
                time.sleep(_LOCATE_RETRY_PAUSE_S)
        return None

    # ---- 实验编排：单次 trial（Task 11） --------------------------------------------

    def run_trial(self, exp, *, mid_fraction=DEFAULT_MID_FRACTION,
                  settle_before_start_s=DEFAULT_START_SETTLE_S,
                  settle_after_done_s=DEFAULT_AFTER_DONE_SETTLE_S,
                  c_1rev=ODOM_C_1REV, sign=ODOM_SIGN,
                  l_xy_over_r=ODOM_LXY_OVER_R, r_eff_mm=ODOM_R_EFF_MM):
        """跑一条实验（`vgt.matrix.Experiment`）：起/中/终三点视觉真值 + 流式遥测 + 里程计。

        返回字典（`ok=False` 的 `reason` 词表见模块 docstring）：

            {"ok", "reason", "exp_id", "run_line", "run_ms",
             "vision": {"start", "mid", "end"},          # None = 该帧没解出来
             "vision_delta": {"dist_mm", "yaw_deg", "dx_mm", "dy_mm"} | None,
             "odometry_delta": {"dist_mm", "yaw_deg", "dx_mm", "dy_mm"} | None,
             "samples", "n_samples", "mid_capture_failed",
             "sign_ok", "sign_dot_mm2", "yaw_sign_ok",     # 有符号判据（Important 3）
             "stop_confirmed", "events", "reset_lines", "warnings"}

        **失败不抛**（标 `reason` 交给批处理继续）；例外只有三类：

        1. 必要 STOP 无法确认且没有停车证据 → `StopFailed`（收到它请**给车断电**）；
        2. `exp`/参数非法 → `ValueError`（连开场 STOP 都不发）；
        3. `KeyboardInterrupt` 等原样冒泡 —— 但收尾 STOP 已经在 finally 里发过。

        任何一次 RUN 之后都有停车证据链：`DONE`（固件先 `stopAll()` 再发，
        main.cpp:592-598）或确认过的 `OK STOP`；两者都没有、收尾 STOP 也发不出去时，
        本方法**只能**抛 `StopFailed` —— 不许静默把"不知道停没停"当成功。

        时序与抓帧新鲜度（R54）见模块 docstring 的 `run_trial` 一节。
        `c_1rev/sign/l_xy_over_r/r_eff_mm` 直接透传给 `odometry.integrate_encoders`。
        """
        ms = int(round(float(exp.duration_s) * 1000.0))
        if ms < 1:
            raise ValueError(
                f"实验 #{getattr(exp, 'id', '?')} 的 duration_s={exp.duration_s!r} 取整后 "
                f"是 {ms} ms：固件 RUN 的 ms 必须 ≥ 1。")
        frac = float(mid_fraction)
        if not (0.0 < frac < 1.0):
            raise ValueError(f"mid_fraction 必须在 (0,1) 里，实际 {mid_fraction!r}")
        settle_start = float(settle_before_start_s)
        settle_done = float(settle_after_done_s)
        if settle_start < 0.0 or settle_done < 0.0:
            raise ValueError(
                f"沉降等待不能为负：settle_before_start_s={settle_start!r}、"
                f"settle_after_done_s={settle_done!r}")

        run_line = "RUN {:.4f} {:.4f} {:.4f} {}".format(
            float(exp.vx), float(exp.vy), float(exp.om), ms)
        cfg = {"mid_fraction": frac, "settle_before_start_s": settle_start,
               "settle_after_done_s": settle_done}

        events: list[dict] = []
        warns: list[str] = []
        samples: list[dict] = []
        # state 记录"做到哪一步了"，收尾时据此判断 StopFailed 的严重度：
        #   run_sent      —— 已经给车发过 RUN（发之前车不会有我们造成的运动）；
        #   stop_evidence —— RUN 之后有停车证据（DONE 或确认过的 OK STOP）；
        #   baseline_resets —— 开场时的 RESET 文本快照（R56 定性用）。
        state = {"run_sent": False, "stop_evidence": False,
                 "baseline_resets": frozenset()}

        result = {
            "ok": False,
            "reason": None,
            "exp_id": int(getattr(exp, "id", -1)),
            "run_line": run_line,
            "run_ms": ms,
            "vision": {"start": None, "mid": None, "end": None},
            "vision_delta": None,
            "odometry_delta": None,
            "samples": samples,
            "n_samples": 0,
            # R66：停车后补读编码器（量惯性滑行）的结果。`None` = 这一步没走到
            # （起点帧就失败之类）；`True` = 走了但读不到/读数不可信，此时
            # `odometry_delta` 的窗口短于 `vision_delta`，两者不可直接比。
            "coast_read_failed": None,
            "mid_capture_failed": False,
            "sign_ok": None,
            "sign_dot_mm2": None,
            "yaw_sign_ok": None,
            "stop_confirmed": False,
            "events": events,
            "reset_lines": [],
            "warnings": warns,
        }

        pending = None
        try:
            self._trial_body(exp, cfg, state, result, run_line, ms)
        except StopFailed as exc:
            pending = exc                       # 已经是最严重的状态，别包装
        except CameraUnavailable as exc:
            result["reason"] = "camera_failed"
            warns.append(f"关键帧抓帧失败（{type(exc).__name__}）：{exc}")
        except BoundaryMarkersMoved as exc:
            result["reason"] = "boundary_moved"
            result["boundary_error"] = str(exc)
            warns.append(f"边界码漂移超限：{exc}")
        except Exception as exc:
            result["reason"] = "trial_error"
            warns.append(f"未预期异常 {type(exc).__name__}：{exc}")
        except BaseException as exc:            # KeyboardInterrupt 等：停车后再冒泡
            pending = exc
        finally:
            # 硬要求：实验结束**无论如何**都发 STOP（异常/中断路径也走到这里）。
            stopped = self._try_stop(self.car_link)
            result["stop_confirmed"] = stopped
            events.append({"event": "final_stop", "ok": stopped})
            if not stopped:
                msg = ("收尾兜底 STOP 未确认：链路可能已死/固件卡住，不能从本次编排"
                       "推断车已停（R58：WiFi 掉线时固件不执行 RUN 的到时停车）。")
                warns.append(msg)
                warnings.warn(msg, RuntimeWarning, stacklevel=2)

        if pending is not None:
            if isinstance(pending, StopFailed):
                raise pending
            if not result["stop_confirmed"] and state["run_sent"] and not state["stop_evidence"]:
                exc = StopFailed(
                    "run_trial 异常收尾时 STOP 未确认，且没有 DONE/OK STOP 的停车证据"
                    f"（本次已发过 RUN）。车可能还在开 —— 立即给车断电。"
                    f"（同时还有一个异常在冒泡：{type(pending).__name__}: {pending}）")
                exc.events = list(events)
                raise exc from pending
            raise pending

        # 收尾 STOP 之后统一算指标：失败条目也带上能算的证据（Task 12 要标注行）。
        # R66：`_odo_samples` 是 `_trial_body` 备好的"含惯性滑行"的积分副本；None =
        # 没读到/读数不可信，退回只积固件 `D` 行（此时已记过 warning，不是静默降级）。
        self._trial_metrics(result, samples, c_1rev=c_1rev, sign=sign,
                            l_xy_over_r=l_xy_over_r, r_eff_mm=r_eff_mm,
                            odo_samples=result.pop("_odo_samples", None))
        if result["reason"] is None and result["sign_ok"] is False:
            # 有符号判据：量级对、方向反 —— 只比 dist_mm 会静默通过的那一类。
            result["reason"] = "sign_mismatch"
        result["ok"] = result["reason"] is None
        result["reset_lines"] = list(self.reset_lines)

        if not result["stop_confirmed"] and state["run_sent"] and not state["stop_evidence"]:
            exc = StopFailed(
                "run_trial 收尾 STOP 未确认，且没有收到 DONE（固件没有执行到停车）："
                "车可能还在开 —— 立即给车断电。")
            exc.events = list(events)
            raise exc
        return result

    def _trial_body(self, exp, cfg, state, result, run_line, ms):
        """`run_trial` 的主时序：开场 → 起点 → RUN → 中点 → 终点。

        只负责"做到哪里算哪里"并填 `result`；异常按语义留给 `run_trial` 归类
        （`StopFailed`/`CameraUnavailable`/`BoundaryMarkersMoved` 原样冒泡）。
        """
        events = result["events"]
        warns = result["warnings"]
        samples = result["samples"]

        # ① 开场：PING 探活（同时拿到 RESET 基线）→ STOP（清环 + 兜底停车）。
        #    开场就确认不了"链路活着且车已停"就别开实验 —— 上一轮若在失控保护上
        #    漏网，车可能还在开，起点帧和 RUN 都会发生在错误的状态上。
        if self._probe_resets() is None:
            self._reconnect_or_raise(events, "open_probe")
        if not self._try_stop(self.car_link):
            self._reconnect_or_raise(events, "open_stop")
        self._drain_pending(self.car_link)
        # RESET 基线：此刻已经见过的文本（R58 的 RESET 行每次 PING/ID 都重发，
        # 只有**新的**文本才是本段实验里的新复位证据）。
        state["baseline_resets"] = frozenset(text for _wall, text in self.reset_lines)

        # ② 起点：等车静止再抓帧。等待不保鲜（R54），新鲜度在 locate() 里。
        time.sleep(max(0.0, float(cfg["settle_before_start_s"])))
        start = self._locate_retry()
        if start is None:
            result["reason"] = self.last_locate_reason or "no_vehicle_marker"
            return
        result["vision"]["start"] = tuple(start)

        # ③ 下发 RUN；从流式遥测里触发中点抓帧（非致命）。
        mid_state = {"attempted": False}

        def on_sample(sample):
            if mid_state["attempted"] or sample["t_ms"] < ms * cfg["mid_fraction"]:
                return
            mid_state["attempted"] = True
            try:
                pos = self.locate()
            except Exception as exc:            # Review Focus #1：中点失败不致命
                result["mid_capture_failed"] = True
                warns.append(f"mid_capture_failed: {type(exc).__name__}: {exc}")
                return
            if pos is None:
                result["mid_capture_failed"] = True
                warns.append(f"mid_capture_failed: {self.last_locate_reason}"
                             "（降级为起/终两点）")
                return
            result["vision"]["mid"] = tuple(pos)

        try:
            self._send_line(self.car_link, run_line)
        except Exception as exc:
            # 整行没发出去 = 固件不会执行（它按行读）：但链路状态未知，按"等待失败"兜底。
            outcome, stats = "closed", _empty_stats(
                f"发送失败：{type(exc).__name__}: {exc}")
        else:
            state["run_sent"] = True
            outcome, stats = self._wait_done(self.car_link, samples=samples,
                                             on_sample=on_sample)

        rec = {"event": "run_wait", "outcome": outcome,
               "run_reply": stats.get("run_reply"), "n_d": stats.get("n_d", 0),
               "n_reset": stats.get("n_reset", 0), "n_other": stats.get("n_other", 0)}
        if stats.get("errors"):
            rec["errors"] = list(stats["errors"])
        if stats.get("detail"):
            rec["detail"] = stats["detail"]
        events.append(rec)
        result["run_reply"] = stats.get("run_reply")
        result["run_outcome"] = outcome

        if outcome == "error":
            # 固件回了 ERR：RUN 被拒（不可能在跑），不做失控保护，但绝不静默当成功。
            result["reason"] = "run_rejected"
            return

        if outcome != "done":
            # ④ 无 DONE：失控保护（STOP；不确认就重连 + STOP），再按 R56 定性。
            reset_done = self._recover_after_failed_run(outcome, stats, events, rec)
            state["stop_evidence"] = True       # 走到这里 = STOP 已在某条链路上确认
            if not reset_done:
                # 当前链路 STOP 确认了（链路活着）—— R56 仍要求强制重连 + 查 RESET，
                # 不拿"超时"直接当 brownout。
                new_link, why = self._try_reconnect()
                if new_link is not None:
                    self.car_link = new_link
                    events.append({"event": "timeout_diagnosis", "ok": True})
                else:
                    events.append({"event": "timeout_diagnosis", "ok": False,
                                   "detail": why})
                    warns.append(f"超时定性用的强制重连失败（{why}）；"
                                 "车已在恢复阶段 STOP 确认，实验仍判无效。")
            new_resets = [text for _wall, text in self.reset_lines
                          if text not in state["baseline_resets"]]
            if new_resets:
                result["new_reset_lines"] = new_resets
                result["reason"] = "board_reset"
            else:
                result["reason"] = "deadline_timeout"
            return

        # ⑤ DONE：固件在发 DONE 前已 stopAll（main.cpp:592-598）—— 这是停车证据。
        state["stop_evidence"] = True
        time.sleep(max(0.0, float(cfg["settle_after_done_s"])))
        end = self._locate_retry()
        if end is None:
            result["reason"] = self.last_locate_reason or "no_vehicle_marker"
            return
        result["vision"]["end"] = tuple(end)

        if len(samples) < 2:
            # 收到 DONE 却没有样本流：不能拿"0 位移"当"没动"（静默假数据）。
            result["reason"] = "no_telemetry"
            return

        # ⑥ R66：补读停车后的编码器，把惯性滑行并进里程计窗口（见 `coasted_samples`）。
        #    **必须放在终点帧之后**：车已停、计数已冻结，这一读覆盖的时段才包住
        #    视觉的终点帧，两个窗口才算一样长。
        coast = self._read_idle_enc(self.car_link)
        if coast is None:
            result["coast_read_failed"] = True
            warns.append(
                "coast_read_failed：停车后读不到空闲编码器（`STREAM 1` 的 `I` 行）。"
                "本次 `odometry_delta` 只覆盖电机通电期，比把滑行也量进去的 "
                "`vision_delta` 短一截（停车是自由滑行，固件没有刹车），两者不可直接比。")
            result["_odo_samples"] = None
            return
        result["coast_read_failed"] = False
        result["coast_enc"] = list(coast["enc"])
        merged = coasted_samples(samples, coast)
        if merged is samples:
            warns.append(
                f"coast_read_discarded：停车后读到的编码器 {list(coast['enc'])} 不比最后"
                f"一条 D 样本 {list(samples[-1]['enc'])} 大（或本段没转过）—— 读数不可信，"
                "丢弃（多半是 RUN 之前的旧 I 行）。本次里程计窗口仍短于视觉窗口。")
            result["_odo_samples"] = None
        else:
            result["coast_counts"] = [b - a for a, b
                                      in zip(samples[-1]["enc"], coast["enc"])]
            result["_odo_samples"] = merged

    def _probe_resets(self):
        """在当前链路上 PING 一次（探活），并把回复连同 RESET 行收干净。

        返回本次回复里收集到的 RESET 文本集合；发不出去/等不到 PONG → None。
        固件把 `RESET …` 单独一行发在 PONG **之后**（main.cpp:400-401），
        同一 recv 批次可能读不到，所以等到 PONG 后再补一次短 drain —— 漏了它，
        它会淌到下一轮读取里，把后面的证据弄脏。
        """
        marker = len(self.reset_lines)
        try:
            self._send_line(self.car_link, "PING")
        except Exception:
            return None
        reply, _noise = self._wait_reply(self.car_link, "PONG", self.reply_timeout_s)
        if reply is None:
            return None
        try:
            self._drain(self.car_link, _PROBE_TAIL_DRAIN_S)
        except ConnectionError:
            pass
        return {text for _wall, text in self.reset_lines[marker:]}

    def _read_idle_enc(self, link, *, timeout_s=_COAST_ENC_TIMEOUT_S,
                       max_lines=_COAST_ENC_LINES):
        """读一次**空闲**（非 RUN）编码器累计值：`STREAM 1` → 收 `I` 行 → `STREAM 0`。

        RUN 之外固件只在 `stream_idle` 打开时发样本（main.cpp:698-699），所以这是
        停车后唯一能拿到编码器的途径 —— R66 用它量惯性滑行。

        返回 `{"enc": [4 int], "rpy": [3 float] | None}`，或 `None`（发不出去 / 链路
        读失败 / 没收到合法 `I` 行）。**非致命**：调用方只记 warning，不改 `reason`。

        取**已收到的那几行里 `sum(enc)` 最大的一条**：车已停、计数只增，最大的一条就是
        最新的；顺带挡掉一条 RUN 之前残留的旧 `I` 行（它的计数更小）。

        `STREAM 0` 在**任何**返回路径上都会发：留着空闲流会让之后的每条命令回复都掺
        `I` 行，把 `_wait_reply` 的预期对齐弄歪。
        """
        try:
            self._send_line(link, "STREAM 1")
        except Exception:
            return None
        found: list[dict] = []
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            try:
                lines, resets = self._drain(link, _RECV_POLL_S)
            except ConnectionError:
                break
            if resets:
                self._record_resets(resets)
            new = odometry.parse_telemetry(lines, tags=("I",))
            found.extend(new)
            if len(found) >= max_lines:
                break
            if found and not new:
                break        # 已经有读数、这一轮又没有新的 —— 够了，别空转到超时
            time.sleep(0.005)
        try:                                    # 关掉空闲流（成败都关）
            self._send_line(link, "STREAM 0")
            self._wait_reply(link, "OK STREAM", self.reply_timeout_s)
        except Exception:
            pass
        if not found:
            return None
        best = max(found, key=lambda s: sum(s["enc"]))
        return {"enc": [int(v) for v in best["enc"]],
                "rpy": [float(v) for v in best["rpy"]]}

    def _reconnect_or_raise(self, events, phase):
        """STOP 未确认时的强制恢复：关旧链路 → 重连（握手 + STOP）。失败抛 `StopFailed`。

        与 `_recover_after_failed_run` 的区别：这里**没有**"RUN 失败"这个前提
        （开场探活就失败也走这里），异常消息按 `phase` 交代是哪一步。
        """
        new_link, why = self._try_reconnect()
        if new_link is None:
            exc = StopFailed(
                f"{phase}：STOP 未确认，重连/再 STOP 也失败（{why}）。"
                "无法确认车已停 —— 固件在 WiFi 掉线时不执行 RUN 的到时停车（R58），"
                "若有残留 RUN，车可能还在开；立即给车断电。")
            exc.events = list(events) + [{"event": phase, "ok": False, "detail": why}]
            raise exc
        self.car_link = new_link
        events.append({"event": phase, "ok": True, "reconnected": True})

    def _trial_metrics(self, result, samples, *, c_1rev, sign, l_xy_over_r, r_eff_mm,
                       odo_samples=None):
        """把样本积成里程计位移，并算净位移/净转角与有符号判据（纯计算，不碰 I/O）。

        `odo_samples`：**只喂给积分**的样本序列（默认 `samples`）。R66 用它把停车后的
        惯性滑行补进来，好让里程计窗口和视觉窗口一样长（见 `coasted_samples`）；
        `result["samples"]` 里保留的仍是固件原样的 `D` 行，不掺合成样本。
        """
        result["n_samples"] = len(samples)
        odo_in = samples if odo_samples is None else odo_samples
        odo_last = None
        if len(odo_in) >= 2:
            try:
                traj = odometry.integrate_encoders(
                    odo_in, c_1rev=c_1rev, sign=sign,
                    l_xy_over_r=l_xy_over_r, r_eff_mm=r_eff_mm)
            except ValueError as exc:
                result["warnings"].append(f"里程计积分失败：{exc}")
            else:
                odo_last = (float(traj[-1][0]), float(traj[-1][1]), float(traj[-1][2]))
                result["odometry_delta"] = {
                    "dist_mm": math.hypot(odo_last[0], odo_last[1]),
                    "yaw_deg": odo_last[2],
                    "dx_mm": odo_last[0],
                    "dy_mm": odo_last[1],
                }

        start = result["vision"]["start"]
        end = result["vision"]["end"]
        if start is None or end is None:
            return
        vdx, vdy = end[0] - start[0], end[1] - start[1]
        vyaw = _wrap180_deg(end[2] - start[2])
        result["vision_delta"] = {
            "dist_mm": math.hypot(vdx, vdy),
            "yaw_deg": vyaw,
            "dx_mm": vdx,
            "dy_mm": vdy,
        }
        if odo_last is None:
            return
        # 里程计的位移在**起点的车体系**里；比方向先转到场地系（同一条符号链）。
        fdx, fdy = body_delta_to_field(odo_last[0], odo_last[1], start[2])
        result["sign_dot_mm2"] = vdx * fdx + vdy * fdy
        result["sign_ok"] = translation_sign_ok((vdx, vdy), (fdx, fdy))
        result["yaw_sign_ok"] = _yaw_sign_ok(vyaw, odo_last[2])

    # ---- 失控保护 ------------------------------------------------------------------

    def _recover_after_failed_run(self, outcome, stats, events, rec) -> bool:
        """RUN 等待失败后的失控保护；恢复不了抛 `StopFailed`。

        先试当前链路 STOP：确认 `OK STOP` 就**不重连** —— 链路活着、车已停，缺的
        DONE 很可能只是固件环形缓冲满时被丢（R58 的已知现象），重连只会白折腾。
        没确认（链路死/固件卡）才关旧链路、重连、第一件事 STOP 再确认。

        返回 **True = 重连换过链路**（R56 的定性证据要从新链路的握手/查 RESET 里拿）；
        False = 当前链路 STOP 已确认（链路没换）。
        """
        stopped = self._try_stop(self.car_link)
        rec["stop_after_failure"] = stopped
        events.append({"event": "run_wait_failed", "outcome": outcome,
                       "detail": stats.get("detail"), "stopped_on_current_link": stopped})
        if stopped:
            return False
        new_link, why = self._try_reconnect()
        if new_link is not None:
            self.car_link = new_link
            events.append({"event": "link_reset", "ok": True})
            return True
        events.append({"event": "link_reset", "ok": False, "detail": why})
        exc = StopFailed(
            f"RUN 等待失败（{outcome}）后：当前链路 STOP 未确认，重连/STOP 也失败（{why}）。"
            "固件在 WiFi 掉线时不会执行 RUN 的到时停车（loop 在采样段前 return），"
            "车可能还在开 —— 立即给车断电，不要用手去拦。")
        exc.events = list(events)
        raise exc

    def _try_stop(self, link) -> bool:
        """发 STOP 并尽力确认 `OK STOP`；任何链路异常都返回 False（不抛）。

        `OK STOP` 是"车停了"的唯一证据：STOP 的 handler 先 `stopAll()` 再回复。
        """
        try:
            self._send_line(link, "STOP")
        except Exception:
            return False
        reply, _noise = self._wait_reply(link, "OK STOP", self.reply_timeout_s)
        return reply is not None

    def _try_reconnect(self):
        """关旧链路 → 建新链路 → PING/PONG 握手 → 第一件事 STOP。

        返回 `(link | None, why | None)`；失败原因写进 `why`（给 `StopFailed` 用）。
        先关旧链路：固件是单客户端 TCP，旧 socket 不关（R58 僵尸 client），
        新的连不进来 / 连进来也没有回复。
        """
        self._close_quiet(self.car_link)
        factory = self._link_factory
        try:
            link = factory()
        except Exception as exc:
            return None, f"重连失败：{type(exc).__name__}: {exc}"
        if link is None:
            return None, "重连工厂返回 None"
        if not self._handshake(link):
            self._close_quiet(link)
            return None, f"重连后 {self.handshake_timeout_s:g} s 内没等到 PONG"
        # 新会话第一件事 STOP：清环形缓冲积压 + 确保停车（wheelmap 的同款做法）。
        if not self._try_stop(link) and not self._try_stop(link):
            self._close_quiet(link)
            return None, "重连后的 STOP 两次都没确认"
        return link, None

    def _handshake(self, link) -> bool:
        """重连后循环发 PING 直到 PONG（固件对陈旧 client 的接受有滞后，R56）。

        期间的 `RESET` 行会被收集进 `reset_lines`（brownout 证据）。
        """
        deadline = time.monotonic() + self.handshake_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            try:
                self._send_line(link, "PING")
            except Exception:
                return False
            reply, _noise = self._wait_reply(link, "PONG", min(1.0, remaining))
            if reply is not None:
                return True

    @staticmethod
    def _close_quiet(link) -> None:
        """尽力关掉一条链路的 socket（重连前必须做：固件单客户端）。"""
        sock = getattr(link, "sock", None)
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
            try:
                link.sock = None
            except Exception:
                pass                    # 只读属性的壳子：关掉就够，置 None 只是整洁

    # ---- 链路读写 ------------------------------------------------------------------

    def _send_line(self, link, text: str) -> None:
        """发一行（不带等待）。走 CarLink 的 `cmd(wait=False)` 发送路径。

        失败（断链）原样抛给调用方 —— 只有 `drive_to` 知道此时该走哪支。
        """
        link.cmd(text, wait=False)

    def _drain_pending(self, link) -> None:
        """把链路上已经积压的旧字节丢掉（残帧会冒充本会话的回复）。

        只在 `drive_to` 开场做一次：之后的每次读取都紧跟一条自己的命令，缓冲区里
        不该再有别人家的东西。
        """
        try:
            _lines, resets = self._drain(link, _RECV_POLL_S)
        except ConnectionError:
            return
        if resets:
            self._record_resets(resets)

    def _drain(self, link, budget_s: float):
        """在 `budget_s` 秒内把 `link.sock` 里能读到的字节读干净，拆成整行。

        返回 `(lines, resets)`：`RESET` 行**单独拿出来**（R58 硬要求：它跟在
        PONG/ID 回复后面，不单独拿出来就会把之后每条命令的回复整体错位一行）；
        其余整行先过 `_split_glue` 拆粘连。对端关闭/读错误 → `ConnectionError`
        （调用方按"链路死了"处置），`socket.timeout` 只是"这一刻没数据"。
        """
        sock = getattr(link, "sock", None)
        if sock is None:
            raise ConnectionError("链路没有 socket（已关闭？）")
        deadline = time.monotonic() + budget_s
        sock.settimeout(_RECV_POLL_S)
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                break
            except OSError as exc:
                raise ConnectionError(f"链路读失败：{exc!r}") from exc
            if not chunk:
                raise ConnectionError("对端关闭了连接")
            link.buf += chunk

        lines, resets = [], []
        while b"\n" in link.buf:
            raw, link.buf = link.buf.split(b"\n", 1)
            text = raw.decode("utf-8", "replace").strip()
            if not text:
                continue
            for part in _split_glue(text):
                if part.startswith("RESET"):
                    resets.append(part)
                else:
                    lines.append(part)
        return lines, resets

    def _wait_reply(self, link, want: str, timeout_s: float):
        """等一条以 `want` 开头的回复；`RESET` 行跳过并收集（R58）。

        返回 `(reply | None, noise)`：noise = 吞掉的非目标行数（D/I 样本等）。
        链路读失败 → (None, noise)（调用方按"没等到"处置）。
        """
        deadline = time.monotonic() + timeout_s
        noise = 0
        while time.monotonic() < deadline:
            try:
                lines, resets = self._drain(link, _RECV_POLL_S)
            except ConnectionError:
                return None, noise
            if resets:
                self._record_resets(resets)
            for ln in lines:
                if ln.startswith(want):
                    return ln, noise
                noise += 1
            time.sleep(0.002)
        return None, noise

    def _wait_done(self, link, *, samples=None, on_sample=None):
        """发完 RUN 后等 `DONE`（判"跑完"只认 DONE，R56）。

        返回 `(outcome, stats)`：

        - `"done"`：收到 DONE（固件已按到时停车）；
        - `"error"`：收到 `ERR` 行（RUN 被拒 —— 不可能在跑）；
        - `"timeout"`：`done_timeout_s` 内没等到 DONE；
        - `"silent"`：连续 `silence_timeout_s` 没收到任何字节（链路死了）；
        - `"closed"`：socket 断了。

        `stats`：`run_reply` / `n_d` / `errors` / `n_other` / `n_reset`（+可选 detail）。

        `samples` / `on_sample`（Task 11）：每条 `D` 行经 `odometry.parse_telemetry`
        解析后 append 进 `samples`、再喂给 `on_sample`（中点抓帧的触发器）。`DONE`
        之前的最后一条样本（固件停车后补发、跟着才发 DONE）在同一 recv 批次里排在
        DONE **前面**，不会漏。
        """
        deadline = time.monotonic() + self.done_timeout_s
        last_rx = time.monotonic()
        stats = _empty_stats()
        while time.monotonic() < deadline:
            try:
                lines, resets = self._drain(link, _RECV_POLL_S)
            except ConnectionError as exc:
                stats["detail"] = str(exc)
                return "closed", stats
            if resets:
                stats["n_reset"] += len(resets)
                self._record_resets(resets)
                last_rx = time.monotonic()
            for ln in lines:
                last_rx = time.monotonic()
                if ln == "DONE":
                    return "done", stats
                if ln.startswith("D "):
                    stats["n_d"] += 1
                    if samples is not None or on_sample is not None:
                        for sample in odometry.parse_telemetry([ln]):
                            if samples is not None:
                                samples.append(sample)
                            if on_sample is not None:
                                on_sample(sample)
                elif ln.startswith("OK RUN"):
                    stats["run_reply"] = ln
                elif ln.startswith("ERR"):
                    stats["errors"].append(ln)
                    return "error", stats
                else:
                    stats["n_other"] += 1
            if time.monotonic() - last_rx > self.silence_timeout_s:
                return "silent", stats
            time.sleep(0.002)
        return "timeout", stats

    def _record_resets(self, resets) -> None:
        """收集 RESET 行的墙钟时间线（报告/事后定性用）。"""
        wall = time.monotonic()
        for text in resets:
            self.reset_lines.append((wall, text))


__all__ = [
    "BOUNDARY_DRIFT_TOL_PX",
    "BoundaryMarkersMoved",
    "Harness",
    "HarnessError",
    "ODOM_C_1REV",
    "ODOM_LXY_OVER_R",
    "ODOM_R_EFF_MM",
    "ODOM_SIGN",
    "SIGN_MIN_NORM_MM",
    "StopFailed",
    "TRIAL_REASONS",
    "body_command",
    "body_delta_to_field",
    "translation_sign_ok",
]
