"""`vgt.harness` 的行为锁：`locate()` 单帧真值 + 边界码漂移复核（R18）+ 闭环归位。

## 这些测试怎么做到不需要车、也不需要网络

- `FakeSock` 是**按命令前缀取回复**的脚本化 socket：发 `STOP` 就吐 STOP 的脚本、
  发 `RUN` 就吐 RUN 的脚本。发送与回复的因果关系和真机一致 —— `drive_to` 开场
  drain 时不会"提前"读到后面命令才该出现的字节。
- `make_car_link()` 造一个**真 CarLink 壳**：`CarLink.__new__` 跳过 `__init__`
  （不建 TCP），只补上 `sock/addr/buf` —— 发送路径走的是仓库里真 `CarLink.cmd`。
- 相机是假对象（只需要 `grab()`）；`detect_markers` 用 monkeypatch 换成脚本化位姿，
  但 `build_field_frame` / `body_pose_in_field` 走**真**管线：合成场景把 4 个边界码
  摆在相机系 z=500 的平面上，场地系与车体位姿是真算出来的。
- 所有时序参数（DONE 超时、静默看门狗、回复超时、握手超时）在 `_make()` 里压到
  0.05–1 s，真机默认值（15 s 等）在代码里，不在测试里等。

需要真相机的项（RUN 一下车、边界码被撞）在测试里没有、也不该有 —— 见
Task 10 报告的「待真机验证清单」。
"""
from __future__ import annotations

import math
import socket
from pathlib import Path

import numpy as np
import pytest

from car_link import CarLink
from vgt import harness as H
from vgt import matrix
from vgt import odometry
from vgt.camera import CameraSession, CameraUnavailable
from vision_core import load_calib
from vision_core.markers import MarkerPose

# 合成标定：K/dist 不会被用到（detect_markers 被换掉），bias 全零让期望值好算。
CALIB = {
    "K": [[1460.76, 0.0, 1019.44], [0.0, 1460.76, 592.77], [0.0, 0.0, 1.0]],
    "dist": [0.0, 0.0, 0.0, 0.0, 0.0],
    "boundary_side_mm": 25.0,
    "vehicle_side_mm": 30.0,
    "bias": {"dx_mm": 0.0, "dy_mm": 0.0, "dtheta_deg": 0.0},
}

# 4 个边界码在场地系里的角点位置（R4：ID20=TL、21=TR、22=BR、23=BL）。
BOUNDARY_XY = {
    20: (-252.5, -125.9),
    21: (252.5, -125.9),
    22: (252.5, 125.9),
    23: (-252.5, 125.9),
}

# 正对相机的码：rvec=[π,0,0] ⇒ 码上轴指向图像上方 ⇒ 车体 yaw=+90°（field 口径）。
R_FACING = (math.pi, 0.0, 0.0)
# 绕 z 转 −90° 的 rvec ⇒ 码上轴指向图像右方 ⇒ 车体 yaw=0°。
R_YAW0 = (0.0, 0.0, -math.pi / 2.0)


def _run_line(exp) -> str:
    """这条实验会下发的 RUN 行（与 `Harness.run_trial` 的格式化逐字一致）。

    R68：矩阵的 `duration_s` 是按实测速度模型反解的，别再往断言里硬编码 "1000"。
    """
    return "RUN {:.4f} {:.4f} {:.4f} {}\n".format(
        float(exp.vx), float(exp.vy), float(exp.om),
        int(round(float(exp.duration_s) * 1000.0)))


class FakeSock:
    """按命令前缀取回复的脚本化 socket。

    `replies = {"STOP": [[b"OK STOP\\n"], ...], "RUN": [[...], ...]}`：每条命令
    发出后，从自己的队列里 pop 一组字节进 `pending`（一组里可以是多行、也可以
    是粘连行）。队列空/没配 → 什么都不回：`recv` 抛 `socket.timeout`
    （= "链路上这一刻没有新字节"）。
    """

    def __init__(self, replies=None):
        self.replies = {k: [list(group) for group in v] for k, v in (replies or {}).items()}
        self.pending: list[bytes] = []
        self.sent: list[str] = []
        self.timeout = None
        self.closed = False

    @staticmethod
    def _key(line: str) -> str:
        parts = line.split()
        return parts[0] if parts else line

    def sendall(self, data: bytes) -> None:
        if self.closed:
            raise OSError("sendall on closed socket")
        text = data.decode()
        self.sent.append(text)
        queue = self.replies.get(self._key(text.strip()))
        if queue:
            self.pending.extend(queue.pop(0))

    def recv(self, n: int) -> bytes:
        if self.pending:
            return self.pending.pop(0)
        raise socket.timeout("fake：没有新字节")

    def settimeout(self, t) -> None:
        self.timeout = t

    def close(self) -> None:
        self.closed = True


def make_car_link(sock=None, addr=("127.0.0.1", 8266)) -> CarLink:
    """真 CarLink 壳：跳过 __init__（不联网），只补 `sock/buf/addr` 三个字段。

    `cmd(wait=False)` 与 `cmd(wait=True)` 都是仓库里真 CarLink 的实现
    （harness 只用 wait=False 的发送路径，读取自己来 —— 为了 RESET-aware）。
    """
    link = CarLink.__new__(CarLink)
    link.addr = addr
    link.sock = sock if sock is not None else FakeSock()
    link.buf = b""
    return link


class FakeCamera:
    """假相机：只需要 `grab()`。记录每次调用的有效 `fresh` 值（默认应为 True）。"""

    def __init__(self, frames=()):
        self.frames = list(frames)
        self.calls: list[bool] = []

    def grab(self, fresh=True):
        self.calls.append(bool(fresh))
        if self.frames:
            return self.frames.pop(0)
        return b"frame"


def mk_pose(mid, tvec, px, rvec=R_FACING, rms=0.3, half=8.0) -> MarkerPose:
    """合成一个 MarkerPose；`px` 是 4 个角点的均值（漂移复核用的就是它）。

    `tvec` 单位 mm（相机系）。角点绕 `px` 摆一个小方框，`corners.mean(axis=0)`
    精确等于 `px` —— 漂移复核读 `FieldFrame.marker_px` 时看到的就是这个数。
    """
    x, y = float(px[0]), float(px[1])
    corners = np.array(
        [[x - half, y - half], [x + half, y - half],
         [x + half, y + half], [x - half, y + half]], dtype=np.float64)
    return MarkerPose(
        marker_id=int(mid),
        rvec=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        tvec=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        reproj_rms_px=float(rms),
        corners_px=corners,
    )


def scene(vehicle_xy=None, yaw_rvec=R_FACING, px_shift=(0.0, 0.0), drop=()) -> dict:
    """合成一帧的 poses：4 个边界码摆在场地角点、相机正俯视、场地系 = 相机系。

    相机正下方 z=500 的平面上，场地坐标 (x, y) 就是相机坐标 (x, y, 500)；
    质心 (0,0) 即场地原点，`build_field_frame` 会解出 R=单位阵。像素位置任意
    但唯一（能分清码就行）；`px_shift` 整体平移像素（模拟"码在画面里动了"）。
    """
    poses = {}
    for mid, (fx, fy) in BOUNDARY_XY.items():
        if mid in drop:
            continue
        px = (300.0 + fx * 0.5 + px_shift[0], 300.0 + fy * 0.5 + px_shift[1])
        poses[mid] = mk_pose(mid, (fx, fy, 500.0), px)
    if vehicle_xy is not None:
        poses[29] = mk_pose(29, (vehicle_xy[0], vehicle_xy[1], 490.0),
                            px=(900.0, 700.0), rvec=yaw_rvec)
    return poses


def _make(poses_list=(), replies=None, link_factory=None, frames=(), camera=None,
          coast_enc=None, **kw):
    """造一套假设备：返回 `(harness, camera, detect, sock)`。

    `detect` 是脚本化的 `detect_markers` 替身（逐帧 pop `poses_list`，用完给 {}）；
    由每个测试自己 `monkeypatch.setattr(H, "detect_markers", detect)`。
    时序参数压小，让所有等待路径都在毫秒级跑完。

    `camera`：要注入特殊相机（比如抓帧会抛的）时传进来，默认 `FakeCamera(frames)`。
    `coast_enc`：R66 停车后那次空闲编码器读数（`STREAM 1` 的 `I` 行）的值。缺省
    `None` = **不配**这个回复 ⇒ `_read_idle_enc` 读不到 ⇒ 记一条 `coast_read_failed`
    warning、里程计退回只积 `D` 行。想验"滑行被并进来"的测试必须显式给值。
    """
    timeouts = dict(done_timeout_s=1.0, silence_timeout_s=0.05,
                    reply_timeout_s=0.05, handshake_timeout_s=0.5,
                    locate_tries=1)
    timeouts.update(kw)
    replies = dict(replies or {})
    if coast_enc is not None:
        replies.setdefault("STREAM", _coast_replies(coast_enc))
    sock = FakeSock(replies)
    camera = camera if camera is not None else FakeCamera(frames)
    h = H.Harness(make_car_link(sock), camera, dict(CALIB),
                  link_factory=link_factory, **timeouts)
    script = list(poses_list)

    def detect(frame, calib):
        return script.pop(0) if script else {}

    return h, camera, detect, sock


def _run_chunks(ms=400):
    """一次正常 RUN 的脚本回复：OK RUN + 一条 D 样本 + DONE。"""
    return [f"OK RUN pwm=-110,110,-110,110 ms={ms} kick=150ms/105\n".encode(),
            b"D 0 -110 110 -110 110 0 0 0 0 0.00 0.00 90.00\n",
            b"DONE\n"]


# ------------------------------------------------------------------ 构造函数与 home_xy


def test_home_xy_sentinel_separates_default_from_explicit_zero():
    """复核 Important 1：默认 None = 松散口径；显式 (0,0) 要能被下游分辨。"""
    h = H.Harness(make_car_link(), FakeCamera(), dict(CALIB))
    assert h.home_xy == (0.0, 0.0)
    assert h.home_xy_explicit is False

    h2 = H.Harness(make_car_link(), FakeCamera(), dict(CALIB), home_xy=(0.0, 0.0))
    assert h2.home_xy == (0.0, 0.0)
    assert h2.home_xy_explicit is True      # 同一个坐标，两种口径不许混淆

    h3 = H.Harness(make_car_link(), FakeCamera(), dict(CALIB), home_xy=(-80.0, 10.0))
    assert h3.home_xy == (-80.0, 10.0)
    assert h3.home_xy_explicit is True

    with pytest.raises(ValueError):
        H.Harness(make_car_link(), FakeCamera(), dict(CALIB), home_xy=(1.0,))
    with pytest.raises(ValueError):
        H.Harness(make_car_link(), FakeCamera(), dict(CALIB), home_xy=(float("nan"), 0.0))


def test_construction_does_no_io():
    """构造不发命令、不开相机（与 CameraSession 的"导入无副作用"同口径）。"""
    sock = FakeSock({})
    cam = FakeCamera()
    H.Harness(make_car_link(sock), cam, dict(CALIB))
    assert sock.sent == []
    assert cam.calls == []


# -------------------------------------------------------------------------- locate()


def test_locate_returns_vehicle_pose_in_the_field(monkeypatch):
    h, cam, detect, _ = _make(poses_list=[scene(vehicle_xy=(100.0, -30.0))])
    monkeypatch.setattr(H, "detect_markers", detect)

    pos = h.locate()
    assert pos == pytest.approx((100.0, -30.0, 90.0))
    assert h.last_locate_reason is None
    assert cam.calls == [True]      # 单帧抓取，且是 fresh=True 的默认路径（R54）


def test_locate_uses_the_calibration_bias(monkeypatch):
    """bias 在**码系**里减（body_pose_in_field 的契约），本测试只是接线检查。"""
    calib = dict(CALIB)
    calib["bias"] = {"dx_mm": 10.0, "dy_mm": 0.0, "dtheta_deg": 0.0}
    sock = FakeSock({})
    h = H.Harness(make_car_link(sock), FakeCamera(), calib)
    poses = scene(vehicle_xy=(100.0, 0.0), yaw_rvec=R_YAW0)
    monkeypatch.setattr(H, "detect_markers", lambda frame, c: poses)

    x, y, yaw = h.locate()
    # dx 沿**码的规范 +X** 减：R_YAW0 时码规范 +X 指向场地 −Y ⇒ 位置 +10 mm 在 y。
    assert (x, y) == pytest.approx((100.0, 10.0))
    assert yaw == pytest.approx(0.0)


def test_locate_returns_none_when_a_boundary_marker_is_missing(monkeypatch):
    h, _, detect, _ = _make(poses_list=[scene(vehicle_xy=(0.0, 0.0), drop=(22,))])
    monkeypatch.setattr(H, "detect_markers", detect)

    assert h.locate() is None
    assert h.last_locate_reason == "boundary_moved"


def test_locate_returns_none_when_vehicle_marker_is_missing(monkeypatch):
    h, _, detect, _ = _make(poses_list=[scene(vehicle_xy=None)])
    monkeypatch.setattr(H, "detect_markers", detect)

    assert h.locate() is None
    assert h.last_locate_reason == "no_vehicle_marker"


def test_locate_clears_last_reason_on_success(monkeypatch):
    h, _, detect, _ = _make(poses_list=[scene(vehicle_xy=None), scene(vehicle_xy=(1.0, 2.0))])
    monkeypatch.setattr(H, "detect_markers", detect)

    assert h.locate() is None and h.last_locate_reason == "no_vehicle_marker"
    assert h.locate() is not None and h.last_locate_reason is None


# ---------------------------------------------------- 边界码漂移复核（R18，硬要求）


def test_boundary_baseline_is_captured_once_and_16px_raises(monkeypatch):
    """基线只在第一次成功 locate() 抓；10 px 过、恰好 15 px 过、16 px 抛。"""
    h, _, detect, _ = _make(poses_list=[
        scene(vehicle_xy=(0.0, 0.0)),                        # 抓基线
        scene(vehicle_xy=(0.0, 0.0), px_shift=(10.0, 0.0)),  # +10 px：过
        scene(vehicle_xy=(0.0, 0.0), px_shift=(0.0, 15.0)),  # 恰好 15 px：过（> 才抛）
        scene(vehicle_xy=(0.0, 0.0), px_shift=(16.0, 0.0)),  # +16 px：抛
    ])
    monkeypatch.setattr(H, "detect_markers", detect)

    assert h.locate() is not None
    assert h.locate() is not None
    assert h.locate() is not None
    with pytest.raises(H.BoundaryMarkersMoved, match="ID20"):
        h.locate()
    # 抛完基线还在原地：再来一帧 +16 px 依旧抛。
    assert h._boundary_px_baseline[20][0] == pytest.approx(300.0 - 252.5 * 0.5)
    monkeypatch.setattr(H, "detect_markers",
                        lambda frame, c: scene(vehicle_xy=(0.0, 0.0), px_shift=(16.0, 0.0)))
    with pytest.raises(H.BoundaryMarkersMoved):
        h.locate()


def test_drift_check_compares_against_the_first_baseline_not_the_last_frame(monkeypatch):
    """+10 过、+15 过之后 +16 仍然抛 —— 说明基线没被中途帧刷掉。"""
    h, _, detect, _ = _make(poses_list=[
        scene(vehicle_xy=(0.0, 0.0)),
        scene(vehicle_xy=(0.0, 0.0), px_shift=(10.0, 0.0)),
        scene(vehicle_xy=(0.0, 0.0), px_shift=(15.0, 0.0)),
        scene(vehicle_xy=(0.0, 0.0), px_shift=(16.0, 0.0)),
    ])
    monkeypatch.setattr(H, "detect_markers", detect)
    assert h.locate() is not None
    assert h.locate() is not None
    assert h.locate() is not None
    with pytest.raises(H.BoundaryMarkersMoved):
        h.locate()


def test_baseline_is_not_captured_until_the_first_full_success(monkeypatch):
    """无 ID29 的帧（locate 返回 None）不许抓基线；基线抓在真正成功那一帧。"""
    h, _, detect, _ = _make(poses_list=[
        scene(vehicle_xy=None),                                    # 只有边界码
        scene(vehicle_xy=(0.0, 0.0), px_shift=(5.0, 0.0)),         # 第一次真成功
        scene(vehicle_xy=(0.0, 0.0), px_shift=(5.0 + 16.0, 0.0)),  # 相对成功帧 +16
    ])
    monkeypatch.setattr(H, "detect_markers", detect)

    assert h.locate() is None
    assert h._boundary_px_baseline is None
    assert h.locate() is not None
    assert h._boundary_px_baseline[20][0] == pytest.approx(300.0 - 252.5 * 0.5 + 5.0)
    with pytest.raises(H.BoundaryMarkersMoved):
        h.locate()


def test_failed_locate_does_not_reset_or_update_the_baseline(monkeypatch):
    h, _, detect, _ = _make(poses_list=[
        scene(vehicle_xy=(0.0, 0.0)),
        scene(vehicle_xy=None, drop=(20, 21, 22, 23)),   # 全缺：返回 None
        scene(vehicle_xy=(0.0, 0.0), px_shift=(16.0, 0.0)),
    ])
    monkeypatch.setattr(H, "detect_markers", detect)
    assert h.locate() is not None
    assert h.locate() is None                        # 失败一帧
    with pytest.raises(H.BoundaryMarkersMoved):      # 基线还在（既没清掉也没重抓）
        h.locate()


def test_drift_check_fires_even_when_vehicle_marker_is_missing(monkeypatch):
    """4 个边界码都在但挪了 16 px、ID29 又没看到 → 漂移优先，抛。"""
    h, _, detect, _ = _make(poses_list=[
        scene(vehicle_xy=(0.0, 0.0)),
        scene(vehicle_xy=None, px_shift=(0.0, 16.0)),
    ])
    monkeypatch.setattr(H, "detect_markers", detect)
    assert h.locate() is not None
    with pytest.raises(H.BoundaryMarkersMoved):
        h.locate()


# ------------------------------------------------------------------- body_command()


@pytest.mark.parametrize("yaw_deg, err, expect_dir", [
    (0.0, (100.0, 0.0), (1.0, 0.0)),      # 车头朝 +X：目标在前 → 正 vx
    (0.0, (0.0, 100.0), (0.0, -1.0)),     # 目标在 +Y（图像下）= 车右侧 → vy<0
    (0.0, (-100.0, 0.0), (-1.0, 0.0)),    # 目标在正后
    (0.0, (0.0, -100.0), (0.0, 1.0)),     # 目标在图像上 = 车左侧 → vy>0
    (90.0, (0.0, -100.0), (1.0, 0.0)),    # 车头朝 −Y（图像上）：目标在前
    (90.0, (-100.0, 0.0), (0.0, 1.0)),    # 车头朝图像上：车左 = −X → vy>0
    (180.0, (100.0, 0.0), (-1.0, 0.0)),   # 车头朝 −X：目标 +X 在正后
    (-90.0, (100.0, 0.0), (0.0, 1.0)),    # 车头朝 +Y（图像下）：车左 = +X
])
def test_body_command_direction_convention(yaw_deg, err, expect_dir):
    vx, vy = H.body_command(err[0], err[1], yaw_deg)
    speed = math.hypot(vx, vy)
    assert speed == pytest.approx(0.1)            # kp·100 mm = 0.1 m/s，未触上下限
    assert (vx / speed, vy / speed) == pytest.approx(expect_dir, abs=1e-9)


def test_body_command_clamps_speed_and_rejects_bad_input():
    vx, vy = H.body_command(1000.0, 0.0, 0.0)
    assert math.hypot(vx, vy) == pytest.approx(H.MAX_CMD_MPS)
    vx, vy = H.body_command(1.0, 0.0, 0.0)
    assert math.hypot(vx, vy) == pytest.approx(H.MIN_CMD_MPS)

    with pytest.raises(ValueError):
        H.body_command(0.0, 0.0, 0.0)                     # 零误差方向无定义
    with pytest.raises(ValueError):
        H.body_command(10.0, 0.0, 0.0, v_max=0.01, v_min=0.02)


# ---------------------------------------------------------------- drive_to() 闭环


def test_drive_to_converges_and_commands_the_right_directions(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [_run_chunks(), _run_chunks()]})
    positions = [(90.0, 0.0, 0.0), (40.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)

    assert res["reached"] is True and res["reason"] == "converged"
    assert res["iters"] == 2 and res["final"] == (5.0, 0.0, 0.0)
    assert res["stop_confirmed"] is True
    assert len(res["trace"]) == 2
    assert res["trace"][0]["dist_mm"] == pytest.approx(90.0)
    assert sock.sent == [
        "STOP\n",                                     # 开场红灯
        "RUN -0.0900 0.0000 0 400\n",                 # 误差 −90 mm → 朝 −X 走
        "RUN -0.0400 0.0000 0 400\n",
        "STOP\n",                                     # 收尾
    ]


def test_drive_to_at_target_needs_no_pulse(monkeypatch):
    """起点已在容差内 ⇒ 一条 RUN 都不发（这是省动作，不是成功）。"""
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]]})
    h.locate = lambda: (3.0, 2.0, 0.0)          # 距目标 3.6 mm ≤ 12 mm

    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert res["iters"] == 0
    assert res["final"] == (3.0, 2.0, 0.0)
    assert [s for s in sock.sent if s.startswith("RUN")] == []


def test_drive_to_zero_pulse_is_not_a_success(monkeypatch):
    """R67：`iters=0` 的"归位成功"是**静默假成功**，不许再发出去。

    真机现场：`drive_to(0,0)` 连跑三次全 `reached=True, iters=0`，车根本没动。
    Task 12 只读 `reached`，于是把"没动过"记成"归位成功"。契约钉死：

    - `reached=True` **必须**蕴含至少真的执行过一次动作（`moved`/`iters ≥ 1`）；
    - 零迭代收敛改判 `reached=False` + `reason="already_at_target"`，
      同时用 `at_target=True` 告诉调用方"车确实已经在容差内"（可以照常往下走，
      但归位这件事本身没有执行过任何动作，必须能分辨）。
    """
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]]})
    h.locate = lambda: (3.0, 2.0, 0.0)          # 距目标 3.6 mm ≤ 12 mm

    res = h.drive_to(0.0, 0.0, settle_s=0.0)

    assert res["reached"] is False, "零次迭代不能算归位成功（R67）"
    assert res["reason"] == "already_at_target"
    assert res["reason"] in H.DRIVE_REASONS
    assert res["at_target"] is True             # 车确实在容差内（≠ 闭环成功）
    assert res["moved"] is False and res["iters"] == 0
    assert res["start_dist_mm"] == pytest.approx(math.hypot(3.0, 2.0))
    assert [s for s in sock.sent if s.startswith("RUN")] == []


def test_drive_to_reached_implies_at_least_one_executed_pulse(monkeypatch):
    """反过来也钉住：真收敛（有脉冲）仍然是成功，`reached` 不许被误伤。"""
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [_run_chunks()]})
    positions = [(90.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)

    assert res["reached"] is True and res["reason"] == "converged"
    assert res["moved"] is True and res["iters"] == 1
    assert res["at_target"] is True
    assert res["start_dist_mm"] == pytest.approx(90.0)
    assert [s for s in sock.sent if s.startswith("RUN")] == ["RUN -0.0900 0.0000 0 400\n"]


def test_drive_to_never_executed_is_reachable_from_every_reason(monkeypatch):
    """零迭代的三种收工原因都不许报 `reached=True`（防止以后又改回单一布尔值）。"""
    # ① 第一次 locate 就失败
    h, _, _, _ = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]]}, locate_tries=1)
    h.locate = lambda: None
    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert (res["reached"], res["moved"], res["at_target"]) == (False, False, False)
    assert res["reason"] == "locate_failed" and res["start_dist_mm"] is None

    # ② RUN 被固件拒（没在跑）
    h2, _, _, _ = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                "RUN": [[b"ERR RUN busy\n"]]})
    h2.locate = lambda: (80.0, 0.0, 0.0)
    res2 = h2.drive_to(0.0, 0.0, settle_s=0.0)
    assert res2["reason"] == "run_rejected" and res2["reached"] is False
    assert res2["moved"] is True               # 脉冲发过（被拒），但位置没到
    assert res2["at_target"] is False


def test_drive_to_stops_at_max_iter(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [_run_chunks(), _run_chunks(), _run_chunks()]})
    h.locate = lambda: (100.0, 0.0, 0.0)        # 永远不动

    res = h.drive_to(0.0, 0.0, max_iter=3, settle_s=0.0)
    assert res["reached"] is False and res["reason"] == "max_iter"
    assert res["iters"] == 3
    assert len([s for s in sock.sent if s.startswith("RUN")]) == 3


def test_drive_to_reports_locate_failure_without_driving(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]]}, locate_tries=1)
    h.locate = lambda: None

    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert res["reached"] is False and res["reason"] == "locate_failed"
    assert res["iters"] == 0 and res["final"] is None
    assert [s for s in sock.sent if s.startswith("RUN")] == []
    assert sock.sent == ["STOP\n", "STOP\n"]      # 开场红灯 + 收尾


def test_drive_to_retries_transient_locate_failures(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [_run_chunks()]}, locate_tries=3)
    positions = [None, (80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert res["reached"] is True and res["iters"] == 1


def test_drive_to_propagates_boundary_markers_moved(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [_run_chunks()]})
    positions = [(90.0, 0.0, 0.0)]

    def locate():
        if positions:
            return positions.pop(0)
        raise H.BoundaryMarkersMoved("测试：边界码被撞")

    h.locate = locate
    with pytest.raises(H.BoundaryMarkersMoved):
        h.drive_to(0.0, 0.0, settle_s=0.0)      # 脉冲后的下一次 locate 抛 → 立即中止


def test_drive_to_rejects_targets_outside_the_field():
    h, _, _, sock = _make()
    with pytest.raises(ValueError, match="越出"):
        h.drive_to(200.0, 0.0)                  # |200|+135 > 252.5
    with pytest.raises(ValueError, match="越出"):
        h.drive_to(0.0, 80.0)                   # |80|+74.5 > 125.9
    assert sock.sent == []                      # 连开场 STOP 都不发


def test_drive_to_accepts_the_exact_field_limit():
    h, _, _, _ = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]]})
    h.locate = lambda: (117.5, 0.0, 0.0)        # |x| 恰好 = 252.5 − 135
    res = h.drive_to(117.5, 0.0, settle_s=0.0)
    assert res["at_target"] is True            # 目标本身合法（零脉冲，R67 语义见上）


def test_drive_to_validates_scalar_arguments():
    h, _, _, _ = _make()
    with pytest.raises(ValueError):
        h.drive_to(0.0, 0.0, tol_mm=0.0)
    with pytest.raises(ValueError):
        h.drive_to(0.0, 0.0, max_iter=0)
    with pytest.raises(ValueError):
        h.drive_to(0.0, 0.0, pulse_ms=0.4)
    with pytest.raises(ValueError):
        h.drive_to(0.0, 0.0, v_max_mps=0.01)      # 低于 P 律下限


def test_drive_to_pulse_ms_is_configurable():
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [_run_chunks(ms=150)]})
    positions = [(80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, pulse_ms=150, settle_s=0.0)
    assert res["reached"] is True
    assert "RUN -0.0800 0.0000 0 150\n" in sock.sent


# ------------------------------------------------- 归位脉冲模型与两段式步长（R70）

def test_expected_step_mm_matches_every_measured_point():
    """R70 的步长模型：R68 三条直行腿 + Task 10 归位 + R70 归位扫描都要对得上。

    模型 = `max(kick 地板, 0.5·(T−120))`：
    - `T ≥ 150 ms` 用 `0.5·(T−120)`（对 300/350/400/600 ms 的实测偏差 ≤ 7%；
      固件的角度是 `stopAll()` 自由滑行 + 起步 kick，与 R68 行程模型 0.56·(T−143)
      是同一个物理过程的两种拟合，R70 用归位实测重新拟合成一条**永不高估**的曲线）；
    - `T < 150 ms` 落在固件 `kick=150ms/105` 的地板上（实测 120 ms → 7.4 mm、
      130 ms → ~9 mm）：**模型取 max ⇒ 拿地板值偏保守，归位可达距离不会被高估**。
    """
    for ms, measured in ((300, 88.6), (350, 110.3), (600, 256.3)):
        assert H.expected_step_mm(ms) == pytest.approx(measured, rel=0.10), ms
    assert 131.8 <= H.expected_step_mm(400) <= 148.2            # Task 10 实测区间
    assert 13.0 <= H.expected_step_mm(150) <= 18.0              # Task 10 / R70 实测
    assert H.expected_step_mm(120) == pytest.approx(7.4, rel=0.2)
    assert H.expected_step_mm(130) <= 9.5                       # 永不高估
    assert H.expected_step_mm(130) < H.expected_step_mm(150) < H.expected_step_mm(240)
    # 离算结论：任何误差、任何轴向，四轮 duty 都是 110 ⇒ 步长**与误差无关**，
    # 只由 pulse 决定（唯一例外是 45° 混合指令让两轮落进 0.3 rad/s 死区）。
    assert H.expected_step_mm(40 if False else 120) == pytest.approx(7.4, rel=0.2)


def test_homing_reach_is_max_iter_times_the_step():
    """`tol + max_iter × 单步` = 能吃掉的距离上限（R70 硬结论：120 ms 只有 ~101 mm）。

    真机证据：`pulse_ms=120` 从 **118.5 mm** 起跑，12 次迭代只走到 37.5 mm ——
    起点本来就在可达集之外，"设计上就不可能收敛"，与 120 还是 140 无关。
    """
    assert H.homing_reach_mm(120, max_iter=12) == pytest.approx(12 + 12 * 7.4, rel=0.05)
    assert H.homing_reach_mm(150, max_iter=12) == pytest.approx(12 + 12 * 15.0, rel=0.05)
    assert H.homing_reach_mm(400, max_iter=12) == pytest.approx(12 + 12 * 140.0, rel=0.05)
    # 两段式（粗 240 → 步长 60）把可达距离拉长：12 次够覆盖 12 + 12×60
    assert H.homing_reach_mm(130, max_iter=12, coarse_pulse_ms=240) == pytest.approx(
        12 + 12 * 60.0, rel=0.05)
    with pytest.raises(ValueError):
        H.homing_reach_mm(150, max_iter=0)


def test_drive_to_fails_fast_when_the_start_is_out_of_reach(monkeypatch):
    """R70：起点误差超过 `max_iter × 单步` 就**当场失败并命名**，不许烧 12 个脉冲。

    真机事故链：今天的车就是被 12 次"小步前冲"顶进 +X/−Y 角、压住 ID21 死锁的。
    """
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                    "RUN": [_run_chunks()] * 20},
                           locate_tries=1)
    h.locate = lambda: (120.0, 0.0, 0.0)            # 120 mm > 12 × 7.5 = 90 mm

    res = h.drive_to(0.0, 0.0, pulse_ms=130, settle_s=0.0)

    assert res["reached"] is False and res["reason"] == "homing_too_far"
    assert res["reason"] in H.DRIVE_REASONS
    assert res["iters"] == 0 and res["trace"] == []
    assert [s for s in sock.sent if s.startswith("RUN")] == []      # 一条 RUN 都没发


def test_drive_to_two_stage_pulse_far_is_coarse_then_fine(monkeypatch):
    """两段式：远用粗脉冲、近用细脉冲；粗步 overshoot 之后必须切回细脉冲。"""
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                    "RUN": [_run_chunks()] * 20})
    # 起点 120 mm；粗脉冲（240 ms，步长 ~60 mm）打 2 下 → 误差 120→60→30 → 切细
    positions = [(120.0, 0.0, 0.0), (60.0, 0.0, 0.0), (30.0, 0.0, 0.0), (6.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, pulse_ms=130, coarse_pulse_ms=240,
                     coarse_switch_mm=45.0, settle_s=0.0)

    assert res["reached"] is True and res["reason"] == "converged"
    assert res["iters"] == 3
    runs = [s for s in sock.sent if s.startswith("RUN")]
    assert runs == ["RUN -0.1200 0.0000 0 240\n",      # 误差 120 > 45 ⇒ 粗
                    "RUN -0.0600 0.0000 0 240\n",      # 误差  60 > 45 ⇒ 粗
                    "RUN -0.0300 0.0000 0 130\n"]      # 误差  30 ≤ 45 ⇒ 细
    # 粗段的过冲上限 = 粗步长 − switch（60 − 45 = 15 mm），细步长 7.4 < tol 12 ⇒ 必收敛


def test_drive_to_without_coarse_keeps_the_single_stage_behaviour(monkeypatch):
    """默认（不传 `coarse_pulse_ms`）= 每次都用 `pulse_ms`，一个字都不改。"""
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                    "RUN": [_run_chunks()] * 20})
    positions = [(80.0, 0.0, 0.0), (50.0, 0.0, 0.0), (6.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, pulse_ms=130, settle_s=0.0)
    runs = [s for s in sock.sent if s.startswith("RUN")]
    assert res["reached"] is True and res["iters"] == 2
    assert runs == ["RUN -0.0800 0.0000 0 130\n", "RUN -0.0500 0.0000 0 130\n"]


def test_single_stage_130ms_cannot_reach_the_sweep_start_distance(monkeypatch):
    """R70 真机对照：120/130 ms 单段从 118.5 mm 起跑是**设计上不可能**收敛。

    这正是 `pulse_sweep.json` 里那条 `118.5 → 37.5` 的场景（12 次迭代白烧、车被
    一步步推到场地边上）。现在它在起跑前就判 `homing_too_far`。
    """
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                    "RUN": [_run_chunks()] * 20}, locate_tries=1)
    h.locate = lambda: (118.5, 0.0, 0.0)
    res = h.drive_to(0.0, 0.0, pulse_ms=130, settle_s=0.0)
    assert res["reason"] == "homing_too_far" and res["iters"] == 0
    assert [s for s in sock.sent if s.startswith("RUN")] == []

    # 同一距离、两段式（粗 240 ⇒ 可达 ~732 mm）就能收敛
    h2, _, _, sock2 = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                     "RUN": [_run_chunks()] * 20}, locate_tries=1)
    seq = [(118.5, 0.0, 0.0), (58.5, 0.0, 0.0), (30.0, 0.0, 0.0), (6.0, 0.0, 0.0)]
    h2.locate = lambda: seq.pop(0)
    res2 = h2.drive_to(0.0, 0.0, pulse_ms=130, coarse_pulse_ms=240, settle_s=0.0)
    assert res2["reached"] is True and res2["reason"] == "converged"


def test_drive_to_validates_the_two_stage_arguments():
    h, _, _, sock = _make()
    with pytest.raises(ValueError):
        h.drive_to(0.0, 0.0, coarse_pulse_ms=400.0, coarse_switch_mm=-1.0)
    with pytest.raises(ValueError):
        h.drive_to(0.0, 0.0, coarse_pulse_ms=0.4, coarse_switch_mm=45.0)
    assert sock.sent == []                      # 参数非法：连开场 STOP 都不发


def test_locate_retries_wait_long_enough_for_the_camera(monkeypatch):
    """R70：`locate` 重试之间必须真的等一会儿（0.05 s 只够 1.5 帧，掉码往往要 2–3 帧）。

    `grab(fresh=True)` 每次都丢队列，自动对焦/曝光要几帧才稳；重试间隔低于 ~0.2 s
    等于连拍三张一样的坏帧。
    """
    slept = []
    real_sleep = H.time.sleep
    monkeypatch.setattr(H.time, "sleep", lambda s: (slept.append(float(s)), real_sleep(0))[1])
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                    "RUN": [_run_chunks()]}, locate_tries=3)
    seq = [None, None, (80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: seq.pop(0)

    res = h.drive_to(0.0, 0.0, pulse_ms=130, settle_s=0.0)
    pauses = [s for s in slept if 0.0 < s < 2.0]
    assert res["reached"] is True
    assert len(pauses) >= 2 and sum(pauses) >= 0.4, f"重试间隔太短：{pauses}"


# ------------------------------------------------- 偏航闭环（R71）

def test_expected_yaw_step_matches_the_measured_points():
    """R71 转角模型：152→7.7°、199→15.8°、211→16~17°、266→22.7°、278→29.7°。

    散点 ±25%（偏航率随地板/摩擦变化），模型偏差 ≤ 14% 且**略微高估**步长 ——
    宁可脉冲短一点，也别冲过头。
    """
    for ms, measured in ((152, 7.7), (199, 15.8), (211, 16.5), (266, 22.7),
                         (278, 29.7)):
        assert H.expected_yaw_step_deg(ms) == pytest.approx(measured, rel=0.20), ms
    assert H.expected_yaw_step_deg(150) == pytest.approx(7.5, abs=1.5)   # kick 地板
    # 反解：越大要转越久，但地板之下给 150 ms、上限 320 ms
    assert H.pulse_ms_for_yaw_deg(0.0) == 150
    assert H.pulse_ms_for_yaw_deg(5.0) == 150
    assert H.pulse_ms_for_yaw_deg(16.0) == pytest.approx(203, abs=8)
    assert H.pulse_ms_for_yaw_deg(180.0) == 320


def test_align_yaw_converges_and_only_sends_timed_runs(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"], [b"OK STOP\n"]],
                                    "RUN": [_run_chunks(), _run_chunks()]},
                           locate_tries=1)
    positions = [(0.0, 0.0, 20.0), (0.0, 0.0, 5.0)]     # 20° → 5°（容差 8° 内）
    h.locate = lambda: positions.pop(0)

    res = h.align_yaw(0.0, tol_deg=8.0, settle_s=0.0)

    assert res["converged"] is True and res["reason"] == "converged"
    assert res["steps"] == 1 and res["final_yaw"] == pytest.approx(5.0)
    runs = [s for s in sock.sent if s.startswith("RUN")]
    assert len(runs) == 1 and runs[0].split()[-1].isdigit()      # 末位是 ms（有超时）
    assert "STOP" in sock.sent[-1]                               # 收尾 STOP


def test_align_yaw_sign_convention_and_step_directions(monkeypatch):
    """yaw > 目标 ⇒ 用**负** om 把它转回来（`om>0` = 逆时针 = yaw 增大）。"""
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"]] * 6,
                                    "RUN": [_run_chunks(), _run_chunks()]},
                           locate_tries=1)
    positions = [(0.0, 0.0, -20.0), (0.0, 0.0, -5.0)]    # −20° → 需要 +20°（逆时针）
    h.locate = lambda: positions.pop(0)
    res = h.align_yaw(0.0, tol_deg=8.0, settle_s=0.0)
    assert res["converged"] is True
    om = res["trace"][0]["cmd"][2]
    assert om > 0.0


def test_align_yaw_reports_locate_failure_without_rotating(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"]] * 4}, locate_tries=1)
    h.locate = lambda: None

    res = h.align_yaw(0.0, tol_deg=8.0, settle_s=0.0)
    assert res["converged"] is False and res["reason"] == "locate_failed"
    assert res["steps"] == 0 and res["trace"] == []
    assert [s for s in sock.sent if s.startswith("RUN")] == []


def test_align_yaw_gives_up_with_a_named_reason(monkeypatch):
    """步数用满 ⇒ `max_steps` + 最后 yaw（不许静默算成功）。"""
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"]] * 8,
                                    "RUN": [_run_chunks()] * 8}, locate_tries=1)
    h.locate = lambda: (0.0, 0.0, 30.0)          # 永远转不过来的假定位

    res = h.align_yaw(0.0, tol_deg=8.0, max_steps=2, settle_s=0.0)
    assert res["converged"] is False and res["reason"] == "max_steps"
    assert res["steps"] == 2 and res["final_yaw"] == pytest.approx(30.0)


def test_align_yaw_validates_arguments():
    h, _, _, sock = _make()
    with pytest.raises(ValueError):
        h.align_yaw(0.0, tol_deg=0.0)
    with pytest.raises(ValueError):
        h.align_yaw(0.0, max_steps=0)
    with pytest.raises(ValueError):
        h.align_yaw(0.0, settle_s=-1.0)
    assert [s for s in sock.sent if s.startswith("RUN")] == []


def test_align_yaw_propagates_boundary_markers_moved(monkeypatch):
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"]] * 4}, locate_tries=1)

    def locate():
        raise H.BoundaryMarkersMoved("测试：边界码被撞")

    h.locate = locate
    with pytest.raises(H.BoundaryMarkersMoved):
        h.align_yaw(0.0, settle_s=0.0)


# ------------------------------------------------- 链路失败：判完、看门狗、失控保护


def test_run_wait_timeout_is_stopped_and_reported(monkeypatch):
    """DONE 丢了（固件环满，R58）：等超时 → 当前链路 STOP 确认 → 继续下一圈。

    链路 STOP 确认了就不重连 —— 链路活着、车已停，缺的 DONE 只是被丢。
    """
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"], [b"OK STOP\n"]],
                                   "RUN": [[]]},
                          done_timeout_s=0.05, silence_timeout_s=10.0)
    positions = [(80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)

    assert res["reached"] is True
    assert res["events"][1] == {"event": "run_wait_failed", "outcome": "timeout",
                                "detail": None, "stopped_on_current_link": True}
    assert not any(ev.get("event") == "link_reset" for ev in res["events"])
    assert sock.sent == ["STOP\n", "RUN -0.0800 0.0000 0 400\n", "STOP\n", "STOP\n"]


def test_silent_run_wait_triggers_watchdog(monkeypatch):
    """RUN 之后一个字节都没有：静默看门狗先于 DONE 超时出手（不用傻等 15 s）。"""
    h, _, _, _ = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"], [b"OK STOP\n"]],
                                "RUN": [[]]},
                       done_timeout_s=10.0, silence_timeout_s=0.05)
    positions = [(80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert res["reached"] is True
    assert res["events"][1]["outcome"] == "silent"


def test_unconfirmed_stop_reconnects_and_continues(monkeypatch):
    """STOP 不确认（链路死）→ 关旧链路 → 重连 → 第一件事 STOP → 继续归位。"""
    calls = []
    sock2 = FakeSock({"PING": [[b"PONG\n"]],
                      "STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                      "RUN": [_run_chunks()]})

    def factory():
        calls.append(1)
        return make_car_link(sock2)

    h, _, _, sock1 = _make(replies={"STOP": [[b"OK STOP\n"]]},   # 只有开场 STOP 有回复
                           link_factory=factory)
    positions = [(80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)

    assert res["reached"] is True and res["stop_confirmed"] is True
    assert calls == [1]
    assert h.car_link.sock is sock2
    assert sock1.closed is True
    assert any(ev.get("event") == "link_reset" and ev["ok"] is True for ev in res["events"])
    assert sock1.sent == ["STOP\n", "RUN -0.0800 0.0000 0 400\n", "STOP\n"]
    assert sock2.sent == ["PING\n", "STOP\n", "STOP\n"]   # 握手 + 新会话 STOP + 收尾 STOP


def test_unconfirmed_stop_and_failed_reconnect_raise_stop_failed(monkeypatch):
    """STOP 与重连都失败 = 车可能还在开：必须抛出来，不能静默继续。"""
    def factory():
        raise RuntimeError("发现不了小车")

    h, _, _, sock1 = _make(replies={"STOP": [[b"OK STOP\n"]]}, link_factory=factory)
    h.locate = lambda: (80.0, 0.0, 0.0)

    with pytest.raises(H.StopFailed) as excinfo:
        h.drive_to(0.0, 0.0, settle_s=0.0)

    assert "断电" in str(excinfo.value)
    events = excinfo.value.events
    assert {"event": "run_wait_failed", "outcome": "silent", "detail": None,
            "stopped_on_current_link": False} in events
    assert any(ev.get("event") == "link_reset" and ev["ok"] is False for ev in events)
    assert sock1.closed is True          # 重连前先关旧链路（固件单客户端）


# ----------------------------------------------------------- RESET-aware / 粘连行


def test_wait_reply_skips_reset_lines_and_collects_them():
    """R58 硬要求：RESET 行跟在 PONG/ID 后面；必须跳过，不能当命令回复。"""
    h, _, _, _ = _make()
    sock = FakeSock({"STOP": [[b"RESET ESP_RST_BROWNOUT count=1\n", b"OK STOP\n"]]})
    link = make_car_link(sock)

    assert h._try_stop(link) is True
    assert [t for _wall, t in h.reset_lines] == ["RESET ESP_RST_BROWNOUT count=1"]


def test_reconnect_handshake_collects_reset_and_does_not_shift_replies(monkeypatch):
    calls = []
    sock2 = FakeSock({"PING": [[b"PONG\n", b"RESET ESP_RST_BROWNOUT count=2\n"]],
                      "STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                      "RUN": [_run_chunks()]})

    def factory():
        calls.append(1)
        return make_car_link(sock2)

    h, _, _, _ = _make(replies={"STOP": [[b"OK STOP\n"]]}, link_factory=factory)
    positions = [(80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)

    assert res["reached"] is True and calls == [1]
    assert [t for _wall, t in res["reset_lines"]] == ["RESET ESP_RST_BROWNOUT count=2"]


def test_glued_control_line_is_split_out(monkeypatch):
    """现场原文：样本半行 + `OK RUN` 粘在一起；不拆就会当垃圾丢掉、等超时。"""
    glued = (b"I 2410259 0 0 0 0 0 0 0OK RUN pwm=-110,110,-110,110 "
             b"ms=400 kick=150ms/105\n")
    h, _, _, _ = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]],
                                "RUN": [[glued, b"D 10 -110 110 -110 110 0 0 0 0 0 0 0\n",
                                         b"DONE\n"]]})
    positions = [(80.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    h.locate = lambda: positions.pop(0)

    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert res["reached"] is True
    assert res["trace"][0]["run_reply"].startswith("OK RUN")


def test_split_glue_matches_wheelmap_implementation():
    """两个实现不许漂移：与 Task 7 采集工具的实现对拍（那套是现场校准过的）。"""
    try:
        from wheelmap_sweep import _split_glue as reference
    except ImportError:                      # 工具被挪走/改名不算 harness 的错
        pytest.skip("wheelmap_sweep 不可导入")
    samples = [
        "I 2410259 0 0 0 0 0 0 0OK M",
        "I 1 0 0 0 0 0 0 0D 2 0 0 0 0 0 0 0 0 0 0 0OK STOP",
        "OK RUN pwm=1,2,3,4 ms=400 kick=150ms/105",
        "DONE",
        "RESET ESP_RST_BROWNOUT count=1",
        "D 10 0 0 0 0 1 2 3 4 0 0 0",
    ]
    for sample in samples:
        assert H._split_glue(sample) == reference(sample), sample


# ------------------------------------------------------ 真机联调（本机相机可用就跑）


@pytest.mark.skipif(not Path("/dev/video0").exists(), reason="本机没有 /dev/video0")
def test_locate_on_the_live_camera_never_blows_up():
    """真相机 + 真标定 + 真检测跑一次 locate：只要求"不抛异常、结果自洽"。

    车在不在画面里、掉没掉码都可能（掉码是自动对焦的正常代价），所以
    None 也是合法结果 —— 这条测的是接线，不是精确度。相机被占用/打不开就 skip。
    """
    try:
        camera = CameraSession(settle_frames=5)
        camera.__enter__()
    except CameraUnavailable as exc:
        pytest.skip(f"相机打不开：{exc}")
    try:
        h = H.Harness(make_car_link(), camera, load_calib())
        pos = h.locate()
        if pos is not None:
            assert len(pos) == 3
            assert all(math.isfinite(v) for v in pos)
        else:
            assert h.last_locate_reason in ("boundary_moved", "no_vehicle_marker")
    finally:
        camera.close()


# ===================================================== Task 11：run_trial 编排
#
# 这些测试不碰网络、不碰相机、不开车：
# - `FakeSock` 按命令前缀回脚本，遥测（D 行）与 DONE 的先后关系和真机一致；
# - 相机是假的，但录下每次 `grab(fresh=…)` 的有效值 —— R54 的"默认要清流"靠它钉死；
# - `detect_markers` 脚本化，但 `build_field_frame` / `body_pose_in_field` 走真管线；
# - 时序参数全部压到毫秒级。
#
# 需要真相机的项（一条真 RUN、断链演练、真 RESET 行）不在这里 —— 见 Task 11 报告的
# 「待真机验证清单」。

# 固件三种 pwm 模式（odometry 用 pwm 极性定方向，E13/R13）：
PWM_FWD = [-150, 150, -150, 150]       # 前进：odometry 读成 +x
PWM_BWD = [150, -150, 150, -150]       # 后退：odometry 读成 −x
PWM_ROT_CCW = [150, 150, 150, 150]     # 原地逆时针：odometry 读成 +yaw


def _yaw_rvec(yaw_deg):
    """俯视相机下让车载码解出 field yaw = yaw_deg 的 rvec（与 R_YAW0 同族）。

    R_YAW0 = (0,0,−π/2) → yaw 0；绕 z 再减 deg 弧度 → yaw = +deg（已实测核对）。
    """
    return (0.0, 0.0, -math.pi / 2.0 - math.radians(yaw_deg))


def _counts_for(dist_mm):
    """四轮各滚 `dist_mm` 所需的原始计数（按 harness 的里程计默认常量反推）。"""
    return [int(round(dist_mm * c1 / (2.0 * math.pi * H.ODOM_R_EFF_MM)))
            for c1 in H.ODOM_C_1REV]


def _trial_pose(x, y, yaw_deg=0.0):
    """车载码在场地系 (x, y)、yaw_deg 的一帧。"""
    return scene(vehicle_xy=(float(x), float(y)), yaw_rvec=_yaw_rvec(yaw_deg))


def _d_line(t_ms, enc, pwm=PWM_FWD):
    """一行合法的固件 D 样本（13 字段：t / 4×pwm / 4×enc / 3×rpy）。"""
    return ("D {:d} {:d} {:d} {:d} {:d} {:d} {:d} {:d} {:d} 0.00 0.00 0.00\n"
            .format(t_ms, pwm[0], pwm[1], pwm[2], pwm[3],
                    enc[0], enc[1], enc[2], enc[3]))


def _trial_run_script(enc_end, *, pwm=PWM_FWD, with_done=True,
                      samples_at=(10, 510, 990), last_pwm=PWM_FWD):
    """一次 RUN 的脚本回复：OK RUN + 三条 D 样本（+可选 DONE）。

    中间那条（默认 t=510 ≥ 1000/2）就是中点抓帧的触发器 —— 顺序与固件一致：
    `OK RUN` 先回，样本按时间流式到达，最后才 `DONE`。

    `last_pwm`：最后那条 D 样本的 pwm。**真机上它是 `[0,0,0,0]`** —— 固件在
    补发它之前先 `stopAll()`（main.cpp:693-694）。默认保持整个 RUN 用 `pwm` 只是
    为了让老测试的期望不变；验 R66 的滑行并入时要显式传 `[0,0,0,0]`。
    """
    mid = [c // 2 for c in enc_end]
    enc_by_t = {samples_at[0]: [0, 0, 0, 0], samples_at[1]: mid, samples_at[2]: enc_end}
    group = [b"OK RUN pwm=-110,110,-110,110 ms=1000 kick=150ms/105\n"]
    for t in sorted(enc_by_t):
        group.append(_d_line(t, enc_by_t[t],
                             last_pwm if t == samples_at[2] else pwm).encode())
    if with_done:
        group.append(b"DONE\n")
    return group


def _i_line(enc, millis=123456):
    """一行合法的空闲 `I` 样本（与 D 行同形，t 是 `millis()`、pwm 恒为 0）。"""
    return ("I {:d} 0 0 0 0 {:d} {:d} {:d} {:d} 0.00 0.00 0.00\n"
            .format(millis, enc[0], enc[1], enc[2], enc[3]))


def _coast_replies(enc):
    """**按发送顺序**的两组回复：`STREAM 1`（确认 + 一条 I 行）、`STREAM 0`（确认）。

    `FakeSock` 按命令首词取队列，`STREAM 1` 与 `STREAM 0` 同归于 `"STREAM"`，
    正好按顺序各 pop 一组。
    """
    return [[b"OK STREAM 1\n", _i_line(enc).encode()], [b"OK STREAM 0\n"]]


def _samples(*encs, pwm=PWM_FWD, dt=100, t0=0):
    """直接拼样本字典序列（纯函数测试用，绕开 D 行解析）。"""
    return [{"t_ms": t0 + i * dt, "pwm": list(pwm), "enc": list(e),
             "rpy": [0.0, 0.0, 0.0]} for i, e in enumerate(encs)]


def _stop_replies(n):
    return [[b"OK STOP\n"] for _ in range(n)]


class FailingGrabCamera(FakeCamera):
    """第 `fail_on` 次 `grab()` 抛 `CameraUnavailable`（模拟运动模糊/USB 抖动）。"""

    def __init__(self, fail_on=2):
        super().__init__()
        self.fail_on = int(fail_on)

    def grab(self, fresh=True):
        self.calls.append(bool(fresh))
        if len(self.calls) == self.fail_on:
            raise CameraUnavailable(f"fake：第 {self.fail_on} 次抓帧失败")
        return b"frame"


# --------------------------------------------------- 有符号判据（纯函数，Important 3）


def test_body_delta_to_field_matches_the_symbol_chain():
    """车体系(前,左) → 场地系：θ=0 前进=+X；θ=90° 前进=−Y、左=−X（与 body_command 同链）。"""
    assert H.body_delta_to_field(100.0, 0.0, 0.0) == pytest.approx((100.0, 0.0))
    assert H.body_delta_to_field(0.0, 100.0, 0.0) == pytest.approx((0.0, -100.0))
    assert H.body_delta_to_field(100.0, 0.0, 90.0) == pytest.approx((0.0, -100.0))
    assert H.body_delta_to_field(0.0, 100.0, 90.0) == pytest.approx((-100.0, 0.0))


def test_translation_sign_ok_says_yes_no_and_judge_not():
    ok = H.translation_sign_ok
    assert ok((100.0, 0.0), (95.0, 3.0)) is True          # 同向
    assert ok((100.0, 0.0), (-95.0, 3.0)) is False        # 反向（正是要抓的）
    assert ok((0.0, 80.0), (3.0, 79.0)) is True
    assert ok((0.0, 80.0), (3.0, -79.0)) is False
    # 模长门槛以下不判（噪声/漂移/没动），不许冤枉合法数据：
    assert ok((4.0, 0.0), (-100.0, 0.0)) is None
    assert ok((100.0, 0.0), (3.0, -3.0)) is None
    # 恰好垂直（点积 0）不算同向：
    assert ok((10.0, 0.0), (0.0, -10.0)) is False


# --------------------------------------------------- run_trial 正常路径与降级


def test_run_trial_happy_path_end_to_end(monkeypatch):
    """直行 200 mm：起/中/终三点 + 三条样本 + DONE，全程对得上。"""
    counts = _counts_for(200.0)
    h, cam, detect, sock = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0), _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]],
                 "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts)]},
        coast_enc=counts)
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True and res["reason"] is None
    assert res["coast_read_failed"] is False        # R66：停车后读到了（本段没滑行）
    assert res["coast_counts"] == [0, 0, 0, 0]
    assert res["vision"]["start"] == pytest.approx((0.0, 0.0, 0.0))
    assert res["vision"]["mid"] == pytest.approx((100.0, 0.0, 0.0))
    assert res["vision"]["end"] == pytest.approx((200.0, 0.0, 0.0))
    assert res["vision_delta"] == pytest.approx({"dist_mm": 200.0, "yaw_deg": 0.0,
                                                 "dx_mm": 200.0, "dy_mm": 0.0})
    assert res["odometry_delta"]["dist_mm"] == pytest.approx(200.0, abs=1.0)
    assert res["odometry_delta"]["yaw_deg"] == pytest.approx(0.0, abs=0.5)
    assert res["n_samples"] == 3
    assert res["mid_capture_failed"] is False
    assert res["sign_ok"] is True and res["sign_dot_mm2"] > 0
    assert res["yaw_sign_ok"] is None               # 两边转角都 < 1°，不判
    assert res["run_reply"].startswith("OK RUN")
    assert res["stop_confirmed"] is True
    assert res["warnings"] == []
    # R54：三个关键帧都走 grab() 的默认路径（fresh=True，读前丢队列老帧）
    assert cam.calls == [True, True, True]
    assert sock.sent == ["PING\n",                                   # 开场探活
                         "STOP\n",                                   # 开场兜底停车
                         _run_line(matrix.EXPERIMENTS[0]),
                         "STREAM 1\n",                               # R66：终点帧之后
                         "STREAM 0\n",                               # 读滑行计数
                         "STOP\n"]                                   # 收尾兜底


def test_run_trial_mid_capture_failure_degrades_to_two_points(monkeypatch):
    """brief Step 2 / Review Focus #1：中点抓帧失败**不致命**，降级为起/终两点。"""
    counts = _counts_for(200.0)
    cam = FailingGrabCamera(fail_on=2)                  # 第 2 次 grab = 中点
    h, _, detect, _ = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(200.0, 0.0)],  # 中点的 detect 不会被调用
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts)]},
        camera=cam)
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True and res["reason"] is None
    assert res["vision"]["mid"] is None
    assert res["mid_capture_failed"] is True
    assert any("mid_capture_failed" in w for w in res["warnings"])
    assert res["vision_delta"]["dist_mm"] == pytest.approx(200.0)   # 两点照样算净值
    assert res["odometry_delta"]["dist_mm"] == pytest.approx(200.0, abs=1.0)
    assert res["sign_ok"] is True
    assert cam.calls == [True, True, True]                           # 失败那次也是 fresh=True


def test_run_trial_mid_boundary_drift_is_not_fatal(monkeypatch):
    """中点帧撞上边界码漂移（>15 px）也只降级，不许把实验判死。"""
    counts = _counts_for(200.0)
    h, _, detect, _ = _make(
        poses_list=[_trial_pose(0.0, 0.0),
                    scene(vehicle_xy=(100.0, 0.0), yaw_rvec=_yaw_rvec(0.0),
                          px_shift=(16.0, 0.0)),                     # 漂移帧
                    _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts)]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True
    assert res["vision"]["mid"] is None
    assert any("mid_capture_failed" in w for w in res["warnings"])
    assert any("BoundaryMarkersMoved" in w for w in res["warnings"])


def test_run_trial_end_boundary_drift_fails_the_trial(monkeypatch):
    """终点帧撞上漂移超限：实验判 boundary_moved（起/终是**关键帧**）。"""
    counts = _counts_for(200.0)
    h, _, detect, sock = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0),
                    scene(vehicle_xy=(200.0, 0.0), yaw_rvec=_yaw_rvec(0.0),
                          px_shift=(16.0, 0.0))],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts)]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "boundary_moved"
    assert "BoundaryMarkersMoved" in res["boundary_error"] or "ID" in res["boundary_error"]
    assert sock.sent[-1] == "STOP\n"                    # 收尾兜底照发


def test_run_trial_start_failures_send_no_run(monkeypatch):
    """起点帧失败：没解出 ID29 / 缺边界码 —— 都不发 RUN，但收尾 STOP 照发。"""
    for poses, expect in ((scene(vehicle_xy=None), "no_vehicle_marker"),
                          (scene(vehicle_xy=(0.0, 0.0), drop=(22,)), "boundary_moved")):
        sock = FakeSock({"PING": [[b"PONG\n"]], "STOP": _stop_replies(2)})
        cam = FakeCamera()
        h = H.Harness(make_car_link(sock), cam, dict(CALIB), locate_tries=1,
                      done_timeout_s=1.0, silence_timeout_s=0.05,
                      reply_timeout_s=0.05, handshake_timeout_s=0.5)
        monkeypatch.setattr(H, "detect_markers", lambda frame, c, p=poses: p)

        res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                          settle_after_done_s=0.0)

        assert res["ok"] is False and res["reason"] == expect, expect
        assert res["vision"]["start"] is None
        assert res["vision_delta"] is None and res["odometry_delta"] is None
        assert [s for s in sock.sent if s.startswith("RUN")] == []
        assert sock.sent == ["PING\n", "STOP\n", "STOP\n"]      # 探活 + 开场 + 收尾


# --------------------------------------------------- 有符号判据（整条链路）


def test_run_trial_signed_check_catches_translation_flip(monkeypatch):
    """复核 Important 3 的正面用例：量级对、方向反 —— 只比 dist 会静默通过。

    视觉说 +X 200 mm，编码器说 −X 200 mm：`dist_diff_pct ≈ 0`，
    旧的"<20%"验收会放行；`dot(vision, odometry) < 0` 必须把这条判死。
    """
    counts = _counts_for(200.0)
    h, _, detect, _ = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0), _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts, pwm=PWM_BWD)]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "sign_mismatch"
    assert res["sign_ok"] is False and res["sign_dot_mm2"] < 0
    # 这正是"标量距离看不出来"的那类失败：两个量级几乎相等。
    vd = res["vision_delta"]["dist_mm"]
    od = res["odometry_delta"]["dist_mm"]
    assert od == pytest.approx(200.0, abs=1.0)
    assert abs(vd - od) / vd < 0.05
    # 两个方向确实一正一负：
    assert res["vision_delta"]["dx_mm"] > 0 and res["odometry_delta"]["dx_mm"] < 0


def test_run_trial_rotate_in_place_is_not_judged_by_the_sign_check(monkeypatch):
    """原地旋转：平移两边都≈0 → `sign_ok=None`（判不出，不冤枉）；yaw 同向记下。"""
    roll_mm = H.ODOM_LXY_OVER_R * H.ODOM_R_EFF_MM * (math.radians(45.0))   # LXY·45°
    counts = _counts_for(roll_mm)
    exp = next(e for e in matrix.EXPERIMENTS if e.id == 15)                # 逆时针 45°
    h, _, detect, _ = _make(
        poses_list=[_trial_pose(0.0, 0.0, 0.0), _trial_pose(0.0, 0.0, 22.5),
                    _trial_pose(0.0, 0.0, 45.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts, pwm=PWM_ROT_CCW)]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(exp, settle_before_start_s=0.0, settle_after_done_s=0.0)

    assert res["ok"] is True and res["reason"] is None
    assert res["vision_delta"]["yaw_deg"] == pytest.approx(45.0)
    assert res["odometry_delta"]["yaw_deg"] == pytest.approx(45.0, abs=0.2)
    assert res["sign_ok"] is None                       # 平移太小：不判方向
    assert res["yaw_sign_ok"] is True                   # 同向的 yaw 是记下来的


# --------------------------------------------------- 超时 / RESET 定性（R56/R58）


def _timeout_scenario(*, sock2_ping_extra=b""):
    """造一个"RUN 后无 DONE"的场景，返回 (h, detect, sock1, sock2, calls)。"""
    counts = _counts_for(200.0)
    sock2 = FakeSock({"PING": [[b"PONG\n", sock2_ping_extra] if sock2_ping_extra else [b"PONG\n"]],
                      "STOP": _stop_replies(2)})
    calls = []

    def factory():
        calls.append(1)
        return make_car_link(sock2)

    replies = {"PING": [[b"PONG\n"]],
               "STOP": _stop_replies(2),                 # 开场 + 恢复阶段
               "RUN": [_trial_run_script(counts, with_done=False)]}
    h, _, detect, sock1 = _make(
        [ _trial_pose(0.0, 0.0) ],
        replies=replies, link_factory=factory,
        done_timeout_s=0.05, silence_timeout_s=10.0)
    return h, detect, sock1, sock2, calls


def test_run_trial_timeout_reconnects_and_checks_reset_before_concluding(monkeypatch):
    """R56：超时不许即判 brownout —— 强制重连 + 查 RESET，没有新证据才判 deadline。"""
    h, detect, sock1, sock2, calls = _timeout_scenario()
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "deadline_timeout"
    assert res["vision"]["end"] is None
    assert res["n_samples"] == 3                       # 样本留着当证据
    assert calls == [1]                                # 定性时确实强制重连了一次
    assert any(ev.get("event") == "timeout_diagnosis" and ev["ok"]
               for ev in res["events"])
    assert res["stop_confirmed"] is True
    assert sock2.sent == ["PING\n", "STOP\n", "STOP\n"]   # 握手 + 重连 STOP + 收尾


def test_run_trial_timeout_with_new_reset_line_is_board_reset(monkeypatch):
    """重连的 PING 带回一条**新的** RESET 行（brownout）→ 判 board_reset。"""
    h, detect, _sock1, _sock2, calls = _timeout_scenario(
        sock2_ping_extra=b"RESET ESP_RST_BROWNOUT count=1\n")
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "board_reset"
    assert res["new_reset_lines"] == ["RESET ESP_RST_BROWNOUT count=1"]
    assert any("count=1" in text for _wall, text in res["reset_lines"])


def test_run_trial_preexisting_reset_line_does_not_false_positive(monkeypatch):
    """开场就存在的 RESET 行（旧复位，每次 PING 都重发）不算本次证据 → deadline_timeout。

    没有这条，开机后就再没复位过的板子会永远被超时判成 brownout。
    """
    counts = _counts_for(200.0)
    sock2 = FakeSock({"PING": [[b"PONG\n", b"RESET ESP_RST_PANIC count=1\n"]],
                      "STOP": _stop_replies(2)})
    calls = []

    def factory():
        calls.append(1)
        return make_car_link(sock2)

    h, _, detect, sock1 = _make(
        [_trial_pose(0.0, 0.0)],
        replies={"PING": [[b"PONG\n", b"RESET ESP_RST_PANIC count=1\n"]],   # 开场基线
                 "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts, with_done=False)]},
        link_factory=factory, done_timeout_s=0.05, silence_timeout_s=10.0)
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "deadline_timeout"
    assert "new_reset_lines" not in res                # 同样的文本，不算新


def test_run_trial_reset_line_and_glued_reply_do_not_shift_the_stream(monkeypatch):
    """R58：RUN 流里混着 RESET 行 + 粘连的 OK RUN —— 回复不许错位、DONE 照认。"""
    counts = _counts_for(200.0)
    glued = (b"I 2410259 0 0 0 0 0 0 0OK RUN pwm=-110,110,-110,110 "
             b"ms=1000 kick=150ms/105\n")
    group = [b"RESET ESP_RST_BROWNOUT count=2\n", glued,
             _d_line(10, [0, 0, 0, 0]).encode(),
             _d_line(510, [c // 2 for c in counts]).encode(),
             _d_line(990, counts).encode(),
             b"DONE\n"]
    h, _, detect, _ = _make(
        [_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0), _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2), "RUN": [group]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True
    assert res["run_reply"].startswith("OK RUN")        # 粘连拆开、没错位
    run_wait = next(ev for ev in res["events"] if ev.get("event") == "run_wait")
    assert run_wait["n_reset"] == 1
    assert any("count=2" in text for _wall, text in res["reset_lines"])


# --------------------------------------------------- 停止安全性（R58 未修缺陷兜底）


def test_run_trial_run_rejected_is_reported(monkeypatch):
    h, _, detect, sock = _make(
        [_trial_pose(0.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [[b"ERR RUN vx vy om ms\n"]]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "run_rejected"
    assert res["n_samples"] == 0 and res["odometry_delta"] is None
    assert sock.sent[-1] == "STOP\n"


def test_run_trial_no_telemetry_rejects_the_trial(monkeypatch):
    """DONE 到了但没有 D 样本：不能拿 0 当"没动" —— 判 no_telemetry。"""
    h, _, detect, _ = _make(
        [_trial_pose(0.0, 0.0), _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [[b"OK RUN pwm=-110,110,-110,110 ms=1000 kick=150ms/105\n",
                          b"DONE\n"]]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "no_telemetry"
    assert res["n_samples"] == 0
    assert res["odometry_delta"] is None


def test_run_trial_stop_failure_after_run_raises_stop_failed(monkeypatch):
    """超时 + 当前链路 STOP 不确认 + 重连也失败 = 车可能还在开：必须抛，不能静默。"""
    def factory():
        raise RuntimeError("发现不了小车")

    h, _, detect, sock1 = _make(
        [_trial_pose(0.0, 0.0)],
        replies={"PING": [[b"PONG\n"]],
                 "STOP": [[b"OK STOP\n"], []],          # 开场确认；恢复阶段不确认
                 "RUN": [_trial_run_script(_counts_for(200.0), with_done=False)]},
        link_factory=factory, done_timeout_s=0.05, silence_timeout_s=10.0)
    monkeypatch.setattr(H, "detect_markers", detect)

    with pytest.raises(H.StopFailed, match="断电") as excinfo:
        h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                    settle_after_done_s=0.0)

    events = excinfo.value.events
    assert any(ev.get("event") == "run_wait_failed" and not ev["stopped_on_current_link"]
               for ev in events)
    assert any(ev.get("event") == "link_reset" and ev["ok"] is False for ev in events)
    assert sock1.closed is True                             # 重连前先关旧链路


def test_run_trial_final_stop_with_done_is_warning_not_stop_failed(monkeypatch):
    """收了 DONE（固件先 stopAll 再发）= 有停车证据：收尾 STOP 没确认只警告，不虚报。"""
    counts = _counts_for(200.0)
    h, _, detect, _ = _make(
        [_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0), _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]],
                 "STOP": [[b"OK STOP\n"]],              # 只够开场那次
                 "RUN": [_trial_run_script(counts)]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True
    assert res["stop_confirmed"] is False                   # 如实记录
    assert any("STOP 未确认" in w for w in res["warnings"])


def test_run_trial_final_stop_without_run_evidence_does_not_raise(monkeypatch):
    """起点帧就失败（没发过 RUN）：收尾 STOP 没确认也只是如实标注，不抛 StopFailed。"""
    h, _, detect, _ = _make(
        [scene(vehicle_xy=None)],
        replies={"PING": [[b"PONG\n"]], "STOP": [[b"OK STOP\n"]]})   # 收尾那次没回复
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "no_vehicle_marker"
    assert res["stop_confirmed"] is False
    assert any("STOP 未确认" in w for w in res["warnings"])


def test_run_trial_open_probe_failure_reconnects_then_proceeds(monkeypatch):
    """开场探活失败（链路死）→ 重连成功 → 实验照跑（新链路完成全部时序）。"""
    counts = _counts_for(200.0)
    sock2 = FakeSock({"PING": [[b"PONG\n"]], "STOP": _stop_replies(3),
                      "RUN": [_trial_run_script(counts)],
                      "STREAM": _coast_replies(counts)})
    calls = []

    def factory():
        calls.append(1)
        return make_car_link(sock2)

    h, _, detect, sock1 = _make(
        [_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0), _trial_pose(200.0, 0.0)],
        replies={},                                    # 旧链路：什么都没回
        link_factory=factory)
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True
    assert calls == [1] and sock1.closed is True
    assert sock2.sent == ["PING\n", "STOP\n", "STOP\n",
                          _run_line(matrix.EXPERIMENTS[0]),
                          "STREAM 1\n", "STREAM 0\n",       # R66（新链路）
                          "STOP\n"]
    assert any(ev.get("event") == "open_probe" and ev.get("reconnected")
               for ev in res["events"])


def test_run_trial_validates_arguments_before_any_io():
    sock = FakeSock({})
    h = H.Harness(make_car_link(sock), FakeCamera(), dict(CALIB))
    bad = matrix.Experiment(99, "straight", "坏实验", 0.1, 0.0, 0.0, 0.0001, 1)

    with pytest.raises(ValueError, match="ms"):
        h.run_trial(bad)
    with pytest.raises(ValueError, match="mid_fraction"):
        h.run_trial(matrix.EXPERIMENTS[0], mid_fraction=1.5)
    with pytest.raises(ValueError, match="沉降"):
        h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=-1.0)
    assert sock.sent == []                       # 连开场 PING/STOP 都不发


def test_run_trial_silent_wait_reconnects_once_and_reports_brownout(monkeypatch):
    """RUN 中链路断（silent）：当前链路 STOP 发不出去 → 重连一次就够，别再重连。

    重连的握手 PING 带回新 RESET（brownout）→ `board_reset`；
    恢复阶段已经换过链路，就**不**再打一次"定性专用"的重连。
    """
    counts = _counts_for(200.0)
    sock2 = FakeSock({"PING": [[b"PONG\n", b"RESET ESP_RST_BROWNOUT count=4\n"]],
                      "STOP": _stop_replies(2)})
    calls = []

    def factory():
        calls.append(1)
        return make_car_link(sock2)

    h, _, detect, sock1 = _make(
        [_trial_pose(0.0, 0.0)],
        replies={"PING": [[b"PONG\n"]],
                 "STOP": [[b"OK STOP\n"]],              # 只有开场那次；恢复阶段发不出去
                 "RUN": [_trial_run_script(counts, with_done=False)]},
        link_factory=factory, done_timeout_s=10.0, silence_timeout_s=0.05)
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is False and res["reason"] == "board_reset"
    assert res["run_outcome"] == "silent"
    assert calls == [1]                                 # 只重连一次
    assert not any(ev.get("event") == "timeout_diagnosis" for ev in res["events"])
    assert any(ev.get("event") == "link_reset" and ev["ok"] is True
               for ev in res["events"])
    assert res["stop_confirmed"] is True
    assert res["new_reset_lines"] == ["RESET ESP_RST_BROWNOUT count=4"]


def test_run_trial_twice_on_one_session_keeps_state_separate(monkeypatch):
    """同一 Harness 连跑两条（Task 12 的批量模式）：samples/基线/相机调用都不串。"""
    counts = _counts_for(200.0)
    poses = [_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0), _trial_pose(200.0, 0.0)] * 2
    h, cam, detect, sock = _make(
        poses_list=poses,
        replies={"PING": [[b"PONG\n"], [b"PONG\n"]],
                 "STOP": _stop_replies(4),
                 "RUN": [_trial_run_script(counts), _trial_run_script(counts)],
                 "STREAM": _coast_replies(counts) * 2})
    monkeypatch.setattr(H, "detect_markers", detect)

    first = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                        settle_after_done_s=0.0)
    second = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                         settle_after_done_s=0.0)

    assert first["ok"] is True and second["ok"] is True
    assert first["samples"] is not second["samples"]
    assert first["n_samples"] == 3 and second["n_samples"] == 3
    assert cam.calls == [True] * 6
    # R66：每条都多一对 STREAM 1/0（终点帧之后读滑行计数），两条各读各的
    assert sock.sent == ["PING\n", "STOP\n", _run_line(matrix.EXPERIMENTS[0]),
                         "STREAM 1\n", "STREAM 0\n", "STOP\n"] * 2
    assert first["coast_counts"] == [0, 0, 0, 0]
    assert second["coast_counts"] == [0, 0, 0, 0]


# -------------------------------------------- R66：停车后的惯性滑行（窗口对齐）


def test_coasted_samples_keeps_direction_when_last_d_sample_is_stopped():
    """R66 的核心陷阱：固件补发的最后一条 D 样本 `pwm=[0,0,0,0]`。

    `integrate_encoders` 用 `sgn(pwm[k-1])` 定方向（单相计数不辨向，E13）——
    要是把滑行样本直接接在全零 pwm 的样本后面，**滑行段会被算成 0 位移**：
    不报错、不告警，只是静默少报一截。所以合成那段必须继承本段的行进方向。
    """
    run = _samples([0, 0, 0, 0], [100, 100, 100, 100])
    run[-1] = dict(run[-1], pwm=[0, 0, 0, 0])       # 末条 = 固件停车后补发的 D 样本
    merged = H.coasted_samples(run, {"enc": [150, 150, 150, 150], "rpy": [0.0, 0.0, 0.0]})

    assert merged is not run and len(merged) == 3
    assert merged[-1]["synthetic"] == "coast"
    assert merged[-1]["pwm"] == PWM_FWD             # 继承方向，而不是全零
    # 原样本序列不许被就地改写（调用方拿它当"固件原样证据"）
    assert run[-1]["pwm"] == [0, 0, 0, 0]

    # 整段 150 计数（用全非零 pwm 的同长度序列做基线）应得同样的位移
    same = H.coasted_samples(_samples([0, 0, 0, 0], [150, 150, 150, 150]),
                             {"enc": [150, 150, 150, 150], "rpy": None})
    a = odometry.integrate_encoders(merged, c_1rev=1276.0, sign=[-1, 1, -1, 1],
                                    l_xy_over_r=0.12825 / 0.0335, r_eff_mm=33.5)
    b = odometry.integrate_encoders(same, c_1rev=1276.0, sign=[-1, 1, -1, 1],
                                    l_xy_over_r=0.12825 / 0.0335, r_eff_mm=33.5)
    assert a[-1][0] == pytest.approx(b[-1][0], abs=0.5)      # 方向没丢，位移对得上


def test_coasted_samples_discards_reading_below_last_sample():
    """读数比最后一条 D 样本还小 = 不可信（多半是 RUN 之前淌进来的旧 I 行）。

    递减会让 `integrate_encoders` 抛 `ValueError` 把整个试次判死（R13 的契约），
    所以这里必须**先**丢掉，而不是原样喂进去。
    """
    run = _samples([0, 0, 0, 0], [300, 300, 300, 300])
    assert H.coasted_samples(run, {"enc": [100, 100, 100, 100], "rpy": None}) is run
    assert H.coasted_samples(run, {"enc": [301, 299, 300, 300], "rpy": None}) is run  # 有一只回退
    # 只不过某一只多跳了一格（其余持平）不算回退：那只的 dc 是 +1，不递减
    half = H.coasted_samples(run, {"enc": [301, 300, 300, 300], "rpy": None})
    assert half is not run and half[-1]["enc"] == [301, 300, 300, 300]
    # 相等 = "这轮确实没滑行"：可以接一段零位移的合成样本，但不许算成负的
    same = H.coasted_samples(run, {"enc": [300, 300, 300, 300], "rpy": None})
    assert same is not run and same[-1]["synthetic"] == "coast"
    traj = odometry.integrate_encoders(same, c_1rev=1276.0, sign=[-1, 1, -1, 1],
                                       l_xy_over_r=0.12825 / 0.0335, r_eff_mm=33.5)
    assert traj[-1][0] == pytest.approx(traj[-2][0], abs=1e-9)      # 零滑行 = 零位移


def test_coasted_samples_noop_without_coast_or_direction():
    run = _samples([0, 0, 0, 0], [100, 100, 100, 100])
    assert H.coasted_samples(run, None) is run                      # 没读到
    one = _samples([0, 0, 0, 0])
    assert H.coasted_samples(one, {"enc": [9, 9, 9, 9], "rpy": None}) is one  # 样本不足两条
    # 整段 RUN 都没转过（pwm 全零）：没有方向可继承，宁可退回、不猜
    still = _samples([0, 0, 0, 0], [50, 50, 50, 50], pwm=[0, 0, 0, 0])
    assert H.coasted_samples(still, {"enc": [80, 80, 80, 80], "rpy": None}) is still


def test_run_trial_coast_makes_odometry_window_match_vision(monkeypatch):
    """R66 端到端：视觉终点帧抓到的是**滑行完**的位置，里程计也得算到那儿。

    真机症状：同一发 S300 脉冲，`vision_delta.dist_mm ≈ 81`、`odometry_delta.dist_mm
    ≈ 30`（差 2.6 倍），看着像 c1 标错了 —— 其实就是停车后 1.5~1.7 倍的通电距离
    自由滑行，而 `D` 行随停车一起停。这一条锁住"补读一次空闲编码器就对齐"。
    """
    counts_run = _counts_for(300.0)                     # 电机通电期
    counts_coast = _counts_for(450.0)                   # 停车 + 自由滑行
    h, _cam, detect, _sock = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(150.0, 0.0),
                    _trial_pose(450.0, 0.0)],           # 视觉量的是 450
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts_run, last_pwm=[0, 0, 0, 0])]},
        coast_enc=counts_coast)
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True and res["warnings"] == []
    assert res["coast_read_failed"] is False
    assert res["coast_counts"] == [b - a for a, b in zip(counts_run, counts_coast)]
    assert res["vision_delta"]["dist_mm"] == pytest.approx(450.0, abs=1.0)
    # 关键断言：里程计窗口也到 450（不是只到电机停车的 300）
    assert res["odometry_delta"]["dist_mm"] == pytest.approx(450.0, abs=2.0)
    # 保留的原始证据仍是固件 D 行本身，合成样本只进积分副本
    assert res["n_samples"] == 3
    assert res["samples"][-1]["pwm"] == [0, 0, 0, 0]
    assert all("synthetic" not in s for s in res["samples"])


def test_run_trial_coast_read_failure_warns_without_failing(monkeypatch):
    """读不到空闲编码器**不是**致命：试次照常判有效，但必须出声、必须标注窗口差异。"""
    counts = _counts_for(200.0)
    h, _cam, detect, sock = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(100.0, 0.0),
                    _trial_pose(200.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts)]})     # 没有 coast_enc ⇒ STREAM 无回复
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True and res["coast_read_failed"] is True
    assert any("coast_read_failed" in w for w in res["warnings"])
    assert "coast_counts" not in res
    assert res["odometry_delta"]["dist_mm"] == pytest.approx(200.0, abs=1.0)  # 退回只积 D 行
    assert "STREAM 0\n" in sock.sent                     # 失败路径也要把空闲流关掉


def test_read_idle_enc_picks_newest_not_last_line(monkeypatch):
    """`_read_idle_enc` 必须挑**计数最大**的那条 `I` 行，不是最后收到的那条。

    真机上车停着不动也会持续发 `I` 行，而链路上还可能淌着 RUN 之前残留的旧 `I` 行
    （R62 那类陈旧读数的亲戚）。它们到达的顺序和计数的大小**无关** —— 这里就是
    "新行先到、旧行后到"。挑最后一条会选中陈旧读数：要么被 `coasted_samples` 的单调性
    守卫当噪声丢掉（滑行量丢失，里程计窗口又短一截），要么更糟：假绿。
    """
    fresh = [900, 880, 910, 870]        # 计数更大 = 更新（车只会往前滚）
    stale = [200, 190, 205, 195]        # RUN 之前残留的旧行，后到
    sock = FakeSock({"STREAM": [[b"OK STREAM 1\n",
                                 _i_line(fresh).encode(), _i_line(stale).encode()],
                               [b"OK STREAM 0\n"]]})
    h = H.Harness(make_car_link(sock), FakeCamera(()), dict(CALIB))

    got = h._read_idle_enc(h.car_link)

    assert got is not None
    assert got["enc"] == fresh
    assert sock.sent == ["STREAM 1\n", "STREAM 0\n"]


def test_run_trial_coast_survives_stale_idle_line_arriving_last(monkeypatch):
    """端到端：陈旧 `I` 行排在新鲜行后面到达，滑行量仍然必须并进来。"""
    counts_run = _counts_for(300.0)
    counts_coast = _counts_for(450.0)
    stale = _counts_for(120.0)                   # 比 RUN 期间还小 ⇒ 一定是陈旧行
    h, _cam, detect, _sock = _make(
        poses_list=[_trial_pose(0.0, 0.0), _trial_pose(150.0, 0.0),
                    _trial_pose(450.0, 0.0)],
        replies={"PING": [[b"PONG\n"]], "STOP": _stop_replies(2),
                 "RUN": [_trial_run_script(counts_run, last_pwm=[0, 0, 0, 0])],
                 "STREAM": [[b"OK STREAM 1\n",
                             _i_line(counts_coast).encode(),
                             _i_line(stale).encode()],
                            [b"OK STREAM 0\n"]]})
    monkeypatch.setattr(H, "detect_markers", detect)

    res = h.run_trial(matrix.EXPERIMENTS[0], settle_before_start_s=0.0,
                      settle_after_done_s=0.0)

    assert res["ok"] is True and res["warnings"] == []
    assert res["coast_read_failed"] is False
    assert res["coast_counts"] == [b - a for a, b in zip(counts_run, counts_coast)]
    assert res["odometry_delta"]["dist_mm"] == pytest.approx(450.0, abs=2.0)
