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


def _make(poses_list=(), replies=None, link_factory=None, frames=(), **kw):
    """造一套假设备：返回 `(harness, camera, detect, sock)`。

    `detect` 是脚本化的 `detect_markers` 替身（逐帧 pop `poses_list`，用完给 {}）；
    由每个测试自己 `monkeypatch.setattr(H, "detect_markers", detect)`。
    时序参数压小，让所有等待路径都在毫秒级跑完。
    """
    timeouts = dict(done_timeout_s=1.0, silence_timeout_s=0.05,
                    reply_timeout_s=0.05, handshake_timeout_s=0.5,
                    locate_tries=1)
    timeouts.update(kw)
    sock = FakeSock(replies)
    camera = FakeCamera(frames)
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
    h, _, _, sock = _make(replies={"STOP": [[b"OK STOP\n"], [b"OK STOP\n"]]})
    h.locate = lambda: (3.0, 2.0, 0.0)          # 距目标 3.6 mm ≤ 12 mm

    res = h.drive_to(0.0, 0.0, settle_s=0.0)
    assert res["reached"] is True and res["iters"] == 0
    assert res["final"] == (3.0, 2.0, 0.0)
    assert [s for s in sock.sent if s.startswith("RUN")] == []


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
    assert res["reached"] is True


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
