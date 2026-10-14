"""`vgt.camera` 的行为锁：**一个相机属性都不许写**（R49/R53）+ 会话生命周期。

全部用例不需要硬件：设备句柄用 `FakeCap` 替身，开设备的唯一入口
`vgt.camera.open_capture`（以及它内部的 `cv2.VideoCapture`）由 monkeypatch 换掉。

钉住的四件事：

1. **R53 反过来钉的那条测试**：`apply_locked_settings` 只写 FOURCC/宽/高
   （值与 `verify_vision.grab_frame` 逐字一致），**不写** FOCUS / AUTOFOCUS /
   AUTO_EXPOSURE / EXPOSURE。计划 Step 1 断言的是"写了 166/275"，**实测否掉了**
   （`AUTO_EXPOSURE=0` 写不进去、`FOCUS=275` 把 ID20 检出率打到 7%），
   所以这里反过来钉 —— 见 `vgt/camera.py` 的模块 docstring。
2. **失败路径**：打不开、读不到帧、没进上下文 → `CameraUnavailable`
   （`RuntimeError` 子类，一句人话，不是 traceback）；存盘失败 → `RuntimeError`。
3. **句柄生命周期**：开设备只发生在 `__enter__`；`__exit__` 一定 `release()`
   —— 漏掉它 `/dev/video0` 会被占住，下一个工具直接打不开。
4. **C1 的新鲜度**：`grab()` 在**读到返回值之前**先丢 `FRESH_DISCARD_FRAMES` 帧
   （≥ 实测队列深度 4）。"丢了几帧、丢在读之前"是可测的，而"返回的到底是新帧还是
   队列里的老帧"需要真实静置才能造 —— 防线就钉在计数与顺序上。
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

# ⚠ 先 import `vgt`：`实验/vgt/__init__.py` 会把 `视觉测量/脚本` 前置进 sys.path，
# 之后的 `vision_core` / `verify_vision` 才解析得到（R11）。顺序反过来会在
# import 期就 ModuleNotFoundError。
from vgt import camera
from vgt.camera import (
    CAMERA_DEV,
    LOCKED_EXPOSURE,
    LOCKED_FOCUS,
    CameraSession,
    CameraUnavailable,
    apply_locked_settings,
)

from vision_core import calib_store  # noqa: E402  （等 vgt 补完 sys.path）

# 相机参数从 verify_vision import：本测试要钉的正是"两边的三个值一致"这件事，
# 在测试里再抄一遍 1920/1080/"MJPG" 就永远钉不出漂移。
from verify_vision import (  # noqa: E402
    CAPTURE_FOURCC,
    CAPTURE_HEIGHT,
    CAPTURE_WIDTH,
    DEFAULT_SETTLE_FRAMES,
)

# `apply_locked_settings` **允许**写的属性（就这三个）。
_FORMAT_PROPS = frozenset(
    int(p) for p in (cv2.CAP_PROP_FOURCC, cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT)
)

# **禁止**写的四个属性：名字 + `camera_props.json` 里的历史快照值（失败信息要能指认现场）。
_FORBIDDEN_PROPS = (
    (cv2.CAP_PROP_FOCUS, "CAP_PROP_FOCUS", LOCKED_FOCUS),
    (cv2.CAP_PROP_AUTOFOCUS, "CAP_PROP_AUTOFOCUS", 1),
    (cv2.CAP_PROP_AUTO_EXPOSURE, "CAP_PROP_AUTO_EXPOSURE", 3),
    (cv2.CAP_PROP_EXPOSURE, "CAP_PROP_EXPOSURE", LOCKED_EXPOSURE),
)

# C1 的实测队列深度（/dev/video0、cv2 4.6.0、MJPG 1920x1080、29.8 fps）：
# 本机静置 3.0 s / 0.5 s 后连读，前 3 帧的采集时刻都只比静置开始时晚 36/68/100 ms
# （即最老约 2.9 s 旧），第 4 帧才是实时帧；复核者在同一台相机上量到 4 帧。
# 深度的**上界**是 `CAP_PROP_BUFFERSIZE`（camera_props.json: 4）。
C1_MEASURED_QUEUE_DEPTH = 4


class FakeCap:
    """`cv2.VideoCapture` 的替身：**按调用顺序**记下每一次 `grab`/`read`（`events`），
    以及每一次 `set`（`props` 列表）；按给定顺序吐帧，帧用完就返回 `(False, None)`。

    `events` 是给 C1 的"丢了几帧、丢在读之前还是之后"用的；`props` 的断言用
    set/dict 看"写了哪些属性"，重复写本来就不是那两个断言的关注点。
    """

    def __init__(self, frames=(), opened=True):
        self.frames = list(frames)
        self.opened = opened
        self.props = []             # [(prop, value), ...] 按调用顺序
        self.events = []            # ["grab" / "read", ...] 按调用顺序
        self.grabbed = 0
        self.reads = 0
        self.released = 0

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.props.append((prop, value))
        return True

    def grab(self):
        self.events.append("grab")
        self.grabbed += 1
        return True

    def read(self):
        self.events.append("read")
        self.reads += 1
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self):
        self.released += 1


def _session(monkeypatch, fake, **kwargs) -> CameraSession:
    """把 `vgt.camera.open_capture` 换成吐 `fake` 的替身，返回还没打开的会话。"""
    monkeypatch.setattr(camera, "open_capture", lambda device: fake)
    return CameraSession(**kwargs)


def _frame(shape=(8, 12, 3), value=180) -> np.ndarray:
    return np.full(shape, value, np.uint8)


# --- 1. 只写采集格式（R53 的逐字裁决）---------------------------------------------


def test_apply_locked_settings_writes_only_the_capture_format():
    """反过来钉：写了 FOURCC/宽/高（与 verify_vision 一致），且**没有**写那四个属性。"""
    cap = FakeCap()
    apply_locked_settings(cap)
    written = dict(cap.props)

    assert set(written) == _FORMAT_PROPS, (
        "apply_locked_settings 只许写 FOURCC/宽/高；实际写了 "
        f"{sorted(written)}（cv2 常量值见 cv2.CAP_PROP_*）")
    assert written[int(cv2.CAP_PROP_FOURCC)] == cv2.VideoWriter_fourcc(*CAPTURE_FOURCC)
    assert written[int(cv2.CAP_PROP_FRAME_WIDTH)] == CAPTURE_WIDTH == 1920
    assert written[int(cv2.CAP_PROP_FRAME_HEIGHT)] == CAPTURE_HEIGHT == 1080

    for prop, name, snapshot in _FORBIDDEN_PROPS:
        assert int(prop) not in written, (
            f"apply_locked_settings 写了 {name}（历史快照值 {snapshot}）——"
            "R49/R53 明确禁止：这台相机的确定性来自它自己的自动对焦/曝光，"
            "锁焦会把检出率打到 7%/20%。")


def test_historical_snapshot_constants_are_kept_but_not_applied():
    """三个常量按接口保留（历史快照值）；"不应用"由上一个用例逐项钉住。"""
    assert CAMERA_DEV == "/dev/video0"
    assert LOCKED_EXPOSURE == 166
    assert LOCKED_FOCUS == 275


# --- 2. 打开设备 -----------------------------------------------------------------


def test_open_capture_uses_v4l2_and_applies_the_capture_format(monkeypatch):
    fake = FakeCap()
    seen = []
    monkeypatch.setattr(camera.cv2, "VideoCapture",
                        lambda *args, **kwargs: seen.append((args, kwargs)) or fake)

    cap = camera.open_capture("/dev/video0")

    assert cap is fake
    assert seen == [(("/dev/video0", cv2.CAP_V4L2), {})]
    assert set(dict(fake.props)) == _FORMAT_PROPS


def test_open_capture_raises_and_releases_when_the_device_cannot_be_opened(monkeypatch):
    fake = FakeCap(opened=False)
    monkeypatch.setattr(camera.cv2, "VideoCapture", lambda *args, **kwargs: fake)

    with pytest.raises(CameraUnavailable) as excinfo:
        camera.open_capture("/dev/video0")

    assert isinstance(excinfo.value, RuntimeError)
    assert "打不开" in str(excinfo.value)
    assert fake.released == 1, "打不开时也要 release，否则设备被这根废句柄占住"


# --- 3. 会话生命周期 -------------------------------------------------------------


def test_context_manager_drops_settle_frames_then_reads_fresh_frames(monkeypatch):
    fake = FakeCap(frames=[_frame()])
    session = _session(monkeypatch, fake, settle_frames=7)

    with session as cam:
        assert cam is session
        assert cam.is_open
        assert fake.grabbed == 7, "__enter__ 要丢掉缓冲帧（等自动对焦收敛，R39）"
        frame = cam.grab()

    assert fake.reads == 1, "grab() 消费的必须是丢帧之后的那一帧"
    assert frame.shape == (8, 12, 3)
    assert fake.released == 1
    assert not session.is_open


def test_default_settle_frames_comes_from_verify_vision(monkeypatch):
    fake = FakeCap(frames=[_frame()])
    with _session(monkeypatch, fake):
        pass
    assert fake.grabbed == DEFAULT_SETTLE_FRAMES


def test_open_capture_is_called_with_the_default_device(monkeypatch):
    opened = []
    monkeypatch.setattr(camera, "open_capture", lambda device: opened.append(device) or FakeCap())
    with CameraSession(settle_frames=0):
        pass
    assert opened == [CAMERA_DEV]


def test_grab_failure_raises_runtime_error(monkeypatch):
    session = _session(monkeypatch, FakeCap(frames=()), settle_frames=0)

    with session as cam:
        with pytest.raises(RuntimeError) as excinfo:
            cam.grab()

    assert isinstance(excinfo.value, CameraUnavailable)
    assert "读帧失败" in str(excinfo.value)


# --- 3b. C1：读之前先清流 --------------------------------------------------------


def test_grab_discards_the_measured_queue_depth_before_every_read(monkeypatch):
    """C1：每次 `grab()` 都要**先丢** ≥实测队列深度 的帧，再读。

    丢少了会静默返回静置开始时拍的旧帧（本机实测最老约 2.9 s）；丢多了只多花 ~33 ms。
    """
    assert camera.FRESH_DISCARD_FRAMES >= C1_MEASURED_QUEUE_DEPTH, (
        "队列深度实测 3–4 帧（上界 CAP_PROP_BUFFERSIZE=4）；丢帧数少于它，"
        "静置后第一次 grab() 就会把老帧当成本次测量返回，而且不报错")

    fake = FakeCap(frames=[_frame(), _frame()])
    session = _session(monkeypatch, fake, settle_frames=0)

    with session as cam:
        cam.grab()
        cam.grab()

    expected = (["grab"] * camera.FRESH_DISCARD_FRAMES + ["read"]) * 2
    assert fake.events == expected, "每一次 grab 都要先清流、再读"


def test_grab_fresh_false_reads_without_discarding(monkeypatch):
    """显式逃生门：刚读过一帧、队列是空的连拍场景才用（docstring 写明要慎用）。"""
    fake = FakeCap(frames=[_frame()])
    session = _session(monkeypatch, fake, settle_frames=0)

    with session as cam:
        frame = cam.grab(fresh=False)

    assert fake.events == ["read"]
    assert frame.shape == (8, 12, 3)


def test_session_discard_count_is_configurable(monkeypatch):
    """清流帧数走实例属性（换相机/测试可调），不是 `grab()` 里写死的常量。"""
    fake = FakeCap(frames=[_frame()])
    session = _session(monkeypatch, fake, settle_frames=0, fresh_discard_frames=2)

    with session as cam:
        cam.grab()

    assert fake.events == ["grab", "grab", "read"]


def test_grab_outside_the_context_never_opens_the_device(monkeypatch):
    opened = []
    monkeypatch.setattr(camera, "open_capture", lambda device: opened.append(device))
    session = CameraSession(settle_frames=0)

    with pytest.raises(CameraUnavailable):
        session.grab()
    assert opened == [], "没进上下文时 grab() 不许偷偷开设备"

    monkeypatch.setattr(camera, "open_capture",
                        lambda device: opened.append(device) or FakeCap(frames=[_frame()]))
    with session:
        session.grab()
    assert opened == [CAMERA_DEV], "__enter__ 开这一次；grab() 自己不许再开"
    with pytest.raises(CameraUnavailable):
        session.grab()
    assert opened == [CAMERA_DEV], "退出之后同样不许偷偷重开"


def test_exit_releases_the_device_even_when_the_body_raises(monkeypatch):
    fake = FakeCap(frames=[_frame()])
    session = _session(monkeypatch, fake, settle_frames=0)

    with pytest.raises(ValueError):
        with session:
            raise ValueError("会话体炸了")

    assert fake.released == 1
    assert not session.is_open


def test_close_is_idempotent(monkeypatch):
    fake = FakeCap(frames=[_frame()])
    session = _session(monkeypatch, fake, settle_frames=0)
    session.__enter__()

    session.close()
    session.close()

    assert fake.released == 1, "close() 幂等：第二次不许再 release 一次"


def test_double_enter_is_refused(monkeypatch):
    fake = FakeCap()
    session = _session(monkeypatch, fake, settle_frames=0)

    with session:
        with pytest.raises(CameraUnavailable):
            session.__enter__()

    assert fake.released == 1


# --- 4. 存盘 ---------------------------------------------------------------------


def test_save_writes_a_readable_jpeg_and_returns_the_path(monkeypatch, tmp_path):
    frame = _frame((16, 24, 3), value=210)
    session = _session(monkeypatch, FakeCap(frames=[frame]), settle_frames=0)
    target = tmp_path / "shots" / "trial_01.jpg"

    with session as cam:
        out = cam.save(target)

    assert Path(out) == target
    back = cv2.imread(out)
    assert back is not None, f"存出来的图要能读回来：{out}"
    assert back.shape == frame.shape


def test_save_resolves_relative_paths_against_the_repo_root(monkeypatch):
    """R3：相对路径按仓库根解析，与 cwd 无关（`calib_store.resolve_path` 是唯一来源）。"""
    seen = {}
    monkeypatch.setattr(camera.cv2, "imwrite",
                        lambda path, img, params: seen.setdefault("path", path) or True)
    session = _session(monkeypatch, FakeCap(frames=[_frame((4, 4, 3))]), settle_frames=0)

    with session as cam:
        out = cam.save("视觉测量/结果/captures/.task8_path_probe.jpg")

    assert seen["path"] == out
    assert Path(out).is_absolute()
    assert Path(out).relative_to(calib_store.REPO_ROOT) == Path(
        "视觉测量/结果/captures/.task8_path_probe.jpg")


def test_save_reports_an_imwrite_false_instead_of_pretending_success(monkeypatch, tmp_path):
    """`imwrite` 返回 False（磁盘满/权限不足）不许静默当成功 —— docstring 的承诺。"""
    session = _session(monkeypatch, FakeCap(frames=[_frame()]), settle_frames=0)
    monkeypatch.setattr(camera.cv2, "imwrite", lambda path, img, params: False)

    with session as cam, pytest.raises(RuntimeError) as excinfo:
        cam.save(tmp_path / "nope.jpg")

    assert "存图失败" in str(excinfo.value)


def test_save_wraps_cv2_errors_from_imwrite(monkeypatch, tmp_path):
    """未知扩展名时 `imwrite` 是**抛** `cv2.error`（`live_field.py` 踩过），要收成一句人话。"""
    session = _session(monkeypatch, FakeCap(frames=[_frame()]), settle_frames=0)

    def boom(path, img, params):
        raise cv2.error("could not find a writer")

    monkeypatch.setattr(camera.cv2, "imwrite", boom)

    with session as cam, pytest.raises(RuntimeError) as excinfo:
        cam.save(tmp_path / "nope.zzz")

    assert "存图失败" in str(excinfo.value)


# --- 5. 导入/构造不许碰设备 -------------------------------------------------------


def test_constructing_a_session_never_touches_the_device(monkeypatch):
    calls = []
    monkeypatch.setattr(camera.cv2, "VideoCapture", lambda *args, **kwargs: calls.append(args))
    session = CameraSession()
    session.close()
    assert calls == []


def test_importing_vgt_camera_does_not_open_the_device():
    """`import vgt.camera` 不许有副作用（与 `live_field.py` 的同款用例同一意图）。

    在子进程里把 `cv2.VideoCapture` 先换成"一碰就炸"，再 import 本模块 —— 导入期
    偷偷开相机的写法会当场 AssertionError。cwd 用 `实验/`（本包只在那里可导入）。
    """
    experiments_dir = Path(camera.__file__).resolve().parents[1]
    code = (
        "import cv2\n"
        "def boom(*args, **kwargs):\n"
        "    raise AssertionError('import vgt.camera 不该开相机')\n"
        "cv2.VideoCapture = boom\n"
        "import vgt.camera  # noqa: F401\n"
        "print('ok')\n"
    )
    proc = subprocess.run([sys.executable, "-c", code], cwd=str(experiments_dir),
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, f"stdout={proc.stdout!r}\nstderr={proc.stderr!r}"
    assert "ok" in proc.stdout
