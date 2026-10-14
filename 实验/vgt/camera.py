"""持久相机会话 `CameraSession`：一次打开，整个实验会话复用同一根句柄。

    # 从 `实验/` 跑（`vgt` 只在 `实验/` 于 sys.path 上时可导入，R11/R17）
    cd 实验
    python3 -c "
    from vgt.camera import CameraSession
    with CameraSession() as cam:
        frame = cam.grab()                                  # BGR ndarray, 1920x1080
        print(cam.save('视觉测量/结果/captures/probe.jpg'))  # 相对路径按仓库根解析
    "

## 与既有两个相机入口的关系（都不改，只是各取所需）

- `verify_vision.grab_frame()`：每张照片"开 → 丢帧 → 读 → 关"。一次性拍照的正确做法
  （失败换新句柄重试、每张独立采样），但每次都要等自动对焦预热（默认丢 60 帧，约 2 s）。
- `live_field.open_camera()` + `settle()`：常驻句柄 + 启动丢帧。本类就是它的**无窗口版本**：
  同样"打不开给一句人话、不 traceback"的口径，但没有实时循环、没有叠加层，
  供 Task 10–12 的 harness 在整个会话里反复 `grab()`/`save()`。

## ⚠ 不锁曝光、不锁对焦（SDD 台账 R49/R53，与计划原文冲突，以实测为准）

计划给的接口是 `__enter__` 关闭自动曝光/自动对焦并写死 `EXPOSURE=166`、`FOCUS=275`
（取自 `视觉测量/结果/camera_props.json`）。**实测把这条前提整条否掉了：**

1. `CAP_PROP_AUTO_EXPOSURE=0` 在这台相机上**写不进去** —— 写完读回仍是 `AE=3`。
   检出率与帧间抖动和"不碰"没有差别（差异全在 n=40 的噪声内）。锁曝光是**无操作**。
2. 锁焦是**负收益**：`FOCUS=275` 把 ID20 的检出率打到 **7%**、ID23 打到 **20%**；
   而 `FOCUS=120/240`（100% 检出）的帧间抖动比自动对焦更大（R49 的四配置表）。
   这台相机的确定性来自它自己的自动对焦，**不来自把镜头钉死**。
3. 独立的一条坑：`AUTOFOCUS=1` 时再写 `CAP_PROP_FOCUS` 会**扰动镜头** ——
   同一场景下解出的 y 差 1.8 mm（R49 第 2 条）。所以"顺手写一下 FOCUS"从来不是无操作。

于是 `apply_locked_settings()` **只设采集格式**（FOURCC/宽/高，与
`verify_vision.grab_frame` 逐字一致 —— 采集格式换了，标定内参就不是同一个成像条件），
`CAMERA_DEV` / `LOCKED_EXPOSURE` / `LOCKED_FOCUS` 只是历史快照的**记录**，本项目不应用。
掉码（17–28%）是自动对焦的正常代价：由上层"每位置重拍"消化，不是把相机锁死。
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from vision_core import calib_store

# 相机格式、预热帧数、JPEG 质量**从 verify_vision import，不复制**（与 live_field.py
# 同一条纪律）：复制的那一刻就是下一次"两边参数漂移、只有一边对"的起点。
# verify_vision 的顶层只有常量与函数定义（`__main__` 才跑 CLI），import 它不开相机、不写盘。
from verify_vision import (  # noqa: E402
    CAPTURE_FOURCC,
    CAPTURE_HEIGHT,
    CAPTURE_WIDTH,
    CAMERA_DEVICE,
    DEFAULT_SETTLE_FRAMES,
    JPEG_QUALITY,
)

# 设备路径。`CAMERA_DEV` 是 Task 8 接口里的名字，值从 verify_vision 的常量取（单一来源）。
CAMERA_DEV = CAMERA_DEVICE

# ⚠ `视觉测量/结果/camera_props.json`（2026-10-12 的快照）里的历史值：
# **本项目明确不应用它们**，理由见模块 docstring 与 SDD 台账 R49/R53。
# 保留它们是因为：① "为什么不锁"这件事在代码里要看得见；② 将来换相机重测时有个参照点。
# `apply_locked_settings` 绝不写 CAP_PROP_FOCUS / CAP_PROP_AUTOFOCUS /
# CAP_PROP_AUTO_EXPOSURE / CAP_PROP_EXPOSURE 这四个属性。
LOCKED_EXPOSURE = 166
LOCKED_FOCUS = 275


class CameraUnavailable(RuntimeError):
    """相机打不开，或已打开的会话中途读帧失败、无法继续。

    是 `RuntimeError` 的子类，Task 8 接口里 `.grab() 失败抛 RuntimeError` 由它满足；
    调用方想区分"相机的问题"与"别的问题"时按这个名字 catch。
    **只**变成一句人话，不 traceback（与 `live_field.CameraUnavailable` 同一口径）。
    """


def apply_locked_settings(cap) -> None:
    """设采集格式：FOURCC + 宽 + 高。**只写这三个属性。**

    名字里的 "locked settings" 是历史叫法（计划原文要"锁死相机参数"）；实测之后
    真正被"钉死"的只有采集格式。曝光/对焦**一概不写**：`AUTO_EXPOSURE=0` 在这台
    相机上写不进去（无操作），而写 `FOCUS` 会把检出率打到 7%/20% 且会扰动镜头
    —— 见模块 docstring 与 R49/R53。

    与 `verify_vision.grab_frame` 里的三行逐字一致：格式换了，
    标定内参就不是同一个成像条件，之后每个位姿都带一个看不出来的系统误差。
    """
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAPTURE_FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)


def open_capture(device: str = CAMERA_DEV):
    """打开一根**常驻**的 capture handle 并设好采集格式；打不开抛 `CameraUnavailable`。

    这是本模块**唯一**调用 `cv2.VideoCapture` 的地方，也是测试替换相机的接缝
    （`monkeypatch.setattr(vgt.camera, "open_capture", fake)`）。

    不在这里重试：`verify_vision.grab_frame` 的重试是"每次拍照都换新句柄"的配套做法；
    常驻会话的打开是**一次**事件，失败就该带着原因退出（多半是设备被占用/路径错/没权限），
    重试只会让同一句话晚 2 秒出现。上层要重试就自己再来一次。
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()               # 拿到句柄但没打开成功时也要放掉，否则设备被占住
        raise CameraUnavailable(
            f"相机 {device} 打不开：多半是被别的程序占着（浏览器/会议软件/v4l2-ctl/"
            f"live_field.py），也可能是设备路径不对或当前用户没有读权限。"
            "关掉占用它的程序再开一次；本类不会 traceback 糊你一脸。"
        )
    apply_locked_settings(cap)
    return cap


def settle(cap, frames: int = DEFAULT_SETTLE_FRAMES) -> None:
    """丢 `frames` 个缓冲帧，等自动对焦收敛（R39，帧数取 `verify_vision` 的默认值）。

    只在**刚打开**设备之后跑一次：USB 相机的头几帧往往是上一次推流残留在驱动里的画面，
    自动对焦也要时间收敛；不等的话第一批 `grab()` 会拿到糊的/旧的帧，码检不出来，
    而调用方看到的现象是"相机坏了"。持续会话里不再每帧丢帧 —— 那正是常驻句柄的意义。
    """
    for _ in range(max(0, int(frames))):
        cap.grab()


class CameraSession:
    """持久相机会话：上下文管理器，`__enter__` 开一次，会话内反复 `grab()`/`save()`。

        with CameraSession() as cam:
            frame = cam.grab()

    ## 生命周期

    - `__enter__`：开设备 → `apply_locked_settings()` → 丢 `settle_frames` 个缓冲帧。
      开设备**只发生在这里**：构造 `CameraSession()` 与 `grab()`/`save()` 都不会
      偷偷打开相机（导入一个类不该有副作用，更不该悄悄点亮设备）。
    - `__exit__`：无论正常退出还是中途抛异常都 `release()`。漏掉它 `/dev/video0`
      会被占住，下一个工具（`live_field.py` / `verify_vision.py`）直接打不开。
    - 没进上下文（或已退出）时 `grab()`/`save()` 立刻抛 `CameraUnavailable`。

    ## 读帧失败不重试

    `grab()` 读不到就抛：设备热插拔/被抢走之后，**同一根句柄上的 `read()` 会一直失败**
    （R39），重试没有意义。上层要么重开一个会话，要么带着这个异常结束本 trial。
    """

    def __init__(self, device: str = CAMERA_DEV,
                 settle_frames: int = DEFAULT_SETTLE_FRAMES):
        self.device = str(device)
        self.settle_frames = max(0, int(settle_frames))
        self.cap = None

    @property
    def is_open(self) -> bool:
        """会话是否持有打开的句柄。"""
        return self.cap is not None

    def __enter__(self) -> "CameraSession":
        if self.cap is not None:
            raise CameraUnavailable(
                "这个 CameraSession 已经打开了：不要重复 __enter__（会丢掉第一根句柄，"
                "设备被自己占住）。要重开就先 close()。")
        self.cap = open_capture(self.device)
        try:
            settle(self.cap, self.settle_frames)
        except Exception:
            # 丢帧途中炸了也要把设备放掉，否则这个会话对象带着一根坏句柄活着。
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close()
        return False                # 不吞异常：__exit__ 只负责放掉设备

    def close(self) -> None:
        """释放句柄，幂等。**不释放 `/dev/video0` 就一直被这个进程占着。**"""
        if self.cap is not None:
            cap, self.cap = self.cap, None
            cap.release()

    def _require_open(self):
        if self.cap is None:
            raise CameraUnavailable(
                "会话没打开（还没 __enter__，或者已经 __exit__）。打开设备只发生在 "
                "__enter__，不会在 grab()/save() 里悄悄发生。")
        return self.cap

    def grab(self) -> np.ndarray:
        """读**一帧新鲜**的 BGR 图（`cv2.VideoCapture.read`）；失败抛 `CameraUnavailable`。"""
        cap = self._require_open()
        ok, frame = cap.read()
        if not ok or frame is None:
            raise CameraUnavailable(
                f"读帧失败：相机 {self.device} 中途断开了，或者被别的程序抢走了。"
                "同一根句柄上重试没有意义（R39）——重开一个 CameraSession，"
                "或者把这一轮 trial 标废。")
        return frame

    def save(self, path: str | Path) -> str:
        """抓一帧、存盘，返回**实际落盘的路径**（字符串）。

        相对路径按**仓库根**解析（R3，`calib_store.resolve_path` 是唯一来源），
        父目录自动创建：Task 10–12 的 trial 产物可以写
        `视觉测量/结果/captures/…` 这种仓库相对路径，而不管当时 cwd 在哪。

        写盘失败抛 `RuntimeError`。`cv2.imwrite` 对不认识的扩展名是**抛** `cv2.error`
        而不是返回 False（`live_field.py` 踩过），两条都归到同一句话里。
        """
        frame = self.grab()                     # 先抓帧再建目录：抓不到就不留空目录
        target = calib_store.resolve_path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            ok = cv2.imwrite(str(target), frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        except cv2.error as exc:
            raise RuntimeError(f"存图失败（{target}）：{exc}") from exc
        if not ok:
            raise RuntimeError(
                f"存图失败（{target}）：cv2.imwrite 返回 False（磁盘满 / 权限不足 / "
                "扩展名不认识都是这个表现）。")
        return str(target)
