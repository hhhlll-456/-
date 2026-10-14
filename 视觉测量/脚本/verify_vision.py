#!/usr/bin/env python3
"""A1–A4 一次性人工标定的命令行工具：只读求解照片 + 把人工量到的数写回标定文件。

    python3 视觉测量/脚本/verify_vision.py 视觉测量/照片/photo_review_20261013.jpg
    python3 视觉测量/脚本/verify_vision.py --capture --capture \
        --truth=-80,-40,0 --truth=60,30,90 --truth=0,100,180 --write-validation

**这个工具不测量任何东西。** A1（量 ID29 的偏置）、A3（塞尺复核共面）、A4（电池压降）
是人拿卷尺/塞尺/万用表量的。工具只做两件事：

1. 把照片解成车体在场地系里的 `(x_mm, y_mm, yaw_deg)` 并打印（计划的 Step 2）
2. 把人量到的数写进 `视觉测量/结果/car_calib.json`，**用 `deep_merge`**

没有写标志（`--bias-*` / `--battery-*` / `--coplanar` / `--write-validation`）时是**只读
模式**：求解、打印，一个字节都不落盘。

## 输出的三个数是什么

- `x/y`：车体原点（**四轮轴心矩形的几何中心**，不是码的中心）在场地系里的位置，mm。
- `yaw`：**零基准是场地 +X（ID20 TL → ID21 TR），从上往下看逆时针为正**，归一到
  (−180, 180]。**不是图像 up**（裁决 R34）：本场地系里图像 up 位于 **+92.79°**。
  拿这个数去和计划原文的"图像 up = 0°"比较时**加上** +92.79°，不是去"纠正"它；
  也不要去和某个目标角相减 —— 这里出的是绝对朝向，误差永远是 `solved − truth`。
- `reproj_rms`：车载码四个角点的重投影 RMS，px。它是**模型自检**，不是位置精度。

## 裁决落在代码里的地方

- **R34** 零点：`yaw` 是绝对朝向，零基准 = 场地 +X。表头每次都印这条。
- **R35** `bias` 的参考系：`dx_mm/dy_mm` 是沿**打印码 ID29 自身的规范轴**量的
  （+X = 角点0→角点1，+Y = 角点3→角点0 = 码正面的上轴），**不是车体轴**。码相对车体
  转了约 90°，所以车体"左 b mm"要填成 `--bias-dx ±b --bias-dy 0 --bias-dtheta ±90`。
  `--bias-*` 的 help 里抄了这条换算：操作者马上就要拿卷尺量车，而这里正是 90° 会静默
  进错数字、且之后每条实验的 x/y 都带同一个系统偏差的地方。
- **R37** 偏置还没量：标定文件不存在（或 `bias` 三项都是 0.0）时，**表头醒目指出来**，
  并说明这一轮解出来的 x/y 是"码的中心"而不是"车体原点"。`dy = 55.0` 只出现在
  `vision_core/field.py` 的测试夹具里，**不是测量结果**。
- **R40** 写盘一律 `deep_merge`。`{**a, **b}` 会整段换掉 `bias`：`--bias-dy 77` 存下去
  之后，原有的 `dx_mm`/`dtheta_deg` 消失，而 `load_calib` 又拿默认值补齐 —— 量好的
  `dy_mm=55` 读回来是 `0.0`，全程无报错。这是 Task 1 真出现过的缺陷，不是假想。
- **R41** `--coplanar yes|no` / `--battery-sag-v` 保留，但明说它们**不解决什么**：
  `coplanar` 只是把 A3 的人工复核结论记下来，解决 `v1_values.json`（"是"）与 xlsx
  （"否"）的记录冲突；`battery.sag_v` 是 D5（欠压先查硬件）的输入。**`battery.verdict`
  只能由人给**（`--battery-verdict`），本工具任何情况下都不会从 `sag_v` 推它。

## 退出码

    0  全部照片解出来了；做了 A2 验证的话，判据也满足
    1  至少一张照片没解出来（缺边界码 / 缺车载码 / 文件读不了 / 采集失败）
    2  命令行用法错误（argparse 自己也是 2）
    3  A2 验证做了，但**没达标**：RMS 超限，或者样本数 < 3

单张图解不出来**不中断整轮**：打印一行 `photo=... ok=False reason=...`，继续下一张，
最后再以 1 退出。

## 已知不归本工具管的事

- 边界码被撞歪了几毫米。`boundary_moved` 的意思是"这张图的场地系建不起来"，
  不是"本工具判过边界码位移"。
- `coplanar` 的塞尺读数、`battery` 的压降读数：`yes`/`否` 和电压值都得人给。
- 相机装夹与畸变重标定：那是 `calib_distortion.py` 的事。
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# `python3 视觉测量/脚本/verify_vision.py` 是**从仓库根**跑的，脚本目录不在 sys.path 上。
# 这里补上自己所在的目录，于是调用方既不需要设 PYTHONPATH，也不需要 cd 到脚本目录；
# 路径由 `__file__` 推出来，不是写死的绝对路径。
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from vision_core import calib_store  # noqa: E402
from vision_core.field import body_pose_in_field, build_field_frame  # noqa: E402
from vision_core.markers import detect_markers  # noqa: E402

# --- 常量 -----------------------------------------------------------------------

CAMERA_DEVICE = "/dev/video0"
CAPTURE_WIDTH = 1920
CAPTURE_HEIGHT = 1080
CAPTURE_FOURCC = "MJPG"
# 60 帧自动对焦预热，与 `capture_photo.py` 的默认值一致 —— 这台 USB 相机不预热到 AF
# 收敛时拍出来的图是糊的，码解不出来。焦点已经用 `capture_photo.py --focus 275`
# 钉死时可以调小。
DEFAULT_SETTLE_FRAMES = 60
# 重试要**换一根全新的 capture handle**：设备被占用/热插拔之后，同一个句柄上
# `read()` 会一直返回 None，在原句柄上重试没有意义。
CAPTURE_OPEN_RETRIES = 3
CAPTURE_RETRY_DELAY_S = 0.7

DEFAULT_CAPTURE_DIR = "视觉测量/结果/captures"
DEFAULT_CAPTURE_PREFIX = "shot"
JPEG_QUALITY = 95

# A2 的通过判据（计划 Step 4 明写）：残差 RMS ≤ 3 mm 且 ≤ 1.5°，且至少 3 个位置。
# "至少 3 个位置"和两个 RMS 同属判据：2 个点的残差 RMS 不是证据。
A2_GATE_RMS_MM = 3.0
A2_GATE_RMS_DEG = 1.5
A2_GATE_MIN_SAMPLES = 3

# R34 的实测常数（推导见 `vision_core/field.py` 模块 docstring）。表头印它，是为了让
# 任何把本工具输出和计划原文"图像 up = 0°"比较的人知道该加多少。
IMAGE_UP_IN_FIELD_DEG = 92.79

EXIT_OK = 0
EXIT_PHOTO_FAILED = 1
EXIT_USAGE = 2          # 与 argparse 的 UsageError 同值：本工具的用法错误也走它
EXIT_GATE = 3

# 失败原因词表。前两个沿用计划 Task 11 的 `ok=False, reason ∈ {...}` 原词，免得同一个
# 项目里两张嘴说同一件事。后三个是**输入层**失败（文件/相机），连解算都没开始，与
# "解算失败"不是一回事，单列出来不污染 Task 11 的词表。
REASON_BOUNDARY_MOVED = "boundary_moved"
REASON_NO_VEHICLE = "no_vehicle_marker"
REASON_IMAGE_MISSING = "image_missing"
REASON_IMAGE_UNREADABLE = "image_unreadable"
REASON_CAPTURE_FAILED = "capture_failed"

BIAS_AXES_HINT = ("bias 参考系 = 打印码 ID29 的规范轴（+X = 角点0→角点1，"
                  "+Y = 角点3→角点0 = 码正面上轴），不是车体轴")


class CaptureError(RuntimeError):
    """打开不了相机，或重试若干次仍读不出帧。"""


@dataclass
class PhotoResult:
    """一张照片的解算结果（给了 `--truth` 时还带残差）。"""

    label: str                        # 打印用的名字（文件名）
    origin: str                       # 命令行给的原始串 / 采集时保存的路径
    ok: bool
    reason: str | None = None         # ok=False 时的原因词表项
    detail: str = ""                  # 给人看的一句话
    x_mm: float | None = None
    y_mm: float | None = None
    yaw_deg: float | None = None
    reproj_rms_px: float | None = None
    truth: tuple[float, float, float] | None = None
    err_xy_mm: tuple[float, float] | None = None
    err_mm: float | None = None       # 位置残差的模，mm
    err_deg: float | None = None      # 航向残差，deg（已归一到 (−180, 180]）


# --- 数值与参数解析 -------------------------------------------------------------


def wrap180(deg: float) -> float:
    """归一到 (−180, 180]。与 `vision_core.field._wrap180` 同一约定。"""
    wrapped = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def signed(value: float, digits: int = 1) -> str:
    """带符号定点数（`+12.3` / `-8.7`），让输出既能 grep 又能肉眼读。

    `+ 0.0` 顺手把 `-0.0` 变成 `+0.0`：`-0.0` 打印出来是 `-0.0`，人看着像"负零"，
    还得想一下是不是哪里减反了。
    """
    return f"{float(value) + 0.0:+.{digits}f}"


def parse_truth(text: str) -> tuple[float, float, float]:
    """`--truth X,Y,YAW_DEG` → `(x_mm, y_mm, yaw_deg)`。形态不对一律报用法错误。"""
    parts = [p.strip() for p in str(text).split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"--truth 要 3 个逗号分隔的数 X,Y,YAW_DEG（场地系 mm 与 deg），实际 {text!r}"
        )
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        raise argparse.ArgumentTypeError(f"--truth 的三项都必须是数字，实际 {text!r}") from None
    if not all(math.isfinite(v) for v in vals):
        raise argparse.ArgumentTypeError(f"--truth 不能有 NaN/Inf，实际 {text!r}")
    return (vals[0], vals[1], wrap180(vals[2]))


def normalise_truth_argv(argv: list[str]) -> list[str]:
    """把 `--truth -80,-40,0` 预先改写成 `--truth=-80,-40,0`。

    argparse 只把**纯**负数（`-80`）当值；`-80,-40,0` 带逗号，正则不匹配，会被当成
    位置参数，于是 `--truth` 报"缺参数"、用户还以为那是照片名。在这里拼成 `=` 形式，
    两种写法都不会踩坑。
    """
    out: list[str] = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok == "--truth" and i + 1 < len(argv):
            out.append(f"--truth={argv[i + 1]}")
            i += 2
            continue
        out.append(tok)
        i += 1
    return out


def repo_rel(path: Path) -> str:
    """能相对仓库根表示就打回相对串（打印 / 落进 JSON 用），否则原样绝对路径。"""
    try:
        return str(Path(path).resolve().relative_to(calib_store.REPO_ROOT))
    except ValueError:
        return str(path)


# --- 解算 -----------------------------------------------------------------------


def load_image(arg: str) -> tuple[np.ndarray | None, PhotoResult]:
    """按仓库根解析路径再读图（相对路径与 cwd 无关，R3）。读不到给一个失败的 result。"""
    path = calib_store.resolve_path(arg)
    label = Path(arg).name
    if not path.is_file():
        return None, PhotoResult(
            label=label, origin=str(arg), ok=False,
            reason=REASON_IMAGE_MISSING,
            detail=f"文件不存在：{repo_rel(path)}（相对路径按仓库根解析，与 cwd 无关）",
        )
    img = cv2.imread(str(path))
    if img is None:
        return None, PhotoResult(
            label=label, origin=str(arg), ok=False,
            reason=REASON_IMAGE_UNREADABLE,
            detail=f"OpenCV 读不出这张图（可能不是图像文件或已损坏）：{repo_rel(path)}",
        )
    return img, PhotoResult(label=label, origin=str(arg), ok=True)


def solve_image(img: np.ndarray, calib: dict, label: str, origin: str) -> PhotoResult:
    """一帧图 → 车体在场地系里的 `(x_mm, y_mm, yaw_deg)`。

    失败分两类，用 Task 11 的 `ok=False, reason ∈ {...}` 词表：

    - `boundary_moved`：4 个边界码缺任一（少一个码的场地系是**错的**且不报错），
      或者四点退化到场地平面无定义。
    - `no_vehicle_marker`：场地图，但没看到车载码 ID29（车不在画面里 / 被挡住 /
      码掉了 / 位姿退化）。
    """
    result = PhotoResult(label=label, origin=origin, ok=False)
    poses = detect_markers(img, calib)

    missing = [mid for mid in calib_store.BOUNDARY_IDS if mid not in poses]
    if missing:
        result.reason = REASON_BOUNDARY_MOVED
        result.detail = (f"缺边界码 {missing}（需要 {list(calib_store.BOUNDARY_IDS)} 四个）。"
                         "少一个码的场地系是错的而且不报错，必须重拍，不能凑合。")
        return result

    try:
        ff = build_field_frame(poses)
    except ValueError as exc:
        result.reason = REASON_BOUNDARY_MOVED
        result.detail = f"4 个边界码建不出场地系：{exc}"
        return result

    vehicle_id = int(calib_store.VEHICLE_ID)
    if vehicle_id not in poses:
        result.reason = REASON_NO_VEHICLE
        result.detail = (f"场地图但没看到车载码 ID{vehicle_id}（本帧检出 {sorted(poses)}）。"
                         "车不在画面里/被挡住，或者码掉了。")
        return result

    pose = poses[vehicle_id]
    try:
        x_mm, y_mm, yaw_deg = body_pose_in_field(ff, pose, calib["bias"])
    except ValueError as exc:
        result.reason = REASON_NO_VEHICLE
        result.detail = f"车载码位姿不可用：{exc}"
        return result

    result.ok = True
    result.x_mm, result.y_mm, result.yaw_deg = x_mm, y_mm, yaw_deg
    result.reproj_rms_px = pose.reproj_rms_px
    return result


# --- 采集（R39）----------------------------------------------------------------


def grab_frame(device: str = CAMERA_DEVICE,
               settle_frames: int = DEFAULT_SETTLE_FRAMES,
               retries: int = CAPTURE_OPEN_RETRIES) -> np.ndarray:
    """从相机抓一帧**新鲜的**图。

    三件事都是 R39 要求的，每一件都在防同一件事："把上一个位置的旧帧当成这次的测量"。
    A2 的流程是"摆好车 → 告诉 PC → PC 拍照"，两帧之间车已经动了，拿到旧帧不会报错，
    只会算出一个看起来合理的错数。

    1. `grab()` 丢掉 `settle_frames` 帧：USB 相机有自己的缓冲，第一批帧往往是上一次
       推流残留在驱动里的画面；自动对焦也要时间收敛。
    2. 失败**换一根全新的 capture handle** 重试（见 CAPTURE_OPEN_RETRIES 的注释）。
    3. 格式/分辨率与 `capture_photo.py` 一致（MJPG 1920x1080），否则拿到的图和标定
       内参不是同一个成像条件。

    实在拿不到就抛 `CaptureError`（由调用方变成一行的 `ok=False`，不是 traceback）。
    """
    last = ""
    for attempt in range(1, max(1, retries) + 1):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                last = f"{device} 打不开（被别的程序占着？）"
            else:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAPTURE_FOURCC))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
                for _ in range(max(0, settle_frames)):
                    cap.grab()          # 只推进不解码：清缓冲 + 等 AF/曝光收敛
                ok, frame = cap.read()
                if ok and frame is not None:
                    return frame
                last = f"{device} 第 {attempt} 次尝试读帧返回 None"
        finally:
            cap.release()
        if attempt < max(1, retries):
            time.sleep(CAPTURE_RETRY_DELAY_S)
    raise CaptureError(
        f"采集 {max(1, retries)} 次都没拿到帧（最后一次：{last}）。"
        "确认设备路径、关掉占用相机的程序（浏览器/会议软件/v4l2-ctl）、"
        "以及当前用户对设备节点有读权限。"
    )


def _unique_path(out_dir: Path, stem: str) -> Path:
    """同目录同名不覆盖：采集照片是 A2 的证据，被下一轮悄悄盖掉就没法复现了。"""
    candidate = out_dir / f"{stem}.jpg"
    n = 2
    while candidate.exists():
        candidate = out_dir / f"{stem}_{n}.jpg"
        n += 1
    return candidate


def capture_shots(count: int, args, stamp: str):
    """连拍 `count` 张：每张先采集、写盘，再交给解算。每张独立采样、独立重试。

    产出 `(result, frame_or_None)`；失败时 result.ok=False、frame=None。
    """
    out_dir = calib_store.resolve_path(args.capture_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for i in range(1, count + 1):
        stem = f"{args.capture_prefix}_{i:02d}_{stamp}"
        rel = repo_rel(out_dir / f"{stem}.jpg")
        try:
            frame = grab_frame(args.device, args.settle_frames)
        except CaptureError as exc:
            yield (PhotoResult(label=f"{stem}.jpg", origin=rel, ok=False,
                               reason=REASON_CAPTURE_FAILED, detail=str(exc)), None)
            continue
        path = _unique_path(out_dir, stem)
        if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]):
            yield (PhotoResult(label=path.name, origin=repo_rel(path), ok=False,
                               reason=REASON_CAPTURE_FAILED,
                               detail=f"采集到了帧但写盘失败：{repo_rel(path)}"), None)
            continue
        rel = repo_rel(path)
        # 保存路径打在结果行**前面**：结果行的形状是计划的 Step 2 定下的契约，不往里塞
        # 额外字段；但 A2 的每一轮都必须能指回它拍的是哪张图（R39）。
        print(f"[{i}/{count}] saved={rel}  {frame.shape[1]}x{frame.shape[0]}  "
              f"settle_frames={args.settle_frames}  device={args.device}")
        yield (PhotoResult(label=path.name, origin=rel, ok=True), frame)


def iter_shots(args):
    """按命令行的取图顺序产出 `(result, frame_or_None)`：先是采集，再是给定照片。

    `--capture` 与照片路径互斥（见 `validate_args`），所以"第 N 个 `--truth`"永远
    只可能对应一种东西：采集顺序或命令行顺序。
    """
    if args.capture:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        yield from capture_shots(args.capture, args, stamp)
    for arg in args.photos:
        img, res = load_image(arg)
        yield (res, img)


# --- 打印 -----------------------------------------------------------------------

_LINE = "{label}  x={x}mm  y={y}mm  yaw={yaw}deg  reproj_rms={rms:.2f}px"


def format_line(res: PhotoResult) -> str:
    """一行结果，机器可 grep —— 计划的 Step 2 要的就是这个形状：

        photo_review_20261013.jpg  x=-91.1mm  y=-21.2mm  yaw=+179.5deg  reproj_rms=0.19px
        photo=foo.jpg  ok=False  reason=boundary_moved  缺边界码 [21] ...

    残差**不**塞进这一行（`format_truth_line` 另起一行）：这一行的形状是别的任务
    要 grep 的契约，多余字段会让按空格切列的下游解析错位。
    """
    if not res.ok:
        return f"photo={res.label}  ok=False  reason={res.reason}  {res.detail}"
    return _LINE.format(label=res.label, x=signed(res.x_mm), y=signed(res.y_mm),
                        yaw=signed(res.yaw_deg), rms=res.reproj_rms_px)


def format_truth_line(res: PhotoResult) -> str:
    """一行残差（给了 `--truth` 时才有）：

        truth  foo.jpg  truth=(-92.1,-21.2,+179.0)  err_xy=(+1.0,-0.0)mm  err=1.00mm  dyaw=+0.5deg
    """
    if res.truth is None or not res.ok:
        return ""
    return (f"truth  {res.label}  truth=({signed(res.truth[0])},{signed(res.truth[1])},"
            f"{signed(res.truth[2])})  err_xy=({signed(res.err_xy_mm[0])},"
            f"{signed(res.err_xy_mm[1])})mm  err={res.err_mm:.2f}mm  "
            f"dyaw={signed(res.err_deg)}deg")


def bias_status(calib: dict, calib_path: Path) -> tuple[str, bool]:
    """偏置状态文字 + 是否已测。R37：没量过就不能拿 0.0 冒充测量值。"""
    vals = (float(calib["bias"]["dx_mm"]), float(calib["bias"]["dy_mm"]),
            float(calib["bias"]["dtheta_deg"]))
    if any(abs(v) > 0.0 for v in vals):
        return (f"A1 偏置已有记录：dx_mm={signed(vals[0])} dy_mm={signed(vals[1])} "
                f"dtheta_deg={signed(vals[2])}（沿打印码规范轴）", True)
    if calib_path.is_file():
        return ("A1 偏置在标定文件里记的是三项 0.0：这要么是\"量到码正好在几何中心\"，"
                "要么是 A1 根本还没做（R37）。下面解出的 x/y 是不是车体原点就取决于这个，"
                "用之前先确认 A1 做了。", False)
    return ("A1 偏置**尚未测量**：标定文件不存在，bias 三项是内建默认 0.0。"
            "下面解出的 x/y 是**车载码的中心**，不是车体原点 —— 差一个未知的偏置，"
            "而且不报错。做完 A1（--bias-dx/--bias-dy/--bias-dtheta）再拿这些数做位移。",
            False)


def print_header(calib: dict, calib_path: Path) -> None:
    """表头：零点约定 + bias 状态。每次都印，因为这些正是最贵的"看不出来的错"。"""
    print(f"标定文件：{repo_rel(calib_path)}"
          f"{'' if calib_path.is_file() else '（不存在，用内建默认值）'}")
    print("  yaw 零基准 = 场地 +X（ID20 TL → ID21 TR），从上往下看逆时针为正（R34）。")
    print(f"    图像 up 在本场地系里位于 +{IMAGE_UP_IN_FIELD_DEG}°，不是 0°："
          "要和计划原文\"图像 up = 0°\"比较请加上这个常数，不要去\"纠正\"它。")
    print(f"  {BIAS_AXES_HINT}（R35）。")
    text, measured = bias_status(calib, calib_path)
    print(f"{'  ' if measured else '!! '}{text}")


# --- 验证（A2）-----------------------------------------------------------------


def compute_residuals(results: list[PhotoResult]) -> None:
    """就地算 `err = solved − truth`。有照片没解出来就抛错：残差样本不完整。"""
    failed = [r.label for r in results if not r.ok]
    if failed:
        raise ValueError("有照片没解出来，残差样本不完整：" + "、".join(failed))
    for r in results:
        ex = r.x_mm - r.truth[0]
        ey = r.y_mm - r.truth[1]
        r.err_xy_mm = (ex, ey)
        r.err_mm = math.hypot(ex, ey)
        r.err_deg = wrap180(r.yaw_deg - r.truth[2])


def validation_summary(results: list[PhotoResult]) -> dict:
    """`residual_rms_mm` = sqrt(mean(|err_i|²))，`residual_rms_deg` = sqrt(mean(dyaw_i²))。

    `n_samples` 一定等于照片张数（`--truth` 要么全给要么不给），它要回答的是
    "这个 RMS 是几个点算出来的"。2 个点的残差 RMS 不是证据，所以它必须看得见、
    也必须落进标定文件。
    """
    rms_mm = math.sqrt(sum(r.err_mm ** 2 for r in results) / len(results))
    rms_deg = math.sqrt(sum(r.err_deg ** 2 for r in results) / len(results))
    n = len(results)
    return {
        "residual_rms_mm": round(rms_mm, 4),
        "residual_rms_deg": round(rms_deg, 4),
        "n_samples": n,
        "gate_passed": (rms_mm <= A2_GATE_RMS_MM and rms_deg <= A2_GATE_RMS_DEG
                        and n >= A2_GATE_MIN_SAMPLES),
        "photos": [r.origin for r in results],
    }


def print_validation(summary: dict) -> None:
    n = summary["n_samples"]
    print(f"A2 尺子验证：n_samples={n}  residual_rms={summary['residual_rms_mm']:.2f}mm  "
          f"residual_yaw_rms={summary['residual_rms_deg']:.2f}deg")
    gate = f"RMS ≤ {A2_GATE_RMS_MM} mm 且 ≤ {A2_GATE_RMS_DEG}° 且 ≥ {A2_GATE_MIN_SAMPLES} 个位置"
    if summary["gate_passed"]:
        print(f"  判据：{gate} → A2 gate: PASS")
    elif n < A2_GATE_MIN_SAMPLES:
        print(f"  判据：{gate} → A2 gate: INSUFFICIENT"
              f"（{n} < {A2_GATE_MIN_SAMPLES} 个位置，这个 RMS 不能当证据；"
              "把位置加到 3 个以上重测）")
    else:
        over = []
        if summary["residual_rms_mm"] > A2_GATE_RMS_MM:
            over.append(f"位置 RMS {summary['residual_rms_mm']:.2f} > {A2_GATE_RMS_MM} mm")
        if summary["residual_rms_deg"] > A2_GATE_RMS_DEG:
            over.append(f"航向 RMS {summary['residual_rms_deg']:.2f} > {A2_GATE_RMS_DEG}°")
        print(f"  判据：{gate} → A2 gate: FAIL（{'；'.join(over)}）"
              "——先查 A1 偏置是不是量错（尤其 90° 装反），再考虑重拍棋盘格铺到画面四角收紧畸变。")


# --- 写标定（R40 / R41）---------------------------------------------------------


def build_updates(args) -> dict:
    """把命令行上的写标志收成 `deep_merge` 的 override 段。写哪些键只在这里出现。"""
    updates: dict = {}

    bias = {k: v for k, v in (("dx_mm", args.bias_dx),
                              ("dy_mm", args.bias_dy),
                              ("dtheta_deg", args.bias_dtheta))
            if v is not None}
    if bias:
        updates["bias"] = bias

    battery = {}
    if args.battery_sag_v is not None:
        battery["sag_v"] = args.battery_sag_v
    if args.battery_verdict is not None:
        battery["verdict"] = args.battery_verdict
    if battery:
        updates["battery"] = battery

    if args.coplanar is not None:
        updates["coplanar"] = args.coplanar == "yes"
    return updates


def apply_updates(calib: dict, updates: dict, path: Path) -> None:
    """**整份文件唯一的写盘点子。** `deep_merge`，不是 `{**calib, **updates}`（R40）。

    浅合并会把整段 `bias` 换掉：`--bias-dy 77` 存下去之后，原有的 `dx_mm`/`dtheta_deg`
    消失，而 `load_calib` 又拿内建默认值补齐 —— 量好的 `dy_mm=55` 读回来是 `0.0`，
    全程无报错。
    """
    calib_store.save_calib(calib_store.deep_merge(calib, updates), path)


def describe_updates(updates: dict) -> str:
    bits = []
    for section, value in updates.items():
        if isinstance(value, dict):
            inner = " ".join(f"{k}={round(v, 4) if isinstance(v, float) else v}"
                             for k, v in value.items())
            bits.append(f"{section}=[{inner}]")
        else:
            bits.append(f"{section}={value}")
    return "  ".join(bits) if bits else "（无）"


# --- 命令行 ---------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="verify_vision.py",
        # Raw**Text**（不是 RawDescription）：参数 help 里那两张换算表和判据是照着填的
        # 表格，默认的折行会把它们压成一坨。
        formatter_class=argparse.RawTextHelpFormatter,
        description="A1–A4 一次性人工标定工具：解照片 + 把人工量到的数写回标定文件。",
        epilog=(
            "例子：\n"
            "  只读看一张：\n"
            "    python3 视觉测量/脚本/verify_vision.py 视觉测量/照片/photo_review_20261013.jpg\n"
            "  A2 尺子验证（每摆一个位置拍一张，3 个以上）：\n"
            "    python3 视觉测量/脚本/verify_vision.py --capture --capture --capture \\\n"
            "        --truth=0,0,0 --truth=100,50,90 --truth=-80,120,180 --write-validation\n"
            "  注意：负坐标要写成 --truth=-80,120,180（带等号），否则会被当成照片名。\n"
        ),
    )
    p.add_argument("photos", nargs="*",
                   help="照片路径，相对路径按**仓库根**解析（与 cwd 无关）")

    g = p.add_argument_group("A1 车体偏置（沿打印码 ID29 的规范轴量，R35）")
    g.add_argument("--bias-dx", type=float, metavar="MM", help=(
        "A1：ID29 相对四轮轴心矩形中心、沿**码规范 +X**（角点0→角点1）的偏移，mm。\n"
        "当前贴法下码相对车体转了约 90°，所以车体方向的偏置要这样换算：\n"
        "    车体「左」 b mm → --bias-dx ±b  --bias-dy 0   --bias-dtheta ±90\n"
        "    车体「前」 f mm → --bias-dx 0   --bias-dy ±f  --bias-dtheta ±90\n"
        "正负号由码的印刷/安装朝向定，A1 时实测一次确认。按车体轴填会把「左 55 mm」\n"
        "当成「前后 55 mm」，之后每条实验的 x/y 都带 55 mm 的系统偏差，而且没有一处报错。\n"
        "三项可以只给一部分：没给的保留标定文件里的原值（deep_merge，不会被清零）。"))
    g.add_argument("--bias-dy", type=float, metavar="MM",
                   help="A1：沿码规范 +Y（角点3→角点0，码正面的上轴）的偏移，mm。换算见 --bias-dx。")
    g.add_argument("--bias-dtheta", type=float, metavar="DEG", help=(
        "A1：码相对车体的朝向偏置，deg。车体航向 = 码航向 − dtheta。\n"
        "当前贴法下这个数接近 ±90（码转了约 90°），不是 0；量成 0 会让每个 yaw 都差 90°。"))

    g = p.add_argument_group("A3 / A4")
    g.add_argument("--coplanar", choices=("yes", "no"), help=(
        "A3：四轮接地点是否共面（塞尺复核）的人工结论。\n"
        "它只是把结论记下来，解决 v1_values.json（\"是\"）与 xlsx（\"否\"）的记录冲突，\n"
        "**不做任何复核**，也不会去改别的标定量。"))
    g.add_argument("--battery-sag-v", type=float, metavar="V", help=(
        "A4：电池带载压降，V（人用万用表量的）。它是决策 D5（欠压先查硬件）的输入。\n"
        "本工具**不**从它推导结论：battery.verdict 只能由人用 --battery-verdict 给。"))
    g.add_argument("--battery-verdict", metavar="TEXT", help=(
        "A4：人给的电池结论原文，例如\"接插件压降，接触正常\"。只能是人工结论 —— "
        "阈值判断是人的判断，工具不猜。"))

    g = p.add_argument_group("A2 尺子验证")
    g.add_argument("--truth", type=parse_truth, action="append", metavar="X,Y,YAW_DEG", help=(
        "该照片的**场地系真值**（尺子量出来的 mm 与 deg），可重复，**按位置**一一对应：\n"
        "第 1 个 --truth ↔ 第 1 张照片，第 2 个 ↔ 第 2 张，以此类推。\n"
        "要么给全部，要么一个都不给；只给一半是错误，不会静默跳过。\n"
        "负坐标写成 --truth=-80,120,180（带等号）。"))
    g.add_argument("--write-validation", action="store_true", help=(
        "把残差 RMS 与 n_samples 写进标定文件的 validation 段（需每张照片都有 --truth）。\n"
        f"A2 判据：RMS ≤ {A2_GATE_RMS_MM} mm 且 ≤ {A2_GATE_RMS_DEG}° 且 ≥ {A2_GATE_MIN_SAMPLES} 个位置。\n"
        "n_samples 会一并存下来 —— 2 个点的残差 RMS 不是证据。"))

    g = p.add_argument_group("采集")
    g.add_argument("--capture", action="count", default=0,
                   help="从相机抓一张并解算，可重复（--capture --capture = 2 张）")
    g.add_argument("--capture-count", type=int, default=0, metavar="N",
                   help="一次抓 N 张（等价于 --capture 重复 N 次）")
    g.add_argument("--device", default=CAMERA_DEVICE,
                   help=f"相机设备（默认 {CAMERA_DEVICE}）")
    g.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR,
                   help=f"采集照片保存目录，相对仓库根（默认 {DEFAULT_CAPTURE_DIR}）")
    g.add_argument("--capture-prefix", default=DEFAULT_CAPTURE_PREFIX,
                   help=f"采集文件名前缀（默认 {DEFAULT_CAPTURE_PREFIX}）")
    g.add_argument("--settle-frames", type=int, default=DEFAULT_SETTLE_FRAMES, metavar="N",
                   help=f"抓图前丢弃的缓冲帧数（默认 {DEFAULT_SETTLE_FRAMES}，等自动对焦收敛）")

    p.add_argument("--calib", default=calib_store.DEFAULT_CALIB_PATH, metavar="PATH",
                   help=f"标定文件，相对仓库根（默认 {calib_store.DEFAULT_CALIB_PATH}）")
    return p


def validate_args(parser: argparse.ArgumentParser, args) -> None:
    """用法层面的自检。只碰人一定会踩的几条，别的交给 argparse。"""
    args.capture = args.capture_count if args.capture_count else args.capture
    n_shots = len(args.photos) + args.capture
    if args.capture < 0:
        parser.error("--capture-count 不能为负")
    if args.capture and args.photos:
        parser.error("--capture 与照片路径不能混用：采集顺序就是 --truth 的对应顺序，"
                     "混进来就说不清第几个 --truth 配哪一张了")
    if n_shots == 0:
        parser.error("至少给一张照片路径，或者用 --capture 抓图")
    if args.write_validation and not args.truth:
        parser.error("--write-validation 需要每张照片都有 --truth（残差要拿真值比）")
    if args.truth and len(args.truth) != n_shots:
        parser.error(f"--truth 给了 {len(args.truth)} 个，照片/采集共 {n_shots} 张："
                     "必须按位置一一对应，要么全给要么全不给")
    if any(v is not None for v in (args.bias_dx, args.bias_dy, args.bias_dtheta)):
        print("提示：bias 沿**打印码**规范轴量，车体「左 b」→ --bias-dx ±b --bias-dy 0 "
              "--bias-dtheta ±90（R35）。")


def run(args) -> int:
    calib_path = calib_store.resolve_path(args.calib)
    try:
        calib = calib_store.load_calib(calib_path)
    except ValueError as exc:
        print(f"标定文件读不出来，先修它：{exc}")
        return EXIT_PHOTO_FAILED

    results: list[PhotoResult] = []
    print_header(calib, calib_path)
    for res, frame in iter_shots(args):
        if frame is not None:
            res = solve_image(frame, calib, res.label, res.origin)
        results.append(res)
        print(format_line(res))

    failures = [r for r in results if not r.ok]
    if failures:
        print(f"失败 {len(failures)}/{len(results)} 张：" +
              "、".join(f"{r.label}={r.reason}" for r in failures))
        print("单张失败不影响其余照片，但这一轮不能当测量结果用（退出码 1）。")

    summary = None
    if args.truth:
        for res, truth in zip(results, args.truth):
            res.truth = truth
        if failures:
            print("有照片失败，残差样本不完整：不计算、也不写入 validation。")
        else:
            compute_residuals(results)
            for res in results:        # 真值/残差单独一行：结果行的形状是下游的契约
                print(format_truth_line(res))
            summary = validation_summary(results)
            print_validation(summary)

    # --- 写盘（R40：整份文件只有这一个写盘点子）-----------------------------------
    updates = build_updates(args)
    if updates:
        apply_updates(calib, updates, calib_path)
        print(f"已写入 {repo_rel(calib_path)}（deep_merge）：{describe_updates(updates)}")
        if "bias" in updates:
            print("  bias 只在打印码规范轴上有意义（R35）；写完再跑一次只读，确认表头那行状态变了。")
        if "coplanar" in updates:
            print("  A3 只记录人工结论，不做复核；下游不会拿它改任何标定量。")
        if "battery" in updates and "sag_v" in updates["battery"] \
                and "verdict" not in updates["battery"]:
            print("  A4 的 battery.verdict 仍然为空：D5（欠压先查硬件）要人给结论，用 --battery-verdict 补。")
    if args.write_validation and summary is not None:
        apply_updates(calib, {"validation": summary}, calib_path)
        print(f"已写入 {repo_rel(calib_path)}（deep_merge）：{describe_updates({'validation': summary})}")
    if not updates and not (args.write_validation and summary is not None):
        print("只读模式：没有写任何文件（要写请给 "
              "--bias-* / --battery-* / --coplanar / --write-validation 之一）。")

    if failures:
        return EXIT_PHOTO_FAILED
    if summary is not None and not summary["gate_passed"]:
        return EXIT_GATE
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    argv = list(sys.argv[1:] if argv is None else argv)
    args = parser.parse_args(normalise_truth_argv(argv))
    validate_args(parser, args)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())