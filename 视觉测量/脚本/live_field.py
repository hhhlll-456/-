#!/usr/bin/env python3
"""A1–A4 人工标定的**实时取景预览**：操作者一边挪相机一边看真管道的判定。

    python3 视觉测量/脚本/live_field.py                 # 开窗口，实时叠加
    python3 视觉测量/脚本/live_field.py --once          # 只解算一帧，打印状态行后退出（不开窗）
    python3 视觉测量/脚本/live_field.py --frames 30     # 无窗口解 30 帧后退出（快速验证 / 压测）

## 这个窗口存在的理由

操作者今天卡在**取景**上（R48）：`verify_vision.py` 报 `boundary_moved`，但那个工具只给一行
文字，看不出"码在画面哪里、离边缘几像素"。仓库里已有的 `live_aruco.py` 能看画面，可是

1. **它的检测参数和真管道不一致** —— 它自己设了 `adaptiveThreshWinSize*` /
   `adaptiveThreshConstant`，而 `vision_core/markers.py:detect_markers` 只用默认参数 +
   `CORNER_REFINE_SUBPIX`。于是它可能显示"找到 4 个码"而 `verify_vision.py` 仍然失败，
   操作者白调半天。
2. **只显示检出，不显示"能不能用"** —— 不建场地系、不解车体位姿、不报 `reason=` 词表。

所以这里**逐帧调用 `verify_vision.solve_image`**：窗口里显示的 READY / 失败原因，与
`verify_vision.py` 对**同一帧**的判定逐字一致。相机参数（设备、分辨率、FourCC）也一律
`from verify_vision import`，不复制 —— 复制的那一刻就是下一次"预览能过、正式工具失败"的起点。

## 为什么每帧跑两遍检测（明知不是最省）

叠加层要画四边形/中心/边缘余量，这些来自 `detect_markers`；判定来自 `solve_image`，
它内部又调了一次 `detect_markers`。同一帧上两次检测是**确定性**的同一结果（实测 17.7 ms/帧），
这个前提由 `test_detecting_markers_twice_on_one_frame_gives_the_same_poses` 钉住
（它只证"检测是确定性的"；`run_window` 把这一份位姿**接到**叠加层上是另一条接线，
由 `test_run_window_snapshots_a_frame_that_solve_image_can_still_read` 那几条钉住）。
省掉那 17 ms 的唯一办法是让 `solve_image` 接收位姿，那要改 `verify_vision.py`，本任务禁止。

## 叠加内容（从下往上）

1. 每个码的四边形 + `ID=20 (400,70)` 中心像素。
2. **边界告警**（今天这个 bug 的正面处理）：四边形离画面边缘的最小余量
   `edge_margin_px` 小于 `EDGE_MARGIN_PX`（20）就红框 + `ID20 距上边缘 3px ⚠被切`。
3. 场地系坐标轴（+X = ID20→ID21、+Y = ID20→ID23）+ 100 mm 比例尺。仅在场地系建起来时画。
4. 车体原点十字（用 `calib["bias"]` 修正过），标出它解出来的 `(x, y)` mm。
5. 状态横幅：READY 绿字 / 失败红字 + `reason=` + `result.detail`；A1 偏置没量过时加黄字警告。
6. 终端行：**状态变化时**才打一行（`live_aruco.py` 的风格），不用盯着窗口也能读到码的中心坐标。
   另有一条 0.5 s 的冷却：边缘上抖动的码会让"变化"每帧都成立，没有它终端每秒十几行，
   而这条日志的目的恰恰是"随时 tail 一下就看到当下的状态"（见 `TerminalLog`）。

## 按键

    q / ESC  退出        s  存快照（原帧，可直接喂给 verify_vision.py）
    g        网格开关（每 240 px，与 live_aruco.py 一致）

**按 `s` 存的是未叠加的原帧**：叠加会把码的黑边糊掉（实测画完之后五个码一个都检不出来），
所以 `draw_overlay` 只作用在 `frame.copy()` 上，`frame` 在任何绘制路径下都保持干净。
存下来的图能直接 `verify_vision.py <路径>`，解出来的位姿与存之前那一帧逐项相同。

## 退出码

**两种模式的判据不同，是用法决定的，不是笔误**（详见 `run_window` 的 docstring）：

    0  会话跑完了，且整场**有过一帧** READY。无窗口模式（--once / --frames N）：
       **每一帧都 READY**；窗口模式：**至少有一帧 READY** —— 调取景途中掉几帧不算
       失败，但**整场一帧都没解出来算失败**（否则按 $? 判断会拿到假通过）。
    1  本轮没产出可用的解。三种来源，窗口模式与无窗口模式判据不同：
       · 有帧没解出来（取景/掉码，`no_vehicle_marker` / `boundary_moved`）。无窗口
         模式下**任何一帧没解出来就是 1**；窗口模式下则是**整场零 READY** 才算失败。
       · 中途读帧失败（相机被拔/被别的程序抢走）。**只发生在无窗口模式**：它是有界
         采样（`--frames N` 就是要 N 帧），读不到就是这一轮没跑完。窗口模式中途掉线
         不算失败 —— 已经解出来的帧是有效的测量，按上面 0 的判据走（有 READY 仍给 0）。
       · 标定文件读不出来（与 `verify_vision.py` 同一口径：同一个输入错误、同一类
         返回码）。这一条两种模式一样，因为那时相机还没开。
    2  命令行用法错误（与 argparse 同值）；**或**窗口打不开（没有显示环境 / 连不上
       显示服务器 —— 提示会分别给出 `export DISPLAY=:0` 与 X 转发的说法）。
    4  相机打不开 —— 与 1 **故意分开**：1 是"本轮没产出可用的解"（**含配置错误**，
       标定文件读不出来也是 1，那时相机还没开），4 是"**连相机都没打开**"。
       操作者今天在这两端之间来回猜（R48），分开之后一眼能分。

## 硬规矩

- 本文件**只读**标定、**只写** `--snapshot-dir`（默认在仓库外），不改任何标定量。
- 逐帧判定只走 `verify_vision.solve_image`；本文件**不出现** `cv2.aruco.detectMarkers`
  与 `DetectorParameters`，那些分叉由 `test_live_field_never_forks_the_detector` 钉死。
- 中文叠加文字走 PIL + 思源黑体；找不到字体时退回 `cv2.putText`（非 ASCII 打成 `?`），
  所以在没有 CJK 字体的机器上窗口仍然可用。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import NoReturn

# `python3 视觉测量/脚本/live_field.py` 是从**仓库根**跑的，脚本目录不在 sys.path 上。
# 这里补上自己所在的目录，路径由 `__file__` 推出来，不写死绝对路径、也不要求 PYTHONPATH
# （与 `verify_vision.py` 同一套做法，R3/R22）。
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from vision_core import calib_store  # noqa: E402
from vision_core.field import FieldFrame  # noqa: E402
from vision_core.markers import detect_markers  # noqa: E402

# 真管道的入口 + 相机参数**全部 import，不复制**。复制的那一刻就是下一次
# "预览能过、verify_vision.py 失败"的起点 —— 今天的活儿就是为了消灭那个分叉。
# `_unique_path` 是 `verify_vision` 的私有助手，但"同名不覆盖"是全仓库的纪律，
# 抄一份就会出现两份各自漂移的实现，所以连它一起复用。
from verify_vision import (  # noqa: E402
    CAPTURE_FOURCC,
    CAPTURE_HEIGHT,
    CAPTURE_WIDTH,
    CAMERA_DEVICE,
    DEFAULT_SETTLE_FRAMES,
    EXIT_OK,
    EXIT_PHOTO_FAILED,
    EXIT_USAGE,
    JPEG_QUALITY,
    _unique_path,
    bias_status,
    print_header,
    signed,
    solve_image,
)

# --- 常量 -----------------------------------------------------------------------

# 边界码四边形离画面边缘小于这个像素就告警。取 20 而不是 0：R48 里 ID20 被上边缘切掉
# 15 mm 时它离边缘只剩 3 px，而 ID21 当时离右边缘 20 px —— 20 px 是"码的亚像素角点细化
# 还能不能稳住"的量级，也正是 R48 给操作者的建议（黑边离边缘 ≥ 15 px 更稳）。
EDGE_MARGIN_PX = 20.0

DEFAULT_SCALE = 0.75
# 快照默认**不进仓库**：它是调取景的中间产物，不是证据。证据由 `verify_vision.py
# --capture` 写进 `视觉测量/结果/captures/`。
DEFAULT_SNAPSHOT_DIR = "/tmp/opencode/live_field"
GRID_STEP_PX = 240

# 场地坐标轴与比例尺都用这个长度（mm）。用**同一个**长度画 +X 轴和比例尺，于是两者
# 在画面上等长，比例尺那一行的 "≈ N px" 就是 +X 轴长度的自解释。
AXIS_LEN_MM = 100.0

# 终端状态行的最短间隔（秒）。见 `TerminalLog` 的 docstring：没有它，边缘上抖动的码
# 会让终端每秒刷十几行，而这条日志的目的恰恰是"不盯窗口也能读到当下的状态"。
TERMINAL_MIN_INTERVAL_S = 0.5

WINDOW_NAME = "live_field (q/ESC quit | g grid | s snapshot)"

TEXT_PX = 24           # 横幅
LABEL_PX = 20          # 码的 ID/中心标签
SMALL_PX = 18          # 告警行、比例尺

COLOR_OK = (0, 230, 0)               # 绿：READY
COLOR_BAD = (0, 70, 255)             # 红：失败 / 边缘告警
COLOR_WARN = (0, 200, 255)           # 黄：A1 未测
COLOR_WHITE = (255, 255, 255)
COLOR_AXIS_X = (255, 0, 255)         # 洋红：场地 +X（与黄/红都不撞色，方便一眼分辨）
COLOR_AXIS_Y = (255, 255, 0)         # 青：场地 +Y
COLOR_BODY = (0, 0, 255)             # 红：车体原点十字
COLOR_EDGE_BAD = (0, 70, 255)        # 红：贴边的码
COLOR_GRID = (0, 170, 0)             # 绿：网格

# 中文叠加需要的字体。逐个找，**不做死路径兜底**：找不到就退回 ASCII 渲染，
# 窗口仍然可用，只是中文变 `?`。PIL 同理，缺了也不该让整个脚本 import 失败。
FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
    "/home/hanli/.local/share/fonts/windows-fonts-extra/Noto Sans SC (TrueType).otf",
)

EXIT_CAMERA_UNAVAILABLE = 4    # 见模块 docstring：与 EXIT_PHOTO_FAILED 故意分开


class CameraUnavailable(RuntimeError):
    """相机打不开（被别的程序占着 / 设备路径错 / 没权限）。**只**变成一行提示，不 traceback。"""


# --- 文本渲染（PIL，有中文字体就用；没有就退回 cv2.putText）------------------------


_FONT_CACHE: dict = {}
_PIL = None
try:  # PIL 是可选依赖：没有它窗口仍然能开，只是中文变 `?`
    from PIL import Image as _PILImage, ImageDraw as _PILDraw, ImageFont as _PILFont
except ImportError:  # pragma: no cover - 本机有 PIL，走不到这条
    _PILImage = _PILDraw = _PILFont = None


def _font(px: int):
    """按像素大小取中文字体；找不到（或没有 PIL）返回 None，由 `put_text` 退回 cv2。"""
    if _PILFont is None:
        return None
    if px not in _FONT_CACHE:
        font = None
        for candidate in FONT_CANDIDATES:
            if not Path(candidate).is_file():
                continue
            try:
                font = _PILFont.truetype(candidate, px)
            except OSError:
                continue
            break
        _FONT_CACHE[px] = font
    return _FONT_CACHE[px]


def _ascii_only(text: str) -> str:
    """`cv2.putText` 只会画 Hershey 字形，中文会变成 `?`；显式换掉，别让人以为画面坏了。"""
    return "".join(ch if ord(ch) < 128 else "?" for ch in text)


def text_width(text: str, px: int) -> int:
    """文本像素宽度。量得到就量（字体真实 advance），量不到用保守估计。"""
    font = _font(px)
    if font is not None:
        try:
            return int(math.ceil(font.getlength(text)))
        except (AttributeError, OSError, ValueError):  # pragma: no cover
            pass
    return int(sum(1.0 if ord(ch) > 0x2E80 else 0.56 for ch in text) * px)


def _blend(img: np.ndarray, rgb: np.ndarray, alpha: np.ndarray, x: int, y: int) -> None:
    """把一块带 alpha 的 **RGB** 补丁贴到 `img`（**BGR** uint8）上，越界部分裁掉。

    越界裁剪而不是丢帧：场地原点在画面外时坐标轴仍然要画得出来（见 `clamp_px`），
    而文字标签一律贴着画面内侧写。RGB→BGR 的通道换在这一处做，别在调用点上各换各的 ——
    忘了换的话红字会变成蓝字、而画面上没有任何东西会提示这一点。
    """
    h, w = img.shape[:2]
    ph, pw = alpha.shape[:2]
    sx0, sy0 = max(0, -x), max(0, -y)
    dx0, dy0 = max(0, x), max(0, y)
    dx1, dy1 = min(w, x + pw), min(h, y + ph)
    if dx1 <= dx0 or dy1 <= dy0:
        return
    k = (slice(sy0, sy0 + dy1 - dy0), slice(sx0, sx0 + dx1 - dx0))
    patch = rgb[k][:, :, ::-1].astype(np.float32)       # RGB → BGR
    a = alpha[k][..., None]
    dst = img[dy0:dy1, dx0:dx1].astype(np.float32)
    img[dy0:dy1, dx0:dx1] = (patch * a + dst * (1.0 - a)).astype(np.uint8)


def put_text(img: np.ndarray, text: str, org, color, px: int = TEXT_PX) -> None:
    """在 `img` 上写一行文本，原点 `org` 是**左上角**。就地改 `img`。

    带 1 px 黑描边：实时预览的画面明暗是随机的，没有描边的文字在亮背景上会读不出来 ——
    而"读不出来"在这里等于操作者重新拍一遍照。
    """
    text = str(text)
    if not text:
        return
    x, y = int(round(org[0])), int(round(org[1]))
    font = _font(px)
    if font is None:
        cv2.putText(img, _ascii_only(text), (x, y + px),
                    cv2.FONT_HERSHEY_SIMPLEX, px / 32.0, color, 1, cv2.LINE_AA)
        return
    left, top, right, bottom = font.getbbox(text)
    pad = 3
    patch = _PILImage.new("RGBA", (right - left + 2 * pad, bottom - top + 2 * pad), (0, 0, 0, 0))
    _PILDraw.Draw(patch).text(
        (pad - left, pad - top), text, font=font,
        fill=(int(color[2]), int(color[1]), int(color[0]), 255),
        stroke_width=1, stroke_fill=(0, 0, 0, 255),
    )
    arr = np.asarray(patch)
    _blend(img, arr[:, :, :3], arr[:, :, 3].astype(np.float32) / 255.0, x, y)


def put_rich_text(img: np.ndarray, segments, org, px: int = TEXT_PX, gap: int = 18) -> int:
    """一行里**依次**写若干段不同颜色的文字，返回最后一段之后的 x。

    横幅里"黄字 A1 警告 + 红字边缘告警"必须挤在一行（见 `warning_strip`），
    靠 `cv2.putText` 的 `org` 做不到 —— 它不会告诉你上一段写到哪了。
    """
    x, y = int(round(org[0])), int(round(org[1]))
    for text, color in segments:
        put_text(img, text, (x, y), color, px)
        x += text_width(str(text), px) + gap
    return x


# --- 几何：边缘余量 / 投影 --------------------------------------------------------


def edge_distances_px(corners_px, shape) -> dict:
    """一个四边形到画面**四条边**的距离（px），逐边给。

    `shape` 是 `(h, w)`（OpenCV 的图像形状）。画面被当成 `[0,w) × [0,h)` 的像素区间：
    左边的距离 = `min(x)`，右边的 = `w − max(x)`，其余同理。**不做绝对值** ——
    码被切掉时这些值是负的，负多少就是切进去多少，那是操作者最想知道的数。

    角点用**规范次序**（跟着码走，见 `vision_core/markers.py` 模块 docstring），
    但这里只取 min/max，与次序无关。
    """
    pts = np.asarray(corners_px, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 3 or not np.isfinite(pts).all():
        raise ValueError(f"四边形至少要 3 个有限角点，实际 {np.shape(corners_px)}")
    height, width = int(shape[0]), int(shape[1])
    if height <= 0 or width <= 0:
        raise ValueError(f"图像形状必须是正的 (h, w)，实际 {shape!r}")
    xs, ys = pts[:, 0], pts[:, 1]
    return {
        "left": float(xs.min()),
        "right": float(width - xs.max()),
        "top": float(ys.min()),
        "bottom": float(height - ys.max()),
    }


def edge_margin_px(corners_px, shape) -> float:
    """四边形到画面四边的**最小**距离（px）。可负（被切进画面）。

    "最小值"是对**四条边**取最小，不是"离哪个角点最近"：一个码的四角都在画面内、
    但整体只离右边缘 12 px 时，这个数是 12 —— R48 里 ID21 当时就是这个状态。
    """
    return min(edge_distances_px(corners_px, shape).values())


EDGE_NAMES = {"left": "左", "right": "右", "top": "上", "bottom": "下"}


def edge_warnings(poses, shape, margin: float = EDGE_MARGIN_PX) -> list:
    """所有余量不足 `margin` 的码 → 告警文字（按 ID 排序，保证逐帧输出稳定）。

        ID20 距上边缘 3px ⚠被切
        ID21 超右边缘 6px ⚠被切

    四个码都健康时返回空列表，横幅上就不出现这一行（噪声必须能被"不在"这件事表达出来）。
    """
    out = []
    for marker_id in sorted(poses):
        dists = edge_distances_px(getattr(poses[marker_id], "corners_px", None), shape)
        side = min(dists, key=lambda key: dists[key])
        value = dists[side]
        if value >= margin:
            continue
        name = EDGE_NAMES[side]
        if value >= 0.0:
            out.append(f"ID{marker_id} 距{name}边缘 {value:.0f}px ⚠被切")
        else:
            out.append(f"ID{marker_id} 超{name}边缘 {-value:.0f}px ⚠被切")
    return out


def missing_ids(poses) -> list:
    """这一帧没检出的场地码（边界 20/21/22/23 + 车载 29），按固定次序。"""
    return [i for i in (*calib_store.BOUNDARY_IDS, calib_store.VEHICLE_ID) if i not in poses]


def marker_center_px(pose) -> tuple:
    """码的中心像素（四个角点的均值），取整成两个 int —— 与 `live_aruco.py` 印的一样。"""
    centre = np.asarray(pose.corners_px, dtype=np.float64).reshape(-1, 2).mean(axis=0)
    return (int(round(float(centre[0]))), int(round(float(centre[1]))))


def project_point(K, p_cam, dist=None):
    """相机系 3D 点 → 像素。`z <= 0`（在相机后方）或非有限值时返回 `None`（不画）。

    给了 `dist` 就走 `cv2.projectPoints`，与 `detect_markers` 解码位姿用的是**同一个**
    畸变模型 —— 叠加层和检测结果必须在同一个像素空间里，否则重标定过畸变之后
    四边形和十字会差好几个像素。当前 `calib_intrinsics_chess.json` 的 dist 全 0，
    两条路数值相同；保留这个参数是为了 `calib_distortion.py` 之后不用改这里。
    """
    point = np.asarray(p_cam, dtype=np.float64).reshape(-1)
    if point.shape != (3,) or not np.isfinite(point).all():
        return None
    cam_mtx = np.asarray(K, dtype=np.float64)
    if cam_mtx.shape != (3, 3):
        raise ValueError(f"K 必须是 3x3，实际 {cam_mtx.shape}")
    if float(point[2]) <= 1e-9:            # 在相机后方/横切面上，投影无定义
        return None
    if dist is None:
        px = cam_mtx @ point
        uv = np.float64([px[0] / px[2], px[1] / px[2]])
    else:
        coeffs = np.asarray(dist, dtype=np.float64).ravel()
        # 点已经在相机系里，所以 rvec/tvec 都是零：把同一个点**再**当 tvec 传一遍会平移两次。
        # projectPoints 返回的已经是**像素**，不要再除一次 z（除完会缩到原点的百分之零点几）。
        proj, _ = cv2.projectPoints(point.reshape(1, 1, 3), np.zeros((3, 1)),
                                    np.zeros((3, 1)), cam_mtx, coeffs)
        uv = np.asarray(proj, dtype=np.float64).reshape(-1)[:2]
    if not np.isfinite(uv).all():
        return None
    return (float(uv[0]), float(uv[1]))


def field_point_cam(ff: FieldFrame, u_mm: float, v_mm: float) -> np.ndarray:
    """场地系平面上的点 `(u, v)` mm → 相机系 3D 点。

    `ff.R_cam2field` 的**行**是场地三轴在相机系下的方向，所以反过来用转置：
    `p_cam = origin_cam + R.T @ [u, v, 0]`。
    """
    R = np.asarray(ff.R_cam2field, dtype=np.float64)
    origin = np.asarray(ff.origin_cam, dtype=np.float64).reshape(-1)
    if R.shape != (3, 3) or origin.shape != (3,):
        raise ValueError("ff 需要 3x3 的 R_cam2field 与 (3,) 的 origin_cam")
    return origin + (R.T @ np.float64([float(u_mm), float(v_mm), 0.0]))


def body_origin_cam(pose, bias) -> np.ndarray:
    """车体原点（`bias` 修正之后）在**相机系**里的 3D 点，mm。

    ## 符号：与 `vision_core/field.py:body_pose_in_field` 里那一步逐字相同

        p_body_cam = tvec − R_cam2marker @ [dx_mm, dy_mm, 0]

    两个容易写错的地方：

    1. **矩阵是 `R_cam2marker`（码自身系 → 相机系），不是 `R_cam2field`。**
       `dx_mm/dy_mm` 是沿**打印码自身的规范轴**量的（R35，见 field.py 的推导），
       跟场地系毫无关系。两者差 ~90°，用错会让十字画到车的另一侧，而且**数字看着
       还挺合理**。
    2. **是减不是加**。`body_pose_in_field` 里减的那一项在**相机系**就已经减完了，
       之后再 `to_field` 转到场地系 —— 顺序不能换，更不能"换到场地系再减"。

    不重写这条几何是刻意的：叠加层上的十字必须落在**横幅里印的那个数**上，
    两者由 `test_body_origin_pixel_agrees_with_body_pose_in_field` 钉住。
    """
    rvec = np.asarray(getattr(pose, "rvec", None), dtype=np.float64).reshape(-1)
    tvec = np.asarray(getattr(pose, "tvec", None), dtype=np.float64).reshape(-1)
    if rvec.shape != (3,) or tvec.shape != (3,) or not np.isfinite(rvec).all() \
            or not np.isfinite(tvec).all():
        raise ValueError("pose 需要 (3,1) 的 rvec/tvec 且全有限")
    try:
        offset = np.float64([float(bias["dx_mm"]), float(bias["dy_mm"]), 0.0])
    except (KeyError, TypeError, ValueError) as exc:
        # 与 `field._bias3` 一样给 ValueError：缺键静默当 0 = 凭空造一个"码就在几何中心"
        # 的车体原点（R37 的同一条纪律）。
        raise ValueError(f"bias 里缺 dx_mm/dy_mm 或不是数：{bias!r}") from exc
    if not np.isfinite(offset).all():
        raise ValueError("bias 的 dx_mm/dy_mm 必须有限")
    R_cam2marker, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    return tvec - (R_cam2marker @ offset).reshape(3)


def clamp_px(point, shape, margin: int = 8) -> tuple:
    """把像素点夹进画面内（留 `margin` px）。场地原点在画面外时坐标轴仍然要画得出来。"""
    height, width = int(shape[0]), int(shape[1])
    x = min(max(float(point[0]), float(margin)), width - 1 - margin)
    y = min(max(float(point[1]), float(margin)), height - 1 - margin)
    return (int(round(x)), int(round(y)))


# --- 状态摘要（终端行 / 横幅共用）------------------------------------------------


def summarize(result, poses) -> str:
    """这一帧的紧凑状态行，**不含时间戳**（时间戳每帧都变，会毁掉"变化时才打"的比较）。

        READY  ID20@(400,70)  ID21@(1808,90)  ID22@(1790,795)  ID23@(365,700)  ID29@(845,292)  x=-65.0 y=+1.5 yaw=+0.3
        boundary_moved 缺[20]  ID21@(1808,90) ID22@(1790,795) ID23@(365,700) ID29@(845,292)

    `ok=False` 时**原样**印 `result.reason`（Task 11 的词表），不自造词：
    同一个项目里两张嘴说同一件事，grep 的下游会先坏掉。
    """
    marks = "  ".join(f"ID{mid}@({cx},{cy})"
                      for mid in sorted(poses)
                      for cx, cy in (marker_center_px(poses[mid]),))
    if result.ok:
        pose = (f"  x={signed(result.x_mm)} y={signed(result.y_mm)} "
                f"yaw={signed(result.yaw_deg)}")
        return f"READY  {marks}{pose}".rstrip()
    missing = missing_ids(poses)
    head = f"{result.reason} 缺[{','.join(str(i) for i in missing)}]" if missing else str(result.reason)
    return f"{head}  {marks}".rstrip()


def banner_lines(result, poses, bias_measured: bool) -> list:
    """状态横幅的行：`[(文字, 颜色), ...]`，黑底。READY 绿字，失败红字 + reason + detail。

    `bias_measured=False`（A1 还没做）时在下面补一行黄字：现在解出来的 x/y 是**车载码
    的中心**，不是车体原点。这句话少一行，A1 做完之前的每一个数都可能被当成位移用掉。
    """
    lines = []
    if result.ok:
        lines.append((
            f"READY  x={signed(result.x_mm)}mm  y={signed(result.y_mm)}mm  "
            f"yaw={signed(result.yaw_deg)}deg  reproj_rms={result.reproj_rms_px:.2f}px",
            COLOR_OK,
        ))
    else:
        missing = missing_ids(poses)
        boundary = [i for i in missing if i in calib_store.BOUNDARY_IDS]
        if boundary:
            head = f"缺边界码 [{', '.join(str(i) for i in boundary)}]"
        elif missing:
            head = f"缺车载码 [{', '.join(str(i) for i in missing)}]"
        else:
            head = "解算失败"
        lines.append((head, COLOR_BAD))
        lines.append((f"reason={result.reason}  {result.detail}", COLOR_BAD))
    return lines


A1_WARNING = "A1 偏置还没写：x/y 是车载码的中心，不是车体原点（差一个未知偏置，且不报错）"


def warning_strip(bias_measured: bool, warnings) -> list:
    """横幅里那条"A1 未测 + 边缘告警"的混合行：`[(文字, 颜色), ...]`。

    **为什么挤在一行**：今天四个边界码全贴着画面边缘（R48），横幅每多一行就多吃掉
    一条边界码。横幅必须矮 —— 它盖住的正是操作者要看的那个码。
    """
    strip = [] if bias_measured else [(A1_WARNING, COLOR_WARN)]
    strip += [(text, COLOR_BAD) for text in warnings]
    return strip


# --- 叠加绘制 --------------------------------------------------------------------


def draw_grid(img: np.ndarray, step: int = GRID_STEP_PX) -> None:
    """每 `step` px 一条绿线，并在上/左边缘标出坐标（`live_aruco.py` 的同一套）。"""
    height, width = img.shape[:2]
    for x in range(0, width + 1, step):
        cv2.line(img, (x, 0), (x, height), COLOR_GRID, 1)
        put_text(img, str(x), (x + 4, 4), COLOR_GRID, SMALL_PX)
    for y in range(step, height, step):
        cv2.line(img, (0, y), (width, y), COLOR_GRID, 1)
        put_text(img, str(y), (4, y + 2), COLOR_GRID, SMALL_PX)


def draw_markers(img: np.ndarray, poses, shape) -> None:
    """画四边形 + `ID=20 (400,70)`；余量不足的码红框加粗。

    框用 `cv2.aruco.drawDetectedMarkers` 画（显式给绿色，不让它每帧随机配色 ——
    随机色会在实时窗口里闪，操作者会以为码在变），贴边的码再叠一层红色粗多边形。
    **这一步会把码的黑边糊掉**：画完的图 `detect_markers` 检不出任何码，
    所以它只能作用在副本上（见 `draw_overlay`）。
    """
    healthy, healthy_ids = [], []
    risky = []
    for marker_id in sorted(poses):
        quad = np.asarray(poses[marker_id].corners_px, dtype=np.float64).reshape(4, 2)
        if edge_margin_px(quad, shape) < EDGE_MARGIN_PX:
            risky.append(np.round(quad).astype(np.int32).reshape(-1, 1, 2))
        else:
            # drawDetectedMarkers 要 CV_32FC2 的 (N,1,4,2)；给 int32 会断言失败。
            healthy.append(quad.astype(np.float32).reshape(-1, 1, 2))
            healthy_ids.append(marker_id)
    if healthy:
        # OpenCV 4.6.0 的 Python 绑定只接受 4 个参数（没有 `color=` 关键字，也没有 lineWidth）。
        cv2.aruco.drawDetectedMarkers(img, healthy, np.array(healthy_ids, dtype=np.int32), COLOR_OK)
    for pts in risky:
        cv2.polylines(img, [pts], True, COLOR_EDGE_BAD, 3)
    for marker_id in sorted(poses):
        cx, cy = marker_center_px(poses[marker_id])
        put_text(img, f"ID={marker_id} ({cx},{cy})", (cx + 8, cy - 10), COLOR_WHITE, LABEL_PX)


def draw_field_axes(img: np.ndarray, ff: FieldFrame, K, dist=None) -> None:
    """场地系原点与两个轴（+X = ID20→ID21，+Y = ID20→ID23），外加一条 100 mm 比例尺。

    投影到画面外的点先 `clamp_px` 再画：场地原点跑到画面外时，坐标轴仍然要能指出
    "场地中心在画面哪个方向"，而不是把线画到画布外面去。
    """
    shape = img.shape[:2]
    origin = project_point(K, ff.origin_cam, dist)
    tip_x = project_point(K, field_point_cam(ff, AXIS_LEN_MM, 0.0), dist)
    tip_y = project_point(K, field_point_cam(ff, 0.0, AXIS_LEN_MM), dist)
    if origin is None:
        return
    px0 = clamp_px(origin, shape)
    for tip, color, name in ((tip_x, COLOR_AXIS_X, "+X"), (tip_y, COLOR_AXIS_Y, "+Y")):
        if tip is None:
            continue
        px1 = clamp_px(tip, shape)
        cv2.arrowedLine(img, px0, px1, color, 3, tipLength=0.08)
        put_text(img, name, (px1[0] + 6, px1[1] - 8), color, LABEL_PX)
    cv2.drawMarker(img, px0, COLOR_AXIS_X, cv2.MARKER_TILTED_CROSS, 26, 2)
    put_text(img, "场地原点", (px0[0] + 8, px0[1] + 18), COLOR_AXIS_X, SMALL_PX)

    # 比例尺：AXIS_LEN_MM 在场地原点那个深度上折成多少像素，就画多长。
    if tip_x is not None:
        span = math.hypot(tip_x[0] - origin[0], tip_x[1] - origin[1])
        bar_x0, bar_y = 40, shape[0] - 48
        bar_x1 = int(min(bar_x0 + span, shape[1] - 40))
        if bar_x1 - bar_x0 >= 20:
            cv2.line(img, (bar_x0, bar_y), (bar_x1, bar_y), COLOR_WHITE, 3)
            for x in (bar_x0, bar_x1):
                cv2.line(img, (x, bar_y - 8), (x, bar_y + 8), COLOR_WHITE, 3)
            put_text(img, f"{AXIS_LEN_MM:.0f} mm ≈ {span:.0f} px", (bar_x0, bar_y - 34),
                     COLOR_WHITE, SMALL_PX)


def draw_body_origin(img: np.ndarray, result, pose, calib) -> None:
    """车体原点十字 + 它解出来的 `(x, y)` mm。

    画的是 `body_origin_cam()` 算出的那个相机系点，与横幅里印的 `x/y` 是**同一个点**
    的两种表示（R4：bias 三项全 0 时两者当然重合，但那时看不出符号对不对；A1 一填上
    偏置，两者必须仍然重合 —— 那才是这条线的自检）。
    """
    K = calib["K"]
    point = project_point(K, body_origin_cam(pose, calib["bias"]), calib.get("dist"))
    if point is None:
        return
    px = clamp_px(point, img.shape[:2])
    cv2.drawMarker(img, px, COLOR_BODY, cv2.MARKER_CROSS, 44, 3)
    # 标签放在十字的**下方**：bias=0 时它与 ID29 的中心标签重叠在同一像素上。
    put_text(img, f"车体原点 ({signed(result.x_mm)}, {signed(result.y_mm)})mm",
             (px[0] + 14, px[1] + 26), COLOR_BODY, LABEL_PX)


def draw_banner(img: np.ndarray, rows, alert: bool = False, alpha: float = 0.75) -> None:
    """顶部黑底横幅。`rows` 每行是若干个 `(文字, 颜色)` 段，同一行里颜色不同、依次向右排。

    横幅**必须矮、而且必须半透明**：今天出问题的那两个码（ID20 贴上边缘、ID21 贴右边缘）
    恰好就在横幅底下。纯黑会把它们整个盖住 —— 而"看不见那个码"正是这个工具要解决的
    问题本身。0.75 的不透明度让下面的四边形仍然透出来，文字仍然读得清。
    `alert=True`（有边缘告警）时底部压一条红线。
    """
    height, width = img.shape[:2]
    if not rows:
        return
    line_h = TEXT_PX + 10
    bar_h = min(6 + line_h * len(rows), height)
    band = img[0:bar_h]
    cv2.addWeighted(np.zeros_like(band), 1.0 - alpha, band, alpha, 0.0, dst=band)
    y = 6
    for segments in rows:
        put_rich_text(img, segments, (12, y), TEXT_PX)
        y += line_h
    if alert:
        cv2.line(img, (0, bar_h - 1), (width, bar_h - 1), COLOR_BAD, 2)


def draw_fps(img: np.ndarray, fps: float) -> None:
    """右下角帧率小字。实时调取景时要能一眼看出"画面卡了没有"，所以它不占横幅。"""
    text = f"{fps:4.1f} FPS"
    height, width = img.shape[:2]
    box_w = text_width(text, SMALL_PX) + 10
    cv2.rectangle(img, (width - box_w - 6, height - SMALL_PX - 12), (width - 6, height - 6),
                  (0, 0, 0), -1)
    put_text(img, text, (width - box_w, height - SMALL_PX - 8), COLOR_WHITE, SMALL_PX)


def draw_overlay(img: np.ndarray, result, poses, calib, *,
                 bias_measured: bool = False, show_grid: bool = False,
                 fps: float | None = None) -> np.ndarray:
    """把一帧画成"操作者要看的现场"，**就地**改 `img` 并把它返回。

    返回画好的画布而不是边缘告警列表：告警是 `edge_warnings(poses, shape)` 这个纯函数
    的事，调用方自己算一遍更清楚，也让调用方能拿到"显示用的那张图"。

    分工：`result`（`solve_image` 的判定）决定横幅与车体原点，`poses`（`detect_markers`
    的检出）决定四边形。场地系/车体位姿只在 `result.ok` 时才有 —— `boundary_moved`
    的那一帧**根本没有场地系**，画出来的坐标轴是编的，所以不画。

    **就地改写是故意的，调用方必须传副本**（`run_window` 传 `frame.copy()`）：叠加会把码的
    黑边糊掉，存下来的图 `detect_markers` 一个码都检不出来。
    """
    shape = img.shape[:2]
    if show_grid:
        draw_grid(img)
    draw_markers(img, poses, shape)
    if result.ok and result.field_frame is not None:
        draw_field_axes(img, result.field_frame, calib["K"], calib.get("dist"))
        pose = result.vehicle_pose or poses.get(calib_store.VEHICLE_ID)
        if pose is not None:
            draw_body_origin(img, result, pose, calib)
    warnings = edge_warnings(poses, shape)
    rows = [[segment] for segment in banner_lines(result, poses, bias_measured)]
    strip = warning_strip(bias_measured, warnings)
    if strip:
        rows.append(strip)
    draw_banner(img, rows, alert=bool(warnings))
    if fps is not None:
        draw_fps(img, fps)
    return img


# --- 相机 -----------------------------------------------------------------------


def open_camera(device: str = CAMERA_DEVICE):
    """开**一根常驻**的 capture handle（`verify_vision.grab_frame` 是"开→丢帧→读→关"）。

    分辨率/FourCC/设备路径全部取自 `verify_vision` 的常量，不在这里重写一遍；
    自动对焦**保持开启**（与 `live_aruco.py` 一致）—— 本任务不锁曝光/对焦。

    打不开就抛 `CameraUnavailable`，由调用方变成一行提示 + 退出码 4。
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise CameraUnavailable(
            f"相机 {device} 打不开：多半是被别的程序占着（浏览器/会议软件/v4l2-ctl/"
            "live_aruco.py），也可能是设备路径不对或当前用户没有读权限。"
            "关掉占用它的程序再跑一次；本脚本不会 traceback 糊你一脸。"
        )
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAPTURE_FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    return cap


def settle(cap, frames: int = DEFAULT_SETTLE_FRAMES) -> None:
    """启动时丢 `frames` 个缓冲帧，等自动对焦收敛（R39，帧数取 `verify_vision` 的默认值）。

    实时循环里不每帧丢 —— 那样画面会一顿一顿的；但**启动那一刻**必须丢，
    否则操作者看到的头几帧是糊的、码检不出来，会以为相机坏了。
    """
    for _ in range(max(0, int(frames))):
        cap.grab()


def read_frame(cap):
    """读一帧。读不到返回 None（调用方变成一行提示，不是 traceback）。"""
    ok, frame = cap.read()
    if not ok or frame is None:
        return None
    return frame


def save_snapshot(frame: np.ndarray, out_dir) -> Path:
    """存**未叠加**的原帧到 `out_dir`，同名不覆盖，返回绝对路径。

    存原帧而不是叠加后的画面：快照的用途是"拿去做证据"，
    `python3 视觉测量/脚本/verify_vision.py <打印出来的路径>` 要能直接解它 ——
    而叠加过的图实测一个码都检不出来（框与标签糊掉了黑边）。

    **调用方必须传原始像素**：本函数不判断"这张图是不是被画过"，那是 `run_window` 的
    接线责任（它给的是 `frame`，叠加走的是 `frame.copy()`）。
    """
    directory = Path(out_dir)
    directory.mkdir(parents=True, exist_ok=True)
    path = _unique_path(directory, f"live_{time.strftime('%Y%m%d_%H%M%S')}")
    if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]):
        raise OSError(f"快照写盘失败：{path}")
    return path.resolve()


# --- 两条主循环 ------------------------------------------------------------------


class TerminalLog:
    """状态行日志：**状态变化时**打一行带时间戳的摘要。

    ## 为什么还有一条冷却（`TERMINAL_MIN_INTERVAL_S`）

    实测开着窗口跑 1 秒：终端打出 12 行。原因是码在画面边缘上抖 —— ID22/ID23 时有时无，
    中心像素每次差 1 px，于是"和上一次不同"每帧都成立。而 brief 给这条日志的**目的**是
    "操作者不用盯着窗口也能在终端读到码的中心坐标和 READY 状态"，每秒十几行同一个意思
    的日志把这个目的彻底废掉了。

    所以在"变化才打"之外再加一条"距上一行至少 0.5 秒"。注意它是**延后**而不是**丢弃**：
    冷却期间 `last_line` 不更新，冷却一过当前行仍然与上次打过的不一样，照样会打出来。
    短暂的变化只是晚 0.5 秒出现，不会消失。
    """

    def __init__(self, interval: float | None = None):
        self.interval = TERMINAL_MIN_INTERVAL_S if interval is None else float(interval)
        self.last_line: str | None = None
        self.last_at = 0.0

    def report(self, result, poses) -> str:
        """打一行（该打才打），返回本帧的状态行 —— 调用方不需要自己记状态。"""
        line = summarize(result, poses)
        now = time.perf_counter()
        if line != self.last_line and now - self.last_at >= self.interval:
            print(f"[{time.strftime('%H:%M:%S')}] {line}", flush=True)
            self.last_line, self.last_at = line, now
        return line


def run_headless(cap, calib, n_frames: int, label: str, origin: str) -> int:
    """`--once` / `--frames`：解算 N 帧，**不开窗口**（无 DISPLAY 环境 / CI 用）。

    ## 判据：任何一帧没解出来就是 1，中途读帧失败也是 1

    与 `run_window` **刻意不同**（理由写在那个函数的 docstring 里）：这里每一帧都是
    一次**独立的采样** —— `--frames N` 的字面意思就是"要 N 帧"，采样没跑完、或者跑完
    却一帧都没解出来，都意味着这一轮没产出可用的测量。与 `verify_vision` "至少一张照片
    没解出来 → 1"同一口径。窗口模式是长时间会话，判据因此放宽到"整场有过一帧 READY"。

    两者的差异（同一场景：2 帧 READY 之后相机断开）：

        --frames N → 1（这一轮采样没跑完）
        窗口模式   → 0（跑了一半的测量仍然拿得走）

    这个方向相反**不是笔误**，别"顺手统一"掉；退出码表（模块 docstring / `--help` /
    `run_window` docstring 三处）都写着它，并由
    `test_the_camera_dropping_out_mid_session_keeps_the_frames_it_already_solved` 钉住。
    """
    log = TerminalLog()
    solved = 0
    started = time.perf_counter()
    for index in range(1, max(1, n_frames) + 1):
        frame = read_frame(cap)
        if frame is None:
            print(f"读帧失败（第 {index} 帧），退出：相机中途断开了或被别的程序抢走了。", flush=True)
            return EXIT_PHOTO_FAILED
        poses = detect_markers(frame, calib)
        result = solve_image(frame, calib, label, origin)
        solved += int(bool(result.ok))
        log.report(result, poses)
    elapsed = time.perf_counter() - started
    print(f"{solved}/{max(1, n_frames)} 帧 READY  "
          f"耗时 {elapsed * 1000:.0f} ms（{elapsed / max(1, n_frames) * 1000:.0f} ms/帧，"
          f"含两次 detect_markers）", flush=True)
    return EXIT_OK if solved == max(1, n_frames) else EXIT_PHOTO_FAILED


def _first_line(exc) -> str:
    """异常消息的第一行；**空消息也要给出 ""** 而不是抛 `IndexError`。

    `cv2.error("")` 与 `OSError("")` 都会 `str(exc) == ""`，于是 `splitlines()[0]` 在
    **except 里**抛 `IndexError`，把真正的错误盖掉 —— 报错的地方正在报错。
    """
    text = str(exc)
    return text.splitlines()[0] if text else ""


def _display_problem(detail: str = "") -> str:
    """开不了窗口时打给操作者看的一段话（原因 + 怎么修 + 原始错误）。

    **首行按 `_has_display()` 分两种说法**，不能说死"没有显示环境"：
    兜底层收窄到 `_window_call` 之后，这条路径在 `DISPLAY` **有值**时也会走到
    （`imshow` 收到畸形图、Qt 连接断了都会），那时候"当前没有显示环境"是假话，
    而"加 `--once`"的补救又恰好废掉这个工具存在的理由 —— 它就是给要看画面的人用的。

    **分三种情况，而不是两种**：`_has_display()` 在非 Linux 上恒为 True，而
    `DISPLAY`/`WAYLAND_DISPLAY` 在 macOS/Windows 上通常都没设，于是"有显示环境"这件事
    和"这两个变量有值"不是一回事。旧版把两者当成一件，在 `seen is None` 时拼
    `"xdpyinfo -display " + seen` 抛 `TypeError` —— 那个 `TypeError` 发生在
    `except _DisplayUnavailable` **里面**，于是本该给的提示与退出码 2 一起被顶掉，
    操作者看到的是一屏 traceback 而不是一行说明（macOS 上必然踩到）。

    `xdpyinfo` 那条自查命令只对 **X11** 有意义（它连的是 X display），所以它**只在
    `seen` 真是一个 X display（以 `:` 开头）时**出现；纯 Wayland（`wayland-0`）时给出
    的必然失败命令比不给更糟。

    这是**远端 shell 里最容易犯的用法错**：忘了加 `--once`，于是在没有 X 转发的机器上
    直接开窗口。本机实测（OpenCV 4.6.0 + Qt）无 DISPLAY 时 `namedWindow`/`imshow`
    **不抛异常、只是什么都不显示**，`waitKey` 永远返回 -1 —— 脚本于是无声无息地空转，
    比抛 traceback 更难查。所以 `run()` 先看环境变量，OpenCV 那边再兜一层。
    """
    seen = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    fallback = "  实在起不来：加 --once（只看一帧）或 --frames N（无窗口解 N 帧）。"
    if not _has_display():
        lines = [
            "当前没有显示环境（DISPLAY / WAYLAND_DISPLAY 都是空的）。",
            "  本机图形界面：`echo $DISPLAY` 应该有个值（X11 常见是 :0）。",
            "  远端 shell：需要 X 转发 —— `ssh -X`，或 `export DISPLAY=:0` 且本机有 X；"
            "本机的 X11 会话随时可能被关掉，连不上时换一个 DISPLAY 试。",
            fallback,
        ]
    elif not seen:
        # 走到这里只有一种可能：非 Linux（`_has_display()` 恒为 True）。
        # **不能**说"显示环境变量有值"—— 这台机器上一个显示环境变量都没有。
        lines = [
            f"连不上图形窗口：系统是 {sys.platform}，它的窗口系统不靠 DISPLAY，"
            "但 OpenCV 这边仍然起不来窗口。",
            "  常见原因：装的是 opencv-python 的 headless 版，或编译时没带 GUI 支持。",
            "  （没有显示环境变量可查 —— macOS/Windows 不走 X11，所以这一条没有自查命令。）",
            fallback,
        ]
    else:
        lines = [
            f"连不上图形窗口：显示环境变量有值（{seen}），但窗口还是开不起来。",
        ]
        if seen.startswith(":"):
            lines += [
                "  常见原因：X 转发没开、远端的 :0 已经失效、或者 OpenCV 没有 GUI 支持。",
                "  查一下：`xdpyinfo -display " + seen + " >/dev/null && echo ok` 能通就是窗口的问题。",
            ]
        else:
            lines.append(
                f"  这是 Wayland 的显示环境变量（{seen}），不是 X11 —— X 的那套自查命令在"
                "这里用不了；而 OpenCV 在纯 Wayland 会话里常常没有可用的后端。"
            )
        lines.append(fallback)
    if detail:
        lines.append(f"  原始错误：{detail}")
    return "\n".join(lines)


def _has_display() -> bool:
    """当前环境能不能开窗口。

    **只看 Linux**：Windows/macOS 的窗口系统不需要 DISPLAY 环境变量，
    在那儿拿 DISPLAY 判断会误报"没有显示"。Wayland 另有一套 `WAYLAND_DISPLAY`。
    """
    if not sys.platform.startswith("linux"):
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


class _DisplayUnavailable(RuntimeError):
    """窗口相关的 `cv2.*` 调用失败了（没有显示环境 / 无 GUI 编译 / X 连接断了）。"""


def _window_call(func, *args, **kwargs):
    """跑一个**窗口相关**的 `cv2` 调用；失败就换成 `_DisplayUnavailable` 再抛。

    ## 为什么要这一层，而不是"小心地把 try 缩到窗口调用那里"

    上一版的 `except cv2.error` 从 `namedWindow` 一路包到循环里的 `detect_markers` /
    `solve_image` / `save_snapshot`。那三个环节与显示毫无关系，它们抛的 `cv2.error`
    却被贴上"开不了图形窗口：当前没有显示环境"的标签、给出"加 `--once`"这种帮不上忙
    的补救，并以 `EXIT_USAGE` 结束整个实时会话 —— **误导性诊断** + 错的退出码。
    真实触发路径：`cv2.imwrite` 对不认识的扩展名是**抛** `cv2.error` 而不是返回 False
    （本机实测 `imwrite("x.zzz")` 抛 `could not find a writer`），磁盘满/权限不足同理。

    用**异常类型**而不是 try 的作用域来划界，就不会出现"以后往循环里加一行代码，
    它又被贴上显示问题的标签"这种事：能抛 `_DisplayUnavailable` 的只有那几个窗口调用。
    """
    try:
        return func(*args, **kwargs)
    except cv2.error as exc:
        raise _DisplayUnavailable(_first_line(exc)) from exc


def run_window(cap, calib, args, bias_measured: bool) -> int:
    """实时窗口主循环。返回 0（会话跑完，且整场**有过一帧** READY）。

    ## 退出码判据：至少有一帧 READY 就算成功，整场一帧都没解出来算失败

    窗口模式是长时间会话：操作者开窗 → 挪相机 → 看码进没进画面 → `q` 退出。
    这条判据是按这个用法定的：

    - **有掉帧不是失败**（0 的判据是"至少有一帧 READY"）。调取景的途中码时有时无
      （今天 ID23 就是），拿"有帧没解出来"算失败，等于每次挪一下相机都要吃一个非 0
      退出码。
    - **整场一帧都没解出来算失败**（1 的判据是"整场零 READY"）。这次会话确实没产出
      任何可用的测量，而 `EXIT_OK` 会让按 `$?` 判断的脚本拿到假通过。

    与 `run_headless` **刻意不同**：无窗口模式下每帧都是一次独立的采样（`--frames N`
    的字面意思就是要 N 帧），**任何一帧没解出来就是 1**，中途读帧失败也是 1，
    0 的判据是"每一帧都 READY"。

    **中途掉线走的是上面那条判据**：相机中途断开只是 `break` 出循环，已经解出来的那
    几帧仍然是有效的测量，所以"两帧 READY 之后拔线"仍然给 0，而 `run_headless` 在
    同一场景给 1。**这个差异写进了模块 docstring、`--help` 与 `run_headless` 的
    docstring**，别当成笔误"统一"掉。

    **Ctrl-C 也不打 traceback**：`except KeyboardInterrupt` 打一行"已退出（Ctrl-C）"
    之后**落到下面那条同一个判据上**返回（有过 READY → 0，零 READY → 1）——
    操作者在终端里随手 Ctrl-C 是很可能的交互，而本工具的出发点就是"别给操作者一屏
    红字"。`finally` 里的 `destroyAllWindows()` 与 `run()` 里的 `cap.release()` 在
    Ctrl-C 下同样执行（所以 `/dev/video0` 不会被占住）。

    窗口打不开时**不抛 traceback**：打一段说清怎么修的话，返回 `EXIT_USAGE`。
    那一层兜底只包**窗口调用**（`_window_call`）；少了它，无显示/无 GUI 编译的
    OpenCV 上 `namedWindow`/`imshow` 抛的 `cv2.error` 会穿出 `main()`；
    而它一旦包得过大，解算/写盘失败又会被说成"没有显示环境"。
    """
    solved = 0
    try:
        _window_call(cv2.namedWindow, WINDOW_NAME, cv2.WINDOW_NORMAL)
        _window_call(cv2.resizeWindow, WINDOW_NAME,
                     int(CAPTURE_WIDTH * args.scale), int(CAPTURE_HEIGHT * args.scale))
        _window_call(cv2.moveWindow, WINDOW_NAME, 0, 0)
        print(f"窗口已启动（DISPLAY={os.environ.get('DISPLAY', '')}）；"
              "q/ESC 退出，s 存快照，g 网格开关", flush=True)

        show_grid = False
        log = TerminalLog()
        fps = 0.0
        last = time.perf_counter()
        while True:
            frame = read_frame(cap)
            if frame is None:
                print("读帧失败，退出：相机中途断开了或被别的程序抢走了。", flush=True)
                break
            now = time.perf_counter()
            fps = fps * 0.9 + (1.0 / max(now - last, 1e-6)) * 0.1
            last = now

            h, w = frame.shape[:2]
            poses = detect_markers(frame, calib)
            result = solve_image(frame, calib, f"live {w}x{h}", args.device)
            solved += int(bool(result.ok))
            # **叠加只作用在副本上**，`frame` 始终是未叠加的原帧。
            #
            # 之前这里是 `draw_overlay(frame, ...)`（就地改写），于是 `s` 存下来的是
            # 画过框、压过标签的图：`drawDetectedMarkers` 的绿框 + `ID=xx (x,y)` 标签
            # + 红色粗多边形把码的黑边糊掉，**实测五个码一个都检不出来**，
            # `verify_vision.py` 对那张快照报 `boundary_moved 缺[20,21,22,23]`。
            # 而快照的唯一用途就是留 A1/A2 的证据 —— 这正是本任务要消灭的那类分叉
            # （"预览说能存、正式工具说码没了"）。
            #
            # 用副本而不是"叠加前先 copy 一份留着"，是因为副本让 `frame` 在**任何**
            # 绘制路径下都保持干净：以后再加一种叠加也不会漏掉快照。
            # 代价是一帧 6 MB 的 memcpy（约 1 ms，占 45 ms 一帧的 2%），换一条结构性保证。
            display_frame = draw_overlay(frame.copy(), result, poses, calib,
                                         bias_measured=bias_measured, show_grid=show_grid,
                                         fps=fps)
            log.report(result, poses)

            display = cv2.resize(display_frame, None, fx=args.scale, fy=args.scale) \
                if args.scale != 1.0 else display_frame
            _window_call(cv2.imshow, WINDOW_NAME, display)
            key = _window_call(cv2.waitKey, 1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("g"):
                show_grid = not show_grid
            if key == ord("s"):
                # 存 `frame`（原帧），不是 `display_frame`（叠加后的）。
                try:
                    print(f"快照已存：{save_snapshot(frame, args.snapshot_dir)}", flush=True)
                except (OSError, cv2.error) as exc:
                    # 写盘失败是**按一次 s** 的事，不该把整个实时会话带走：
                    # 磁盘满/权限不足/编码器不认识扩展名（cv2.imwrite 是抛 cv2.error，
                    # 不是返回 False）都归这里。报一行实话，预览继续跑。
                    print(f"快照存不了：{_first_line(exc)}", flush=True)
    except KeyboardInterrupt:
        # 操作者在终端里随手 Ctrl-C 很常见。打一行实话，然后**落到底下那条同一个判据**
        # 上返回（有过 READY → 0 / 零 READY → 1）—— 不在这里随手 return EXIT_OK，
        # 那会让"整场零 READY 却返回 0"这条假通过换个入口回来。
        print("已退出（Ctrl-C）。", flush=True)
    except _DisplayUnavailable as exc:
        print(_display_problem(str(exc)), flush=True)
        return EXIT_USAGE
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:       # GUI 后端不可用/连接已断时这句会抛；清理不该盖掉真正的错
            pass
    # 整场**一帧都没解出来**才算失败（判据见 docstring）：窗口模式是长时间会话，
    # 调取景的途中掉几帧是常态，拿它算失败等于每次调相机都要被判一次；
    # 但整场下来一个 READY 都没有，那这次会话确实没产出任何可用的测量 ——
    # 与 `run_headless` 的口径（有任何一帧没解出来就是 1）不同，这里刻意放宽到"全部"。
    return EXIT_OK if solved > 0 else EXIT_PHOTO_FAILED


# --- 命令行 ---------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="live_field.py",
        formatter_class=argparse.RawTextHelpFormatter,
        description="A1–A4 人工标定的实时取景预览：逐帧调用 verify_vision.solve_image 真管道。",
        epilog=(
            "例子：\n"
            "  开着窗口挪相机（操作者日常用的就是这条）：\n"
            "    python3 视觉测量/脚本/live_field.py\n"
            "  只看一帧就退出（不开窗口，无 DISPLAY 也能跑）：\n"
            "    python3 视觉测量/脚本/live_field.py --once\n"
            "  压测/连续采样 N 帧：\n"
            "    python3 视觉测量/脚本/live_field.py --frames 30\n"
            "  窗口里按 s 存下的快照是**未叠加的原帧**（画过框的图一个码都检不出来），可以直接：\n"
            "    python3 视觉测量/脚本/verify_vision.py /tmp/opencode/live_field/live_20261014_141500.jpg\n"
            "\n"
            "按键（只有开窗口时有效）：q / ESC 退出 | s 存快照 | g 网格开关（每 240 px）\n"
            "\n"
            "退出码（两种模式判据不同，见 live_field.py 的 run_window docstring）：\n"
            "  0 = 会话跑完了，且整场有过一帧 READY\n"
            "      无窗口模式（--once / --frames N）：每一帧都 READY\n"
            "      窗口模式：至少有一帧 READY —— 但整场一帧都没解出来算失败\n"
            "          （否则按 $? 判断会拿到假通过）\n"
            "  1 = 本轮没产出可用的解\n"
            "      无窗口模式：有帧没解出来（任何一帧） / 中途读帧失败 / 标定文件读不出来\n"
            "      窗口模式：整场零 READY 才算失败（中途掉线不算，已有 READY 仍给 0）\n"
            "  2 = 用法错误，或窗口打不开（没有显示环境 / 连不上显示服务器）\n"
            "  4 = 相机打不开（被别的程序占着）—— 连相机都没打开\n"
            "\n"
        ),
    )
    p.add_argument("--device", default=CAMERA_DEVICE,
                   help=f"相机设备（默认 {CAMERA_DEVICE}，取自 verify_vision）")
    p.add_argument("--calib", default=calib_store.DEFAULT_CALIB_PATH, metavar="PATH",
                   help=f"标定文件，相对路径按仓库根解析（默认 {calib_store.DEFAULT_CALIB_PATH}；"
                        "不存在就用内建默认值，横幅会说明现在解的是不是车体原点）")
    p.add_argument("--once", action="store_true",
                   help="只解算一帧，打印状态行后退出，**不开窗口**（等价于 --frames 1）")
    p.add_argument("--frames", type=int, default=0, metavar="N",
                   help="无窗口解算 N 帧后退出（不开窗口；快速验证 / 压测用）")
    p.add_argument("--scale", type=float, default=DEFAULT_SCALE, metavar="F",
                   help=f"窗口显示缩放（默认 {DEFAULT_SCALE}；只影响显示，不影响解算）")
    p.add_argument("--snapshot-dir", default=DEFAULT_SNAPSHOT_DIR, metavar="PATH",
                   help=f"按 s 存快照的目录（默认 {DEFAULT_SNAPSHOT_DIR}，"
                        "故意在仓库外：快照是调取景的中间产物，不是证据）")
    return p


def _usage_error(parser: argparse.ArgumentParser, message: str) -> NoReturn:
    """用法错误：用法摘要 + 一行原因，退出码 `EXIT_USAGE`（与 argparse 的 2 同值）。

    与 `verify_vision._usage_error` 同一套写法：argparse 的 `parser.error` 会把 usage
    整段打到 stderr，而这里只需要一行说清"哪个值不对"。
    """
    parser.print_usage(sys.stderr)
    print(f"{parser.prog}: error: {message}", file=sys.stderr)
    raise SystemExit(EXIT_USAGE)


def validate_args(parser: argparse.ArgumentParser, args) -> None:
    """只碰人一定会踩的几条。"""
    if args.frames < 0:
        _usage_error(parser, f"--frames 不能为负（实际 {args.frames}）")
    if not (args.scale > 0.0) or args.scale > 4.0:
        _usage_error(parser, f"--scale 必须在 (0, 4] 之间（实际 {args.scale}）")
    if args.once and args.frames > 0:
        # 验收命令就是 `--once --frames 1`，两者不是互斥的：给了 --frames 就按它算帧数。
        print("提示：--once 与 --frames 同时给了，按 --frames 的帧数解算。", flush=True)


def run(args) -> int:
    calib_path = calib_store.resolve_path(args.calib)
    try:
        calib = calib_store.load_calib(calib_path)
    except ValueError as exc:
        print(f"标定文件读不出来，先修它：{exc}", flush=True)
        return EXIT_PHOTO_FAILED

    # 表头直接复用 `verify_vision.print_header`：yaw 零基准、bias 参考系、A1 未测警告
    # 全都是既有裁决（R34/R35/R37）写死在那里的，抄一份就是下一次各说各话。
    print_header(calib, calib_path)
    _bias_text, bias_measured = bias_status(calib, calib_path)

    headless = bool(args.once) or args.frames > 0
    n_frames = args.frames if args.frames > 0 else 1
    print(f"模式：{'无窗口' if headless else '实时窗口'}"
          f"  设备={args.device}  标定={args.calib}", flush=True)

    # 显示环境**在开相机之前**查：没有显示就别去占 /dev/video0（那是共享设备，
    # 操作者正开着 live_aruco.py 时被抢走几秒是没必要的），提示里直接告诉他加哪个标志。
    if not headless and not _has_display():
        print(_display_problem(), flush=True)
        return EXIT_USAGE

    try:
        cap = open_camera(args.device)
    except CameraUnavailable as exc:
        print(str(exc), flush=True)
        return EXIT_CAMERA_UNAVAILABLE

    try:
        settle(cap, DEFAULT_SETTLE_FRAMES)      # 模块全局按调用时取值，测试才能把它压到 0
        if headless:
            return run_headless(cap, calib, n_frames, f"live {args.device}", args.device)
        return run_window(cap, calib, args, bias_measured)
    finally:
        cap.release()


def main(argv: list | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(sys.argv[1:] if argv is None else argv))
    validate_args(parser, args)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
