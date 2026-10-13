"""场地系构建 + 车体偏置修正。

输入是 Task 2 解出来的 5 个单码相机系位姿，输出是下游 Task 4/10/11/12 唯一在意的
三个数：车体原点在**场地系**里的 `(x_mm, y_mm, yaw_deg)`。

## 场地系怎么定义的（R4）

    原点   4 个边界码（ID 20/21/22/23）质心
    +X     ID20 → ID21（TL → TR）
    +Y     指向 ID23（BL）一侧
    +Z     cross(X, Y)
    法向 n 桌面平面法向，指向相机

**原点是质心，不是 ID20。** 计划里 Interfaces 与 Step 3 两处写的是质心，只有三个
数字阈值（`norm(tl) < 1.0`、`tr[0] > 400`、`bl[1] > 200`）是按 TL 原点写的 ——
那两个阈值自己就和计划、和 Task 9 的"半长 252.5 / 半宽 125.9"互相矛盾。
质心赢：spec §6.2、计划接口行、计划 Step 3 第 1 条、Task 9 的半尺寸，四条独立信号。
实拍照片上四个码落在 (±252, ±126) mm，最大偏差 4.3 mm（ID23 的 y），来源是
相机不是严格垂直俯视（四个码质心高度 510.4~521.8 mm，差 11 mm）。

## 场地 +Z 扎进桌面，`to_field` 的 z 分量在桌面上方是负的（R33）

X、Y 都躺在像平面里，所以 `cross(X, Y)` 必然指向相机 +Z —— 俯视相机 +Z 是**朝被摄物**，
也就是**扎进桌面**。而计划要求的法向 `n` 指向相机。于是

    z_axis = −n ，  to_field(...)[2] = −n · (p_cam − origin_cam)

桌面上方（相机一侧）的点因此拿到**负**的 field z（车载码 ID29 实测 −79.1 mm，
它悬在桌面上方 78.6 mm）。两个法向反号是对的，不是 bug；读代码的人第一个反应
一定是"这里符号错了"，所以在这里、也在 `to_field` 的 docstring 里写死。

## 航向角：从上往下看逆时针为正，零点在场地 +X（R5b′）

`heading_deg` 取码的**规范 +Y（上轴）**在场地系里的方位角：

    v_field = R_cam2field @ (R_cam2marker @ [0, 1, 0])
    heading  = −degrees(atan2(v_field[1], v_field[0]))      归一到 (−180, 180]

**为什么要取负号**：在这个场地系里，从 +X 转到 +Y 在画面上是**顺时针**
（X = TL→TR 是画面向右，Y = 朝 BL 是画面向下），所以裸的 `atan2(v_y, v_x)` 是
顺时针为正，而计划与固件 `bodyToPWM` 用的 ω 是**逆时针为正**。差这一个负号，
Task 12 的 `yaw_diff_deg` 就会和编码器解算的 yaw 永远对不上 —— 而且数字看起来
完全合理（都是角度、都随车转动），没有一处会报错。这是整条管线最贵的一个符号。

**零点为什么是场地 +X 而不是"图像 up"**：计划 Global Constraints 写的是
"yaw 参考系 = 场地系，图像 up = 0°"，这两句在真实照片上是打架的（实测场地 +X
与图像 up 差 −87.2°）。零基准必须钉在**场地系**里，否则 Task 12 没法把
`yaw_end − yaw_start` 与 `odometry.integrate_encoders` 的 yaw 相减。取场地 +X。

## 绝不用边界码的姿态（R27）

四个边界码的**位置**都好（z 分量彼此差不到 1.7 mm，说明四点共面），但**姿态**
不能用：ID20 解出来离"正对相机"差 63.6°（打印面法向对齐度 0.445、重投影 RMS
0.846 px 全场最差、上边离画面边界只有 6 px，是被裁切 + 亚像素细化带歪的）。
所以 `build_field_frame` **一个 `pose.rvec` 都不读**：原点、法向、两个轴全部来自
`pose.tvec`。`test_field_frame_ignores_boundary_marker_rotations` 把四个码的姿态
全换成垃圾之后要求场地系一模一样，就是为了钉死这条。

## 抛异常的边界

`build_field_frame` 只在两种情况下抛：缺边界码（少一个码的场地系是**错的**且
不报错），以及四点退化到构不成平面（法向无定义）。四点不共面只发 warning ——
现实里"某个码被撞歪了几毫米"不该让整轮采图崩掉，但必须喊出来。

下游的数学函数（`to_field` / `heading_deg` / `body_pose_in_field`）对退化输入一律
抛 ValueError：它们对外是数学函数，悄悄返回一个假数比崩掉糟得多。
`detect_markers` 那种"绝不让采图循环挂掉"的义务属于调用方，不属于这里。
"""
from __future__ import annotations

import math
import warnings
from collections.abc import Mapping
from dataclasses import dataclass

import cv2
import numpy as np

from . import calib_store

# 场地平面拟合残差超过这个值（mm）就发 warning：某个边界码离桌面太远了。
# 5 mm 对 25 mm 的码来说是很大的离面量，而相机本身的装夹误差只有零点几毫米。
COPLANAR_TOL_MM = 5.0

# 几何退化判据（mm）。场地 ≈ 505 × 252 mm，1 mm 的边长必然是"码没解出来"，
# 不是"场地就这么小"。
_MIN_SPAN_MM = 1.0


@dataclass
class FieldFrame:
    """场地坐标系（`build_field_frame` 的产物）。

    R_cam2field  (3,3) **行**向量 = 场地三轴在相机系下的方向：
                  第 0 行是场地 +X（TL→TR）在相机系的方向，第 1 行是 +Y（朝 BL），
                  第 2 行是 +Z = cross(X, Y)（**扎进桌面**，见模块 docstring）。
                  正交、det = +1。
    origin_cam   (3,)   场地原点（4 个边界码质心）在相机系下的位置，mm
    marker_px    {id: (2,)} 4 个边界码角点的图像坐标均值，供 Task 10 复核
                  "边界码被车撞移位"（Review Focus #2）
    normal_cam   (3,)   桌面平面法向，**指向相机**（俯视相机下 z 分量为负）。
                  与 R_cam2field 第 2 行**反号**：`z_axis = −normal_cam`。
    """

    R_cam2field: np.ndarray
    origin_cam: np.ndarray
    marker_px: dict[int, np.ndarray]
    normal_cam: np.ndarray


def _unit(v: np.ndarray) -> np.ndarray:
    """单位化。零向量没有方向 —— 抛，不返回 NaN。"""
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError(f"向量退化（模长 {n!r}），方向无定义")
    return v / n


def _point3(value, what: str) -> np.ndarray:
    """把 (3,) / (3,1) / [3] 统一成 (3,) float64，非有限值抛。"""
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError(f"{what} 必须是 3 个有限值，实际 {np.shape(value)}")
    return arr


def _boundary_points(poses) -> np.ndarray:
    """取 4 个边界码的 `tvec`，形状 (4,3)，顺序 ID20/21/22/23。

    只读 `tvec`。**任何姿态都不碰**（R27，见模块 docstring）。
    """
    if not isinstance(poses, Mapping):
        raise ValueError(f"poses 必须是 {{marker_id: MarkerPose}} 这样的映射，实际 {type(poses).__name__}")
    missing = [mid for mid in calib_store.BOUNDARY_IDS if mid not in poses]
    if missing:
        raise ValueError(
            f"场地系需要 4 个边界码 {list(calib_store.BOUNDARY_IDS)}，缺 {missing}。"
            "少一个码的场地系是错的（且不报错），必须重拍，不能凑合。"
        )
    # getattr(..., None)：poses 里塞了非 MarkerPose 的东西时也要给 ValueError，
    # 不能漏一个 AttributeError 出去（下游 catch 的是 ValueError）。
    return np.vstack([_point3(getattr(poses[mid], "tvec", None), f"ID{mid} 的 tvec")
                      for mid in calib_store.BOUNDARY_IDS])


def _marker_px(poses) -> dict[int, np.ndarray]:
    out: dict[int, np.ndarray] = {}
    for mid in calib_store.BOUNDARY_IDS:
        corners = np.asarray(getattr(poses[mid], "corners_px", None), dtype=np.float64)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise ValueError(f"ID{mid} 的 corners_px 必须是 4x2 有限值，实际 {corners.shape}")
        out[mid] = corners.mean(axis=0)
    return out


def build_field_frame(poses) -> FieldFrame:
    """由 4 个边界码的位姿建场地系。

    poses  `dict[int, MarkerPose]`，必须含 ID 20/21/22/23（车载码 29 用不到）

    缺任一边界码、或四点退化到构不成平面 → `ValueError`。
    四点不共面（离面残差 > `COPLANAR_TOL_MM`）→ 发 `RuntimeWarning` 但仍出结果。

    只用 `pose.tvec` 和 `pose.corners_px`；**绝不读 `pose.rvec`**（R27）。
    """
    P = _boundary_points(poses)                  # (4,3)，行序 = 20/21/22/23
    tl, tr, _br, bl = P                          # ID20=TL, ID21=TR, ID22=BR, ID23=BL
    origin = P.mean(axis=0)

    along_x = float(np.linalg.norm(tr - tl))
    along_y = float(np.linalg.norm(bl - tl))
    if along_x < _MIN_SPAN_MM or along_y < _MIN_SPAN_MM:
        raise ValueError(
            f"4 个边界码退化（ID20→ID21 只有 {along_x:.3f} mm，ID20→ID23 只有 "
            f"{along_y:.3f} mm），场地平面无定义。"
        )

    # --- 平面法向：四点 PCA，最小特征值对应的特征向量 ---------------------------
    centred = P - origin
    cov = centred.T @ centred / centred.shape[0]
    _eigvals, eigvecs = np.linalg.eigh(cov)
    n = _unit(eigvecs[:, 0])
    # 指向相机：俯视相机下就是相机 Z 分量为负。写成"与原点向量反号"是为了对
    # 装了歪的相机也成立（法向永远背朝被摄点、朝向相机中心）。
    if float(n @ origin) >= 0.0:
        n = -n

    # --- +X = ID20 → ID21，先投到平面内再单位化 --------------------------------
    x_axis = _unit((tr - tl) - float((tr - tl) @ n) * n)

    # --- +Y：cross(n, X)，再按"指向 ID23 一侧"定符号 ----------------------------
    # cross 的手性给不出"往哪边翻"，只能靠位置定 —— 这一步是最容易写反的地方。
    y_axis = _unit(np.cross(n, x_axis))
    if float(y_axis @ (bl - tl)) < 0.0:
        y_axis = -y_axis
    if abs(float(y_axis @ (bl - tl))) < _MIN_SPAN_MM:
        raise ValueError("ID23 落在 ID20→ID21 直线上，场地 +Y 无定义。")

    # --- +Z = cross(X, Y)：右手系，det = +1 -------------------------------------
    z_axis = _unit(np.cross(x_axis, y_axis))

    R_cam2field = np.vstack([x_axis, y_axis, z_axis])

    # 四点共面性自检：z 就是每个码离平面的残差。某个码被撞到几十毫米外时，
    # 平面拟合会歪掉，之后每一条实验的真值全错且无人报错 —— 必须喊。
    residual = np.abs(centred @ z_axis)
    worst = int(np.argmax(residual))
    if float(residual[worst]) > COPLANAR_TOL_MM:
        mid = calib_store.BOUNDARY_IDS[worst]
        warnings.warn(
            f"4 个边界码不共面：ID{mid} 离拟合平面 {residual[worst]:.1f} mm"
            f"（> {COPLANAR_TOL_MM} mm）。场地平面法向会因此带偏，所有位移/转角跟着偏。"
            "先确认边界码有没有被撞歪或没贴平桌面。",
            RuntimeWarning,
            stacklevel=2,
        )

    return FieldFrame(
        R_cam2field=R_cam2field,
        origin_cam=origin,
        marker_px=_marker_px(poses),
        normal_cam=n,
    )


def to_field(ff: FieldFrame, p_cam) -> np.ndarray:
    """相机系位置 → 场地系位置（mm）。`ff.R_cam2field @ (p_cam − ff.origin_cam)`。

    返回 (x_mm, y_mm, z_mm)。**z 在桌面上方（相机一侧）是负的**：场地 +Z 扎进
    桌面（X、Y 都躺在像平面里，cross(X, Y) 必然指向相机 +Z），而计划要求的法向
    `normal_cam` 指向相机，两者反号 —— 见模块 docstring 的 R33 一节。
    """
    return np.asarray(ff.R_cam2field, dtype=np.float64) @ (
        _point3(p_cam, "p_cam") - np.asarray(ff.origin_cam, dtype=np.float64))


def _wrap180(deg: float) -> float:
    """归一到 (−180, 180]。−180 归到 +180，避免 ±180 两个等价表示。"""
    wrapped = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def heading_deg(R_cam2marker: np.ndarray, ff: FieldFrame) -> float:
    """车体航向（度）：码的**规范 +Y（上轴）**在场地平面内的方位角。

    `R_cam2marker` 码自身 → 相机的旋转矩阵（`cv2.Rodrigues(pose.rvec)[0]`）。

    零点在**场地 +X**（TL→TR），**从上往下看逆时针为正**，归一到 (−180, 180]。
    与 `odometry.integrate_encoders` 的 yaw 同号（Task 12 要把两者相减）。

    ## 符号：为什么裸方位角要取负

        v = R_cam2field @ (R_cam2marker @ [0, 1, 0])     码上轴在场地系的方向
        heading = −degrees(atan2(v[1], v[0]))

    场地 +X = TL→TR（画面向右）、+Y = 朝 BL（画面向下），所以"从 +X 转到 +Y"
    在画面上是**顺时针**；裸 `atan2(v_y, v_x)` 因此是顺时针为正，而计划与固件
    `bodyToPWM` 的 ω 是逆时针为正。少这个负号，视觉和编码器的 yaw 会永远对不上，
    而且数字看起来完全合理。

    ## 为什么取 +Y 而不是 −Y

    码自身的坐标系由 `_ippe_object_points` 定：+Y = 角点3→角点0，是码正面的**上轴**。
    对俯视相机，"正对相机"的码解出 `rvec=[π,0,0]`、`R = diag(1,−1,−1)`，于是
    规范 +Y 映到相机 −Y = **图像上方**。取 −Y 会把每个 yaw 翻 180°（R5b 的原案，
    已按 R5b′ 更正）。

    up 轴与场地平面近乎垂直（码躺着朝天）时方位角无定义 → 抛 ValueError，
    不返回假 0°。
    """
    R = np.asarray(R_cam2marker, dtype=np.float64)
    if R.shape != (3, 3) or not np.isfinite(R).all():
        raise ValueError(f"R_cam2marker 必须是 3x3 有限值，实际 {R.shape}")

    up_field = np.asarray(ff.R_cam2field, dtype=np.float64) @ (R @ np.float64([0.0, 1.0, 0.0]))
    if float(np.hypot(up_field[0], up_field[1])) < 1e-6:
        raise ValueError(
            f"码的上轴与场地平面近乎垂直（平面内分量 {np.hypot(up_field[0], up_field[1]):.2e}），"
            "航向无定义。"
        )
    return _wrap180(-math.degrees(math.atan2(float(up_field[1]), float(up_field[0]))))


def _bias3(bias) -> tuple[float, float, float]:
    """取 `(dx_mm, dy_mm, dtheta_deg)`。缺键/非有限 → ValueError。

    **不做缺省兜底**：ID29 在左甲板，偏置本来就不是零（人工量得 dy ≈ 55 mm）。
    缺键静默当 0 = 凭空造一个"码就在几何中心"的车体原点。
    """
    if not isinstance(bias, Mapping):
        raise ValueError(f"bias 必须是 dict（是 {type(bias).__name__}）")
    out = []
    for key in ("dx_mm", "dy_mm", "dtheta_deg"):
        if key not in bias:
            raise ValueError(f"bias 里缺 {key}（标定里的车体偏置必须三项齐全）")
        val = float(bias[key])
        if not math.isfinite(val):
            raise ValueError(f"bias[{key}] 不是有限数：{bias[key]!r}")
        out.append(val)
    return out[0], out[1], out[2]


def body_pose_in_field(ff: FieldFrame, pose, bias) -> tuple[float, float, float]:
    """车体原点在场地系里的 `(x_mm, y_mm, yaw_deg)`。

    pose  车载码（ID29）的 `MarkerPose`
    bias  `{"dx_mm": ..., "dy_mm": ..., "dtheta_deg": ...}`，A1 人工量，
          **在码/车体系里**（不是场地系）

    偏置先在码系里减掉（`p_body = p_marker − R @ [dx, dy, 0]`）再变到场地系：
    车载码贴在左甲板上，它相对车体原点的偏移是**跟着车头转**的。
    yaw = 码的航向 − dtheta（dtheta 是码相对车体的朝向偏置，故取负）。
    """
    dx_mm, dy_mm, dtheta_deg = _bias3(bias)
    rvec = _point3(getattr(pose, "rvec", None), "pose.rvec").reshape(3, 1)
    R_cam2marker, _ = cv2.Rodrigues(rvec)
    p_body_cam = _point3(getattr(pose, "tvec", None), "pose.tvec") - (
        R_cam2marker @ np.float64([dx_mm, dy_mm, 0.0]))
    p_field = to_field(ff, p_body_cam)
    return (float(p_field[0]), float(p_field[1]),
            _wrap180(heading_deg(R_cam2marker, ff) - dtheta_deg))


__all__ = [
    "COPLANAR_TOL_MM",
    "FieldFrame",
    "body_pose_in_field",
    "build_field_frame",
    "heading_deg",
    "to_field",
]
