"""ArUco 检出 + 单码位姿（solvePnP / IPPE_SQUARE）。

给下游（场地系、单帧解算、实车采图循环）提供两样东西：

    MarkerPose              单个码的相机系位姿（mm），含自检用的重投影 RMS
    detect_markers          一帧图里所有场地码的位姿；检不到就没有，绝不抛异常

## 标记坐标系（Task 3 的航向角直接依赖这一节）

物体点数组 `_ippe_object_points` 的四个点就是 `detectMarkers` 返回的四个角点
**按次序一一对应**（obj[i] ↔ corners_px[i]），由此定义标记自身的坐标系：

    +X   角点0 → 角点1  （码正面的"向右"）
    +Y   角点3 → 角点0  （码正面的"向上"）  ← **这就是"上轴"，Task 3 取它算航向**
    +Z   打印面的外法向（从纸背指向印字的那一面）

一个"正对相机"的码（`R = diag(1,-1,-1)`，即 `rvec=[π,0,0]`）因此解出：

    rvec ≈ [π, 0, 0]，法向 `R@[0,0,1] ≈ [0,0,-1]`，`R@[0,1,0] ≈ [0,-1,0]`（图像上方）

注意法向是 **−Z** 不是 +Z：相机 +Z 指向被摄物，**面向相机的物体，外法向必然指回相机，
也就是相机系的 −Z**。这一点搞反会把每个位姿翻 180°。

## 角点次序：是"规范次序"，不是图像的 TL/TR/BR/BL

`detectMarkers` 解码后会**跟着码走**：把码的位图旋转 90° 再检测，`corners[0]`
仍然指着码自己的左上角，而不是图像的左上角。本项目实拍照片上五个码里，
只有 ID20 的 `corners[0]` 恰好落在图像左上角，ID21/ID29 在左下、ID22/ID23 在右上。
所以 `corners_px` 不能按图像方位解释。

## 绝不抛异常

掉码、曝光炸掉、标定没加载，结果只能是"这一帧无解"（返回空 dict / 缺键）。
实车是按帧循环调用这里的，一个异常就能让整轮采图挂掉。

`solve_marker_pose` 本身**会**抛（它对外是个数学函数，退化输入应当显式失败）；
保证不抛的是 `detect_markers`，它把每一条都吞掉、发 warning、并返回已解出的那一部分。
"""
from __future__ import annotations

import warnings
from dataclasses import dataclass

import cv2
import numpy as np

from . import calib_store

# 场地码字典。必须显式指定：小 id 在 DICT_4X4_50/100/250/1000 里码型完全一样，
# 不钉死字典，id 就是有歧义的。
DICT_ID = cv2.aruco.DICT_4X4_50


@dataclass
class MarkerPose:
    """单个 ArUco 码的相机系位姿。

    rvec/tvec      形状 (3,1) 的 float64；tvec 单位 **mm**（相机系，z 向前）
    reproj_rms_px 四个角点的重投影 RMS（像素），自检用：模型对不上就会偏大
    corners_px     形状 (4,2) 的 float64，**规范次序**（同 detectMarkers：
                   角点跟着码走），不是图像的 TL/TR/BR/BL。详见模块 docstring。
    """

    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_rms_px: float
    corners_px: np.ndarray


class _BadCalib(Exception):
    """标定不可用（缺键 / 类型不对 / 值非法）。内部信号，不外泄给调用方。"""


class _BadImage(Exception):
    """图像不可用（无法判定取值范围等）。内部信号。"""


def _ippe_object_points(side_mm: float) -> np.ndarray:
    """SOLVEPNP_IPPE_SQUARE 的规范物体点，形状 (4,1,3)。

    与 `corners_px` **逐位对应**：obj[i] 就是第 i 个检测角点的坐标。
    实测（OpenCV 4.6.0）这个次序是唯一能用的：把 Y 取反换成图像 TL,TR,BR,BL
    再配规范次序的角点，解出来 RMS 是 8787px 的垃圾。

    坐标系定义见模块 docstring：+X 角点0→角点1，+Y 角点3→角点0（**上轴**），
    +Z 打印面外法向。
    """
    h = side_mm / 2.0
    return np.float64([
        [-h, h, 0.0],
        [h, h, 0.0],
        [h, -h, 0.0],
        [-h, -h, 0.0],
    ]).reshape(4, 1, 3)


def _reproj_rms(obj_pts, corners, rvec, tvec, K, dist) -> float:
    """四个角点的重投影 RMS（像素），实算，不抄求解器返回的 error。

    RMS = sqrt( Σ|投影 − 实测|² / 角点数 )。
    """
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    d = proj.reshape(4, 2) - corners
    return float(np.sqrt((d * d).sum(axis=1).mean()))


def solve_marker_pose(
    corners_px: np.ndarray,
    side_mm: float,
    K: list,
    dist: list,
) -> tuple[np.ndarray, np.ndarray, float]:
    """解单个码的位姿。返回 (rvec, tvec, reproj_rms_px)，形状 (3,1)、单位 mm。

    corners_px  : (4,2)，**规范次序**（同 detectMarkers，角点跟着码走）
    side_mm     : 码黑色有效区边长
    K, dist     : 相机内参与畸变系数
    退化输入（角点数不对、边长非正、内参形状不对、求解器无解）抛 ValueError。

    坐标系与"正对相机 = rvec[π,0,0]"的对应关系见模块 docstring。

    ## 为什么还要挑解

    IPPE_SQUARE 返回两个解，这里取**全部**解里 RMS 最小且 `tvec[2] > 0` 的那个，
    RMS 用 `cv2.projectPoints` 实算。**这是版本防御，不是修本机见到的 bug**：
    OpenCV 4.6.0 上随机扫 3000+ 个位姿，"取第 0 个解"、"argmin(求解器 error)"、
    "`cv2.solvePnP(SOLVEPNP_IPPE_SQUARE)`" 三者与本策略 0 次分歧 —— 它自己
    就把好解排在前面。但真实照片上两个解的 RMS 差 0.4~0.7 px（不是同一个解），
    一旦 OpenCV 改了排序或求解器返回别的顺序，错解就是 180° 镜像。代价几十纳秒。
    """
    corners = np.asarray(corners_px, dtype=np.float64).reshape(-1, 2)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError(f"角点必须是 4x2 有限值，实际 {corners.shape}")
    side = float(side_mm)
    if not np.isfinite(side) or side <= 0.0:
        raise ValueError(f"码边长必须是正的有限值，实际 {side_mm!r}")

    cam_mtx = np.asarray(K, dtype=np.float64)
    if cam_mtx.shape != (3, 3) or not np.isfinite(cam_mtx).all():
        raise ValueError(f"K 必须是 3x3 有限值，实际形状 {cam_mtx.shape}")
    coeffs = np.asarray(dist, dtype=np.float64).ravel()
    if coeffs.size not in (0, 4, 5, 8) or not np.isfinite(coeffs).all():
        raise ValueError(f"dist 必须是 0/4/5/8 个有限系数，实际 {coeffs.size}")

    obj_pts = _ippe_object_points(side)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj_pts, corners, cam_mtx, coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or len(rvecs) == 0:
        raise ValueError("solvePnPGeneric 没有解出来（退化角点？）")

    candidates = [
        (_reproj_rms(obj_pts, corners, rvecs[i], tvecs[i], cam_mtx, coeffs), i)
        for i in range(len(rvecs))
    ]
    in_front = [(rms, i) for i, (rms, _) in enumerate(candidates) if tvecs[i][2, 0] > 0.0]
    if in_front:
        rms, pick = min(in_front)
    else:
        # 实际上到不了：IPPE_SQUARE 的两个解共用同一个 tvec[2]，同正同负。
        # 留着是为了将来求解器行为变了不至于 IndexError；宁可给一个可疑解，
        # 也不能静默返回空。
        rms, pick = min(candidates)
        warnings.warn(
            "solvePnPGeneric 的所有解都在相机后方（z<=0），已退回 RMS 最小的那个；"
            "位姿可疑，检查角点次序或图像。",
            RuntimeWarning,
            stacklevel=2,
        )

    return (np.asarray(rvecs[pick], dtype=np.float64).reshape(3, 1),
            np.asarray(tvecs[pick], dtype=np.float64).reshape(3, 1),
            rms)


# --- 标定取值：所有"不可用"都归一成 _BadCalib，调用方只判一次 ----------------------


def _need_dict(calib) -> dict:
    if not isinstance(calib, dict):
        raise _BadCalib(f"标定不是 dict（是 {type(calib).__name__}）")
    return calib


def _need_num(calib: dict, key: str) -> float:
    try:
        val = float(calib[key])
    except (KeyError, TypeError, ValueError):
        raise _BadCalib(f"标定里 {key} 缺失或不是数") from None
    if not np.isfinite(val):
        raise _BadCalib(f"标定里 {key} 不是有限数：{val}")
    return val


def _need_matrix(calib: dict, key: str, shape: tuple) -> np.ndarray:
    try:
        arr = np.asarray(calib[key], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        raise _BadCalib(f"标定里 {key} 缺失或无法转成数组") from None
    if arr.shape != shape or not np.isfinite(arr).all():
        raise _BadCalib(f"标定里 {key} 形状应为 {shape} 且全有限，实际 {arr.shape}")
    return arr


def _need_coeffs(calib: dict, key: str) -> np.ndarray:
    try:
        arr = np.asarray(calib[key], dtype=np.float64).ravel()
    except (KeyError, TypeError, ValueError):
        raise _BadCalib(f"标定里 {key} 缺失或无法转成一维数组") from None
    if arr.size not in (0, 4, 5, 8) or not np.isfinite(arr).all():
        raise _BadCalib(f"标定里 {key} 应是 0/4/5/8 个有限系数，实际 {arr.size}")
    return arr


def _calib_for_detection(calib) -> tuple[np.ndarray, np.ndarray, dict[int, float]]:
    """从 calib 取出 (K, dist, {id: 码边长})。不可用就抛 _BadCalib。

    按 id 查边长：20~23 → boundary_side_mm，29 → vehicle_side_mm。
    字典外的 id 不在这里 —— 码表没给它的边长，猜一个等于凭空造一个位姿。
    **不做兜底**：标定缺了就该报无解，静默套用内建默认值等于拿可能过期的 K 出数。
    """
    _need_dict(calib)
    cam_mtx = _need_matrix(calib, "K", (3, 3))
    coeffs = _need_coeffs(calib, "dist")
    boundary = _need_num(calib, "boundary_side_mm")
    vehicle = _need_num(calib, "vehicle_side_mm")
    if boundary <= 0 or vehicle <= 0:
        raise _BadCalib(f"码边长必须为正，实际 boundary={boundary} vehicle={vehicle}")
    sides = {int(i): boundary for i in calib_store.BOUNDARY_IDS}
    sides[int(calib_store.VEHICLE_ID)] = vehicle
    return cam_mtx, coeffs, sides


# --- 图像归一化 ------------------------------------------------------------------


def _to_uint8(arr: np.ndarray) -> np.ndarray:
    """把非 uint8 的数组变成可检测的 uint8。范围判不了就抛 _BadImage。

    浮点图有两种常见约定：[0,1] 的缩放图，和 [0,255] 直接存的值。两者必须分开：
    盲乘 `convertScaleAbs(alpha=255)` 会把 [0,255] 的图压成全白（实测均值 250.6，
    一个码都检不出）。而"图坏了检不出"和"画面里本来就没有码"长得一模一样 ——
    那会**静默废掉一整轮实验**，比抛异常糟得多。

    已知歧义：一张 max<=1.0 的 [0,255] 图会被当成 [0,1]。但那种图已经黑到没有
    任何码，不构成实际风险。
    """
    lo, hi = float(np.nanmin(arr)), float(np.nanmax(arr))
    if not (np.isfinite(lo) and np.isfinite(hi)):
        raise _BadImage("图像里有 NaN/Inf")
    if hi <= 1.0:
        return np.clip(arr * 255.0, 0.0, 255.0).astype(np.uint8)   # [0,1] 缩放图
    if hi <= 255.0:
        return np.clip(arr, 0.0, 255.0).astype(np.uint8)           # [0,255] 浮点图
    raise _BadImage(f"图像取值范围 [{lo:.3g}, {hi:.3g}] 既不是 [0,1] 也不是 [0,255]")


def _to_gray(img_bgr) -> np.ndarray | None:
    """转灰度。None / 维度不对 → None；范围判不了 → 抛 _BadImage。

    接受 BGR / BGRA / 单通道灰度 / 非 uint8。
    """
    if img_bgr is None:
        return None
    arr = np.asarray(img_bgr)
    if arr.ndim == 2:
        gray = arr
    elif arr.ndim == 3 and arr.shape[2] in (1, 3, 4):
        if arr.shape[2] == 1:
            arr = arr[:, :, 0]
        if arr.dtype != np.uint8:
            arr = _to_uint8(arr)
        code = cv2.COLOR_BGR2GRAY if arr.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
        gray = cv2.cvtColor(arr, code)
    else:
        return None
    return gray if gray.dtype == np.uint8 else _to_uint8(gray)


def detect_markers(img_bgr, calib: dict) -> dict[int, MarkerPose]:
    """检出一帧图里的全部场地码并解位姿。

    返回 {marker_id: MarkerPose}。**检不出的码就不会出现在返回值里**（缺键或空
    dict），**任何情况下都不抛异常** —— 掉码、坏图、坏标定一律等于"这一帧无解"，
    并各发一条 RuntimeWarning 说明原因。字典外的 id（不是 20~23、也不是 29）
    静默跳过。

    允许 img_bgr 是 BGR / BGRA / 灰度 / 浮点图（[0,1] 或 [0,255] 都认）；
    None 和维度不对的图返回 {}。
    """
    poses: dict[int, MarkerPose] = {}
    try:
        gray = _to_gray(img_bgr)
        cam_mtx, coeffs, sides = _calib_for_detection(calib)
        if gray is None:
            warnings.warn("detect_markers 收到 None 或维度不对的图，本帧无解。",
                          RuntimeWarning, stacklevel=2)
            return poses

        dictionary = cv2.aruco.Dictionary_get(DICT_ID)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is None:
            return poses

        for quad, raw_id in zip(corners, np.asarray(ids).ravel()):
            marker_id = int(raw_id)
            side = sides.get(marker_id)
            if side is None:
                continue  # 字典外的码：跳过
            try:
                pts = np.asarray(quad, dtype=np.float64).reshape(4, 2)
                rvec, tvec, rms = solve_marker_pose(pts, side, cam_mtx, coeffs)
            except (cv2.error, ValueError):
                continue  # 这一个码解不出来，不影响其它码
            poses[marker_id] = MarkerPose(
                marker_id=marker_id,
                rvec=rvec,
                tvec=tvec,
                reproj_rms_px=rms,
                corners_px=pts,
            )
    except MemoryError:
        raise  # 内存耗尽时返回半帧结果比崩掉更糟，不该被这条兜底吞掉
    except (_BadCalib, _BadImage) as exc:
        warnings.warn(f"detect_markers 本帧无解：{exc}", RuntimeWarning, stacklevel=2)
    except Exception as exc:  # 兜底：实车按帧循环调用，一个异常就能让整轮采图挂掉
        warnings.warn(
            f"detect_markers 收尾异常 {type(exc).__name__}: {exc}；"
            f"已返回解出的 {len(poses)} 个码。",
            RuntimeWarning,
            stacklevel=2,
        )
    return poses


__all__ = ["DICT_ID", "MarkerPose", "detect_markers", "solve_marker_pose"]
