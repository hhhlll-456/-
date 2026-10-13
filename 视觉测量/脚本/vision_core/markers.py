"""ArUco 检出 + 单码位姿（solvePnP / IPPE_SQUARE）。

给下游（场地系、单帧解算、实车采图循环）提供两样东西：

    MarkerPose              单个码的相机系位姿（mm），含自检用的重投影 RMS
    detect_markers          一帧图里所有场地码的位姿；检不到就没有，绝不抛异常

**两条硬规矩**（改代码前先读，都是踩过的坑）：

1. **角点次序**：本模块的 `solve_marker_pose` 收的角点一律按 `detectMarkers`
   返回的次序 **TL,TR,BR,BL**（OpenCV 文档口径），`MarkerPose.corners_px` 也是
   这个次序。而 `SOLVEPNP_IPPE_SQUARE` 的物体点规范次序是 **BL,BR,TR,TL**，
   所以求解前必须把角点反序（见 `_IPPE_ORDER`）。
   配错的后果很阴：不报错、重投影 RMS 照样很小（噪声图上实测 0.23px），
   但解出来是**翻面**位姿 —— 码的正面法向指向 z=-1，也就是“纸面背对相机”。
   位移数值几乎不变，唯独转角差 180°，Task 3 的航向角会整个翻掉。
2. **绝不抛异常**：掉码、曝光炸掉、标定没加载，结果只能是“这一帧无解”
   （返回空 dict / 缺键）。实车是按帧循环调用这里的，一个异常就能让整轮采图挂掉。

`solve_marker_pose` 本身**会**抛（它对外是个数学函数，退化输入应当显式失败）；
保证不抛的是 `detect_markers`，它把每一条都吞掉并返回已解出的那一部分。
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

# 上面第 1 条：detectMarkers 给 TL,TR,BR,BL，IPPE_SQUARE 要 BL,BR,TR,TL。
_IPPE_ORDER = [3, 2, 1, 0]


@dataclass
class MarkerPose:
    """单个 ArUco 码的相机系位姿。

    rvec/tvec      形状 (3,1) 的 float64；tvec 单位 **mm**（相机系，z 向前）
    reproj_rms_px 四个角点的重投影 RMS（像素），自检用：模型对不上就会偏大
    corners_px     形状 (4,2) 的 float64，次序 **TL,TR,BR,BL**（同 detectMarkers）
    """

    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_rms_px: float
    corners_px: np.ndarray


def _ippe_object_points(side_mm: float) -> np.ndarray:
    """SOLVEPNP_IPPE_SQUARE 的规范物体点，次序 BL,BR,TR,TL，形状 (4,1,3)。

    用的是简报给的字面值。实测（OpenCV 4.6.0）：`SOLVEPNP_IPPE_SQUARE` 只吃这个
    次序 —— 把 Y 取反换成 TL,TR,BR,BL 再配 TL,TR,BR,BL 角点，解出来 RMS 是
    8787px 的垃圾。所以想改这里只能连 `_IPPE_ORDER` 一起改。
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

    corners_px  : (4,2)，次序 **TL,TR,BR,BL**（detectMarkers 的原样次序）
    side_mm     : 码黑色有效区边长
    K, dist     : 相机内参与畸变系数
    退化输入（角点数不对、边长非正、内参形状不对、求解器无解）抛 ValueError。

    IPPE_SQUARE 有两个解，取**全部**解里 RMS 最小且 `tvec[2] > 0` 的那个
    （Review Focus #5）。挑错解会得到镜像/翻面的位姿，位移和转角一起错。
    两个解都 z<=0 时退而取 RMS 最小的那个，并把 warning 抛出来 —— 宁可给一个
    可疑的解，也不能静默返回空。
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

    # TL,TR,BR,BL -> BL,BR,TR,TL（IPPE_SQUARE 规范次序，见模块 docstring 第 1 条）
    ippe_corners = corners[_IPPE_ORDER]
    obj_pts = _ippe_object_points(side)

    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj_pts, ippe_corners, cam_mtx, coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or len(rvecs) == 0:
        raise ValueError("solvePnPGeneric 没有解出来（退化角点？）")

    candidates = [
        (float(tvecs[i][2, 0]), _reproj_rms(obj_pts, ippe_corners, rvecs[i], tvecs[i],
                                            cam_mtx, coeffs), i)
        for i in range(len(rvecs))
    ]
    in_front = [c for c in candidates if c[0] > 0.0]
    if in_front:
        _, _, pick = min(in_front, key=lambda c: c[1])
    else:
        _, _, pick = min(candidates, key=lambda c: c[1])
        warnings.warn(
            "solvePnPGeneric 的所有解都在相机后方（z<=0），已退回 RMS 最小的那个；"
            "位姿可疑，检查角点次序或图像。",
            RuntimeWarning,
            stacklevel=2,
        )

    rvec = np.asarray(rvecs[pick], dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(tvecs[pick], dtype=np.float64).reshape(3, 1)
    return rvec, tvec, _reproj_rms(obj_pts, ippe_corners, rvec, tvec, cam_mtx, coeffs)


def _side_table(calib: dict) -> dict[int, float] | None:
    """按 id 查码边长（20~23 → boundary_side_mm，29 → vehicle_side_mm）。

    返回 None 表示“标定不可用”，调用方据此判定这一帧无解。**不做兜底**：
    标定缺了就该报无解，静默套用内建默认值等于拿可能过期的 K 去出数。
    """
    if not isinstance(calib, dict):
        return None
    try:
        boundary = float(calib["boundary_side_mm"])
        vehicle = float(calib["vehicle_side_mm"])
    except (KeyError, TypeError, ValueError):
        return None
    if not (np.isfinite(boundary) and np.isfinite(vehicle)) or boundary <= 0 or vehicle <= 0:
        return None
    table = {int(i): boundary for i in calib_store.BOUNDARY_IDS}
    table[int(calib_store.VEHICLE_ID)] = vehicle
    return table


def _as_camera(calib: dict):
    """取 (K, dist)；标定不可用时返回 (None, None)。"""
    if not isinstance(calib, dict):
        return None, None
    try:
        cam_mtx = np.asarray(calib["K"], dtype=np.float64)
        coeffs = np.asarray(calib["dist"], dtype=np.float64).ravel()
    except (KeyError, TypeError, ValueError):
        return None, None
    if cam_mtx.shape != (3, 3) or not np.isfinite(cam_mtx).all():
        return None, None
    if coeffs.size not in (0, 4, 5, 8) or not np.isfinite(coeffs).all():
        return None, None
    return cam_mtx, coeffs


def _to_gray(img_bgr) -> np.ndarray | None:
    """转灰度。已经是单通道的直接用；图都没有就返回 None（这一帧无解）。

    非 uint8 的图（float32/float64 缩放图）先转 uint8 —— cvtColor 不吃 CV_64F，
    直接喂进去会抛 cv2.error。
    """
    if img_bgr is None:
        return None
    arr = np.asarray(img_bgr)
    if arr.ndim == 2:
        return arr
    if arr.ndim != 3 or arr.shape[2] not in (1, 3, 4):
        return None
    if arr.dtype != np.uint8:
        arr = cv2.convertScaleAbs(arr, alpha=255.0) if arr.dtype.kind == "f" \
            else cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    if arr.shape[2] == 1:
        return arr[:, :, 0]
    code = cv2.COLOR_BGR2GRAY if arr.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
    return cv2.cvtColor(arr, code)


def detect_markers(img_bgr, calib: dict) -> dict[int, MarkerPose]:
    """检出一帧图里的全部场地码并解位姿。

    返回 {marker_id: MarkerPose}。**检不出的码就不会出现在返回值里**（缺键或空
    dict），**任何情况下都不抛异常** —— 掉码、坏图、坏标定一律等于“这一帧无解”。
    字典外的 id（不是 20~23、也不是 29）静默跳过：码表没给它的边长，猜一个边长
    等于凭空造一个位姿。

    允许 img_bgr 是 BGR / BGRA / 灰度 / 非 uint8；None 和维度不对的图返回 {}。
    """
    poses: dict[int, MarkerPose] = {}
    try:
        gray = _to_gray(img_bgr)
        cam_mtx, coeffs = _as_camera(calib)
        sides = _side_table(calib)
        if gray is None or cam_mtx is None or sides is None:
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
    except Exception as exc:  # 兜底：实车按帧循环调用，一个异常就能让整轮采图挂掉
        warnings.warn(
            f"detect_markers 收尾异常 {type(exc).__name__}: {exc}；"
            f"已返回解出的 {len(poses)} 个码。",
            RuntimeWarning,
            stacklevel=2,
        )
    return poses


__all__ = ["DICT_ID", "MarkerPose", "detect_markers", "solve_marker_pose"]
