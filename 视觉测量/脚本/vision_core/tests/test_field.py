"""场地系构建 + 车体偏置修正的行为锁。

四条最贵的规格，都是出过一次问题（或差点出问题）才写下来的：

1. **原点在 4 个边界码的质心，不在 ID20（TL）。** 计划里 Interfaces 与 Step 3
   两处都写质心，只有三个数字阈值（`norm(tl) < 1.0` / `tr[0] > 400` /
   `bl[1] > 200`）是按 TL 原点写的，自相矛盾。质心赢：spec §6.2、计划接口行、
   计划 Step 3 第 1 条、以及 Task 9 的"场地半长 252.5 / 半宽 125.9"
   （只有中心坐标系才有意义）。实测四个码落在 (±252, ±126)，见
   `test_boundary_markers_land_on_the_expected_half_extents`。
2. **航向角是"从上往下看逆时针为正"，零点是场地 +X（TL→TR），不是图像 up。**
   计划把零基准写成"图像 up = 0°"，但图像 up 与场地 +X 之间差着一个非零角
   （实测 −87.2°），而且零点必须在场地系里，否则 Task 12 没法把
   `yaw_end − yaw_start` 和 `odometry.integrate_encoders` 的 yaw 相减。
   正负号见 `test_heading_increases_with_ccw_rotation_seen_from_above`。
3. **场地系只由 4 个边界码的**位置**决定，绝不读它们的姿态**（R27）。ID20 的
   解出来离"正对相机"差 63.6°（法向对齐度 0.445、重投影 RMS 0.846 px 全场最差、
   上边离画面边界只有 6 px），用它定场地法向会把整个场地系拧歪，之后每一条实验
   的真值全错。由 `test_field_frame_ignores_boundary_marker_rotations` 直接钉住。
4. **场地 +Z 朝桌面里，平面法向朝相机，两者反号。** 计划的 X（TL→TR）与
   Y（朝 ID23）都躺在像平面里，`z = cross(x, y)` 必然指向相机 +Z，也就是
   **扎进桌面**；而计划要求的法向 n 指向相机。于是 `z_axis = −n`，桌面上方
   （相机一侧）的点拿到的是**负**的 field z。由
   `test_to_field_z_is_negative_above_the_table` 钉住。

偏置在**车体/码自身系**里，不是场地系：`p_body = p_marker − R @ [dx, dy, 0]`。
由 `test_body_bias_is_expressed_in_the_marker_frame` 钉住 —— 车转 90°，
偏置造成的位移也跟着转 90°；若错写成场地系偏置，这个位移纹丝不动，
而位移正是 Task 4/10/11/12 唯一在意的输出。
"""
import warnings

import cv2
import numpy as np
import pytest

from vision_core import calib_store
from vision_core.field import (
    COPLANAR_TOL_MM,
    FieldFrame,
    body_pose_in_field,
    build_field_frame,
    heading_deg,
    to_field,
)
from vision_core.markers import MarkerPose, detect_markers

PHOTO = "视觉测量/照片/photo_review_20261013.jpg"

TL_ID, TR_ID, BR_ID, BL_ID = calib_store.BOUNDARY_IDS   # 20, 21, 22, 23
VEHICLE_ID = calib_store.VEHICLE_ID                    # 29

# 四个边界码在场地系里的名义位置（mm）。Task 9 的 assert_within_field 用的是
# 同一对数字（半长 252.5 / 半宽 125.9），差 0.5 mm 是它自己四舍五入留的余量。
NOMINAL = {
    TL_ID: (-252.0, -126.0),
    TR_ID: (+252.0, -126.0),
    BR_ID: (+252.0, +126.0),
    BL_ID: (-252.0, +126.0),
}

# 容差 ±20 mm。相机不是严格垂直俯视，四个码的质心高度实测 510.4~521.8 mm
# （相差 11 mm），平面拟合出来的法向带一点倾角，于是长边方向的 y 会偏几个毫米：
# 实测最大偏差是 ID23 的 y = +121.7（名义 126，差 4.3），TL/TR 侧 ±1 mm 以内。
# 这里不追求更紧 —— 收紧只会把"相机没装正"这件无害的事变成回归。
TOL_MM = 20.0

_POSES: dict[int, MarkerPose] | None = None

# **占位数字，不是 A1 的测量结果。** 它抄自计划 Task 1 的测试夹具，只是为了让本文件
# 里的偏置用例有一个具体数值。真实偏置由 Task 4 的人工步骤 A1 量，写进
# `视觉测量/结果/car_calib.json` —— 那个文件目前**不存在**，
# `calib_store.load_calib()["bias"]` 三项都还是 0.0。
# A1 做完之后，本占位数就该被真实值取代（或者整条用例改成读标定文件）。
PLACEHOLDER_DY_MM = 55.0


def _bias(dy_mm: float = 0.0, dx_mm: float = 0.0, dtheta_deg: float = 0.0) -> dict:
    """bias 字面量。dx/dy 沿**码的规范轴**（+X = 角点0→角点1，+Y = 角点3→角点0）。"""
    return {"dx_mm": dx_mm, "dy_mm": dy_mm, "dtheta_deg": dtheta_deg}


def _poses() -> dict[int, MarkerPose]:
    """在实拍照片上解一遍 5 个码（整份文件只跑一次，检测要几十毫秒）。"""
    global _POSES
    if _POSES is None:
        path = calib_store.resolve_path(PHOTO)
        img = cv2.imread(str(path))
        assert img is not None, f"读不到测试照片：{path}"
        _POSES = detect_markers(img, calib_store.load_calib())
    return dict(_POSES)


def _identity_ff(origin=(0.0, 0.0, 0.0)) -> FieldFrame:
    """恒等场地系：场地三轴就是相机三轴。合成航向测试专用。

    真实照片建出来的场地轴与图像差着一个 −87.2° 的角，拿它测"航向 = 0"
    根本测不出定义，只能测出偏差；恒等场地系下，航向就是它字面上的意思。
    """
    return FieldFrame(
        R_cam2field=np.eye(3),
        origin_cam=np.float64(origin),
        marker_px={},
        normal_cam=np.float64([0.0, 0.0, -1.0]),
    )


def _marker_R(yaw_deg: float) -> np.ndarray:
    """规范 +Y（up）轴指向"航向 = yaw_deg"的那个码的正交矩阵（恒等场地系下）。

    up 在场地系里是 `(cos y, −sin y, 0)`：负号来自场地 +Y 是**顺时针**方向
    （X=TL→TR、Y=朝 BL，在像平面里从右转到下就是顺时针），而航向定义要
    **逆时针**为正。
    """
    t = np.radians(float(yaw_deg))
    c, s = np.cos(t), np.sin(t)
    return np.float64([[-s, c, 0.0],
                       [-c, -s, 0.0],
                       [0.0, 0.0, 1.0]])


def _pose(t_cam, R_cam2marker, marker_id: int = VEHICLE_ID) -> MarkerPose:
    """造一个只用来喂数学函数的 MarkerPose（角点/RMS 不参与计算）。"""
    rvec, _ = cv2.Rodrigues(np.asarray(R_cam2marker, dtype=np.float64))
    return MarkerPose(
        marker_id=marker_id,
        rvec=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        tvec=np.asarray(t_cam, dtype=np.float64).reshape(3, 1),
        reproj_rms_px=0.0,
        corners_px=np.zeros((4, 2), dtype=np.float64),
    )


def _shift(ff, pose, bias_a, bias_b):
    """两次 body_pose_in_field 的差（mm / deg）。"""
    a = body_pose_in_field(ff, pose, bias_a)
    b = body_pose_in_field(ff, pose, bias_b)
    return np.float64([b[0] - a[0], b[1] - a[1], b[2] - a[2]])


def _wrap180(deg: float) -> float:
    """测试侧的 (−180, 180]。与 field._wrap180 同一约定，用来写期望值。"""
    wrapped = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


# --- build_field_frame：完整性 ------------------------------------------------------


@pytest.mark.parametrize("missing", list(calib_store.BOUNDARY_IDS))
def test_build_field_frame_requires_all_four_boundary_markers(missing):
    """少任何一个边界码都必须抛 ValueError，不能"少一个也能凑合"。

    少一个码的场地系是**错的**，而且不报错 —— 后面每条实验的位移和转角跟着一起错。
    """
    poses = _poses()
    assert missing in poses
    poses.pop(missing)
    with pytest.raises(ValueError, match=str(missing)):
        build_field_frame(poses)


def test_build_field_frame_rejects_empty_and_malformed_input():
    with pytest.raises(ValueError):
        build_field_frame({})
    with pytest.raises(ValueError):
        build_field_frame(None)
    with pytest.raises(ValueError):
        build_field_frame({i: object() for i in calib_store.BOUNDARY_IDS})


@pytest.mark.parametrize("half_span_mm", [0.0, 0.5], ids=["挤成一点", "只有0.5mm宽"])
def test_build_field_frame_rejects_degenerate_geometry(half_span_mm):
    """四个码挤成一团 → 场地平面无定义，必须抛而不是返回一个垃圾场地系。

    `match` 钉的是**跨度守卫**自己的话（"4 个边界码退化"），不是 `_unit()` 那句
    泛泛的"向量退化"：0.5 mm 的场在删掉跨度守卫后会一路走到 ID23 的 y 轴检查，
    报出"ID23 落在 ID20→ID21 直线上，场地 +Y 无定义" —— 方向完全误导（真实病因是
    四个码挤成一团），所以这条用例必须因此变红。
    """
    h = half_span_mm / 2.0
    corners = {20: (-h, -h), 21: (+h, -h), 22: (+h, +h), 23: (-h, +h)}
    poses = {i: _pose([x, y, 500.0], np.eye(3), marker_id=i) for i, (x, y) in corners.items()}
    with pytest.raises(ValueError, match="4 个边界码退化"):
        build_field_frame(poses)


def test_build_field_frame_ignores_boundary_marker_rotations():
    """R27：场地系只由 4 个边界码的**位置**决定。

    把四个边界码的姿态全部换成 `rvec=0`（R = 单位阵，对俯视相机而言是"码躺着
    朝天"，和真实姿态差 90°），场地系必须**一模一样**。只要实现里读了一次
    `pose.rvec`，这条就红。ID20 的真实姿态本身就离正对相机差 63.6°，用它定法向
    会把整个场地系拧歪，之后每一条实验的真值全错且无人报错。
    """
    poses = _poses()
    real = build_field_frame(poses)

    poisoned = {
        mid: MarkerPose(marker_id=p.marker_id,
                        rvec=np.zeros((3, 1), dtype=np.float64),  # 垃圾姿态
                        tvec=p.tvec,
                        reproj_rms_px=p.reproj_rms_px,
                        corners_px=p.corners_px)
        for mid, p in poses.items()
    }
    dirty = build_field_frame(poisoned)

    assert np.allclose(real.R_cam2field, dirty.R_cam2field)
    assert np.allclose(real.origin_cam, dirty.origin_cam)
    assert np.allclose(real.normal_cam, dirty.normal_cam)


# --- 场地系的定义 ------------------------------------------------------------------


def test_field_origin_is_centroid_of_the_four_boundary_markers():
    """R4：原点在 4 个边界码的**质心**（不是 ID20/TL）。"""
    poses = _poses()
    ff = build_field_frame(poses)

    centroid_field = np.mean([to_field(ff, poses[i].tvec.reshape(3))[:2]
                              for i in calib_store.BOUNDARY_IDS], axis=0)
    assert np.allclose(centroid_field, [0.0, 0.0], atol=1e-6), centroid_field

    assert np.allclose(ff.origin_cam,
                       np.mean([poses[i].tvec.reshape(3) for i in calib_store.BOUNDARY_IDS],
                               axis=0),
                       atol=1e-9)

    tl = to_field(ff, poses[TL_ID].tvec.reshape(3))
    assert np.linalg.norm(tl[:2]) > 200.0, (
        f"TL 落在场地系原点附近（{tl}），说明原点又退回 TL 了")


def test_boundary_markers_land_on_the_expected_half_extents():
    """R4：质心为原点时，四个码落在 (±252, ±126) 附近，X 朝 TR、Y 朝 BL。"""
    poses = _poses()
    ff = build_field_frame(poses)
    got = {i: to_field(ff, poses[i].tvec.reshape(3)) for i in calib_store.BOUNDARY_IDS}

    for mid, (x_mm, y_mm) in NOMINAL.items():
        assert abs(got[mid][0] - x_mm) <= TOL_MM, f"ID{mid} x={got[mid][0]:.1f}"
        assert abs(got[mid][1] - y_mm) <= TOL_MM, f"ID{mid} y={got[mid][1]:.1f}"

    # X 轴 = ID20→ID21：TR 必须既远又在负 y 一侧；BL 必须既远又在正 y 一侧。
    assert got[TR_ID][0] > 200.0
    assert got[BL_ID][1] > 100.0
    # 场地 ≈ 505 × 252 mm，四个码几乎在同一个平面上（z 就是平面的离面残差）。
    assert max(abs(got[i][2]) for i in calib_store.BOUNDARY_IDS) < 5.0


def test_field_axes_are_right_handed_and_z_points_into_the_table():
    """场地三轴正交、右手（det=+1），且 +Z = cross(x, y) 扎进桌面、与法向反号。"""
    ff = build_field_frame(_poses())
    R = ff.R_cam2field
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert float(np.linalg.det(R)) == pytest.approx(1.0, abs=1e-12)

    x_axis, y_axis, z_axis = R[0], R[1], R[2]
    assert np.allclose(np.cross(x_axis, y_axis), z_axis, atol=1e-12)
    # x 朝 ID21、y 朝 ID23：符号由位置定，不是由 cross 的手性碰运气。
    poses = _poses()
    assert x_axis @ (poses[TR_ID].tvec.reshape(3) - poses[TL_ID].tvec.reshape(3)) > 0
    assert y_axis @ (poses[BL_ID].tvec.reshape(3) - poses[TL_ID].tvec.reshape(3)) > 0

    # 计划要求法向"指向相机"，俯视相机就是 z 分量为负；z 轴因此与它反号。
    assert float(z_axis @ ff.normal_cam) == pytest.approx(-1.0, abs=1e-12)


def test_plane_normal_points_toward_the_camera():
    """R33：法向 n 指向相机。本机近乎正俯视，所以等价于**负**相机 Z 分量。

    注意这一条里 `normal_cam[2] < 0` 是**本机的性质**，不是规则本身；规则是
    `n · origin_cam < 0`（法向背朝被摄点、朝向相机中心），对装了歪的相机照样成立
    —— 见下面那条合成用例。
    """
    ff = build_field_frame(_poses())
    assert ff.normal_cam[2] < 0.0, ff.normal_cam
    # 与相机光轴方向相反：n 朝相机中心（相机中心在相机系原点）。
    assert float(ff.normal_cam @ ff.origin_cam) < 0.0
    assert float(np.linalg.norm(ff.normal_cam)) == pytest.approx(1.0, abs=1e-12)


def _tilted_field_poses():
    """合成一个近乎**侧视**的场地：真法向与光轴夹 82°，`normal_cam[2] > 0`。

    相机装歪到 82° 时，"指向相机"的法向 z 分量**是正的**，于是本机那种
    "n[2] > 0 就翻号"的朴素规则会把它翻反。四个码都在相机前方（z 250~750 mm），
    是真实相机拍得到的构型。
    """
    n_true = np.float64([0.99, 0.0, 0.14])
    n_true /= np.linalg.norm(n_true)
    centre = np.float64([-100.0, 0.0, 500.0])          # n_true · centre = -29 < 0
    e_x = np.float64([0.0, 0.0, 1.0])
    e_x -= n_true * float(e_x @ n_true)                # 场地 +X 方向（平面内）
    e_x /= np.linalg.norm(e_x)
    e_y = np.cross(n_true, e_x)
    corners = {
        TL_ID: centre - 252.0 * e_x - 126.0 * e_y,
        TR_ID: centre + 252.0 * e_x - 126.0 * e_y,
        BR_ID: centre + 252.0 * e_x + 126.0 * e_y,
        BL_ID: centre - 252.0 * e_x + 126.0 * e_y,
    }
    poses = {mid: _pose(p, np.eye(3), marker_id=mid) for mid, p in corners.items()}
    return poses, n_true


def test_plane_normal_sign_survives_a_tilted_camera():
    """M3：法向符号规则是 `n · origin_cam < 0`，**不是** `n[2] < 0`。

    本机近乎正俯视，两条规则结果完全一样，所以真实照片上的任何测试都分不开它们。
    这里用一个法向与光轴夹 82° 的合成场地（真法向 z 分量 **+0.14**）把它们分开：
    朴素规则给出的法向 `n · origin = +29`，正对着桌子外侧 —— 即背对相机。
    """
    poses, n_true = _tilted_field_poses()
    ff = build_field_frame(poses)

    assert float(ff.normal_cam @ n_true) == pytest.approx(1.0, abs=1e-9), (
        f"法向翻反了：{ff.normal_cam} vs 真值 {n_true}")
    assert float(ff.normal_cam @ ff.origin_cam) < 0.0
    # 这条构型里"指向相机"的法向 z 分量确实为正，所以它分得开两条规则。
    assert float(ff.normal_cam[2]) > 0.1, ff.normal_cam


def test_to_field_z_is_negative_above_the_table():
    """R33：`to_field` 的 z 分量符号必须显式钉住 —— 桌面上方（相机侧）是**负**。

    四个边界码（z≈±0.83 mm）分不出正负，拿车载码 ID29 才能测出来：它悬在桌面上方
    约 78.6 mm，属于相机一侧，field z 实测 −79.1 mm。符号搞反不会让任何 NaN 出现，
    只会让后面所有"离桌面多高"的推断悄悄错掉。
    """
    poses = _poses()
    ff = build_field_frame(poses)
    p29 = poses[VEHICLE_ID].tvec.reshape(3)
    z29 = to_field(ff, p29)[2]
    assert z29 < -50.0, f"ID29 在桌面上方，field z 应该是负的，实测 {z29:.1f}"
    assert float(z29) == pytest.approx(
        float(-(ff.normal_cam @ (p29 - ff.origin_cam))), abs=1e-9)
    # 原点自己就在平面上。
    assert np.allclose(to_field(ff, ff.origin_cam), [0.0, 0.0, 0.0], atol=1e-9)


def test_marker_px_holds_boundary_corner_centroids():
    """`marker_px` 是 4 个边界码角点的均值 —— Task 10 拿它做"码被撞移位"的基线。"""
    poses = _poses()
    ff = build_field_frame(poses)
    assert set(ff.marker_px) == set(calib_store.BOUNDARY_IDS)
    for mid, px in ff.marker_px.items():
        assert px.shape == (2,)
        assert np.allclose(px, poses[mid].corners_px.reshape(4, 2).mean(axis=0))
        # 是**图像**坐标，不可能是场地系毫米坐标。
        assert 0.0 <= px[0] <= 1920.0 and 0.0 <= px[1] <= 1080.0


@pytest.mark.parametrize("lift_mm", [20.0, 60.0])
def test_build_field_frame_warns_when_boundary_markers_are_not_coplanar(lift_mm):
    """某个码被撞得离桌面几十毫米 → 平面拟合歪掉，必须喊出来。

    抬起量到离面残差是**次线性**的：四角矩形的最小二乘平面会把一个角抬起的量
    摊到四个角上（实测抬起 4.5/15/25/40/60 mm → 残差 1.99/4.70/7.27/11.09/16.09 mm），
    5 mm 阈值对应的抬起量约 17 mm，所以阈值两侧各留一条用例。

    别去断言"消息里点名哪个码"：抬起一个角之后四角残差只差 0.01 mm
    （抬起 25 mm 时 ID20 7.261 vs ID23 7.270），点谁的名是浮点噪声。
    """
    poses = _poses()
    tl = poses[TL_ID].tvec.reshape(3) + np.float64([0.0, 0.0, lift_mm])
    poses[TL_ID] = MarkerPose(TL_ID, poses[TL_ID].rvec, tl.reshape(3, 1),
                              poses[TL_ID].reproj_rms_px, poses[TL_ID].corners_px)
    with pytest.warns(RuntimeWarning, match="不共面") as caught:
        build_field_frame(poses)
    assert f"> {COPLANAR_TOL_MM} mm" in str(caught[0].message), caught[0].message


def test_build_field_frame_stays_quiet_just_below_the_coplanar_tolerance():
    """阈值另一侧：离面残差 < `COPLANAR_TOL_MM` 时不该喊（否则就是狼来了）。"""
    poses = _poses()
    lift = 4.5                                  # → 残差 1.99 mm，低于 5.0
    tl = poses[TL_ID].tvec.reshape(3) + np.float64([0.0, 0.0, lift])
    poses[TL_ID] = MarkerPose(TL_ID, poses[TL_ID].rvec, tl.reshape(3, 1),
                              poses[TL_ID].reproj_rms_px, poses[TL_ID].corners_px)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        build_field_frame(poses)


def test_to_field_rejects_malformed_points():
    ff = build_field_frame(_poses())
    for bad in ([1.0, 2.0], [1.0, 2.0, 3.0, 4.0], [1.0, 2.0, float("nan")]):
        with pytest.raises(ValueError):
            to_field(ff, bad)


# --- heading_deg：零基准与符号 -------------------------------------------------------


def test_heading_is_zero_when_up_axis_equals_field_x():
    """R5-1：码的 up 轴（规范 +Y）与场地 +X 重合时，航向 = 0。"""
    ff = _identity_ff()
    assert heading_deg(_marker_R(0.0), ff) == pytest.approx(0.0, abs=1e-12)


def test_heading_increases_with_ccw_rotation_seen_from_above():
    """R5b′/R5-2：车**逆时针**（从上往下看）转 +θ，航向就 +θ。

    这条钉的是符号。计划的 X（TL→TR）与 Y（朝 BL）在像平面里，"从 +X 转到 +Y"
    在画面上是**顺时针**，所以裸的 `atan2(v_y, v_x)` 是顺时针为正，必须取负。
    符号搞反的代价是后面每一条对比都变成"看起来合理但全错" —— 差一个负号，
    视觉和编码器永远对不上，且没有任何一处会报错。
    """
    ff = _identity_ff()
    for yaw in (10.0, 45.0, 90.0, 137.5):
        assert heading_deg(_marker_R(yaw), ff) == pytest.approx(yaw, abs=1e-9)


def test_heading_is_independent_of_origin_cam():
    """R5-3：平移场地原点不能改变航向 —— 航向只由姿态决定。"""
    h0 = heading_deg(_marker_R(37.0), _identity_ff((0.0, 0.0, 0.0)))
    for origin in ((500.0, -300.0, 450.0), (-1000.0, 800.0, -20.0)):
        h1 = heading_deg(_marker_R(37.0), _identity_ff(origin))
        assert h1 == pytest.approx(h0, abs=1e-12)


def test_heading_normalizes_into_half_open_range():
    """R5-4：190° 归一到 (−180, 180]；190° 与 −170° 是同一个姿态，结果必须相同。"""
    ff = _identity_ff()
    h = heading_deg(_marker_R(190.0), ff)
    assert h == pytest.approx(-170.0, abs=1e-9)
    assert -180.0 < h <= 180.0
    assert heading_deg(_marker_R(-170.0), ff) == pytest.approx(h, abs=1e-9)
    # 半开：恰好 180° 归到 +180 而不是 −180。
    assert heading_deg(_marker_R(180.0), ff) == pytest.approx(180.0, abs=1e-9)


def test_heading_is_equivariant_under_field_plane_rotation_on_the_real_photo():
    """真实照片 + 真实场地系：绕场地 +Z 转 α，航向就 −α（+Z 朝里，右手定则）。"""
    poses = _poses()
    ff = build_field_frame(poses)
    R29, _ = cv2.Rodrigues(poses[VEHICLE_ID].rvec)
    base = heading_deg(R29, ff)

    for alpha in (30.0, -45.0, 12.5):
        c, s = np.cos(np.radians(alpha)), np.sin(np.radians(alpha))
        Q = np.float64([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        turned = ff.R_cam2field.T @ Q @ ff.R_cam2field @ R29
        assert heading_deg(turned, ff) == pytest.approx(_wrap180(base - alpha), abs=1e-9)


def test_heading_rejects_malformed_rotation_matrices():
    """`heading_deg` 校验的是**形状与有限性**，不是正交性 —— 名字要跟着改。

    旧名 `..._rejects_non_orthonormal_input` 名不副实：`2*np.eye(3)` 这种非正交矩阵
    是能算的（返回 −90°），函数并不拒绝它，测它只会逼出一句"我没打算拦这个"。
    `R_cam2marker` 永远来自 `cv2.Rodrigues`，本来就正交；真要拦非正交矩阵是一项
    行为变更（会拒绝某些合法的带尺度矩阵），不该由一条校验测试顺手定下来。
    """
    ff = _identity_ff()
    for bad in (np.zeros((3, 3)), np.eye(2), [[np.nan] * 3] * 3, np.zeros((3, 3, 1))):
        with pytest.raises(ValueError):
            heading_deg(bad, ff)


def test_heading_rejects_up_axis_perpendicular_to_the_field_plane():
    """up 轴垂直于场地平面（码躺着朝天）时方位角无定义 —— 抛，不返回假 0°。"""
    ff = _identity_ff()
    lying = np.float64([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
    with pytest.raises(ValueError):
        heading_deg(lying, ff)


# --- body_pose_in_field：偏置与车体 yaw ---------------------------------------------


def test_body_bias_shifts_origin_by_its_own_magnitude():
    """ID29 装在左甲板：dy=55 的偏置必须把车体原点挪掉约 55 mm。

    `PLACEHOLDER_DY_MM` 是**占位数字**（抄自计划 Task 1 的测试夹具），不是 A1 的
    测量结果 —— `视觉测量/结果/car_calib.json` 至今不存在，真实偏置还没量。

    **为什么断言位移模长而不是 y 分量**：计划写的 `abs(y − my) > 5` 在这张照片上
    恒假（实测只动 0.47 mm）。原因是 ID29 贴出来时转了约 90°（R25：它的规范 +X
    指向图像上方，于是规范 +Y 指向图像左侧 ≈ 场地 −X），所以"码系的 +Y 偏置"
    几乎整个落在场地 X 上：实测位移向量 (54.96, 0.47, 2.10) mm。模长才是与
    车头朝向无关、且对"偏置根本没生效"这种实现错误同样敏感的那个量。
    """
    poses = _poses()
    ff = build_field_frame(poses)
    d = _shift(ff, poses[VEHICLE_ID], _bias(0.0), _bias(PLACEHOLDER_DY_MM))
    assert float(np.hypot(d[0], d[1])) == pytest.approx(PLACEHOLDER_DY_MM, abs=1.0), d
    # ID29 的上轴几乎躺在场地平面内（|Δz| = 2.1 mm < 偏置的 4%），z 不该带走一大截。
    assert abs(float(d[2])) < 1e-9, "纯平移偏置不该动航向"

    dx_only = _shift(ff, poses[VEHICLE_ID], _bias(0.0), _bias(0.0, dx_mm=-12.0))
    assert float(np.hypot(dx_only[0], dx_only[1])) == pytest.approx(12.0, abs=0.5), dx_only


def test_body_bias_is_expressed_in_the_marker_frame():
    """bias 的参考系是**码的规范轴**，不是车体轴、更不是场地轴：车转 90°，位移跟着转 90°。

    错写成场地系偏置（`p_field − [dx, dy]`）时，位移方向与车头朝向脱钩，
    每条实验的位移都会带上一个和朝向有关的系统偏差 —— 而且车头朝 +X 时正好为零，
    抽查最容易漏过去。

    这条同时是 **I1 的人肉防线**：A1 的人若用卷尺量到"车体中心在码的左边 55 mm"，
    却按车体系填成 `dy_mm = 55`，代码会把它当前后偏置转 90° 用掉 —— 每条实验的
    x 和 y 都带 ~55 mm 的系统偏差且无人报错。本车当前贴法下（ID29 规范 +Y →
    图像左、规范 +X → 图像上）正确的填法是 `dx_mm ≈ ±b, dy_mm ≈ 0, dtheta_deg ≈ ±90°`。
    """
    ff = _identity_ff()
    t_cam = np.float64([10.0, -20.0, 500.0])
    zero = _bias(0.0)
    dy = _bias(PLACEHOLDER_DY_MM)

    d0 = _shift(ff, _pose(t_cam, _marker_R(0.0)), zero, dy)
    d90 = _shift(ff, _pose(t_cam, _marker_R(90.0)), zero, dy)

    assert d0[:2] == pytest.approx([-PLACEHOLDER_DY_MM, 0.0], abs=1e-9)
    assert d90[:2] == pytest.approx([0.0, PLACEHOLDER_DY_MM], abs=1e-9)   # 跟着转了 90°


def test_body_pose_yaw_equals_marker_yaw_when_dtheta_zero():
    poses = _poses()
    ff = build_field_frame(poses)
    bias = {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0}
    _, _, yaw = body_pose_in_field(ff, poses[VEHICLE_ID], bias)
    _, _, yaw_m = body_pose_in_field(ff, poses[VEHICLE_ID],
                                     {"dx_mm": 0.0, "dy_mm": 0.0, "dtheta_deg": 0.0})
    assert abs(yaw - yaw_m) < 1e-9


def test_dtheta_subtracts_from_yaw():
    """dtheta 是"码相对车体的朝向偏置"，所以车体 yaw = 码 yaw − dtheta。"""
    poses = _poses()
    ff = build_field_frame(poses)
    _, _, yaw0 = body_pose_in_field(ff, poses[VEHICLE_ID],
                                    {"dx_mm": 0.0, "dy_mm": 0.0, "dtheta_deg": 0.0})
    for dtheta in (30.0, -30.0, 90.0):
        _, _, yaw = body_pose_in_field(ff, poses[VEHICLE_ID],
                                       {"dx_mm": 0.0, "dy_mm": 0.0, "dtheta_deg": dtheta})
        assert yaw == pytest.approx(_wrap180(yaw0 - dtheta), abs=1e-9)
        assert -180.0 < yaw <= 180.0


def test_body_pose_requires_all_three_bias_keys():
    """缺键必须报错：ID29 偏置本来就不是零，静默当 0 等于凭空造一个车体原点。"""
    poses = _poses()
    ff = build_field_frame(poses)
    for bad in ({}, {"dx_mm": 0.0}, {"dx_mm": 0.0, "dy_mm": 55.0}, None):
        with pytest.raises(ValueError):
            body_pose_in_field(ff, poses[VEHICLE_ID], bad)
    with pytest.raises(ValueError):
        body_pose_in_field(ff, poses[VEHICLE_ID],
                           {"dx_mm": float("nan"), "dy_mm": 0.0, "dtheta_deg": 0.0})


def test_body_pose_warns_when_given_a_boundary_marker():
    """M4：传边界码的位姿进来会算出一个**看起来正常**、实则差几百毫米的车体原点。

    bias 是按 ID29 的贴法标定的；边界码在场地角点上，用它算等于把 ±252/±126 mm
    的场地系原点搬到车体外。不拦（换个 id 就能绕开），但必须喊。
    """
    poses = _poses()
    ff = build_field_frame(poses)
    with pytest.warns(RuntimeWarning, match=f"ID{TL_ID}"):
        x, y, yaw = body_pose_in_field(ff, poses[TL_ID], _bias(0.0))
    assert abs(x) < 1e6 and abs(y) < 1e6 and -180.0 < yaw <= 180.0   # 算得出，但不可信


def test_to_field_and_heading_reject_malformed_field_frames():
    """M5：`FieldFrame` 没有 `__post_init__`，形状/有限性错误要在**使用点**上炸掉。

    校验的是形状与有限性，与 `heading_deg` 对 `R_cam2marker` 的口径一致；
    正交性不查（真要查就是一项行为变更，见上一条用例的说明）。
    `origin_cam` 的 `(3,1)`（OpenCV 的列向量形状）**是合法输入**，不算坏数据。
    """
    for bad_R, bad_origin in [(np.zeros((3, 2)), np.zeros(3)),
                              (np.full((3, 3), np.nan), np.zeros(3)),
                              (np.eye(3), np.zeros((3, 2))),
                              (np.eye(3), np.zeros(2)),
                              (np.eye(3), [0.0, 0.0, float("nan")])]:
        ff = FieldFrame(R_cam2field=bad_R, origin_cam=bad_origin,
                        marker_px={}, normal_cam=np.float64([0.0, 0.0, -1.0]))
        with pytest.raises(ValueError):
            to_field(ff, [1.0, 2.0, 3.0])
        with pytest.raises(ValueError):
            heading_deg(np.eye(3), ff)
