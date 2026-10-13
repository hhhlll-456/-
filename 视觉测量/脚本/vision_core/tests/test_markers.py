"""ArUco 检测 + 单码位姿的行为锁。

两条最贵的规格，都是出过一次问题才写下来的：

1. **角点次序**：`detectMarkers` 返回的是 **TL,TR,BR,BL**（OpenCV 文档口径，
   本文件里用 _frontal_corners / _tilted_corners 从针孔投影独立复现过）。
   而 `SOLVEPNP_IPPE_SQUARE` 的物体点规范次序是 **BL,BR,TR,TL**。
   两者配错不会报错、不会崩，重投影 RMS 也仍然很小（本机实测噪声图上 0.23px），
   但解出来的位姿是**翻面**的：码的正面法向指向 z=-1，也就是“纸面背对相机”。
   位移照样看着像模像样，唯独转角错 180°。所以每个位姿测试都同时断言 tvec 和
   rvec —— 只断言 tvec 的测试是抓不住这个 bug 的（简报里那条就只有 tvec）。
2. **绝不抛异常**：某帧掉码 / 曝光炸掉 / 标定没加载，都必须是“这一帧无解”，
   不能让实车采图循环整个挂掉。见 test_detect_markers_never_raises。
"""
import cv2
import numpy as np
import pytest

from vision_core import calib_store
from vision_core.markers import detect_markers, solve_marker_pose

# 路径必须按仓库根解析（R3）：pytest 从 视觉测量/脚本 跑，cwd 相对路径会读不到。
PHOTO = "视觉测量/照片/photo_review_20261013.jpg"

K = [[1460.76, 0, 1019.44], [0, 1460.76, 592.77], [0, 0, 1]]
D = [0.0] * 5
_FX = 1460.76
_CX = 1019.44
_CY = 592.77


def _frontal_corners(tvec, side):
    """正对相机、摆正的码在图像上的四角，次序 **TL,TR,BR,BL**。

    从针孔投影直接算，不经过 markers.py 里的任何物体点约定 —— 这样它才是
    一把独立的尺子，而不是拿实现去量实现。tvec 是码中心在相机系的位置（mm）。
    """
    h = side / 2.0
    tx, ty, tz = (float(v) for v in np.asarray(tvec, dtype=np.float64).ravel())
    # 相机系 +Y 指向图像下方，所以图像“上”= 相机 -Y
    return np.float64([
        [_CX + _FX * (tx - h) / tz, _CY + _FX * (ty - h) / tz],   # TL
        [_CX + _FX * (tx + h) / tz, _CY + _FX * (ty - h) / tz],   # TR
        [_CX + _FX * (tx + h) / tz, _CY + _FX * (ty + h) / tz],   # BR
        [_CX + _FX * (tx - h) / tz, _CY + _FX * (ty + h) / tz],   # BL
    ])


def _tilted_corners(rvec, tvec, side):
    """按 rvec/tvec 摆好再投影，四角仍按 TL,TR,BR,BL 排好。"""
    h = side / 2.0
    obj = np.float64([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]]).reshape(-1, 1, 3)
    img, _ = cv2.projectPoints(obj, rvec, tvec, np.float64(K), np.float64(D))
    return img.reshape(4, 2).astype(np.float64)


def _load_photo():
    p = calib_store.resolve_path(PHOTO)
    img = cv2.imread(str(p))
    assert img is not None, f"读不到测试照片：{p}"
    return img


def _rot_angle(rvec):
    return float(np.linalg.norm(np.asarray(rvec, dtype=np.float64).ravel()))


# --- solve_marker_pose：位姿解算 ------------------------------------------------


def test_solve_marker_pose_recovers_known_pose():
    """30mm 码摆在相机系 (0, -400, 100) mm、正对相机，位置和姿态都要原样回来。"""
    side = 30.0
    tvec = np.float64([[0.0], [-400.0], [100.0]])
    r2, t2, rms = solve_marker_pose(_frontal_corners(tvec, side), side, K, D)

    assert np.allclose(t2, tvec, atol=1e-3)
    assert rms < 1e-6
    # 摆正的码解出来必须还是摆正的：角点次序配错会给出 rvec≈[pi,0,0]（翻面）
    assert _rot_angle(r2) < 1e-6, f"姿态被翻了，rvec={np.asarray(r2).ravel()}"


def test_solve_marker_pose_picks_correct_solution():
    """IPPE_SQUARE 有两个解，必须选重投影误差小且 tvec z>0 的正解（Review Focus #5）。

    简报给的用例：码偏心放在 (30, -400, 120)。两个解差 ~0.5mm，
    所以 atol=1e-3 足以把它们区分开；同时断言姿态不被翻面。
    """
    side = 30.0
    tvec = np.float64([[30.0], [-400.0], [120.0]])
    r2, t2, rms = solve_marker_pose(_frontal_corners(tvec, side), side, K, D)

    assert t2[2, 0] > 0
    assert np.allclose(t2, tvec, atol=1e-3)
    assert _rot_angle(r2) < 1e-6, f"姿态被翻了，rvec={np.asarray(r2).ravel()}"


def test_solve_marker_pose_recovers_tilted_marker():
    """斜着的码（绕 X 0.6rad / 绕 Y 0.5rad）：位姿全量还原，另一个解差 0.45mm。

    简报那条正对相机的用例里两个解太接近了；这里用倾斜姿态把“选错解”
    的后果放大到一眼能看出来。
    """
    side = 30.0
    rx, ry = 0.6, 0.5
    R = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)],
    ]) @ np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)],
    ])
    rvec_true, _ = cv2.Rodrigues(R)
    tvec_true = np.float64([[40.0], [-40.0], [430.0]])

    r2, t2, rms = solve_marker_pose(
        _tilted_corners(rvec_true, tvec_true, side), side, K, D)

    assert np.allclose(t2, tvec_true, atol=1e-3)
    assert rms < 1e-6
    assert _rot_angle(r2 - rvec_true) < 1e-6


# --- detect_markers：真实照片 ---------------------------------------------------


def test_detect_markers_on_real_photo_finds_all_five():
    poses = detect_markers(_load_photo(), calib_store.load_calib())
    assert set(poses) == {20, 21, 22, 23, 29}


def test_detect_markers_on_real_photo_fits_every_marker():
    """五个码都要落在画面内、正对相机、重投影残差亚像素级。

    码是印在纸上的，正面朝着头顶相机，所以解出来的正面法向必须朝相机（z 分量为正）。
    这条同时挡住角点次序配错 —— 配错时法向是 [-0.03, +0.01, -1.0]。
    """
    poses = detect_markers(_load_photo(), calib_store.load_calib())
    for mid, pose in sorted(poses.items()):
        assert pose.marker_id == mid
        assert pose.reproj_rms_px < 2.0, f"id={mid} 重投影 {pose.reproj_rms_px:.3f}px 偏大"
        assert pose.tvec[2, 0] > 0, f"id={mid} 解到了相机后面"

        R, _ = cv2.Rodrigues(pose.rvec)
        normal = R @ np.float64([[0.0], [0.0], [1.0]])
        assert normal[2, 0] > 0.2, (
            f"id={mid} 码正面法向 {np.round(normal.ravel(), 3)} 背对相机 —— 角点次序配错了？")


def test_detect_markers_returns_empty_on_blank_image():
    """Review Focus #4：检不到不能抛异常"""
    poses = detect_markers(np.zeros((1080, 1920, 3), np.uint8), calib_store.load_calib())
    assert poses == {}


def test_vehicle_marker_height_is_not_forced_to_zero():
    """ID29 悬空 55~75mm，PnP 必须解出与量级相符的相机系 Z，而不是塌到桌面上。

    这是 **D1 决策的回归锁**。photo_review_20261013.jpg 上实测（本机 OpenCV 4.6.0）：

        id    camX     camY    camZ(mm)   正面法向 z    码边长(px)
        20   -209.5   -194.7    517.5       +0.45          69.8
        21   +295.2   -170.0    510.4       +1.00          72.1
        22   +282.0    +85.3    518.1       +1.00          69.6
        23   -222.7    +52.4    521.8       +1.00          69.8
        29    -54.6    -80.5    438.4       +1.00          99.7

    (a) ID29 的 Z：30mm 的码在 ~100px 处，fx=1460.76 ⇒ Z ≈ fx*边长/像素边长
        ≈ 1460.76*30/99.7 ≈ 438mm，和规格里“相机约 0.44m 远”对得上。
        取 350~550 的宽容带。
    (b) 更有分量的是**次序**：边界码贴在桌面上，车载码高出桌面，实测 Z 要**小**
        518-438 ≈ 78mm。如果有人改回桌面单应，把 ID29 反投影到桌面平面，
        实测它的 Z 会变成 517.5mm，此时“边界均值 − ID29”只剩 -0.5mm，这条会红。
        所以门槛取 30mm —— 离实测 78mm 有足够余量，又在“塌到桌面”的 -0.5mm 之上。
    """
    poses = detect_markers(_load_photo(), calib_store.load_calib())
    z_car = float(poses[calib_store.VEHICLE_ID].tvec[2, 0])
    z_boundary = float(np.mean(
        [poses[i].tvec[2, 0] for i in calib_store.BOUNDARY_IDS]))

    assert 350.0 < z_car < 550.0, f"ID29 的相机系 Z={z_car:.1f}mm 不在 350~550mm"
    assert z_boundary - z_car >= 30.0, (
        f"边界码 Z 均值 {z_boundary:.1f}mm 只比车载码 {z_car:.1f}mm 高 "
        f"{z_boundary - z_car:.1f}mm —— 车载码被压到桌面上了？")


# --- detect_markers：合成图 + 坏输入 --------------------------------------------


def _paste_marker(mid, side_px, u0, v0, w=1920, h=1080):
    """把一个 DICT_4X4_50 码水平正贴到画布上（合成真值，无透视畸变）。"""
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    mk = np.zeros((side_px, side_px), np.uint8)
    d.drawMarker(mid, side_px, mk)
    canvas = np.full((h, w), 255, np.uint8)
    canvas[v0:v0 + side_px, u0:u0 + side_px] = mk
    return canvas


def test_detect_markers_skips_unknown_ids_and_recovers_known_pose():
    """非 20~23/29 的码静默跳过；ID29 照样解出真值姿态。

    顺带端到端钉住角点次序：合成码是水平正贴的，解出来 rvec 必须约等于 0。
    角点次序配错时 tvec 几乎不变（只有 0.2mm 量级的差别）但 rvec 会变成 [pi,0,0]。
    """
    side_mm = 30.0
    side_px = 144
    u0, v0 = 700, 300
    canvas = _paste_marker(29, side_px, u0, v0)
    canvas[640:784, 300:444] = _paste_marker(7, 144, 300, 640)[640:784, 300:444]

    poses = detect_markers(canvas, calib_store.load_calib())
    assert set(poses) == {29}, "ID7 不是场地码，必须被静默跳过"

    z_true = _FX * side_mm / side_px
    t_true = np.float64([
        [(u0 + (side_px - 1) / 2.0 - _CX) * z_true / _FX],
        [(v0 + (side_px - 1) / 2.0 - _CY) * z_true / _FX],
        [z_true],
    ])
    # 亚像素角点定位带来 ~0.25mm 的系统偏差（实测 dZ=+0.244），留 0.5mm 余量
    assert np.allclose(poses[29].tvec, t_true, atol=0.5), (
        f"ID29 解出 {np.round(poses[29].tvec.ravel(), 3)}，真值 {np.round(t_true.ravel(), 3)}")
    assert _rot_angle(poses[29].rvec) < 0.01, (
        f"水平正贴的码解出 rvec={np.round(np.asarray(poses[29].rvec).ravel(), 3)} —— 角点次序配错了？")


_CALIB_NO_SIDE = {"K": K, "dist": D}
_CALIB_JUNK = {"K": "nope", "dist": "nope",
               "boundary_side_mm": None, "vehicle_side_mm": None}
_CALIB_BAD_SIDE = {"K": K, "dist": D,
                   "boundary_side_mm": -1.0, "vehicle_side_mm": 0.0}


@pytest.mark.parametrize("label,bad_input", [
    ("None 图", (None, "good_calib", True)),
    ("灰度图", ("gray", "good_calib", None)),
    ("float64 图", ("float", "good_calib", None)),
    ("空标定 dict", ("blank", {}, True)),
    ("标定缺键", ("blank", _CALIB_NO_SIDE, True)),
    ("标定 K 是垃圾", ("blank", _CALIB_JUNK, True)),
    ("码边长非正", ("blank", _CALIB_BAD_SIDE, True)),
    ("标定不是 dict", ("blank", None, True)),
])
def test_detect_markers_never_raises(label, bad_input):
    """Review Focus #4：掉码/坏标定只允许得到“这一帧无解”，不许抛异常。

    expect_empty 为 None 的几组喂的是**能解出来的**图（灰度图、float64 图），
    它们只验证“不抛异常 + 返回 dict”，不该断言空。
    """
    kind, payload, expect_empty = bad_input
    if kind == "good_calib":
        img, calib = _load_photo(), calib_store.load_calib()
    elif kind == "gray":
        img = cv2.cvtColor(_load_photo(), cv2.COLOR_BGR2GRAY)
        calib = calib_store.load_calib()
    elif kind == "float":
        img, calib = _load_photo().astype(np.float64), calib_store.load_calib()
    else:
        img, calib = np.zeros((1080, 1920, 3), np.uint8), payload

    poses = detect_markers(img, calib)
    assert isinstance(poses, dict)
    if expect_empty:
        assert poses == {}, f"{label} 应得到无解，实得 {sorted(poses)}"
