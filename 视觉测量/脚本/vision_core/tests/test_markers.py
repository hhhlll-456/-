"""ArUco 检测 + 单码位姿的行为锁。

三条最贵的规格，都是出过一次问题才写下来的：

1. **角点次序 = 规范次序，不是图像 TL/TR/BR/BL。** `detectMarkers` 解码后角点
   **跟着码走**：把码的位图转 90° 再检测，`corners[0]` 仍指着码自己的左上角
   （本文件 `test_detect_markers_canonical_corner_order_follows_the_marker` 钉住）。
   `SOLVEPNP_IPPE_SQUARE` 的物体点数组与检测角点**逐位对应**，配错不会报错、
   重投影 RMS 照样很小，却把每个位姿翻 180°。所以每个位姿测试都同时断言 rvec
   和打印面法向 —— 只断言 tvec 的测试抓不住这个（tvec 两种配法完全一样）。
2. **"面向相机"= 法向 −Z。** 相机 +Z 指向被摄物，面向相机的物体外法向必然
   指回相机，即相机系 −Z。正对相机的码解出 `rvec=[π,0,0]`、法向 `[0,0,−1]`。
3. **绝不抛异常**：掉码 / 曝光炸掉 / 标定没加载，都必须是"这一帧无解"，
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

# 标记规范物体点，与 vision_core.markers._ippe_object_points 同义。
# +X 角点0→角点1（码正面的右），+Y 角点3→角点0（码正面的上），+Z 打印面外法向。
OBJ_PTS = np.float64([[-15.0, 15.0, 0.0], [15.0, 15.0, 0.0],
                      [15.0, -15.0, 0.0], [-15.0, -15.0, 0.0]]).reshape(4, 1, 3)

UPRIGHT_R = np.diag([1.0, -1.0, -1.0])   # 正对相机的码：rvec = [pi, 0, 0]


def _project(rvec, tvec, obj=OBJ_PTS):
    """按 rvec/tvec 投影物体点，返回**规范次序**的四个角点。"""
    img, _ = cv2.projectPoints(obj, np.float64(rvec).reshape(3, 1),
                               np.float64(tvec).reshape(3, 1),
                               np.float64(K), np.float64(D))
    return img.reshape(4, 2).astype(np.float64)


def _load_photo():
    p = calib_store.resolve_path(PHOTO)
    img = cv2.imread(str(p))
    assert img is not None, f"读不到测试照片：{p}"
    return img


def _rot_angle(v):
    return float(np.linalg.norm(np.asarray(v, dtype=np.float64).ravel()))


def _printed_normal(rvec):
    """打印面外法向在相机系的方向。"""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return (R @ np.float64([[0.0], [0.0], [1.0]])).ravel()


# --- solve_marker_pose：位姿解算 ------------------------------------------------


def test_solve_marker_pose_recovers_known_pose():
    """30mm 码摆在相机系 (0, -400, 100) mm，位置和姿态都要原样回来。"""
    side = 30.0
    rvec_true = np.float64([[0.0], [0.0], [0.0]])
    tvec_true = np.float64([[0.0], [-400.0], [100.0]])

    r2, t2, rms = solve_marker_pose(_project(rvec_true, tvec_true), side, K, D)

    assert np.allclose(t2, tvec_true, atol=1e-3)
    assert rms < 1e-6
    assert _rot_angle(r2 - rvec_true) < 1e-6, (
        f"姿态反了，rvec={np.asarray(r2).ravel()} 真值={rvec_true.ravel()}")


def test_solve_marker_pose_picks_correct_solution():
    """IPPE_SQUARE 有两个解，必须选重投影误差小且 tvec z>0 的正解（Review Focus #5）。

    两个解差 ~0.5mm，atol=1e-3 足以把它们分开；同时断言姿态不被翻面。
    """
    side = 30.0
    rvec_true = np.float64([[0.0], [0.0], [0.0]])
    tvec_true = np.float64([[30.0], [-400.0], [120.0]])

    r2, t2, rms = solve_marker_pose(_project(rvec_true, tvec_true), side, K, D)

    assert t2[2, 0] > 0
    assert np.allclose(t2, tvec_true, atol=1e-3)
    assert _rot_angle(r2 - rvec_true) < 1e-6
    assert rms < 1e-6


def test_solve_marker_pose_recovers_tilted_marker():
    """斜着的码（绕 X 0.6rad / 绕 Y 0.5rad），位姿全量还原。"""
    side = 30.0
    rx, ry = 0.6, 0.5
    R = UPRIGHT_R @ np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)],
    ]) @ np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)],
    ])
    rvec_true, _ = cv2.Rodrigues(R)
    tvec_true = np.float64([[40.0], [-40.0], [430.0]])

    r2, t2, rms = solve_marker_pose(_project(rvec_true, tvec_true), side, K, D)

    assert np.allclose(t2, tvec_true, atol=1e-3)
    assert rms < 1e-6
    R2, _ = cv2.Rodrigues(np.asarray(r2, dtype=np.float64).reshape(3, 1))
    assert np.linalg.norm(R2 - R) < 1e-6


def test_upright_marker_reports_printed_face_toward_camera():
    """正对相机的码：rvec=[pi,0,0]、打印面法向 [0,0,-1]、标记 +Y 指向图像上方。

    这条是角点次序的物理回归锁。相机 +Z 指向被摄物，**面向相机的物体外法向
    指回相机 = 相机系 -Z**。把角点次序配反会让法向变成 [0,0,+1]（"纸面背对
    相机"，对不透光的印刷码不可能），tvec 却一模一样 —— 只有这条能抓住。

    这里刻意加了 2° 倾斜：`SOLVEPNP_IPPE_SQUARE` 在**恰好轴对齐**（0°）这个
    奇异点上会解坏（best rms = 2.06px，`solvePnP(ITERATIVE)` 则是 0）。
    0.02° 就恢复正常，所以真图（角点必然带亚像素噪声）碰不到这个点，
    但合成图会正中它。
    """
    tilt = np.radians(2.0)
    R_gt = np.array([[1, 0, 0], [0, np.cos(tilt), -np.sin(tilt)],
                     [0, np.sin(tilt), np.cos(tilt)]]) @ UPRIGHT_R
    rvec_gt, _ = cv2.Rodrigues(R_gt)

    r2, t2, rms = solve_marker_pose(
        _project(rvec_gt, [0.0, -400.0, 400.0]), 30.0, K, D)

    assert rms < 1e-6
    R, _ = cv2.Rodrigues(np.asarray(r2, dtype=np.float64).reshape(3, 1))
    assert np.linalg.norm(R - R_gt) < 1e-6, (
        f"姿态反了：‖R−R_gt‖={np.linalg.norm(R - R_gt):.3f}")
    assert np.allclose(_printed_normal(r2), [0.0, 0.0, -1.0], atol=0.04)

    # 标记的 +Y 是"上轴"，正对相机的码解出来应指向相机 -Y，也就是图像上方
    up_in_cam = (R @ np.float64([[0.0], [1.0], [0.0]])).ravel()
    assert np.allclose(up_in_cam, [0.0, -1.0, 0.0], atol=0.04), (
        f"标记的 +Y（上轴）= {np.round(up_in_cam, 3)}，应当指向相机 -Y（图像上方）")


# --- detect_markers：规范角点次序 ------------------------------------------------


def test_detect_markers_canonical_corner_order_follows_the_marker():
    """把码的位图转 90°/180°/270° 再检测，corners[0..3] 仍依次指着位图的四角。

    若角点次序是"图像 TL,TR,BR,BL"，这里四种旋转的结果就会不同。
    """
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    base = np.zeros((144, 144), np.uint8)
    d.drawMarker(29, 144, base)
    # 位图四角 (row, col) -> 名字
    names = ["TL", "TR", "BR", "BL"]
    base_corners = {"TL": (0, 0), "TR": (0, 143), "BR": (143, 143), "BL": (143, 0)}
    # cv2.rotate 把 base 的 (i,j) 送到旋转后位图的哪个位置
    moves = {
        "0°": (None, lambda i, j: (i, j)),
        "90°": (cv2.ROTATE_90_CLOCKWISE, lambda i, j: (j, 143 - i)),
        "180°": (cv2.ROTATE_180, lambda i, j: (143 - i, 143 - j)),
        "270°": (cv2.ROTATE_90_COUNTERCLOCKWISE, lambda i, j: (143 - j, i)),
    }
    for label, (flag, move) in moves.items():
        mk = base if flag is None else cv2.rotate(base, flag)
        canvas = np.full((1080, 1920), 255, np.uint8)
        canvas[300:444, 700:844] = mk
        got = detect_markers(canvas, calib_store.load_calib())
        assert set(got) == {29}, f"{label} 未检出 ID29"

        cn = got[29].corners_px
        pos = {}
        for name, (i0, j0) in base_corners.items():
            r, c = move(i0, j0)
            pos[name] = np.float64([700.0 + c, 300.0 + r])
        seq = []
        for p in cn:
            best = min(pos, key=lambda k: np.linalg.norm(pos[k] - p))
            seq.append(best if np.linalg.norm(pos[best] - p) < 3.0 else "?")
        assert seq == names, f"{label} 角点次序 {seq}，期望 {names}"


def test_detect_markers_pasted_marker_recovers_canonical_frame():
    """水平正贴的码：解出 rvec=[pi,0,0]、法向 [0,0,-1]、标记 +Y 朝图像上方。

    端到端钉住角点次序：合成码正对相机，tvec 几乎不变（两种配法只差 0.2mm 量级），
    但角点次序配反时 rvec 会变成 [0,0,0]、法向变成 +Z。
    """
    poses = detect_markers(_paste(29, 144, 700, 300), calib_store.load_calib())
    assert set(poses) == {29}

    # 真值：正对相机、边长 30mm、像素边长 144
    z_true = _FX * 30.0 / 144
    t_true = np.float64([
        [(700 + 71.5 - _CX) * z_true / _FX],
        [(300 + 71.5 - _CY) * z_true / _FX],
        [z_true],
    ])
    # 亚像素角点定位带来 ~0.25mm 系统偏差（实测 dZ=+0.244）
    assert np.allclose(poses[29].tvec, t_true, atol=0.5)
    assert _rot_angle(poses[29].rvec - np.float64([[np.pi], [0.0], [0.0]])) < 0.01
    assert np.allclose(_printed_normal(poses[29].rvec), [0.0, 0.0, -1.0], atol=0.02)


def test_detect_markers_end_to_end_perspective_ground_truth():
    """把**印好的**码按真实透视贴到图上，跑完整 detect_markers，位姿对真值。

    这条是 C1 那个 bug 真正该有的回归锁：合成图带真实透视，所以角点次序配反
    会得到一个真正偏离的位姿（‖R-Rgt‖ = 2.83），而 tvec 依然正确。

    画法上有两个坑，踩过：
    - `warpPerspective` 的 M 是 **源→目标**（OpenCV 内部再取逆），别手动求逆。
    - 码正面朝相机对应 `rvec=[pi,0,0]`；用 rvec=0 等于从**背面**看，
      印字会被镜像，ArUco 4.6 不解镜像码，表现为"合成图永远检不出"。
    """
    angles = [(0.0, 0.0, 0.0, 430.0), (0.5, 0.3, 0.2, 430.0), (0.3, -0.4, 0.0, 320.0)]
    for rx, ry, rz, tz in angles:
        canvas, R_gt, t_gt = _perspective_scene(29, 30.0, rx, ry, rz,
                                                [-40.0, -60.0, tz])
        poses = detect_markers(canvas, calib_store.load_calib())
        assert set(poses) == {29}, f"倾角 {(rx, ry, rz)} 未检出"

        R, _ = cv2.Rodrigues(np.asarray(poses[29].rvec, np.float64).reshape(3, 1))
        assert np.linalg.norm(R - R_gt) < 0.05, (
            f"倾角 {(rx, ry, rz)} 姿态偏差 {np.linalg.norm(R - R_gt):.3f} —— 角点次序配反了？")
        assert np.allclose(poses[29].tvec, t_gt, atol=1.0)
        assert poses[29].reproj_rms_px < 0.5


# --- detect_markers：真实照片 ---------------------------------------------------


def test_detect_markers_on_real_photo_finds_all_five():
    poses = detect_markers(_load_photo(), calib_store.load_calib())
    assert set(poses) == {20, 21, 22, 23, 29}


def test_detect_markers_on_real_photo_fits_every_marker():
    """五个码都要落在画面内、在相机前方、重投影残差亚像素级。

    打印面法向只对 **ID29** 断言：ID20 上边缘离画面边界只有 6px，是全图条件数
    最差的码，实测法向 z 只有 -0.445（偏离正视 63.6°）。本任务不应该依赖它的姿态。
    """
    poses = detect_markers(_load_photo(), calib_store.load_calib())
    for mid, pose in sorted(poses.items()):
        assert pose.marker_id == mid
        assert pose.reproj_rms_px < 2.0, f"id={mid} 重投影 {pose.reproj_rms_px:.3f}px 偏大"
        assert pose.tvec[2, 0] > 0, f"id={mid} 解到了相机后面"
        assert pose.corners_px.shape == (4, 2)

    normal = _printed_normal(poses[calib_store.VEHICLE_ID].rvec)
    assert normal[2] < -0.9, (
        f"ID29 打印面法向 {np.round(normal, 3)} —— 码是印在纸上的，法向必然指回相机")


def test_detect_markers_returns_empty_on_blank_image():
    """Review Focus #4：检不到不能抛异常"""
    poses = detect_markers(np.zeros((1080, 1920, 3), np.uint8), calib_store.load_calib())
    assert poses == {}


def test_vehicle_marker_height_is_not_forced_to_zero():
    """ID29 悬空 55~75mm，PnP 必须解出与量级相符的相机系 Z，而不是塌到桌面上。

    这是 **D1 决策的回归锁**。photo_review_20261013.jpg 上实测（本机 OpenCV 4.6.0）：

        id    camX     camY    camZ(mm)   码边长(px)   重投影RMS(px)
        20   -209.5   -194.7    517.5        69.8         0.846
        21   +295.2   -170.0    510.4        72.1         0.175
        22   +282.0    +85.3    518.1        69.6         0.339
        23   -222.7    +52.4    521.8        69.8         0.068
        29    -54.6    -80.5    438.4        99.7         0.188

    (a) ID29 的 Z：30mm 的码在 ~100px 处，fx=1460.76 ⇒ Z ≈ fx*边长/像素边长
        ≈ 1460.76*30/99.7 ≈ 438mm，和规格里"相机约 0.44m 远"对得上。
        取 350~550 的宽容带。
    (b) 更有分量的是**次序**：边界码贴在桌面上，车载码高出桌面，实测 Z 要**小**
        518-438 ≈ 78mm。如果有人改回桌面单应，把 ID29 反投影到桌面平面，
        实测它的 Z 会变成 517.5mm，此时"边界均值 − ID29"只剩 -0.5mm，这条会红。
        所以门槛取 30mm —— 离实测 78mm 有足够余量，又在"塌到桌面"的 -0.5mm 之上。

    注意：角点次序配反**不会**影响本条（tvec 完全不变，实测 Δ = 0.0000mm）。
    角点次序由上面那几条专门盯。
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


def _paste(mid, side_px, u0, v0, w=1920, h=1080):
    """把一个 DICT_4X4_50 码水平正贴到画布上（合成真值，无透视畸变）。"""
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    mk = np.zeros((side_px, side_px), np.uint8)
    d.drawMarker(mid, side_px, mk)
    canvas = np.full((h, w), 255, np.uint8)
    canvas[v0:v0 + side_px, u0:u0 + side_px] = mk
    return canvas


def _perspective_scene(mid, side_mm, rx, ry, rz, tvec, M0=600, sup=2):
    """按真实透视把印好的码贴到画布上，返回 (图, R真值, t真值)。

    基准姿态 UPRIGHT_R（码正面朝相机）再叠绕 X/Y 的倾斜和绕光轴的滚转。
    """
    R_gt = (np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
            @ np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
            @ np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
            @ UPRIGHT_R)
    rvec, _ = cv2.Rodrigues(R_gt)
    t = np.float64(tvec).reshape(3, 1)

    big = np.zeros((M0, M0), np.uint8)
    cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50).drawMarker(mid, M0, big)
    proj, _ = cv2.projectPoints(OBJ_PTS, rvec, t, np.float64(K), np.float64(D))
    dst = proj.reshape(-1, 2).astype(np.float32)
    # obj[i] 就是位图正面视角的第 i 个角：TL,TR,BR,BL
    src = np.float32([[0, 0], [M0 - 1, 0], [M0 - 1, M0 - 1], [0, M0 - 1]])
    M = cv2.getPerspectiveTransform(src, dst * sup)      # 源 -> 目标
    h, w = 1080 * sup, 1920 * sup
    warped = cv2.warpPerspective(big, M, (w, h), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    mask = cv2.warpPerspective(np.full_like(big, 255), M, (w, h), flags=cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    out = np.full((h, w), 255, np.uint8)
    out[mask > 0] = warped[mask > 0]
    return cv2.resize(out, (1920, 1080), interpolation=cv2.INTER_AREA), R_gt, t


_CALIB_NO_SIDE = {"K": K, "dist": D}
_CALIB_JUNK = {"K": "nope", "dist": "nope",
               "boundary_side_mm": None, "vehicle_side_mm": None}
_CALIB_BAD_SIDE = {"K": K, "dist": D,
                   "boundary_side_mm": -1.0, "vehicle_side_mm": 0.0}


def test_detect_markers_skips_unknown_ids():
    """非 20~23/29 的码静默跳过：码表没给它的边长，猜一个等于凭空造位姿。"""
    canvas = _paste(29, 144, 700, 300)
    canvas[640:784, 300:444] = _paste(7, 144, 300, 640)[640:784, 300:444]
    poses = detect_markers(canvas, calib_store.load_calib())
    assert set(poses) == {29}


def test_detect_markers_never_raises_on_unusable_input():
    """Review Focus #4：坏输入只允许得到"这一帧无解"，不许抛异常。"""
    good = calib_store.load_calib()
    blank = np.zeros((1080, 1920, 3), np.uint8)

    cases = {
        "None 图": (None, good),
        "全黑图": (blank, good),
        "小图": (np.zeros((10, 10), np.uint8), good),
        "空标定 dict": (blank, {}),
        "标定缺键": (blank, _CALIB_NO_SIDE),
        "标定 K 是垃圾": (blank, _CALIB_JUNK),
        "码边长非正": (blank, _CALIB_BAD_SIDE),
        "标定不是 dict": (blank, None),
    }
    for label, (img, calib) in cases.items():
        poses = detect_markers(img, calib)
        assert poses == {}, f"{label} 应得到无解，实得 {sorted(poses)}"


def test_detect_markers_warns_loudly_on_malformed_input():
    """真·坏输入必须**发 RuntimeWarning**，不能静默返回空。

    "图坏了"和"画面里本来就没有码"如果都安静地返回空字典，那就没法区分 ——
    那会静默废掉一整轮实验。所以：**格式非法**要喊，**格式合法但没码**不喊。
    """
    good = calib_store.load_calib()
    blank = np.zeros((1080, 1920, 3), np.uint8)

    cases = {
        "None 图": (None, good),
        "空标定 dict": (blank, {}),
        "标定缺键": (blank, _CALIB_NO_SIDE),
        "标定 K 是垃圾": (blank, _CALIB_JUNK),
        "码边长非正": (blank, _CALIB_BAD_SIDE),
        "标定不是 dict": (blank, None),
    }
    for label, (img, calib) in cases.items():
        with pytest.warns(RuntimeWarning):
            assert detect_markers(img, calib) == {}, f"{label} 应得到无解"


def test_detect_markers_stays_silent_when_image_is_merely_empty(recwarn):
    """合法但没有码的图（全黑、小图）不该发 warning —— 那不是错误。"""
    blank = np.zeros((1080, 1920, 3), np.uint8)
    assert detect_markers(blank, calib_store.load_calib()) == {}
    assert detect_markers(np.zeros((10, 10), np.uint8), calib_store.load_calib()) == {}
    assert [w for w in recwarn if issubclass(w.category, RuntimeWarning)] == []


def test_detect_markers_float_image_scales_both_conventions():
    """浮点图两种约定都要真的检出 5 个码，不能"静默检不出"。"""
    photo = _load_photo()
    for label, arr in [("[0,1]", photo.astype(np.float32) / 255.0),
                       ("[0,255]", photo.astype(np.float32))]:
        poses = detect_markers(arr, calib_store.load_calib())
        assert set(poses) == {20, 21, 22, 23, 29}, f"浮点图 {label} 只解出 {sorted(poses)}"


def test_detect_markers_rejects_out_of_range_float_loudly():
    """取值范围判不了的浮点图要**大声拒绝**，不能猜一个缩放然后静默检不出。"""
    with pytest.warns(RuntimeWarning, match="取值范围"):
        poses = detect_markers(_load_photo().astype(np.float32) * 7.0,
                               calib_store.load_calib())
    assert poses == {}


def test_detect_markers_accepts_grayscale():
    poses = detect_markers(cv2.cvtColor(_load_photo(), cv2.COLOR_BGR2GRAY),
                           calib_store.load_calib())
    assert set(poses) == {20, 21, 22, 23, 29}
