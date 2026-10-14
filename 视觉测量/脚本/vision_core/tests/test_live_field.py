"""`live_field.py` 的行为锁：真管道一致性、边缘余量、偏置符号、CLI 与"不开窗"。

这些用例守的是**已经出过一次问题、或者差一步就会静默出错**的地方：

- **本任务本身要消灭的分叉**：`live_aruco.py` 自己设 `adaptiveThreshWinSize*`，
  于是它显示"找到 4 个码"而 `verify_vision.py` 仍然失败。所以
  `test_live_field_never_forks_the_detector` 直接扫源码禁掉任何自设检测参数，
  `test_camera_constants_come_from_verify_vision` 把相机参数钉在 `verify_vision` 的常量上。
- **R48（今天卡住的原因）**：边界码被画面边缘切掉。`edge_margin_px` 是这条的正面处理；
  参考照片上 ID20 的上边余量实测 5.54 px（R27 当时记的"6 px"），必须被点名。
- **bias 符号**（A1 要量的就是它）：叠加层上的车体原点十字与横幅里印的 `(x, y)` 必须是
  同一个点。`body_origin_cam` 用错矩阵（拿 `R_cam2field` 当码的旋转）会得到一个
  **看起来完全正常**的十字 —— 只有拿 `body_pose_in_field` 对拍才能抓住。
- **不开窗也要能用**：`import live_field` 不许碰相机、不许建窗口；`--once` / `--frames`
  在无 DISPLAY 的环境里必须跑得出结果（CI 与本机的验收命令都靠它）。

正例用实拍照片 `视觉测量/照片/photo_review_20261013.jpg`（五码齐全）。相机相关的用例在设备
不可用时 **skip**，不让测试红 —— 那不是代码的问题。
"""
import ast
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pytest

import live_field
import verify_vision
from vision_core import calib_store
from vision_core.field import FieldFrame, body_pose_in_field, build_field_frame, to_field
from vision_core.markers import MarkerPose, detect_markers

PHOTO = "视觉测量/照片/photo_review_20261013.jpg"
SCRIPT = "视觉测量/脚本/live_field.py"

# 参考照片上 ID20 上边到画面上边缘的实测余量（px）。R27 记的是"6 px"，
# `edge_distances_px` 给的是未取整的 5.5444…，所以这里也用未取整的数。
PHOTO_ID20_TOP_PX = 5.544453144073486

# 盖住某个码的矩形（`test_verify_vision.py` 里同一张表：这张照片上五个码的实际位置）。
COVER_BOX = {
    20: (370, 0, 490, 105),
    23: (345, 695, 450, 790),
    29: (760, 245, 915, 405),
}


# --- 夹具 -----------------------------------------------------------------------


def _fixture_image():
    img = cv2.imread(str(calib_store.resolve_path(PHOTO)))
    assert img is not None, f"读不到测试照片 {PHOTO}"
    return img


def _default_calib() -> dict:
    """内建默认标定：标定文件不存在 → bias 三项 0.0（A1 还没做，R37）。不写任何盘。"""
    return calib_store.load_calib("/nonexistent/car_calib.json")


def _cover(img, box, donor=(1000, 300)):
    """用图里一块空场盖住 `box`，造出"某个码检不出来"的反例（盖完当场验，见用例）。"""
    x0, y0, x1, y1 = box
    img[y0:y1, x0:x1] = img[donor[1]:donor[1] + (y1 - y0), donor[0]:donor[0] + (x1 - x0)]
    return img


def _pose_at(corners_px) -> MarkerPose:
    """只带角点的极简 MarkerPose —— 边缘余量只看角点，不需要位姿。"""
    quad = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    return MarkerPose(marker_id=0, rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)),
                      reproj_rms_px=0.0, corners_px=quad)


class FakeCap:
    """`open_camera` 的替身：按给定顺序吐帧（用完重复最后一帧）。"""

    def __init__(self, frames=(), drop_after=None):
        self.frames = list(frames)
        self.index = 0
        self.grabbed = 0
        self.released = 0
        self.drop_after = drop_after
        self.sets = []

    def set(self, prop, value):
        self.sets.append((prop, value))
        return True

    def grab(self):
        self.grabbed += 1

    def read(self):
        if not self.frames or (self.drop_after is not None and self.index >= self.drop_after):
            return False, None
        frame = self.frames[min(self.index, len(self.frames) - 1)].copy()
        self.index += 1
        return True, frame

    def release(self):
        self.released += 1


def _run_headless(monkeypatch, frames, argv_extra=(), drop_after=None,
                  raiser=None, raiser_name=""):
    """把 `open_camera` 换成假相机，跑一次 CLI 路径，返回 `(rc, cap)`。

    `argv_extra` 为空时走的是**窗口**主循环（`--once`/`--frames` 才走无窗口那条），
    所以 `raiser`/`raiser_name` 用来在某个 `cv2` 窗口调用上注入 `cv2.error`。

    **退出只能靠注入 raiser 或喂空 frames**：本夹具把 `cv2.waitKey` 钉成恒 −1
    （`FakeCap` 的帧用完之后会无限重复最后一帧），所以 `argv_extra=()` 且不注入
    raiser/drop_after 的用例会**永远退不出来**，在 pytest 里挂成死循环。要走窗口主
    循环请用 `_drive_window`（它有按键序列、`drop_after` 与 `interrupt_after`）。
    """
    cap = FakeCap(frames, drop_after=drop_after)
    monkeypatch.setattr(live_field, "open_camera", lambda *a, **k: cap)
    monkeypatch.setattr(live_field, "DEFAULT_SETTLE_FRAMES", 0)
    # 冷却在下面单独有用例覆盖；这里关掉，好把"变化才打"的规则单独验清楚。
    monkeypatch.setattr(live_field, "TERMINAL_MIN_INTERVAL_S", 0.0)
    monkeypatch.setattr(live_field, "_has_display", lambda: True)
    # **GUI 全 stub**（含 namedWindow/imshow/waitKey）：否则在操作者的桌面上真会弹出一个
    # 窗口，而且 destroyAllWindows 被 stub 掉之后它会一直留在屏幕上。本夹具的
    # argv_extra 为空时走的是**窗口主循环**，所以这几个必须都盖住。
    for name in ("namedWindow", "resizeWindow", "moveWindow", "destroyAllWindows"):
        monkeypatch.setattr(cv2, name, lambda *a, **k: None)
    monkeypatch.setattr(cv2, "imshow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "waitKey", lambda *a, **k: -1)   # -1 = 没有按键
    if raiser is not None:
        def boom(*_a, **_k):
            raise cv2.error(raiser)

        monkeypatch.setattr(cv2, raiser_name, boom)
    return live_field.main(list(argv_extra)), cap


# --- edge_margin_px：R48 的正面处理 ------------------------------------------------


def test_edge_margin_is_large_when_the_marker_is_well_inside_the_frame():
    quad = np.float64([[400.0, 100.0], [440.0, 100.0], [440.0, 140.0], [400.0, 140.0]])
    assert live_field.edge_margin_px(quad, (1080, 1920)) == 100.0


def test_edge_margin_takes_the_smallest_side_when_a_corner_touches_the_top():
    quad = np.float64([[400.0, 3.0], [440.0, 3.0], [440.0, 43.0], [400.0, 43.0]])
    assert live_field.edge_margin_px(quad, (1080, 1920)) == 3.0


def test_edge_margin_measures_the_frame_side_not_the_nearest_corner():
    """四角都在画面内、但整体离右边缘只有 12 px → 12.0。

    "四个边上的最小值"是 brief 定的判据：实现成"离最近的角点/别的码有多远"，这个码会被
    判成健康，而它正是今天被切的那一类。
    """
    quad = np.float64([[1900.0, 400.0], [1908.0, 400.0], [1908.0, 440.0], [1900.0, 440.0]])
    dists = live_field.edge_distances_px(quad, (1080, 1920))
    assert dists["right"] == 12.0
    assert live_field.edge_margin_px(quad, (1080, 1920)) == 12.0


def test_edge_margin_is_negative_when_the_marker_is_cut_by_the_frame():
    quad = np.float64([[1900.0, 400.0], [1925.0, 400.0], [1925.0, 440.0], [1900.0, 440.0]])
    assert live_field.edge_margin_px(quad, (1080, 1920)) == -5.0
    assert live_field.edge_warnings({7: _pose_at(quad)}, (1080, 1920)) \
        == ["ID7 超右边缘 5px ⚠被切"]


def test_edge_margin_on_the_reference_photo_reproduces_the_recorded_six_pixels():
    """R27 记的"ID20 上边离画面边界只有 6 px"必须能由代码算出来 —— 这正是今天的判据。"""
    img = _fixture_image()
    poses = detect_markers(img, _default_calib())
    top = live_field.edge_distances_px(poses[20].corners_px, img.shape[:2])["top"]
    assert top == pytest.approx(PHOTO_ID20_TOP_PX, abs=1e-6)


def test_edge_warnings_name_the_offending_markers_and_stay_silent_when_healthy():
    img = _fixture_image()
    poses = detect_markers(img, _default_calib())
    warnings = live_field.edge_warnings(poses, img.shape[:2])
    # 参考照片上 ID20 贴上边缘、ID21 贴右边缘；其余三个健康（噪声必须能被"不在"表达出来）。
    assert warnings == ["ID20 距上边缘 6px ⚠被切", "ID21 距右边缘 16px ⚠被切"]

    roomy = np.float64([[500.0, 300.0], [540.0, 300.0], [540.0, 340.0], [500.0, 340.0]])
    assert live_field.edge_warnings({20: _pose_at(roomy)}, (1080, 1920)) == []


def test_edge_margin_rejects_a_degenerate_quad():
    with pytest.raises(ValueError):
        live_field.edge_margin_px(np.float64([[0.0, 0.0], [1.0, 1.0]]), (1080, 1920))


# --- summarize：终端行与横幅的共同来源 ---------------------------------------------


def test_summarize_ready_line_carries_ready_x_y_and_yaw():
    img = _fixture_image()
    calib = _default_calib()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    assert result.ok

    line = live_field.summarize(result, poses)
    assert line.startswith("READY  ")
    assert f"x={verify_vision.signed(result.x_mm)}" in line
    assert f"y={verify_vision.signed(result.y_mm)}" in line
    assert f"yaw={verify_vision.signed(result.yaw_deg)}" in line
    # 每个检出的码都带中心像素：操作者不盯着窗口也能读到码在哪。
    for marker_id in sorted(poses):
        cx, cy = live_field.marker_center_px(poses[marker_id])
        assert f"ID{marker_id}@({cx},{cy})" in line


def test_summarize_failure_carries_the_reason_verbatim_and_the_missing_ids():
    img = _cover(_fixture_image(), COVER_BOX[20])
    calib = _default_calib()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "broken", "broken.jpg")
    assert not result.ok
    assert 20 not in poses, "反例不成立：盖完 ID20 还检得出来"

    line = live_field.summarize(result, poses)
    assert line.startswith("boundary_moved")
    assert "缺[20]" in line
    assert "ID21@" in line


def test_summarize_uses_the_reason_word_for_a_missing_vehicle_marker_too():
    """词表是 Task 11 定的那五个，`no_vehicle_marker` 与 `boundary_moved` 不是一回事。"""
    img = _cover(_fixture_image(), COVER_BOX[29])
    calib = _default_calib()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "no29", "no29.jpg")
    assert not result.ok and 29 not in poses

    line = live_field.summarize(result, poses)
    assert line.startswith("no_vehicle_marker")
    assert "缺[29]" in line and "boundary_moved" not in line


# --- 车体原点的像素位置：bias 的符号与参考系 ---------------------------------------


def _synthetic_scene(rvec_deg_z: float = 90.0):
    """一个可手算的场景：场地系与相机系对齐、场地原点落在图像中心、ID29 也在图像中心。

    `R_cam2field = I` 意味着 场地 +X = 图像右、+Y = 图像下 —— 对俯视相机这是正确取向，
    于是"像素往哪边动"与"场地系往哪边动"一一对应，符号钉得住。
    """
    K = np.float64([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
    ff = FieldFrame(R_cam2field=np.eye(3), origin_cam=np.float64([0.0, 0.0, 500.0]),
                    marker_px={}, normal_cam=np.float64([0.0, 0.0, -1.0]))
    rvec = np.float64([[0.0], [0.0], [math.radians(rvec_deg_z)]])
    pose = MarkerPose(marker_id=29, rvec=rvec, tvec=np.float64([[0.0], [0.0], [500.0]]),
                      reproj_rms_px=0.0,
                      corners_px=np.float64([[960.0, 540.0], [980.0, 540.0],
                                              [980.0, 560.0], [960.0, 560.0]]))
    return ff, pose, K


def test_body_origin_pixel_moves_the_way_the_bias_says():
    """A1 的偏置符号钉死。

    场景里码绕相机 Z 转了 90°，所以它的规范 +Y（角点3→角点0）指向图像**左**。
    `p_body = tvec − R_cam2marker @ [dx, dy, 0]`，于是 `dy_mm = +40` 把车体原点推到
    图像**右**边 —— 同时场地区 x 变成 +40 mm（`to_field` 的方向与像素方向一致）。

    用错矩阵（把 `R_cam2field` 当成码的旋转）会把它推到图像**上**边：同样是一个正常的
    十字、同样有一个正常的 `(x, y)`，没有一处会报错 —— A1 的数就是这么被写错的。
    """
    ff, pose, K = _synthetic_scene(rvec_deg_z=90.0)

    for dy_mm, expect_right in ((40.0, True), (-40.0, False)):
        bias = {"dx_mm": 0.0, "dy_mm": dy_mm, "dtheta_deg": 0.0}
        px = live_field.project_point(K, live_field.body_origin_cam(pose, bias))
        assert (px[0] > 960.0) is expect_right, f"dy_mm={dy_mm} 推错了方向：{px}"
        assert px[1] == pytest.approx(540.0, abs=1e-6)          # 没有上下分量
        # 场地区落点与 field.py 的权威实现一致
        assert to_field(ff, live_field.body_origin_cam(pose, bias))[:2] \
            == pytest.approx(body_pose_in_field(ff, pose, bias)[:2], abs=1e-9)

    wrong = np.float64([0.0, 0.0, 500.0]) - (np.eye(3) @ np.float64([0.0, 40.0, 0.0]))
    assert live_field.project_point(K, wrong)[1] < 540.0, "反例不成立：用错矩阵落到了同一侧"


def test_body_origin_pixel_moves_in_x_for_dx_in_the_unrotated_case():
    """码不转（`R = I`）时 `dx_mm` 是纯图像水平方向。

    方向由 `p_body = tvec − R@[dx,dy,0]` 决定：`dx_mm = +30` 意味着**码**在车体原点的
    码 +X 方向 30 mm，所以车体原点在码的 −X 方向 —— 图像**左** 60 px（1000·30/500）。
    这正是 R35 的读法（bias = 码相对车体原点的偏移），别把它当成"车体原点朝码那边偏"。
    """
    _ff, pose, K = _synthetic_scene(rvec_deg_z=0.0)
    px = live_field.project_point(K, live_field.body_origin_cam(
        pose, {"dx_mm": 30.0, "dy_mm": 0.0, "dtheta_deg": 0.0}))
    assert px[0] == pytest.approx(960.0 - 1000.0 * 30.0 / 500.0, abs=1e-6)
    assert px[1] == pytest.approx(540.0, abs=1e-6)


def test_body_origin_pixel_agrees_with_body_pose_in_field_on_the_real_photo():
    """叠加层上的十字与横幅里的 `(x, y)` 必须是同一个点的两种表示。

    bias 全 0 时这条恒成立（看不出符号对不对），所以这里扫一组**非零**偏置 ——
    A1 一填上真实偏置，两者必须仍然重合。
    """
    img = _fixture_image()
    calib = _default_calib()
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    assert result.ok
    ff, pose = result.field_frame, result.vehicle_pose

    for bias in ({"dx_mm": 0.0, "dy_mm": 55.0, "dtheta_deg": 90.0},
                 {"dx_mm": -30.0, "dy_mm": 0.0, "dtheta_deg": -90.0},
                 {"dx_mm": 12.5, "dy_mm": -7.25, "dtheta_deg": 89.5}):
        point = live_field.body_origin_cam(pose, bias)
        assert to_field(ff, point)[:2] \
            == pytest.approx(body_pose_in_field(ff, pose, bias)[:2], abs=1e-9), bias
        assert live_field.project_point(calib["K"], point, calib["dist"]) is not None

    # bias = 0（今天的实际状态）时，车体原点就落在车载码的中心像素上。
    zero = live_field.project_point(calib["K"], live_field.body_origin_cam(pose, calib["bias"]))
    assert zero == pytest.approx(live_field.marker_center_px(pose), abs=1.0)


def test_field_point_cam_inverts_to_field_coordinates():
    """场地系平面上的点 → 相机系 → 再 `to_field` 回去必须原样（坐标轴转置没搞反）。"""
    ff, _pose, _K = _synthetic_scene()
    for u, v in ((0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (-37.5, 88.25)):
        back = to_field(ff, live_field.field_point_cam(ff, u, v))
        assert back[0] == pytest.approx(u, abs=1e-9)
        assert back[1] == pytest.approx(v, abs=1e-9)


def test_field_axes_point_the_way_the_field_frame_says():
    """参考照片上 场地 +X = ID20→ID21 = 图像**右**、+Y = ID20→ID23 = 图像**下**（R4/R34）。"""
    calib = _default_calib()
    img = _fixture_image()
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    ff, K = result.field_frame, calib["K"]
    origin = live_field.project_point(K, ff.origin_cam, calib["dist"])
    tip_x = live_field.project_point(K, live_field.field_point_cam(ff, live_field.AXIS_LEN_MM, 0.0),
                                     calib["dist"])
    tip_y = live_field.project_point(K, live_field.field_point_cam(ff, 0.0, live_field.AXIS_LEN_MM),
                                     calib["dist"])
    assert tip_x[0] > origin[0], "场地 +X 必须指向图像右"
    assert tip_y[1] > origin[1], "场地 +Y 必须指向图像下"


def test_projection_rejects_points_behind_or_on_the_camera_plane():
    _ff, _pose, K = _synthetic_scene()
    assert live_field.project_point(K, np.float64([0.0, 0.0, -1.0])) is None
    assert live_field.project_point(K, np.float64([0.0, 0.0, 0.0])) is None
    assert live_field.project_point(K, np.float64([0.0, 0.0, np.nan])) is None


def test_projection_with_and_without_distortion_agree_on_the_current_calibration():
    """`calib_intrinsics_chess.json` 的 dist 全是 0：两条路必须给出同一个像素。

    参数保留是为了 `calib_distortion.py` 之后不用改这里；今天两条路若不等，
    就说明叠加层与检测结果已经不在同一个像素空间里了。
    """
    calib = _default_calib()
    assert len(calib["dist"]) == 5 and not any(calib["dist"])
    poses = detect_markers(_fixture_image(), calib)
    for marker_id in sorted(poses):
        point = poses[marker_id].tvec.reshape(3)
        assert live_field.project_point(calib["K"], point) \
            == pytest.approx(live_field.project_point(calib["K"], point, calib["dist"]), abs=1e-6)


def test_clamp_px_keeps_drawn_points_inside_the_canvas():
    shape = (1080, 1920)
    assert live_field.clamp_px((-500, -500), shape) == (8, 8)
    assert live_field.clamp_px((99999, 99999), shape) == (1911, 1071)
    assert live_field.clamp_px((100.4, 200.6), shape) == (100, 201)


# --- 真管道一致性：不分叉、不复制 ---------------------------------------------------


def test_both_files_end_with_a_newline_and_are_utf8():
    """`d8850c4` 立的规矩：末尾换行 + UTF-8。

    少了末尾换行，git diff 里那行会显示成 "\\ No newline at end of file"，而仓库里
    有别的工具按行处理文本 —— 这条不是洁癖。
    """
    for name in (SCRIPT, "视觉测量/脚本/vision_core/tests/test_live_field.py"):
        raw = calib_store.resolve_path(name).read_bytes()
        raw.decode("utf-8")                                  # 不是 UTF-8 就抛
        assert raw.endswith(b"\n"), f"{name} 末尾没有换行"
        assert not raw.endswith(b"\n\n"), f"{name} 末尾多了一个空行"


def test_live_field_never_forks_the_detector():
    """本任务要消灭的分叉：自设 `DetectorParameters` / 自己调 `detectMarkers` 的副本。

    `live_aruco.py` 就是这么做的，于是它显示"找到码"而 `verify_vision.py` 仍然失败。

    这里扫的是 **AST 的属性名与函数名**，不是源码文本：模块 docstring 里必须能写清楚
    "为什么不能自己设 `adaptiveThreshWinSize*`"，写了就会把纯文本扫描变成自欺欺人。
    """
    src = calib_store.resolve_path(SCRIPT).read_text(encoding="utf-8")
    tree = ast.parse(src)
    used = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    used |= {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    for banned in ("detectMarkers", "DetectorParameters_create", "adaptiveThreshWinSizeMin",
                   "adaptiveThreshConstant", "cornerRefinementMethod", "Dictionary_get"):
        assert banned not in used, f"live_field.py 里出现了自设检测：{banned}"
    # 反向确认：真管道的两个入口都在（模块里确实写着它们的名字）。
    assert "solve_image" in used and "detect_markers" in used
    # 画框是允许的（`drawDetectedMarkers` 只画不检），但不许出现另一个 detect 调用点。
    assert "drawDetectedMarkers" in used


def test_camera_constants_come_from_verify_vision():
    assert live_field.CAMERA_DEVICE == verify_vision.CAMERA_DEVICE == "/dev/video0"
    assert live_field.CAPTURE_WIDTH == verify_vision.CAPTURE_WIDTH == 1920
    assert live_field.CAPTURE_HEIGHT == verify_vision.CAPTURE_HEIGHT == 1080
    assert live_field.CAPTURE_FOURCC == verify_vision.CAPTURE_FOURCC == "MJPG"
    assert live_field.DEFAULT_SETTLE_FRAMES == verify_vision.DEFAULT_SETTLE_FRAMES


def test_detecting_markers_twice_on_one_frame_gives_the_same_poses():
    """同一帧上跑两遍 `detect_markers`（叠加层一遍、`solve_image` 里一遍）必须是同一份结果。

    这是"每帧跑两次检测"这个取舍的前提：它换来的性质必须成立，否则两次解算之间车动了，
    叠加层画的就是**上一刻**的位姿，而画面上没有任何东西会提示这一点。

    （这条只证"检测是确定性的"。`run_window` 把这一份位姿**接到**叠加层上，是另一条接线，
    由 `test_run_window_snapshots_a_frame_that_solve_image_can_still_read` 钉住 ——
    那条才是能藏住"快照存成叠加帧"这种 bug 的。）
    """
    img = _fixture_image()
    calib = _default_calib()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    assert result.ok
    assert result.vehicle_pose.marker_id == calib_store.VEHICLE_ID
    assert np.allclose(result.vehicle_pose.corners_px,
                       poses[calib_store.VEHICLE_ID].corners_px)
    assert np.allclose(result.field_frame.origin_cam, build_field_frame(poses).origin_cam)


def test_the_snapshot_default_directory_is_outside_the_repo():
    """快照默认不落进仓库：它是调取景的中间产物，不是证据（证据走 verify_vision --capture）。"""
    default = live_field.DEFAULT_SNAPSHOT_DIR
    assert default.startswith("/tmp/")
    assert calib_store.REPO_ROOT not in Path(default).parents
    assert "视觉测量" not in default


# --- import 安全性与相机 ------------------------------------------------------------


def test_importing_live_field_touches_no_camera_and_no_window(monkeypatch):
    """`import live_field` 不许开设备、不许建窗口（测试与 CI 都靠它能 import）。"""
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    monkeypatch.delenv("QT_QPA_PLATFORM", raising=False)

    def boom(*_args, **_kwargs):
        raise AssertionError("模块顶层不许碰相机/窗口")

    for name in ("VideoCapture", "namedWindow", "imshow"):
        monkeypatch.setattr(cv2, name, boom)

    spec_path = calib_store.resolve_path(SCRIPT)
    code = compile(spec_path.read_text(encoding="utf-8"), str(spec_path), "exec")
    namespace = {"__name__": "live_field_import_probe", "__file__": str(spec_path)}
    exec(code, namespace)                      # 这就是要验的那件事本身
    assert callable(namespace["edge_margin_px"])
    assert namespace["EXIT_CAMERA_UNAVAILABLE"] == 4


def test_open_camera_sets_the_pipeline_format_and_keeps_autofocus_on(monkeypatch):
    """分辨率/FourCC 必须与 `verify_vision.grab_frame` 一致，自动对焦保持开启。"""

    class Cap(FakeCap):
        def __init__(self, *_args):
            super().__init__([_fixture_image()])

        def isOpened(self):
            return True

    monkeypatch.setattr(cv2, "VideoCapture", Cap)
    cap = live_field.open_camera("/dev/video0")
    props = dict(cap.sets)
    assert props[cv2.CAP_PROP_FOURCC] == cv2.VideoWriter_fourcc(*live_field.CAPTURE_FOURCC)
    assert props[cv2.CAP_PROP_FRAME_WIDTH] == live_field.CAPTURE_WIDTH
    assert props[cv2.CAP_PROP_FRAME_HEIGHT] == live_field.CAPTURE_HEIGHT
    assert props[cv2.CAP_PROP_AUTOFOCUS] == 1                # 本任务不锁对焦
    assert cv2.CAP_PROP_EXPOSURE not in props, "不该去锁曝光"


def test_a_busy_camera_is_one_readable_line_not_a_traceback(monkeypatch, capsys):
    class DeadCap:
        def __init__(self, *_args):
            pass

        def isOpened(self):
            return False

        def release(self):
            pass

    monkeypatch.setattr(cv2, "VideoCapture", DeadCap)
    with pytest.raises(live_field.CameraUnavailable) as exc:
        live_field.open_camera("/dev/video9")
    assert "占用" in str(exc.value)

    rc = live_field.main(["--once", "--device", "/dev/video9"])
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_CAMERA_UNAVAILABLE == 4
    assert "占用" in out and "Traceback" not in out


def test_settle_discards_buffer_frames_without_decoding():
    """R39：启动那一刻必须丢缓冲帧等自动对焦收敛（实时循环里不每帧丢）。"""
    cap = FakeCap([_fixture_image()])
    live_field.settle(cap, frames=7)
    assert cap.grabbed == 7
    assert cap.index == 0                      # grab 不解码


# --- 无窗口命令行路径 --------------------------------------------------------------


def test_once_prints_a_ready_line_from_a_solvable_frame_without_a_window(monkeypatch, capsys):
    def boom(*_args, **_kwargs):
        raise AssertionError("--once 不许开窗口")

    for name in ("namedWindow", "imshow"):
        monkeypatch.setattr(cv2, name, boom)

    rc, cap = _run_headless(monkeypatch, [_fixture_image()], ["--once"])
    out = capsys.readouterr().out

    assert rc == live_field.EXIT_OK == 0
    assert "READY" in out and "ID20@(" in out and "ID29@(" in out
    assert "x=" in out and "yaw=" in out
    assert "Traceback" not in out
    assert cap.released == 1, "退出时必须 release 相机"


def test_once_together_with_frames_one_is_the_acceptance_command(monkeypatch, capsys):
    """验收命令是 `--once --frames 1`：两个标志不是互斥的，按 --frames 的帧数解算。"""
    rc, cap = _run_headless(monkeypatch, [_fixture_image(), _fixture_image()],
                            ["--once", "--frames", "1"])
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_OK
    assert cap.index == 1
    assert "1/1 帧 READY" in out


def test_frames_solves_exactly_n_frames(monkeypatch, capsys):
    rc, cap = _run_headless(monkeypatch, [_fixture_image()] * 5, ["--frames", "5"])
    out = capsys.readouterr().out
    assert cap.index == 5
    assert rc == live_field.EXIT_OK
    assert "5/5 帧 READY" in out
    assert "ms/帧" in out and "两次 detect_markers" in out     # 压测要给出每帧耗时


def test_an_unsolvable_frame_exits_one_and_prints_the_reason_word(monkeypatch, capsys):
    """今天实拍就是这个状态：相机正常，帧拿到了，但解不出来（取景问题，R48）。"""
    broken = _cover(_fixture_image(), COVER_BOX[20])
    rc, _cap = _run_headless(monkeypatch, [broken], ["--once"])
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_PHOTO_FAILED == 1
    assert "boundary_moved" in out and "缺[20]" in out
    assert "0/1 帧 READY" in out


def test_the_terminal_line_is_printed_only_when_the_state_changes(monkeypatch, capsys):
    """状态不变就不重复打：否则实时模式会把终端刷爆，操作者 `tail` 不到东西。"""
    good = _fixture_image()
    bad = _cover(good.copy(), COVER_BOX[20])
    rc, _cap = _run_headless(monkeypatch, [good, good, good, bad, bad, good], ["--frames", "6"])
    lines = [ln for ln in capsys.readouterr().out.splitlines() if ln.startswith("[")]
    assert rc == live_field.EXIT_PHOTO_FAILED
    assert len(lines) == 3, lines                    # 好 →（好 好）→ 坏 →（坏）→ 好
    assert "READY" in lines[0] and "boundary_moved" in lines[1] and "READY" in lines[2]


def test_the_terminal_log_cools_down_so_jitter_does_not_flood_it(capsys):
    """边缘上抖动的码（ID22/ID23 时有时无、中心差 1 px）不能把终端刷爆。

    实测：开着窗口跑 1 秒会打 12 行，全是同一个意思。冷却把频率按住，而
    "短暂的变化只是**延后**、不丢弃"是这条设计的要点。
    """
    good = _fixture_image()
    bad = _cover(good.copy(), COVER_BOX[20])
    assert live_field.TERMINAL_MIN_INTERVAL_S > 0.0

    log = live_field.TerminalLog(interval=60.0)          # 冷却期内不会打第二行
    for i in range(40):
        log.report(*_pose_result(good if i % 2 == 0 else bad))
    lines = [ln for ln in capsys.readouterr().out.splitlines() if ln.startswith("[")]
    assert len(lines) == 1 and "READY" in lines[0]      # 40 帧抖动 → 1 行

    log.last_at = time.perf_counter() - 120.0            # 模拟冷却过去
    log.report(*_pose_result(bad))
    lines = [ln for ln in capsys.readouterr().out.splitlines() if ln.startswith("[")]
    assert len(lines) == 1 and "boundary_moved" in lines[0]

    log.last_at = time.perf_counter() - 120.0
    log.report(*_pose_result(good))                      # 冷却期内发生的变化，现在才打出来
    lines = [ln for ln in capsys.readouterr().out.splitlines() if ln.startswith("[")]
    assert len(lines) == 1 and "READY" in lines[0]


def _pose_result(img):
    """`(result, poses)`：给纯函数测试用的一对，避免每处都写两行。"""
    calib = _default_calib()
    return live_field.solve_image(img, calib, "p", "p"), detect_markers(img, calib)


def test_a_read_failure_is_one_line_not_a_traceback(monkeypatch, capsys):
    rc, cap = _run_headless(monkeypatch, [_fixture_image()], ["--frames", "3"], drop_after=1)
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_PHOTO_FAILED
    assert "读帧失败" in out and "Traceback" not in out
    assert cap.released == 1


def test_negative_frames_is_a_usage_error(capsys):
    with pytest.raises(SystemExit) as exc:
        live_field.main(["--frames", "-2"])
    assert exc.value.code == live_field.EXIT_USAGE == 2
    assert "不能为负" in capsys.readouterr().err


def test_a_zero_scale_is_a_usage_error(capsys):
    with pytest.raises(SystemExit) as exc:
        live_field.main(["--scale", "0"])
    assert exc.value.code == 2
    assert "--scale" in capsys.readouterr().err


def test_help_runs_and_documents_the_keys_and_exit_codes(capsys):
    with pytest.raises(SystemExit) as exc:
        live_field.main(["--help"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    for token in ("--once", "--frames", "--scale", "--snapshot-dir",
                  "ESC", "存快照", "退出码", "4 = 相机打不开",
                  "没有显示环境"):
        assert token in out, token


# --- 叠加绘制 ----------------------------------------------------------------------


def _axis_x_pixels(frame: np.ndarray) -> np.ndarray:
    """画面上"场地 +X 轴"那种洋红（BGR 255,0,255）的像素掩码。

    黑画布上只有本脚本会画它（黄/红/绿/白都不撞这个色），所以它同时是
    "画没画坐标轴"的判据与"颜色有没有搞反 BGR"的探针。
    """
    return (frame[:, :, 0] > 200) & (frame[:, :, 1] < 60) & (frame[:, :, 2] > 200)


def test_overlay_text_is_drawn_in_bgr_not_rgb():
    """PIL 出的是 RGB，`_blend` 贴进 OpenCV 的 BGR 画布必须换通道。

    换反了不会报错，只会把"红色失败横幅"画成蓝色、把黄色 A1 警告画成橙色 ——
    而操作者正是靠颜色区分这三行的。用黑画布取像素，颜色一写错就会红。
    """
    calib = _default_calib()
    bad = _cover(_fixture_image(), COVER_BOX[20])
    poses = detect_markers(bad, calib)
    result = live_field.solve_image(bad, calib, "bad", "bad.jpg")
    assert not result.ok

    canvas = np.zeros_like(bad)
    live_field.draw_overlay(canvas, result, poses, calib, bias_measured=False)
    band = canvas[:80]
    red = (band[:, :, 2] > 150) & (band[:, :, 1] < 100) & (band[:, :, 0] < 100)
    blue = (band[:, :, 0] > 150) & (band[:, :, 2] < 100)
    assert red.any(), "失败横幅应该是红色"
    assert not blue.any(), "红色被画成了蓝色：BGR/RGB 通道搞反了"


def test_draw_overlay_paints_the_body_cross_at_the_solved_point_and_a_banner():
    img = _fixture_image()
    calib = _default_calib()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    assert result.ok

    canvas = np.zeros_like(img)          # 黑画布：画出来的每个像素都是本脚本画的
    out = live_field.draw_overlay(canvas, result, poses, calib,
                                  bias_measured=False, fps=20.0)
    assert out is canvas, "draw_overlay 必须把画好的画布原样返回"
    assert live_field.edge_warnings(poses, canvas.shape[:2]) \
        == ["ID20 距上边缘 6px ⚠被切", "ID21 距右边缘 16px ⚠被切"]

    # 车体原点十字是红的（BGR 0,0,255），且必须落在解出来的那个点上。
    bx, by = live_field.project_point(
        calib["K"], live_field.body_origin_cam(result.vehicle_pose, calib["bias"]), calib["dist"])
    patch = canvas[int(by) - 30:int(by) + 30, int(bx) - 30:int(bx) + 30]
    red = (patch[:, :, 2] > 180) & (patch[:, :, 1] < 80) & (patch[:, :, 0] < 80)
    assert red.any(), "车体原点十字没画在解出来的那个点上"

    # 场地坐标轴（只有场地系建起来时才画）
    assert _axis_x_pixels(canvas).any()
    # 横幅在顶部：那里有亮像素（文字）；左下角有比例尺文字
    assert (canvas[:80, :, :].max(axis=2) > 150).any()
    assert (canvas[-80:, :, :].max(axis=2) > 150).any()


def test_draw_overlay_does_not_invent_a_field_frame_on_a_failed_frame():
    """`boundary_moved` 的那一帧根本没有场地系：画出来的坐标轴是编的，所以不画。"""
    bad = _cover(_fixture_image(), COVER_BOX[20])
    calib = _default_calib()
    poses = detect_markers(bad, calib)
    result = live_field.solve_image(bad, calib, "bad", "bad.jpg")
    assert not result.ok and result.field_frame is None

    canvas = np.zeros_like(bad)
    live_field.draw_overlay(canvas, result, poses, calib, bias_measured=False)
    assert not _axis_x_pixels(canvas).any(), "没有场地系却画出了坐标轴"
    assert live_field.edge_warnings(poses, canvas.shape[:2]) == ["ID21 距右边缘 16px ⚠被切"]


def test_an_annotated_frame_is_unreadable_which_is_why_the_overlay_gets_a_copy():
    """反例钉子：叠加会把码的黑边糊掉，画完的图**一个码都检不出来**。

    没有这条，下游任何"顺手把叠加后的 `frame` 存下来 / 传下去"的改动都会静悄悄地
    毁掉证据，而画面上看起来一切正常 —— 快照还是"存下来了"，只是里面没有码。
    """
    img = _fixture_image()
    calib = _default_calib()
    assert sorted(detect_markers(img, calib)) == [20, 21, 22, 23, 29]

    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    annotated = live_field.draw_overlay(img.copy(), result, poses, calib, bias_measured=False)

    assert sorted(detect_markers(annotated, calib)) == []
    assert not live_field.solve_image(annotated, calib, "annotated", "annotated.jpg").ok
    # 原图当然不受影响
    assert sorted(detect_markers(img, calib)) == [20, 21, 22, 23, 29]


def test_grid_toggle_changes_the_pixels():
    img = _fixture_image()
    calib = _default_calib()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    plain, gridded = img.copy(), img.copy()
    live_field.draw_overlay(plain, result, poses, calib)
    live_field.draw_overlay(gridded, result, poses, calib, show_grid=True)
    assert not np.array_equal(plain, gridded)


def test_banner_says_the_bias_is_unmeasured_only_until_a1_is_done():
    calib = _default_calib()
    img = _fixture_image()
    poses = detect_markers(img, calib)
    result = live_field.solve_image(img, calib, "photo", PHOTO)
    assert result.ok

    # R37：READY 那一行本身不带 A1 警告，警告在独立的混合行里。
    rows = live_field.banner_lines(result, poses, bias_measured=True)
    assert len(rows) == 1 and rows[0][1] == live_field.COLOR_OK

    assert live_field.warning_strip(True, []) == []
    assert live_field.warning_strip(False, [])[0][0] == live_field.A1_WARNING
    # 有边缘告警时两件事挤在同一行（横幅必须矮：它盖住的正是贴边的那个码）
    strip = live_field.warning_strip(False, ["ID20 距上边缘 3px ⚠被切"])
    assert len(strip) == 2 and strip[1][1] == live_field.COLOR_BAD
    assert "车载码的中心" in live_field.A1_WARNING


def test_snapshot_never_overwrites_and_stays_usable_by_verify_vision(tmp_path):
    """快照存的是**未叠加的原帧**：拿它跑 `verify_vision.solve_image` 必须还能解出同一个位姿。"""
    img = _fixture_image()
    calib = _default_calib()
    first = live_field.save_snapshot(img, tmp_path / "snaps")
    second = live_field.save_snapshot(img, tmp_path / "snaps")
    assert first != second and first.exists() and second.exists()
    assert first.is_absolute() and first.suffix == ".jpg"

    back = cv2.imread(str(first))
    assert back is not None
    solved = verify_vision.solve_image(back, calib, "snap", str(first))
    reference = verify_vision.solve_image(img, calib, "photo", PHOTO)
    assert solved.ok
    assert solved.x_mm == pytest.approx(reference.x_mm, abs=0.5)
    assert solved.yaw_deg == pytest.approx(reference.yaw_deg, abs=0.5)


def test_a_subprocess_run_of_once_needs_no_display(tmp_path):
    """无 DISPLAY、无窗口的环境里 `--once` 必须跑得出结果（CI 与本机验收都靠这条）。

    `print_header` 在**开相机之前**就打，所以断言"标定文件"那行就能证明脚本确实跑到了
    该跑的地方 —— 之前这条用 `returncode != 4` 一带而过，CI 没相机时等于什么都没验。
    """
    env = {k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
    proc = subprocess.run(
        [sys.executable, str(calib_store.resolve_path(SCRIPT)), "--once",
         "--calib", str(tmp_path / "car_calib.json")],
        cwd=str(tmp_path), env=env, capture_output=True, text=True, timeout=300,
    )
    assert "Traceback" not in proc.stderr
    assert "模式：无窗口" in proc.stdout
    assert "标定文件" in proc.stdout, "连表头都没打出来：脚本没跑到解算那一步"
    assert proc.returncode in (0, 1, 4), proc.stdout + proc.stderr   # 4 = 相机被占着
    if proc.returncode != 4:
        assert "READY" in proc.stdout or "boundary_moved" in proc.stdout


# --- 窗口模式的接线（`run_window` 的真实顺序）-------------------------------------


def _drive_window(monkeypatch, frames, keys, argv_extra=(), raiser=None, raiser_name="",
                  drop_after=None, interrupt_after=None, swallow_interrupt=False):
    """用假 GUI + 假相机跑一次真正的 `run_window`，返回 `(rc, 显示过的图, cap)`。

    这条路径**不 stub 掉** `detect_markers` / `solve_image` / `draw_overlay` / `save_snapshot`
    —— 要验的恰恰是它们**串在一起**的顺序（复核的 Critical 1 就藏在这段接线里：
    叠加就地改写了 `frame`，于是 `s` 存下来的是画过框的图）。
    `keys` 是逐帧喂给 `waitKey` 的按键序列；喂完 `q` 收尾。

    `drop_after=N` 让相机在第 N 帧之后读不出来（中途掉线），`interrupt_after=N` 让第
    N 次 `waitKey` 抛 `KeyboardInterrupt`（操作者按 Ctrl-C）。两者都不是"可变异检测的
    覆盖"，而是让这两条会话终止路径**跑得到**的前向性夹具。

    `swallow_interrupt=True` 把穿出 `main()` 的 `KeyboardInterrupt` 收下来（rc 记成
    None）：不这么做的话，Ctrl-C 那条用例在修复被撤掉时会把**整个 pytest 会话**连同
    正在跑的其它用例一起掀掉，而不是干净地红一条。
    """
    cap = FakeCap(frames, drop_after=drop_after)
    shown: list = []
    destroyed: list = []

    monkeypatch.setattr(live_field, "open_camera", lambda *a, **k: cap)
    monkeypatch.setattr(live_field, "DEFAULT_SETTLE_FRAMES", 0)
    monkeypatch.setattr(live_field, "TERMINAL_MIN_INTERVAL_S", 0.0)
    monkeypatch.setattr(live_field, "_has_display", lambda: True)   # 显示检查交给别的用例
    monkeypatch.setattr(cv2, "namedWindow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "resizeWindow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "moveWindow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: destroyed.append(True))
    monkeypatch.setattr(cv2, "imshow", lambda _win, img: shown.append(img.copy()))

    pending = list(keys)
    waited = {"n": 0}

    def fake_wait_key(_delay):
        if interrupt_after is not None and waited["n"] >= interrupt_after:
            # 与操作者按 Ctrl-C 一样：KeyboardInterrupt 从 waitKey 穿出主循环。
            raise KeyboardInterrupt
        waited["n"] += 1
        return pending.pop(0) if pending else ord("q")

    monkeypatch.setattr(cv2, "waitKey", fake_wait_key)
    if raiser is not None:
        def boom(*_a, **_k):
            raise cv2.error(raiser)

        monkeypatch.setattr(cv2, raiser_name, boom)
    rc = None
    if swallow_interrupt:
        try:
            rc = live_field.main(list(argv_extra))
        except KeyboardInterrupt:
            rc = None                      # 修复被撤掉时的真实形态，由用例断言出来
    else:
        rc = live_field.main(list(argv_extra))
    assert destroyed, "无论怎么退出，destroyAllWindows 都必须跑（窗口/相机别留在操作者桌面上）"
    return rc, shown, cap


def test_run_window_snapshots_a_frame_that_solve_image_can_still_read(monkeypatch, tmp_path, capsys):
    """按 `s` 存下来的必须是**能解**的原帧 —— 与叠加之前那一帧逐项一致。

    复核的 Critical 1：叠加（`drawDetectedMarkers` 的框 + `ID=xx` 标签 + 红色粗多边形）
    把码的黑边糊掉，`detect_markers` 在画完的图上**五个码一个都检不出来**；
    旧接线 `draw_overlay(frame, …)` 就地改写 `frame`，`s` 存的正是这张图。
    之前那条用例直接拿干净数组调 `save_snapshot`，验的是函数没坏，不是"存出来的东西能用"。
    """
    img = _fixture_image()
    calib = _default_calib()
    before = verify_vision.solve_image(img, calib, "live", live_field.CAMERA_DEVICE)
    assert before.ok

    rc, shown, cap = _drive_window(
        monkeypatch, [img], keys=[ord("s"), ord("q")],
        argv_extra=["--snapshot-dir", str(tmp_path / "snaps")])
    out = capsys.readouterr().out

    assert rc == live_field.EXIT_OK
    saved = sorted((tmp_path / "snaps").glob("*.jpg"))
    assert len(saved) == 1 and str(saved[0].resolve()) in out, out
    assert cap.released == 1

    after = verify_vision.solve_image(cv2.imread(str(saved[0])), calib, "snap", str(saved[0]))
    assert after.ok, f"存下来的快照解不出来：{after.reason} {after.detail}"
    assert after.x_mm == pytest.approx(before.x_mm, abs=0.5)
    assert after.y_mm == pytest.approx(before.y_mm, abs=0.5)
    assert after.yaw_deg == pytest.approx(before.yaw_deg, abs=0.5)
    assert after.reproj_rms_px == pytest.approx(before.reproj_rms_px, abs=0.05)

    # 叠加只落在**显示副本**上：imshow 拿到的是画过的图，原帧原封不动。
    assert shown and not np.array_equal(shown[0], img)
    assert sorted(detect_markers(shown[0], calib)) == []        # 叠加确实毁掉了可读性
    assert sorted(detect_markers(img, calib)) == [20, 21, 22, 23, 29]


def test_window_mode_exits_one_when_not_a_single_frame_was_ever_solved(monkeypatch, capsys):
    """整场一帧都没解出来 → `EXIT_PHOTO_FAILED`，与文档、与 `run_headless` 的口径都不冲突。

    之前的 bug：窗口模式无条件返回 `EXIT_OK`，而退出码表写着"0 = 每帧都 READY" ——
    同样一屏 `boundary_moved`，`--frames 2` 给 1、窗口模式给 0，按 `$?` 判断的脚本
    在窗口模式下拿到假通过。
    """
    bad = _cover(_fixture_image(), COVER_BOX[20])
    rc, _shown, _cap = _drive_window(monkeypatch, [bad, bad, bad], keys=[0, 0, ord("q")])
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_PHOTO_FAILED == 1
    assert "boundary_moved" in out

    # 对照组：无窗口模式同样内容也是 1（那边是"任何一帧没解出来就算失败"）
    rc_headless, _cap = _run_headless(monkeypatch, [bad, bad], ["--frames", "2"])
    assert rc_headless == rc


def test_window_mode_exits_zero_once_any_frame_was_solved(monkeypatch, capsys):
    """只要有过一帧 READY，整场就算成功 —— 调取景途中掉几帧是常态。

    这条与上一条是一对：上一条钉住"整场零解算不能算通过"，这一条钉住
    "**不能**反过来把任何掉帧都当成失败"（否则每次挪一下相机都吃一个非 0 退出码）。
    """
    good = _fixture_image()
    bad = _cover(good.copy(), COVER_BOX[20])
    # 前 3 帧按 0（"没有按键"）让它继续跑，第 4 次 waitKey 才按 q 退出 ——
    # 于是这轮真的处理了 bad/bad/good 三帧，其中有一帧 READY。
    rc, _shown, _cap = _drive_window(monkeypatch, [bad, bad, good, good],
                                     keys=[0, 0, 0, ord("q")])
    capsys.readouterr()
    assert rc == live_field.EXIT_OK == 0

    # 全程解得出来也一样是 0
    rc, _shown, _cap = _drive_window(monkeypatch, [good, good], keys=[0, ord("q")])
    capsys.readouterr()
    assert rc == live_field.EXIT_OK


def _plain_doc(text: str) -> str:
    """去掉 markdown 强调与所有空白，让"判据**整句**"能逐字比对。

    文档里整句常被 `**` 打断、也被换行拆开（表格要缩进才读得下去），所以比对之前
    先归一 —— 但**只**归一空白与强调，不归一用词，否则"整场一帧都没解出来算失败"改成
    "也算成功"这种说反话照样能过。
    """
    return re.sub(r"\s+", "", text.replace("*", ""))


def test_the_exit_code_table_in_the_docs_matches_what_the_code_does():
    """三处文档（模块 docstring / `--help` / `run_window.__doc__`）必须**逐字**给出判据整句。

    上一版只 grep 了 5 个子串，于是三个变异全部逃逸：把"整场一帧都没解出来算失败"改成
    "也算成功"、`run_window` 的 docstring 整段判据删掉、`--help` 的交叉指引删掉 ——
    三次都是全绿。行为侧另有 `test_window_mode_exits_one_when_not_a_single_frame_was_ever_`
    `_solved` 兜着，所以那不是假通过的风险；但**这条用例守的正是本轮的核心命题**：
    文档不许说反话，而子串级的断言只守得住"提到过"，守不住"说得对"。
    """
    help_text = _plain_doc(live_field.build_parser().format_help())
    module_doc = _plain_doc(live_field.__doc__)
    window_doc = _plain_doc(live_field.run_window.__doc__)
    three_way = (("--help", help_text), ("模块 docstring", module_doc),
                 ("run_window docstring", window_doc))

    # 窗口模式的 0/1 分界：三处都必须整句出现（删掉任何一段整句都要红）。
    for name, text in three_way:
        for sentence in ("整场一帧都没解出来算失败", "至少有一帧READY"):
            assert sentence in text, f"{name} 里没有这条判据：{sentence}"

    # 无窗口模式那半张表：窗口模式的 docstring 也要能说清对照，否则"刻意不同"没人看得到。
    for name, text in three_way:
        for sentence in ("每一帧都READY", "整场零READY"):
            assert sentence in text, f"{name} 里没有无窗口模式的判据：{sentence}"

    # 中途掉线：两种模式方向相反（窗口 0 / 无窗口 1），三处都必须写明，否则
    # 下一个人会当笔误"统一"掉。判据本身由
    # test_the_camera_dropping_out_mid_session_keeps_the_frames_it_already_solved 钉住。
    for name, text in three_way:
        assert "中途" in text and "读帧失败" in text, f"{name} 没写中途掉线怎么算"

    # `--help` 必须指回模块 docstring（判据为什么两种模式不一样，写在哪里）。
    assert "两种模式判据不同" in help_text and "run_window" in help_text

    # 1 与 4 的分工：标定文件读不出来也是 1，那时相机还没开，所以"1 = 拍到了但不能用"
    # 这句旧文案是不成立的。
    assert "连相机都没打开" in help_text and "连相机都没打开" in module_doc
    assert "含配置错误" in module_doc


def test_the_camera_dropping_out_mid_session_keeps_the_frames_it_already_solved(monkeypatch, capsys):
    """窗口模式中途掉线：解出来的帧是有效的测量 → 0；`run_headless` 同一场景 → 1。

    这个差异是**刻意的**（理由见 `run_window` / `run_headless` 的 docstring）：无窗口
    模式是有界采样（`--frames N` 就是要 N 帧），读不到就是这一轮没跑完；窗口模式是
    长时间会话，跑了一半的测量仍然拿得走。之前它只写在"整场零 READY"那句话的尾巴上，
    读者要自己反推 —— 而 `run_headless` 那边第一句就是"中途读帧失败 → 1"。
    """
    good = _fixture_image()
    rc, shown, cap = _drive_window(monkeypatch, [good] * 4, keys=[-1, -1],
                                   drop_after=2)          # 两帧 READY 之后相机被拔
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_OK == 0, out
    assert "读帧失败" in out and len(shown) == 2
    assert cap.released == 1

    # 对照组：同样两帧 READY + 掉线，无窗口模式给 1（有界采样没跑完）。
    rc_headless, _cap = _run_headless(monkeypatch, [good] * 4, ["--frames", "3"], drop_after=2)
    assert rc_headless == live_field.EXIT_PHOTO_FAILED == 1

    # 反例钉子：真的一个问题都没解出来时，窗口模式**也**是 1（别让"掉线"把这条盖掉）。
    bad = _cover(good.copy(), COVER_BOX[20])
    rc_zero, _shown, _cap = _drive_window(monkeypatch, [bad] * 4, keys=[-1], drop_after=1)
    capsys.readouterr()
    assert rc_zero == live_field.EXIT_PHOTO_FAILED == 1


def test_ctrl_c_leaves_the_window_mode_with_one_line_and_the_usual_verdict(monkeypatch, capsys):
    """Ctrl-C：一行提示 + **按同一判据**返回，不打 traceback，也不把相机/窗口留在原地。

    之前 `KeyboardInterrupt` 直接穿出 `main()`：操作者在终端里随手 Ctrl-C 就会吃到一屏
    红字 —— 而这个工具的出发点就是"别给操作者一屏红字"。
    注意返回的是**判据**（有过 READY → 0 / 零 READY → 1），不是随手 `return EXIT_OK`：
    否则"整场零 READY 却返回 0"这条假通过换个入口又回来了。
    """
    img = _fixture_image()

    rc, shown, cap = _drive_window(monkeypatch, [img] * 4, keys=[-1, -1], interrupt_after=2,
                                   swallow_interrupt=True)
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_OK == 0, f"Ctrl-C 穿出了 main()（rc={rc}）：操作者会吃到一屏红字"
    assert "Ctrl-C" in out and "KeyboardInterrupt" not in out and "Traceback" not in out
    assert len(shown) == 3                         # 第 3 次 waitKey 才是 Ctrl-C
    assert cap.released == 1, "Ctrl-C 不许把 /dev/video0 占住"

    # 同一判据的另一半：整场零 READY 时 Ctrl-C 给 1。
    bad = _cover(img.copy(), COVER_BOX[20])
    rc_zero, _shown, _cap = _drive_window(monkeypatch, [bad] * 4, keys=[-1, -1],
                                          interrupt_after=2, swallow_interrupt=True)
    out = capsys.readouterr().out
    assert rc_zero == live_field.EXIT_PHOTO_FAILED == 1
    assert "Ctrl-C" in out and "Traceback" not in out


def test_a_calib_file_that_cannot_be_read_returns_one(monkeypatch, capsys, tmp_path):
    """标定文件损坏 → 一行提示 + 1（与 `verify_vision.py` 同一口径：同一类输入错误，同一返回码）。"""
    broken = tmp_path / "car_calib.json"
    broken.write_text("{ 这不是 JSON", encoding="utf-8")
    rc = live_field.main(["--once", "--calib", str(broken)])
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_PHOTO_FAILED == 1
    assert "标定文件读不出来" in out and "Traceback" not in out


def test_a_read_failure_returns_one_and_never_two(monkeypatch, capsys):
    """中途读帧失败是 1（没产出解），不是 2（用法错）—— 两种"失败"不许混。"""
    rc, _cap = _run_headless(monkeypatch, [_fixture_image()], ["--frames", "3"], drop_after=1)
    out = capsys.readouterr().out
    assert rc == live_field.EXIT_PHOTO_FAILED == 1
    assert "读帧失败" in out


def test_run_window_never_mutates_the_frame_it_solves_and_saves(monkeypatch, tmp_path, capsys):
    """`s` 连按两次 + 网格开关跑一轮：原帧必须始终干净，快照逐张可解。"""
    img = _fixture_image()
    calib = _default_calib()
    _rc, shown, _cap = _drive_window(
        monkeypatch, [img], keys=[ord("s"), ord("g"), ord("s"), ord("q")],
        argv_extra=["--snapshot-dir", str(tmp_path / "snaps")])
    capsys.readouterr()

    saved = sorted((tmp_path / "snaps").glob("*.jpg"))
    assert len(saved) == 2
    for path in saved:
        assert verify_vision.solve_image(cv2.imread(str(path)), calib, "snap", str(path)).ok
    assert len(shown) == 4


def test_window_mode_without_a_display_says_what_to_add_instead_of_a_traceback(monkeypatch, capsys):
    """没有显示环境 → 一行提示 + 非零退出码；而且**不该先去占相机**（那是共享设备）。"""
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)

    def boom(*_a, **_k):
        raise AssertionError("没有显示环境就不该去开相机")

    monkeypatch.setattr(live_field, "open_camera", boom)
    rc = live_field.main([])
    captured = capsys.readouterr()

    assert rc != 0 and rc == live_field.EXIT_USAGE
    assert "--once" in captured.out and "--frames" in captured.out
    assert "没有显示环境" in captured.out
    assert "Traceback" not in captured.out and "Traceback" not in captured.err


def test_headless_modes_do_not_need_a_display(monkeypatch, capsys):
    """对照组：环境变量检查不许误伤无窗口模式（Windows/macOS 也没有 DISPLAY）。"""
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    rc, _cap = _run_headless(monkeypatch, [_fixture_image()], ["--once"])
    assert rc == live_field.EXIT_OK
    assert "READY" in capsys.readouterr().out


@pytest.mark.parametrize("raiser_name", ["namedWindow", "resizeWindow", "moveWindow",
                                           "imshow", "waitKey"])
def test_every_window_call_that_raises_is_one_line_not_a_traceback(monkeypatch, capsys, raiser_name):
    """**五个**窗口调用逐个注入 `cv2.error`：都必须变成一行提示 + 退出码 2，不许 traceback。

    五个都参数化，是因为它们全都得走 `_window_call` —— 只要有一个漏掉（直接写
    `cv2.xxx(...)`），那条 `cv2.error` 就会穿出 `main()` 在远端 shell 里糊一屏红字，
    而只参数化 `namedWindow`/`imshow` 是**测不出**这种漏掉的。
    """
    monkeypatch.setenv("DISPLAY", ":0")           # 过了环境检查，逼出 cv2.error 那条路
    rc, _cap = _run_headless(
        monkeypatch, [_fixture_image()], [],
        raiser="The function is not implemented. Rebuild the library with GUI support",
        raiser_name=raiser_name)
    captured = capsys.readouterr()

    assert rc == live_field.EXIT_USAGE != 0
    assert "窗口" in captured.out and "--once" in captured.out
    assert "Traceback" not in captured.out and "Traceback" not in captured.err


def test_the_display_message_does_not_claim_no_display_when_a_display_is_set(monkeypatch):
    """`DISPLAY` 有值却开不了窗口时，不许说"当前没有显示环境" —— 那句话是假的。

    兜底层收窄到 `_window_call` 之后，这条路径在 `DISPLAY` **有值**时也会走到
    （Qt 连接断了、`imshow` 收到畸形图都会）。那时候给出的补救要是只有
    "加 `--once`"，恰好废掉这个工具存在的理由 —— 它就是给要看画面的人用的。
    """
    monkeypatch.setattr(live_field.sys, "platform", "linux")
    monkeypatch.setenv("DISPLAY", ":0")
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    text = live_field._display_problem("boom")
    assert text.splitlines()[0].startswith("连不上图形窗口")
    assert "当前没有显示环境" not in text.splitlines()[0]      # 首行不许说假话
    assert ":0" in text.splitlines()[0]                     # 带上具体是哪个 DISPLAY
    assert "X 转发" in text or "ssh -X" in text

    monkeypatch.delenv("DISPLAY", raising=False)
    cold = live_field._display_problem("")
    assert cold.splitlines()[0].startswith("当前没有显示环境")
    assert "连不上图形窗口" not in cold
    assert "export DISPLAY=:0" in cold or "ssh -X" in cold    # 上一轮遗留的提示一起做掉
    assert not cold.endswith("\n  ") and "\n\n" not in cold


def test_window_mode_on_macos_reports_the_failure_instead_of_crashing_in_the_except_block(monkeypatch, capsys):
    """darwin：`_has_display()` 恒为 True，而 `DISPLAY`/`WAYLAND_DISPLAY` 通常都没设。

    之前的 bug：`_display_problem` 无条件拼 `"xdpyinfo -display " + seen`，而这里
    `seen is None` → `TypeError: can only concatenate str (not "NoneType") to str`，
    **发生在 `except _DisplayUnavailable` 里面** —— 于是该给的提示与退出码 2 一起被
    顶掉，退出码变成 1、stderr 一屏 traceback。macOS 上这条路必然踩到（那里根本没有
    `DISPLAY`），而 `_has_display` 的 docstring 明说它支持 Windows/macOS：
    **声称了却没兑现**。

    讽刺的是当时唯一钉这条分支的用例把 `sys.platform` 显式钉成了 `"linux"`。
    """
    monkeypatch.setattr(live_field.sys, "platform", "darwin")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)

    def boom(*_a, **_k):
        raise cv2.error("The function is not implemented. Rebuild the library with GUI support")

    cap = FakeCap([_fixture_image()])
    monkeypatch.setattr(live_field, "open_camera", lambda *a, **k: cap)
    monkeypatch.setattr(live_field, "DEFAULT_SETTLE_FRAMES", 0)
    monkeypatch.setattr(live_field, "TERMINAL_MIN_INTERVAL_S", 0.0)
    monkeypatch.setattr(cv2, "namedWindow", boom)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda *a, **k: None)

    rc = live_field.main([])
    captured = capsys.readouterr()

    assert rc == live_field.EXIT_USAGE == 2
    assert "xdpyinfo" not in captured.out
    assert "显示环境变量有值" not in captured.out, "这台机器上一个显示环境变量都没有，不能这么说"
    assert "darwin" in captured.out and "--once" in captured.out
    assert "Traceback" not in captured.out and "Traceback" not in captured.err
    assert cap.released == 1


def test_the_xdpyinfo_self_check_is_offered_only_for_an_x_display(monkeypatch):
    """`xdpyinfo` 连的是 X display：纯 Wayland（`wayland-0`）时给出去必然失败。

    给一条必然失败的命令比不给更糟 —— 操作者会以为显示环境本身有问题，
    于是去折腾 X 转发，而真正的原因是 OpenCV 在纯 Wayland 会话里没有后端。
    """
    monkeypatch.setattr(live_field.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)

    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    wayland = live_field._display_problem("boom")
    assert "wayland-0" in wayland                 # 还是说清了它看到了哪个变量
    assert "xdpyinfo" not in wayland
    assert "Wayland" in wayland

    monkeypatch.setenv("DISPLAY", ":0")           # X11（`:0`）才有自查命令
    assert "xdpyinfo -display :0" in live_field._display_problem("boom")
    # X 转发那条提示只在真的连着 X display 时给（Wayland 那条路径里它是误导）
    assert "X 转发" not in wayland


@pytest.mark.parametrize("exc_factory", [lambda: cv2.error(""), lambda: OSError("")])
def test_an_exception_with_an_empty_message_does_not_raise_inside_except(exc_factory):
    """`cv2.error("")` / `OSError("")` 的 `str()` 是空串，`splitlines()[0]` 会在 **except 里**
    抛 `IndexError`，把真正的错误盖掉 —— 报错的地方正在报错。"""
    assert live_field._first_line(exc_factory()) == ""
    assert live_field._first_line(cv2.error("第一行\n第二行")) == "第一行"
    # 窗口调用那一层（`_window_call` 只转换 `cv2.error`）也要拿到空串而不是 IndexError
    with pytest.raises(live_field._DisplayUnavailable) as caught:
        live_field._window_call(lambda: (_ for _ in ()).throw(cv2.error("")))
    assert str(caught.value) == ""


def test_a_snapshot_failure_is_not_disguised_as_a_display_problem(monkeypatch, capsys):
    """窗口已经开起来之后的 `cv2.error` **不许**被贴上"没有显示环境"的标签。

    `cv2.error` 的兜底是为开窗口准备的；它的作用域一旦盖住 `detect_markers` /
    `solve_image` / `save_snapshot`，写盘失败就会被说成"没有显示环境"、给出
    "加 `--once`"这种帮不上忙的补救、并以 `EXIT_USAGE` 结束整个实时会话。
    真实触发路径：`cv2.imwrite` 对不认识的扩展名是**抛** `cv2.error` 而不是返回 False
    （本机实测 `imwrite("x.zzz")` 抛 `could not find a writer`）。

    这里注入 `save_snapshot` 抛 `cv2.error`：断言**不是**显示那条文案、**不是**退出码 2，
    而且会话还活着（预览继续跑，按一次 `s` 失败不该把实时窗口带走）。
    """
    img = _fixture_image()

    def boom(_frame, _out_dir):
        raise cv2.error("could not find a writer for the specified extension")

    monkeypatch.setattr(live_field, "save_snapshot", boom)
    rc, shown, _cap = _drive_window(monkeypatch, [img], keys=[ord("s"), ord("s"), ord("q")])
    out = capsys.readouterr().out

    assert rc == live_field.EXIT_OK != live_field.EXIT_USAGE
    assert "开不了图形窗口" not in out and "没有显示环境" not in out
    assert "快照存不了" in out and "could not find a writer" in out
    assert len(shown) == 3, "按 s 失败不该结束会话：后面的帧还要继续显示"


def test_a_cv2_error_outside_the_window_calls_is_not_reported_as_a_display_problem(monkeypatch, capsys):
    """兜底层的**类型**边界：循环里的 `cv2.error` 不许被当成"没有显示环境"。

    `_window_call` 只包住 `namedWindow`/`resizeWindow`/`moveWindow`/`imshow`/`waitKey`，
    所以 `draw_overlay` 这类非窗口环节抛的 `cv2.error` 原样穿出 —— 那是真 bug，
    贴一个"加 `--once`"的标签既误导、退出码也错。
    """

    def boom(*_a, **_k):
        raise cv2.error("boom inside the overlay stage")

    monkeypatch.setattr(live_field, "draw_overlay", boom)
    with pytest.raises(cv2.error):
        live_field.main([])
    out = capsys.readouterr().out
    assert "开不了图形窗口" not in out and "没有显示环境" not in out


def test_has_display_is_true_when_a_display_variable_is_set(monkeypatch):
    monkeypatch.setattr(live_field.sys, "platform", "linux")
    for name in ("DISPLAY", "WAYLAND_DISPLAY"):
        monkeypatch.delenv("DISPLAY", raising=False)
        monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
        monkeypatch.setenv(name, ":0")
        assert live_field._has_display() is True, name
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    assert live_field._has_display() is False
    # 非 Linux（Windows/macOS 的窗口系统不靠 DISPLAY）不许误报
    monkeypatch.setattr(live_field.sys, "platform", "darwin")
    assert live_field._has_display() is True


# --- 真相机（设备不可用时 skip，不让测试红） ----------------------------------------


def test_once_on_the_real_camera_when_one_is_available():
    """设备在就真跑一帧：格式必须是 1920x1080，且判定与叠加出自同一次解算。"""
    try:
        cap = live_field.open_camera(live_field.CAMERA_DEVICE)
    except live_field.CameraUnavailable as exc:
        pytest.skip(f"本机没有可用的相机：{exc}")
    try:
        live_field.settle(cap, frames=5)
        frame = live_field.read_frame(cap)
    finally:
        cap.release()
    if frame is None:
        pytest.skip("相机打开了但读不出帧")
    assert (frame.shape[1], frame.shape[0]) == (live_field.CAPTURE_WIDTH, live_field.CAPTURE_HEIGHT)

    calib = _default_calib()
    poses = detect_markers(frame, calib)
    result = live_field.solve_image(frame, calib, "live", live_field.CAMERA_DEVICE)
    line = live_field.summarize(result, poses)
    # 这一帧可能 READY 也可能 boundary_moved（取决于操作者把相机摆到哪了），
    # 但状态行必须与 solve_image 的判定一致，且不出现自造的 reason 词。
    if result.ok:
        assert line.startswith("READY")
    else:
        assert line.startswith(result.reason)
        assert result.reason in ("boundary_moved", "no_vehicle_marker")
