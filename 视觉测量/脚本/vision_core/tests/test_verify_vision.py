"""`verify_vision.py` 的行为锁：只读求解、逐张报告失败、A2 残差、写盘的深合并。

这些用例守的是**已经出过一次问题、或者差一步就会静默出错**的地方：

- **R40（Task 1 的真缺陷）**：`--bias-dy` 只改一项时，原有的 `dx_mm`/`dtheta_deg`
  必须原封不动。浅合并 `{**a, **b}` 会把整段 `bias` 换掉，而 `load_calib` 又拿内建
  默认值补齐 —— 量好的 `dy_mm=55` 读回来变成 `0.0`，全程无报错。
  `test_partial_bias_write_keeps_siblings` 是这条的回归锁，
  `test_a_shallow_merge_would_have_dropped_dy_mm` 是它的反例钉子。
- **R37**：偏置没量过的时候，输出必须**说**它没量过，而不是把内建默认的 0.0
  当测量值印出来。
- **R35**：`--help` 里必须带着"沿码规范轴 / 车体左 b → dx ±b、dtheta ±90"这条换算。
  操作者马上就要拿卷尺量车，这段 help 是唯一会出现在他眼前的东西。
- **R34**：表头必须说清 yaw 的零基准是场地 +X，并印出 +92.79° 这个常数。
- **R39**：采集必须丢缓冲帧、必须在读不到帧时**换一根新 handle** 重试。
  "把上一个位置的旧帧当成这次的测量"不会报错，只会算出一个看起来合理的错数。
- **Task 11 词表**：解不出来的照片逐张报告 `boundary_moved` / `no_vehicle_marker`，
  不 traceback、不中断整轮，但整轮以非 0 退出。

正例用实拍照片 `视觉测量/照片/photo_review_20261013.jpg`（5 个码全检出）。反例用
**同一张图盖住某个码**造出来 —— 盖完当场过一遍 `detect_markers` 确认那个码真的
没了，用例才继续（见 `_cover_marker` 里的断言），免得哪天标记检测行为变了，这个用例
还"因为别的原因"绿着。
"""
import importlib.util
import json
import math
import re
import sys

import cv2
import numpy as np
import pytest

from vision_core import calib_store
from vision_core.markers import detect_markers

PHOTO = "视觉测量/照片/photo_review_20261013.jpg"
TOOL = "视觉测量/脚本/verify_vision.py"

# 按文件路径加载工具（它不是包里的模块，是个脚本）。**必须先登记进 sys.modules**：
# `from __future__ import annotations` 让注解变成字符串，`@dataclass` 回头去
# `sys.modules[cls.__module__]` 找模块，模块不在那儿就直接 AttributeError。
_spec = importlib.util.spec_from_file_location("verify_vision", calib_store.resolve_path(TOOL))
verify_vision = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = verify_vision
_spec.loader.exec_module(verify_vision)

# 计划 Step 2 要的结果行形状：一行、一张图、字段之间两个空格。
RESULT_RE = re.compile(
    r"^(?P<label>\S+\.jpg)  x=(?P<x>[+-][\d.]+)mm  y=(?P<y>[+-][\d.]+)mm  "
    r"yaw=(?P<yaw>[+-][\d.]+)deg  reproj_rms=(?P<rms>[\d.]+)px$"
)
FAIL_RE = re.compile(
    r"^photo=(?P<label>\S+\.jpg)  ok=False  reason=(?P<reason>\w+)  (?P<detail>.+)$"
)

# 四个边界码 + 车载码在这张实拍照片上的角点框（`detect_markers` 解出的
# `corners_px` 外扩一圈）。盖整块，别只盖角点：角点残留会让码"半个还在"。
COVER_BOX = {
    20: (370, 0, 490, 105),
    21: (1780, 40, 1918, 180),
    22: (1745, 770, 1918, 890),
    23: (345, 695, 450, 790),
    29: (760, 245, 915, 405),
}


def _fixture_image():
    img = cv2.imread(str(calib_store.resolve_path(PHOTO)))
    assert img is not None, f"读不到测试照片 {PHOTO}"
    return img


def _cover_marker(tmp_path, marker_id: int, name: str) -> str:
    """造一张**指定码检不出来**的照片，返回 imread 能直接用的路径。

    从图里一块空场复制过去盖住整个码，保持纹理自然。盖完当场验一遍
    `detect_markers`：那个码真的消失、其余码还在 —— 否则这个反例就没有意义
    （失败原因会是文件坏了，而不是码掉了）。
    """
    img = _fixture_image()
    x0, y0, x1, y1 = COVER_BOX[marker_id]
    h, w = y1 - y0, x1 - x0
    src_x, src_y = 1000, 300
    img[y0:y1, x0:x1] = img[src_y:src_y + h, src_x:src_x + w]
    found = set(detect_markers(img, calib_store.load_calib()))
    assert marker_id not in found, f"盖住 ID{marker_id} 之后它还在：{sorted(found)}"
    path = tmp_path / name
    assert cv2.imwrite(str(path), img)
    return str(path)


def _lines(out: str) -> list[str]:
    return [ln for ln in out.splitlines() if ln.strip()]


def _results(out: str) -> list[str]:
    return [ln for ln in _lines(out) if RESULT_RE.match(ln)]


def _failures(out: str) -> list[re.Match]:
    return [m for m in (FAIL_RE.match(ln) for ln in _lines(out)) if m]


def _calib_path(tmp_path) -> str:
    return str(tmp_path / "car_calib.json")


def _exact_pose(tmp_path, photo: str = PHOTO) -> tuple[float, float, float]:
    """参考照片的精确解算结果。

    直接调 `solve_image` 而不是去解析打印行：打印只到 0.1，残差是 mm 量级的差，
    0.05 的取整误差会污染断言。打印行本身能不能被解析，由只读那两条用例验。
    """
    img, res = verify_vision.load_image(photo)
    assert img is not None and res.ok
    solved = verify_vision.solve_image(img, calib_store.load_calib(_calib_path(tmp_path)),
                                       res.label, res.origin)
    assert solved.ok, f"{photo} 解不出来：{solved.detail}"
    return (solved.x_mm, solved.y_mm, solved.yaw_deg)


# 仓库里现成的、**五码都能检出**的三张照片 = 三个不同现场。A2 判据按"不同位置"算，
# 所以凡是"3 个样本应该通过"的用例都必须给三张**真**不同的照片；拿同一张重复三遍
# 现在会被判据挡住（那正是 Important 3 要防的事）。
SCENES = [PHOTO, "视觉测量/照片/photo5_final.jpg", "视觉测量/照片/photoA_calib.jpg"]


def _scenes() -> list[str]:
    for photo in SCENES:
        assert calib_store.resolve_path(photo).is_file(), f"测试要用的现场照片没了：{photo}"
    return list(SCENES)


def _truth_values_for(photos, tmp_path, dx: float = 0.0, dy: float = 0.0,
                      dyaw: float = 0.0) -> list[str]:
    """给每张照片造一个"残差恰好已知"的真值串 `X,Y,YAW`（**不带** `--truth=` 前缀）。

    真值 = 该照片自己的解算结果 + 偏移，于是残差恰好是 `(dx, dy)` 与 `dyaw` ——
    RMS 可以精确断言，与"这张照片实际拍到哪"无关。
    """
    out = []
    for photo in photos:
        x, y, yaw = _exact_pose(tmp_path, photo)
        out.append(f"{x + dx:.6f},{y + dy:.6f},{yaw + dyaw:.6f}")
    return out


def _truths_for(photos, tmp_path, dx: float = 0.0, dy: float = 0.0, dyaw: float = 0.0) -> list[str]:
    """同 `_truth_values_for`，但带 `--truth=` 前缀，可以直接展开进 argv。"""
    return [f"--truth={v}" for v in _truth_values_for(photos, tmp_path, dx, dy, dyaw)]


# --- 只读模式 -------------------------------------------------------------------


def test_readonly_prints_one_solvable_line_for_the_fixture_photo(tmp_path, capsys):
    rc = verify_vision.main([PHOTO, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == 0

    lines = _results(out)
    assert len(lines) == 1, out
    m = RESULT_RE.match(lines[0])
    assert m.group("label") == "photo_review_20261013.jpg"
    assert float(m.group("rms")) < 1.0        # 实拍图实测 0.19 px


def test_readonly_mode_writes_nothing(tmp_path, capsys):
    """只读 = 一个字节都不落盘。没有这条，"先跑一遍看看"就会悄悄创建标定文件。"""
    verify_vision.main([PHOTO, "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert not (tmp_path / "car_calib.json").exists()


def test_header_states_the_yaw_zero_and_the_image_up_constant(tmp_path, capsys):
    """R34：表头说清零基准是场地 +X，并印出 +92.79° 这个常数（计划原文是"图像 up"）。"""
    verify_vision.main([PHOTO, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert "场地 +X" in out and "ID20 TL" in out
    assert str(verify_vision.IMAGE_UP_IN_FIELD_DEG) in out
    assert "不是 0°" in out


def test_header_says_the_bias_is_still_unmeasured(tmp_path, capsys):
    """R37：文件不存在 = A1 没做。不能把内建默认的 0.0 当测量值印出去。"""
    verify_vision.main([PHOTO, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert "尚未测量" in out
    assert "!!" in out                       # 醒目，而不是一行普通说明
    assert "车载码的中心" in out              # 说清楚现在解出来的 x/y 到底是什么


def test_header_says_the_bias_once_the_file_records_it(tmp_path, capsys):
    p = tmp_path / "car_calib.json"
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 0.0, "dtheta_deg": 90.0}},
                           path=str(p))
    verify_vision.main([PHOTO, "--calib", str(p)])
    out = capsys.readouterr().out
    assert "A1 偏置已有记录" in out
    assert "尚未测量" not in out


# --- 解不出来的照片（Task 11 的 ok=False 词表）--------------------------------------


def test_missing_boundary_marker_is_reported_per_photo_and_sets_exit_code(tmp_path, capsys):
    """少一个边界码 = 这一帧的场地系是错的且不报错：必须逐张点名，并以非 0 退出。"""
    broken = _cover_marker(tmp_path, 21, "no21.jpg")
    rc = verify_vision.main([PHOTO, broken, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out

    assert rc == verify_vision.EXIT_PHOTO_FAILED
    fails = _failures(out)
    assert len(fails) == 1
    assert fails[0].group("label") == "no21.jpg"
    assert fails[0].group("reason") == "boundary_moved"
    assert "21" in fails[0].group("detail")

    # 好照片那一行照样打出来（不中断整轮），并说明这一轮不能当测量结果
    assert len(_results(out)) == 1
    assert "失败 1/2 张" in out


def test_missing_vehicle_marker_is_reported_as_no_vehicle_marker(tmp_path, capsys):
    """四个边界码都在、车载码不在 —— 与缺边界码是**两回事**，不能共用一个原因词。"""
    broken = _cover_marker(tmp_path, 29, "no29.jpg")
    rc = verify_vision.main([broken, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    m = _failures(out)[0]
    assert m.group("reason") == "no_vehicle_marker"
    assert "29" in m.group("detail")


def test_missing_photo_file_is_reported_without_a_traceback(tmp_path, capsys):
    rc = verify_vision.main(["视觉测量/照片/__没有这张图__.jpg", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    assert "reason=image_missing" in out
    assert "Traceback" not in out


def test_one_bad_photo_does_not_stop_the_good_ones(tmp_path, capsys):
    """顺序：坏的在中间，前后两张照样解出来。"""
    broken = _cover_marker(tmp_path, 22, "no22.jpg")
    rc = verify_vision.main([PHOTO, broken, PHOTO, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    assert len(_results(out)) == 2


# --- A2：残差、样本数、判据 -------------------------------------------------------


def test_truth_computes_rms_and_n_samples_and_passes_the_gate(tmp_path, capsys):
    """真值按位置一一对应；RMS / n_samples 都算出来；判据明写出来，别让人背。"""
    photos = _scenes()
    # 三个位置的真值各自偏一个已知的量：位移 (+1,−2) mm，航向 +0.5°
    truths = _truths_for(photos, tmp_path, dx=1.0, dy=-2.0, dyaw=0.5)

    rc = verify_vision.main([*photos, *truths,
                             "--write-validation", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out

    assert rc == verify_vision.EXIT_OK
    assert "n_samples=3" in out and "distinct_photos=3" in out
    assert "residual_rms=2.24mm" in out               # hypot(1, 2) = 2.2360…
    assert "residual_yaw_rms=0.50deg" in out
    assert "A2 gate: PASS" in out
    assert "RMS ≤ 3.0 mm 且 ≤ 1.5° 且 ≥ 3 个不同位置" in out

    v = json.loads((tmp_path / "car_calib.json").read_text(encoding="utf-8"))["validation"]
    assert v["n_samples"] == 3 and v["n_distinct_photos"] == 3
    assert v["residual_rms_mm"] == pytest.approx(2.2360679, abs=1e-3)
    assert v["residual_rms_deg"] == pytest.approx(0.5, abs=1e-3)
    assert v["gate_passed"] is True


def test_residual_is_solved_minus_truth(tmp_path, capsys):
    """符号钉死：残差 = 解出来 − 真值。真给反了，A2 的结论会整个反过来。"""
    photos = _scenes()
    truths = _truths_for(photos, tmp_path, dx=-1.0, dy=-2.0, dyaw=-0.5)
    rc = verify_vision.main([*photos, *truths, "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_OK
    assert "err_xy=(+1.0,+2.0)mm" in out
    assert "dyaw=+0.5deg" in out


def test_two_samples_are_not_evidence_and_the_gate_says_so(tmp_path, capsys):
    """2 个位置的残差 RMS 不算通过：退出码非 0，且说清是位置不够。"""
    photos = _scenes()[:2]
    rc = verify_vision.main([*photos, *_truths_for(photos, tmp_path),
                             "--write-validation", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_GATE
    assert "INSUFFICIENT" in out and "只有 2 个不同位置" in out
    v = json.loads((tmp_path / "car_calib.json").read_text(encoding="utf-8"))["validation"]
    assert v["n_samples"] == 2 and v["n_distinct_photos"] == 2 and v["gate_passed"] is False


def test_rms_over_the_limit_fails_the_gate(tmp_path, capsys):
    photos = _scenes()
    rc = verify_vision.main([*photos, *_truths_for(photos, tmp_path, dx=10.0),
                             "--write-validation", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_GATE
    assert "A2 gate: FAIL" in out
    assert "先查 A1 偏置" in out          # 失败时先怀疑 dtheta 的 ±90 符号


def test_partial_truth_match_is_rejected(tmp_path, capsys):
    """给一半真值是错误，不是"跳过没有真值的那张"：那会让样本数悄悄少一个。"""
    with pytest.raises(SystemExit) as exc:
        verify_vision.main([PHOTO, PHOTO, "--truth=0,0,0", "--calib", _calib_path(tmp_path)])
    assert exc.value.code == verify_vision.EXIT_USAGE
    assert "一一对应" in capsys.readouterr().err
    assert not (tmp_path / "car_calib.json").exists()


def test_write_validation_without_truth_is_rejected(tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        verify_vision.main([PHOTO, "--write-validation", "--calib", _calib_path(tmp_path)])
    assert exc.value.code == verify_vision.EXIT_USAGE


def test_malformed_truth_is_a_usage_error(tmp_path, capsys):
    for bad in ("0,0", "0,0,0,0", "0,零,0"):
        with pytest.raises(SystemExit) as exc:
            verify_vision.main([PHOTO, f"--truth={bad}", "--calib", _calib_path(tmp_path)])
        assert exc.value.code == verify_vision.EXIT_USAGE


def test_negative_truth_works_with_and_without_the_equals_sign(tmp_path, capsys):
    """`--truth -80,0,0` 会被 argparse 当成位置参数（负数正则不匹配带逗号的串）。

    工具内部把空格写法拼成 `--truth=`，两种写法都得能用 —— 操作者第一次用就写错，
    等于这个功能不存在。
    """
    assert verify_vision.normalise_truth_argv(["--truth", "-80,0,0"]) == ["--truth=-80,0,0"]
    assert verify_vision.parse_truth("-80,10,-190") == (-80.0, 10.0, 170.0)   # 归一到 (−180,180]

    photos = _scenes()
    values = _truth_values_for(photos, tmp_path)
    rc = verify_vision.main([*photos,
                             "--truth", values[0],
                             f"--truth={values[1]}", f"--truth={values[2]}",
                             "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_OK
    assert len([ln for ln in _lines(out) if ln.startswith("truth  ")]) == 3


def test_validation_is_not_written_when_a_photo_fails(tmp_path, capsys):
    """残差样本不完整时，写一个"看起来通过"的 RMS 比不写更糟。"""
    x0, y0, yaw0 = _exact_pose(tmp_path)
    broken = _cover_marker(tmp_path, 29, "no29.jpg")
    rc = verify_vision.main([PHOTO, broken,
                             f"--truth={x0:.4f},{y0:.4f},{yaw0:.4f}",
                             "--truth=0,0,0",
                             "--write-validation", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    assert "残差样本不完整" in out
    assert not (tmp_path / "car_calib.json").exists()


# --- 写标定（R40 / R41）----------------------------------------------------------


def test_partial_bias_write_keeps_siblings(tmp_path, capsys):
    """R40 的回归锁：只给 --bias-dy，dx_mm/dtheta_deg 必须原封不动。"""
    p = tmp_path / "car_calib.json"
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 90.0}},
                           path=str(p))

    rc = verify_vision.main([PHOTO, "--bias-dy", "77.0", "--calib", str(p)])
    capsys.readouterr()
    assert rc == 0

    bias = json.loads(p.read_text(encoding="utf-8"))["bias"]
    assert bias == {"dx_mm": -12.0, "dy_mm": 77.0, "dtheta_deg": 90.0}
    assert calib_store.load_calib(str(p))["bias"]["dy_mm"] == 77.0


def test_a_shallow_merge_would_have_dropped_dy_mm():
    """反例钉子：确认"顶层 ** 确实会丢字段"，所以写盘点子必须是 deep_merge。"""
    before = {"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 90.0}}
    assert "dx_mm" not in {**before, "bias": {"dy_mm": 77.0}}["bias"]
    assert calib_store.deep_merge(before, {"bias": {"dy_mm": 77.0}})["bias"]["dx_mm"] == -12.0


def test_partial_write_keeps_other_sections(tmp_path, capsys):
    """同一件事的第二种形状：写 battery 也不能把 bias 段换成默认的 0。"""
    p = tmp_path / "car_calib.json"
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 90.0},
                            "coplanar": False}, path=str(p))
    verify_vision.main([PHOTO, "--battery-sag-v", "0.4", "--calib", str(p)])
    capsys.readouterr()
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data["bias"]["dy_mm"] == 55.0
    assert data["coplanar"] is False
    assert data["battery"]["sag_v"] == 0.4


def test_sag_v_alone_never_invents_a_verdict(tmp_path, capsys):
    """R41：verdict 是人的判断，工具不从 sag_v 推，而且要提醒它还空着。"""
    p = tmp_path / "car_calib.json"
    verify_vision.main([PHOTO, "--battery-sag-v", "0.4", "--calib", str(p)])
    out = capsys.readouterr().out
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data["battery"] == {"sag_v": 0.4, "verdict": None}
    assert "--battery-verdict" in out


def test_operator_supplied_verdict_is_written_as_given(tmp_path, capsys):
    p = tmp_path / "car_calib.json"
    verify_vision.main([PHOTO, "--battery-sag-v", "0.4",
                        "--battery-verdict", "接插件压降，接触正常", "--calib", str(p)])
    capsys.readouterr()
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data["battery"]["verdict"] == "接插件压降，接触正常"


def test_coplanar_records_the_conclusion_only(tmp_path, capsys):
    """R41：--coplanar 是结论的记录，不是复核，输出必须说明白。"""
    p = tmp_path / "car_calib.json"
    verify_vision.main([PHOTO, "--coplanar", "no", "--calib", str(p)])
    out = capsys.readouterr().out
    assert json.loads(p.read_text(encoding="utf-8"))["coplanar"] is False
    assert "不做复核" in out


# --- 采集（R39）-----------------------------------------------------------------


def test_capture_is_repeatable_and_records_each_saved_photo(tmp_path, capsys, monkeypatch):
    """`--capture --capture` 拍两张：两张都存盘、都被解算，路径逐张记在输出里。"""
    frame = _fixture_image()
    calls = []

    def fake_grab(device, settle_frames):
        calls.append((device, settle_frames))
        return frame.copy()

    monkeypatch.setattr(verify_vision, "grab_frame", fake_grab)
    caps_dir = tmp_path / "caps"
    rc = verify_vision.main([
        "--capture", "--capture",
        "--capture-dir", str(caps_dir),
        "--capture-prefix", "a2",
        "--settle-frames", "3",
        "--calib", _calib_path(tmp_path),
    ])
    out = capsys.readouterr().out

    assert rc == 0
    assert len(calls) == 2
    assert [c[1] for c in calls] == [3, 3]
    saved = sorted(p.name for p in caps_dir.glob("*.jpg"))
    assert len(saved) == 2
    assert saved[0].startswith("a2_01_") and saved[1].startswith("a2_02_")
    assert len(_results(out)) == 2
    for name in saved:
        assert f"saved=" in out and name in out


def test_capture_cannot_be_mixed_with_photo_paths(tmp_path, capsys):
    """混用时"第 N 个 --truth 配哪张"就没答案了，必须报错而不是猜。"""
    with pytest.raises(SystemExit) as exc:
        verify_vision.main([PHOTO, "--capture", "--calib", _calib_path(tmp_path)])
    assert exc.value.code == verify_vision.EXIT_USAGE
    assert "不能混用" in capsys.readouterr().err


def test_capture_count_is_equivalent_to_repeating_the_flag(tmp_path, capsys, monkeypatch):
    frame = _fixture_image()
    calls = []

    def fake_grab(device, settle_frames):
        calls.append(device)
        return frame.copy()

    monkeypatch.setattr(verify_vision, "grab_frame", fake_grab)
    caps_dir = tmp_path / "caps"
    rc = verify_vision.main(["--capture-count", "3", "--capture-dir", str(tmp_path / "c"),
                             "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert rc == 0
    assert len(calls) == 3


def test_grab_frame_flushes_the_buffer_and_retries_on_a_fresh_handle(monkeypatch):
    """R39：先丢缓冲帧再读；读不到帧必须**换一根新的 capture handle**。"""
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    log = {"opened": [], "grabs": [], "reads": 0, "released": 0}

    class FakeCap:
        def __init__(self, device, api):
            self.index = len(log["opened"])
            log["opened"].append(device)
            log["grabs"].append(0)

        def isOpened(self):
            return True

        def set(self, *_args):
            return True

        def grab(self):
            log["grabs"][self.index] += 1

        def read(self):
            log["reads"] += 1
            return (False, None) if self.index == 0 else (True, frame)

        def release(self):
            log["released"] += 1

    monkeypatch.setattr(verify_vision.cv2, "VideoCapture", FakeCap)
    monkeypatch.setattr(verify_vision, "CAPTURE_RETRY_DELAY_S", 0.0)

    assert verify_vision.grab_frame("/dev/video9", settle_frames=7, retries=3) is frame
    assert log["opened"] == ["/dev/video9", "/dev/video9"]      # 新的 handle，不是同一个
    assert log["grabs"] == [7, 7]                                # 每次都丢满缓冲帧
    assert log["reads"] == 2
    assert log["released"] == 2                                  # 两根都放掉了


def test_grab_frame_gives_up_with_a_readable_message(monkeypatch):
    log = {"opened": 0}

    class DeadCap:
        def __init__(self, *_args):
            log["opened"] += 1

        def isOpened(self):
            return False

        def release(self):
            pass

    monkeypatch.setattr(verify_vision.cv2, "VideoCapture", DeadCap)
    monkeypatch.setattr(verify_vision, "CAPTURE_RETRY_DELAY_S", 0.0)
    with pytest.raises(verify_vision.CaptureError) as exc:
        verify_vision.grab_frame("/dev/video9", retries=3)
    assert log["opened"] == 3
    assert "占用" in str(exc.value)


def test_a_failed_capture_becomes_one_failed_line_not_a_traceback(tmp_path, capsys, monkeypatch):
    def boom(device, settle_frames):
        raise verify_vision.CaptureError("/dev/video0 打不开（被别的程序占着？）")

    monkeypatch.setattr(verify_vision, "grab_frame", boom)
    rc = verify_vision.main(["--capture", "--capture-dir", str(tmp_path / "caps"),
                             "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    assert "reason=capture_failed" in out
    assert "Traceback" not in out


def test_capture_never_writes_into_the_repo(tmp_path, capsys, monkeypatch):
    """测试套件不许在仓库里 mkdir。`capture_shots` 会 mkdir --capture-dir，
    漏传这个标志就会在干净克隆上留下一个未跟踪目录。"""
    frame = _fixture_image()
    monkeypatch.setattr(verify_vision, "grab_frame", lambda d, s: frame.copy())
    caps_dir = tmp_path / "caps"
    repo_caps = calib_store.resolve_path(verify_vision.DEFAULT_CAPTURE_DIR)
    existed_before = repo_caps.exists()          # 操作者真用过 --capture 就会已存在
    rc = verify_vision.main(["--capture", "--capture-dir", str(caps_dir),
                             "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert rc == 0
    assert caps_dir.is_dir() and list(caps_dir.glob("*.jpg"))
    assert repo_caps.exists() == existed_before


def test_capture_count_adds_to_repeated_capture_flags(tmp_path, capsys, monkeypatch):
    """`--capture --capture --capture-count 2` 拍 4 张，不是 2 张（覆盖 vs 叠加）。"""
    frame = _fixture_image()
    calls = []

    def fake_grab(device, settle_frames):
        calls.append(device)
        return frame.copy()

    monkeypatch.setattr(verify_vision, "grab_frame", fake_grab)
    rc = verify_vision.main(["--capture", "--capture", "--capture-count", "2",
                             "--capture-dir", str(tmp_path / "c"),
                             "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert rc == 0
    assert len(calls) == 4


def test_unreadable_image_is_reported_as_image_unreadable(tmp_path, capsys):
    """`image_unreadable` 是词表里唯一一个还没有用例的原因词。"""
    broken = tmp_path / "not_an_image.jpg"
    broken.write_text("这不是 JPEG，只是一段文本。", encoding="utf-8")
    rc = verify_vision.main([str(broken), "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    assert "reason=image_unreadable" in out
    assert "Traceback" not in out


# --- help 文本（R35：操作者马上就要去量车了）--------------------------------------


def test_help_carries_the_marker_frame_bias_conversion():
    """R35：车体「左 b」→ dx ±b、dy 0、dtheta ±90。help 是操作者眼前唯一的东西。"""
    stripped = [ln.strip() for ln in verify_vision.build_parser().format_help().splitlines()]
    assert any("码规范 +X" in ln for ln in stripped)
    assert any("车体「左」 b → --bias-dx ±b, --bias-dy 0, --bias-dtheta ±90" in ln
               for ln in stripped)
    assert any("车体「前」 f → --bias-dx 0,  --bias-dy ±f, --bias-dtheta ±90" in ln
               for ln in stripped)


def test_help_states_the_a2_gate_and_the_truth_ordering():
    text = verify_vision.build_parser().format_help()
    assert "X,Y,YAW_DEG" in text
    assert "第 1 个 --truth ↔ 第 1 张照片" in text
    assert "RMS ≤ 3.0 mm 且 ≤ 1.5° 且 ≥ 3 个不同位置" in text
    assert "n_samples" in text


def test_help_keeps_the_bias_conversion_on_its_own_lines():
    """argparse 默认会把参数 help 重新折行，折掉的换算表就没法照着填了。"""
    stripped = [ln.strip() for ln in verify_vision.build_parser().format_help().splitlines()]
    assert "dx = FWD·cos(dtheta) + LEFT·sin(dtheta)" in stripped
    assert "dy = −FWD·sin(dtheta) + LEFT·cos(dtheta)" in stripped


def test_help_mentions_the_battery_verdict_is_human():
    """R41：verdict 只能人给，help 里不能说成工具能算出来。"""
    text = verify_vision.build_parser().format_help()
    assert "battery.verdict 只能由人用 --battery-verdict 给" in text
    assert "工具不猜" in text


def test_help_offers_the_body_frame_form_for_a1():
    """R35 的"±"只有在操作者知道码的轴朝哪时才可用 —— `--bias-body` 就是那个东西。"""
    text = verify_vision.build_parser().format_help()
    assert "--bias-body FWD,LEFT" in text
    assert "--bias-dtheta" in text
    assert "车头方向" in text and "车左方向" in text


# --- 一次调用里写两处（Critical：第二处把第一处抹掉）-----------------------------
#
# 这组是 review 的 Critical 回归锁：一次调用里既给人工标志又给 --write-validation，
# 两处必须**都**落盘。`save_calib` 是覆盖写，所以任何"第二次拿旧内存再写一遍"的写法
# 都会静默把第一次写的内容抹掉，且两次都打印成功回执。


def test_manual_writes_and_validation_in_one_run_both_land(tmp_path, capsys):
    """R40 的同一条纪律，两个方向都要挡：`--write-validation` 不能抹掉人工写的那几段。"""
    p = tmp_path / "car_calib.json"
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 90.0},
                            "coplanar": False,
                            "battery": {"sag_v": 0.4, "verdict": None}}, path=str(p))
    photos = _scenes()

    rc = verify_vision.main([*photos,
                             "--coplanar", "yes",
                             "--battery-verdict", "接插件压降，接触正常",
                             *_truths_for(photos, tmp_path),
                             "--write-validation", "--calib", str(p)])
    capsys.readouterr()
    assert rc == verify_vision.EXIT_OK

    data = json.loads(p.read_text(encoding="utf-8"))
    assert data["coplanar"] is True                                    # 人工写的
    assert data["battery"]["verdict"] == "接插件压降，接触正常"          # 人工写的
    assert data["battery"]["sag_v"] == 0.4                             # 原有值没丢
    assert data["bias"] == {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 90.0}
    assert data["validation"]["n_samples"] == 3                        # 本轮写的
    assert data["validation"]["gate_passed"] is True


def test_bias_and_validation_in_one_run_both_land(tmp_path, capsys):
    """review 的另一个复现：`--bias-dy 77` + `--write-validation`，问 77 得 77。"""
    p = tmp_path / "car_calib.json"
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 90.0}},
                           path=str(p))
    photos = _scenes()
    verify_vision.main([*photos,
                        "--bias-dy", "77.0",
                        *_truths_for(photos, tmp_path),
                        "--write-validation", "--calib", str(p)])
    capsys.readouterr()

    bias = json.loads(p.read_text(encoding="utf-8"))["bias"]
    assert bias["dy_mm"] == 77.0
    assert bias["dx_mm"] == -12.0 and bias["dtheta_deg"] == 90.0
    assert json.loads(p.read_text(encoding="utf-8"))["validation"]["n_samples"] == 3


def test_the_run_writes_the_file_exactly_once(tmp_path, capsys, monkeypatch):
    """结构性钉子：不管给多少写标志，一次调用只许 `save_calib` 一次。

    两次写 = 第二次要么覆盖第一次，要么得从磁盘重读 —— 后者是能修的写法，但两次写
    本身就是"能静默丢数据"的形状，不该留着。
    """
    calls = []
    real_save = calib_store.save_calib

    def counting_save(data, path=None):
        calls.append(json.loads(json.dumps(data)))
        real_save(data, path)

    monkeypatch.setattr(verify_vision.calib_store, "save_calib", counting_save)
    photos = _scenes()
    verify_vision.main([*photos,
                        "--bias-dy", "77.0", "--coplanar", "yes", "--battery-sag-v", "0.4",
                        *_truths_for(photos, tmp_path),
                        "--write-validation", "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert len(calls) == 1
    written = calls[0]
    assert written["coplanar"] is True and written["battery"]["sag_v"] == 0.4
    assert written["bias"]["dy_mm"] == 77.0 and written["validation"]["n_samples"] == 3


# --- Important 2：结果行不能是旧 bias 算出来的 ------------------------------------


def test_result_line_is_solved_with_the_bias_being_written(tmp_path, capsys):
    """同一次调用里给了 --bias-dy，打印的 x/y/yaw 必须已经是新 bias 的结果。

    旧代码用**写盘前**的 calib 求解，于是屏幕上那个数比文件里最终的值差几百毫米，
    而操作者正是照着屏幕上那个数去判断"填对没有"。
    """
    rc = verify_vision.main([PHOTO, "--bias-dy", "-55.0", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == 0
    printed = _results(out)[0]
    stored = json.loads((tmp_path / "car_calib.json").read_text(encoding="utf-8"))

    # 用**落盘后的**标定重解一次，结果行必须逐字相同
    x0, y0, yaw0 = _exact_pose(tmp_path)       # 此刻文件里已是 dy=-55
    verify_vision.main([PHOTO, "--calib", _calib_path(tmp_path)])
    assert _results(capsys.readouterr().out)[0] == printed
    assert RESULT_RE.match(printed).group("y") == f"{y0:+.1f}"
    assert stored["bias"]["dy_mm"] == -55.0


def test_stale_bias_line_is_marked_as_already_updated(tmp_path, capsys):
    """写 bias 时表头要说明"下面几行已经用上了新值"，不是让人以为还是旧值。"""
    verify_vision.main([PHOTO, "--bias-dy", "-55.0", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert "即将写入" in out and "已经是写入后的值" in out


# --- Important 3：同一张图重复 N 次不算 N 个位置 -----------------------------------


def test_repeating_one_photo_three_times_does_not_pass_the_gate(tmp_path, capsys):
    """一张图 + 三个 `--truth`：位置只有一个，判据不能算它过。"""
    x0, y0, yaw0 = _exact_pose(tmp_path)
    rc = verify_vision.main([PHOTO, PHOTO, PHOTO,
                             f"--truth={x0:.4f},{y0:.4f},{yaw0:.4f}",
                             f"--truth={x0:.4f},{y0:.4f},{yaw0:.4f}",
                             f"--truth={x0:.4f},{y0:.4f},{yaw0:.4f}",
                             "--write-validation", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_GATE
    assert "INSUFFICIENT" in out and "1 个不同位置" in out
    v = json.loads((tmp_path / "car_calib.json").read_text(encoding="utf-8"))["validation"]
    assert v["n_samples"] == 3 and v["n_distinct_photos"] == 1
    assert v["gate_passed"] is False


def test_three_distinct_photos_still_pass(tmp_path, capsys):
    """对照组：换了三张**不同**的照片就该照常通过（别把去重做成"永远不通过"）。"""
    photos = _scenes()
    rc = verify_vision.main([*photos, *_truths_for(photos, tmp_path), "--write-validation",
                             "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_OK
    assert "distinct_photos=3" in out and "A2 gate: PASS" in out
    assert "只有 1 个不同位置" not in out


def test_help_says_the_gate_counts_distinct_positions():
    text = verify_vision.build_parser().format_help()
    assert "≥ 3 个不同位置" in text
    assert "同一张照片重复" in text


# --- Important 4：--bias-body 与 --explain-bias ------------------------------------


def test_bias_body_round_trips_with_bias_dx_dy(tmp_path, capsys):
    """R35 的锁：同一个物理偏置，走车体系入口和走码规范轴入口，解出来的结果必须逐字相同。

    转换错了符号就只会让 ±90 变成 ∓90 —— x/y 各差一个偏置的两倍、yaw 差 180°，
    而 RMS 判据要到 A2 才抓得到。这里当场就抓。
    """
    for dtheta in (-90.0, 90.0, 37.5):
        for fwd, left in ((0.0, 120.0), (30.0, 0.0), (25.0, -80.0)):
            dx = fwd * math.cos(math.radians(dtheta)) + left * math.sin(math.radians(dtheta))
            dy = -fwd * math.sin(math.radians(dtheta)) + left * math.cos(math.radians(dtheta))

            verify_vision.main([PHOTO, "--bias-dx", f"{dx:.9f}", "--bias-dy", f"{dy:.9f}",
                                "--bias-dtheta", f"{dtheta:.9f}",
                                "--calib", _calib_path(tmp_path)])
            via_marker = _results(capsys.readouterr().out)[0]

            verify_vision.main([PHOTO, "--bias-body", f"{fwd:.9f},{left:.9f}",
                                "--bias-dtheta", f"{dtheta:.9f}",
                                "--calib", _calib_path(tmp_path)])
            via_body = _results(capsys.readouterr().out)[0]
            assert via_body == via_marker, (dtheta, fwd, left, via_body, via_marker)


def test_bias_body_left_offset_lands_on_dx_at_plus_90(tmp_path, capsys):
    """R35 docstring 说的那条换算，在这里由代码直接兑现：dtheta=+90 时车左 b → dx=b。"""
    dx, dy = verify_vision.body_to_marker(0.0, 120.0, 90.0)
    assert dx == pytest.approx(120.0, abs=1e-9)
    assert dy == pytest.approx(0.0, abs=1e-9)


def test_bias_body_and_bias_dx_are_mutually_exclusive(tmp_path, capsys):
    """两个入口给同一个量的两套数：混起来时以谁为准没有答案，必须报错。"""
    with pytest.raises(SystemExit) as exc:
        verify_vision.main([PHOTO, "--bias-body", "30,120", "--bias-dx", "5",
                            "--calib", _calib_path(tmp_path)])
    assert exc.value.code == verify_vision.EXIT_USAGE
    assert "不能同时" in capsys.readouterr().err


def test_bias_body_and_dtheta_only_is_enough(tmp_path, capsys):
    """只给 --bias-body（两项一起）就够了，dtheta=+90 时车左 120 → dx=120, dy=−30。"""
    rc = verify_vision.main([PHOTO, "--bias-body", "30,120", "--bias-dtheta", "90",
                             "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert rc == 0
    bias = json.loads((tmp_path / "car_calib.json").read_text(encoding="utf-8"))["bias"]
    assert bias["dx_mm"] == pytest.approx(120.0, abs=1e-9)
    assert bias["dy_mm"] == pytest.approx(-30.0, abs=1e-9)
    assert bias["dtheta_deg"] == 90.0


def test_explain_bias_prints_the_marker_axes_in_the_image(tmp_path, capsys):
    """`--explain-bias` 必须给出码的规范 +X/+Y 在**图像**里的指向，并给出对应 dtheta 表。

    没有这张表，help 里的"±90"就只能猜。
    """
    rc = verify_vision.main([PHOTO, "--explain-bias", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "explain-bias" in out
    assert "码规范 +X 在图像里指向" in out and "码规范 +Y 在图像里指向" in out
    assert "车头朝图像上" in out and "--bias-dtheta" in out
    # 参考照片实测（field.py 同一组数）：ID29 规范 +Y → 图像左，+X → 图像上
    assert "图像左" in out and "图像上" in out
    assert "码的航向（规范 +Y" in out


def test_explain_bias_needs_a_photo(tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        verify_vision.main(["--explain-bias", "--calib", _calib_path(tmp_path)])
    assert exc.value.code == verify_vision.EXIT_USAGE


def test_explain_bias_writes_nothing(tmp_path, capsys):
    verify_vision.main([PHOTO, "--explain-bias", "--calib", _calib_path(tmp_path)])
    capsys.readouterr()
    assert not (tmp_path / "car_calib.json").exists()


# --- Minor：要说真话 --------------------------------------------------------------


def test_refused_validation_does_not_claim_read_only_mode(tmp_path, capsys):
    """要了 --write-validation 却没写时，不能打"只读模式：没有写任何文件"。"""
    _exact_pose(tmp_path)
    broken = _cover_marker(tmp_path, 23, "no23.jpg")
    rc = verify_vision.main([PHOTO, broken,
                             "--truth=0,0,0", "--truth=0,0,0",
                             "--write-validation", "--calib", _calib_path(tmp_path)])
    out = capsys.readouterr().out
    assert rc == verify_vision.EXIT_PHOTO_FAILED
    assert "只读模式" not in out
    assert "要写 validation 但没写成" in out


def test_exit_usage_constant_matches_argparse():
    """`EXIT_USAGE` 与 argparse 的 UsageError 同值，否则退出码契约就裂了。"""
    assert verify_vision.EXIT_USAGE == 2
