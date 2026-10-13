"""calib_store 的行为锁：默认值完整、路径与 cwd 无关、常量名、深合并不可退化。

这里每条测试都对应一个已经出过一次问题的规格：
R3 相对路径必须按仓库根解析、7 个键一个都不能少、常量名/字面量不能漂、
写盘必须 indent=2 + 不转义非 ASCII、改个别字段必须深合并。
"""
import json
import warnings
from pathlib import Path

import pytest

from vision_core import calib_store

REQUIRED_KEYS = {
    "K",
    "dist",
    "boundary_side_mm",
    "vehicle_side_mm",
    "bias",
    "battery",
    "coplanar",
}


def test_load_missing_returns_defaults():
    c = calib_store.load_calib(path="/nonexistent/x.json")
    assert c["K"][0][0] == 1460.76
    assert c["dist"] == [0.0] * 5
    assert c["boundary_side_mm"] == 25.0
    assert c["vehicle_side_mm"] == 30.0


def test_save_then_load_roundtrip(tmp_path):
    p = str(tmp_path / "c.json")
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0}}, path=p)
    assert calib_store.load_calib(path=p)["bias"]["dy_mm"] == 55.0


# --- 7 个必需键 ---------------------------------------------------------------


def test_defaults_have_exactly_the_required_keys():
    """少一个键（例如 bias/battery/coplanar 被误删）下游立刻 KeyError —— 这里钉死"""
    assert set(calib_store.default_calib()) == REQUIRED_KEYS
    assert set(calib_store.load_calib(path="/nonexistent/x.json")) == REQUIRED_KEYS


def test_partial_file_still_has_all_keys(tmp_path):
    """文件里只写了一个字段，合并后仍必须补齐全部 7 个键"""
    p = str(tmp_path / "c.json")
    calib_store.save_calib({"bias": {"dy_mm": 55.0}}, path=p)
    c = calib_store.load_calib(path=p)
    assert set(c) == REQUIRED_KEYS
    assert c["bias"] == {"dx_mm": 0.0, "dy_mm": 55.0, "dtheta_deg": 0.0}


def test_defaults_never_share_mutable_state_between_calls():
    """改坏一次返回值不能污染下一次（否则测试之间互相咬）"""
    a = calib_store.load_calib(path="/nonexistent/x.json")
    a["bias"]["dy_mm"] = 999.0
    a["K"][0][0] = 1.0
    a["dist"].append(1.0)
    b = calib_store.load_calib(path="/nonexistent/x.json")
    assert b["bias"]["dy_mm"] == 0.0
    assert b["K"][0][0] == 1460.76
    assert b["dist"] == [0.0] * 5


# --- R3：路径与 cwd 无关 -------------------------------------------------------


def test_repo_root_is_derived_from_file_location():
    assert calib_store.REPO_ROOT == Path(calib_store.__file__).resolve().parents[3]
    assert (calib_store.REPO_ROOT / "视觉测量" / "脚本").is_dir()


def test_relative_paths_resolve_against_repo_root_not_cwd(monkeypatch, tmp_path):
    """R3 的回归锁：cwd 跑到别处，相对路径仍必须指向仓库根

    把 resolve_path 改成 Path.cwd()/p 会让这条测试失败。
    """
    monkeypatch.chdir(tmp_path)
    assert Path.cwd() == tmp_path.resolve()

    r = calib_store.resolve_path("视觉测量/结果/car_calib.json")
    assert r == calib_store.REPO_ROOT / "视觉测量/结果/car_calib.json"
    assert r.is_relative_to(calib_store.REPO_ROOT)
    assert not r.is_relative_to(tmp_path)

    # 默认路径（无参）走的是同一条路，内参默认值也不能变
    c = calib_store.load_calib()
    assert c["K"][0][0] == 1460.76
    assert c["dist"] == [0.0] * 5


def test_absolute_paths_are_passed_through_untouched():
    assert calib_store.resolve_path("/nonexistent/x.json") == Path("/nonexistent/x.json")


# --- 常量名与字面量 -----------------------------------------------------------


def test_public_constants_are_the_contract_values():
    assert calib_store.BOUNDARY_IDS == (20, 21, 22, 23)
    assert calib_store.VEHICLE_ID == 29
    # 保持仓库相对**字符串**，后续任务按这个字面量断言；解析只在打开文件时做
    assert calib_store.DEFAULT_CALIB_PATH == "视觉测量/结果/car_calib.json"
    assert isinstance(calib_store.DEFAULT_CALIB_PATH, str)


# --- 内参来源 -----------------------------------------------------------------


def test_intrinsics_come_from_the_chessboard_json_not_the_hardcoded_fallback():
    """真值来源是那个 JSON 文件；硬编码兜底只是失效路径"""
    raw = json.loads(
        calib_store.resolve_path(calib_store.CHESS_INTRINSICS_PATH).read_text(encoding="utf-8")
    )
    c = calib_store.load_calib(path="/nonexistent/x.json")
    assert c["K"] == raw["K"]
    assert c["dist"] == raw["dist"]


def test_no_warning_when_the_chessboard_json_is_readable():
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        calib_store.default_calib()
    assert [w for w in caught if issubclass(w.category, RuntimeWarning)] == []


def test_unreadable_chessboard_json_warns_and_still_returns_full_defaults(monkeypatch):
    """兜底值保留（默认必须完整），但必须吵：过期 K 静默回退比报错更危险"""
    monkeypatch.setattr(calib_store, "CHESS_INTRINSICS_PATH", "视觉测量/结果/__不存在__.json")
    with pytest.warns(RuntimeWarning):
        K, dist = calib_store._load_chess_intrinsics()
    assert K[0][0] == 1460.76
    assert dist == [0.0] * 5


# --- 写盘格式 -----------------------------------------------------------------


def test_save_writes_indent2_and_unescaped_utf8(tmp_path):
    """ensure_ascii=False + indent=2：中文结论要能直接读，缩进要能 diff"""
    p = tmp_path / "c.json"
    calib_store.save_calib(
        {
            "bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0},
            "battery": {"sag_v": 0.4, "verdict": "接插件压降，接触正常"},
        },
        path=str(p),
    )
    raw = p.read_text(encoding="utf-8")
    assert '\n  "bias"' in raw
    assert '\n  "battery"' in raw
    assert "接插件压降，接触正常" in raw
    assert "\\u" not in raw
    assert json.loads(raw)["battery"]["verdict"] == "接插件压降，接触正常"


# --- 深合并（save_calib 的 docstring 推荐的那条路）-----------------------------


def test_deep_merge_updates_nested_keys_without_dropping_siblings(tmp_path):
    """只改 dy_mm，dx_mm/dtheta_deg 必须还在 —— 这就是 docstring 教的路"""
    p = str(tmp_path / "c.json")
    calib_store.save_calib({"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0}}, path=p)

    calib_store.save_calib(calib_store.deep_merge(calib_store.load_calib(p), {"bias": {"dy_mm": 77.0}}), p)

    c = calib_store.load_calib(p)
    assert c["bias"] == {"dx_mm": -12.0, "dy_mm": 77.0, "dtheta_deg": 0.0}
    # 落盘文件里也还在（save_calib 原样写出，不该在这一步丢）
    assert json.loads(Path(p).read_text(encoding="utf-8"))["bias"]["dx_mm"] == -12.0


def test_shallow_merge_would_lose_bias_siblings():
    """反例钉子：顶层 ** 确实会丢字段 —— 所以文档/代码不能推荐它"""
    before = {"bias": {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0}}
    shallow = {**before, "bias": {"dy_mm": 77.0}}
    assert "dx_mm" not in shallow["bias"]
    assert calib_store.deep_merge(before, {"bias": {"dy_mm": 77.0}}) == {
        "bias": {"dx_mm": -12.0, "dy_mm": 77.0, "dtheta_deg": 0.0}
    }


def test_deep_merge_does_not_mutate_its_arguments():
    base = {"bias": {"dx_mm": -12.0, "dy_mm": 55.0}}
    override = {"bias": {"dy_mm": 77.0}}
    calib_store.deep_merge(base, override)
    assert base == {"bias": {"dx_mm": -12.0, "dy_mm": 55.0}}
    assert override == {"bias": {"dy_mm": 77.0}}


# --- 损坏的标定文件 -----------------------------------------------------------


def test_corrupt_calib_file_raises_instead_of_silently_defaulting(tmp_path):
    """标定文件坏了必须报错：静默退回默认值 = 把已测好的 bias 当没测过"""
    p = tmp_path / "broken.json"
    p.write_text("{not json", encoding="utf-8")
    with pytest.raises(ValueError):
        calib_store.load_calib(path=str(p))