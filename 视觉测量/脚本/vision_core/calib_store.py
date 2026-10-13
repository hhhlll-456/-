#!/usr/bin/env python3
"""标定数据存储：场地/相机/车体的全部标定量，一个 JSON 文件进出。

数据文件 `视觉测量/结果/car_calib.json`（不存在时用内建默认值）：

    K                  3x3 相机内参（默认取自 视觉测量/结果/calib_intrinsics_chess.json）
    dist               5 元素畸变系数
    boundary_side_mm   边界码 20/21/22/23 黑色有效区边长，实测 25.0
    vehicle_side_mm    车载码 29 边长，实测 30.0
    bias               车载码相对四轮轴心矩形中心的平面/朝向偏置（A1 人工量）
    battery            电池带载压降与结论（A4 人工量）
    coplanar           四轮接地点是否共面（A3 塞尺复核）

路径约定：所有相对路径都相对**仓库根**解析，不相对当前工作目录
（本代码会在 视觉测量/脚本、实验、仓库根 等不同 cwd 下被调用）。
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

# 本文件位于 <repo>/视觉测量/脚本/vision_core/calib_store.py
#   parents[0]=vision_core  [1]=脚本  [2]=视觉测量  [3]=仓库根
REPO_ROOT = Path(__file__).resolve().parents[3]

# 公共常量（字符串保持仓库相对，后续任务按此断言）
DEFAULT_CALIB_PATH = "视觉测量/结果/car_calib.json"
CHESS_INTRINSICS_PATH = "视觉测量/结果/calib_intrinsics_chess.json"

BOUNDARY_IDS = (20, 21, 22, 23)
VEHICLE_ID = 29

# 实测值，不可改（见 plan Global Constraints）
DEFAULT_BOUNDARY_SIDE_MM = 25.0
DEFAULT_VEHICLE_SIDE_MM = 30.0

# 棋盘内参文件缺失/损坏时的兜底值，与该文件的 K/dist 一致
_FALLBACK_K = [
    [1460.76, 0.0, 1019.44],
    [0.0, 1460.76, 592.77],
    [0.0, 0.0, 1.0],
]
_FALLBACK_DIST = [0.0, 0.0, 0.0, 0.0, 0.0]


def resolve_path(path: str | Path) -> Path:
    """绝对路径原样使用；相对路径按仓库根解析（与 cwd 无关）。"""
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def _load_chess_intrinsics() -> tuple[list, list]:
    """读棋盘内参文件；缺失或不可解析时退回硬编码兜底值。"""
    K = copy.deepcopy(_FALLBACK_K)
    dist = list(_FALLBACK_DIST)
    f = resolve_path(CHESS_INTRINSICS_PATH)
    if not f.is_file():
        return K, dist
    try:
        data = json.loads(f.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return K, dist
    if isinstance(data, dict):
        if isinstance(data.get("K"), list):
            K = copy.deepcopy(data["K"])
        if isinstance(data.get("dist"), list):
            dist = copy.deepcopy(data["dist"])
    return K, dist


def default_calib() -> dict:
    """内建默认标定。A1–A4 人工测量结果为 None 表示“尚未测量”。"""
    K, dist = _load_chess_intrinsics()
    return {
        "K": K,
        "dist": dist,
        "boundary_side_mm": DEFAULT_BOUNDARY_SIDE_MM,
        "vehicle_side_mm": DEFAULT_VEHICLE_SIDE_MM,
        "bias": {"dx_mm": 0.0, "dy_mm": 0.0, "dtheta_deg": 0.0},
        "battery": {"sag_v": None, "verdict": None},
        "coplanar": None,
    }


def _deep_merge(base: dict, override: dict) -> dict:
    """override 覆盖 base，dict 递归合并（文件里只写了一部分字段也能补全）。"""
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_calib(path: str | Path | None = None) -> dict:
    """读标定；文件不存在则返回完整默认值，存在则与默认值深合并。"""
    p = resolve_path(DEFAULT_CALIB_PATH if path is None else path)
    defaults = default_calib()
    if not p.is_file():
        return defaults
    raw = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"标定文件应为 JSON 对象: {p}")
    return _deep_merge(defaults, raw)


def save_calib(data: dict, path: str | Path | None = None) -> None:
    """原样写出 data（不自动与旧文件合并）。

    要保留默认值再改个别字段，请用：
        save_calib({**load_calib(), **updates})
    """
    p = resolve_path(DEFAULT_CALIB_PATH if path is None else path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


__all__ = [
    "REPO_ROOT",
    "DEFAULT_CALIB_PATH",
    "CHESS_INTRINSICS_PATH",
    "BOUNDARY_IDS",
    "VEHICLE_ID",
    "DEFAULT_BOUNDARY_SIDE_MM",
    "DEFAULT_VEHICLE_SIDE_MM",
    "resolve_path",
    "default_calib",
    "load_calib",
    "save_calib",
]