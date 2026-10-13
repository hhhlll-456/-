"""标定数据存储：场地/相机/车体的全部标定量，一个 JSON 文件进出。

数据文件 `视觉测量/结果/car_calib.json`（不存在时用内建默认值）：

    K                  3x3 相机内参（默认取自 视觉测量/结果/calib_intrinsics_chess.json）
    dist               5 元素畸变系数
    boundary_side_mm   边界码 20/21/22/23 黑色有效区边长，实测 25.0
    vehicle_side_mm    车载码 29 边长，实测 30.0
    bias               车载码相对四轮轴心矩形中心的平面/朝向偏置（A1 人工量）
    battery            电池带载压降与结论（A4 人工量）
    coplanar           四轮接地点是否共面（A3 塞尺复核）

两条硬规矩（下游 12 个任务都靠它）：
1. 路径：所有相对路径都相对**仓库根**解析，不相对当前工作目录
   （本代码会在 视觉测量/脚本、实验、仓库根 等不同 cwd 下被调用）。
2. 写入：只改个别字段必须用 `deep_merge`，顶层 `**` 是浅合并，
   会整段换掉 bias/battery，静默丢掉没列出的字段（例如 dy_mm=55 变回 0）。
"""
from __future__ import annotations

import copy
import json
import warnings
from functools import lru_cache
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

# 棋盘内参文件读不到时的兜底值 = 2026-10-13 那次标定的数字。
# 用它是为了 load_calib() 在任何情况下都返回完整默认值；
# 但它可能已经过期，所以这条路径会发 RuntimeWarning（见 _chess_intrinsics_cached）。
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


def _read_json_object(path: Path) -> dict | None:
    """读 JSON 对象；文件缺失/损坏/顶层不是对象时返回 None（不抛异常）。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


@lru_cache(maxsize=None)
def _chess_intrinsics_cached(path_str: str, mtime_ns: int, size: int) -> tuple[list, list]:
    """解析棋盘内参。`mtime_ns`/`size` 只是缓存键：文件一变就重新解析，
    所以既不会每次 load_calib 都重读 JSON，也不会返回过期的 K。"""
    data = _read_json_object(Path(path_str))
    if data is None:
        warnings.warn(
            f"读不到相机内参 {path_str}，退回 2026-10-13 棋盘标定的硬编码兜底 K/dist。"
            "若已重新标定，这个兜底值就是过期的，请修好该文件。",
            RuntimeWarning,
            stacklevel=2,
        )
        data = {}
    K, dist = data.get("K"), data.get("dist")
    return (
        copy.deepcopy(K) if isinstance(K, list) else copy.deepcopy(_FALLBACK_K),
        copy.deepcopy(dist) if isinstance(dist, list) else list(_FALLBACK_DIST),
    )


def _load_chess_intrinsics() -> tuple[list, list]:
    """取相机内参默认值（K, dist），按文件 mtime 缓存。"""
    p = resolve_path(CHESS_INTRINSICS_PATH)
    try:
        st = p.stat()
    except OSError:
        return _chess_intrinsics_cached(str(p), -1, -1)
    return _chess_intrinsics_cached(str(p), st.st_mtime_ns, st.st_size)


def default_calib() -> dict:
    """内建默认标定。A1–A4 人工测量结果为 None 表示“尚未测量”。

    深拷贝内参：内参来自进程级缓存，调用方改返回值不能污染缓存（否则下一次
    load_calib 会拿到被上一个调用者改坏的 K）。
    """
    K, dist = _load_chess_intrinsics()
    return {
        "K": copy.deepcopy(K),
        "dist": copy.deepcopy(dist),
        "boundary_side_mm": DEFAULT_BOUNDARY_SIDE_MM,
        "vehicle_side_mm": DEFAULT_VEHICLE_SIDE_MM,
        "bias": {"dx_mm": 0.0, "dy_mm": 0.0, "dtheta_deg": 0.0},
        "battery": {"sag_v": None, "verdict": None},
        "coplanar": None,
    }


def deep_merge(base: dict, override: dict) -> dict:
    """override 覆盖 base，dict 递归合并。

    这是改个别字段时唯一安全的合并方式：
        deep_merge({"bias": {"dx_mm": -12.0, "dy_mm": 55.0}}, {"bias": {"dy_mm": 77.0}})
        -> {"bias": {"dx_mm": -12.0, "dy_mm": 77.0}}
    而 `{**base, **override}` 会把整个 bias 段换成 `{"dy_mm": 77.0}`，dx_mm 静默丢失。
    """
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_calib(path: str | Path | None = None) -> dict:
    """读标定；文件不存在则返回完整默认值，存在则与默认值深合并。

    文件存在但损坏 → 抛 ValueError。**不**静默退回默认值：
    那等于把已测好的 bias/battery 当成没测过。
    """
    p = resolve_path(DEFAULT_CALIB_PATH if path is None else path)
    defaults = default_calib()
    if not p.is_file():
        return defaults
    raw = _read_json_object(p)
    if raw is None:
        raise ValueError(f"标定文件读不出来（不存在/非 UTF-8/非法 JSON/顶层不是对象）: {p}")
    return deep_merge(defaults, raw)


def save_calib(data: dict, path: str | Path | None = None) -> None:
    """把 data 原样写入标定文件（不与旧文件合并，父目录自动创建）。

    要“保留旧值、只改某几个字段”，**必须**深合并；顶层 `**` 会整段替换：

        save_calib(deep_merge(load_calib(p), {"bias": {"dy_mm": 55.0}}), p)
        # 或者更啰嗦但同样安全：
        data = load_calib(p)
        data["bias"]["dy_mm"] = 55.0
        save_calib(data, p)
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
    "deep_merge",
    "load_calib",
    "save_calib",
]