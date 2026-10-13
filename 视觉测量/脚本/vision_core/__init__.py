"""视觉真值核心（vision core）：标定存储、ArUco 检测、场地系求解。

纯 numpy/cv2，无硬件依赖，可离线单测。相对路径一律按仓库根解析。
"""

from . import calib_store
from .calib_store import (
    BOUNDARY_IDS,
    DEFAULT_CALIB_PATH,
    REPO_ROOT,
    VEHICLE_ID,
    deep_merge,
    default_calib,
    load_calib,
    resolve_path,
    save_calib,
)

__all__ = [
    "calib_store",
    "load_calib",
    "save_calib",
    "deep_merge",
    "default_calib",
    "resolve_path",
    "DEFAULT_CALIB_PATH",
    "REPO_ROOT",
    "BOUNDARY_IDS",
    "VEHICLE_ID",
]
