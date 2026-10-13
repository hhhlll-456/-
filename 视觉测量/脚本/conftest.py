"""pytest 从任意 cwd（视觉测量/脚本、实验、仓库根）调用时都能 import vision_core。

只做 sys.path 注入 —— 路径解析规则只有一个来源 calib_store.resolve_path()，
测试文件里不各自改 sys.path。
"""
import sys
from pathlib import Path

_SCRIPT_DIR = str(Path(__file__).resolve().parent)  # <repo>/视觉测量/脚本
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
