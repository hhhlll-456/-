"""`vgt`：视觉真值实验的编排包（Task 8–12 的产物），住在 `实验/vgt/`。

## 为什么这个文件要改 `sys.path`（R11）

视觉核心（检测/场地系/标定）住在 `视觉测量/脚本/vision_core/`，**不在本包内**；
两个包跨目录，唯一的公共入口就是 `sys.path`。这里把 `视觉测量/脚本` **前置**进
`sys.path`，于是本包内任何模块都能直接

    from vision_core.markers import detect_markers
    from vision_core import calib_store

而不必要求调用方设 `PYTHONPATH`、也不必挑一个特定的 cwd 来 import 本包。
路径由 `Path(__file__)` 的父目录链推出来（`实验/vgt` → `实验` → 仓库根），
**不写死绝对路径**。

（计划原文给两个包都起名 `vgt`。两个同名顶层包**不能共存于一个解释器**，
而 Task 10 的 harness 真的要同时 import 两边 —— 所以视觉核心改名 `vision_core`，
本包保留 `vgt`。裁决见 SDD 台账 R11。）

## 前置条件：`实验/` 在 `sys.path` 上

`vgt` 这个顶层名只在 `实验/` 是 `sys.path` 一员时才解析得到，所以冒烟/验收命令
一律从 `实验/` 出发：

    cd /home/hanli/麦轮小车pid/实验 && python3 -m pytest vgt/tests/ -v

（`python3 -m pytest` 会把 cwd 放进 `sys.path`。从仓库根跑整套时，
`实验/vgt/conftest.py` 会补上这一条 —— 测试文件自己不碰 `sys.path`，R22。）
"""
from __future__ import annotations

import sys
from pathlib import Path

# 本文件位于 <repo>/实验/vgt/__init__.py
#   parents[0]=vgt  [1]=实验  [2]=仓库根
_SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "视觉测量" / "脚本"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
