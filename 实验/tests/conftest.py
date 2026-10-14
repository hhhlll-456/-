"""让 `import wheelmap_sweep` 在任意 cwd / 任意 pytest 配置下都能解析。

与 `实验/vgt/conftest.py` 是同一份保险、同一个理由（R22：路径注入只在一处，
测试文件自己不碰 sys.path）：

- `实验/wheelmap_sweep.py` 不是包，从仓库根 `python3 -m pytest` 收集本目录的
  测试时，`实验/` 不一定已在 sys.path 上；conftest.py 位于测试文件的祖先目录，
  任何 rootdir 下都会被**先**加载，所以在这里把 `实验/` 补上。
- 同时 `wheelmap_sweep` 自己会 insert `实验/`（为了 import car_link），但那只在
  它被 import 之后才发生 —— 来不及帮"第一次 import 它"的那一步。
"""
import sys
from pathlib import Path

_EXPERIMENTS_DIR = str(Path(__file__).resolve().parents[1])  # <repo>/实验
if _EXPERIMENTS_DIR not in sys.path:
    sys.path.insert(0, _EXPERIMENTS_DIR)
