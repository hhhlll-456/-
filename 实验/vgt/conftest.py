"""让 `import vgt` 在任意 cwd / 任意 pytest 配置下都能解析。

与 `视觉测量/脚本/vision_core/conftest.py` 是同一份保险、同一个理由：

- `实验/vgt/__init__.py` 里那次 `sys.path.insert` 只在**有人 import 本包之后**才执行；
  而从仓库根 `python3 -m pytest` 时，pytest 先要 import 测试模块，那一刻 `实验/`
  还不在 sys.path 上，`from vgt.camera import …` 会直接变成**收集错误**（不是测试失败）。
- conftest.py 位于测试文件的祖先目录上，任何 rootdir 下都会被**先**加载，
  所以在这里把本包的父目录（`实验/`）补上，`cd 实验 && pytest` 与
  `cd <仓库根> && pytest` 两种跑法都能收集到本包。

本机从仓库根能跑通**不能**当作这条保险不存在：那是 ROS 的 `launch_testing` pytest
插件在收集期提前 import 测试模块、顺带把 `实验/` 插进了 sys.path —— 换一台没装这套
插件的机器就失败。路径注入只在这一个地方，测试文件自己不碰 `sys.path`（R22）。
"""
import sys
from pathlib import Path

_EXPERIMENTS_DIR = str(Path(__file__).resolve().parents[1])  # <repo>/实验
if _EXPERIMENTS_DIR not in sys.path:
    sys.path.insert(0, _EXPERIMENTS_DIR)
