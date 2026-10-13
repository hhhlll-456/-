"""让 `import vision_core` 在任意 cwd / 任意 pytest 配置下都能解析。

为什么 视觉测量/脚本/conftest.py 之外还要这一份：pytest 的 confcutdir 默认等于
rootdir，而 rootdir 会随“cwd + 命令行参数”变化（本机插件环境下实测会算出仓库根、
/tmp 等不同值）。confcutdir 之上的 conftest.py 根本不会被加载，于是
`import vision_core` 失败。包内的 conftest.py 位于测试文件的祖先目录上，
任何 rootdir 下都会被收集，所以这里再放一份保险。
"""
import sys
from pathlib import Path

_SCRIPT_DIR = str(Path(__file__).resolve().parents[1])  # <repo>/视觉测量/脚本
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
