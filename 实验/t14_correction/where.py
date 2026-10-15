#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**只定位、不驱动**：看一眼小车现在在场地哪里。

为什么需要它：`campaign.py` 在「终点帧解不出车码」时会停批（这是对的 —— 位置未知
就不该继续跑），但那时车停在哪儿、还安不安全，得有个副作用为零的工具去查。
本脚本只开相机 + 连板子做一次 `locate()`，**不发任何 `RUN`/`M`**。

    cd 实验 && python3 t14_correction/where.py

退出码：0 = 解出位姿且整车在场地内；3 = 没解出；4 = 解出来但车体越界/贴边。
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPERIMENT_DIR = HERE.parent
ROOT = EXPERIMENT_DIR.parent
for _p in (str(EXPERIMENT_DIR), str(ROOT / "视觉测量" / "脚本"), str(HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from vgt.camera import CameraSession                           # noqa: E402
from vgt.harness import Harness                                # noqa: E402
from vision_core import calib_store                            # noqa: E402

from campaign import open_link, FIELD_HALF_X, FIELD_HALF_Y, CAR_HALF_X, CAR_HALF_Y


def main() -> int:
    calib = calib_store.load_calib(calib_store.DEFAULT_CALIB_PATH)
    with CameraSession() as cam, open_link() as car:
        h = Harness(car, cam, calib, home_xy=(0.0, 0.0))
        pose = h.locate()
        if pose is None:
            print(f"❌ locate 失败：{h.last_locate_reason}", file=sys.stderr)
            return 3
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    inside = abs(x) + CAR_HALF_X <= FIELD_HALF_X and abs(y) + CAR_HALF_Y <= FIELD_HALF_Y
    clear_x = FIELD_HALF_X - CAR_HALF_X - abs(x)
    clear_y = FIELD_HALF_Y - CAR_HALF_Y - abs(y)
    print(f"车在 x={x:8.2f} y={y:8.2f} yaw={yaw:+7.2f}°")
    print(f"  车中心余量：x 方向 {clear_x:6.1f} mm、y 方向 {clear_y:6.1f} mm"
          f"（= 车体到场地边的距离）")
    print(f"  整车在场地内：{'是' if inside else '❗否'}")
    return 0 if inside else 4


if __name__ == "__main__":
    raise SystemExit(main())
