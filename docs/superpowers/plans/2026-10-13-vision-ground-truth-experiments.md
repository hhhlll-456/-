# 视觉 Ground Truth 自动化实验系统 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立一条零手工的实验管线——PC 经 Wi-Fi 下发一条走/转指令，自动拍摄起/中/终三点、用 `solvePnP` 解算场地系下的位移 mm 与转角 deg，并与编码器/IMU 解算并排入表。

**Architecture:** 分三层。**视觉真值核心**（纯 numpy/cv2，无硬件依赖，可离线单测）：检测 5 个 ArUco → `solvePnP` 单码位姿 → 4 边界码构建场地系 → 车体偏置修正 → 输出 `(x_mm, y_mm, yaw_deg)`。**固件**（ESP32）：流式遥测（修掉"全缓冲到 DONE"）、掉电缓解、wheel_map 双表重建。**PC 调度器**：视觉闭环归位 → 三点拍照 → 收遥测 → 对比入表。

**Tech Stack:** Python 3 / OpenCV 4.6.0（`cv2.aruco` 旧 API `detectMarkers`，**无** `ArucoDetector`）/ numpy 1.26.4 / scipy 1.11.4 / openpyxl / pytest 7.4.4；ESP32 Arduino via PlatformIO；`cv2.solvePnPGeneric` + `SOLVEPNP_IPPE_SQUARE`。

**Spec:** `docs/superpowers/specs/2026-10-13-vision-ground-truth-experiments-design.md`

---

## Global Constraints

- **码尺寸（实测，不可改）**：边界码 ID 20/21/22/23 黑色有效区边长 **25.0 mm**；车载码 ID 29 边长 **30.0 mm**
- **相机内参**：`视觉测量/结果/calib_intrinsics_chess.json` → `K=[[1460.76,0,1019.44],[0,1460.76,592.77],[0,0,1]]`，`dist=[0,0,0,0,0]`（针孔模型）
- **场地** ≈ 505 × 252 mm；车体包络 270 × 149 mm；纵向余量 235 mm、横向余量 103 mm
- **旋转上限 45°**（转 90° 后车宽 270 mm > 短边 252 mm）
- **直行距离 ≤ 200 mm**、**横移步长 ≤ 80 mm**、**斜行位移 ≤ 120 mm**
- **车载码位置**：ID29 在**左甲板**，即 **FL 与 RL 之间的上方**，偏置不在几何中心，必须做偏置修正
- **yaw 参考系 = 场地系**，图像 up = 0°
- **固件轮序**：索引 `0=FL 1=FR 2=RL 3=RR`；`invert[4]={true,false,true,false}`；forward 符号 FL=−1 FR=+1 RL=−1 RR=+1
- **引脚**：电机 `FL{13,15} FR{12,14} RL{27,26} RR{25,33}`；编码器 `FL32 FR34 RL35 RR36`
- **IMU 保持 6 Hz / 9600 波特**（用户已决定不改），仅作粗略交叉校验，**不是**控制反馈
- **命令只从 Wi-Fi TCP 8266 进**；`/dev/ttyACM0` 串口仅调试输出
- **编码器单相（A 相）**：只能计数不能辨向，速度靠符号约定
- **不做大角度旋转、不换硬件、不做完整闭环 PID 整定**
- **`.workbuddy/` 目录勿动**

---

## Review Focus

以下五类是规格暗示、但任何任务的常规测试都不会覆盖、且最可能让一个理性使用者踩坑的情况。每个都在下方对应任务里补了钉住它的测试。

1. **中点拍照时车正在动** → 运动模糊导致 ID29 检测失败。期望：优雅降级为两点（起/终），pipeline 不崩、不产出假数据。
2. **边界码被车撞移位** → 场地系静默错误，之后每条实验的真值全错。期望：每条实验前复核 4 码像素位置，漂移超阈值则拒绝跑。
3. **brownout 中途死板** → PC 分不清"跑完了"还是"板子死了"。期望：超时 + 重连 + 复位原因回报，该条实验标记为 invalid 而非静默产出截断数据。
4. **ID29 偶发检测丢失**（帧/光/遮挡） → 期望：不抛异常，返回"该帧无解"，重试上限内拿不到就报告缺失。
5. **IPPE_SQUARE 返回两个解** → 选错解得到镜像/翻转位姿，位移和转角都会是错的。期望：按重投影误差选解，并断言选出的解是正解。

---

## File Structure

```
docs/superpowers/plans/2026-10-13-vision-ground-truth-experiments.md   # 本文件

.gitignore                                    # 新建：排除 .workbuddy/、__pycache__、大图
视觉测量/脚本/vgt/__init__.py                  # 新建：包入口，导出公共符号
视觉测量/脚本/vgt/calib_store.py               # 新建：加载/保存标定（K、码尺寸、车体偏置）
视觉测量/脚本/vgt/markers.py                   # 新建：检测 + solvePnP 单码位姿
视觉测量/脚本/vgt/field.py                     # 新建：场地系构建 + 车体偏置修正
视觉测量/脚本/vgt/tests/test_markers.py        # 新建
视觉测量/脚本/vgt/tests/test_field.py          # 新建
视觉测量/脚本/verify_vision.py                 # 新建：A2 尺子验证工具（人工步骤用）

实验/vgt/__init__.py                          # 新建
实验/vgt/camera.py                            # 新建：持久 VideoCapture + 锁曝光/对焦
实验/vgt/matrix.py                            # 新建：按场地裁剪后的实验矩阵
实验/vgt/odometry.py                          # 新建：编码器/IMU 解算（纯数学）
实验/vgt/harness.py                           # 新建：归位 + 三点拍照 + 下发指令
实验/vgt/compare.py                           # 新建：视觉 vs 内部传感器对比 + 写表
实验/vgt/tests/test_odometry.py               # 新建
实验/vgt/tests/test_matrix.py                 # 新建
实验/wheelmap_sweep.py                        # 新建：duty↔ω 采集（架空 + 落地）
实验/wheelmap_gen.py                          # 新建：由采集 JSON 生成 wheel_map.h
实验/run_all_experiments.py                   # 新建：批量入口

固件/exp/src/main.cpp                         # 修改：流式遥测 + 掉电缓解 + 复位原因回报
固件/exp/src/stagger.h                        # 新建：错开起转（编译期开关，默认关）
固件/exp/src/wheel_map.h                      # 重新生成（由 wheelmap_gen.py 产出）
固件/exp/data/wheelmap_airborne.json          # 新建：架空套采集原始数据
固件/exp/data/wheelmap_grounded.json          # 新建：落地套采集原始数据

视觉测量/结果/car_calib.json                  # 新建：A1–A4 人工标定结果
交付物/麦轮小车参数测量清单.xlsx               # 修改（经 build_v4.py 重新生成）
交付物/.麦轮小车参数测量清单.ref/build_v4.py   # 修改：视觉测量章节 + 修正冲突项
小车档案.md                                    # 新建：长期维护的小车事实档案
docs/superpowers/analysis/error-mece.md        # 新建：MECE 误差分析成文
```

---

### Task 1: 版本库基线 + 标定数据存储

**Files:**
- Create: `.gitignore`
- Create: `视觉测量/脚本/vgt/__init__.py`
- Create: `视觉测量/脚本/vgt/calib_store.py`
- Test: `视觉测量/脚本/vgt/tests/test_calib_store.py`

**Interfaces:**
- Consumes: 无
- Produces:
  - `load_calib(path=None) -> dict`，返回含 `K`、`dist`、`boundary_side_mm`、`vehicle_side_mm`、`bias`、`battery`、`coplanar` 键的字典；`path` 缺省为 `视觉测量/结果/car_calib.json`
  - `save_calib(data: dict, path=None) -> None`
  - `DEFAULT_CALIB_PATH = "视觉测量/结果/car_calib.json"`
  - 常量 `BOUNDARY_IDS = (20, 21, 22, 23)`、`VEHICLE_ID = 29`

- [ ] **Step 1: 初始化版本库并写 .gitignore**

```bash
cd /home/hanli/麦轮小车pid
git init
```

`.gitignore` 内容（每行一条）：`.workbuddy/`、`__pycache__/`、`*.pyc`、`实验/原始日志/*.jpg`、`视觉测量/照片/calib_chess_v3/`、`.pio/`

> `.workbuddy/` 是工具记忆目录，HANDOFF 明令勿动——必须进 `.gitignore`，不要删除它。

- [ ] **Step 2: 写失败的测试**

`视觉测量/脚本/vgt/tests/test_calib_store.py`：

```python
from vgt import calib_store

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
```

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/hanli/麦轮小车pid/视觉测量/脚本 && python3 -m pytest vgt/tests/test_calib_store.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vgt'`

- [ ] **Step 4: 实现**

`vgt/__init__.py` 导出 `BOUNDARY_IDS`、`VEHICLE_ID`。

`vgt/calib_store.py`：`load_calib` 在文件缺失时返回内建默认值（K 取自 `视觉测量/结果/calib_intrinsics_chess.json`，其余按 Global Constraints 硬编码），存在时与默认值深合并。`save_calib` 用 `json.dump(..., ensure_ascii=False, indent=2)`。

- [ ] **Step 5: 跑测试确认通过**

Run: `cd /home/hanli/麦轮小车pid/视觉测量/脚本 && python3 -m pytest vgt/tests/test_calib_store.py -v`
Expected: 2 passed

- [ ] **Step 6: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add .gitignore 视觉测量/脚本/vgt/
git commit -m "chore: init repo baseline + calibration store"
```

---

### Task 2: ArUco 检测 + solvePnP 单码位姿

**Files:**
- Create: `视觉测量/脚本/vgt/markers.py`
- Test: `视觉测量/脚本/vgt/tests/test_markers.py`

**Interfaces:**
- Consumes: Task 1 的 `calib_store.load_calib`
- Produces:
  - `@dataclass MarkerPose`：`marker_id: int`、`rvec: np.ndarray (3,1)`、`tvec: np.ndarray (3,1)`（相机系 mm）、`reproj_rms_px: float`、`corners_px: np.ndarray (4,2)`
  - `detect_markers(img_bgr, calib: dict) -> dict[int, MarkerPose]`：检出全部 DICT_4X4_50 码并解位姿；**未检出的码不出现在返回值里**（返回空 dict 或缺键），**绝不抛异常**
  - `solve_marker_pose(corners_px: np.ndarray (4,2), side_mm: float, K: list, dist: list) -> tuple[np.ndarray, np.ndarray, float]`

- [ ] **Step 1: 写失败的测试**

`vgt/tests/test_markers.py`：

```python
import cv2, numpy as np, pytest
from vgt.markers import detect_markers, solve_marker_pose
from vgt import calib_store

K = [[1460.76,0,1019.44],[0,1460.76,592.77],[0,0,1]]
D = [0.0]*5

def test_solve_marker_pose_recovers_known_pose():
    # 构造：码 30mm 放在相机系 (0, -400, 100) mm，法向朝相机
    side = 30.0
    obj = np.float32([[-side/2, side/2, 0],[side/2, side/2, 0],
                      [side/2,-side/2, 0],[-side/2,-side/2, 0]])
    rvec = np.float64([[0.0],[0.0],[0.0]])
    tvec = np.float64([[0.0],[-400.0],[100.0]])
    img, _ = cv2.projectPoints(obj, rvec, tvec, np.float64(K), np.float64(D))
    r2, t2, rms = solve_marker_pose(img.reshape(4,2).astype(np.float64), side, K, D)
    assert np.allclose(t2, tvec, atol=1e-3)
    assert rms < 1e-6

def test_solve_marker_pose_picks_correct_solution():
    """IPPE_SQUARE 有两个解，必须选重投影误差小且 tvec z>0 的正解（Review Focus #5）"""
    side = 30.0
    obj = np.float32([[-side/2, side/2, 0],[side/2, side/2, 0],
                      [side/2,-side/2, 0],[-side/2,-side/2, 0]])
    tvec = np.float64([[30.0],[-400.0],[120.0]])
    img, _ = cv2.projectPoints(obj, np.float64([[0.0],[0.0],[0.0]]), tvec, np.float64(K), np.float64(D))
    r2, t2, rms = solve_marker_pose(img.reshape(4,2).astype(np.float64), side, K, D)
    assert t2[2, 0] > 0
    assert np.allclose(t2, tvec, atol=1e-3)

def test_detect_markers_on_real_photo_finds_all_five():
    img = cv2.imread("视觉测量/照片/photo_review_20261013.jpg")
    poses = detect_markers(img, calib_store.load_calib())
    assert set(poses) == {20, 21, 22, 23, 29}

def test_detect_markers_returns_empty_on_blank_image():
    """Review Focus #4：检不到不能抛异常"""
    poses = detect_markers(np.zeros((1080,1920,3), np.uint8), calib_store.load_calib())
    assert poses == {}

def test_vehicle_marker_height_is_not_forced_to_zero():
    """ID29 悬空 55~75mm，PnP 必须解出非零高度（这是弃用桌面单应的理由）"""
    img = cv2.imread("视觉测量/照片/photo_review_20261013.jpg")
    t = detect_markers(img, calib_store.load_calib())[29].tvec
    assert 40.0 < t[2, 0] < 120.0, f"ID29 高度 {t[2,0]:.1f}mm 不在预期区间"
```

> 最后一个测试是 **D1 决策的回归锁**：如果有人改回用桌面单应投影，这个断言会失败。

- [ ] **Step 2: 跑测试确认失败**

Run: `cd /home/hanli/麦轮小车pid/视觉测量/脚本 && python3 -m pytest vgt/tests/test_markers.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vgt.markers'`

- [ ] **Step 3: 实现**

`solve_marker_pose`：物体点按 ArUco 角序 `[-s/2,s/2,0] [s/2,s/2,0] [s/2,-s/2,0] [-s/2,-s/2,0]` 构造（与 `detectMarkers` 返回的 TL/TR/BR/BL 角序一致）。用 `cv2.solvePnPGeneric(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)` 取全部解，对每个解算重投影 RMS，**取 RMS 最小且 `tvec[2] > 0` 的那个**，重投影 RMS 用 `cv2.projectPoints` 实算。

`detect_markers`：`cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)` + `DetectorParameters_create()` 设 `cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX`。码边长按 id 查表（20–23 → `boundary_side_mm`，29 → `vehicle_side_mm`，其它 id 跳过）。整个函数体包在 `try/except cv2.error` 里返回已解出的部分。

- [ ] **Step 4: 跑测试确认通过**

Run: `cd /home/hanli/麦轮小车pid/视觉测量/脚本 && python3 -m pytest vgt/tests/test_markers.py -v`
Expected: 5 passed

- [ ] **Step 5: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 视觉测量/脚本/vgt/
git commit -m "feat(vgt): ArUco detection + IPPE_SQUARE single-marker pose"
```

---

### Task 3: 场地系构建 + 车体偏置修正

**Files:**
- Create: `视觉测量/脚本/vgt/field.py`
- Test: `视觉测量/脚本/vgt/tests/test_field.py`

**Interfaces:**
- Consumes: Task 2 的 `MarkerPose`、`detect_markers`
- Produces:
  - `@dataclass FieldFrame`：`R_cam2field: np.ndarray (3,3)`（行向量 = 场地三轴在相机系下的方向）、`origin_cam: np.ndarray (3,)`（场地原点在相机系 mm）、`marker_px: dict[int, np.ndarray (2,)]`
  - `build_field_frame(poses: dict[int, MarkerPose]) -> FieldFrame`：原点 = 4 边界码质心；X = ID20→ID21；Y 指向 ID23 侧；缺任一码则抛 `ValueError`
  - `to_field(ff: FieldFrame, p_cam: np.ndarray (3,)) -> np.ndarray (3,)`
  - `body_pose_in_field(ff: FieldFrame, pose: MarkerPose, bias: dict) -> tuple[float, float, float]`：返回车体原点在场地系的 `(x_mm, y_mm, yaw_deg)`
  - `heading_deg(R_cam2marker: np.ndarray, ff: FieldFrame) -> float`：图像 up = 0°

- [ ] **Step 1: 写失败的测试**

`vgt/tests/test_field.py`：

```python
import numpy as np, pytest, cv2
from vgt.markers import detect_markers
from vgt.field import build_field_frame, to_field, body_pose_in_field, heading_deg
from vgt import calib_store

def _poses():
    img = cv2.imread("视觉测量/照片/photo_review_20261013.jpg")
    return detect_markers(img, calib_store.load_calib())

def test_build_field_frame_requires_all_four_boundary_markers():
    poses = _poses(); poses.pop(23)
    with pytest.raises(ValueError):
        build_field_frame(poses)

def test_field_origin_is_centroid_and_TL_maps_near_origin():
    ff = build_field_frame(_poses())
    tl = to_field(ff, _poses()[20].tvec.reshape(3))
    assert np.linalg.norm(tl) < 1.0        # TL 是原点

def test_TR_is_positive_x_and_BL_is_positive_y():
    poses = _poses(); ff = build_field_frame(poses)
    tr = to_field(ff, poses[21].tvec.reshape(3))
    bl = to_field(ff, poses[23].tvec.reshape(3))
    assert tr[0] > 400 and abs(tr[1]) < 5      # TR 在 +X 约 505mm
    assert bl[1] > 200 and abs(bl[0]) < 5      # BL 在 +Y 约 252mm

def test_heading_is_zero_when_car_points_up_in_image():
    ff = build_field_frame(_poses())
    poses = _poses()
    # 构造一个「朝向与场地 +X 同向」的假设位姿，检查 heading 定义自洽
    R = np.eye(3)
    assert abs(heading_deg(R, ff)) < 1e-6

def test_body_bias_shifts_origin_left_of_marker():
    """ID29 在左甲板：偏置 dy>0 时，车体原点应在码的右侧(field +y 小)"""
    ff = build_field_frame(_poses()); poses = _poses()
    bias = {"dx_mm": 0.0, "dy_mm": 55.0, "dtheta_deg": 0.0}
    x, y, yaw = body_pose_in_field(ff, poses[29], bias)
    mx, my, _ = body_pose_in_field(ff, poses[29], {"dx_mm":0,"dy_mm":0,"dtheta_deg":0})
    assert abs(y - my) > 5.0

def test_body_pose_yaw_equals_marker_yaw_when_dtheta_zero():
    ff = build_field_frame(_poses()); poses = _poses()
    bias = {"dx_mm": -12.0, "dy_mm": 55.0, "dtheta_deg": 0.0}
    _, _, yaw = body_pose_in_field(ff, poses[29], bias)
    _, _, yaw_m = body_pose_in_field(ff, poses[29], {"dx_mm":0,"dy_mm":0,"dtheta_deg":0})
    assert abs(yaw - yaw_m) < 1e-6
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd /home/hanli/麦轮小车pid/视觉测量/脚本 && python3 -m pytest vgt/tests/test_field.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vgt.field'`

- [ ] **Step 3: 实现**

`build_field_frame`：
1. 取 4 个边界码的 `tvec.reshape(3)`，质心 = `origin_cam`
2. 场地平面法向 `n`：用四角点叉积平均，或对四角点做平面拟合（PCA，最小特征值对应法向），符号取"指向相机"（与 `(质心 - K主点)` 同向）
3. `x_axis = normalize(t21 - t20)`，投影到平面内；`y_axis = normalize(cross(n, x_axis))`，**若与 `(t23 - t20)` 点积为负则整体翻号**
4. `z_axis = cross(x_axis, y_axis)`；`R_cam2field = vstack([x_axis, y_axis, z_axis])`
5. `marker_px` 存 4 码的角点均值，供漂移复核

`to_field(ff, p_cam) = ff.R_cam2field @ (p_cam - ff.origin_cam)`

`heading_deg`：把 marker 旋转矩阵的列轴变换到场地系，取车体前向轴在场地平面的方位角，按 `图像 up = 0°` 归一化到 (−180, 180]。

`body_pose_in_field`：先 `p_origin_cam = p_marker_cam - R_marker @ [dx,dy,0]`，再 `to_field`；yaw = `heading_deg` − `dtheta`。

- [ ] **Step 4: 跑测试确认通过**

Run: `cd /home/hanli/麦轮小车pid/视觉测量/脚本 && python3 -m pytest vgt/tests/test_field.py -v`
Expected: 6 passed

- [ ] **Step 5: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 视觉测量/脚本/vgt/
git commit -m "feat(vgt): field frame construction + car body bias correction"
```

---

### Task 4: 一次性人工标定工具（A1–A4 数据采集）

**Files:**
- Create: `视觉测量/脚本/verify_vision.py`
- Modify: `视觉测量/结果/car_calib.json`（由工具写入）

**Interfaces:**
- Consumes: Task 2 `detect_markers`、Task 3 `build_field_frame` / `body_pose_in_field`
- Produces: CLI `python3 视觉测量/脚本/verify_vision.py <照片路径...>`，往 stdout 打印每张图的 `(x_mm, y_mm, yaw_deg)` 与跨图残差 RMS，并把人工填写的 `bias` / `battery` / `coplanar` 合并写回 `视觉测量/结果/car_calib.json`

- [ ] **Step 1: 写工具骨架**

CLI 参数：`--bias-dx` `--bias-dy` `--bias-dtheta`（可选，写入 car_calib.json）、`--battery-sag-v`（可选）、`--coplanar yes|no`（可选）。无参数时只读照片求解并打印。

- [ ] **Step 2: 用参考照片跑通**

Run: `cd /home/hanli/麦轮小车pid && python3 视觉测量/脚本/verify_vision.py 视觉测量/照片/photo_review_20261013.jpg`
Expected: 打印 1 行，形如 `photo_review_20261013.jpg  x=+12.3mm  y=-8.7mm  yaw=+3.1deg  reproj_rms=0.42px`

> 这张图是单张，无法给 RMS 残差；A2 的尺子验证需要**多张**已知位置的图（见 Step 4）。

- [ ] **Step 3: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 视觉测量/脚本/verify_vision.py
git commit -m "feat(vgt): vision verification CLI for manual calibration"
```

- [ ] **Step 4: 【人工步骤 A1–A4，需用户在场】**

按顺序执行，把结果写进 `car_calib.json`：

| 步骤 | 动作 | 写入字段 |
|---|---|---|
| A1 | 量 ID29 相对四轮轴心矩形中心的平面偏置与朝向偏置 | `bias.dx_mm/dy_mm/dtheta_deg` |
| A2 | 把车摆在 3–5 个尺子已知位置，各拍一张，跑 `verify_vision.py` 比对 | `validation.residual_rms_mm`、`validation.residual_rms_deg` |
| A3 | 塞尺复核四轮接地点是否共面 | `coplanar`（解决 `v1_values.json`="是" vs xlsx="否" 的冲突） |
| A4 | 电池带载压降 + 接插件检查 | `battery.sag_v`、`battery.verdict` |

A2 的通过判据：**残差 RMS ≤ 3 mm 且 ≤ 1.5°**。若不达标，先查偏置是否量错，再考虑重拍棋盘格铺到画面四角收紧畸变。

- [ ] **Step 5: 提交标定结果**

```bash
cd /home/hanli/麦轮小车pid
git add 视觉测量/结果/car_calib.json
git commit -m "feat(vgt): record A1-A4 manual calibration results"
```

---

### Task 5: 固件——流式遥测

**Files:**
- Modify: `固件/exp/src/main.cpp`（`streamSample` 约 177-207、`flushRunBuf` 约 196-207、loop 采样段约 385-394）

**Interfaces:**
- Consumes: 无（固件内部）
- Produces: TCP 上 **运行期间**持续收到 `D` 行（约每 50 ms 一次，每次含 5 个样本），而非只在 `DONE` 后一次性收到；`DONE` 仍在结束时发一次。PC 可据此判定"还在跑"vs"跑完了"vs"死了"。

- [ ] **Step 1: 改 `streamSample` 的 'D' 分支**

把「写入 `run_buf` 等 `DONE` 再 flush」改为「写入环形缓冲，累积 ≥5 行就 flush 并清空计数」。保留 `run_buf[32768]` 作为环形缓冲，额外加 `run_pending` 计数。

- [ ] **Step 2: 在 `RUN` 命令处理里清空 `run_pending`**

`zeroEncoders()` 之后加 `run_pending = 0;`

- [ ] **Step 3: 烧录并验证**

```bash
cd /home/hanli/麦轮小车pid/固件/exp && pio run -t upload
```

Run: `cd /home/hanli/麦轮小车pid/实验 && python3 -c "
import car_link, time
c = car_link.CarLink(ip='192.168.1.115')
c.cmd('RUN 0.10 0 0 1500')
t0=time.time(); n=0; first=None
while time.time()-t0 < 3:
    line = c.sock.recv(4096).decode('utf-8','replace')
    if 'D ' in line and first is None: first = time.time()-t0
    n += line.count('D ')
print(f'DONE前收到 {n} 行, 首个D行在 {first:.2f}s')
c.close()
"`
Expected: `n` 远大于 0 且 `first` 明显小于 1.5s（证明是流式而非末尾一次性）。

- [ ] **Step 4: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 固件/exp/src/main.cpp
git commit -m "feat(fw): stream telemetry during RUN instead of buffering to DONE"
```

---

### Task 6: 固件——掉电缓解 + 复位原因回报

**Files:**
- Create: `固件/exp/src/stagger.h`
- Modify: `固件/exp/src/main.cpp`

**Interfaces:**
- Consumes: 无
- Produces:
  - `stagger.h` 暴露 `constexpr bool STAGGER_ENABLED = false;` 与 `constexpr uint32_t STAGGER_STEP_MS = 40;`
  - `M` 与 `RUN` 命令在 `STAGGER_ENABLED` 为真时，按轮序每 `STAGGER_STEP_MS` 抬升一档 duty；为假时行为与现在完全一致
  - `PING`/`ID` 回复末尾附带复位原因：连接时若检测到上次是非正常复位，额外发一行 `RESET <reason>`（`ESP.getResetReason()` 枚举名）
  - 轮速/电流相关的 brownout 状态写入 `RTC_DATA_ATTR` 计数器

- [ ] **Step 1: 创建 `stagger.h`**

放开关常量与 `apply_stagger(int duty[4], uint32_t t0_us)` 声明；`STAGGER_ENABLED=false` 时该函数直接返回（编译期即可被优化掉）。

- [ ] **Step 2: `main.cpp` 接入**

在 `setup()` 里读复位原因存进 `RTC_DATA_ATTR` 变量；`handleLine` 的 `PING` 分支在检测到非 `POWERON`/`SW_CPU_RESET` 的原因时多发一行 `RESET <name>`。

- [ ] **Step 3: 烧录并验证 brownout 可被观测**

```bash
cd /home/hanli/麦轮小车pid/固件/exp && pio run -t upload
```

把车**架空**（四轮悬空），下发一次四轮同转指令以复现 brownout：
Run: `cd /home/hanli/麦轮小车pid/实验 && python3 -c "
import time, car_link
c = car_link.CarLink(ip='192.168.1.115')
c.cmd('RUN 0.10 0 0 1200')
deadline = time.time() + 8
data = b''
while time.time() < deadline:
    try: data += c.sock.recv(4096)
    except OSError: break
    if b'DONE' in data: break
if b'DONE' not in data:
    print('BROWNOUT: 未收到 DONE')
    c.close(); c = car_link.CarLink(ip='192.168.1.115')   # 等重连
    print('重连后读到:', c.cmd('PING'))
    c.cmd('ID')
else:
    print('本次未触发 brownout（电量可能变好）')
c.close()
"`
Expected: 若仍复现，则重连后应能看到 `PING` → `PONG`；再用 `ID` 读取时，连接建立的**首行**应为 `RESET <name>`（`ESP_BROWNOUT` 之类），而非直接是 `MECANUM-ESP32`。

- [ ] **Step 4: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 固件/exp/src/
git commit -m "feat(fw): brownout mitigation switch + reset reason reporting"
```

> ⚠️ `STAGGER_ENABLED` 默认 **false**。是否开启取决于 A4 电池诊断：若电池/接插件本身有问题，软件缓解会掩盖真实电气故障（Spec §8）。

---

### Task 7: wheel_map 双表重建

**Files:**
- Create: `实验/wheelmap_sweep.py`、`实验/wheelmap_gen.py`
- Create: `固件/exp/data/wheelmap_airborne.json`、`固件/exp/data/wheelmap_grounded.json`
- Modify: `固件/exp/src/wheel_map.h`（由生成器产出）

**Interfaces:**
- Consumes: Task 5 的流式遥测；`car_link.CarLink`
- Produces:
  - `wheelmap_sweep.py --mode {air,ground} --dwell 1.2`：逐轮逐占空比采集稳态轮速，写 `固件/exp/data/wheelmap_{mode}.json`，结构 `{"c_1rev":[...], "duty":[...], "omega":[[fl,fr,rl,rr], ...]}`（`omega` 按占空比升序，外层每行一个 duty ×4 轮，正向/反向各一份 `"omega_fwd"`/`"omega_bwd"`）
  - `wheelmap_gen.py --source {air,ground}` → stdout 打印完整 `wheel_map.h` 源码（含 `DutyOmega` 表与 `DZ_POS/DZ_NEG`）

- [ ] **Step 1: 写采集脚本**

占空比序列 `duty ∈ {50,60,80,100,120,140,160,180,200,220,240}`（与旧表同档位，便于对比）。每档：正转 1.2 s + 反转 1.2 s，取后 0.6 s 的编码器平均速率换算 `rad/s`（`ω = Δcnt/s · 2π / C_1rev`）。轮速低于 0.5 rad/s 的档位记 0（死区内）。`C_1rev` 用 `固件/历史/` 已标定的 `[1276,1270,1276,1274]`，但脚本要支持 `--c1rev` 覆盖。

- [ ] **Step 2: 架空采集并检查离散度**

把车抬起悬空（当前状态），Run: `cd /home/hanli/麦轮小车pid/实验 && python3 wheelmap_sweep.py --mode air`
Expected: 生成 `wheelmap_airborne.json`；脚本末尾打印四轮同占空比的离散度，应显著低于旧架空实测的 13.5%（若没改善，说明是电机个体差异，需要在 wheel_map.h 里逐轮补偿而不是共用一套表）。

- [ ] **Step 3: 落地采集**

把车放到场地内四轮着地，Run: `cd /home/hanli/麦轮小车pid/实验 && python3 wheelmap_sweep.py --mode ground`

- [ ] **Step 4: 生成并替换 wheel_map.h**

```bash
cd /home/hanli/麦轮小车pid/实验 && python3 wheelmap_gen.py --source ground > /tmp/opencode/wheel_map_new.h
```
人工检查后替换 `固件/exp/src/wheel_map.h`，保留 `MIN_DUTY=110` 与 `invert[4]`，并加注释标明这是 ground 套；air 套的表存为 `wheel_map_airborne.h` 供诊断用。

- [ ] **Step 5: 烧录并验证速度跟踪**

```bash
cd /home/hanli/麦轮小车pid/固件/exp && pio run -t upload
```
下发 `RUN 0.15 0 0 2000`，用遥测算出的实际 `vx` 应落在指令 0.15 m/s 的 ±15% 内（旧表偏差约 50%）。

- [ ] **Step 6: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 实验/wheelmap_sweep.py 实验/wheelmap_gen.py 固件/exp/data/ 固件/exp/src/wheel_map.h
git commit -m "feat: rebuild wheel_map duty->omega tables (airborne + grounded)"
```

---

### Task 8: 相机会话（持久句柄 + 锁曝光/对焦）

**Files:**
- Create: `实验/vgt/__init__.py`、`实验/vgt/camera.py`
- Test: `实验/vgt/tests/test_camera_settings.py`（纯常量与参数映射，不需硬件）

**Interfaces:**
- Consumes: 无
- Produces:
  - `class CameraSession`：上下文管理器；`__enter__` 打开 `/dev/video0` 并**关闭自动曝光与自动对焦**（`cv2.CAP_PROP_AUTO_EXPOSURE=0`、`cv2.CAP_PROP_AUTOFOCUS=0`，显式设 `EXPOSURE=166`、`FOCUS=275`，取自 `视觉测量/结果/camera_props.json`）
  - `.grab() -> np.ndarray`：读一帧（BGR），失败抛 `RuntimeError`
  - `.save(path: str) -> str`：抓帧存盘并返回路径
  - `CAMERA_DEV = "/dev/video0"`、`LOCKED_EXPOSURE = 166`、`LOCKED_FOCUS = 275`

> 现有 `capture_photo.py` 每次开关设备、且不锁参数（`camera_props.json` 显示 `AUTO_EXPOSURE=3`、`AUTOFOCUS=1`）。自动对焦在运动中会"游走"导致检测失败——锁死是 Review Focus #1 的前提。

- [ ] **Step 1: 写常量映射测试**

```python
from vgt.camera import CAMERA_DEV, LOCKED_EXPOSURE, LOCKED_FOCUS, apply_locked_settings

def test_locked_settings_disable_auto():
    class FakeCap:
        def __init__(self): self.props = {}
        def set(self, k, v): self.props[k] = v
    cap = FakeCap()
    apply_locked_settings(cap)
    import cv2
    assert cap.props[cv2.CAP_PROP_AUTO_EXPOSURE] == 0
    assert cap.props[cv2.CAP_PROP_AUTOFOCUS] == 0
    assert cap.props[cv2.CAP_PROP_EXPOSURE] == 166
    assert cap.props[cv2.CAP_PROP_FOCUS] == 275
```

- [ ] **Step 2: 跑测试确认失败 → 实现 → 跑测试确认通过**

Run: `cd /home/hanli/麦轮小车pid/实验 && python3 -m pytest vgt/tests/test_camera_settings.py -v`
Expected: FAIL → 实现 `vgt/camera.py` → PASS

- [ ] **Step 3: 验证锁参数后 5 码仍能全检出**

Run: `cd /home/hanli/麦轮小车pid && python3 -c "
from vgt.camera import CameraSession
from vgt.markers import detect_markers
from vgt import calib_store
with CameraSession() as cam:
    for i in range(5):
        p = detect_markers(cam.grab(), calib_store.load_calib())
        print(i, sorted(p))
" 2>&1 | tail -6`
Expected: 5 行都是 `[20, 21, 22, 23, 29]`

- [ ] **Step 4: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 实验/vgt/
git commit -m "feat(vgt): persistent camera session with locked exposure and focus"
```

---

### Task 9: 实验矩阵裁剪 + 编码器/IMU 解算

**Files:**
- Create: `实验/vgt/matrix.py`、`实验/vgt/odometry.py`
- Test: `实验/vgt/tests/test_matrix.py`、`实验/vgt/tests/test_odometry.py`

**Interfaces:**
- Consumes: 无
- Produces:
  - `matrix.EXPERIMENTS: list[Experiment]`，`Experiment` 为 namedtuple `(id, action, desc, vx, vy, om, duration_s, repeats)`；覆盖：直行 ±200/±150 mm、**横移 ±80/±50 mm**、斜行 ±120 mm、**旋转 ±30°/±45°**，每条 `repeats >= 3`
  - `matrix.assert_within_field(exp) -> None`：超出场地方程（横向 235 mm / 纵向 103 mm 余量）抛 `ValueError`
  - `odometry.parse_telemetry(lines: list[str]) -> list[dict]`：把固件 `D` 行（`D <t_ms> <pwm0..3> <enc0..3> <roll> <pitch> <yaw>`）解析成 `{"t_ms": int, "pwm": [4], "enc": [4], "rpy": [3]}`；非法行跳过
  - `odometry.integrate_encoders(samples, *, c_1rev, sign, l_xy_over_r, r_eff_mm) -> np.ndarray (N,3)`：`samples` 为 `parse_telemetry` 的输出，返回 `(x_mm, y_mm, yaw_deg)`
  - `odometry.imu_yaw_series(samples) -> np.ndarray (N,)`

- [ ] **Step 1: 写矩阵约束测试**

```python
import math
from vgt import matrix

def test_no_experiment_exceeds_field_clearance():
    for e in matrix.EXPERIMENTS:
        matrix.assert_within_field(e)          # 不抛异常即通过

def test_all_rotations_are_at_most_45_deg():
    for e in matrix.EXPERIMENTS:
        if e.action == "rotate_in_place":
            assert abs(e.om) * e.duration_s * 180/math.pi <= 45.0 + 1e-9

def test_strafe_step_is_at_most_80mm():
    for e in matrix.EXPERIMENTS:
        if e.action == "strafe":
            assert abs(e.vy) * e.duration_s * 1000 <= 80.0 + 1e-9

def test_straight_distance_is_at_most_200mm():
    for e in matrix.EXPERIMENTS:
        if e.action == "straight":
            assert abs(e.vx) * e.duration_s * 1000 <= 200.0 + 1e-9

def test_every_experiment_repeats_at_least_three_times():
    assert all(e.repeats >= 3 for e in matrix.EXPERIMENTS)
```

- [ ] **Step 2: 写解算测试**

```python
import numpy as np
from vgt import odometry

def test_parse_telemetry_roundtrip():
    s = odometry.parse_telemetry(["D 10 -57 53 -73 49 100 200 300 400 1.0 2.0 3.0"])
    assert s[0]["t_ms"] == 10 and s[0]["enc"] == [100,200,300,400] and s[0]["rpy"] == [1.0,2.0,3.0]

def test_parse_telemetry_skips_junk():
    s = odometry.parse_telemetry(["OK RUN pwm=110,110,110,110", "", "D 0 0 0 0 0 1 2 3 4 0 0 0"])
    assert len(s) == 1

def test_integrate_pure_forward_uses_both_sides():
    c1, sign, lxy_r = [1276.0]*4, [-1,1,-1,1], 0.12825/0.0335
    r_eff, n = 33.5, 400          # 33.5mm 半径，400 计数/轮
    s = [{"t_ms": i*10, "enc": [sign[i]*n//4 for i in range(4)]} for i in range(41)]
    out = odometry.integrate_encoders(s, c_1rev=c1, sign=sign, l_xy_over_r=lxy_r, r_eff_mm=r_eff)
    assert out[-1][0] > 0 and abs(out[-1][1]) < 0.5 and abs(out[-1][2]) < 0.5

def test_equal_left_right_gives_no_yaw():
    c1, sign, lxy_r = [1276.0]*4, [-1,1,-1,1], 0.12825/0.0335
    s = [{"t_ms": i*10, "enc": [sign[i]*1000 for i in range(4)]} for i in range(41)]
    out = odometry.integrate_encoders(s, c_1rev=c1, sign=sign, l_xy_over_r=lxy_r, r_eff_mm=33.5)
    assert abs(out[-1][2]) < 0.5
```

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/hanli/麦轮小车pid/实验 && python3 -m pytest vgt/tests/test_matrix.py vgt/tests/test_odometry.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vgt.matrix'`

- [ ] **Step 4: 实现**

`matrix.py`：按 Global Constraints 的四个上限生成实验列表。`assert_within_field` 用场地半长 252.5 / 半宽 125.9 与车体半包络 135 × 74.5 算出净余量，位移超限抛 `ValueError`。

`odometry.py`：`integrate_encoders` 逐步用逆运动学 `w_i = (vx ∓ vy ∓ LXY·ω)/R` 的等价形式——按各轮增量算 `(vx, vy, ω)` 再积分；`sign` 用于把累计计数转成带符号位移（单相编码器不辨向，此处靠约定）。

- [ ] **Step 5: 跑测试确认通过**

Run: `cd /home/hanli/麦轮小车pid/实验 && python3 -m pytest vgt/tests/ -v`
Expected: 全部 PASS

- [ ] **Step 6: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 实验/vgt/
git commit -m "feat(vgt): field-clamped experiment matrix + encoder/IMU odometry"
```

---

### Task 10: 视觉闭环归位

**Files:**
- Create: `实验/vgt/harness.py`（归位部分）

**Interfaces:**
- Consumes: Task 8 `CameraSession`、Task 3 `body_pose_in_field`、Task 9 `odometry`
- Produces: `class Harness`：`__init__(car_link, camera, calib, home_xy=(0.0, 0.0))`、`.locate() -> tuple[float,float,float] | None`、`.drive_to(x_mm, y_mm, tol_mm=12.0, max_iter=12) -> dict`（返回 `{"reached": bool, "iters": int, "final": (x,y,yaw)}`）

- [ ] **Step 1: 实现 `locate()`**

抓一帧 → `detect_markers` → 若 4 边界码缺任一则返回 `None` → 否则 `build_field_frame` + `body_pose_in_field`。

- [ ] **Step 2: 实现 `drive_to()`**

P 控制：目标误差 `(dx,dy)`（场地系 mm）→ 换算到车体系 `(vx, vy)`（车体 yaw 朝向由 `locate()` 给出）→ 下发 `RUN vx vy 0 400` → 等 0.5 s → 重新 `locate()` → 收敛或超 `max_iter`。

- [ ] **Step 3: 加边界码漂移复核（Review Focus #2）**

每次 `locate()` 时比对 4 码的 `marker_px` 与 `FieldFrame` 建立时的基准；任一码位移 > 15 px 则 `drive_to` 立即中止并抛 `BoundaryMarkersMoved`。

- [ ] **Step 4: 真机验证**

把车放在场地内偏离原点 ~100 mm 处，跑 `drive_to((0,0))`，`reached` 应为 `True` 且 `final` 距原点 < 12 mm。

- [ ] **Step 5: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 实验/vgt/harness.py
git commit -m "feat(vgt): vision closed-loop homing with boundary drift check"
```

---

### Task 11: 单次实验编排（三点拍照 + 遥测 + 真值）

**Files:**
- Create: `实验/vgt/harness.py`（`run_trial` 部分）

**Interfaces:**
- Consumes: Task 10 `Harness`、Task 5 流式遥测
- Produces: `Harness.run_trial(exp) -> dict`，返回
  ```
  {"ok": bool, "reason": str, "vision": {"start": (x,y,yaw), "mid": (x,y,yaw)|None,
                                         "end": (x,y,yaw)},
   "vision_delta": {"dist_mm": float, "yaw_deg": float},
   "odometry_delta": {"dist_mm": float, "yaw_deg": float},
   "samples": list[dict], "n_samples": int}
  ```
  `ok=False` 时 `reason ∈ {"boundary_moved","no_vehicle_marker","deadline_timeout","board_reset","mid_capture_failed"}`

- [ ] **Step 1: 实现时序**

1. `locate()` 失败 → `ok=False, reason="boundary_moved"`
2. 起点：等车静止 0.3 s → 抓帧 → 解位姿 = `start`
3. 下发 `RUN vx vy om ms`，同时开始读流式遥测
4. 收到 `D` 行里 `t_ms` 首次 ≥ `ms/2` 时抓一帧 = `mid`（**抓失败不致命**，置 `None`）
5. 收到 `DONE` → 等 0.4 s 沉降 → 抓帧 = `end`
6. 无 `DONE` 且 5 s 内无数据 → 重连检查 `RESET` 行 → `ok=False, reason="board_reset"`（Review Focus #3）
7. 任一关键帧解不出 ID29 → `reason="no_vehicle_marker"`

- [ ] **Step 2: 加中点降级测试**

```python
def test_mid_capture_failure_degrades_to_two_points(monkeypatch):
    """Review Focus #1：中点抓失败不致命，降级为两点"""
```
用假 `CameraSession`（第 3 次 `grab()` 抛异常）+ 假 `CarLink`（返回 3 个 D 行 + DONE），断言 `ok=True`、`vision["mid"] is None`、`vision_delta` 仍正确。

- [ ] **Step 3: 跑测试确认通过**

Run: `cd /home/hanli/麦轮小车pid/实验 && python3 -m pytest vgt/tests/test_harness.py -v`

- [ ] **Step 4: 真机验证**

跑一条 `forward 200mm`，`vision_delta.dist_mm` 与 `odometry_delta.dist_mm` 的相对差应 < 20%。

- [ ] **Step 5: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 实验/vgt/harness.py 实验/vgt/tests/test_harness.py
git commit -m "feat(vgt): single-trial orchestration with start/mid/end ground truth"
```

---

### Task 12: 对比入表

**Files:**
- Create: `实验/vgt/compare.py`、`实验/run_all_experiments.py`
- Modify: `实验/实验数据记录_build.py`（新增视觉列）、`实验/实验数据记录.xlsx`（由脚本追加写入）

**Interfaces:**
- Consumes: Task 11 的 trial 结果
- Produces:
  - `compare.write_trial(trial: dict, exp) -> None`：向 `实验数据记录.xlsx` 的 `data` 表追加样本行 + 一行汇总行
  - `run_all_experiments.py [--only ID] [--dry]`：批量执行矩阵，逐条打印 `视觉 vs 编码器` 对比表

新增列（在既有 31 列之后）：`vision_start_x`、`vision_start_y`、`vision_start_yaw`、`vision_end_x`、`vision_end_y`、`vision_end_yaw`、`vision_dist_mm`、`vision_yaw_deg`、`odo_dist_mm`、`odo_yaw_deg`、`dist_diff_pct`、`yaw_diff_deg`、`trial_ok`、`trial_reason`

- [ ] **Step 1: 先改生成器再加列**

在 `实验数据记录_build.py` 的 `COLS` 末尾追加上述 14 个列名（保持"改生成器不直接改表"的既有约定），重跑生成器。

- [ ] **Step 2: 实现 `compare.write_trial`**

用 openpyxl `append`；`trial_ok=False` 的实验仍写行但 `trial_reason` 标注原因，便于事后筛除。

- [ ] **Step 3: 实现批量入口**

遍历 `matrix.EXPERIMENTS` × `repeats`，每条前 `drive_to(home)`，跑 `run_trial`，写表，打印实时对比。每条结束 `car.stop()`。

- [ ] **Step 4: 全量跑一轮**

Run: `cd /home/hanli/麦轮小车pid/实验 && python3 run_all_experiments.py 2>&1 | tail -40`
Expected: 每条实验一行，含 `vision_dist_mm / odo_dist_mm / dist_diff_pct`；失败条目标注原因而非崩溃。

- [ ] **Step 5: 提交**

```bash
cd /home/hanli/麦轮小车pid
git add 实验/vgt/compare.py 实验/run_all_experiments.py 实验/实验数据记录_build.py 实验/实验数据记录.xlsx
git commit -m "feat(vgt): vision-vs-odometry comparison + batch experiment runner"
```

---

### Task 13: 文档与参数表

**Files:**
- Create: `小车档案.md`、`docs/superpowers/analysis/error-mece.md`
- Modify: `交付物/.麦轮小车参数测量清单.ref/build_v4.py`（新增视觉测量章节 + 修正共面冲突），重跑生成 `交付物/麦轮小车参数测量清单.xlsx`

**Interfaces:**
- Consumes: Task 4 标定结果、Task 7 双表、Task 12 全量数据
- Produces: 三个文档 + 更新后的参数表

- [ ] **Step 1: 写 `小车档案.md`**

内容取自 Spec §2：硬件、引脚、轮序定义与固件索引对应、车载码位置（**FL 与 RL 之间的左甲板，30 mm**）、场地边界码（25 mm）、场地尺寸、供电。标注"长期维护，每次硬件变动即更新"。

- [ ] **Step 2: 写 `docs/superpowers/analysis/error-mece.md`**

把 Spec §5 的 E1–E16 落成独立文档，每条标注：现象、机理、当前证据、本期是否处理、责任人/后续动作。补上 Task 12 全量数据算出的**各项误差实测幅度**。

- [ ] **Step 3: 更新参数表生成器**

在 `build_v4.py` 加一个「视觉测量」sheet：ID29 偏置、PnP 重投影 RMS、A2 验证残差、场地系定义、PnP 与桌面单应的交叉校验差值。修正"一次性检查"sheet 里共面项（按 A3 结论）。

- [ ] **Step 4: 重跑生成器并核对**

Run: `cd /home/hanli/麦轮小车pid/交付物/.麦轮小车参数测量清单.ref && python3 build_v4.py`
Expected: 生成的 xlsx 含新 sheet，无公式（本机无 LibreOffice 的既有约束）

- [ ] **Step 5: 更新 HANDOFF.md §3.6 并提交**

```bash
cd /home/hanli/麦轮小车pid
git add 小车档案.md docs/superpowers/analysis/error-mece.md 交付物/
git commit -m "docs: car profile, MECE error analysis, parameter sheet update"
```

---

## 执行顺序与并行

```
Task 1  ──► Task 2 ──► Task 3 ──► Task 4(人工) ──► Task 10 ──► Task 11 ──► Task 12 ──► Task 13
                        └──────────► (视觉核心，可离线跑，无硬件依赖)
Task 5  ──► Task 6                                    
        └──────► Task 7 ────────────────────────────► (固件线，与视觉线可并行)
Task 8  ──► Task 9 ──────────────────────────────► (相机+数学，可并行)

汇合点: Task 10 需要 Task 4(标定) + Task 5(流式遥测)
```

**建议并行**：视觉线（Task 1–4）与固件线（Task 5–7）可同时推进，唯一汇合点是 Task 10。