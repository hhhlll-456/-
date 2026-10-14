# HANDOFF — 麦轮小车 PID 调参项目

> 更新时间：2026-10-13（**新方向确立**：视觉 ground truth 自动化实验；spec + plan 已就绪，见 §0）
> 项目根目录：`/home/hanli/麦轮小车pid`
>
> ⚠️ **下一 session 请从这里读起** → §0，然后按 `docs/superpowers/plans/2026-10-13-vision-ground-truth-experiments.md` 执行。

---

## 0. 当前主线：视觉 ground truth 自动化实验（2026-10-13 定）

**目标**：小车在 4 个固定边界 ArUco 码围成的区域内运动，用**车载码 ID29 + 摄像头**作 ground truth 自动测**位移 mm / 转角 deg**，取代 22 条实验的手工尺子测量。

**两份必读文档**（新 session 直接读这两份即可接手）：

| 文档 | 作用 |
|---|---|
| `docs/superpowers/specs/2026-10-13-vision-ground-truth-experiments-design.md` | 设计：目标、MECE 误差分析（E1–E16）、10 条已锁定决策 |
| `docs/superpowers/plans/2026-10-13-vision-ground-truth-experiments.md` | 实施计划：13 个 task，含测试代码与验证命令 |

**关键已确认事实**（细节见 spec §2）：
- 车载码 **ID29 = 30 mm**，贴在**左甲板 FL 与 RL 之间的上方**
- 边界码 **ID 20/21/22/23 = 各 25 mm**，TL/TR/BR/BL，固定不动
- 场地 ≈ **505 × 252 mm**；车体 270 × 149 mm → 横向余量仅 103 mm
- **旋转上限 45°**（转 90° 后车宽 270 > 短边 252）；直行 ≤200 mm；横移 ≤80 mm
- **视觉用 `solvePnP`**（不是桌面单应）——码尺寸已知，PnP 解出 ID29 的真实高度，消除悬空 55–75 mm 造成的 10–17% 尺度误差
- IMU 维持 6 Hz / 9600 波特（**决定不改**），仅作粗略交叉校验

**10 条锁定决策**：D1 solvePnP｜D2 yaw=场地系(图像up=0°)｜D3 起/中/终三点｜D4 PC 驱动｜D5 先测电池再写软件｜D6 视觉闭环归位｜D7 接受一次性人工标定(A1–A4)｜D8 矩阵按场地裁剪、只做≤45°旋转｜D9 先建新固件再对比｜D10 wheel_map 架空+落地双表

**执行方式**：subagent-driven（每 task 一个新 subagent 实现 + 一个新 reviewer 复核）。
**并行**：视觉线 Task 1–4 与固件线 Task 5–7 互不依赖，可并行；唯一汇合点是 Task 10（视觉闭环归位）。

---

### ⚡ 当前进度（2026-10-14 21:40 更新 —— **新 session 从这里接手**）

**完整台账在** `.superpowers/sdd/2026-10-13-vision-ground-truth-experiments/progress.md`
（含全部裁决 R1–R58、每个 task 的 fix 轮记录、以及末尾的「🛑 停点」一节 —— **先读那一节**）。

| Task | 状态 | commit |
|---|---|---|
| 1–3 | ✅ 完成 | `d8850c4`…`42c7922` |
| **4** 人工标定 | ✅ **完成**（A1 偏置 + A3 共面；A2 单点结案；**A4 由用户取消**） | `df5e08a` |
| **5** 流式遥测 | ✅ 代码 + 复核批准 + **上机验证通过**；另修了一个 ~1s 遥测黑屏 | `4aa908f`、`e2600b7` |
| **6** 掉电缓解 | ✅ 完成（`STAGGER_ENABLED` 建成即 `true`；brownout 5 次未复现） | `99b1040` |
| 8 相机会话 | ✅ 完成 + Critical（陈旧帧）已修 | `7d0a004`、`7b8244b` |
| 9 实验矩阵 + 里程计 | ✅ 完成，复核**批准** | `349e6d0` |
| **7** wheel_map 双表 | ⚠️ **架空表已采；落地表没采成** | `92e1c70` |
| 10–13 | ⏳ 未开始 | — |
| `live_field.py`（操作者工具） | ✅ 四轮修复完成，**第 4 轮复核欠着** | `dede3e4` |

**小车状态**：**没电**、USB 未插、四轮着地、在场地里。**任何碰车的事（烧录、采集、真机验证）都要先充电。**

**不依赖车、现在就能做的**：
1. **5 项复核欠账**（全部纯代码，见台账「复核欠账」表）：`live_field` 第 4 轮、Task 5 黑屏修复、Task 6、Task 8 fix round 1、Task 7
2. **Task 10 / 11 的代码**（可写可测，真机验证等车）
3. **Task 13 的文档与参数表**

**⚠️ 三条必须带进后续任务**（台账末尾有全文）：
- Task 11/12 的对比要加**有符号**判据，只比 `dist_mm` 抓不住"平移符号翻转"
- `assert_within_field` 的 `(0.0,0.0)` 默认值会静默走宽松分支 —— Task 12 必须传**目标执行起点**
- **`MIN_DUTY=110` 钳位**是"速度跟踪差 318%"的主因，换新表也修不了；Step 5 过不了时该动的是 `MIN_DUTY`（**先问用户**）

**⚠️ 固件还有 3 个未修缺陷**（都不在已完成的 task 授权范围内，台账 R58 有全文）：
`STREAM 1` 后 TCP 堵死无法自救（只能断电）、粘连行、僵尸 client。

---

> 更新时间：2026-10-13（视觉测量：棋盘格内参标定完成 + Skill 更新；见 §3.5）。
> 新 session 读此文件即可接手。项目根目录：`/home/hanli/麦轮小车pid`
>
> **最近一次 session 只动了「视觉测量」线**：把镜头内参从名义值换成棋盘格实测值，并更新了
> `plane-metric-calibration` skill。下一 session 最自然的动作是 §4 的视觉项**「把相机 ground truth
> 接入 22 条实验管道」**；其余主线（补人工测量、数据分析、z_c、URDF）与上次 session 相比无变化。

---

## 1. 最终目标

四轮麦轮小车按指令精确运动（直行、定角旋转如 30°）。为此需要：

- **A** 运动学方程（输入 编码器+IMU+目标 → 输出 4 轮 PWM）
- **B** 质心报告（平面 x_c/y_c + 高度 z_c）
- **C** MECE 误差因素分析
- 用 **URDF** 做仿真，最后标定 + 调 PID

**硬件**
- 4 × 直流减速电机 `TK-ENC-TT-26B V1.2`（带编码器，仅接 A 相）
- IMU：`GY-25T`（Serial2，RX=37，TX=38 备用，9600 8N1）
- 主控：**ESP32 经典版**（非 STM32）；Wi-Fi 链路（SSID `B1222`）
- 引脚（实测）：电机 FL{13,15} FR{12,14} RL{27,26} RR{25,33}；编码器 FL32 FR34 RL35 RR36

## 2. 目录地图（2026-10-12 重组后）

```
交付物/    麦轮小车参数测量清单.xlsx（主交付物）+ 3 份测量文档 + 测量示意图/ + capture_photos.sh
           .麦轮小车参数测量清单.ref/  ← Excel 生成脚本（改表只能改 build_v4.py 再重生成）
实验/      实验数据记录.xlsx（data 3894 行 / 历史数据 910 行）+ 管道脚本 + 原始日志/
固件/      exp/（当前实验固件）wifi/（链路）caltable/（标定工具）历史/（direction encoder monitor kpwm）
视觉测量/  脚本/（拍照+检测）照片/（photo5*.jpg 等）结果/（可视化）码表/（aruco_markers.json/md）
资料/      图纸/（底盘 DWG 导出图）评审/（4 份 AI 评审）Grok历史记录.md GY25T/ imu/ encoder/ wheelmap_host/
```

隐藏目录 `.workbuddy/`（工具记忆）勿动。

## 3. 已完成的进展（3.1–3.4 为 2026-10-12；3.5 为 2026-10-13）

### 3.1 固件标定链（已全部烧录实测，源码归档在 固件/历史/）

direction → encoder → monitor → kpwm → caltable，测得：

- 单圈计数 **C_1rev = 1276 / 1270 / 1276 / 1274**（FL/FR/RL/RR）
- 起转死区 **DZ_POS = 40/48/56/44**，**DZ_NEG = 52/48/68/60**
- 前进方向符号：**FL/RL = -1，FR/RR = +1**
- 原始数据：`实验/原始日志/dense_log.txt`（密集占空比扫描）、`enc_log.txt`（手动转圈）

### 3.2 实验固件与管道（当前主线）

- **`固件/exp/`**：Wi-Fi 控制 + 100Hz 遥测。TCP 8266 / UDP 发现 8267 / OTA hostname `mecanum`；
  `RUN <vx> <vy> <om> <ms>` 跑一次实验并回传 `'D t pwm0-3 enc0-3 roll pitch yaw'`
- 运动学（X 型，`exp/src/main.cpp` 的 `bodyToPWM`，LXY=0.12825 m）：
  `w0=(vx−vy−LXY·ω)/R`，`w1=(vx+vy+LXY·ω)/R`，`w2=(vx+vy−LXY·ω)/R`，`w3=(vx−vy+LXY·ω)/R`
- `exp/src/wheel_map.h`：duty↔ω 查表插值 + 死区 + **MIN_DUTY=110**（横移低速起转修正，必需）
- 起步 kick：KICK_DUTY=105 / 150 ms（突破静摩擦）
- PC 端：`实验/car_link.py`（UDP 发现 + TCP 命令）→ `run_experiment.py <id>` 执行
  `experiments.py` 的 **22 条实验方案**并写表；`update_plan_sheet.py` 只刷新清单表
- **状态：22 条实验已全部执行完，数据已入表**
  - `manual_distance` / `manual_angle`：22 条全填
  - `manual_final_x`：8 条（10,12,14,18–22）；`manual_final_y`：13 条；`manual_final_yaw`：未填
  - 补填方法：`run_experiment.py --fill <id> --distance <m> --angle <deg>` 或直接编辑 xlsx

### 3.3 Excel 迁移

`实验数据记录.xlsx` 已按 22 条新方案重编号；旧方案数据在「历史数据」表。

### 3.4 视觉测量（新方向：位置 ground truth）

- USB 摄像头 `/dev/video0`（MJPG 1920x1080@30，连续 AF 工作正常，别锁死焦点）
- 场地 4 个边界 ArUco 码（DICT_4X4_50：ID20 TL / 21 TR / 22 BR / 23 BL）+ 车载码 29
- 码表：`视觉测量/码表/aruco_markers.json`（含参考像素坐标 + 归一化场地约定），参考照片 `照片/photo5_final.jpg`
- 检测经验已沉淀为全局 skill **`aruco-detection`**（opencode / Claude Code / Pi 三平台已安装）
- 关键结论：小码（~73 px）需**多参数融合检测**（默认参数会漏 ID23）；ID=23 比特级验证完好；
  2x 参数 `win=7..23/8 c=5` 在 1x 全图检出 5/5 仅 ~25 ms

### 3.6 架空诊断：IMU / 电机 / 编码器 / 供电（2026-10-13）

- 场景：小车抬起四轮悬空，USB 连电脑（CH340 串口 + 供电），Wi-Fi TCP 命令
- 脚本 `实验/diag_air2.py`，发现全文 `实验/原始日志/DIAG_FINDINGS_20261013.md`
- **IMU 静态精度优秀**（yaw 漂移 -0.06°/min），但**有效更新率仅 ~6Hz**（9600 波特瓶颈）
- **编码器无假计数**；单相不能辨向（已知）
- **四轮同占空比空载转速离散 13.5%**（FL 14.96 / RR 17.13 rad/s @ PWM120）
- **wheel_map.h 表整体高估 ~50%**，逐轮补偿比例不足
- **Brownout：四轮同时驱动时 ESP32 复位**（USB 供电带不动 4 电机涌入电流）
- 结论：**不是传感器问题** — 是开环 RUN + 映射 stale + 供电不足

### 3.5 视觉测量：棋盘格内参标定（2026-10-13 完成）

- **背景**：`calib_distortion.py`（= skill 里的 `plane_calib.py`）原用**名义焦距** `F_NOMINAL=1400`，
  因为 4 个边界码无法定 K。本次用棋盘格**实测**内参替换它。
- **棋盘规格**：图注 "Chessboard 9x7, square = 20 mm" → **9×7 方格 = 8×6 内角点**，方格 20 mm，
  `PAT=(8,6)`。⚠️ 传给检测器的是**内角点数**，不是方格的 9x7。
- **采集**：15 个强倾斜姿态（板子离台、三维倾斜），`视觉测量/照片/calib_chess_v3/chess_01..15.jpg`，
  每帧 48 个内角点全检出。
- **脚本**：`~/.claude/skills/plane-metric-calibration/scripts/chess_intrinsics.py`
  （`findChessboardCornersSB` + `cornerSubPix` + `calibrateCameraExtended`；LOO-CV 选模型 + 物理守门）
- **结果**（针孔模型，畸变=0）：**fx=fy=1460.76、cx=1019.4、cy=592.8、RMS 1.32 px**（LOO-CV 1.29 px）
  → `视觉测量/结果/calib_intrinsics_chess.json`
- **焦距不确定度**：LOO std ~9 px；**系统范围 1414–1516 px**（主点钉中心 ↔ 主点自由+全畸变，~7%）。
  引用 fx 时给这个范围，别报 0.1 px 的假精度。
- **接入**：`calib_distortion.py` 新增 `_load_measured_K()` / `make_K()`，**优先读实测 K，缺失回退 1400**
  （防御式，测量路径不硬依赖棋盘格结果）。重跑场地标定：px/mm 2.8209→2.8095、残余 6.24→5.92 px——
  **几乎不变**。这是**预期内的确认**：畸变≈0、场地映射由边界码单应矩阵 `H_px2mm` 决定，K 只影响
  去畸变与 FOV 报告；不要把"换了 K"说成"改善了残余"。
- **关键判据（务必保留）**：5 参数模型 LOO-CV"胜出"~10%，却给出 k2≈1（≈30% 边角畸变，物理不可能）
  → **取针孔**。根因：棋盘始终居中 → 高阶畸变与主点不可辨识（CV 的改善是拟合噪声）。
- **Skill 更新**：`plane-metric-calibration` 已加棋盘格内参能力 + 本次所有坑（见 §5）；
  `索引.md` 已登记 `calib_intrinsics_chess.json`、`calib_chess_v3/`、skill 脚本。

## 4. 下一步 TODO（建议优先级）

**视觉测量线（本 session 刚把标定链收尾，天然衔接 —— 下一 session 首选）**

1. **把相机 ground truth 接入 22 条实验管道**：用 `视觉测量/脚本/detect5.py` 或 skill 的
   `measure_motion.py`，对每条实验的照片/录像算出小车的**平面位移 (mm) 与转角 (deg)**，
   与编码器/IMU 航迹对比，标定运动学模型。（= 旧 TODO #6）
2. （可选，仅当需要更准的畸变/主点）重拍棋盘格，把板子**铺到画面四角**：本次板子始终居中，
   高阶畸变与主点不可辨识，故被迫取针孔模型；覆盖边角才能把它们定出来。当前畸变≈0，一般够用。
3. （可选）场地标定 `usable=false` 是相机轻微**俯仰**造成的物理残差，非代码 bug；测量请用
   `H_px2mm`（已在 `calib_report.txt` 说明），或物理调平相机后复标。

**全项目主线（与上次 session 相同）**

4. 补 `manual_final_x / final_y / final_yaw`（人工测量收尾）
5. 分析 22 条实验数据：航向漂移率 / 速度标定 / 横移耦合 → 回填 `交付物/麦轮小车参数测量清单.xlsx`
6. **质心高度 z_c**（仍缺；倾斜称重或双线悬吊，方法见 `交付物/麦轮小车详细测量步骤.md`）
7. 四轮表剩余空项（R_eff、C_m、τ_m、k、toe）+ 整车 I_zz
8. 运动学方程成文 → MECE 误差 PDF → URDF（**非共面**建模：前/后轮距 120/118 mm）

## 5. 环境坑（重要，别重复踩）

- 平台是 ESP32 经典版，不是 STM32
- Excel 不放公式（本机无 LibreOffice）：改表一律改 `build_v4.py` 后重生成，勿直接改 xlsx
- 根分区长期紧张（现 ~680 MB 空闲）；pip 缓存指到工作区；`/tmp` 是 10 MB tmpfs
- 串口 `/dev/ttyACM0`（备用）需非沙箱访问；PlatformIO 烧录：`pio run -t upload`（upload_port 已配）
- SVG→PNG 用 cairosvg（ImageMagick MSVG 会丢旋转/椭圆），文本显式 `font-family="Noto Sans SC"`
- `pkill -f live_aruco` 会误杀 shell，用 `pkill -f "[l]ive_aruco"` 防自匹配
- 旧文档（评审材料/Grok历史）中的路径引用保留原文，未随重组更新
- 视觉/棋盘格标定坑（本 session 全部踩过，细则见 skill `plane-metric-calibration` 的
  `references/failure-modes.md` §J–N）：
  - 棋盘图注 "9x7" = 9×7 **方格** → 检测器要传 **8×6 内角点数**，传错则每帧都检不到
  - 板子被画面**边缘裁剪** → 表现为"检不到"或**误检出小网格**（如 (5,4)）；必须留边距、看返回的网格尺寸
  - 非交互连拍会产出**几何完全相同**的画面（板子没动）→ 必须**交互式**采集（人工移板、你按键存图）
  - `cv2.undistortPoints` 在异常畸变系数下**静默返回输入**（不报错）→ 勿用它做畸变 sanity check，
    直接按公式算径向因子 `1+k1r²+k2r⁴+k3r⁶`
  - 棋盘始终居中 → 高阶畸变/主点**不可辨识**，交叉验证的改善是拟合噪声 → 取针孔
  - 本机 `cv2.imshow` 可用（DISPLAY=:0，Wayland 下也 OK），做实时取景/存图 UI 没问题

## 6. 待用户确认

1. **视觉线是否立刻做「相机 ground truth 接入 22 条实验管道」**（§4 视觉项 1）？还是先做别的？
2. 是否需要**重拍棋盘格**（把板子铺到画面四角）以收紧畸变/主点？当前畸变≈0，多数量够用，属可选。
3. 相机是否**物理调平**以消除场地标定的 `usable=false`（透视残差）？
4. z_c 测量方法（倾斜称重 or 悬吊）与时机
5. 布局 X 型 / O 型最终判定（当前代码按 X 型实现）
6. m = 0.697 kg 是否含电池与配重
7. `encoder/esp32_encoder_test.ino`（旧 Arduino IDE 版）功能已被 PlatformIO 版覆盖，可弃？
