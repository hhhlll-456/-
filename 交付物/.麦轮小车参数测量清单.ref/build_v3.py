# -*- coding: utf-8 -*-
"""v3：按「标量 / 四轮表 / 检查 / 整定 / 附录」重排《麦轮小车参数测量清单》。

原则：
- 不删任何信息，只分流。凡进公式的标量 → 《必需参数》；
  四轮重复的 → 《四轮参数表》； yes/no 判定 → 《一次性检查》；
  调出来的数 → 《PID 整定与验收》；其余自动扫进《附录·可选与外观》。
- 每条必需参数都要写清「进入哪个公式」——写不出来的，说明它不该留在这张表里。
- 用户已填的实测值全部继承。
"""
import os
from openpyxl import Workbook, load_workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.worksheet.datavalidation import DataValidation

ROOT = "/home/hanli/麦轮小车pid"
REF = os.path.join(ROOT, ".麦轮小车参数测量清单.ref")
OUT = os.path.join(ROOT, "麦轮小车参数测量清单.xlsx")
SRC = os.path.join(REF, "v2_stage.xlsx")

# ---------------------------------------------------------------- 读取 v2 已修正内容 + 已填值
_src_file = SRC if (os.path.exists(SRC) and "参数测量清单" in load_workbook(SRC).sheetnames) else OUT
wb2 = load_workbook(_src_file)
if _src_file == OUT:
    wb2.save(SRC)
ws2 = wb2["参数测量清单"]
H2 = [ws2.cell(row=1, column=c).value for c in range(1, ws2.max_column + 1)]
COL = {n: i + 1 for i, n in enumerate(H2)}
INFO = {}
for r in range(2, ws2.max_row + 1):
    name = ws2.cell(row=r, column=COL["参数名称"]).value
    if not name:
        continue
    INFO[name] = dict(
        cat=ws2.cell(row=r, column=COL["一级分类"]).value,
        sub=ws2.cell(row=r, column=COL["二级分类"]).value,
        sym=ws2.cell(row=r, column=COL["符号"]).value,
        unit=ws2.cell(row=r, column=COL["单位"]).value,
        src=ws2.cell(row=r, column=COL["数据来源"]).value,
        how=ws2.cell(row=r, column=COL["测量方法 / 工具"]).value,
        prec=ws2.cell(row=r, column=COL["精度要求"]).value,
        imp=ws2.cell(row=r, column=COL["重要性"]).value,
        note=ws2.cell(row=r, column=COL["备注"]).value,
        val=ws2.cell(row=r, column=COL["实测值（待填）"]).value,
    )

used = set()
def mark(old):
    if old:
        used.add(old)
    return old

# ================================================================ Sheet1 必需参数（标量）
# (组, 参数名称, 符号, 单位, 数据来源, 怎么测/怎么定, 进入哪个公式（为什么必须）, 继承旧值来自, 备注, 强制值)
P1 = [
 ("0 约定与符号", "车体原点与坐标系定义", "—", "—", "推断",
  "原点 = 四轮轴心俯视投影矩形中心；X 沿前进方向；Y 向左；ω 俯视逆时针为正",
  "所有坐标、速度与正负号的基准。默认旋转中心就是原点，方程才最简",
  "车体原点位置约定（已统一）",
  "接地点随辊子相位跳动、不可复现，所以不要拿「接地矩形」当原点。"),
 ("0 约定与符号", "轮序编号", "—", "—", "推断",
  "1=左前 2=右前 3=左后 4=右后，定好立刻贴标签拍照",
  "决定四条方程分别对应哪个轮",
  "四个轮子的编号顺序", ""),
 ("0 约定与符号", "麦轮布局型式（X 型 / O 型）", "—", "—", "实测",
  "俯视看四个轮子的辊子连线：平行对角线 = X；同轴两轮同向 = O",
  "决定四条方程的形式本身；判错整套符号失效",
  "麦克纳姆轮布局类型（X 型 / O 型）",
  "看到的是 X 还是 O 直接决定能不能套用后面的标准方程。"),

 ("A 几何 · 控制与 URDF 都要", "车体包络长（含轮外缘）", "L", "mm", "实测",
  "卷尺沿最长方向，两端取最凸出点", "URDF base_link 视觉/碰撞盒尺寸",
  "车体总长（含轮外缘）", ""),
 ("A 几何 · 控制与 URDF 都要", "车体包络宽（含轮外缘）", "W", "mm", "实测",
  "卷尺沿最宽方向", "URDF base_link 视觉/碰撞盒尺寸",
  "车体总宽（含轮外缘）", ""),
 ("A 几何 · 控制与 URDF 都要", "车体包络高（含最高件）", "H", "mm", "实测",
  "水平尺压在最高点，量到地面", "URDF base_link 高度上限",
  "车体总高（含电池等最高件）", ""),
 ("A 几何 · 控制与 URDF 都要", "轮子悬出量（车体侧边 → 轮最外点）", "s", "mm", "实测",
  "在过轮心的横向断面上，直尺垂直车体侧边量；不是量到全车最宽处",
  "车体本体宽 = 总宽 − 2s，用于 URDF base box 宽度",
  "轮子外缘到车体边缘的悬出量",
  "缩进车体内记负值。"),
 ("A 几何 · 控制与 URDF 都要", "离地间隙", "h_g", "mm", "实测",
  "直尺量底盘最下沿到地面", "URDF base_link 的 z 坐标",
  "底盘最低点到地面距离", ""),
 ("A 几何 · 控制与 URDF 都要", "轴距（前后轮轮心纵向距离）", "L_b", "mm", "实测",
  "卷尺量同一侧前/后轮轴心；取轴心、不加轮径；左右各量一次取平均",
  "l_x = L_b/2 → 逆运动学 ω 系数 (l_x+l_y)；URDF 前/后关节 x = ±L_b/2",
  "轴距（前后轮轮心纵向距离）", ""),
 ("A 几何 · 控制与 URDF 都要", "前轮距（左右轮轮心横向距离）", "W_bf", "mm", "实测",
  "量左右轮轴心间的横向距离；不加轮厚（等价：外缘距 − 轮厚 t）",
  "l_yf = W_bf/2；URDF 前关节 y = ±W_bf/2",
  "轮距（左右轮轮心横向距离）", "", 120),
 ("A 几何 · 控制与 URDF 都要", "后轮距（左右轮轮心横向距离）", "W_br", "mm", "实测",
  "同前轮距方法，量后轴",
  "l_yr = W_br/2；URDF 后关节 y = ±W_br/2。前后不等会在模型里留下残差",
  "前轮距 / 后轮距（若前后不等）",
  "你实测前 120 / 后 118，差 2 mm：需要在 URDF 里如实建，不能当成均值。", 118),
 ("A 几何 · 控制与 URDF 都要", "轮子数量", "n_w", "个", "实测",
  "目视计数", "URDF 关节数 / 逆运动学方程行数", "轮子数量", ""),
 ("A 几何 · 控制与 URDF 都要", "轮子外径（含辊子外廓）", "D", "mm", "实测",
  "卡尺量最大直径；麦轮外廓近似多边形，慢转找最大值，3 个方位取平均",
  "R_eff 的几何初值 ≈ D/2；URDF wheel cylinder radius",
  "轮子外径（含辊子外廓）", "真机算里程一律用标定后的 R_eff，不要用 D/2。"),
 ("A 几何 · 控制与 URDF 都要", "轮宽（厚度）", "t", "mm", "实测",
  "卡尺量轮子两侧面间距", "URDF wheel cylinder length", "轮宽（厚度）", ""),

 ("B 质量与惯量", "整车整备质量（含电池）", "m", "kg", "实测",
  "电子秤称整车", "URDF 总质量校核；前馈与惯量估算的输入",
  "整车整备质量（含电池）",
  "URDF 用 kg（本表其余地方用 g，这一条请填 kg）。"),
 ("B 质量与惯量", "单个轮子质量", "m_w", "kg", "实测",
  "电子秤逐个称四个轮子", "URDF wheel link <mass>；车体质量 = m − 4·m_w",
  "单个轮子质量", ""),
 ("B 质量与惯量", "车体（不含轮）质心平面坐标", "(x_c, y_c)", "mm", "实测",
  "四轮载荷法：x_c = ΣN_i x_i / ΣN_i，y_c = ΣN_i y_i / ΣN_i；四轮必须垫等高垫块保持水平",
  "URDF base <inertial><origin xyz>；同时也就是「质心相对旋转轴的偏移」",
  "质心平面坐标 (x_c, y_c)",
  "因为原点已定义为旋转轴，所以 x_c/y_c 天然就是质心偏离旋转中心的量。"),
 ("B 质量与惯量", "车体（不含轮）质心高度", "z_c", "mm", "实测",
  "倾斜称重法：一侧两轮垫高 h 后重称四轮载荷，由载荷转移量反推 z_c；或用双线悬吊法拍照反推",
  "URDF base inertial 的 z；也是加减速时轴荷转移的力臂",
  "质心高度（悬吊法实测校验）",
  "四轮称重只能算出平面的质心，z_c 必须靠倾斜/悬吊，这一步省不掉。"),
 ("B 质量与惯量", "车体绕 Z 轴转动惯量（对质心）", "I_zz", "kg·m²", "计算",
  "Σ(m_i r_i²) + 平行轴项，有条件用扭摆法实测校核",
  "URDF <inertia izz>；注意 URDF 的惯量必须绕质心表达",
  "绕 Z 轴转动惯量",
  "I_xx / I_yy 不必单测：用长方体公式 m(H²+W²)/12 与 m(H²+L²)/12 估算即可。"),

 ("C 驱动映射", "减速比", "i", "—", "铭牌",
  "铭牌/手册，或数齿数", "输出轴转速 = 电机转速 / i；把 PPR 折算到轮轴的 C_1rev",
  "减速比", ""),
 ("C 驱动映射", "编码器每转脉冲数（单相）", "PPR", "线", "手册",
  "手册/铭牌", "C_1rev = PPR × 倍频 × i（编码器装在电机轴时）",
  "每转脉冲数（单相）", ""),
 ("C 驱动映射", "计数倍频方式", "—", "—", "实测",
  "记录你实际用的是 1x / 2x / 4x", "同上式；倍频搞错，C_1rev 会差 2~4 倍",
  "计数倍频方式", ""),
 ("C 驱动映射", "编码器安装位置（电机轴 / 输出轴）", "—", "—", "实测",
  "拍照片确认编码器盘磁环装在哪根轴上",
  "决定 C_1rev 要不要再乘一个 i —— 这是整套标定里最常见的量级错误来源",
  "编码器装在电机轴还是减速输出轴", ""),
 ("C 驱动映射", "满占空比轮速上限（名义）", "n_max", "rpm", "实测",
  "悬空给 100% 占空比，读编码器；同时记录转向符号",
  "URDF joint <limit velocity>；PID 输出限幅依据",
  "满占空比下四轮实测转速", "四个轮的个体差见《四轮参数表》。"),
 ("C 驱动映射", "地面滚动方向等效摩擦系数", "μ1", "—", "实机标定",
  "预压 Sigmat rotating alpine 复合纵滑的 jerk stage，或用「最大不打滑加速度 / g」估算",
  "Gazebo <mu1>；决定纵向打滑边界、前馈能开多大", "地面摩擦系数（估）",
  "真机 μ 与教材查表值差很多，建议实测估算而不是查表。"),
 ("C 驱动映射", "辊子自由方向等效摩擦系数", "μ2", "—", "实机标定",
  "预压 Sigma rotating 横向静置段或横向推行测试",
  "Gazebo <mu2>；决定横移时的侧滑量", None,
  "麦轮在滚动方向与辊子自由方向的摩擦不一样，所以 μ1/μ2 要分开。"),
 ("C 驱动映射", "摩擦是否方向相关", "—", "—", "实测",
  "正向 / 反向 / 左移 / 右移各做一次滑移测试比较",
  "判断要不要做方向相关的摩擦与滑移补偿", None, ""),

 ("D 标定常数", "有效几何常数", "(l_x+l_y)/R_eff", "—", "实测",
  "指令原地纯旋转 n 圈（建议 n=5），记第 i 轮累计数绝对值 ΔN_i，则 = ΔN_i /(C_1rev·n)，四轮取平均",
  "纯 ∧ 旋转唯一的标定常数；逆运动学 ω 系数、码盘航向解算都用它",
  "有效几何常数 (l_x+l_y)/R_eff（取代原 L_eff、W_eff 两条）",
  "X 型布局下 l_x 与 l_y 恒以和出现，任何运动测试都无法把 L_eff / W_eff 分开标定，"
  "所以只留这一条；l_x、l_y 单值取上面第 9/10/11 条的几何实测。"),
 ("D 标定常数", "陀螺仪 yaw 标度因数修正系数", "k_yaw", "—", "实测",
  "地面画基准，用匀速手动/自动正转 3 圈、反转 3 圈，比较 IMU 积分角与实际角",
  "ψ_true = k_yaw·∫(ω_z − b_z)dt；不标定的话「指令转 30°」会系统性偏 0.3~0.9°",
  "陀螺仪 yaw 标度因数修正系数",
  "这是「精确转角度」的关键项，与零偏是两回事，不能省。"),
 ("D 标定常数", "陀螺仪角速度零偏", "b_z", "°/s", "实测",
  "静置 60 s 取输出平均值", "上式的 b_z；静止时航向缓慢漂移的主因",
  "角速度零偏", ""),
 ("D 标定常数", "IMU 姿态输出率", "f_imu", "Hz", "手册",
  "手册值或实测", "积分步长与链路延迟；决定航向能不能只靠陀螺积分",
  "数据输出率", ""),
 ("D 标定常数", "IMU 接口类型与波特率", "—", "—", "实测",
  "串口 TTL 还是 I2C；串口记波特率", "解算链路的延迟与丢帧风险",
  "接口类型与波特率", ""),

 ("E 控制时序与环境", "轮速环控制周期", "T_s", "ms", "实测",
  "记录循环实际周期（不要填设定值）", "PID 差分/积分步长：Ki·T_s 与 Kd/T_s",
  "轮速环控制周期", ""),
 ("E 控制时序与环境", "航向 / 位置环控制周期", "T_a", "ms", "实测",
  "同上", "外环步长；与 T_s 的比值决定串级是否稳定",
  "航向/姿态环控制周期", ""),
 ("E 控制时序与环境", "测试地面材质", "—", "—", "实测",
  "拍照 + 文字记录", "决定上面 μ1/μ2 的适用工况", "测试地面材质", ""),
 ("E 控制时序与环境", "常态载重（实际使用配置）", "—", "g", "实测",
  "按实际使用状态称重", "质量、惯量、摩擦的实际工况修正",
  "常态载重（实际使用状态）", ""),
 ("E 控制时序与环境", "电机运转时的带载供电电压", "U_load", "V", "实测",
  "电机运转时用万用表或 ADC 采样", "速度随电压漂移；判断要不要做电压前馈",
  "运行中的带载电压跌落", ""),
]

# ================================================================ Sheet3 一次性检查
# (类别, 检查项, 判据 / 合格标准, 继承旧值来自)
P3 = [
 ("结构", "是否四轮独立承重（有无辅助万向轮）", "必须四轮独立承重，不能有万向轮分担载荷", "是否四轮独立承重（有无辅助万向轮）"),
 ("结构", "四轮接地点是否共面", "平板（玻璃/大理石）上塞尺检查，间隙 ≤1 mm", "四轮接地点是否共面"),
 ("结构", "底盘扭转刚度（对角垫起后另一角离地变化）", "≤2 mm", "底盘结构扭转刚度（对角施力的形变量）"),
 ("结构", "电机安装座与轮心的同轴 / 偏斜偏差", "≤0.5 mm", "电机安装座与轮心的同轴/偏斜偏差"),
 ("结构", "轮子径向 / 端面跳动", "≤0.5 mm", "轮子径向/端面跳动"),
 ("结构", "轮轴与底盘垂直度偏差（camber）", "≤1°", "轮轴与底盘的垂直度偏差"),
 ("结构", "轮子与轴的固定是否打滑", "手扭轮子不能与轴相对转动（顶丝/D 轴/夹紧）", "轮子与轴的固定方式（是否打滑）"),
 ("结构", "四轮辊子初始相位是否一致", "±2°（同一时刻俯拍四轮比较）", "辊子初始相位四轮是否一致"),
 ("结构", "四轮辊子磨损程度是否一致", "目视一致，无单轮明显磨平", "四轮辊子磨损程度是否一致"),
 ("结构", "俯视辊子朝向记录（照片）", "四个轮的辊子斜向逐一拍照留存", "俯视辊子朝向（判定 X/O 型）"),
 ("电气", "电机运转时编码器是否丢脉冲", "长时间对比计数，无异常跳变", "电机运转时编码器是否丢脉冲"),
 ("电气", "线束与接插件可靠性（抖动不复位）", "抖动线束，不复位不丢数", "线束与接插件接触可靠性"),
 ("电气", "主控能否 4 路编码器同时正交解码", "必须 4 路硬件正交解码", "可用正交解码接口数量"),
 ("电气", "编码器采样与 PWM 更新是否同一中断", "是（时序同步）", "编码器采样与 PWM 更新是否同一中断"),
 ("电气", "满电与亏电同占空比的速度差", "≤3%", "满电与亏电下同占空比的速度差"),
 ("电气", "驱动板是否过热 / 触发限流", "连续运行 3 min 不触发保护", None),
 ("IMU", "IMU 航向符号与车体 ω 一致", "手动把车转 90°，看输出符号是否符合预期", "坐标系与车体坐标系是否一致"),
 ("IMU", "IMU 安装水平（俯仰 / 横滚安装误差）", "≤1°", "安装朝向与车体 X 轴的夹角偏差"),
 ("IMU", "上电是否需要静止零点标定", "记录流程即可（无关乎精度，关乎操作）", "是否需要上电静止零点标定"),
]

# ================================================================ Sheet4 PID 整定与验收
# (类别, 项目, 目标 / 判据, 继承旧值来自)
P4 = [
 ("整定结果", "轮 1 速度环 Kp / Ki / Kd", "带宽 ≈ 15~25 rad/s，阶跃无超调或 <5%", "轮 1 速度环 (Kp, Ki, Kd)"),
 ("整定结果", "轮 2 速度环 Kp / Ki / Kd", "同上", "轮 2 速度环 (Kp, Ki, Kd)"),
 ("整定结果", "轮 3 速度环 Kp / Ki / Kd", "同上", "轮 3 速度环 (Kp, Ki, Kd)"),
 ("整定结果", "轮 4 速度环 Kp / Ki / Kd", "同上", "轮 4 速度环 (Kp, Ki, Kd)"),
 ("整定结果", "航向环 Kp / Ki / Kd", "带宽约为轮速环的 1/5~1/10", "航向环 (Kp, Ki, Kd)"),
 ("整定结果", "位置 / 里程环 Kp / Ki / Kd", "视是否有位置闭环而定", "位置环 / 里程环 (Kp, Ki, Kd)"),
 ("整定结果", "积分限幅与输出限幅", "记录数值即可", "积分抗饱和阈值与输出限幅"),
 ("整定结果", "速度反馈滤波参数", "一阶低通时间常数或滑动平均长度", "速度反馈滤波参数"),
 ("验收指标", "指令直行 2 m 的横向偏差", "≤20 mm", "指令直行 2 m 的横向偏差"),
 ("验收指标", "指令直行 2 m 的航向偏差", "≤2°", "指令直行 2 m 的航向偏差"),
 ("验收指标", "指令原地旋转 360° 的实际转角", "误差 ≤5°", "指令原地旋转 360° 的实际转角"),
 ("验收指标", "指令横移 1 m 的实际位移", "偏差 ≤30 mm", "指令横移 1 m 的实际位移"),
 ("验收指标", "最大线速度 / 最大线加速度", "不丢步、不打滑前提下的上限", "最大线速度 / 最大线加速度"),
 ("验收指标", "最大角速度 / 最大角加速度", "同上", "最大角速度 / 最大角加速度"),
]
for it in P4:
    mark(it[3])

# ================================================================ Sheet5 拍照清单
SHOTS = [
 ("P1", "整车俯视正射（画面内放钢尺）", "量轴距/轮距/车体包络；URDF 平面尺寸",
  "相机正上方垂直向下，画面放钢尺做比例尺，四角拍齐避免透视倾斜", "高", "01_top.png"),
 ("P2", "整车侧视（水平视角）", "轴距、轮径、离地间隙、车高",
  "镜头与车体中心等高，正对侧面，画面保持水平", "高", "02_side.png"),
 ("P3", "单轮特写：侧视 + 45° 斜视（只拍 1 个轮）", "辊子几何、相位、磨损（四轮同型号，一个足够）",
  "近距对焦，画面包含轮子完整高度", "高", "03_wheel.png"),
 ("P4", "电机 + 编码器 + 铭牌特写", "型号、减速比、编码器装在哪根轴",
  "铭牌字迹清晰；同时拍到编码器盘/磁环位置", "高", "04_motor_encoder.png"),
 ("P5", "四轮静态载荷读数（1 张拼图或 4 张）", "四轮载荷 → 平面质心",
  "秤的读数清晰可读，并标明是哪个轮；四轮垫等高垫块", "高", "05_load.png"),
 ("P6", "整车正视（水平视角）", "车体高度、轮垂直度、camber",
  "正对车头，画面水平", "中", "06_front.png"),
 ("P7", "整车底视", "确认布局 X/O、走线、有无干涉",
  "小车架空，从正下方补光拍摄", "中", "07_bottom.png"),
 ("P8", "IMU / 主控 / 驱动板特写", "接口形式与 IMU 安装朝向",
  "正面垂直拍摄，丝印清晰", "中", "08_board_imu.png"),
 ("P9", "棋盘格标定板 ≥15 张不同角度", "相机畸变标定（只有要做图上量测才需要）",
  "打印 8x6 格、格边长 25 mm 的棋盘格，不同角度各一张", "低", "09_calib_XX.png"),
]

# ================================================================ 样式
def xlc(h): return "FF" + h.lstrip("#").upper()
XHDR, XLIGHT, XALT, XB = xlc("#4472C4"), xlc("#D9E2F3"), xlc("#F2F2F2"), xlc("#BFBFBF")
XRED, XAMB, XDARK, XGRN = xlc("#C00000"), xlc("#BF8F00"), xlc("#1A1A1A"), xlc("#C6EFCE")
thin = Side(style="thin", color=XB)
BORD = Border(left=thin, right=thin, top=thin, bottom=thin)
HF = Font(bold=True, color="FFFFFFFF", size=11)
HFIL = PatternFill("solid", fgColor=XHDR)
HA = Alignment(horizontal="center", vertical="center", wrap_text=True)
TOP = Alignment(horizontal="left", vertical="top", wrap_text=True)
CEN = Alignment(horizontal="center", vertical="center", wrap_text=True)

wb = Workbook()

def put_header(ws, headers, height=32):
    for c, n in enumerate(headers, 1):
        cell = ws.cell(row=1, column=c, value=n)
        cell.font, cell.fill, cell.alignment, cell.border = HF, HFIL, HA, BORD
    ws.row_dimensions[1].height = height

def put_row(ws, r, vals, center_cols=(), fill_cols=()):
    for c, v in enumerate(vals, 1):
        cell = ws.cell(row=r, column=c, value=v)
        cell.border = BORD
        cell.alignment = CEN if c in center_cols else TOP
    for c in fill_cols:
        ws.cell(row=r, column=c).fill = PatternFill("solid", fgColor=XLIGHT)

# ---------------------------------------------------------------- Sheet1 必需参数
ws = wb.active
ws.title = "必需参数（标量）"
FIGREF = {
    "车体包络长（含轮外缘）": "图2", "车体包络宽（含轮外缘）": "图2", "车体包络高（含最高件）": "图2",
    "离地间隙": "图2", "轮子悬出量（车体侧边 → 轮最外点）": "图4",
    "轴距（前后轮轮心纵向距离）": "图3", "前轮距（左右轮轮心横向距离）": "图3",
    "后轮距（左右轮轮心横向距离）": "图3",
    "轮子外径（含辊子外廓）": "图8", "轮宽（厚度）": "图8",
    "整车整备质量（含电池）": "图11", "单个轮子质量": "图11",
    "车体（不含轮）质心平面坐标": "图11", "车体（不含轮）质心高度": "图12",
    "车体绕 Z 轴转动惯量（对质心）": "图11",
    "满占空比轮速上限（名义）": "图13", "减速比": "图13",
    "地面滚动方向等效摩擦系数": "图17", "辊子自由方向等效摩擦系数": "图17",
    "陀螺仪角速度零偏": "图15", "陀螺仪 yaw 标度因数修正系数": "图15",
    "有效几何常数": "图17", "麦轮布局型式（X 型 / O 型）": "图10",
    "车体原点与坐标系定义": "图1", "轮序编号": "图1",
}
put_header(ws, ["序号", "组", "参数名称", "符号", "单位", "数据来源",
                "怎么测 / 怎么定", "进入哪个公式（为什么必须）", "实测值", "备注", "参考图"])
grp, n_rest = {}, 0
for i, row in enumerate(P1):
    group, name, sym, unit, src, how, why, old, note = row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8]
    force = row[9] if len(row) > 9 else None
    if old:
        mark(old)
    grp.setdefault(group, len(grp))
    band = XLIGHT if grp[group] % 2 == 0 else XALT
    val = force if force is not None else (INFO.get(old, {}).get("val") if old else None)
    if old and old in INFO and INFO[old]["val"] not in (None, ""):
        n_rest += 1
    put_row(ws, 2 + i, [i + 1, group, name, sym, unit, src, how, why, val, note, FIGREF.get(name, "—")],
            center_cols=(1, 2, 4, 5, 6, 11), fill_cols=(9,))
    ws.cell(row=2 + i, column=2).fill = PatternFill("solid", fgColor=band)
for col, w in {"A": 5, "B": 22, "C": 30, "D": 13, "E": 8, "F": 10,
               "G": 46, "H": 46, "I": 16, "J": 40, "K": 9}.items():
    ws.column_dimensions[col].width = w
last = 1 + len(P1)
ws.freeze_panes = "C2"
ws.auto_filter.ref = f"A1:K{last}"
dv = DataValidation(type="list", formula1='"铭牌,手册,实测,计算,推断,实机标定"', allow_blank=True, showDropDown=False)
ws.add_data_validation(dv); dv.add(f"F2:F{last}")
print("[Sheet1] 必需参数 %d 条，继承已填值 %d 个" % (len(P1), n_rest))

# ---------------------------------------------------------------- Sheet2 四轮参数表
# 原表里 these 项已被《四轮参数表》按「每轮一列」吸收，或并入派生行 / 必需参数，不再进附录
for _m in [
    "单轮手转整圈的计数值", "每米行驶计数值", "四轮每米计数一致性", "轮子有效滚动半径",
    "四个电机同占空比下的转速差", "PWM 起转死区占空比", "各轮速度标定系数 k_i（轮 1~4）",
    "四个轮子的静态法向载荷", "各轮静态载荷分配比", "四轮静态载荷不平衡度",
    "左前轮水平偏摆角（toe）", "右前轮水平偏摆角（toe）",
    "左后轮水平偏摆角（toe）", "右后轮水平偏摆角（toe）", "四轮 toe 极差（最大减最小）",
    "质心相对车体几何中心的纵向偏移", "质心相对车体几何中心的横向偏移",
    "有效轮径标定值", "四轮外径一致性（最大减最小）", "纯旋转指令的旋转轴约定",
]:
    mark(_m)
ws2n = wb.create_sheet("四轮参数表")
H2N = ["轮序", "位置", "辊子自由方向（俯视）", "给 + 占空比时转向", "正转时计数增减",
       "外径 D_i (mm)", "单圈计数 C_1rev_i", "每米计数 C_m_i", "有效半径 R_eff_i (mm)",
       "满占空比转速 n_i (rpm)", "起转死区 DZ_i (%)", "速度标定系数 k_i",
       "偏摆角 toe_i (°)", "静态载荷 N_i (g)", "备注"]
put_header(ws2n, H2N, height=46)
ROWS21 = [
 (1, "左前 FL", "+45°（指向左前）"),
 (2, "右前 FR", "−45°（指向右前）"),
 (3, "左后 RL", "−45°（指向右前）"),
 (4, "右后 RR", "+45°（指向左前）"),
]
for i, (no, pos, rol) in enumerate(ROWS21):
    put_row(ws2n, 2 + i, [no, pos, rol] + ["☐ 待填"] * 11,
            center_cols=tuple(range(1, 15)), fill_cols=tuple(range(4, 15)))
put_row(ws2n, 6, ["极差 Δ"] + ["—"] * 2 + ["—"] * 11,
        center_cols=tuple(range(1, 16)))
ws2n.cell(row=6, column=2, value="最大 − 最小").alignment = CEN
ws2n.cell(row=7, column=1, value=(
    "【第 2–5 行是「一套符号」】：辊子自由方向 → 给正占空比时转向 → 正转时计数增减。"
    "这三列决定四条逆运动学方程的正负号，只要错一格，小车就朝指令的反方向走，PID 再怎么调都没用。"))
ws2n.cell(row=8, column=1, value=(
    "【标准方程（X 型，本表默认）】R·ω₁ = v_x − v_y − (l_x+l_y)ω ；R·ω₂ = v_x + v_y + (l_x+l_y)ω ；"
    "R·ω₃ = v_x + v_y − (l_x+l_y)ω ；R·ω₄ = v_x − v_y + (l_x+l_y)ω"))
ws2n.cell(row=9, column=1, value=(
    "【怎么算】R_eff_i = 1000·C_1rev_i /(2π·C_m_i) ；k_i = 实测轮速 / 指令轮速（同占空比下）；"
    "N_i 必须由四次称重得到、且四轮垫等高垫块；toe_i 是水平偏摆角，|toe| ≤0.3° 才合格。"))
ws2n.cell(row=10, column=1, value=(
    "【为什么这些留在表里】它们都是「每轮一个值」的量：同一个物理量的四份拷贝在原表里占了几十行，"
    "而且差别本身（极差）才是真正的误差源——四轮一致比单个轮的绝对值更重要。"))
ws2n.cell(row=11, column=1, value=(
    "【若“必需参数”第 3 条判为 O 型】上面这组方程不成立：O 型是同轴两轮辊子方向相同，需另行推导"
    "（值得一提的是，那种布局反而能把 l_x 和 l_y 分别标定出来）。"))
for r in range(7, 12):
    ws2n.merge_cells(start_row=r, start_column=1, end_row=r, end_column=15)
    ws2n.cell(row=r, column=1).alignment = TOP
    ws2n.row_dimensions[r].height = 32
for col, w in {"A": 7, "B": 10, "C": 18, "D": 15, "E": 15, "F": 12, "G": 15, "H": 14,
               "I": 15, "J": 17, "K": 14, "L": 14, "M": 13, "N": 13, "O": 30}.items():
    ws2n.column_dimensions[col].width = w

# ---------------------------------------------------------------- Sheet3 一次性检查
ws3 = wb.create_sheet("一次性检查")
put_header(ws3, ["序号", "类别", "检查项", "判据 / 合格标准", "实测或判定", "状态"])
for i, (cat, item, crit, old) in enumerate(P3):
    if old:
        mark(old)
    v = INFO.get(old, {}).get("val") if old else None
    put_row(ws3, 2 + i, [i + 1, cat, item, crit, v, "☑" if v in ("是",) else "☐"],
            center_cols=(1, 2, 5, 6), fill_cols=(5,))
for col, w in {"A": 6, "B": 10, "C": 34, "D": 44, "E": 18, "F": 8}.items():
    ws3.column_dimensions[col].width = w
last3 = 1 + len(P3)
ws3.freeze_panes = "A2"
ws3.auto_filter.ref = f"A1:F{last3}"
d2 = DataValidation(type="list", formula1='"☐,☑,✗"', allow_blank=True, showDropDown=False)
ws3.add_data_validation(d2); d2.add(f"F2:F{last3}")

# ---------------------------------------------------------------- Sheet4 PID 整定与验收
ws4 = wb.create_sheet("PID整定与验收")
put_header(ws4, ["序号", "类别", "项目", "目标 / 判据", "整定值 或 实测", "备注"])
for i, (cat, item, crit, old) in enumerate(P4):
    v = INFO.get(old, {}).get("val") if old else None
    put_row(ws4, 2 + i, [i + 1, cat, item, crit, v, None],
            center_cols=(1, 2), fill_cols=(5,))
for col, w in {"A": 6, "B": 12, "C": 30, "D": 40, "E": 24, "F": 26}.items():
    ws4.column_dimensions[col].width = w
ws4.freeze_panes = "A2"
last4 = 1 + len(P4)
ws4.auto_filter.ref = f"A1:F{last4}"

# ---------------------------------------------------------------- Sheet5 拍照清单
ws5 = wb.create_sheet("拍照清单")
put_header(ws5, ["编号", "拍摄对象 / 视图", "拍摄目的", "拍摄要求", "优先级", "建议文件名", "状态", "备注"])
for i, row in enumerate(SHOTS):
    put_row(ws5, 2 + i, list(row) + ["☐", None], center_cols=(1, 5, 7), fill_cols=(7,))
for col, w in {"A": 7, "B": 32, "C": 36, "D": 46, "E": 9, "F": 22, "G": 8, "H": 24}.items():
    ws5.column_dimensions[col].width = w
last5 = 1 + len(SHOTS)
ws5.auto_filter.ref = f"A1:H{last5}"
d3 = DataValidation(type="list", formula1='"高,中,低"', allow_blank=True, showDropDown=False)
ws5.add_data_validation(d3); d3.add(f"E2:E{last5}")
d4 = DataValidation(type="list", formula1='"☐,☑"', allow_blank=True, showDropDown=False)
ws5.add_data_validation(d4); d4.add(f"G2:G{last5}")

# ---------------------------------------------------------------- Sheet6 附录（自动扫余量）
ws6 = wb.create_sheet("附录·可选与外观")
put_header(ws6, ["序号", "一级分类", "二级分类", "参数名称", "符号", "单位",
                 "测量方法 / 工具", "重要性", "实测值", "备注"])
rest = [(n, d) for n, d in INFO.items() if n not in used]
rest.sort(key=lambda x: (str(x[1]["cat"]), str(x[1]["sub"]), x[0]))
for i, (n, d) in enumerate(rest):
    put_row(ws6, 2 + i, [i + 1, d["cat"], d["sub"], n, d["sym"], d["unit"],
                         d["how"], d["imp"], d["val"], d["note"]],
            center_cols=(1, 6, 8), fill_cols=(9,))
for col, w in {"A": 5, "B": 18, "C": 16, "D": 30, "E": 12, "F": 10,
               "G": 44, "H": 9, "I": 16, "J": 40}.items():
    ws6.column_dimensions[col].width = w
last6 = 1 + len(rest)
ws6.freeze_panes = "D2"
ws6.auto_filter.ref = f"A1:J{last6}"
print("[附录] 自动迁移 %d 条（未被前面任何一张表引用）" % len(rest))

# ---------------------------------------------------------------- Sheet7 使用说明
ws7 = wb.create_sheet("使用说明")
ws7.column_dimensions["A"].width = 26
ws7.column_dimensions["B"].width = 110
ws7.cell(row=1, column=1, value="麦轮小车参数测量清单 · 使用说明（v3 · 重排版）").font = Font(bold=True, size=14, color=XDARK)
ws7.merge_cells("A1:B1")
ws7.row_dimensions[1].height = 26
NOTES = [
 ("表格分工", "必需参数（标量）= 进公式的量 ｜ 四轮参数表 = 每轮一个值的量 ｜ "
              "一次性检查 = 只看 yes/no，不用报数 ｜ PID整定与验收 = 调出来的输出 ｜ 附录 = 可选/外观，不建模也能跑。"),
 ("为什么这么分", "原来 156 行混着四类东西：要测的输入、要做的检查、要算的中间量、要用 PID 调出来的输出。"
                 "混在一起就会觉得「参数多得没边」。分开以后，真正要动手去量的只有《必需参数》+《四轮参数表》。"),
 ("判断标准", "一条参数能不能留在《必需参数》里只看一条：能不能说出它进哪个公式（见该列）。说不出来的，一律下放附录。"),
 ("这版的删甄建议",
  "来源：本轮输入输出~7 行的删除建议（更狠，建议压到 29 条）。方向对，但有 5 处直接伤目标，所以我没采纳："
  "① 陀螺仪标度因数——你要「精确转 30°」，它决定角度的系统性偏差，删除等于放弃角度精度；"
  "② toe（水平偏摆角）——它是数值不是检查项，偏 1~2° 是走不直的头号机械原因；"
  "③ 前/后轮距——你实测前 120 / 后 118 不对称，URDF 必须如实建，也给一条方程残差；"
  "④ 质心高度 z_c——四轮称重只能算平面质心，z 靠倾斜称重/悬吊，建议里把它删了但 URDF 必须要；"
  "⑤ 摩擦分成 μ1/μ2——麦轮滚动方向和辊子自由方向的摩擦不一样，只填一个 μ 横移会明显偏。"
  "其余的建议（合并同轮均 Top 项、这边 4× 重复行成一张表、检查项与数值分离、Ixx/Iyy 用长方体公式算）我全部采纳了。"),
 ("先做再做", "《一次性检查》里的项目不用报数字，只要打勾；不合格的当场修机械，别指望软件补偿——"
              "尤其是轮子与轴打滑、四轮不共面、编码器丢脉冲这三条。"),
 ("最小可起步集", "急着先看效果的话，最小集是：坐标轴定义与轮序 → 轴距/前后轮距 → 轮子外径 → PPR × 倍频 × 减速比 → "
                 "单圈计数 C_1rev → 每米计数 C_m → 满占空比转速 → 起转死区 → 陀螺零偏与标度因数 → 控制周期。"
                 "这 10 类量齐了，就能让小车按编码器+IMU 闭环走起来（ URDF 可以后面再补尺寸）。"),
 ("仿真这条线", "做 URDF 时：车体 link 用《必需参数》A 组尺寸 + B 组质量惯量；四个轮子用《四轮参数表》的 D_i / m_w；"
                "摩擦用 μ1（滚动方向）和 μ2（辊子自由方向）两个值，不要只填一个 μ。"
                "惯量必须写成对质心的（URDF 的 <inertial><origin> 就是质心）。"),
 ("仿真 ≠ 真机", "仿真里参数是「干净」的：地形平、μ 恒定、无死区无敌 Tooth、无敌 PWM 死区。真机一上地面就有打滑与迟滞。"
                 "所以先在仿真里跑通「符号 + 极限 + 模型」，再用真机标定去补偿仿真补不上的那部分（死区、个体差、μ）。"),
 ("拍照清单", "从 16 张压到 9 张，去掉了重复角度与必要 tiresome 的细节。"
              "同一型号的四个轮只拍 1 个特写；四轮差别靠称重/计数/转速这几个数字体现，不靠照片。"),
 ("后续产出", "参数表填完后输出：① 逆运动学/正运动学方程（带符号表）；"
                 "② 质心·惯量·URDF inertial 段；③ MECE 误差因素 PDF；④ PID 整定方案与验收方法。"),
]
r = 3
for k, v in NOTES:
    kc = ws7.cell(row=r, column=1, value=k); kc.font = Font(bold=True, color=XDARK, size=11)
    kc.alignment = TOP; kc.fill = PatternFill("solid", fgColor=XALT); kc.border = BORD
    vc = ws7.cell(row=r, column=2, value=v); vc.alignment = TOP; vc.border = BORD
    r += 1

wb.properties.title = "麦轮小车参数测量清单 v3"
wb.save(OUT)
print("saved:", OUT)
print("必需参数 %d 条 ｜ 四轮表 %d 列 ｜ 检查 %d 条 ｜ 整定验收 %d 条 ｜ 拍照 %d 条 ｜ 附录 %d 条"
      % (len(P1), len(H2N), len(P3), len(P4), len(SHOTS), len(rest)))
