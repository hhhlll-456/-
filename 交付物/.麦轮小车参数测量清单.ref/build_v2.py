# -*- coding: utf-8 -*-
"""v2：按复核意见修正《麦轮小车参数测量清单》，保留用户已填的实测值。

设计要点：
1) 从 build.py 的数据段加载 v1 的 PARAMS/SHOTS（不执行其写文件副作用）；
2) 先读旧 xlsx 取出用户已填值；
3) 修正 / 插入 / 合并条目；
4) 计算 旧序号 -> 新序号 映射，并把新写文案里的 {#N} 占位符替换成新序号；
5) 重建整个工作簿（新增「对应图示」列与「四轮符号对照表」工作表）；
6) 输出 v2_refs.json 给后续脚本用来同步 gen_figs.py 与 md 里的编号。
"""
import json, os, re
from openpyxl import Workbook, load_workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.worksheet.datavalidation import DataValidation

ROOT = "/home/hanli/麦轮小车pid"
REF = os.path.join(ROOT, ".麦轮小车参数测量清单.ref")
OUT = os.path.join(ROOT, "麦轮小车参数测量清单.xlsx")
STAGE = os.path.join(REF, "v2_stage.xlsx")
VALJSON = os.path.join(REF, "v1_values.json")
BACKUP = os.path.join(REF, "v1_备份.xlsx")

# ---------------------------------------------------------------- 加载 v1 数据
src = open(os.path.join(REF, "build.py"), encoding="utf-8").read()
data = src.split("wb = Workbook()")[0]
ns = {}
exec(compile(data, "build.py", "exec"), ns)
PARAMS = [list(p) for p in ns["PARAMS"]]      # 每项补到第 11 位（备注）
SHOTS = ns["SHOTS"]
for p in PARAMS:
    while len(p) < 11:
        p.append(None)
V1_NAMES = [p[2] for p in PARAMS]
V1_NUM = {p[2]: i + 1 for i, p in enumerate(PARAMS)}   # 名称 -> 旧序号

# ---------------------------------------------------------------- 备份 + 读取已填值
USER_VAL, USER_NOTE = {}, {}
if os.path.exists(VALJSON):
    USER_VAL.update(json.load(open(VALJSON, encoding="utf-8")))
    print("[restore] 从快照回填 %d 条实测值" % len(USER_VAL))
else:
    try:
        _w = load_workbook(OUT)
        if "参数测量清单" in _w.sheetnames:
            _w.save(BACKUP)
            _ws = _w["参数测量清单"]
            for _r in range(2, _ws.max_row + 1):
                _n = _ws.cell(row=_r, column=4).value
                if not _n:
                    continue
                _v = _ws.cell(row=_r, column=11).value
                if _v not in (None, ""):
                    USER_VAL[_n] = _v
    except Exception as _e:
        print("[restore] 无法读取旧文件：", _e)
print("[restore] 待回填实测值 %d 条" % len(USER_VAL))

# ---------------------------------------------------------------- 编辑工具
def find(name):
    return next(i for i, p in enumerate(PARAMS) if p[2] == name)

def setf(name, **kw):
    i = find(name)
    key = {"cat": 0, "sub": 1, "newname": 2, "sym": 3, "unit": 4, "src": 5,
           "method": 6, "prec": 7, "imp": 8, "note": 10}
    for k, v in kw.items():
        PARAMS[i][key[k]] = v

def insert_after(name, items):
    i = find(name)
    PARAMS[i + 1:i + 1] = [list(x) + [None] * (11 - len(x)) for x in items]

# ================================================================ 一、修正错误
# ① 第5条 原点定义（去掉「接地矩形中心」）
setf("车体原点位置约定",
     newname="车体原点位置约定（已统一）",
     method=("原点的三个候选：① 四轮轴心在俯视平面投影点连成的矩形中心（本清单采用）；"
             "② 质心；③ 车体几何中心。三者不重合时必须换算，并在报告中写明取的是哪一个。"),
     note=("【v2 修正】原写法「四轮接地矩形中心」作废：麦轮的接地点随辊子相位周期性跳动、不可复现，"
           "用它会导致原点随轮子转角漂移。改用轴心俯视投影矩形的中心——轴心是固定几何量。"
           "标准逆运动学方程默认旋转中心就是这个点，取它方程最简。"))

# ② 惯量单位 kg·mm² -> kg·m²
for nm in ("绕 Z 轴转动惯量", "绕 X 轴转动惯量", "绕 Y 轴转动惯量"):
    setf(nm, unit="kg·m²",
         method=("I = Σ m_i·r_i²（含平行轴项）。质量用 g、长度用 mm 直接算出的是 g·mm²，"
                 "再除以 1e9 得到 kg·m²。有条件用扭摆法实测 I_zz 校核。"),
         note=("【v2 修正】单位由 kg·mm² 改为 kg·m²：本表质量用 g、长度用 mm，自然单位是 g·mm²，"
               "而 kg·mm² 与之相差 1000 倍，直接算会错一个数量级。"
               "参考量级：1.5 kg 小车、轴距 137.5 mm、轮距 119 mm → I_zz ≈ m(L²+W²)/12 ≈ 0.004 kg·m²。"))

# ④ 第32条 e 的表述
setf("辊子轴线到轮平面的偏移量",
     method="侧视把基准直尺贴住轮盘平面，卡尺量辊子轴心线相对该平面的偏移；或查轮子图纸。",
     note=("【v2 修正】不再写「决定有效半径大于 D/2」。e 只是理论几何输入，用于估算 R_eff；"
           "R_eff 还受辊子曲率、接触形变、装配与磨损影响，方向不一定。"
           "工程上一律以第 {#26} 条「每米计数反推」的标定值为准。"))

# ⑤ 第30条 β 定义写死
setf("辊子轴线与轮平面的夹角",
     newname="辊子轴线与轮盘平面的夹角（先固定定义再量）",
     method=("定义写死：β = 辊子自转轴 与 轮盘平面（垂直于轮轴的那个平面）的夹角，理想 45°；"
             "它同时等于辊子轴线在地面上的投影 与 滚动方向 X 的夹角。量法：侧视拍照后用量角器/图像量测。"),
     note=("【v2 修正】同一夹角有三种等价但数值不同的写法：与轮盘平面 β、与轮轴（=90°−β）、"
           "与轮盘平面法线（=90°−β）。理想 45° 时三者巧合相同，一旦偏离就对不上。"
           "本表统一采用「与轮盘平面的夹角」，若你用了别的定义请在备注注明。"))

# ③ 第25条 D 的量法
setf("轮子外径（含辊子外廓）",
     method=("游标卡尺量含辊子外廓的【最大】直径；麦轮外廓近似多边形，需缓慢转动轮子找到最大值处再读数，"
             "每轮至少 3 个方位（相差约 90°/120°）取平均，四轮分别记录。"))

# ③ 第66条 n_max 记符号
setf("满占空比下四轮实测转速",
     method=("悬空给 100% 占空比，分别读 4 轮编码器转速；⚠ 必须同时记录转向符号"
             "（给定 + 占空比时轮子向前还是向后），并与第 {#73} 条电机极性、第 {#84} 条计数方向"
             "一起填到「四轮符号对照表」工作表。"),
     note="四个轮若转速符号不一致，说明某一路接线/装配反了，先修硬件或改映射，不要指望 PID 救回来。")

# 方法论：第56条称重要垫等高垫块
setf("四个轮子的静态法向载荷",
     method=("每个轮子下垫一台电子秤读数（或同一台秤依次挪到四个轮位）。"
             "⚠ 四个轮子必须垫【厚度一致的等高垫块】（±0.1 mm）保持车身水平——"
             "四个支点不等高会破坏力矩平衡，算出的 CoM 会明显偏。"),
     note="垫块建议用同一块亚克力/板材裁四片，卡尺确认等厚。")

# 方法论：第53条主算法改为载荷法
setf("质心平面坐标 (x_c, y_c)",
     method=("主算法＝四轮载荷法：x_c = Σ(N_i·x_i)/ΣN_i，y_c = Σ(N_i·y_i)/ΣN_i"
             "（N_i 见第 {#56} 条，x_i/y_i 为各轮轴心在本体坐标系的坐标）。"
             "次要算法＝拆件质量加权 Σ(m_i·x_i)/Σm_i，仅作互校。"),
     note=("【v2 修正】原表把拆件质量加权写成主算法，已改：载荷法不需要拆车、不受漏称零件影响，"
           "是最直接可信的。两种算法结果相差 >5 mm，说明有零件没称到或坐标量错，必须复查。"))

# ================================================================ 二、新增参数
# toe 角（C 组 · 安装）
TOE_M = ("把车放在平板上，用长直尺把车体纵轴延长投影到地面并画基准线，量轮子中分面与该基准线的夹角；"
         "或用卡尺量轮子前后两端到同一纵梁的距离差 Δ 与这两点的纵向间距 Lm，toe = arctan(Δ/Lm)。"
         "约定：轮子前端向内偏（内八）记正。")
insert_after("轮轴与底盘的垂直度偏差", [
    ("C 麦克纳姆轮与轮系", "安装偏摆", "左前轮水平偏摆角（toe）", "toe_1", "°", "实测", TOE_M, "±0.3°", "必需"),
    ("C 麦克纳姆轮与轮系", "安装偏摆", "右前轮水平偏摆角（toe）", "toe_2", "°", "实测", "同 toe_1（四个轮分别量）", "±0.3°", "必需"),
    ("C 麦克纳姆轮与轮系", "安装偏摆", "左后轮水平偏摆角（toe）", "toe_3", "°", "实测", "同 toe_1（四个轮分别量）", "±0.3°", "必需"),
    ("C 麦克纳姆轮与轮系", "安装偏摆", "右后轮水平偏摆角（toe）", "toe_4", "°", "实测", "同 toe_1（四个轮分别量）", "±0.3°", "必需"),
    ("C 麦克纳姆轮与轮系", "安装偏摆", "四轮 toe 极差（最大减最小）", "Δtoe", "°", "计算", "四个 toe 值中最大值减最小值", "±0.3°", "必需"),
])
for nm in ("左前轮水平偏摆角（toe）", "右前轮水平偏摆角（toe）",
           "左后轮水平偏摆角（toe）", "右后轮水平偏摆角（toe）"):
    setf(nm, note=("【v2 新增】toe（水平偏摆）与 {#39} 条的垂直度（camber）是两回事："
                   "toe 偏 1~2° 就是小车走不直的头号机械原因，而且纯软件基本补不回来。"
                   "四轮均应 |toe| ≤ 0.3°，超差请重新调整电机座定位/垫片。"))
setf("四轮 toe 极差（最大减最小）",
     note="【v2 新增】四轮一致比单轮绝对角更重要；一致性差会引起横移时车身持续偏转。")

# 质心相对旋转轴的偏移（D 组 · 合成）
insert_after("质心高度 z_c", [
    ("D 质量质心与惯量", "合成", "质心相对车体几何中心的纵向偏移", "dx_c", "mm", "计算",
     "dx_c = x_c − x_geom（x_geom 为车体几何中心，默认与第 {#5} 条原点重合则为 0）", "±1 mm", "必需"),
    ("D 质量质心与惯量", "合成", "质心相对车体几何中心的横向偏移", "dy_c", "mm", "计算",
     "dy_c = y_c − y_geom", "±1 mm", "必需"),
    ("D 质量质心与惯量", "合成", "纯旋转指令的旋转轴约定", "-", "-", "推断",
     "写明控制算法里纯旋转绕的是哪个点：轴心矩形中心 / 质心 / IMU 安装点；三者不一致时必须换算", "-", "必需"),
])
setf("质心相对车体几何中心的纵向偏移",
     note=("【v2 新增】若质心与旋转中心不重合，纯旋转指令下车体会因惯性力产生平移漂移。"
           "低速（<0.3 m/s）影响很小，急加减速时才明显——若你的场景是慢速精确定位，可作为次要项。"))
setf("质心相对车体几何中心的横向偏移", note="同 dx_c，横向分量。")
setf("纯旋转指令的旋转轴约定",
     note="【v2 新增】推荐：运动学用轴心矩形中心（方程最简），动力学影响用 dx_c / dy_c 评估。")

# 电机电气常数（E 组 · 可选）
insert_after("空载电流 / 堵转电流", [
    ("E 驱动电机与减速箱", "电气常数（可选）", "电机转矩常数（折算到输出轴）", "K_t", "N·m/A", "手册",
     "手册值；输出轴侧 = 电机侧 K_t × i × η（η 取 0.7~0.9）。用于加速度前馈与力矩估算", "±10%", "可选"),
    ("E 驱动电机与减速箱", "电气常数（可选）", "反电动势常数", "K_e", "V·s/rad", "手册",
     "手册值；SI 单位下数值近似等于 K_t，用于搭建电机动态模型", "±10%", "可选"),
    ("E 驱动电机与减速箱", "电气常数（可选）", "电枢电阻", "R_a", "Ω", "实测",
     "万用表直接量电机两个引脚（堵住转子），多次转动后取最小读数", "±10%", "可选"),
])
for nm in ("电机转矩常数（折算到输出轴）", "反电动势常数", "电枢电阻"):
    setf(nm, note=("【v2 新增·可选】做速度/加速度前馈、估算负载扰动时才用得着；"
                   "而且必须先有第 {#64} 条的减速比 i 才有意义。"))

# 陀螺仪标度因数（G 组 · 标定）
insert_after("是否需要上电静止零点标定", [
    ("G IMU（GY-25T）", "标定", "陀螺仪 yaw 标度因数修正系数", "k_yaw", "-", "实测",
     "地面贴 0°/180°/360° 标记，手动或用电机匀速正转 3 圈、再反转 3 圈，比较 IMU 积分出的总角度"
     "与实际角度，k_yaw = 实际角度 / 积分角度，正反两次取平均", "±0.5%", "必需"),
])
setf("陀螺仪 yaw 标度因数修正系数",
     note=("【v2 新增】你要「精确转 30°」，航向精度直接取决于此项。GY-25T 的 gyro 标度误差典型 1~3%，"
           "不修正的话「指令转 30°」会系统性偏 0.3~0.9°，而且换向重复时也偏。"
           "它与第 {#93} 条的零偏/漂移是两回事，两个都要测。"
           "正反各一次还能顺便把标度误差和零偏分离。"))

# ================================================================ 三、J 组拆分 + 合并 L_eff/W_eff
i_w = find("有效轮距标定值")
i_l = find("有效轴距标定值")
assert i_l == i_w + 1
MERGED = [
    "J1 标定与测试输入", "几何常数标定",
    "有效几何常数 (l_x+l_y)/R_eff（取代原 L_eff、W_eff 两条）",
    "(l_x+l_y)/R_eff", "-", "实测",
    "指令原地纯旋转 n 圈（建议 n=5，用地面标记确认实际转过 n 整圈），"
    "记录第 i 轮累计计数的绝对值 ΔN_i；按 (l_x+l_y)/R_eff = ΔN_i / (C_1rev × n) 计算，四轮取平均。"
    "再乘第 {#26} 条的 R_eff 即得 (l_x+l_y)_eff。",
    "±0.5%", "必需",
    ("【v2 修正·可辨识性】X 型布局的逆运动学里 l_x 与 l_y 恒以 (l_x+l_y) 的和出现，"
     "四个方程的 ω 系数全是 ±(l_x+l_y)，所以任何运动试验（直行/横移/旋转）都只能辨识出这个和，"
     "无法把 L_eff 与 W_eff 分离——原表把它们拆成两条独立标定是不成立的。"
     "l_x、l_y 的单值必须取几何实测（第 {#14}、{#15} 条）。"
     "按你现有的实测值：(l_x+l_y) ≈ (137.5 + 119)/2 = 128.25 mm；"
     "又因前轮距 120 ≠ 后轮距 118，前后两组的系数实际是 128.75 / 127.75 mm，"
     "取平均即为最小二乘意义下的最优单一常数，残余约 0.8%，表现为轻微打滑。")
]
PARAMS[i_w:i_l + 1] = [MERGED + [None]]

setf("有效轮径标定值", src="计算",
     note="与 C 组第 {#26} 条 R_eff 是同一个量（D_eff = 2·R_eff），此处仅为汇总引用，不要重复测。")

for p in PARAMS:
    if p[0] == "J 控制与标定":
        p[0] = "J1 标定与测试输入" if p[1] in ("标定", "测试", "极限", "补偿", "滤波") else "J2 PID 整定结果"

# ================================================================ 四、编号重排 + 占位符替换
NEW_NUM = {}
for i, p in enumerate(PARAMS):
    NEW_NUM[i + 1] = i + 1
OLDMAP = {}
for i, p in enumerate(PARAMS):
    n = p[2]
    if n in V1_NUM:
        OLDMAP[V1_NUM[n]] = i + 1

def sub(s):
    if not s:
        return s
    def rp(m):
        old = int(m.group(1))
        return "#%d" % OLDMAP.get(old, old)
    return re.sub(r"\{#(\d+)\}", rp, s)

for p in PARAMS:
    for j in (6, 10):
        p[j] = sub(p[j])

# ---------------------------------------------------------------- 图示映射
FIG_RULES = [
    (("车体总长", "车体总宽", "车体总高", "底盘最低点"), "图2"),
    (("车体原点", "X 轴正方向", "Y 轴正方向", "ω 旋转正方向", "四个轮子的编号顺序"), "图1、图18"),
    (("麦克纳姆轮布局类型",), "图10、图18"),
    (("底盘板层数", "每层板厚度", "层与层之间的间距"), "图6"),
    (("轴距（", "轮距（", "前轮距 / 后轮距", "轮心相对车体几何中心的 X", "轮心相对车体几何中心的 Y"), "图3"),
    (("轮子外缘到车体边缘的悬出量",), "图4"),
    (("是否四轮独立承重", "四轮接地点是否共面", "底盘结构扭转刚度",
      "四个轮子的静态法向载荷", "各轮静态载荷分配比", "四轮静态载荷不平衡度"), "图5"),
    (("电机安装座与轮心的同轴",), "图7"),
    (("轮子数量", "轮子外径（", "轮宽（厚度）", "四轮外径一致性"), "图8"),
    (("轮子有效滚动半径",), "图8、图14"),
    (("辊子初始相位四轮是否一致", "俯视辊子朝向"), "图10、图18"),
    (("每个轮子的辊子数量", "辊子轴线与轮盘平面", "辊子直径", "辊子轴线到轮平面的偏移量",
      "相邻辊子中心的弧长", "辊面材料与摩擦特性", "四轮辊子磨损程度"), "图9"),
    (("轮子径向/端面跳动", "轮轴与底盘的垂直度偏差", "轮子与轴的固定方式", "输出轴形式与直径"), "图7"),
    (("水平偏摆角（toe）", "四轮 toe 极差"), "图18"),
    (("整车整备质量", "最大设计负载质量", "底盘板总质量", "电池质量", "主控+驱动板总质量",
      "单个电机（含减速箱）质量", "单个轮子质量", "其余零件合计质量",
      "电池安装中心三维坐标", "主控/驱动板安装中心三维坐标", "四个电机安装中心三维坐标",
      "配重块位置与质量", "质心平面坐标", "质心高度 z_c",
      "质心相对车体几何中心的纵向偏移", "质心相对车体几何中心的横向偏移",
      "纯旋转指令的旋转轴约定", "绕 Z 轴转动惯量", "绕 X 轴转动惯量", "绕 Y 轴转动惯量"), "图11"),
    (("质心高度（悬吊法实测校验）",), "图12"),
    (("电机型号与版本", "额定电压", "减速比", "空载转速（额定电压下）", "满占空比下四轮实测转速",
      "四个电机同占空比下的转速差", "额定转矩 / 堵转转矩", "PWM 起转死区占空比",
      "减速箱齿轮回差", "电机正转对应的接线", "PWM-转速曲线线性度",
      "电机转矩常数", "反电动势常数", "电枢电阻"), "图13"),
    (("空载电流 / 堵转电流",), "图13、图16"),
    (("编码器类型", "编码器供电电压", "每转脉冲数", "输出相数与是否正交", "计数倍频方式",
      "单轮手转整圈的计数值", "每米行驶计数值", "四轮每米计数一致性",
      "计数方向与轮子正转的关系", "最大响应频率", "计数方式"), "图14"),
    (("编码器装在电机轴还是减速输出轴",), "图8、图14"),
    (("陀螺仪 yaw 标度因数修正系数",), "图15、图17"),
    (("各轮速度标定系数", "有效几何常数", "有效轮径标定值"), "图14、图17"),
    (("指令直行 2 m 的横向偏差", "指令直行 2 m 的航向偏差", "指令原地旋转 360° 的实际转角",
      "指令横移 1 m 的实际位移", "最大线速度", "最大角速度"), "图17"),
    (("各轮死区补偿占空比",), "图13"),
]
CAT_DEFAULT = {
    "G IMU（GY-25T）": "图15",
    "H 供电与电气": "图16",
    "I 主控驱动与通信": "图16",
    "K 环境与工况": "图17",
    "J2 PID 整定结果": "—",
    "J1 标定与测试输入": "图17",
    "B 底盘几何结构": "图3",
}

def fig_for(p):
    cat, sub, name = p[0], p[1], p[2]
    for keys, fig in FIG_RULES:
        for k in keys:
            if name.startswith(k) or k in name:
                return fig
    return CAT_DEFAULT.get(cat, "—")

# ================================================================ 五、样式常量
def xlc(h):
    return "FF" + h.lstrip("#").upper()
XHDR, XLIGHT, XALT, XBORDER = xlc("#4472C4"), xlc("#D9E2F3"), xlc("#F2F2F2"), xlc("#BFBFBF")
XRED, XAMBER, XDARK = xlc("#C00000"), xlc("#BF8F00"), xlc("#1A1A1A")
thin = Side(style="thin", color=XBORDER)
BORDER = Border(left=thin, right=thin, top=thin, bottom=thin)
HFONT = Font(bold=True, color="FFFFFFFF", size=11)
HFILL = PatternFill("solid", fgColor=XHDR)
HALIGN = Alignment(horizontal="center", vertical="center", wrap_text=True)
TOP = Alignment(horizontal="left", vertical="top", wrap_text=True)
CEN = Alignment(horizontal="center", vertical="center", wrap_text=True)

wb = Workbook()
# ================================================================ Sheet1 参数测量清单
ws = wb.active
ws.title = "参数测量清单"
HEADERS = ["序号", "一级分类", "二级分类", "参数名称", "符号", "单位", "数据来源",
           "测量方法 / 工具", "精度要求", "重要性", "实测值（待填）", "备注", "对应图示"]
for c, n in enumerate(HEADERS, 1):
    cell = ws.cell(row=1, column=c, value=n)
    cell.font, cell.fill, cell.alignment, cell.border = HFONT, HFILL, HALIGN, BORDER
ws.row_dimensions[1].height = 30

grp = {}
n_restore = 0
for i, row in enumerate(PARAMS):
    r = 2 + i
    cat, sub, name, sym, unit, src, method, prec, imp, _, note = row
    grp.setdefault(cat, len(grp))
    band = XLIGHT if grp[cat] % 2 == 0 else XALT
    fig = fig_for(row)
    vals = [i + 1, cat, sub, name, sym, unit, src, method, prec, imp, None, note, fig]
    if name in USER_VAL:
        vals[10] = USER_VAL[name]
        n_restore += 1
    if name in USER_NOTE:
        vals[11] = USER_NOTE[name]
    for c, v in enumerate(vals, 1):
        cell = ws.cell(row=r, column=c, value=v)
        cell.border = BORDER
        cell.alignment = TOP
    for c in (1, 2, 6, 7, 9, 10, 13):
        ws.cell(row=r, column=c).alignment = CEN
    ws.cell(row=r, column=2).fill = PatternFill("solid", fgColor=band)
    ws.cell(row=r, column=11).fill = PatternFill("solid", fgColor=XLIGHT)
    ic = ws.cell(row=r, column=10)
    ic.font = Font(color=XRED, bold=True, size=11) if imp == "必需" else (
        Font(color=XAMBER, size=11) if imp == "重要" else Font(color=XDARK, size=11))
    if imp == "可选":
        ws.cell(row=r, column=4).font = Font(color=XDARK, size=11)
print("[restore] 回写实测值 %d 条" % n_restore)

last = 1 + len(PARAMS)
for col, w in {"A": 6, "B": 20, "C": 16, "D": 36, "E": 16, "F": 11, "G": 10,
               "H": 50, "I": 11, "J": 9, "K": 20, "L": 46, "M": 10}.items():
    ws.column_dimensions[col].width = w
ws.freeze_panes = "D2"
ws.auto_filter.ref = f"A1:M{last}"

dv1 = DataValidation(type="list", formula1='"铭牌,手册,实测,计算,推断,整定"', allow_blank=True, showDropDown=False)
ws.add_data_validation(dv1); dv1.add(f"G2:G{last}")
dv2 = DataValidation(type="list", formula1='"必需,重要,可选"', allow_blank=True, showDropDown=False)
ws.add_data_validation(dv2); dv2.add(f"J2:J{last}")

# ================================================================ Sheet2 四轮符号对照表
ws2 = wb.create_sheet("四轮符号对照表")
S_HDR = ["轮序", "位置", "辊子自由方向（俯视，X 型默认）", "v_x 系数", "v_y 系数", "ω 系数",
         "给 + 占空比时轮子转向（实测待填）", "轮子「正转」时计数增减（实测待填）", "核对依据"]
for c, n in enumerate(S_HDR, 1):
    cell = ws2.cell(row=1, column=c, value=n)
    cell.font, cell.fill, cell.alignment, cell.border = HFONT, HFILL, HALIGN, BORDER
ws2.row_dimensions[1].height = 42
S_ROWS = [
    (1, "左前 FL", "+45°（指向左前）", "+1", "−1", "−(l_x+l_y)"),
    (2, "右前 FR", "−45°（指向右前）", "+1", "+1", "+(l_x+l_y)"),
    (3, "左后 RL", "−45°（指向右前）", "+1", "+1", "−(l_x+l_y)"),
    (4, "右后 RR", "+45°（指向左前）", "+1", "−1", "+(l_x+l_y)"),
]
for i, (no, pos, rol, a, b, c_) in enumerate(S_ROWS):
    r = 2 + i
    for c, v in enumerate([no, pos, rol, a, b, c_, "☐ 待填", "☐ 待填",
                           "对照 #%d 辊子朝向 / #%d 电机极性 / #%d 计数方向" % (
                               OLDMAP.get(37, 37), OLDMAP.get(73, 73), OLDMAP.get(84, 84))], 1):
        cell = ws2.cell(row=r, column=c, value=v)
        cell.border = BORDER
        cell.alignment = CEN if c in (1, 2, 4, 5, 6, 7, 8) else TOP
    for c in (7, 8):
        ws2.cell(row=r, column=c).fill = PatternFill("solid", fgColor=XLIGHT)
nrow = 6
ws2.cell(row=nrow, column=1, value=(
    "【基准约定】X 型布局；车体 X 前、Y 左、ω 俯视逆时针为正；轮序 1=左前 2=右前 3=左后 4=右后。"))
ws2.cell(row=nrow + 1, column=1, value=(
    "【逆运动学】R·ω₁ = v_x − v_y − (l_x+l_y)ω ；R·ω₂ = v_x + v_y + (l_x+l_y)ω ；"
    "R·ω₃ = v_x + v_y − (l_x+l_y)ω ；R·ω₄ = v_x − v_y + (l_x+l_y)ω"))
ws2.cell(row=nrow + 2, column=1, value=(
    "【为什么只有一张表】这张表把原先分散在 #%d/#%d/#%d/#%d 四条的约定合成一套：辊子朝向 → 电机极性 → "
    "计数方向 → 方程符号。右列两格必须用实车测出来填上；只要有一格符号错，四个方程的符号全错，"
    "小车会「按指令的反方向走」，PID 再怎么调都没用。" % (
        OLDMAP.get(10, 10), OLDMAP.get(37, 37), OLDMAP.get(73, 73), OLDMAP.get(84, 84))))
ws2.cell(row=nrow + 3, column=1, value=(
    "【若第 #%d 条实判为 O 型】本表不适用：O 型是同轴两轮辊子方向相同，四条方程不再是上面这组，"
    "需要单独推导（有趣的是，那种布局反而能把 l_x 和 l_y 分别辨识出来）。" % OLDMAP.get(10, 10)))
for r in range(nrow, nrow + 4):
    ws2.cell(row=r, column=1).alignment = TOP
    ws2.merge_cells(start_row=r, start_column=1, end_row=r, end_column=9)
    ws2.row_dimensions[r].height = 30
for col, w in {"A": 10, "B": 12, "C": 26, "D": 10, "E": 10, "F": 14, "G": 26, "H": 28, "I": 34}.items():
    ws2.column_dimensions[col].width = w

# ================================================================ Sheet3 拍照清单
ws3 = wb.create_sheet("拍照清单")
H3 = ["编号", "拍摄对象 / 视图", "拍摄目的", "拍摄要求（角度·距离·参照物）", "优先级", "建议文件名", "状态", "备注"]
for c, n in enumerate(H3, 1):
    cell = ws3.cell(row=1, column=c, value=n)
    cell.font, cell.fill, cell.alignment, cell.border = HFONT, HFILL, HALIGN, BORDER
ws3.row_dimensions[1].height = 30
for i, row in enumerate(SHOTS):
    r = 2 + i
    for c, v in enumerate(row, 1):
        cell = ws3.cell(row=r, column=c, value=v)
        cell.border = BORDER
        cell.alignment = TOP
    ws3.cell(row=r, column=1).alignment = CEN
    ws3.cell(row=r, column=5).alignment = CEN
    ws3.cell(row=r, column=7, value="☐").alignment = CEN
    ws3.cell(row=r, column=7).border = BORDER
    ws3.cell(row=r, column=8, value=None).border = BORDER
for col, w in {"A": 8, "B": 30, "C": 30, "D": 56, "E": 9, "F": 26, "G": 8, "H": 26}.items():
    ws3.column_dimensions[col].width = w
ws3.freeze_panes = "A2"
last3 = 1 + len(SHOTS)
ws3.auto_filter.ref = f"A1:H{last3}"
dvp = DataValidation(type="list", formula1='"高,中,低"', allow_blank=True, showDropDown=False)
ws3.add_data_validation(dvp); dvp.add(f"E2:E{last3}")
dvs = DataValidation(type="list", formula1='"☐,☑"', allow_blank=True, showDropDown=False)
ws3.add_data_validation(dvs); dvs.add(f"G2:G{last3}")

# ================================================================ Sheet4 使用说明
ws4 = wb.create_sheet("使用说明")
ws4.column_dimensions["A"].width = 26
ws4.column_dimensions["B"].width = 112
t = ws4.cell(row=1, column=1, value="麦轮小车参数测量清单 · 使用说明（v2）")
t.font = Font(bold=True, size=14, color=XDARK)
ws4.merge_cells("A1:B1")
ws4.cell(row=1, column=1).alignment = Alignment(horizontal="left", vertical="center")
ws4.row_dimensions[1].height = 26

cats = []
for p in PARAMS:
    if p[0] not in cats:
        cats.append(p[0])
NOTES = [
    ("文档用途", "用于麦轮小车 PID 调参前的参数实测记录。「实测值（待填）」列由你测量后填写；填完交给 AI 即可继续建模、标定与整定。"),
    ("结构说明（MECE）", "参数按互斥且穷尽的 %d 个顶层类别划分：%s。" % (len(cats), " / ".join(cats))),
    ("单位约定（v2 统一）",
     "长度一律 mm；质量一律 g；转动惯量一律 kg·m²（若你用 g·mm² 算出中间结果，请除以 1e9 再填入，"
     "不要写成 kg·mm²，会差 1000 倍）；角度一律 °；转速一律 rpm；时间一律 ms 或 s（看单位列注明）。"),
    ("符号约定（务必先看）",
     "新增工作表「四轮符号对照表」把辊子朝向 → 电机极性 → 编码器计数方向 → 运动学方程符号合成一张表。"
     "这一套符号决定逆运动学每一项的正负；错一处，四个方程全错，小车会朝反方向走。"),
    ("可辨识性提示（重要）",
     "X 型布局下 l_x、l_y 恒以 (l_x+l_y) 的和出现，任何运动试验都只能标定出这个和（见 J1 组相关条目），"
     "不能分别反推 L_eff 与 W_eff；两者的单值只能靠几何实测（B 组轴距/轮距）。"),
    ("列含义", "数据来源：铭牌 / 手册 / 实测 / 计算 / 推断 / 整定。重要性：必需＝不测就无法建模或整定；重要＝影响精度与效果；可选＝有时间再补。备注里【v2 修正】/【v2 新增】标明了本次改了什么。"),
    ("重要性统计", "共 %d 条参数：必需 %d 条、重要 %d 条、可选 %d 条。" % (
        len(PARAMS),
        sum(1 for p in PARAMS if p[8] == "必需"),
        sum(1 for p in PARAMS if p[8] == "重要"),
        sum(1 for p in PARAMS if p[8] == "可选"))),
    ("质心计算最小输入集",
     "只要四轮静态载荷 N_i + 四轮轴心坐标（由 B 组轴距/轮距换算），即可直接算 x_c = Σ(N_i x_i)/ΣN_i、"
     "y_c = Σ(N_i y_i)/ΣN_i —— 这是主算法。D 组拆件称重与坐标是互校手段，不是必需的。"
     "称重时四个轮子必须垫等高垫块，否则 CoM 会算偏。"),
    ("拍照配合", "「拍照清单」工作表列出需要拍摄的全部视图与要求。照片放进本项目文件夹后，AI 会做去畸变量测并输出图片描述文档。"),
    ("建议执行顺序",
     "① 先定符号：填「四轮符号对照表」+ A 组坐标系；② 量几何：B 组轴距/轮距、C 组轮径/toe/辊子；"
     "③ 称重：D 组四轮载荷；④ 定电气：E/F/G 组死区、每米计数、IMU 零偏与标度因数；"
     "⑤ 最后做 J1 组的旋转 n 圈标定与直线/横移验证，再进 J2 组整定 PID。"),
    ("后续产出", "参数表填完 + 照片就位后输出：① 底盘速度 ↔ 四轮转速的转换方程（含符号表）；② 质心/载荷分配/转动惯量报告；③ MECE 误差因素 PDF；④ PID 整定方案与验证方法。"),
    ("v2 修订记录",
     "① 原点定义由「接地矩形中心」改为「四轮轴心俯视投影矩形中心」；"
     "② 惯量单位 kg·mm² → kg·m²；③ 原 L_eff / W_eff 两条合并为一条「有效几何常数 (l_x+l_y)/R_eff」；"
     "④ 第 32 条 e 的备注改为「仅为几何输入，R_eff 以标定值为准」；⑤ 第 30 条固定 β 的定义；"
     "⑥ 新增 toe 角（4 轮 + 极差）、陀螺仪标度因数、质心相对几何中心偏移与旋转轴约定、电机电气常数（可选）；"
     "⑦ 原 J 组拆成 J1 标定/测试输入 与 J2 PID 整定结果；"
     "⑧ 质心主算法改为四轮载荷法；⑨ 新增「四轮符号对照表」工作表。"),
]
r = 3
for k, v in NOTES:
    kc = ws4.cell(row=r, column=1, value=k)
    kc.font = Font(bold=True, color=XDARK, size=11)
    kc.alignment = TOP; kc.fill = PatternFill("solid", fgColor=XALT); kc.border = BORDER
    vc = ws4.cell(row=r, column=2, value=v)
    vc.alignment = TOP; vc.border = BORDER
    r += 1

wb.properties.title = "麦轮小车参数测量清单 v2"
wb.save(OUT)
wb.save(STAGE)
print("saved:", OUT, "参数 %d 条" % len(PARAMS))

# ---------------------------------------------------------------- 输出编号映射给后续脚本
json.dump({"old2new": {str(k): v for k, v in sorted(OLDMAP.items())},
           "count_new": len(PARAMS)},
          open(os.path.join(REF, "v2_refs.json"), "w"), ensure_ascii=False, indent=1)
print("map written; 新增 %d 条，净变化 %+d" % (len(PARAMS) - len(V1_NAMES), len(PARAMS) - len(V1_NAMES)))
