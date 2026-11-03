#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画 A2 量法说明（第二版）：重点讲清【① ② 到底量哪两个点】。

上一版只画了全场的两条线，操作者照着量出来的 ② 与视觉解差 12.8 mm。
12.8 mm 这个量级正好等于「把黑格中心连线当成了白纸外沿」或「量到了车身侧沿/悬空的码」
这类**参照点取错**能造成的量，所以这一版把两处放大画出来：

    放大①：顶边 = 过 ID20/ID21 **黑格中心**的连线 —— 不是白纸外沿，不是桌面画线
    放大②：② 读到**后轮接地点**（在桌面上）—— 不是车身侧沿，不是悬空的 ID29 码

场地系（R4 / R34）：
    原点 O = 四个边界码质心；+x = ID20→ID21；+y = ID20→ID23；yaw=0 表示车头指向 +x
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, FancyArrowPatch
import matplotlib.font_manager as fm

FONT = fm.FontProperties(fname="/home/hanli/.local/share/fonts/windows-fonts-extra/Noto Sans SC (TrueType).otf")

FLD_L, FLD_W = 503.8, 251.0      # 场地长/宽（视觉解算实测）
HALF_W, HALF_L = FLD_W / 2, FLD_L / 2
WHEELBASE, TRACK = 140.0, 117.0
LEN, WID = 270.0, 149.0
HALF_WB = WHEELBASE / 2

CAR_X, CAR_Y = -29.0, -20.6       # 示例：车体原点在场地系的位置
P_X, P_Y = CAR_X - HALF_WB, CAR_Y

fig = plt.figure(figsize=(18.0, 11.4), dpi=105)
fig.patch.set_facecolor("white")
gs = fig.add_gridspec(2, 3, width_ratios=[1.30, 0.95, 0.95], height_ratios=[1.0, 0.92],
                      left=0.018, right=0.985, top=0.905, bottom=0.025,
                      hspace=0.20, wspace=0.16)

fig.text(0.018, 0.968, "A2 量法 —— 量的是【后轮接地点 P】到【ID20–ID21 黑格中心连线】的两条线",
         fontproperties=FONT, fontsize=20, color="#0f172a", fontweight="bold")
fig.text(0.018, 0.936,
         "上一版量出来的 ② 与视觉差 12.8 mm —— 下面两张放大图就是查这个用的",
         fontproperties=FONT, fontsize=12.5, color="#b91c1c", fontweight="bold")

# ═══════════════════ 上：全场俯视图 ═══════════════════
ax = fig.add_subplot(gs[0, :])
ax.set_aspect("equal"); ax.axis("off")
ax.set_xlim(-HALF_L - 112, HALF_L + 112)
ax.set_ylim(HALF_W + 86, -HALF_W - 74)

ax.add_patch(Rectangle((-HALF_L, -HALF_W), FLD_L, FLD_W,
                       fc="#f8fafc", ec="#94a3b8", lw=1.8, zorder=1))
MK = {20: (-HALF_L, -HALF_W), 21: (HALF_L, -HALF_W),
      22: (HALF_L, HALF_W), 23: (-HALF_L, HALF_W)}
MCOL = {20: "#dc2626", 21: "#16a34a", 22: "#ea580c", 23: "#c026d3"}
for mid, (mx, my) in MK.items():
    ax.add_patch(Rectangle((mx - 13, my - 13), 26, 26, fc="#0f172a", ec=MCOL[mid], lw=2.4, zorder=6))
    dy = 22 if my < 0 else -22          # 上排标签放进场内，下排推到场外
    ax.text(mx, my + dy, f"ID{mid}", fontproperties=FONT, fontsize=13, color=MCOL[mid],
            ha="center", va=("bottom" if my < 0 else "top"), zorder=8, fontweight="bold")

# 顶边：过两个黑格中心的连线（不是白纸外沿）
ax.plot([-HALF_L - 100, HALF_L + 100], [-HALF_W, -HALF_W], ls="--", lw=2.0, color="#2563eb", zorder=4)
ax.text(0, -HALF_W - 12, "蓝色虚线 = 顶边（过 ID20、ID21 两个黑格中心）", fontproperties=FONT,
        fontsize=11, color="#2563eb", ha="center", va="top", zorder=5, fontweight="bold")
ax.plot([0], [-HALF_W], marker="o", ms=12, mfc="none", mec="#2563eb", mew=2.8, zorder=9)
ax.text(0, -HALF_W - 36, "M = ID20 与 ID21 的中点", fontproperties=FONT, fontsize=11.5,
        color="#2563eb", ha="center", va="top", zorder=9, fontweight="bold")
ax.plot([0, 0], [-HALF_W, 0], ls=":", lw=1.6, color="#2563eb", zorder=4)
ax.plot([0], [0], marker="o", ms=14, mfc="none", mec="#0f172a", mew=2.6, zorder=9)
ax.plot([0], [0], marker="+", ms=16, color="#0f172a", mew=2.4, zorder=9)
ax.text(11, -3, "O 场地原点\n= 四码质心", fontproperties=FONT, fontsize=11,
        color="#0f172a", ha="left", va="top", zorder=9, fontweight="bold")

ax.arrow(-HALF_L - 96, -HALF_W, 54, 0, color="#dc2626", lw=3.2, head_width=8, head_length=12, zorder=5)
ax.text(-HALF_L - 36, -HALF_W, "+x", fontproperties=FONT, fontsize=15, color="#dc2626",
        ha="left", va="center", zorder=5, fontweight="bold")
ax.arrow(-HALF_L - 42, -HALF_W, 0, 46, color="#16a34a", lw=3.2, head_width=8, head_length=12, zorder=5)
ax.text(-HALF_L - 42, -HALF_W + 54, "+y", fontproperties=FONT, fontsize=15, color="#16a34a",
        ha="center", va="top", zorder=5, fontweight="bold")

ax.add_patch(FancyBboxPatch((CAR_X - LEN / 2, CAR_Y - WID / 2), LEN, WID,
                            boxstyle="round,pad=0,rounding_size=12",
                            fc="#eef2f6", ec="#475569", lw=2.0, zorder=2))
for sx in (1, -1):
    for sy in (1, -1):
        ax.add_patch(Circle((CAR_X + sx * HALF_WB, CAR_Y + sy * TRACK / 2), 20,
                            fc="#334155", ec="#0f172a", lw=1.4, zorder=3))
ax.plot([P_X], [P_Y], marker="o", ms=18, mfc="none", mec="#7c3aed", mew=3.2, zorder=10)
ax.plot([P_X], [P_Y], marker="o", ms=5, color="#7c3aed", zorder=11)
ax.plot([CAR_X], [CAR_Y], marker="+", ms=20, mew=3.2, color="#dc2626", zorder=10)
ax.arrow(CAR_X + LEN / 2 + 6, CAR_Y, 38, 0, color="#0f172a", lw=2.6, head_width=7, head_length=11, zorder=6)
ax.text(CAR_X + LEN / 2 + 50, CAR_Y, "车头 = +x = yaw 0°", fontproperties=FONT, fontsize=12,
        color="#0f172a", ha="left", va="center", zorder=6, fontweight="bold")

Y1, X2 = HALF_W + 46, HALF_L + 52
ax.plot([P_X, P_X], [P_Y, Y1], ls=":", lw=1.7, color="#2563eb", zorder=3)
ax.annotate("", xy=(0, Y1), xytext=(P_X, Y1),
            arrowprops=dict(arrowstyle="<->", color="#2563eb", lw=3.2), zorder=8)
ax.text((P_X + 0) / 2, Y1 - 4, "①", fontproperties=FONT, fontsize=18, color="#2563eb",
        ha="center", va="top", zorder=9, fontweight="bold")
ax.text((P_X + 0) / 2, Y1 - 30, "P 在 M 的【左 · 朝 ID20】= 负数\nP 在 M 的【右 · 朝 ID21】= 正数",
        fontproperties=FONT, fontsize=11.5, color="#2563eb", ha="center", va="top", zorder=9)

ax.plot([P_X, X2], [P_Y, P_Y], ls=":", lw=1.7, color="#16a34a", zorder=3)
ax.annotate("", xy=(X2, -HALF_W), xytext=(X2, P_Y),
            arrowprops=dict(arrowstyle="<->", color="#16a34a", lw=3.2), zorder=8)
ax.text(X2 - 10, -70, "② 垂直距离\n（顶边 → P）", fontproperties=FONT, fontsize=11.5,
        color="#16a34a", ha="right", va="center", zorder=9, fontweight="bold")
ax.annotate("", xy=(CAR_X, P_Y - 34), xytext=(P_X, P_Y - 34),
            arrowprops=dict(arrowstyle="->", color="#dc2626", lw=3.0), zorder=8)
ax.text((P_X + CAR_X) / 2, P_Y - 40, "+70", fontproperties=FONT, fontsize=12.5, color="#dc2626",
        ha="center", va="top", zorder=9, fontweight="bold")
ax.text(P_X - 8, P_Y - 22, "P", fontproperties=FONT, fontsize=12.5, color="#7c3aed",
        ha="right", va="bottom", zorder=10, fontweight="bold")
ax.text(CAR_X + 8, P_Y - 22, "车体原点", fontproperties=FONT, fontsize=11.5, color="#dc2626",
        ha="left", va="bottom", zorder=10, fontweight="bold")

# ═══════════════════ 左下：放大① 顶边怎么取 ═══════════════════
z1 = fig.add_subplot(gs[1, 0])
z1.set_aspect("equal"); z1.axis("off")
z1.set_xlim(-38, 52); z1.set_ylim(-34, 74)
z1.set_title("放大 ①　顶边这条线怎么取", fontproperties=FONT, fontsize=13.5,
             color="#0f172a", fontweight="bold", pad=6)
z1.add_patch(Rectangle((-20.5, -20.5), 41, 41, fc="white", ec="#cbd5e1", lw=1.8, zorder=2))
z1.add_patch(Rectangle((-12.5, -12.5), 25, 25, fc="#0f172a", ec="#dc2626", lw=2.2, zorder=3))
z1.text(-12.5, 15, "黑格 25 mm", fontproperties=FONT, fontsize=10.5, color="#dc2626",
        ha="left", va="bottom", zorder=4)
z1.text(-20.5, -23, "白纸外沿", fontproperties=FONT, fontsize=10.5, color="#64748b",
        ha="left", va="top", zorder=4)
z1.plot([-34, 40], [0, 0], ls="--", lw=2.2, color="#2563eb", zorder=5)
z1.text(-36, 3, "顶边（正确）", fontproperties=FONT, fontsize=10.5, color="#2563eb",
        ha="left", va="bottom", zorder=6, fontweight="bold")
z1.annotate("", xy=(0, 0), xytext=(0, 12), arrowprops=dict(arrowstyle="-|>", color="#dc2626", lw=2.4), zorder=7)
z1.plot([0], [0], marker="o", ms=13, mfc="none", mec="#dc2626", mew=2.6, zorder=8)
z1.annotate("", xy=(24, 8), xytext=(24, 0),
            arrowprops=dict(arrowstyle="<->", color="#dc2626", lw=2.0), zorder=7)
z1.text(30, 5, "白纸边比\n黑格中心\n靠外 6–8", fontproperties=FONT, fontsize=10,
        color="#dc2626", ha="left", va="center", zorder=8, fontweight="bold")
z1.text(-36, 72, "✓ 卷尺端头/直尺压在【黑格中心】，不是白纸边、\n   也不是桌面上画的线",
        fontproperties=FONT, fontsize=11, color="#15803d", ha="left", va="top", zorder=9, fontweight="bold")
z1.text(-36, 58, "× 用白纸外沿当顶边 → 每边少 6–8 mm\n   这就是上一版 12.8 mm 的头号嫌疑",
        fontproperties=FONT, fontsize=11, color="#b91c1c", ha="left", va="top", zorder=9)

# ═══════════════════ 中下：放大② 尺子读到哪 ═══════════════════
z2 = fig.add_subplot(gs[1, 1])
z2.set_aspect("equal"); z2.axis("off")
z2.set_xlim(-70, 62); z2.set_ylim(-34, 78)
z2.set_title("放大 ②　尺子读到哪一个点", fontproperties=FONT, fontsize=13.5,
             color="#0f172a", fontweight="bold", pad=6)
for i, ln in enumerate([
        "✓ 量这里：后轮【接地点】（在桌面上）",
        "× 车身侧沿：离中心线 74.5 mm，偏 74.5 mm",
        "× ID29 码中心：悬空 55–75 mm，端头贴不准",
        "左右两个后轮都要用，取它们的中点 = P"]):
    col = "#15803d" if ln.startswith("✓") else ("#7c3aed" if "取它们的中点" in ln else "#b91c1c")
    z2.text(-68, 76 - i * 12, ln, fontproperties=FONT, fontsize=11, color=col,
            ha="left", va="top", zorder=9, fontweight="bold")
z2.plot([-70, 62], [0, 0], lw=3.0, color="#a16207", zorder=2)
z2.text(60, 3, "桌面", fontproperties=FONT, fontsize=10.5, color="#a16207",
        ha="right", va="bottom", zorder=3)
z2.add_patch(Rectangle((-4, -6), 36, 5, fc="#dcfce7", ec="#16a34a", lw=1.6, zorder=3))
z2.annotate("", xy=(32, 0), xytext=(-4, 0),
            arrowprops=dict(arrowstyle="<|-|>", color="#16a34a", lw=2.6), zorder=8)
z2.text(14, -11, "卷尺平贴桌面", fontproperties=FONT, fontsize=10.5, color="#16a34a",
        ha="center", va="top", zorder=9)
z2.add_patch(Circle((32, 21), 21, fc="#334155", ec="#0f172a", lw=1.6, zorder=4))
z2.plot([32], [21], ls="--", lw=1.6, color="#334155", zorder=5)
z2.plot([32], [0], marker="o", ms=15, mfc="none", mec="#16a34a", mew=3.2, zorder=8)
z2.plot([32], [0], marker="o", ms=5, color="#16a34a", zorder=9)
z2.text(58, 0, "P", fontproperties=FONT, fontsize=13, color="#16a34a",
        ha="left", va="center", zorder=9, fontweight="bold")

# ═══════════════════ 右下：公式 + 自检 ═══════════════════
z3 = fig.add_subplot(gs[1, 2]); z3.axis("off")
z3.set_xlim(0, 1); z3.set_ylim(0, 1)
y = 1.0
def block(y, title, lines, color="#0f172a", size=12.0, gap=0.072):
    z3.text(0, y, title, fontproperties=FONT, fontsize=13.2, color=color, fontweight="bold", va="top")
    for i, ln in enumerate(lines):
        z3.text(0, y - 0.10 - i * gap, ln, fontproperties=FONT, fontsize=size, color="#334155", va="top")
    return y - 0.10 - len(lines) * gap - 0.05

y = block(y, "① 和 ② 怎么读", [
    "① = 沿顶边，M → P。P 在 M 左边（朝 ID20）记【负】。",
    "② = 从【ID20 中心—ID21 中心】那条线，垂直往下到 P。",
])
y = block(y, "换算成真值", [
    "x_truth = ① + 70        （70 = 轴距 140 ÷ 2）",
    "y_truth = ② − 125.5        （125.5 = W/2）",
    "yaw_truth = 0",
], color="#b91c1c")
y = block(y, "★ 自检（先量这两个，再量 ①②）", [
    "左端 ID20 中心 → ID23 中心 = 场地短边",
    "    视觉解：246.9 mm（ID21→ID22 是 255.1）",
    "ID20 中心 → ID22 中心 = 对角线 ≈ 565 mm",
], color="#0f766e", size=11.4, gap=0.062)
z3.add_patch(Rectangle((0, y - 0.30), 1.0, 0.285, fc="#fef3c7", ec="#d97706", lw=1.7))
z3.text(0.03, y - 0.265,
         "量完请拍一张卷尺在场的照片。\n我要确认三件事：卷尺压在哪条线上、\n读的是哪两个点、尺子有没有贴平。",
         fontproperties=FONT, fontsize=11.4, color="#92400e", va="top")

fig.savefig("/home/hanli/麦轮小车pid/视觉测量/结果/A2_量法说明.png", dpi=105, facecolor="white")
print("saved 视觉测量/结果/A2_量法说明.png")