#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画 A1（量 ID29 偏置）示意图。参照点选「左前轮轴心」——它在车外，看得见摸得着。"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle
import matplotlib.font_manager as fm

FONT = fm.FontProperties(fname="/home/hanli/.local/share/fonts/windows-fonts-extra/Noto Sans SC (TrueType).otf")

WHEELBASE = 128.25      # 轴距 (mm) = 固件 main.cpp:144 的 LXY
TRACK     = 119.0       # 轮距 (mm)（HANDOFF 前 120 / 后 118 的均值）
LEN, WID  = 270.0, 149.0
M_BACK, M_IN = 71.0, 1.5   # 约值，实数以你量的为准

fig = plt.figure(figsize=(16.5, 9.6), dpi=110)
fig.patch.set_facecolor("white")
gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1.0], height_ratios=[1.32, 1.0],
                      left=0.03, right=0.98, top=0.90, bottom=0.04, hspace=0.12, wspace=0.14)

# ================= 左上：俯视图 =================
ax = fig.add_subplot(gs[0, 0])
ax.set_aspect("equal"); ax.axis("off")
ax.set_xlim(-LEN/2 - 46, LEN/2 + 46)
ax.set_ylim(-WID/2 - 108, WID/2 + 72)

ax.add_patch(FancyBboxPatch((-LEN/2, -WID/2), LEN, WID,
                            boxstyle="round,pad=0,rounding_size=14",
                            fc="#eef2f6", ec="#64748b", lw=2.0, zorder=2))

A = (WHEELBASE/2, TRACK/2)
M = (A[0]-M_BACK, A[1]-M_IN)

for sx in (1, -1):
    for sy in (1, -1):
        wx, wy = sx*WHEELBASE/2, sy*TRACK/2
        ax.add_patch(Circle((wx, wy), 23, fc="#334155", ec="#0f172a", lw=1.4, zorder=3))
        name = {(1,1):"FL", (-1,1):"RL", (1,-1):"FR", (-1,-1):"RR"}[(sx, sy)]
        ax.text(wx, wy + (31 if sy > 0 else -38), name, fontproperties=FONT,
                fontsize=11.5, color="#0f172a", ha="center", zorder=6, fontweight="bold")

ax.plot([-WHEELBASE/2, WHEELBASE/2], [TRACK/2, TRACK/2],
        ls=":", color="#0284c7", lw=2.0, zorder=4)
ax.add_patch(Rectangle((-WHEELBASE/2, -TRACK/2), WHEELBASE, TRACK,
                       fc="none", ec="#0ea5e9", lw=1.5, ls="--", zorder=4))
ax.text(0, -TRACK/2 - 46, f"四轮轴心矩形   轴距 {WHEELBASE:.1f}  ×  轮距 {TRACK:.0f}",
        fontproperties=FONT, fontsize=10.5, color="#0284c7", ha="center", zorder=6)

ax.plot([A[0]], [A[1]], marker="o", ms=14, mfc="none", mec="#dc2626", mew=3, zorder=8)
ax.plot([A[0]], [A[1]], marker="+", ms=9, mew=2.0, color="#dc2626", zorder=9)
ax.text(A[0], A[1]+46, "A = 左前轮轴心", fontproperties=FONT, fontsize=11.5,
        color="#dc2626", ha="center", zorder=9, fontweight="bold")

ax.add_patch(Rectangle((M[0]-15, M[1]-15), 30, 30, fc="#111827", ec="#111827",
                       lw=1.0, zorder=6))
for q in ((-8.5, 8.5), (8.5, 8.5), (8.5, -8.5), (-8.5, -8.5)):
    ax.add_patch(Rectangle((M[0]+q[0]-3.0, M[1]+q[1]-3.0), 6.0, 6.0,
                           fc="white", ec="none", zorder=7))
ax.text(M[0], M[1]-24, "ID29", fontproperties=FONT, fontsize=12, color="#111827",
        ha="center", zorder=8, fontweight="bold")

ax.annotate("", xy=(LEN/2+34, -WID/2+18), xytext=(LEN/2+6, -WID/2+18),
            arrowprops=dict(arrowstyle="->", color="#16a34a", lw=3.0))
ax.text(LEN/2+18, -WID/2+27, "车头 FWD", fontproperties=FONT, fontsize=11,
        color="#16a34a", ha="center", fontweight="bold")
ax.annotate("", xy=(-LEN/2-34, -WID/2+18), xytext=(-LEN/2-6, -WID/2+18),
            arrowprops=dict(arrowstyle="->", color="#16a34a", lw=3.0))
ax.text(-LEN/2-18, -WID/2+27, "车左 LEFT", fontproperties=FONT, fontsize=11,
        color="#16a34a", ha="center", fontweight="bold")

# 两个量法说明，画在车下方空白处
ax.text(-150, -WID/2-74, "①  从 A（FL 轮轴心）往车尾，量到 ID29 的中心线",
        fontproperties=FONT, fontsize=12.5, color="#dc2626", ha="left", zorder=10,
        fontweight="bold")
ax.text(-150, -WID/2-92, "②  从 ① 的落点，往车内侧，量到 ID29 的中心线",
        fontproperties=FONT, fontsize=12.5, color="#dc2626", ha="left", zorder=10,
        fontweight="bold")
ax.text(-150, -WID/2-104, "（② 很小，量不准也没关系 —— A2 会把它一起验掉）",
        fontproperties=FONT, fontsize=10.5, color="#94a3b8", ha="left", zorder=10,
        style="italic")

ax.set_title("俯视图：只要量 ① 和 ②", fontproperties=FONT, fontsize=16,
             fontweight="bold", pad=6)

# ================= 右上：量法步骤 =================
ax2 = fig.add_subplot(gs[0, 1]); ax2.axis("off")
ax2.set_xlim(0, 100); ax2.set_ylim(0, 100)
ax2.text(50, 95, "怎么量", fontproperties=FONT, fontsize=17,
         fontweight="bold", ha="center")
steps = [
    ("找 A", "俯视找到左前轮的中心点\n（轮子最外沿往里 轮宽/2）"),
    ("量 ①", "从 A 往车尾量到 ID29 中心线"),
    ("量 ②", "从落点往车内侧量到 ID29 中心线"),
    ("顺带报", "轮距、轴距（我用来校验）"),
]
for i, (tag, txt) in enumerate(steps):
    y = 76 - i*20
    ax2.add_patch(Circle((8, y), 5.6, fc="#0284c7", ec="none"))
    ax2.text(8, y, str(i+1), fontproperties=FONT, fontsize=13.5, color="white",
             ha="center", va="center", fontweight="bold")
    ax2.text(18, y+4, tag, fontproperties=FONT, fontsize=13, color="#0284c7",
             fontweight="bold", va="center")
    ax2.text(18, y-5, txt, fontproperties=FONT, fontsize=11.5, color="#1e293b",
             va="center", linespacing=1.6)
ax2.text(50, 3, "只用车上/旁边有的东西就行 —— 没有卡尺也做得了",
         fontproperties=FONT, fontsize=11, color="#64748b", ha="center", style="italic")

# ================= 左下：换算示意 =================
ax3 = fig.add_subplot(gs[1, 0]); ax3.set_aspect("equal"); ax3.axis("off")
ax3.set_xlim(-70, 215); ax3.set_ylim(-72, 62)
ax3.text(72, 54, "我拿到 ① ② 之后怎么换算成车体原点 O",
         fontproperties=FONT, fontsize=15, fontweight="bold", ha="center")

ax3.add_patch(Rectangle((-58, -40), 208, 46, fc="#f1f5f9", ec="#94a3b8", lw=1.2))
for x, lab, y in ((-24, "FL", 6), (118, "FR", 6), (-24, "RL", -40), (118, "RR", -40)):
    ax3.add_patch(Circle((x, y), 10, fc="#334155", ec="#0f172a"))
    ax3.text(x, y, lab, fontproperties=FONT, fontsize=8, color="white",
             ha="center", va="center")
ax3.plot([-38, 132], [6, 6], ls=":", color="#0284c7", lw=1.6)
ax3.plot([-38, 132], [-40, -40], ls=":", color="#0284c7", lw=1.6)
ax3.text(136, 6, "前轴心线", fontproperties=FONT, fontsize=9.5, color="#0284c7", va="center")
ax3.text(136, -40, "后轴心线", fontproperties=FONT, fontsize=9.5, color="#0284c7", va="center")

ax3.plot([-24], [6], marker="o", ms=11, mfc="none", mec="#dc2626", mew=2.6, zorder=8)
ax3.text(-38, 6, "A", fontproperties=FONT, fontsize=12, color="#dc2626",
         ha="center", va="center", fontweight="bold")
O3 = (47, -17)
ax3.plot([O3[0]], [O3[1]], marker="+", ms=14, mew=2.4, color="#7c3aed", zorder=8)
ax3.text(O3[0], O3[1]-8, "O 车体原点", fontproperties=FONT, fontsize=10.5,
         color="#7c3aed", ha="center", va="top", fontweight="bold")
ax3.annotate("", xy=(-24, 28), xytext=(O3[0], 28),
             arrowprops=dict(arrowstyle="<->", color="#7c3aed", lw=2.2))
ax3.text(11, 33, "轴距 ÷ 2", fontproperties=FONT, fontsize=10.5,
         color="#7c3aed", ha="center")
ax3.annotate("", xy=(O3[0], 6), xytext=(O3[0], -40),
             arrowprops=dict(arrowstyle="<->", color="#7c3aed", lw=2.2))
ax3.text(O3[0]+8, -17, "轮距 ÷ 2", fontproperties=FONT, fontsize=10.5,
         color="#7c3aed", va="center")
ax3.text(72, -62, "偏置 = ID29 的位置 − O 的位置", fontproperties=FONT,
         fontsize=11.5, color="#334155", ha="center")

# ================= 右下：你要报什么 =================
ax4 = fig.add_subplot(gs[1, 1]); ax4.axis("off")
ax4.set_xlim(0, 100); ax4.set_ylim(0, 100)
ax4.text(50, 94, "做完 A1，把这四个数发我", fontproperties=FONT, fontsize=17,
         fontweight="bold", ha="center")
rows = [
    ("①", "从 A 往车尾到 ID29 中心线的距离", "mm"),
    ("②", "从 ① 落点往车内侧到 ID29 中心线的距离", "mm"),
    ("③", "轮距（左前 ↔ 右前 轮心距）", "mm"),
    ("④", "轴距（左前 ↔ 左后 轮心距）", "mm"),
]
for i, (n, t, u) in enumerate(rows):
    y = 74 - i*19
    ax4.add_patch(FancyBboxPatch((2, y-7.5), 96, 15,
                                 boxstyle="round,pad=0,rounding_size=2",
                                 fc="#f8fafc", ec="#cbd5e1", lw=1.1))
    ax4.add_patch(Circle((10, y), 4.6, fc="#dc2626", ec="none"))
    ax4.text(10, y, n, fontproperties=FONT, fontsize=11, color="white",
             ha="center", va="center", fontweight="bold")
    ax4.text(19, y, t, fontproperties=FONT, fontsize=11.5, color="#1e293b", va="center")
    ax4.text(94, y, u, fontproperties=FONT, fontsize=10.5, color="#94a3b8",
             ha="right", va="center")
ax4.text(50, 3, "朝向偏置 dtheta 先不用量 —— 车进场地后我跑 --explain-bias 给你查表",
         fontproperties=FONT, fontsize=10.5, color="#64748b", ha="center", style="italic")

fig.suptitle("A1 —— 量 ID29 偏置（车架着就能做，只要一把卷尺/直尺）",
             fontproperties=FONT, fontsize=18, fontweight="bold", y=0.965)
fig.savefig("/tmp/opencode/A1_measurement.png", facecolor="white")
print("saved")