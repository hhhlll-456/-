# 《麦轮小车参数测量清单》构建管线（参考实现）

交付物 `交付物/麦轮小车参数测量清单.xlsx` 是**生成物** —— **不手工编辑**。

## 纪律（硬性）

1. **不写公式**：工作簿里只放字面值；生成器一律 `openpyxl` 直写（`build.py` 顶部注释即此约定）。
2. **不手工编辑 xlsx**：任何内容变更都改本目录的脚本 / 输入，再重跑生成器。
   拿 Excel 打开存盘一次就会**丢掉图片、条件格式与数据验证**。
3. **不静默覆盖交付物数字**：改数前先做一次**单元格级差异**（见《改数流程》），把 old → new 全列出来。

## 管线

```
build.py           v1 数据模块（PARAMS / SHOTS）。**不直接运行** —— build_v2.py 读它的源码取数据。
  │
v1_备份.xlsx       v1 生成物 + 用户手填的实测值。*历史备份*（已 gitignore）
  │
build_v2.py        ← build.py 的 PARAMS/SHOTS + v1_备份.xlsx 的已填值
  │                → 修正/插入/合并条目、重排编号、加「对应图示」列与「四轮符号对照表」
  │                → 写 v2_stage.xlsx **和** <仓库根>/麦轮小车参数测量清单.xlsx
  │                → 写 v2_refs.json（旧序号 → 新序号映射，供 gen_figs.py 与 md 同步编号）
v2_stage.xlsx      ★ 阶段输入（**含用户已填实测值**）—— 入库
v1_values.json     v2 用到的取值表 —— 入库
v2_refs.json       编号映射 —— 入库
  │
build_v3.py        ← v2_stage.xlsx → 按「标量 / 四轮表 / 一次性检查 / PID 整定 / 附录」重排
  │
build_v4.py        ← v2_stage.xlsx（优先）→ 补 F 组「标定与耦合验证」5 条、起转死区拆正/反向、
  │                  新增阶跃时间常数 τ_m、新增几何自洽性检查
  └──→ 交付物/麦轮小车参数测量清单.xlsx     ★ 交付物（入库）
```

## ⚠ 各脚本的路径现状（跑之前先看）

| 脚本 | ROOT / REF / OUT 指向 | 现在能直接跑吗 |
|---|---|---|
| `build_v4.py` | `交付物/.ref` → `交付物/麦轮小车参数测量清单.xlsx` | ✅ **能** —— 且**只有它**会写交付物 |
| `build.py` / `build_v2.py` / `build_v3.py` | 仓库根 `.ref` → 仓库根 xlsx（**已过时**，目录早已搬到 `交付物/` 下） | ❌ 会写到仓库根；需先把它们的 `REF` 改指 `交付物/.ref` |

即：**只有要重建 `v2_stage.xlsx` 时才需要动 v2/v3（且要先改 `REF`）；日常重建交付物只需 `build_v4.py`。**
同理 `build_v2.py` 读的 `v1_备份.xlsx` 也已 gitignore —— 但**不需要**它，因为
`v2_stage.xlsx` 已入库、其中已含用户填的实测值。

## 重建交付物

```bash
cd /home/hanli/麦轮小车pid
PYTHONPATH=~/.local/lib/python3.12/site-packages \
  python3 交付物/.麦轮小车参数测量清单.ref/build_v4.py
```

`build_v4.py` 优先读 `v2_stage.xlsx`；**只有它不存在时**才回落到读交付物本身，并把读到的快照
另存回 `v2_stage.xlsx`（第 32–34 行）—— 所以两种情形都不会丢已填值。

## 改数流程（不许静默覆盖）

1. 备份当前交付物：`cp 交付物/麦轮小车参数测量清单.xlsx /tmp/before.xlsx`
2. 改 `v2_stage.xlsx` 里的值（**用脚本改**，别用 Excel 存盘）
3. 重跑 `build_v4.py`
4. 逐单元格 diff `/tmp/before.xlsx` vs 新交付物 → **old → new 全列出**；
   **派生量**（凡进公式的）变更 **≥1 mm** 的必须单列并说明来源
5. 差异写进 commit message，再提交

## 备份文件不入库

`v1_备份.xlsx` / `v3_备份.xlsx` / `v4_备份_*.xlsx` 是各轮的历史快照，已 gitignore
（见仓库根 `.gitignore` 的 `交付物/.麦轮小车参数测量清单.ref/*_备份*.xlsx`）。
