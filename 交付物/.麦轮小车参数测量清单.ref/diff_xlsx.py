"""两个 xlsx 的逐单元格差异；数值变化 ≥1 的单独标红列出。"""
import openpyxl, sys

a, b = sys.argv[1], sys.argv[2]
wa, wb = openpyxl.load_workbook(a), openpyxl.load_workbook(b)
ds = []
for sn in wa.sheetnames:
    if sn not in wb.sheetnames:
        ds.append((sn, "-", "**整个工作表**", "缺失/新增", "")); continue
    sa, sb = wa[sn], wb[sn]
    if (sa.max_row, sa.max_column) != (sb.max_row, sb.max_column):
        ds.append((sn, "-", "**尺寸**", f"{sa.max_row}×{sa.max_column}",
                   f"{sb.max_row}×{sb.max_column}", "尺寸"))
    for r in range(1, max(sa.max_row, sb.max_row) + 1):
        for c in range(1, max(sa.max_column, sb.max_column) + 1):
            x, y = sa.cell(r, c).value, sb.cell(r, c).value
            if x != y:
                hdr = sa.cell(1, c).value or sb.cell(1, c).value or f"列{c}"
                ds.append((sn, f"{sa.cell(r,1).value or r}", f"{hdr}", x, y, ""))
for sn in wb.sheetnames:
    if sn not in wa.sheetnames:
        ds.append((sn, "-", "**整个工作表**", "新增", "", "新增"))

print(f"差异单元格数 = {len(ds)}")
if len(ds) > 60:
    print("（超过 60 条，只列前 60）")
big = []
for sn, row, hdr, x, y, tag in ds[:60]:
    print(f"  [{sn}] 行{row} · {hdr}\n      old = {x!r}\n      new = {y!r}")
for sn, row, hdr, x, y, tag in ds:
    try:
        dx = abs(float(y) - float(x))
        if dx >= 1.0:
            big.append((sn, row, hdr, x, y, dx))
    except (TypeError, ValueError):
        pass
print(f"\n>>> 数值变化 ≥1 的单元格：{len(big)} 个")
for sn, row, hdr, x, y, dx in big:
    print(f"  ⚠ [{sn}] 行{row} {hdr}: {x} → {y}  (Δ={dx:g})  {'★≥1 mm' if dx>=1 else ''}")
