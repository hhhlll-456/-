#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""由 wheelmap_{airborne|grounded}.json 生成完整 wheel_map.h 源码（Task 7 Step 4）。

用法：
    python3 wheelmap_gen.py --source ground            # 打印到 stdout（人工检查）
    python3 wheelmap_gen.py --source air --out 固件/exp/data/wheel_map_airborne.h

输出的头文件是**可直接替换 固件/exp/src/wheel_map.h 的完整文件**（结构、函数签名、
`MIN_DUTY`、`invert[4]`、插值/映射逻辑都与旧文件保持一致；只换数据与注释）。

【命名/符号约定 —— 换表前必须看懂这一段】

本文件里有两套容易混淆的"FWD/BWD"命名框架，生成器把它们对齐如下：

1. JSON（采集侧，本工具族）：
   - `omega_fwd[w]` = 该轮按**行进正向**（正 ω_des，firmware 里 `wheelOmegaToPWM`
     的 w≥0 方向）驱动时的稳态轮速；`omega_bwd` = 反向。
   - 对 FL/RL，行进正向 = 板级**负** PWM；对 FR/RR = 板级**正** PWM
     （invert[4]={T,F,T,F}，与 固件/历史/kpwm 的 FWD_SIGN={-1,+1,-1,+1} 一致）。
2. 头文件（执行侧）：`X_FWD` / `X_BWD` 是**板级 PWM 极性**（旧表注释所谓
   "物理正/负占空比曲线"）。固件对 ω_des≥0 用 `pos_tables[i]`、
   对 ω_des<0 用 `neg_tables[i]`，再经 `invert[i]` 翻转板级符号。
   ⇒ FL/RL 的"前进"落在板级负，所以 `FL_BWD`/`RL_BWD` 装的是 `omega_fwd`，
   `FR_FWD`/`RR_FWD` 装的也是 `omega_fwd`（右轮前进=板级正）。

映射表（本文件硬编码，改动前先读上面两段）：
    FL_BWD ← omega_fwd[FL]     FR_FWD ← omega_fwd[FR]
    RL_BWD ← omega_fwd[RL]     RR_FWD ← omega_fwd[RR]
    FL_FWD ← omega_bwd[FL]     FR_BWD ← omega_bwd[FR]
    RL_FWD ← omega_bwd[RL]     RR_BWD ← omega_bwd[RR]

【安全门】若 JSON 带非空 `invalid` 列表（采集时"实际 pwm≠请求 / 样本不足 /
不稳定"的档位），本生成器**拒绝生成**并退出 2 —— 防静默写错表（裁决 R10）。
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
WHEEL_NAMES = ["FL", "FR", "RL", "RR"]

# 头文件数组 ←（轮号, JSON 键, 注释）。见模块 docstring 的映射表。
ARRAYS = [
    ("FL_BWD", 0, "omega_fwd", "FL 前进方向（ω_des>0）；左轮前进=板级负（invert[0]=true）"),
    ("FR_FWD", 1, "omega_fwd", "FR 前进方向（ω_des>0）；右轮前进=板级正"),
    ("RL_BWD", 2, "omega_fwd", "RL 前进方向（ω_des>0）；左轮前进=板级负（invert[2]=true）"),
    ("RR_FWD", 3, "omega_fwd", "RR 前进方向（ω_des>0）；右轮前进=板级正"),
    ("FL_FWD", 0, "omega_bwd", "FL 后退方向（ω_des<0）；板级正"),
    ("FR_BWD", 1, "omega_bwd", "FR 后退方向（ω_des<0）；板级负"),
    ("RL_FWD", 2, "omega_bwd", "RL 后退方向（ω_des<0）；板级正"),
    ("RR_BWD", 3, "omega_bwd", "RR 后退方向（ω_des<0）；板级负"),
]

DEFAULT_JSON = {
    "air": os.path.join(REPO, "固件", "exp", "data", "wheelmap_airborne.json"),
    "ground": os.path.join(REPO, "固件", "exp", "data", "wheelmap_grounded.json"),
}
SOURCE_DESC = {"air": "架空套（诊断用：车抬起、四轮悬空）",
               "ground": "落地套（控制用：车放场地、四轮着地）"}


def load(path):
    if not os.path.exists(path):
        raise SystemExit(f"[gen] 找不到 {path}：先跑 wheelmap_sweep.py --mode ...")
    with open(path, encoding="utf-8") as f:
        obj = json.load(f)
    for key in ("c_1rev", "duty", "omega_fwd", "omega_bwd"):
        if key not in obj:
            raise SystemExit(f"[gen] {path} 缺 {key} 键")
    if obj.get("invalid"):
        print(f"[gen] 拒绝生成：{path} 带 {len(obj['invalid'])} 个 invalid 档位（R10）：",
              file=sys.stderr)
        for iv in obj["invalid"]:
            print(f"      {iv}", file=sys.stderr)
        print("[gen] 先重采这些档位（或查清原因）再生成，绝不把不可用档位写进表。",
              file=sys.stderr)
        raise SystemExit(2)
    n = len(obj["duty"])
    for key in ("omega_fwd", "omega_bwd"):
        if len(obj[key]) != n or any(len(row) != 4 for row in obj[key]):
            raise SystemExit(f"[gen] {path} 的 {key} 形状不对（应为 {n}×4）")
        for row in obj[key]:
            for v in row:
                if not isinstance(v, (int, float)) or v < 0 or v != v:
                    raise SystemExit(f"[gen] {path} 的 {key} 有非法值 {v!r}")
    return obj


def last_zero_duty(duties, curve):
    """死区估计：本次扫到的最后一个"不转"（ω==0）档位；都不为 0 则 0。

    真值在该档与下一档之间（未细分）；当前固件并未使用这两个常量，仅保留结构。
    """
    z = [d for d, v in zip(duties, curve) if v == 0]
    return z[-1] if z else 0


def fmt_row(duties, curve):
    items = [f"{{{d}, {v:.2f}}}" for d, v in zip(duties, curve)]
    # 与旧表排版一致：每行 5 个数据点
    lines = [", ".join(items[i:i + 5]) for i in range(0, len(items), 5)]
    return ",\n  ".join(lines)


def emit(src, obj, json_path):
    duties = obj["duty"]
    fwd, bwd = obj["omega_fwd"], obj["omega_bwd"]
    for w in range(4):
        for row, key in ((fwd, "omega_fwd"), (bwd, "omega_bwd")):
            curve = [row[k][w] for k in range(len(duties))]
            inc = all(curve[i] <= curve[i + 1] + 1e-9 for i in range(len(curve) - 1))
            if not inc:
                print(f"[gen][warn] {WHEEL_NAMES[w]} 的 {key} 曲线非单调，"
                      f"interpDuty 的区间假设会被打扰：{['%.2f' % v for v in curve]}",
                      file=sys.stderr)
    dz_pos = [last_zero_duty(duties, [row[w] for row in fwd]) for w in range(4)]
    dz_neg = [last_zero_duty(duties, [row[w] for row in bwd]) for w in range(4)]
    rel = os.path.relpath(json_path, REPO)

    out = []
    a = out.append
    a("/*")
    a(" * 麦轮期望轮速 → PWM 映射表 —— %s" % SOURCE_DESC[src])
    a(" *")
    a(" * 本文件由 实验/wheelmap_gen.py 从 %s 生成。" % rel)
    a(" * 采集方法：M 直驱逐轮/逐档（不经过 MIN_DUTY 钳位）、取每档后 0.6 s 稳态窗、")
    a(" * 逐行核对实际下发 pwm == 请求值（裁决 R10）；ω = Δcnt/s · 2π / C_1rev，")
    a(" * C_1rev = %s（固件/历史/kpwm 标定值）。" % obj["c_1rev"])
    a(" *")
    a(" * 【符号约定】表按**板级 PWM 极性**命名：X_FWD = 板级正占空比曲线，")
    a(" *   X_BWD = 板级负占空比曲线。固件映射规则（wheelOmegaToPWM）不变：")
    a(" *   ω_des≥0 查 pos_tables={FL_BWD,FR_FWD,RL_BWD,RR_FWD}，ω_des<0 查")
    a(" *   neg_tables={FL_FWD,FR_BWD,RL_FWD,RR_BWD}，再按 invert[4]={true,false,true,false}")
    a(" *   翻板级符号。因左轮前进=板级负：FL_BWD/RL_BWD 装的就是「前进方向」实测曲线；")
    a(" *   右轮前进=板级正：FR_FWD/RR_FWD 也是「前进方向」曲线。")
    a(" *")
    a(" * 【替换前人工检查】① 四轮 fwd/bwd 曲线是否与物理直觉一致（同 duty 转速量级）；")
    a(" *   ② 是否保留 MIN_DUTY=110 与 invert[4]；③ ground 套才用于控制，air 套只作诊断。")
    a(" */")
    a("#pragma once")
    a("#include <Arduino.h>")
    a("#include <math.h>")
    a("")
    a("// =========================================================")
    a("// 1. 实测数据表（%s）" % ("架空" if src == "air" else "落地"))
    a("//    ω 单位 rad/s；档位 duty = %s" % ",".join(str(d) for d in duties))
    a("// =========================================================")
    a("struct DutyOmega {")
    a("  int duty;")
    a("  float omega;")
    a("};")
    a("")
    for name, w, key, note in ARRAYS:
        a("// %s" % note)
        a("const DutyOmega %s[] = {" % name)
        a("  " + fmt_row(duties, [row[w] for row in obj[key]]))
        a("};")
        a("")
    a("const int TABLE_SIZE = %d;" % len(duties))
    a("")
    a("// 死区估计（本次 sweep 最后的不转档位；真值在该档与下一档之间，未细分）。")
    a("// 注意：当前 wheelOmegaToPWM 并不使用这两个常量，保留结构供后续/诊断用。")
    a("const int DZ_POS[4] = {%s};  // FL FR RL RR" % ", ".join(str(v) for v in dz_pos))
    a("const int DZ_NEG[4] = {%s};" % ", ".join(str(v) for v in dz_neg))
    a("")
    a("// 低速续动下限：只要需要转动，duty 不得低于此值，用以克服静摩擦")
    a("// （保持旧值 110 —— brief Step 4 明确要求保留）")
    a("const int MIN_DUTY = 110;")
    a("")
    a("// =========================================================")
    a("// 2. 线性插值（逻辑与旧文件一致）")
    a("// =========================================================")
    a("float interpDuty(const DutyOmega* table, int size, float target_omega) {")
    a("  if (target_omega < 0.3f) return 0.0f;")
    a("")
    a("  if (target_omega <= table[0].omega) {")
    a("    for (int i = 0; i < size; i++) {")
    a("      if (table[i].omega > 0.5f) {")
    a("        return table[i].duty * (target_omega / table[i].omega);")
    a("      }")
    a("    }")
    a("    return 0.0f;")
    a("  }")
    a("")
    a("  if (target_omega >= table[size-1].omega) {")
    a("    return (float)table[size-1].duty;")
    a("  }")
    a("")
    a("  for (int i = 0; i < size - 1; i++) {")
    a("    if (target_omega >= table[i].omega && target_omega <= table[i+1].omega) {")
    a("      float ratio = (target_omega - table[i].omega) / (table[i+1].omega - table[i].omega);")
    a("      return table[i].duty + ratio * (table[i+1].duty - table[i].duty);")
    a("    }")
    a("  }")
    a("  return (float)table[size-1].duty;")
    a("}")
    a("")
    a("// =========================================================")
    a("// 3. 主映射函数（逻辑与旧文件一致：保留 invert[4]）")
    a("// =========================================================")
    a("void wheelOmegaToPWM(const float omega_des[4], int pwm_out[4]) {")
    a("  const DutyOmega* pos_tables[4] = {FL_BWD, FR_FWD, RL_BWD, RR_FWD};")
    a("  const DutyOmega* neg_tables[4] = {FL_FWD, FR_BWD, RL_FWD, RR_BWD};")
    a("")
    a("  const bool invert[4] = {true, false, true, false};")
    a("")
    a("  for (int i = 0; i < 4; i++) {")
    a("    float w = omega_des[i];")
    a("    float abs_w = fabsf(w);")
    a("")
    a("    float duty = 0.0f;")
    a("    if (abs_w > 0.3f) {")
    a("      const DutyOmega* table = (w >= 0.0f) ? pos_tables[i] : neg_tables[i];")
    a("      duty = interpDuty(table, TABLE_SIZE, abs_w);")
    a("")
    a("      // 低速续动下限：任何需要转动的轮子 duty 不低于 MIN_DUTY")
    a("      if (duty > 0.0f && duty < MIN_DUTY) {")
    a("        duty = MIN_DUTY;")
    a("      }")
    a("    }")
    a("")
    a("    int signed_duty = (w >= 0.0f) ? (int)(duty + 0.5f) : -(int)(duty + 0.5f);")
    a("    if (invert[i]) signed_duty = -signed_duty;")
    a("    pwm_out[i] = constrain(signed_duty, -255, 255);")
    a("  }")
    a("}")
    a("")
    return "\n".join(out), dz_pos, dz_neg


def main():
    ap = argparse.ArgumentParser(description="wheel_map.h 生成器（Task 7 Step 4）")
    ap.add_argument("--source", choices=["air", "ground"], required=True)
    ap.add_argument("--json", default=None, help="输入 JSON（默认 固件/exp/data/wheelmap_{airborne|grounded}.json）")
    ap.add_argument("--out", default=None, help="输出路径（默认 stdout）")
    args = ap.parse_args()

    json_path = args.json or DEFAULT_JSON[args.source]
    obj = load(json_path)
    code, dz_pos, dz_neg = emit(args.source, obj, json_path)

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(code)
        print(f"[gen] 已写 {args.out}（source={args.source}, duty={obj['duty']}）", file=sys.stderr)
    else:
        sys.stdout.write(code)

    # 摘要走 stderr，stdout 保持纯净（可重定向）
    print(f"[gen] DZ_POS={dz_pos} DZ_NEG={dz_neg} "
          f"（最后不转档位；MIN_DUTY=110 保留）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
