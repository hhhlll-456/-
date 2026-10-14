/*
 * 麦轮期望轮速 → PWM 映射表 —— 落地套（控制用：车放场地、四轮着地）
 *
 * 本文件由 实验/wheelmap_gen.py 从 固件/exp/data/wheelmap_grounded.json 生成。
 * 采集方法：M 直驱逐轮/逐档（不经过 MIN_DUTY 钳位）、取每档后 0.6 s 稳态窗、
 * 逐行核对实际下发 pwm == 请求值（裁决 R10）；ω = Δcnt/s · 2π / C_1rev，
 * C_1rev = [1276.0, 1270.0, 1276.0, 1274.0]（固件/历史/kpwm 标定值）。
 *
 * 【符号约定】表按**板级 PWM 极性**命名：X_FWD = 板级正占空比曲线，
 *   X_BWD = 板级负占空比曲线。固件映射规则（wheelOmegaToPWM）不变：
 *   ω_des≥0 查 pos_tables={FL_BWD,FR_FWD,RL_BWD,RR_FWD}，ω_des<0 查
 *   neg_tables={FL_FWD,FR_BWD,RL_FWD,RR_BWD}，再按 invert[4]={true,false,true,false}
 *   翻板级符号。因左轮前进=板级负：FL_BWD/RL_BWD 装的就是「前进方向」实测曲线；
 *   右轮前进=板级正：FR_FWD/RR_FWD 也是「前进方向」曲线。
 *
 * 【替换前人工检查】① 四轮 fwd/bwd 曲线是否与物理直觉一致（同 duty 转速量级）；
 *   ② 是否保留 MIN_DUTY=110 与 invert[4]；③ ground 套才用于控制，air 套只作诊断。
 */
#pragma once
#include <Arduino.h>
#include <math.h>

// =========================================================
// 1. 实测数据表（落地）
//    ω 单位 rad/s；档位 duty = 110,120,140,160,180,200,220,240
// =========================================================
struct DutyOmega {
  int duty;
  float omega;
};

// FL 前进方向（ω_des>0）；左轮前进=板级负（invert[0]=true）
const DutyOmega FL_BWD[] = {
  {110, 13.02}, {120, 15.33}, {140, 18.35}, {160, 21.34}, {180, 24.08},
  {200, 25.98}, {220, 28.20}, {240, 30.39}
};

// FR 前进方向（ω_des>0）；右轮前进=板级正
const DutyOmega FR_FWD[] = {
  {110, 12.98}, {120, 15.56}, {140, 18.99}, {160, 22.13}, {180, 24.41},
  {200, 26.43}, {220, 28.73}, {240, 30.69}
};

// RL 前进方向（ω_des>0）；左轮前进=板级负（invert[2]=true）
const DutyOmega RL_BWD[] = {
  {110, 14.14}, {120, 16.49}, {140, 19.29}, {160, 22.87}, {180, 25.81},
  {200, 28.36}, {220, 30.72}, {240, 32.72}
};

// RR 前进方向（ω_des>0）；右轮前进=板级正
const DutyOmega RR_FWD[] = {
  {110, 14.06}, {120, 16.61}, {140, 20.32}, {160, 24.37}, {180, 26.24},
  {200, 29.54}, {220, 31.11}, {240, 34.32}
};

// FL 后退方向（ω_des<0）；板级正
const DutyOmega FL_FWD[] = {
  {110, 12.92}, {120, 15.60}, {140, 18.88}, {160, 22.11}, {180, 23.98},
  {200, 25.81}, {220, 28.22}, {240, 30.34}
};

// FR 后退方向（ω_des<0）；板级负
const DutyOmega FR_BWD[] = {
  {110, 13.32}, {120, 16.09}, {140, 18.72}, {160, 22.47}, {180, 25.52},
  {200, 27.36}, {220, 29.87}, {240, 32.48}
};

// RL 后退方向（ω_des<0）；板级正
const DutyOmega RL_FWD[] = {
  {110, 14.18}, {120, 16.57}, {140, 20.04}, {160, 23.47}, {180, 25.33},
  {200, 28.34}, {220, 30.31}, {240, 32.92}
};

// RR 后退方向（ω_des<0）；板级负
const DutyOmega RR_BWD[] = {
  {110, 15.02}, {120, 17.41}, {140, 20.26}, {160, 23.81}, {180, 26.92},
  {200, 29.18}, {220, 31.73}, {240, 34.37}
};

const int TABLE_SIZE = 8;

// 死区估计（本次 sweep 最后的不转档位；真值在该档与下一档之间，未细分）。
// 注意：当前 wheelOmegaToPWM 并不使用这两个常量，保留结构供后续/诊断用。
const int DZ_POS[4] = {0, 0, 0, 0};  // FL FR RL RR
const int DZ_NEG[4] = {0, 0, 0, 0};

// 低速续动下限：只要需要转动，duty 不得低于此值，用以克服静摩擦
// （保持旧值 110 —— brief Step 4 明确要求保留）
const int MIN_DUTY = 110;

// =========================================================
// 2. 线性插值（逻辑与旧文件一致）
// =========================================================
float interpDuty(const DutyOmega* table, int size, float target_omega) {
  if (target_omega < 0.3f) return 0.0f;

  if (target_omega <= table[0].omega) {
    for (int i = 0; i < size; i++) {
      if (table[i].omega > 0.5f) {
        return table[i].duty * (target_omega / table[i].omega);
      }
    }
    return 0.0f;
  }

  if (target_omega >= table[size-1].omega) {
    return (float)table[size-1].duty;
  }

  for (int i = 0; i < size - 1; i++) {
    if (target_omega >= table[i].omega && target_omega <= table[i+1].omega) {
      float ratio = (target_omega - table[i].omega) / (table[i+1].omega - table[i].omega);
      return table[i].duty + ratio * (table[i+1].duty - table[i].duty);
    }
  }
  return (float)table[size-1].duty;
}

// =========================================================
// 3. 主映射函数（逻辑与旧文件一致：保留 invert[4]）
// =========================================================
void wheelOmegaToPWM(const float omega_des[4], int pwm_out[4]) {
  const DutyOmega* pos_tables[4] = {FL_BWD, FR_FWD, RL_BWD, RR_FWD};
  const DutyOmega* neg_tables[4] = {FL_FWD, FR_BWD, RL_FWD, RR_BWD};

  const bool invert[4] = {true, false, true, false};

  for (int i = 0; i < 4; i++) {
    float w = omega_des[i];
    float abs_w = fabsf(w);

    float duty = 0.0f;
    if (abs_w > 0.3f) {
      const DutyOmega* table = (w >= 0.0f) ? pos_tables[i] : neg_tables[i];
      duty = interpDuty(table, TABLE_SIZE, abs_w);

      // 低速续动下限：任何需要转动的轮子 duty 不低于 MIN_DUTY
      if (duty > 0.0f && duty < MIN_DUTY) {
        duty = MIN_DUTY;
      }
    }

    int signed_duty = (w >= 0.0f) ? (int)(duty + 0.5f) : -(int)(duty + 0.5f);
    if (invert[i]) signed_duty = -signed_duty;
    pwm_out[i] = constrain(signed_duty, -255, 255);
  }
}
