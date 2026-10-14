/*
 * 麦轮期望轮速 → PWM 映射表 —— 架空套（诊断用：车抬起、四轮悬空）
 *
 * 本文件由 实验/wheelmap_gen.py 从 固件/exp/data/wheelmap_airborne.json 生成。
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
// 1. 实测数据表（架空）
//    ω 单位 rad/s；档位 duty = 50,60,80,100,120,140,160,180,200,220,240
// =========================================================
struct DutyOmega {
  int duty;
  float omega;
};

// FL 前进方向（ω_des>0）；左轮前进=板级负（invert[0]=true）
const DutyOmega FL_BWD[] = {
  {50, 0.00}, {60, 0.00}, {80, 9.94}, {100, 14.61}, {120, 17.80},
  {140, 22.07}, {160, 24.77}, {180, 25.48}, {200, 25.70}, {220, 28.57},
  {240, 30.22}
};

// FR 前进方向（ω_des>0）；右轮前进=板级正
const DutyOmega FR_FWD[] = {
  {50, 0.00}, {60, 5.22}, {80, 8.86}, {100, 13.18}, {120, 20.02},
  {140, 24.34}, {160, 24.34}, {180, 29.52}, {200, 29.65}, {220, 28.31},
  {240, 32.58}
};

// RL 前进方向（ω_des>0）；左轮前进=板级负（invert[2]=true）
const DutyOmega RL_BWD[] = {
  {50, 0.00}, {60, 6.58}, {80, 12.40}, {100, 15.21}, {120, 21.33},
  {140, 24.66}, {160, 27.32}, {180, 29.10}, {200, 29.51}, {220, 32.42},
  {240, 33.71}
};

// RR 前进方向（ω_des>0）；右轮前进=板级正
const DutyOmega RR_FWD[] = {
  {50, 0.00}, {60, 7.48}, {80, 16.00}, {100, 18.27}, {120, 24.36},
  {140, 26.31}, {160, 24.68}, {180, 31.93}, {200, 33.40}, {220, 34.95},
  {240, 35.30}
};

// FL 后退方向（ω_des<0）；板级正
const DutyOmega FL_FWD[] = {
  {50, 0.00}, {60, 0.00}, {80, 9.07}, {100, 15.43}, {120, 19.27},
  {140, 21.48}, {160, 25.16}, {180, 26.72}, {200, 23.87}, {220, 29.85},
  {240, 30.75}
};

// FR 后退方向（ω_des<0）；板级负
const DutyOmega FR_BWD[] = {
  {50, 0.00}, {60, 5.78}, {80, 8.60}, {100, 13.77}, {120, 19.49},
  {140, 21.91}, {160, 27.08}, {180, 28.17}, {200, 31.40}, {220, 31.83},
  {240, 31.66}
};

// RL 后退方向（ω_des<0）；板级正
const DutyOmega RL_FWD[] = {
  {50, 0.00}, {60, 0.00}, {80, 12.78}, {100, 17.57}, {120, 21.58},
  {140, 24.87}, {160, 27.34}, {180, 29.08}, {200, 30.93}, {220, 32.20},
  {240, 34.44}
};

// RR 后退方向（ω_des<0）；板级负
const DutyOmega RR_BWD[] = {
  {50, 0.00}, {60, 8.96}, {80, 15.25}, {100, 20.19}, {120, 21.52},
  {140, 27.08}, {160, 30.36}, {180, 32.49}, {200, 31.86}, {220, 35.03},
  {240, 36.45}
};

const int TABLE_SIZE = 11;

// 死区估计（本次 sweep 最后的不转档位；真值在该档与下一档之间，未细分）。
// 注意：当前 wheelOmegaToPWM 并不使用这两个常量，保留结构供后续/诊断用。
const int DZ_POS[4] = {60, 50, 50, 50};  // FL FR RL RR
const int DZ_NEG[4] = {60, 50, 60, 50};

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
