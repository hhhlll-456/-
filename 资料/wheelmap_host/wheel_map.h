/*
 * 麦轮期望轮速 → PWM 映射模块（已修复版）
 * 修复点：
 * 1. 低速强制不低于死区+5，避免欠驱动
 * 2. FL/RL 的正反表已对调，匹配 invert 逻辑
 */
#pragma once
#include <Arduino.h>
#include <math.h>

// =========================================================
// 1. 实测数据表
// =========================================================
struct DutyOmega {
  int duty;
  float omega;
};

// FL
const DutyOmega FL_FWD[] = {   // 物理正占空比曲线
  {50, 2.42}, {60, 7.46}, {80, 13.46}, {100, 19.42}, {120, 24.20},
  {140, 27.93}, {160, 30.76}, {180, 33.11}, {200, 35.03}, {220, 36.66}, {240, 38.42}
};
const DutyOmega FL_BWD[] = {   // 物理负占空比曲线
  {50, 3.32}, {60, 7.85}, {80, 14.49}, {100, 20.68}, {120, 25.47},
  {140, 29.09}, {160, 31.74}, {180, 33.88}, {200, 35.68}, {220, 37.20}, {240, 38.76}
};

// FR
const DutyOmega FR_FWD[] = {
  {50, 5.11}, {60, 8.52}, {80, 14.32}, {100, 20.07}, {120, 24.78},
  {140, 28.49}, {160, 31.70}, {180, 34.35}, {200, 36.44}, {220, 38.36}, {240, 40.07}
};
const DutyOmega FR_BWD[] = {
  {50, 4.05}, {60, 8.11}, {80, 14.56}, {100, 20.95}, {120, 26.55},
  {140, 30.50}, {160, 33.22}, {180, 35.40}, {200, 37.40}, {220, 38.82}, {240, 40.43}
};

// RL
const DutyOmega RL_FWD[] = {
  {50, 0.00}, {60, 0.00}, {80, 13.06}, {100, 19.18}, {120, 23.53},
  {140, 26.71}, {160, 29.63}, {180, 32.24}, {200, 34.33}, {220, 36.52}, {240, 38.66}
};
const DutyOmega RL_BWD[] = {
  {50, 0.73}, {60, 7.85}, {80, 14.22}, {100, 20.02}, {120, 24.67},
  {140, 28.18}, {160, 31.47}, {180, 34.13}, {200, 36.43}, {220, 38.49}, {240, 40.09}
};

// RR
const DutyOmega RR_FWD[] = {
  {50, 5.57}, {60, 9.17}, {80, 15.54}, {100, 21.41}, {120, 26.23},
  {140, 29.70}, {160, 32.53}, {180, 34.85}, {200, 36.74}, {220, 38.52}, {240, 40.22}
};
const DutyOmega RR_BWD[] = {
  {50, 4.52}, {60, 9.04}, {80, 16.11}, {100, 22.27}, {120, 27.03},
  {140, 30.67}, {160, 33.39}, {180, 35.64}, {200, 37.37}, {220, 38.83}, {240, 40.35}
};

const int TABLE_SIZE = 11;

// 死区（用于低速保护）
const int DZ_POS[4] = {40, 48, 56, 44};  // FL FR RL RR
const int DZ_NEG[4] = {52, 48, 68, 60};

// =========================================================
// 2. 线性插值
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
// 3. 主映射函数（已修复）
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

      // ===== 修复1：低速强制不低于死区+5 =====
      int dz = (w >= 0.0f) ? DZ_POS[i] : DZ_NEG[i];
      if (invert[i]) {
        dz = (w >= 0.0f) ? DZ_NEG[i] : DZ_POS[i];
      }
      if (duty > 0.0f && duty < dz + 5) {
        duty = dz + 5;
      }
    }

    int signed_duty = (w >= 0.0f) ? (int)(duty + 0.5f) : -(int)(duty + 0.5f);
    if (invert[i]) signed_duty = -signed_duty;
    pwm_out[i] = constrain(signed_duty, -255, 255);
  }
}
