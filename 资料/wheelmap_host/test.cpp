#include "wheel_map.h"

static const char* WN[4] = {"FL", "FR", "RL", "RR"};

static void one(int i, float o, int* p) {   // 只驱动第 i 个轮
  float w[4] = {0, 0, 0, 0};
  w[i] = o;
  wheelOmegaToPWM(w, p);
}

int main() {
  printf("================ 1) 期望轮速 -> PWM 映射 (kinematic: + = forward) ================\n");
  float sweep[] = {0, 1, 2, 3, 5, 8, 10, 12.5, 15, 18, 20, 25, 30, 35, 38, 40};
  for (int i = 0; i < 4; i++) {
    printf("\n%s  ω_des :", WN[i]);
    for (float o : sweep) printf("%7.1f", o);
    printf("\n   PWM(+) :");
    for (float o : sweep) { int p[4]; one(i, o, p); printf("%7d", p[i]); }
    printf("\n   PWM(-) :");
    for (float o : sweep) { int p[4]; one(i, -o, p); printf("%7d", p[i]); }
    printf("\n");
  }

  printf("\n================ 2) 回代自检：表内每个 (duty,ω) 反推的 PWM ================\n");
  printf("物理 duty=+d 由 ω_des=+ω 产生? 物理 duty=-d 由 ω_des=-ω 产生?\n");
  const DutyOmega* T[4] = {FL_FWD, FR_FWD, RL_FWD, RR_FWD};
  for (int i = 0; i < 4; i++) {
    printf("\n%s  (表内 duty / ω  |  命令+ω→PWM  |  命令-ω→PWM)\n", WN[i]);
    for (int k = 0; k < TABLE_SIZE; k++) {
      int pf[4], pb[4];
      one(i,  T[i][k].omega, pf);
      one(i, -T[i][k].omega, pb);
      printf("   d=%3d ω=%6.2f   |  %5d  |  %5d\n",
             T[i][k].duty, T[i][k].omega, pf[i], pb[i]);
    }
  }

  printf("\n================ 2.5) 验证：低速 vs 死区（修复点1） ================\n");
  printf("物理前进方向: FL/RL 输出应为负, FR/RR 输出应为正\n");
  printf("有效死区: FL/RL(输出负) 用 DZ_NEG, FR/RR(输出正) 用 DZ_POS\n\n");
  {
    const int   dz_for_pos[4] = {DZ_NEG[0], DZ_POS[1], DZ_NEG[2], DZ_POS[3]};
    const char* out_sign[4]   = {"负", "正", "负", "正"};
    float lows[] = {3, 4, 5, 6};
    for (int i = 0; i < 4; i++) {
      printf("%s  有效死区=%2d  期望输出符号=%s\n", WN[i], dz_for_pos[i], out_sign[i]);
      for (float o : lows) {
        int p[4]; one(i, o, p);
        int mag = p[i] < 0 ? -p[i] : p[i];
        bool ok = (mag > dz_for_pos[i]);
        printf("     ω_des=%4.1f  ->  PWM=%5d  |PWM|=%3d  %s\n",
               o, p[i], mag, ok ? "OK 高于死区" : "FAIL 仍在死区");
      }
      printf("\n");
    }
  }

  printf("================ 2.6) 验证：纯前进四轮朝向（修复点2） ================\n");
  {
    float wf[4] = {9, 9, 9, 9};
    int p[4];
    wheelOmegaToPWM(wf, p);
    const bool want_neg[4] = {true, false, true, false};  // FL/RL 应为负, FR/RR 应为正
    printf("ω_des=[9 9 9 9] -> PWM=[%d %d %d %d]\n", p[0], p[1], p[2], p[3]);
    for (int i = 0; i < 4; i++) {
      bool is_neg = p[i] < 0;
      bool ok = (is_neg == want_neg[i]);
      printf("   %s: PWM=%5d  物理方向=%s  %s\n", WN[i], p[i],
             is_neg ? "负(前)" : "正(前)", ok ? "OK 朝前" : "FAIL 方向错");
    }
    printf("\n");
  }

  printf("\n================ 3) 整车常用指令 ================\n");
  struct { const char* name; float vx, vy, om; } cmds[] = {
    {"纯前进",  1, 0, 0}, {"纯后退", -1, 0, 0},
    {"纯右移",  0, 1, 0}, {"纯左移",  0,-1, 0},
    {"原地正转",0, 0, 1}, {"原地反转",0, 0,-1},
  };
  const float R = 0.0335f, LXY = 0.12825f;
  for (auto& c : cmds) {
    // 只当运动学用示例速度：vx/vy 用 0.3 m/s, ω 用 1.5 rad/s 举例
    float vx = c.vx * 0.3f, vy = c.vy * 0.3f, om = c.om * 1.5f;
    float w[4];
    w[0] = (1.0f/R) * ( vx - vy - LXY*om);
    w[1] = (1.0f/R) * ( vx + vy + LXY*om);
    w[2] = (1.0f/R) * ( vx + vy - LXY*om);
    w[3] = (1.0f/R) * ( vx - vy + LXY*om);
    int p[4];
    wheelOmegaToPWM(w, p);
    printf("  %-9s ω_des=[%6.1f %6.1f %6.1f %6.1f]  ->  PWM=[%4d %4d %4d %4d]\n",
           c.name, w[0], w[1], w[2], w[3], p[0], p[1], p[2], p[3]);
  }
  return 0;
}
