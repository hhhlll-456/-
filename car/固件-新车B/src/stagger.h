/*
 * 四轮错峰抬升（stagger）—— 防 brownout 的软件缓解
 *
 * 背景：2026-10-13 架空诊断时，四轮同时从静止抬到工作 duty 的瞬间 ESP32 的 brownout
 * 检测器触发复位（串口：Brownout detector was triggered；见
 * 实验/原始日志/DIAG_FINDINGS_20261013.md §5）。同一时段单轮驱动不复现 —— 是四路
 * 涌入电流在电源上叠加的总电流事件。本模块把四轮的抬升时刻按轮序错开，避免同时叠加。
 *
 * 【为什么 STAGGER_ENABLED 默认是 true】原设计是「硬件优先」：先在 A4（电池带载压降）
 * 里查电池/接插件，再由那份诊断决定要不要开软件缓解（旧裁决 R8 因此让它默认 false）。
 * A4 已被用户取消（裁决 R52），brownout 的硬件病因从此不再诊断，软件缓解成了唯一防线 ——
 * 明知 4 电机同驱会触发 brownout 却把缓解留在 false，等于不设防，故直接建成 true。
 * 若将来补做了电池诊断、确认电气侧无碍，把 STAGGER_ENABLED 改回 false 即可回到
 * 「四轮同时抬升」的旧行为，调用方无需任何改动。
 *
 * 用法：把各轮此刻的目标 duty 填进数组，再调 apply_stagger()；还没轮到放行的轮位会被
 * 就地清零，已到点的轮位原样保留。
 *
 * 已知代价（spec §6.3 已提示）：错峰窗口内（3×STAGGER_STEP_MS = 120 ms）四轮出力不对称，
 * 落地运行时会给起步段带来 ~150–200 ms 的侧向/偏航瞬态 —— 解读早期样本时要计入。
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>

constexpr bool     STAGGER_ENABLED = true;
constexpr uint32_t STAGGER_STEP_MS = 40;   // 相邻两轮的放行间隔(ms)；四轮共 3×40 = 120 ms

// ── B 车改动：错峰轮序 改成「左右交替」，消掉起步偏航瞬态 ────────────────────
// 原轮序是**按轮序**放行：FL(0) → FR(40) → RL(80) → RR(120)。第一个 40 ms 里
// 只有 FL 在出力，车头被拧向一侧 —— 这正是上面「已知代价」说的那个瞬态。
// 实测（2026-10-16 本车、真机）：补偿掉轮速差之后，残余的就是这个**与时长无关**的
// 固定偏航 ~2.4°（400/600/800/1000 ms 分别为 −2.36/−2.36/−2.89/−1.77°，没有趋势）。
//
// 单轮对整车的偏航力矩 τ = ± b·F（左侧 −、右侧 +，b = 半轮距）。要让起步段净冲量为零，
// 只需让**连续两个放行窗口的失衡符号相反**。改成 FL → RR → FR → RL：
//     0– 40ms  只有 FL          τ = −bF
//    40– 80    加 RR（对角）     τ =  0
//    80–120    加 FR            τ = +bF     ⇒ 净冲量 = −bF·40 + 0 + bF·40 = 0
// 120ms 之后四轮齐 ⇒ τ = 0。kick 退出（各轮自己的放行 +KICK_MS）造成的第二段窗口
// 同样被这个顺序抵消。
//
// ⚠ 关键：**仍然是一次只放行一个轮子**（slot 是 0/1/2/3 的一个排列），
//   STAGGER_STEP_MS 也没动 —— brownout 缓解的强度与原设计**完全一致**，
//   只是把放行的顺序换成左右交替。没有引入两轮同时抬升。
constexpr int STAGGER_SLOT[4] = {0, 2, 3, 1};   // 轮序 FL FR RL RR → 放行槽位

// 第 i 个轮子的放行延迟(ms)。关掉开关时全部为 0 —— 四轮同时抬升，与引入错峰前的行为一致。
constexpr uint32_t staggerDelayMs(int i) {
  return STAGGER_ENABLED ? (uint32_t)STAGGER_SLOT[i] * STAGGER_STEP_MS : 0;
}

// 按轮序错峰放行：第 i 个轮子要在 t0_us 之后 i*STAGGER_STEP_MS 才被允许出力。
// STAGGER_ENABLED=false 时直接返回（编译期被优化掉），duty[] 原样不动。
inline void apply_stagger(int duty[4], uint32_t t0_us) {
  if (!STAGGER_ENABLED) return;
  uint32_t elapsed_ms = (uint32_t)((micros() - t0_us) / 1000);
  for (int i = 0; i < 4; i++) {
    if (elapsed_ms < staggerDelayMs(i)) duty[i] = 0;   // 没轮到它，先别出力
  }
}
