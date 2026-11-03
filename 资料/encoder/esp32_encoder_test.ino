/*
 * esp32_encoder_test.ino  —  四路编码器安装/接线检测（ESP32, Arduino-ESP32）
 *
 * 目的：不接电机、不跑 PID，只判断“编码器装得对不对、接得对不对”。
 * 板子：你的车机是经典 ESP32（从串口 boot log 确认，见对话记录）。
 *
 * 用前改两处：
 *   1) 下面的 ENC[4] 引脚表 —— 换成你实际接的 GPIO。
 *   2) 保证这四个编码器都接了电源、且 A/B 两相都接上。
 *
 * ⚠ ESP32 注意：
 *   - GPIO 34~39 是“只输入、无内部上拉”脚。若编码器接在这些脚，必须外接
 *     10k 上拉到 3.3V，否则 INPUT_PULLUP 无效、读数乱跳。
 *   - GPIO 6~11 接内部 flash，不能用；GPIO 0/2/12/15 是 strapping，尽量避免。
 *
 * 判读标准（见文件末尾“怎么判断”）：
 *   - 静止时读数必须纹丝不动；
 *   - 手转某轮，对应 pos 跟着变、aCnt 和 bCnt 都涨；
 *   - 四轮同方向手转，pos 符号应一致；
 *   - 每轮精确转 1 圈，pos 的变化量四轮应接近。
 */
#include <Arduino.h>

// ====== 1) 引脚表：按你的实际接线修改 ======
struct EncPin { uint8_t a, b; };
EncPin ENC[4] = {
  {32, 33},   // 编号1 左前 : A=GPIO32, B=GPIO33
  {25, 26},   // 编号2 右前 : A=GPIO25, B=GPIO26
  {27, 14},   // 编号3 左后 : A=GPIO27, B=GPIO14
  {12, 13},   // 编号4 右后 : A=GPIO12, B=GPIO13
};
// ==========================================

struct EncState {
  volatile long     pos;    // 有符号位置（只在 A 跳变时更新）
  volatile uint32_t aCnt;   // A 相跳变次数
  volatile uint32_t bCnt;   // B 相跳变次数（只为确认 B 是否活）
  uint8_t a, b;
};
static EncState E[4];

// —— 每路两个 ISR ——
#define DEF_ISR(idx)                                                    \
  static void IRAM_ATTR isrA##idx() {                                   \
    EncState *e = &E[idx];                                              \
    e->aCnt++;                                                          \
    e->pos += (digitalRead(e->b) == digitalRead(e->a)) ? +1 : -1;       \
  }                                                                     \
  static void IRAM_ATTR isrB##idx() {                                   \
    E[idx].bCnt++;                                                      \
  }

DEF_ISR(0) DEF_ISR(1) DEF_ISR(2) DEF_ISR(3)

static void (*isrA[4])() = { isrA0, isrA1, isrA2, isrA3 };
static void (*isrB[4])() = { isrB0, isrB1, isrB2, isrB3 };

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== 四路编码器检测 ===");
  for (int i = 0; i < 4; i++) {
    E[i].a = ENC[i].a;   E[i].b = ENC[i].b;
    E[i].pos = 0;        E[i].aCnt = 0;  E[i].bCnt = 0;
    pinMode(E[i].a, INPUT_PULLUP);
    pinMode(E[i].b, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(E[i].a), isrA[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(E[i].b), isrB[i], CHANGE);
    Serial.printf("轮%d: A=GPIO%d  B=GPIO%d\n", i + 1, E[i].a, E[i].b);
  }
  Serial.println("----------------------------------------");
  Serial.println("静止时若 pos 还在变 → 该脚接错/悬空/缺上拉");
  Serial.println("编号 pos aCnt bCnt");
}

void loop() {
  static uint32_t t0 = 0;
  if (millis() - t0 < 500) return;
  t0 = millis();

  noInterrupts();
  long p[4]; uint32_t ac[4], bc[4];
  for (int i = 0; i < 4; i++) {
    p[i]  = E[i].pos;
    ac[i] = E[i].aCnt;
    bc[i] = E[i].bCnt;
  }
  interrupts();

  for (int i = 0; i < 4; i++)
    Serial.printf(" %d   %6ld %6lu %6lu%s\n", i + 1, p[i],
                  (unsigned long)ac[i], (unsigned long)bc[i],
                  (bc[i] == 0 && ac[i] > 0) ? "  <- B 相没动静!" : "");
}

/*
 * 怎么判断“装得对不对”：
 *
 * ① 静止稳定：开机后什么都不碰，连续看 5 秒。
 *    - pos/aCnt/bCnt 全部不变           → 基本接对了。
 *    - pos 乱跳 / 一直加减              → 引脚悬空、没上拉、或接到错误脚。
 *
 * ② 单轮手转：只慢慢转 1 号轮。
 *    - 只有 1 号轮的 pos/aCnt/bCnt 变化   → 一 一对应，没串脚。
 *    - 转了 1 号但别的轮在变             → 引脚映射写错。
 *    - aCnt 不动 / bCnt 不动             → 对应那一相没接通（断线/焊点/接触）。
 *
 * ③ 方向一致：把四个轮子都朝“车前进”方向慢慢转。
 *    - 四个 pos 都朝同一符号增加          → 方向一致，好。
 *    - 有的加有的减                      → A/B 接反或电机装反，记下来，
 *                                          后面用软件取反或调线解决。
 *
 * ④ 一圈计数：在轮上做个记号，每个轮精确转 1 整圈，读 pos 的变化量。
 *    - 四轮数值接近                     → 机械/编码器一致。
 *    - 差很多                           → 该轮间隙大、盘偏心、或漏计数。
 *
 * 把 ②③④ 的结果发我，我帮你判断是“线接错”“AB 反了”还是“机械装歪”。
 */
