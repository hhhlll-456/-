/*
 * 麦轮小车 实验执行 + 100Hz 采集固件
 *
 * 一条 TCP 命令就能跑一次实验，并在运行期间以 100Hz 回传样本：
 *   PWM / 四轮编码器累计脉冲 / IMU(roll,pitch,yaw)
 *
 * 网络：
 *   连接 Wi-Fi B1222 / 12221222
 *   TCP 命令+数据流端口 8266   （数据行以 'D ' 开头）
 *   UDP 发现端口 8267
 *   OTA 主机名 mecanum  （USB 拔掉后可无线升级）
 *
 * 命令：
 *   PING                      -> PONG
 *   ID                        -> MECANUM-ESP32 <ip>
 *   CHIP                      -> 芯片型号/核数
 *   HELP                      -> 命令列表
 *   M <fl> <fr> <rl> <rr>     -> 直接下发四轮 PWM(-255..255)
 *   STOP                      -> 四轮停
 *   RUN <vx> <vy> <om> <ms>   -> 按运动学算四轮 ω→PWM，跑 ms 毫秒，期间 100Hz 回传
 *   STREAM 0|1                -> 空闲时是否也持续回传样本
 *   YAWZERO                   -> 请求 IMU 航向归零（需要 TX 有效，可选）
 *
 * 数据行： 'D <t_ms> <pwm0..3> <enc0..3> <roll> <pitch> <yaw>'
 *          一行 = 一个样本。D 行在运行期间就分批发过来（攒够 5 行一批，约每 50 ms 一批），
 *          运行结束时再补发不足一批的残余，然后发一行 'DONE' 作为结束标志。
 *          所以 PC 侧"还在跑 / 跑完了 / 死了"三种状态的判据是：
 *          持续收到 D 行 = 还在跑；收到 DONE = 跑完；D 行断了又没有 DONE = 死了。
 *
 * 复位原因回报：
 *   非正常复位（上电 POWERON / 软复位 SW 之外的，如 brownout、panic、看门狗）后，
 *   PING 与 ID 的回复末尾会多一行 'RESET <原因名> count=<brownout 累计>'
 *   （例：RESET ESP_RST_BROWNOUT count=1）；正常启动时不发这一行。
 *
 * 掉电缓解：
 *   RUN/M 之后四轮按轮序错峰抬升（stagger.h），不再同时起转 —— 详见 stagger.h 的说明。
 *
 * 硬件：FL{13,15} FR{12,14} RL{27,26} RR{25,33}
 *       编码器 A 相 FL32 FR34 RL35 RR36（单相，只能计数不能辨向）
 *       IMU GY-25T  Serial2  RX=37  (TX=38 仅在 YAWZERO 时使用)  9600 8N1
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <lwip/sockets.h>   // send()/MSG_DONTWAIT：非阻塞发送（见 flushRunBuf）
#include <errno.h>          // errno：区分 send() 硬/软错误（R64）
#include <esp_system.h>     // esp_reset_reason()：复位原因回报
#include "wheel_map.h"
#include "stagger.h"        // 四轮错峰抬升（防 brownout）

// ---------------- 网络 ----------------
static const char*    WIFI_SSID = "B1222";
static const char*    WIFI_PASS = "12221222";
static const uint16_t TCP_PORT  = 8266;
static const uint16_t UDP_PORT  = 8267;

WiFiServer server(TCP_PORT);
WiFiUDP    udp;
WiFiClient client;

// ---------------- 复位原因 / brownout 记录 ----------------
// 用途：PC 侧重连后（PING/ID）即可知道"上一段运行是不是被掉电（brownout）打断的"。
//
// 两个变量、两种存储，别混淆：
//   g_reset_reason_rtc —— 本次启动的 esp_reset_reason()，存 RTC_DATA_ATTR（brief 指定）。
//     ESP32 上 brownout 走软件复位路径（brownout ISR 先 esp_reset_reason_set_hint() 再软复位），
//     IDF 的 esp_reset_reason_init 会把 hint 还原成 ESP_RST_BROWNOUT —— 已核对 core 2.0.17
//     自带 libesp_system.a 里 reset_reason.c 的反汇编：原始原因是 SW 时，
//     hint ∈ {PANIC, INT_WDT, TASK_WDT, BROWNOUT} 会被采用。所以重连后读到的
//     reason 字符串就是 ESP_RST_BROWNOUT，而不是被软复位掩盖掉的 ESP_RST_SW。
//   g_brownout_count —— 累计 brownout 复位次数。**必须存 .rtc_noinit（RTC_NOINIT_ATTR）**：
//     IDF v4.4 的 bootloader 在**任何非 deep-sleep 复位**（含 brownout）都会把
//     .rtc.data/.rtc.bss 从镜像重新装载（bootloader_support/src/esp_image_format.c 的
//     should_load()："Reload the RTC memory segments whenever a non-deepsleep reset is
//     occurring"），所以带初值的 RTC_DATA_ATTR 计数器在 brownout 后会被打回初值、
//     永远数不到 2。.rtc_noinit 段不进镜像、不会被重装载，才能真正跨复位累计。
//     冷启动（首次上电/烧录）或掉电过深时 RTC 内存内容是随机的，用起始魔数判定有效性：
//     魔数不符就清零重来 —— 所以计数是"已确认的下界"，不是绝对次数。
#define RTC_DIAG_MAGIC 0x5243484DUL   // "MHCR"
RTC_NOINIT_ATTR uint32_t g_rtc_diag_magic;
RTC_NOINIT_ATTR uint32_t g_brownout_count;
RTC_DATA_ATTR uint32_t   g_reset_reason_rtc = ESP_RST_UNKNOWN;

static const char* resetReasonName(uint32_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "ESP_RST_POWERON";
    case ESP_RST_EXT:       return "ESP_RST_EXT";
    case ESP_RST_SW:        return "ESP_RST_SW";
    case ESP_RST_PANIC:     return "ESP_RST_PANIC";
    case ESP_RST_INT_WDT:   return "ESP_RST_INT_WDT";
    case ESP_RST_TASK_WDT:  return "ESP_RST_TASK_WDT";
    case ESP_RST_WDT:       return "ESP_RST_WDT";
    case ESP_RST_DEEPSLEEP: return "ESP_RST_DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "ESP_RST_BROWNOUT";
    case ESP_RST_SDIO:      return "ESP_RST_SDIO";
    default:                return "ESP_RST_UNKNOWN";
  }
}

// 正常复位 = 上电 / esp_restart() 软复位；其余（brownout、panic、看门狗…）都要报告。
static bool resetIsAbnormal() {
  return g_reset_reason_rtc != ESP_RST_POWERON && g_reset_reason_rtc != ESP_RST_SW;
}

// setup() 里尽早调用一次：读本次复位原因、维护跨复位的 brownout 计数。
static void initResetDiag() {
  if (g_rtc_diag_magic != RTC_DIAG_MAGIC) {     // 冷启动/掉电过深：内容不可信，重开计数
    g_rtc_diag_magic = RTC_DIAG_MAGIC;
    g_brownout_count = 0;
  }
  g_reset_reason_rtc = (uint32_t)esp_reset_reason();
  if (g_reset_reason_rtc == ESP_RST_BROWNOUT) g_brownout_count++;
}

// ---------------- 电机 ----------------
const int MOTOR_PINS[4][2] = {{13, 15}, {12, 14}, {27, 26}, {25, 33}};
const int PWM_FREQ = 1000, PWM_RES = 8;
int lastPwm[4] = {0, 0, 0, 0};

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i * 2, PWM_FREQ, PWM_RES);
    ledcSetup(i * 2 + 1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i][0], i * 2);
    ledcAttachPin(MOTOR_PINS[i][1], i * 2 + 1);
    ledcWrite(i * 2, 0);
    ledcWrite(i * 2 + 1, 0);
  }
}
void setMotor(int w, int duty) {
  duty = constrain(duty, -255, 255);
  lastPwm[w] = duty;
  if (duty >= 0) { ledcWrite(w * 2, duty);     ledcWrite(w * 2 + 1, 0); }
  else           { ledcWrite(w * 2, 0);        ledcWrite(w * 2 + 1, -duty); }
}
void stopAll() { for (int i = 0; i < 4; i++) setMotor(i, 0); }

// ---------------- 编码器（单相，只计数） ----------------
const int ENC_PINS[4] = {32, 34, 35, 36};
volatile long encCount[4] = {0, 0, 0, 0};
void IRAM_ATTR isr0() { encCount[0]++; }
void IRAM_ATTR isr1() { encCount[1]++; }
void IRAM_ATTR isr2() { encCount[2]++; }
void IRAM_ATTR isr3() { encCount[3]++; }
void setupEncoders() {
  for (int i = 0; i < 4; i++) pinMode(ENC_PINS[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[0]), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[1]), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[2]), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[3]), isr3, CHANGE);
}
void readEncoders(long out[4]) {
  noInterrupts();
  for (int i = 0; i < 4; i++) out[i] = encCount[i];
  interrupts();
}
void zeroEncoders() {
  noInterrupts();
  for (int i = 0; i < 4; i++) encCount[i] = 0;
  interrupts();
}

// ---------------- IMU GY-25T ----------------
int IMU_RX_PIN = 38;          // 实测：IMU 的 TX 接在 GPIO38（37 无数据）
static float imu_roll = 0, imu_pitch = 0, imu_yaw = 0;
static bool  imu_ok = false;
static uint8_t imu_state = 0, imu_reg = 0, imu_len = 0, imu_cnt = 0, imu_buf[16];

// 引脚活动探针（诊断 IMU 的 TX 到底接在 37 还是 38）
volatile uint32_t edge37 = 0, edge38 = 0;
void IRAM_ATTR isr37() { edge37++; }
void IRAM_ATTR isr38() { edge38++; }
void imuSetPin(int rx) {
  Serial2.end();
  delay(20);
  IMU_RX_PIN = rx;
  Serial2.begin(9600, SERIAL_8N1, IMU_RX_PIN, -1);
}

static void imuFeed(uint8_t b) {
  switch (imu_state) {
    case 0: if (b == 0xA4) imu_state = 1; break;
    case 1: imu_state = (b == 0x03) ? 2 : 0; break;
    case 2: imu_reg = b; imu_state = (b < 0x1d) ? 3 : 0; break;
    case 3:
      imu_len = b;
      if (b == 0 || (imu_reg + imu_len) > 0x1d) { imu_state = 0; }
      else { imu_cnt = 0; imu_state = 4; }
      break;
    case 4:
      imu_buf[imu_cnt++] = b;
      if (imu_cnt >= imu_len) imu_state = 5;
      break;
    case 5: {
      uint8_t sum = 0xA4 + 0x03 + imu_reg + imu_len;
      for (uint8_t i = 0; i < imu_len; i++) sum += imu_buf[i];
      if (sum == b && imu_reg == 0x14 && imu_len >= 6) {
        int16_t r = (imu_buf[0] << 8) | imu_buf[1];
        int16_t p = (imu_buf[2] << 8) | imu_buf[3];
        int16_t y = (imu_buf[4] << 8) | imu_buf[5];
        imu_roll = r / 100.0f; imu_pitch = p / 100.0f; imu_yaw = y / 100.0f;
        imu_ok = true;
      }
      imu_state = 0;
    } break;
  }
}
void imuPump() { while (Serial2.available()) imuFeed((uint8_t)Serial2.read()); }

// ---------------- 运动学 ----------------
static const float R_WHEEL = 0.0335f;     // m
static const float LXY     = 0.12825f;    // m  (l_x + l_y 等效)
void bodyToPWM(float vx, float vy, float om, int pwm_out[4]) {
  float w[4];
  w[0] = (1.0f / R_WHEEL) * ( vx - vy - LXY * om);
  w[1] = (1.0f / R_WHEEL) * ( vx + vy + LXY * om);
  w[2] = (1.0f / R_WHEEL) * ( vx + vy - LXY * om);
  w[3] = (1.0f / R_WHEEL) * ( vx - vy + LXY * om);
  wheelOmegaToPWM(w, pwm_out);
}

// ---------------- 运行状态 ----------------
bool  stream_idle = false;     // 空闲也回传
bool  running     = false;
uint32_t run_t0_us = 0, run_dur_us = 0;
int   run_wheel_pwm[4] = {0, 0, 0, 0};
uint32_t next_sample_us = 0;

// 起步 kick：RUN 开始时先给一段大 PWM 突破静摩擦，之后回落到正常映射值
static const int      KICK_DUTY = 105;   // 脉冲目标幅值（可在 100~115 微调）
static const uint32_t KICK_MS   = 150;   // 脉冲时长(ms)，从**各轮自己的放行时刻**起算

// 错峰抬升窗口（stagger.h）：RUN/M 之后四轮按轮序每 STAGGER_STEP_MS 放行一个，
// 放行后先给 kick 幅值、KICK_MS 后再回落到目标 duty。窗口内每次 loop 迭代重算并下发；
// 窗口结束（最后一个轮子 kick 也到时）收敛到目标值并关闭。
// STAGGER_ENABLED=false 时放行延迟全为 0，等价于旧行为（四轮立即 kick、KICK_MS 后一起回落）。
bool     ramp_active    = false;
uint32_t ramp_t0_us     = 0;
int      ramp_target[4] = {0, 0, 0, 0};   // 各轮最终目标 duty
int      ramp_kick[4]   = {0, 0, 0, 0};   // 各轮 kick 阶段幅值（M 直接驱动时 = 目标值）

// 错峰抬升窗口的当前输出：第 i 个轮子在放行延迟之前保持 0；放行后前 KICK_MS 给 kick 幅值，
// 之后回落到目标 duty。放行延迟来自 stagger.h（开关关闭时全为 0，apply_stagger 也是空操作）。
void applyRamp() {
  uint32_t elapsed_ms = (uint32_t)((micros() - ramp_t0_us) / 1000);
  int duty[4];
  for (int i = 0; i < 4; i++) {
    int32_t rel = (int32_t)elapsed_ms - (int32_t)staggerDelayMs(i);   // 该轮自己的进度(ms)
    duty[i] = (rel < (int32_t)KICK_MS) ? ramp_kick[i] : ramp_target[i];
  }
  apply_stagger(duty, ramp_t0_us);       // 还没轮到放行的轮位在这里被清零
  for (int i = 0; i < 4; i++) setMotor(i, duty[i]);
}

// 错峰抬升窗口的总时长：最后一个轮子放行 + 它自己的 kick 结束。
static uint32_t rampSpanMs() { return staggerDelayMs(3) + KICK_MS; }

// ---------------- 链路安全：失联停车 ----------------
// RUN 自带时长超时，但 M 没有超时：链路断了（WiFi 掉线 / client 被断开）时开环输出
// 会一直跑。所以只要电机还在输出、而链路已丢失 >CLIENT_LOST_STOP_MS（500 ms 宽限，
// 容忍瞬时抖动）就强制停车。
// 注意：必须在 loop 最前面调用（WiFi 断线的 early-return 之前）—— WiFi 掉线时 loop
// 会提前 return，RUN 的到时检查根本走不到。
static const uint32_t CLIENT_LOST_STOP_MS = 500;
static uint32_t client_lost_t0_ms = 0;      // 失联开始时刻；0 = 当前链路正常

static bool motorsActive() {
  for (int i = 0; i < 4; i++) if (lastPwm[i] != 0) return true;
  return false;
}

void updateLinkSafety() {
  bool link_ok = (WiFi.status() == WL_CONNECTED) && client && client.connected();
  if (link_ok || !motorsActive()) { client_lost_t0_ms = 0; return; }   // 链路好 / 没在输出：不计时
  if (client_lost_t0_ms == 0) client_lost_t0_ms = millis();
  if ((uint32_t)(millis() - client_lost_t0_ms) < CLIENT_LOST_STOP_MS) return;
  Serial.println("[link] client lost >500ms while motors active: stop");
  running = false;
  ramp_active = false;
  stopAll();
  client_lost_t0_ms = 0;    // 已停；若之后仍失联，电机已是 0，不再重复触发
}

// ---------------- 输出 ----------------
// （sendLine 定义移到了 run 环形缓冲之后：它现在走 ring + 非阻塞 flush）

// 运行样本走 run_buf 环形缓冲，**边跑边发**：攒够 RUN_FLUSH_LINES 行就整批 flush，
// PC 才能在 RUN 还没结束时判断"还在跑 / 跑完了 / 死了"。
// （旧的"全攒 RAM、跑完一次性发"是为了绕开 WiFi 延迟 ACK，代价是整个运行期间 PC 收不到任何样本。）
// 注意：延迟 ACK 那个坑靠的是"**攒批再发** + 连接时 setNoDelay(true)"，不是靠"发得晚"。
// 本改动保留了攒批（一次 flush = 一次非阻塞 send，约 5 行 ≈ 450 B），不会退化成逐行小包。
//
// 【批次节奏】实测：RUN 后约 60 ms 出首批，之后每约 50 ms 一批（5 行 × 10 ms/行 = 50 ms）。
//   一行就是一个样本，不是 5 个样本：周期 = 行数 × 10 ms。
//
// 【非阻塞的由来】2026-10-14 实测复现"约 1/3 概率 ~1s 遥测黑屏"：
//   实测数据（50 次运行、PC 连续排空 + 固件侧打点）表明：
//   - Wi-Fi 省电已关（esp_wifi_get_ps=0）、Nagle 已关（getNoDelay=1）、环缓冲无丢弃；
//   - 黑屏时 loop 被冻结在 client.write/client.println 内 0.2~2.5s：
//     WiFiClient::write 内部是 select(1s)×10 的阻塞实现，socket 一旦不可写就冻住采样循环，
//     于是中段样本**根本没生成**（t_ms 出现 ~1s 缺口），期间连 PING 都不回；
//   - 同窗口 ICMP 7~30ms、独立任务 UDP 心跳 0.15s 级照常 —— 无线电层是好的，
//     卡的是这条 TCP 连接的发送路径（对端收不到任何段：PC 侧 segs_in/bytes_received 冻结）。
//   修法：写操作全部非阻塞（MSG_DONTWAIT），发不出去就留在 ring 里，采样循环永不被网络冻结。
//
// 【环形缓冲的坑】回绕时绝不能盖掉还没发出去的数据。
//   run_head —— 已写入总字节数（绝对游标，写入位 = run_head % CAP）
//   run_tail —— 已 flush 总字节数（绝对游标，发出位 = run_tail % CAP）
//   两者之差就是"还没发出去的字节数"。RUN / STOP / M 换指令时必须**两个游标一起归零**。
//   【只清一个的后果 —— 曾写反过，勿再写反】举 head=5、tail=11（只清了 head，tail 还留着上一轮的值）：
//   ESP32 的 size_t 是 32 位，run_head - run_tail = 2^32-6 = 4294967290；
//   判满式 RUN_BUF_CAP - (run_head - run_tail) 在无符号回绕后 = 32768+6 = 32774，
//   **大于任何一行需要的空间** ⇒ 缓冲被判成"还有空间"，写入继续推进、回绕后
//   **覆盖还没发出去的老数据**（不是"被判满、静默丢弃"—— 那个说法是反的）。
//   （64 位主机上同一算术会得到 2^64-6 ≈ 1.8e19 —— 那是主机侧的演示值，ESP32 上不是它。）
//   "只清 run_pending" 本身不会下溢（tail ≤ head 的不变式没被破坏）；
//   下溢只来自**两个游标的不对称复位**——所以复位一律走 resetRunBuf()。
//   回归锁：固件/exp/host_test/（把这段逻辑拷去主机跑，含不对称复位的变异自检）。
#define RUN_BUF_CAP     32768
#define RUN_FLUSH_LINES 5        // 攒够几行 flush 一批（改它只改"每批几行"，周期随之线性变化）
static char   run_buf[RUN_BUF_CAP];
static size_t run_head = 0;      // 已写入总字节数
static size_t run_tail = 0;      // 已 flush 总字节数
static int    run_pending = 0;   // 已写入、还没 flush 的整行数（flush 的触发条件）
static int    run_dropped = 0;   // 缓冲满被丢弃的行数；丢弃不静默，flush 时上报 Serial

// 清空本次运行的环形缓冲。三个复位点（RUN / STOP / M）共用，必须整体调用。
void resetRunBuf() { run_head = 0; run_tail = 0; run_pending = 0; run_dropped = 0; }

// 整行入环形缓冲；返回 false = 缓冲满、本行被丢弃。
// 满了丢**新**行而不是盖**旧**行：开头的数据比结尾的值钱（起动 kick、初始编码器零位都在开头）。
static bool runPushLine(const char* s, size_t len) {
  size_t need = len + 1;                                   // 含行尾 '\n'
  if (need > RUN_BUF_CAP) return false;
  if (RUN_BUF_CAP - (run_head - run_tail) < need) { run_dropped++; return false; }
  size_t pos   = run_head % RUN_BUF_CAP;
  size_t first = RUN_BUF_CAP - pos;                 // 回绕前还能连着放多少字节
  if (first > len) first = len;
  memcpy(run_buf + pos, s, first);                  // 前半段（不回绕）
  if (len > first) memcpy(run_buf, s + first, len - first);   // 后半段（回绕到 0）
  run_buf[(pos + len) % RUN_BUF_CAP] = '\n';
  run_head += need;
  return true;
}

// ---------------- TX 停摆看门狗（R62 防死兜底；R64 区分硬/软错误） ----------------
// 时间戳在 txSendNonBlocking 真正写出字节时刷新；判定逻辑见 checkTxStall。
// 阈值沿革：R62 取 2 s → R64 上机实测健康链路自然停摆 [0.05–2.5 s]（能自愈的最长 2.32 s），
// 2 s 落在分布内侧 ⇒ 长遥测会话被反复误踢（6.5 分钟里 3 次，长 RUN 丢 99.1 s 遥测）。
// 5 s ≈ 2× 最长自愈停摆：只兜"真死了"的连接。
static const uint32_t TX_STALL_MS = 5000;   // 连续 >5s 一个字节都没发出去 ⇒ 僵尸
static uint32_t tx_last_ok_ms = 0;          // 最近一次 send() 真正写出字节的 millis()

// send() 结果三分类（R64）。原来所有失败都折成"没发出去"（返回 0），于是
// 「对端已经没了」（EPIPE/ECONNRESET/ENOTCONN…）与「对端零窗口暂时写不动」
// （EAGAIN/EWOULDBLOCK）无法区分，只能靠超时猜。分开报之后：
//   TX_SEND_OK   写出去 >=1 字节（调用方按 *sent 推进游标）
//   TX_SEND_SOFT 暂时写不动：由 TX_STALL_MS 兜底（错判成硬会在正常零窗口上砍断连接）
//   TX_SEND_HARD 连接确实死了：立刻丢（漏判成软会退化成"只能靠超时"的旧行为）
enum TxSend : uint8_t { TX_SEND_OK, TX_SEND_SOFT, TX_SEND_HARD };

// 非阻塞发送：把数据尽量推给 TCP，写不动立刻返回，绝不阻塞 loop。
// （WiFiClient::write 是 select(1s)×10 的阻塞实现；send(MSG_DONTWAIT) 不会被它冻住。）
// *sent 只在 TX_SEND_OK 时非 0；硬/软判据见上面枚举注释。
static TxSend txSendNonBlocking(const uint8_t* p, size_t n, size_t* sent) {
  *sent = 0;
  // n==0 也归"软"：调用方 flushRunBuf 用 `r != OK → break` 决定要不要推进游标，
  // 返回 OK 却 *sent=0 会让 while(off < pend) 空转 —— 一个字节都没发出去就不该算成功。
  // （当前 flushRunBuf 的 chunk 恒 >=1，走不到这里；留着是为了不给看门狗路径埋死循环。）
  if (n == 0) return TX_SEND_SOFT;
  if (!client || !client.connected()) return TX_SEND_HARD;  // 对端已关（connected() 只探 RX）
  int fd = client.fd();
  if (fd < 0) return TX_SEND_HARD;                          // 没有有效 fd：没东西可发
  errno = 0;                                                // 防残留：w<0 时立刻读 errno
  int w = send(fd, p, n, MSG_DONTWAIT);
  if (w > 0) { tx_last_ok_ms = millis(); *sent = (size_t)w; return TX_SEND_OK; }  // 真写出字节才算"TX 还活着"
  if (w == 0) return TX_SEND_SOFT;                          // 0 字节被接受（空间不足）：可自愈，别砍
  switch (errno) {
    case EAGAIN:      // == EWOULDBLOCK（lwip）：对端零窗口 / 本端无缓冲
    case EINTR:       // 被信号打断：重试即可
    case ENOBUFS:     // lwip ERR_BUF（对端停读时常见）：暂时缺缓冲，不是连接死
    case ENOMEM:      // lwip ERR_MEM：同上
      return TX_SEND_SOFT;
    default:          // EPIPE/ECONNRESET/ENOTCONN/ECONNABORTED…：连接确实死了
      return TX_SEND_HARD;
  }
}

// 连接确实死了（或判定僵尸）：踢 socket + 清环 + 停车，把接受路径踢回可 accept 状态。
// 两个触发点共用：send() 硬错误（flushRunBuf 里立刻丢，不等阈值）与
// TX 停摆 >TX_STALL_MS（checkTxStall 里的软错误兜底）。
static void dropTxClient(const char* why) {
  Serial.printf("[tx] drop client (%s), ring cleared (pending=%u B)\n",
                why, (unsigned)(run_head - run_tail));
  client.stop();
  resetRunBuf();
  running = false;
  ramp_active = false;
  stopAll();      // 停车：TX 死掉期间 PC 已看不到遥测，而 M 没有超时，绝不能继续开环输出
}

// 把还没 flush 的字节尽量推给 TCP，绝不阻塞。只推进真正写出去的部分，
// 剩下的留到下次 flush 接着发 —— TCP 是字节流，跨批次拼接后仍能还原整行。
void flushRunBuf() {
  size_t pend = run_head - run_tail;
  if (client && client.connected() && pend) {
    size_t off = 0;
    while (off < pend) {
      size_t pos   = (run_tail + off) % RUN_BUF_CAP;
      size_t chunk = RUN_BUF_CAP - pos;
      if (chunk > pend - off) chunk = pend - off;
      size_t w = 0;
      TxSend r = txSendNonBlocking((const uint8_t*)(run_buf + pos), chunk, &w);
      if (r == TX_SEND_HARD) { dropTxClient("send() hard error"); return; }  // 连接死了：立刻踢，不等阈值
      if (r != TX_SEND_OK) break;                                             // 软错误：留着下次接着发
      off += w;
    }
    run_tail += off;
  }
  if (run_tail == run_head) {               // 全部发出去了才清计数（半发出去时保留，下次接着试）
    run_pending = 0;
    if (run_dropped) {
      Serial.printf("[run_buf] 缓冲满，丢弃 %d 行 (cap=%u B)\n", run_dropped, (unsigned)RUN_BUF_CAP);
      run_dropped = 0;
    }
  }
}

// TX 停摆看门狗（每轮 loop 在 flushRunBuf 之后调用）—— R62 的防死兜底，R64 拆成两档：
// 背景：这条 socket 会进入「TCP 层连着、send() 已不成功」的僵尸态（射频层正常；
// 停读/接收窗口关死等场景实测可确定复现）。WiFiClient::connected() 只探测 RX
// （recv(fd,&dummy,0)），完全不看 TX —— 僵尸态下它仍返回 true，于是：
//   1) flushRunBuf 反复尝试、一个字节都发不出去 → 环被新样本填满，DONE/控制行一起被丢；
//   2) 接受路径 `if (!client.connected())` 永不成立 → 新连接永不被 accept（堵死到复位）。
// 兜底分两档（判据见 txSendNonBlocking 的三分类）：
//   - 硬错误（EPIPE/ECONNRESET/ENOTCONN…）：flushRunBuf 里**立刻** dropTxClient，不等阈值；
//   - 软错误（EAGAIN 零窗口等）：本函数按 TX_STALL_MS 判僵尸。阈值 5 s 的依据：
//     R64 实测健康链路自然停摆最长 2.32 s（且之后自行排空），2 s 会误踢，5 s ≈ 2× 余量。
// 正常 100 Hz 流式（健康 socket）每批都发得出去，时间戳一直新鲜，绝不会误触发。
void checkTxStall() {
  if (run_head == run_tail) { tx_last_ok_ms = millis(); return; }   // 环空：没有可判停摆的量
  if ((uint32_t)(millis() - tx_last_ok_ms) <= TX_STALL_MS) return;  // 5s 内还有字节出去
  dropTxClient("stall >5s");      // 软错误长时间不退 ⇒ 判僵尸（硬错误在 flushRunBuf 里已立刻处理）
}

// 所有出站文本统一走 run 环形缓冲 + 非阻塞 flush：
// 控制行（PONG/OK/DONE/ERROR…）与 D 行保持同一字节流顺序，且任何写都不阻塞 loop。
void sendLine(const String& s) {
  if (!client || !client.connected()) return;
  runPushLine(s.c_str(), s.length());   // 环满时由 runPushLine 计数并丢弃（极少见）
  flushRunBuf();                        // 非阻塞，发得出去就发
}

// 非正常复位时，在 PING/ID 回复末尾补一行复位原因；正常启动不发（PC 侧解析不用改）。
static void sendResetLine() {
  if (!resetIsAbnormal()) return;
  char b[80];
  snprintf(b, sizeof(b), "RESET %s count=%lu", resetReasonName(g_reset_reason_rtc),
           (unsigned long)g_brownout_count);
  sendLine(b);
}

// tag: "D"=实验运行样本(时间=相对RUN起点)  "I"=空闲样本(时间=millis)
void streamSample(const char* tag, uint32_t t_ms) {
  long e[4]; readEncoders(e);
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "%s %lu %d %d %d %d %ld %ld %ld %ld %.2f %.2f %.2f",
           tag, (unsigned long)t_ms,
           lastPwm[0], lastPwm[1], lastPwm[2], lastPwm[3],
           e[0], e[1], e[2], e[3],
           imu_roll, imu_pitch, imu_yaw);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;     // snprintf 截断保护：只按实际写入的长度取
  if (tag[0] == 'D') {
    if (runPushLine(buf, (size_t)n)) {
      run_pending++;
      if (run_pending >= RUN_FLUSH_LINES) flushRunBuf();    // 攒够一批就发，不等 DONE
    }
  } else {
    sendLine(buf);
  }
}
uint32_t runElapsedMs() { return (uint32_t)((micros() - run_t0_us) / 1000); }

// ---------------- 命令 ----------------
void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  if (line.equalsIgnoreCase("PING")) { sendLine("PONG"); sendResetLine(); return; }
  if (line.equalsIgnoreCase("ID"))   { sendLine("MECANUM-ESP32 " + WiFi.localIP().toString()); sendResetLine(); return; }
  if (line.equalsIgnoreCase("CHIP")) {
    sendLine("chip=" + String(ESP.getChipModel()) + " cores=" + String(ESP.getChipCores())
             + " rev=" + String(ESP.getChipRevision()));
    return;
  }
  if (line.equalsIgnoreCase("HELP")) {
    sendLine("PING|ID|CHIP|M f f f f|STOP|ZENC|RUN vx vy om ms|STREAM 0/1|SNIFF|IMU <rxpin> [baud]");
    return;
  }
  if (line.equalsIgnoreCase("STOP")) { running = false; ramp_active = false; resetRunBuf(); stopAll(); sendLine("OK STOP"); return; }
  if (line.equalsIgnoreCase("YAWZERO")) { sendLine("ERR TX pin unavailable (RX-only); zero yaw in software"); return; }
  if (line.equalsIgnoreCase("ZENC")) { zeroEncoders(); sendLine("OK ZENC"); return; }
  if (line.startsWith("IMU ")) {
    int pin = 37, baud = 9600;
    int n = sscanf(line.c_str(), "IMU %d %d", &pin, &baud);
    if (n >= 1) { imuSetPin(pin); }
    char b[64]; snprintf(b, sizeof(b), "OK IMU rx=%d baud=%d ok=%d", IMU_RX_PIN, baud, imu_ok ? 1 : 0);
    sendLine(b);
    return;
  }
  if (line.equalsIgnoreCase("SNIFF")) {
    Serial2.end();
    edge37 = 0; edge38 = 0;
    attachInterrupt(digitalPinToInterrupt(37), isr37, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), isr38, CHANGE);
    delay(1000);
    uint32_t e37 = edge37, e38 = edge38;
    detachInterrupt(digitalPinToInterrupt(37));
    detachInterrupt(digitalPinToInterrupt(38));
    Serial2.begin(9600, SERIAL_8N1, IMU_RX_PIN, -1);
    char b[96];
    snprintf(b, sizeof(b), "SNIFF 1s: gpio37_edges=%lu gpio38_edges=%lu (接IMU-TX的脚边沿多)",
             (unsigned long)e37, (unsigned long)e38);
    sendLine(b);
    return;
  }

  if (line.startsWith("M ")) {
    int d[4] = {0, 0, 0, 0};
    if (sscanf(line.c_str(), "M %d %d %d %d", &d[0], &d[1], &d[2], &d[3]) == 4) {
      running = false;
      resetRunBuf();
      // 直接驱动：kick 幅值 = 目标值（无 kick 语义），但同样走错峰放行（brownout 也来自 M 四轮同驱）
      for (int i = 0; i < 4; i++) { ramp_target[i] = d[i]; ramp_kick[i] = d[i]; }
      ramp_t0_us = micros();
      ramp_active = true;
      applyRamp();          // 立即放行第 0 个轮子（其余轮子由 loop 的窗口块按点放行）
      sendLine("OK M");
    } else sendLine("ERR need 4 ints");
    return;
  }
  if (line.startsWith("STREAM")) {
    int v = 0;
    if (sscanf(line.c_str(), "STREAM %d", &v) == 1) {
      stream_idle = (v != 0);
      sendLine(String("OK STREAM ") + (stream_idle ? "1" : "0"));
    } else sendLine("ERR STREAM 0/1");
    return;
  }
  if (line.startsWith("RUN")) {
    float vx = 0, vy = 0, om = 0; long ms = 0;
    if (sscanf(line.c_str(), "RUN %f %f %f %ld", &vx, &vy, &om, &ms) == 4 && ms > 0) {
      zeroEncoders();
      resetRunBuf();        // 环形缓冲的读写游标必须一起归零（run_pending 也在这里清）
      bodyToPWM(vx, vy, om, run_wheel_pwm);
      // 起步 kick：每个转动轮的幅值至少抬到 KICK_DUTY，方向与正常一致；停转轮保持 0。
      // 错峰抬升开启时，各轮的 kick 从**它自己的放行时刻**起算（见 applyRamp/rampSpanMs）。
      for (int i = 0; i < 4; i++) {
        int base = run_wheel_pwm[i];
        int kick = 0;
        if (base != 0) {
          int mag = base < 0 ? -base : base;
          if (mag < KICK_DUTY) mag = KICK_DUTY;
          kick = (base < 0) ? -mag : mag;
        }
        ramp_kick[i]   = kick;
        ramp_target[i] = base;
      }
      run_t0_us  = micros();
      run_dur_us = (uint32_t)ms * 1000UL;
      ramp_t0_us = run_t0_us;
      ramp_active = true;
      running = true;
      applyRamp();          // 立即放行第 0 个轮子（其余轮子由 loop 的窗口块按点放行）
      char b[112];
      snprintf(b, sizeof(b), "OK RUN pwm=%d,%d,%d,%d ms=%ld kick=%dms/%d",
               run_wheel_pwm[0], run_wheel_pwm[1], run_wheel_pwm[2], run_wheel_pwm[3], ms,
               (int)KICK_MS, KICK_DUTY);
      sendLine(b);
    } else sendLine("ERR RUN vx vy om ms");
    return;
  }
  sendLine("ERR unknown");
}

void setup() {
  setupMotors();          // 必须最先：否则电机驱动输入悬空，上电瞬间乱转
  stopAll();              // 且会污染编码器计数
  setupEncoders();
  zeroEncoders();
  initResetDiag();        // 尽早：读复位原因、维护 brownout 计数（RESET 行与启动日志都用它）

  Serial.begin(115200);
  // 经典 ESP32 的 GPIO38 是「只输入」，不能当 UART TX（会 panic）。
  // GY-25T 默认连续输出，只收不发，所以只开 RX=37。
  Serial2.begin(9600, SERIAL_8N1, IMU_RX_PIN, -1);
  delay(200);

  Serial.println("\n=== MECANUM experiment FW ===");
  Serial.printf("chip=%s cores=%d\n", ESP.getChipModel(), ESP.getChipCores());
  Serial.printf("reset_reason=%s brownouts=%lu stagger=%s step=%ums\n",
                resetReasonName(g_reset_reason_rtc), (unsigned long)g_brownout_count,
                STAGGER_ENABLED ? "on" : "off", (unsigned)STAGGER_STEP_MS);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(400); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi OK IP=%s RSSI=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    server.begin();
    udp.begin(UDP_PORT);
    ArduinoOTA.setHostname("mecanum");
    ArduinoOTA.begin();
    Serial.println("OTA hostname=mecanum ; TCP 8266 ; UDP 8267");
  } else {
    Serial.printf("WiFi FAILED status=%d\n", (int)WiFi.status());
  }
  next_sample_us = micros();
}

void loop() {
  ArduinoOTA.handle();
  imuPump();

  // 失联停车：先于 WiFi 断线的 early-return（否则 RUN 的到时检查走不到）。
  updateLinkSafety();

  // 错峰抬升窗口：每次迭代重算四轮输出；最后一轮放行且它的 kick 到时后，
  // 收敛到目标值并关闭窗口（此后轮子保持目标 PWM，不再重算）。
  if (ramp_active) {
    uint32_t elapsed_ms = (uint32_t)((micros() - ramp_t0_us) / 1000);
    if (elapsed_ms >= rampSpanMs()) {
      ramp_active = false;
      for (int i = 0; i < 4; i++) setMotor(i, ramp_target[i]);
    } else {
      applyRamp();
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 3000) { lastTry = millis(); WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); }
    return;
  }

  // UDP 发现
  int pkt = udp.parsePacket();
  if (pkt) {
    char b[32]; int n = udp.read(b, sizeof(b) - 1); if (n < 0) n = 0; b[n] = 0;
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.printf("MECANUM %s %u\n", WiFi.localIP().toString().c_str(), TCP_PORT);
    udp.endPacket();
  }

  // TCP 客户端
  if (!client || !client.connected()) {
    client.stop();             // 先显式释放旧 socket 的 fd（含僵尸），再让 available() 覆盖
    client = server.available();
    if (client) {
      client.setNoDelay(true);   // 关 Nagle：否则 100Hz 小包会被 delayed-ACK 攒成突发、延迟数秒
      resetRunBuf();             // 旧会话残留的字节已无人接收（僵尸期堆的），别混进新会话
      Serial.printf("TCP client %s\n", client.remoteIP().toString().c_str());
    }
  }
  if (client && client.connected()) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      handleLine(line);
    }
  }

  flushRunBuf();   // 周期性非阻塞 flush：把 ring 里未发出的字节（含控制行）继续推出去
  checkTxStall();  // TX 停摆看门狗：僵尸 socket 主动踢掉（R62 防死兜底）

  // 100 Hz 采样
  uint32_t now = micros();
  if ((int32_t)(now - next_sample_us) >= 0) {
    next_sample_us += 10000;               // 10 ms
    if ((int32_t)(now - next_sample_us) > 20000) next_sample_us = now + 10000;  // 落太多就重对齐

    if (running) {
      streamSample("D", runElapsedMs());
      if ((micros() - run_t0_us) >= run_dur_us) {
        running = false;
        ramp_active = false;      // 跑得比错峰窗口还短的 RUN：收窗口，别让 ramp 再把轮子点起来
        stopAll();
        streamSample("D", runElapsedMs());   // 停车后的最后一个样本，仍用相对时间
        flushRunBuf();                        // 把末尾不足一批的残余样本发出去（流式之后这里只清尾账）
        sendLine("DONE");
      }
    } else if (stream_idle) {
      streamSample("I", millis());
    }
  }
}
