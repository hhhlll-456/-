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
#include "wheel_map.h"

// ---------------- 网络 ----------------
static const char*    WIFI_SSID = "B1222";
static const char*    WIFI_PASS = "12221222";
static const uint16_t TCP_PORT  = 8266;
static const uint16_t UDP_PORT  = 8267;

WiFiServer server(TCP_PORT);
WiFiUDP    udp;
WiFiClient client;

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
static const uint32_t KICK_MS   = 150;   // 脉冲时长(ms)
bool     kicking      = false;
uint32_t kick_until_us = 0;

// ---------------- 输出 ----------------
void sendLine(const String& s) {
  if (client && client.connected()) client.println(s);
}

// 运行样本先攒在 RAM，跑完一次性发；逐条 100Hz 小包直发会被 WiFi/延迟ACK 拖到只剩约 1/3
static char   run_buf[32768];
static size_t run_len = 0;

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
  if (tag[0] == 'D') {
    if (run_len + (size_t)n + 1 < sizeof(run_buf)) {
      memcpy(run_buf + run_len, buf, n);
      run_len += n;
      run_buf[run_len++] = '\n';
    }
  } else {
    sendLine(buf);
  }
}
void flushRunBuf() {
  if (client && client.connected() && run_len) {
    size_t off = 0;
    while (off < run_len) {
      size_t w = client.write((const uint8_t*)(run_buf + off), run_len - off);
      if (w == 0) break;
      off += w;
    }
    client.flush();
  }
  run_len = 0;
}
uint32_t runElapsedMs() { return (uint32_t)((micros() - run_t0_us) / 1000); }

// ---------------- 命令 ----------------
void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  if (line.equalsIgnoreCase("PING")) { sendLine("PONG"); return; }
  if (line.equalsIgnoreCase("ID"))   { sendLine("MECANUM-ESP32 " + WiFi.localIP().toString()); return; }
  if (line.equalsIgnoreCase("CHIP")) {
    sendLine("chip=" + String(ESP.getChipModel()) + " cores=" + String(ESP.getChipCores())
             + " rev=" + String(ESP.getChipRevision()));
    return;
  }
  if (line.equalsIgnoreCase("HELP")) {
    sendLine("PING|ID|CHIP|M f f f f|STOP|ZENC|RUN vx vy om ms|STREAM 0/1|SNIFF|IMU <rxpin> [baud]");
    return;
  }
  if (line.equalsIgnoreCase("STOP")) { running = false; kicking = false; run_len = 0; stopAll(); sendLine("OK STOP"); return; }
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
      running = false; kicking = false; run_len = 0;
      for (int i = 0; i < 4; i++) setMotor(i, d[i]);
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
      run_len = 0;
      bodyToPWM(vx, vy, om, run_wheel_pwm);
      // 起步 kick：每个转动轮的幅值至少抬到 KICK_DUTY，方向与正常一致；停转轮保持 0
      for (int i = 0; i < 4; i++) {
        int base = run_wheel_pwm[i];
        int kick = 0;
        if (base != 0) {
          int mag = base < 0 ? -base : base;
          if (mag < KICK_DUTY) mag = KICK_DUTY;
          kick = (base < 0) ? -mag : mag;
        }
        setMotor(i, kick);
      }
      run_t0_us = micros();
      run_dur_us = (uint32_t)ms * 1000UL;
      kick_until_us = run_t0_us + (uint32_t)KICK_MS * 1000UL;
      kicking = true;
      running = true;
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

  Serial.begin(115200);
  // 经典 ESP32 的 GPIO38 是「只输入」，不能当 UART TX（会 panic）。
  // GY-25T 默认连续输出，只收不发，所以只开 RX=37。
  Serial2.begin(9600, SERIAL_8N1, IMU_RX_PIN, -1);
  delay(200);

  Serial.println("\n=== MECANUM experiment FW ===");
  Serial.printf("chip=%s cores=%d\n", ESP.getChipModel(), ESP.getChipCores());

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

  // 起步 kick 到时 -> 回落到正常映射 PWM
  if (kicking && (int32_t)(micros() - kick_until_us) >= 0) {
    kicking = false;
    for (int i = 0; i < 4; i++) setMotor(i, run_wheel_pwm[i]);
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
    client = server.available();
    if (client) {
      client.setNoDelay(true);   // 关 Nagle：否则 100Hz 小包会被 delayed-ACK 攒成突发、延迟数秒
      Serial.printf("TCP client %s\n", client.remoteIP().toString().c_str());
    }
  }
  if (client && client.connected()) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      handleLine(line);
    }
  }

  // 100 Hz 采样
  uint32_t now = micros();
  if ((int32_t)(now - next_sample_us) >= 0) {
    next_sample_us += 10000;               // 10 ms
    if ((int32_t)(now - next_sample_us) > 20000) next_sample_us = now + 10000;  // 落太多就重对齐

    if (running) {
      streamSample("D", runElapsedMs());
      if ((micros() - run_t0_us) >= run_dur_us) {
        running = false;
        kicking = false;
        stopAll();
        streamSample("D", runElapsedMs());   // 停车后的最后一个样本，仍用相对时间
        flushRunBuf();                        // 一次性把本次运行样本发出去
        sendLine("DONE");
      }
    } else if (stream_idle) {
      streamSample("I", millis());
    }
  }
}
