/*
 * 麦轮小车 Wi-Fi 控制固件（第一步：建立「电脑 → 小车」命令链路）
 *
 * 功能：
 *  1) ESP32 以 STA 模式连接 Wi-Fi  B1222 / 12221222
 *  2) 开 TCP 服务器（端口 8266）接收文本行命令
 *  3) 开 UDP 发现（端口 8267）：收到任意包即回 "MECANUM <ip> <tcpport>"
 *  4) TCP 命令：
 *       PING                 -> PONG
 *       ID                   -> MECANUM-ESP32 <ip>
 *       M <fl> <fr> <rl> <rr> -> 设置四轮 PWM(-255..255)，回 OK
 *       STOP                 -> 四轮停，回 OK
 *       HELP                 -> 命令列表
 *  5) 上电时电机引脚全部写 0，避免浮空乱转
 *
 * 引脚（与之前标定一致）：FL{13,15} FR{12,14} RL{27,26} RR{25,33}
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- 网络参数 ----------
static const char* WIFI_SSID = "B1222";
static const char* WIFI_PASS = "12221222";
static const uint16_t TCP_PORT = 8266;
static const uint16_t UDP_PORT = 8267;

WiFiServer server(TCP_PORT);
WiFiUDP    udp;

// ---------- 电机 ----------
const int MOTOR_PINS[4][2] = {{13, 15}, {12, 14}, {27, 26}, {25, 33}};
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i * 2,     PWM_FREQ, PWM_RES);
    ledcSetup(i * 2 + 1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i][0], i * 2);
    ledcAttachPin(MOTOR_PINS[i][1], i * 2 + 1);
    ledcWrite(i * 2,     0);
    ledcWrite(i * 2 + 1, 0);
  }
}

void setMotor(int wheel, int duty) {
  duty = constrain(duty, -255, 255);
  if (duty >= 0) {
    ledcWrite(wheel * 2,     duty);
    ledcWrite(wheel * 2 + 1, 0);
  } else {
    ledcWrite(wheel * 2,     0);
    ledcWrite(wheel * 2 + 1, -duty);
  }
}

void stopAll() {
  for (int i = 0; i < 4; i++) setMotor(i, 0);
}

// ---------- 命令处理 ----------
void handleLine(String line, Print& out) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("PING")) {
    out.println("PONG");
  } else if (line.equalsIgnoreCase("ID")) {
    out.printf("MECANUM-ESP32 %s\n", WiFi.localIP().toString().c_str());
  } else if (line.startsWith("M ") || line.equalsIgnoreCase("M")) {
    int d[4] = {0, 0, 0, 0};
    int n = sscanf(line.c_str(), "M %d %d %d %d", &d[0], &d[1], &d[2], &d[3]);
    if (n == 4) {
      for (int i = 0; i < 4; i++) setMotor(i, d[i]);
      out.printf("OK M %d %d %d %d\n", d[0], d[1], d[2], d[3]);
    } else {
      out.println("ERR need 4 ints: M <fl> <fr> <rl> <rr>");
    }
  } else if (line.equalsIgnoreCase("STOP")) {
    stopAll();
    out.println("OK STOP");
  } else if (line.equalsIgnoreCase("HELP")) {
    out.println("PING | ID | M <fl> <fr> <rl> <rr> | STOP");
  } else {
    out.printf("ERR unknown: %s\n", line.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setupMotors();               // 先保证电机是停的
  stopAll();

  Serial.println();
  Serial.println("=== MECANUM Wi-Fi control ===");
  Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi OK. IP=%s  RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    server.begin();
    udp.begin(UDP_PORT);
    Serial.printf("TCP server on %s:%u ; UDP discovery on :%u\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT, UDP_PORT);
  } else {
    Serial.printf("WiFi FAILED, status=%d\n", (int)WiFi.status());
  }
}

void loop() {
  // Wi-Fi 掉线自动重连
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      Serial.println("WiFi reconnect...");
    }
    return;
  }

  // UDP 发现：收到任何包就回自己的地址
  int pkt = udp.parsePacket();
  if (pkt) {
    char buf[64];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n < 0) n = 0;
    buf[n] = 0;
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.printf("MECANUM %s %u\n", WiFi.localIP().toString().c_str(), TCP_PORT);
    udp.endPacket();
    Serial.printf("UDP discover from %s ('%s') -> replied\n",
                  udp.remoteIP().toString().c_str(), buf);
  }

  // TCP 命令
  WiFiClient client = server.available();
  if (client) {
    Serial.printf("TCP client from %s\n", client.remoteIP().toString().c_str());
    String line;
    unsigned long lastData = millis();
    while (client.connected() && millis() - lastData < 60000) {
      while (client.available()) {
        char c = client.read();
        lastData = millis();
        if (c == '\n' || c == '\r') {
          if (line.length()) {
            Serial.printf("CMD: %s\n", line.c_str());
            handleLine(line, client);
            line = "";
          }
        } else if (line.length() < 120) {
          line += c;
        }
      }
      delay(1);
    }
    client.stop();
    Serial.println("TCP client closed");
    stopAll();                 // 断线即停车，安全
  }
}
