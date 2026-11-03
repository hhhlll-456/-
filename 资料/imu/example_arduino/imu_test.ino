/*
 * example_arduino/imu_test.ino
 * GY-25T 读取 + 初始化示例（Arduino）
 *
 * 接线（软串口示例，与官方 arduino_usart 一致）：
 *   GY25T VCC -> VCC(5V)   GND -> GND
 *   GY25T RC(TX) -> D10    GY25T TD(RX) -> D11
 *   若用硬件串口，把 imu 换成 Serial1/Serial2 即可（RX/TX 引脚按你的板子）。
 *
 * 把 gy25t.h / gy25t.c 放到与本 .ino 同目录（或库目录）。
 */
#include <SoftwareSerial.h>
#include "gy25t.h"

SoftwareSerial imu(10, 11);          // RX, TX

/* 发送回调：交给 gy25t.c 调用 */
static int imu_tx(const uint8_t *buf, uint16_t len)
{
    return imu.write(buf, len);
}

void setup()
{
    Serial.begin(115200);
    imu.begin(9600);
    GY25T_Init(imu_tx);              // 注册回调 + 配连续输出
    delay(300);                      // 等模块上电稳定

    /* —— 关键：初始化要“车静止 + 水平 + 摆到期望的 0° 朝向” —— */
    GY25T_Calibrate();               // 陀螺+加速度校准，约 1s 内别动
    delay(1200);
    GY25T_ZeroYaw();                 // 把当前朝向钉成 0°
    delay(50);

    Serial.println("GY-25T ready, yaw has been zeroed.");
}

void loop()
{
    while (imu.available())
        GY25T_FeedByte((uint8_t)imu.read());   // 逐字节喂解析器

    if (GY25T_HasNew()) {
        const GY25T_Data *d = GY25T_Peek();
        Serial.print("roll=");  Serial.print(d->roll, 2);
        Serial.print(" pitch="); Serial.print(d->pitch, 2);
        Serial.print(" yaw=");  Serial.print(d->yaw, 2);
        Serial.print(" yaw_cont="); Serial.print(GY25T_YawContinuous(), 2);
        Serial.print(" temp="); Serial.println(d->temp, 2);
    }
}
