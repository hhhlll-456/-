/*
 * gy25t.h  —  便携式 GY-25T 角度模块驱动（协议帧解析 + 归零/校准）
 *
 * 依据《GY25T模块说明v1》：
 *   串口帧（模块 -> 主机，读响应）： ID(0xA4) 0x03 起始寄存器 寄存器数量 data... 校验和
 *   写帧（主机 -> 模块）：          ID(0xA4) 0x06 寄存器   数据     校验和
 *   校验和 = 本帧前面所有字节之和，取低 8 位。
 *   默认：9600bps，8N1，连续输出，欧拉角起始寄存器 0x14。
 *
 * 关键寄存器：
 *   0x02 更新速率 0:10hz 1:50hz 2:100hz(默认) 3:200hz
 *   0x03 输出模式 0:连续(默认) 1:查询
 *   0x05 保存设置 0x55 保存 / 0xAA 恢复出厂
 *   0x06 校准     0x10 陀螺+加速度校准 / 0x01 航向归零
 *   0x14..0x19  ROLL/PITCH/YAW (各 H,L)   0x1A..0x1B TEMP
 *
 * 本驱动不依赖任何具体单片机库：只需提供一个“发送 N 字节”的回调，
 * 并把串口收到的每个字节喂给 GY25T_FeedByte()（可在中断里调用）。
 */
#ifndef GY25T_H
#define GY25T_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一帧欧拉角数据 */
typedef struct {
    float   roll;      /* 横滚角, 度        */
    float   pitch;     /* 俯仰角, 度        */
    float   yaw;       /* 航向角, 度, -180~180 */
    float   temp;      /* 温度, 摄氏度      */
    int16_t raw_roll;  /* 原始值(x100) */
    int16_t raw_pitch;
    int16_t raw_yaw;
    uint8_t valid;     /* 至少收到过一帧有效数据 */
} GY25T_Data;

/* 发送回调：把 buf 的 len 个字节写到串口。返回实际发送字节数。 */
typedef int (*gy25t_tx_fn)(const uint8_t *buf, uint16_t len);

/* 初始化：注册发送回调，并把模块配成“连续输出欧拉角”。
 * tx 不能为空。返回 0 成功。 */
int   GY25T_Init(gy25t_tx_fn tx);

/* 把串口收到的 1 个字节喂进来（建议在接收中断里调用）。
 * 返回 1 表示本字节恰好凑成一个校验正确的欧拉角帧（此时数据已更新）。 */
int   GY25T_FeedByte(uint8_t b);

/* 是否有尚未读取的新帧。GYY25T_Get/Peek 会清除该标志。 */
int   GY25T_HasNew(void);
void  GY25T_Get(GY25T_Data *out);      /* 取最新数据并清 new 标志 */
const GY25T_Data *GY25T_Peek(void);    /* 直接读最新数据副本 */

/* —— 初始化（在你把模块固定好、车摆正之后调用）—— */

/* 航向归零：把“当前朝向”设为 0°。车停在期望的 0° 朝向时调用。 */
void  GY25T_ZeroYaw(void);

/* 陀螺+加速度校准：调用时模块必须静止且水平，约 1 秒内不要动。 */
void  GY25T_Calibrate(void);

/* 配置：输出速率 / 是否连续输出 / 掉电保存 */
void  GY25T_SetRate(uint8_t rate);        /* 0..3 -> 10/50/100/200 Hz */
void  GY25T_SetContinuous(void);          /* 连续输出 */
void  GY25T_Save(void);                   /* 掉电保存当前配置 */

/* 连续航向角（度，可超过 ±180，带解卷绕），适合做航向 PID 的反馈量。
 * 内部会随 GY25T_ZeroYaw() 一起清零。 */
float GY25T_YawContinuous(void);

#ifdef __cplusplus
}
#endif
#endif /* GY25T_H */
