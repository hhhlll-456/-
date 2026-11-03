/*
 * gy25t.c  —  便携式 GY-25T 驱动实现
 * 见 gy25t.h 顶部说明。
 */
#include "gy25t.h"

#define GY25T_ID      0xA4u   /* 0x00 寄存器，默认 ID/设备地址 */
#define GY25T_FUNC_READ  0x03u
#define GY25T_FUNC_WRITE 0x06u

/* 寄存器 */
#define REG_RATE      0x02u
#define REG_MODE      0x03u
#define REG_SAVE      0x05u
#define REG_CAL       0x06u
#define REG_EULER     0x14u   /* ROLL_H 起始；连续 8 个寄存器 = R/P/Y/T */

#define CAL_GYRO_ACC  0x10u
#define CAL_YAW_ZERO  0x01u
#define SAVE_MAGIC    0x55u

static gy25t_tx_fn s_tx = 0;

static GY25T_Data  s_data;       /* 对外发布的最新数据 */
static volatile uint8_t s_new = 0;

/* 连续航向解卷绕 */
static float   s_yaw_cont = 0.0f;
static int16_t s_yaw_prev = 0;
static uint8_t s_yaw_have_prev = 0;

/* ——— 组帧并发送 ——— */
static void tx_frame(uint8_t *f, uint8_t n)
{
    uint8_t i, sum = 0;
    for (i = 0; i + 1 < n; i++) sum += f[i];  /* 除最后一字节外全部累加 */
    f[n - 1] = sum;                            /* 末字节 = 校验和(低8位) */
    if (s_tx) (void)s_tx(f, n);
}

/* 写寄存器：A4 06 reg data sum */
static void write_reg(uint8_t reg, uint8_t data)
{
    uint8_t f[5];
    f[0] = GY25T_ID;
    f[1] = GY25T_FUNC_WRITE;
    f[2] = reg;
    f[3] = data;
    tx_frame(f, 5);
}

/* 读寄存器：A4 03 start count sum */
static void read_reg(uint8_t start, uint8_t count)
{
    uint8_t f[5];
    f[0] = GY25T_ID;
    f[1] = GY25T_FUNC_READ;
    f[2] = start;
    f[3] = count;
    tx_frame(f, 5);
}

/* ——— 接收状态机 ——— */
enum { S_ID = 0, S_FUNC, S_REG, S_LEN, S_DATA, S_SUM };
static uint8_t s_state = S_ID;
static uint8_t s_buf[40];     /* [0]=ID [1]=func [2]=reg [3]=len [4..]=data */
static uint8_t s_len = 0, s_pos = 0;
static uint8_t s_sum = 0;

/* 解析已校验正确的帧（buf 长度 = 4 + len） */
static void parse_frame(void)
{
    uint8_t reg = s_buf[2];

    if (reg == REG_EULER && s_len >= 6) {
        const uint8_t *d = &s_buf[4];
        s_data.raw_roll  = (int16_t)((d[0] << 8) | d[1]);
        s_data.raw_pitch = (int16_t)((d[2] << 8) | d[3]);
        s_data.raw_yaw   = (int16_t)((d[4] << 8) | d[5]);
        if (s_len >= 8)
            s_data.temp = ((int16_t)((d[6] << 8) | d[7])) / 100.0f;

        s_data.roll  = s_data.raw_roll  / 100.0f;
        s_data.pitch = s_data.raw_pitch / 100.0f;
        s_data.yaw   = s_data.raw_yaw   / 100.0f;
        s_data.valid = 1;

        /* 连续航向解卷绕（处理 ±180 跳变） */
        if (!s_yaw_have_prev) {
            s_yaw_cont = s_data.yaw;
            s_yaw_have_prev = 1;
        } else {
            int16_t delta = s_data.raw_yaw - s_yaw_prev;   /* 原始值比较，避免浮点 */
            s_yaw_cont += delta / 100.0f;
        }
        s_yaw_prev = s_data.raw_yaw;

        s_new = 1;
    }
}

int GY25T_FeedByte(uint8_t b)
{
    switch (s_state) {
    case S_ID:
        if (b == GY25T_ID) { s_buf[0] = b; s_sum = b; s_state = S_FUNC; }
        break;

    case S_FUNC:
        if (b == GY25T_FUNC_READ) {
            s_buf[1] = b; s_sum += b; s_state = S_REG;
        } else if (b == GY25T_ID) {      /* 重新同步 */
            s_buf[0] = b; s_sum = b;
        } else {
            s_state = S_ID;              /* 丢弃（例如写响应帧 A4 06 ...） */
        }
        break;

    case S_REG:
        s_buf[2] = b; s_sum += b; s_state = S_LEN;
        break;

    case S_LEN:
        s_buf[3] = b; s_sum += b; s_pos = 0;
        if (b == 0 || b > (sizeof(s_buf) - 4)) s_state = S_ID;
        else s_state = S_DATA;
        break;

    case S_DATA:
        s_buf[4 + s_pos] = b; s_sum += b;
        if (++s_pos >= s_len) s_state = S_SUM;
        break;

    case S_SUM:
    default:
        s_state = S_ID;
        if (b == (uint8_t)s_sum) { parse_frame(); return 1; }
        break;
    }
    return 0;
}

int GY25T_HasNew(void)        { return s_new; }
const GY25T_Data *GY25T_Peek(void) { return &s_data; }
void GY25T_Get(GY25T_Data *out)    { if (out) *out = s_data; s_new = 0; }

/* ——— 初始化 / 校准 ——— */
int GY25T_Init(gy25t_tx_fn tx)
{
    if (!tx) return -1;
    s_tx = tx;
    s_state = S_ID;
    s_new = 0;
    s_yaw_have_prev = 0;
    s_yaw_cont = 0.0f;

    /* 先请求欧拉角(0x14 起 8 个)，再置连续输出 */
    read_reg(REG_EULER, 8);
    write_reg(REG_MODE, 0x00);     /* 0 = 连续输出 */
    return 0;
}

void GY25T_ZeroYaw(void)
{
    write_reg(REG_CAL, CAL_YAW_ZERO);
    s_yaw_have_prev = 0;           /* 本地连续航向一并归零 */
    s_yaw_cont = 0.0f;
}

void GY25T_Calibrate(void)
{
    write_reg(REG_CAL, CAL_GYRO_ACC);
}

void GY25T_SetRate(uint8_t rate)  { if (rate <= 3) write_reg(REG_RATE, rate); }
void GY25T_SetContinuous(void)    { write_reg(REG_MODE, 0x00); }
void GY25T_Save(void)             { write_reg(REG_SAVE, SAVE_MAGIC); }

float GY25T_YawContinuous(void)   { return s_yaw_cont; }
