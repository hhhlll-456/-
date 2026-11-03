/*
 * example_stm32/main_imu.c
 * GY-25T 读取 + 初始化示例（STM32F103 + StdPeriph，9600bps，USART1）
 *
 * 接线（与官方 STM32_USART 例子一致）：
 *   1) GY-25t_RX -> STM32_TX(PA9)
 *   2) STM32_TX(PA9) -> FT232(PC 串口，用于看 printf)
 *   3) GY-25t_TX -> STM32_RX(PA10)   （读模块）
 *   VCC->3V3/5V, GND->GND
 *
 * 把 gy25t.h / gy25t.c 加入工程；本文件替代 main.c。
 * 若沿用官方 usart.c，请注意：把它的接收中断改成调用 GY25T_FeedByte()，
 * 下面给的 USART1_IRQHandler 可作为参考（放在 stm32f10x_it.c 里）。
 */
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"      /* 官方例程里的 Usart_Int(baud) */
#include "stdio.h"
#include "gy25t.h"

int fputc(int ch, FILE *f)          /* 让 printf 走 USART1 */
{
    while (!(USART1->SR & USART_FLAG_TXE));
    USART_SendData(USART1, (unsigned char)ch);
    return ch;
}

/* 发送回调：GY25T 驱动通过它把命令帧发给模块 */
static int imu_tx(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while (!(USART1->SR & USART_FLAG_TXE));
        USART1->DR = buf[i];
    }
    return len;
}

/* 接收中断：每来一个字节就喂给解析器（放到 stm32f10x_it.c） */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        GY25T_FeedByte(b);
    }
}

static void nvic_config(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    n.NVIC_IRQChannel = USART1_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

int main(void)
{
    delay_init(72);
    nvic_config();
    Usart_Int(9600);          /* PA9/PA10, 8N1 */
    GY25T_Init(imu_tx);
    delay_ms(300);            /* 等模块上电稳定 */

    /* 关键：车静止、水平、摆到期望的 0° 朝向时再执行 */
    GY25T_Calibrate();        /* 陀螺+加速度校准，约 1s 内别动 */
    delay_ms(1200);
    GY25T_ZeroYaw();          /* 当前朝向 = 0° */
    delay_ms(50);

    while (1) {
        if (GY25T_HasNew()) {
            const GY25T_Data *d = GY25T_Peek();
            printf("roll=%.2f pitch=%.2f yaw=%.2f yaw_cont=%.2f temp=%.2f\r\n",
                   d->roll, d->pitch, d->yaw, GY25T_YawContinuous(), d->temp);
        }
    }
}
