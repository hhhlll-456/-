#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "string.h"
#include "LED.h"
#include "stdio.h"
/*

Keil: MDK5.10.0.2
MCU:stm32f103c8
硬件接法：
GY-25t---STM32
1、GY-25t_RX---STM32_TX
2、STM32_TX---FT232,STM32将数据上传给上位机
3、GY-25t_TX---STM32_RX，接收模块数据
软件说明:
该程序采用串口方式获取模块数据，波特率9600

注：中断函数位于stm32f10x_it.c
联系方式：
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.9qa3Ky&v=1
*/
int fputc(int ch, FILE *f)
{
    while (!(USART1->SR & USART_FLAG_TXE));
    USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等
    return (ch);
}
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_X;

    /* 4个抢占优先级，4个响应优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /*抢占优先级可打断中断级别低的中断*/
    /*响应优先级按等级执行*/
    NVIC_X.NVIC_IRQChannel = USART1_IRQn;//中断向量
    NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
    NVIC_X.NVIC_IRQChannelSubPriority = 0;//响应优先级
    NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
    NVIC_Init(&NVIC_X);
}
typedef struct
{
    int16_t temp;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} gy;
int main(void)
{
    u8 sum=0,i=0;
    int16_t data=0;
    gy my_25t= {0,0,0,0};
    delay_init(72);
    NVIC_Configuration();
    Usart_Int(9600);
    delay_ms(300);//等待模块初始化完成
    while(1)
    {
        if(Receive_ok)//串口接收完毕
        {

            for(sum=0,i=0; i<(usart_rx_data[3]+4); i++) //rgb_data[3]=3
                sum+=usart_rx_data[i];
            if(sum==usart_rx_data[i])//校验和判断
            {
                my_25t.roll=(usart_rx_data[4]<<8)|usart_rx_data[5];
                my_25t.pitch=(usart_rx_data[6]<<8)|usart_rx_data[7];
                my_25t.yaw=(usart_rx_data[8]<<8)|usart_rx_data[9];
                my_25t.temp=(usart_rx_data[10]<<8)|usart_rx_data[12];
                printf("roll:%.2f,",(float)my_25t.roll/100.0f);
                printf("pitch:%.2f,",(float)my_25t.pitch/100.0f);
                printf("yaw:%.2f,",(float)my_25t.yaw/100);
                printf("temp:%.2f\r\n ",(float)my_25t.temp/100.0f);
            }
            else
            {
                printf(" sum %d\r\n ",sum);
                printf(" count %d\r\n ",usart_rx_data[3]+4);
            }
            Receive_ok=0;//处理数据完毕标志
        }
    }
}
