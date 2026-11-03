#include <reg52.h>
#include "usart.h"
#include "iic.h"  
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
该程序采用串口方式获取模块数据，波特率9600行以上操作：
注：中断函数位于stc_it.c
联系方式：
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.9qa3Ky&v=1
*/
char putchar (char c)   
{        
    ES=0;        
    SBUF = c;        
    while(TI==0);        
    TI=0;        
    ES=1;        
    return 0;
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
	gy my_25t;
	Usart_Int(9600);
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
