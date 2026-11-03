#include "stc_it.h"
#include "usart.h"
#include "string.h"
//外部中断1
uint8_t stata=0;
void EXTI1_IRQHandler(void)interrupt 2
{
}
//串口中断
void USART_IRQHandler(void)interrupt 4
{
	static uint8_t i=0,rebuf[30]={0};
	uint8_t sum=0;
    if(TI)//发送完成标志
	{
	  TI=0;//清发送完成标志
	  send_ok=0;//缓存标志置0 
	}
	if(RI)//接收完成标志
	{
		rebuf[i++]=SBUF;
		RI=0;//清中断接收标志
        if (rebuf[0]!=add)//帧头不对
            i=0;
        else if ((i==2)&&(rebuf[1]!=0x03))//帧头不对
            i=0;
        else if ((i==3)&&(rebuf[2]>0x1d))//帧头不对
            i=0;
        else if ((i==4)&&(rebuf[3]>0x1d))//帧头不对
            i=0;
        if(i>3)//i等于4时，已经接收到数据量字节rebuf[3]
        {
            if(i<34)
            {
                if(i==(rebuf[3]+5))//判断是否接收一帧数据完毕
                {

                    if(!Receive_ok)//当数据处理完成后才接收新的数据
                    {
                        memcpy(usart_rx_data,rebuf,i);//拷贝接收到的数据
                        Receive_ok=1;//接收完成标志
                    }
                    i=0;//缓存清0

                }
            }
						else
						i=0;
        }
	
	}

}
