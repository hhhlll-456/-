#include <reg52.h>
#include "usart.h"
#include "iic.h"  
 #include "stdio.h"
/*
硬件接法：
GY-25t----C51
SCL---P3^6
SDA---P3^7
C51---FT232
TX ---RX
RX ---TX
软件说明：

注：
	中断函数位于stc_it.c
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
void delay_ms(unsigned int x)
{
	while(x--);
}

int main(void)
{
	u8  raw_data[9]={0};
	uint16_t datas[4]={0},delay_t;

	gy my_25t;

	u8 td=0;

	Usart_Int(9600);
	delay_ms(300);//等待模块初始化完成
	td=2;
	 Single_WriteI2C_byte(0xa4,0x02,2);//设置更新频率5hz
       switch(td)
       {
         case 0:delay_t=1010;break;
         case 1:delay_t=510;break;
         case 2:delay_t=210;break;
         case 3:delay_t=110;break;
       }
      
	while(1)
	{
		
		 if(input==0)
			{
				while(input==0);
				if(Single_ReadI2C(0xa4,0x14,raw_data,8))
				{
					my_25t.roll=((int16_t)raw_data[0]<<8)|raw_data[1];
					my_25t.pitch=((int16_t)raw_data[2]<<8)|raw_data[3];
					my_25t.yaw=((int16_t)raw_data[4]<<8)|raw_data[5];
					my_25t.temp=((int16_t)raw_data[6]<<8)|raw_data[7];
					printf("roll:%.2f,",(float)my_25t.roll/100.0f);
					printf("pitch:%.2f,",(float)my_25t.pitch/100.0f);
					printf("yaw:%.2f,",(float)my_25t.yaw/100);
					printf("temp:%.2f\r\n ",(float)my_25t.temp/100.0f);  
				}
			}
	
		
		
	}
}
