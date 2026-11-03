//http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.9qa3Ky&v=1
#include "LED.h"
 
void LED_Int(GPIO_TypeDef* GPIOx,uint16_t GPIO_PinX,uint32_t time)
{
	GPIO_InitTypeDef GPIO_X; 
	RCC_APB2PeriphClockCmd(time,ENABLE);
	GPIO_X.GPIO_Pin = GPIO_PinX;//LED
	GPIO_X.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_X.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_X);
	GPIO_SetBits(GPIOx,GPIO_PinX);
}
void input_Int(GPIO_TypeDef* GPIOx,uint16_t GPIO_PinX,uint32_t time)
{
	GPIO_InitTypeDef GPIO_X; 
	RCC_APB2PeriphClockCmd(time,ENABLE);
	GPIO_X.GPIO_Pin = GPIO_PinX;//LED
	GPIO_X.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOx, &GPIO_X);
	
}