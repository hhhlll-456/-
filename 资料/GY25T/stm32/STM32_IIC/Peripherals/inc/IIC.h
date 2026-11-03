//http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.9qa3Ky&v=1
#ifndef _IIC_H
#define _IIC_H
#include "stm32f10x.h"
#define SCL_H()         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L()         GPIOB->BRR  = GPIO_Pin_6
   
#define SDA_H()         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L()         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read()      GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)
#define SDA_read()      GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

u8 Single_WriteI2C_byte(u8 Slave_Address,u8 REG_Address,u8 data);
u8 Single_ReadI2C(u8 Slave_Address,u8 REG_Address,u8 *REG_data,u8 length);



#endif
