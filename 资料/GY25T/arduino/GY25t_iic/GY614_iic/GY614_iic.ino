#include <i2cmaster.h>

/////////////////////////////////
/*
GY-25t-----mini
VCC----VCC
SCL----A5
SDA----A4
GND--GND
*/
/////////////////////////////////

//////////////////////////////////
#define uint16_t unsigned int
#define iic_add  0xa4
typedef struct
{
    float temp;
    float roll;
    float pitch;
    float yaw;
} gy;
unsigned char Re_buf;
unsigned char sign=0;
gy my_25t;
uint16_t delay_t=0;
byte color=0,rgb_data[3]={0};
byte ready_Ok=0;
void setup() {
      byte td=1;
       Serial.begin(115200);
       i2c_init();
       delay(100); 
       iic_read(0x02,&td,1);
       switch(td)
       {
         case 0:delay_t=100;break;
         case 1:delay_t=20;break;
         case 2:delay_t=10;break;
         case 3:delay_t=5;break;
       }
        delay(delay_t); 
        attachInterrupt(0, Exti, RISING);
}
void loop() {
  unsigned char data[16]={0};

 
     
      if(ready_Ok)
      {
   iic_read(0x14,data,8);
   my_25t.roll=(data[0]<<8)|data[1];
   my_25t.pitch=(data[2]<<8)|data[3];
   my_25t.yaw=(data[4]<<8)|data[5];
   my_25t.temp=(data[6]<<8)|data[7];
    Serial.print("roll: ");
   Serial.print(my_25t.roll/100);
   Serial.print(",pitch: ");
   Serial.print( my_25t.pitch/100);
    Serial.print(",yaw:");
    Serial.print( my_25t.yaw/100);
     Serial.print(",temp:");
    Serial.println(my_25t.temp/100);
    ready_Ok=0;
     }
 
   delay(delay_t); 
}
void Exti()
{
 if(!ready_Ok) 
  ready_Ok=1;//数据更新标志
}
void iic_read(unsigned char add,unsigned char *data,unsigned char len)
{
  i2c_start_wait(iic_add);
   i2c_write(add);
   i2c_start_wait(iic_add+1);
   while(len-1)
   {
    *data++=i2c_readAck();
    len--;
    }
    *data=i2c_readNak();
    i2c_stop();
}
void iic_write( char add,unsigned char data)
{
  i2c_start_wait(iic_add);
   i2c_write(add);
   i2c_write(data);
    i2c_stop();
}

