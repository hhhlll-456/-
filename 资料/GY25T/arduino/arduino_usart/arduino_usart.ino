/////////////////////
/*
GY25t----MINI
VCC----VCC
GND----GND
1:GY25t_TX---10
2:GY2_RX---11
*/
//////////////////
#include <SoftwareSerial.h>
typedef struct
{
    int16_t temp;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} gy;
SoftwareSerial mySerial(10, 11); // RX, TX
byte add=0xa4;
byte len=0,start_reg=0;
unsigned char Re_buf[30],counter=0;
unsigned char sign=0;
gy my_25t;
void setup() {
   Serial.begin(115200);
   mySerial.begin(9600);
   mySerial.listen();      
} 

void loop() {
  unsigned char i=0,sum=0;
  while (mySerial.available()) {   
    Re_buf[counter]=(unsigned char)mySerial.read();
    switch(counter)
    {
      case 0:
      if(Re_buf[0]!=add)
      return;
      break;
      case 1:
    
      if(Re_buf[1]!=0x03)
        { 
          counter=0;
          return;
      }
      break;
      case 2:
        if(Re_buf[2]<0x1d)
         start_reg=Re_buf[2];
         else
            { 
          counter=0;
          return;
         }
      break;
      case 3:
         if((start_reg+Re_buf[3])<0x1d)
            len=Re_buf[3];
            else
            { 
              counter=0;
             return;
         }
            
      break;
      default:
         if(len+5==counter)
         {  
            sign=1;
         }
      break;
    }
    
  if(sign)
  {   
     sign=0;
     for(i=0;i<counter-1;i++)
      sum+=Re_buf[i]; 
       counter=0;
     if(sum==Re_buf[i] )        //检查帧头，帧尾
     {  	       
        if(start_reg==0x14)
        {
                my_25t.roll=(Re_buf[4]<<8)|Re_buf[5];
                my_25t.pitch=(Re_buf[6]<<8)|Re_buf[7];
                my_25t.yaw=(Re_buf[8]<<8)|Re_buf[9];
                my_25t.temp=(Re_buf[10]<<8)|Re_buf[12];
        }
      	 Serial.print("roll:");
         Serial.print((float)my_25t.roll/100);
	 Serial.print(",pitch:");
         Serial.print((float)my_25t.pitch/100);
	 Serial.print(",yaw:");
         Serial.print((float)my_25t.yaw/100);
	 Serial.print(" ,temp:");  
         Serial.println((float)my_25t.temp/100);
           
     }
//     else
//        {
//          Serial.print(" sum ");
//          Serial.println(sum);
//      } 
   }
   else
   counter++; 
//   Serial.print("cont:");
//   Serial.println(counter);
  }
   
}

