#define LED_PIN 13
uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0,   0,0, 0,0, 0,0,   0,0, 0,0, 0,0,   0,0, 0x00, 0x00, '\r', '\n' };
static unsigned char ucRxBuffer[250];
static unsigned char ucRxCnt = 0; 
bool blinkState = false;
void setup() 
{
  Serial.begin(115200);  
  Serial1.begin(115200);
  pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
  while (Serial1.available()){    
    ucRxBuffer[ucRxCnt++]=Serial1.read();
    if (ucRxBuffer[0]!=0x55) 
    {
      ucRxCnt=0;
      return;
    }
    if (ucRxCnt<11) {return;}
    else
    {
      switch(ucRxBuffer[1])
      {
        case 0x53:  //角度输出
          /*teapotPacket[2] = ucRxBuffer[3];teapotPacket[3] = ucRxBuffer[2];
          teapotPacket[4] = ucRxBuffer[5];teapotPacket[5] = ucRxBuffer[4];
          teapotPacket[6] = ucRxBuffer[7];teapotPacket[7] = ucRxBuffer[6];
          teapotPacket[8] = ucRxBuffer[9];teapotPacket[9] = ucRxBuffer[8];*/
          teapotPacket[2] = ucRxBuffer[3];teapotPacket[3] = ucRxBuffer[2];
          teapotPacket[4] = ucRxBuffer[5];teapotPacket[5] = ucRxBuffer[4];
          teapotPacket[6] = ucRxBuffer[7];teapotPacket[7] = ucRxBuffer[6];
          teapotPacket[22] = ucRxBuffer[9];teapotPacket[23] = ucRxBuffer[8];//温度
          break;
        case 0x52:  //角速度
          teapotPacket[10] = ucRxBuffer[3];teapotPacket[11] = ucRxBuffer[2];
          teapotPacket[12] = ucRxBuffer[5];teapotPacket[13] = ucRxBuffer[4];
          teapotPacket[14] = ucRxBuffer[7];teapotPacket[15] = ucRxBuffer[6];
          teapotPacket[22] = ucRxBuffer[9];teapotPacket[23] = ucRxBuffer[8];
          break;       
        case 0x51:  //加速度
          teapotPacket[16] = ucRxBuffer[3];teapotPacket[17] = ucRxBuffer[2];
          teapotPacket[18] = ucRxBuffer[5];teapotPacket[19] = ucRxBuffer[4];
          teapotPacket[20] = ucRxBuffer[7];teapotPacket[21] = ucRxBuffer[6];
          teapotPacket[22] = ucRxBuffer[9];teapotPacket[23] = ucRxBuffer[8];
          break;
      }
      //发送数据到上位机
      Serial.write(teapotPacket, 28);
      teapotPacket[25]++;//发送的包计数
      ucRxCnt=0;
      blinkState = !blinkState;//闪灯
      digitalWrite(LED_PIN, blinkState);
    }
  }  
}



