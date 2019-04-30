unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
float angle[3];
void setup() {
  Serial.begin(9600);
}

void loop() {
  if (sign)
  {
    sign = 0;
    if (Re_buf[0] == 0x55)   //检查帧头
    {
      switch (Re_buf [1])
      {
        case 0x53:
          angle[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 180;
          angle[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 180;
          angle[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 180;
          Serial.print("angle:");
          Serial.print(angle[0]); Serial.print(" ");
          Serial.print(angle[1]); Serial.print(" ");
          Serial.print(angle[2]); Serial.println(" ");
          break;
      }
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    //char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code
    Re_buf[counter] = (unsigned char)Serial.read();
    if (counter == 0 && Re_buf[0] != 0x55) return; //第0号数据不是帧头
    counter++;
    if (counter == 11)          //接收到11个数据
    {
      counter = 0;             //重新赋值，准备下一帧数据的接收
      sign = 1;
    }
  }
}


