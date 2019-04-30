int floor1[] = {22, 23, 24, 25, 26, 27};//第一层的引脚ENA IN1 IN2
int floor2[] = {30, 31, 32, 33, 34, 35};
int floor3[] = {38, 39, 40, 41, 42, 43};
int posSwitch[] = {2, 3, 4, 5, 6, 7};

void initPinMode(){
  for(int i = 0; i < 6; i++){
    pinMode(floor1[i], OUTPUT);
    pinMode(floor2[i], OUTPUT);
    pinMode(floor3[i], OUTPUT);
    pinMode(posSwitch[i], INPUT_PULLUP);
  } 
}
void startRun(int enPin, int in1Pin, int in2Pin, char direc){
  if(direc == '1'){//正传
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    digitalWrite(enPin, HIGH);
  }else if(direc == '2'){//反转
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    digitalWrite(enPin, HIGH);
  }else if(direc == '0'){//停止转动
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(enPin, LOW);
  }
}
/**
floorNum: 指定哪一层 1 2 3
motorNum: 'F' 'B' 前面的电机 后面的电机
direc: 电机转动方向 1正传 2反转 0停止
*/
void controlMotor(char floorNum, char motorNum, char direc){
  if(floorNum == '1'){//第一层电机
    if(motorNum == 'F'){//第一层前面的电机
      startRun(floor1[0], floor1[1], floor1[2], direc);
    }else if(motorNum == 'B'){//第一层后面的电机
      startRun(floor1[3], floor1[4], floor1[5], direc);
    } 
  }else if(floorNum == '2'){//第二层电机
    if(motorNum == 'F'){//第一层前面的电机
      startRun(floor2[0], floor2[1], floor2[2], direc);
    }else if(motorNum == 'B'){//第一层后面的电机
      startRun(floor2[3], floor2[4], floor2[5], direc);
    } 
  }else if(floorNum == '3'){//第三层电机
    if(motorNum == 'F'){//第一层前面的电机
      startRun(floor3[0], floor3[1], floor3[2], direc);
    }else if(motorNum == 'B'){//第一层后面的电机
      startRun(floor3[3], floor3[4], floor3[5], direc);
    } 
  }
}

int match_flag = 0;
byte inChar;//用于缓存一帧的有用数据
byte match[4];//存储三个字节头部
boolean stringComplete = false;
byte serial_cnt; //global
byte recv_index = 0;
byte speed_buffer[10], speed_data[10];//前变量用于接收，后变量用于计算
void serialEvent() {//接收上位机发送的控制命令
  if (Serial.available()) {
    if (match_flag == 0) {//接收第一个消息头
      match[0] = (unsigned char)Serial.read();
      if (match[0] == 'X') {
        match_flag = 1;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 1) {//接收第二个消息头
      match[1] = (unsigned char)Serial.read();
      if (match[1] == 'Y') {
        match_flag = 2;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 2) {//接收第三个消息头
      match[2] = (unsigned char)Serial.read();
      if (match[2] == 'Z') {
        match_flag = 3;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 3) {//接收第四个字节,有用数据长度
      serial_cnt = (unsigned char)Serial.read();
      serial_cnt = atoi(serial_cnt);//将整数字符转换成整数
      match_flag = 4;
      return;
    }
    if (match_flag == 4) {
      inChar = (unsigned char)Serial.read();//接收帧数据有用字节
      speed_buffer[recv_index] = inChar;
      recv_index++;//接收的数据长度
      serial_cnt--;
      if (serial_cnt <= 0) {
        stringComplete = true;
        recv_index = 0;
        serial_cnt = 0;
        match_flag = 0;
        return;
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  initPinMode();
}

int getSwitchPinNum(char floorNum, char motorNum){
  int index = 0;
  int whichFloor = 0;
  if(floorNum == '1'){
    if(motorNum == 'F'){
      index = 2;  
    }else if(motorNum == 'B'){
      index = 3;  
    }
  }else if(floorNum == '2'){
    if(motorNum == 'F'){
      index = 4;  
    }else if(motorNum == 'B'){
      index = 5;  
    }
  }else if(floorNum == '3'){
    if(motorNum == 'F'){
      index = 6;  
    }else if(motorNum == 'B'){
      index = 7;  
    }
  }
  return index;
}

int switchPinNum = 0;//限位开关的引脚编号
char saveSerialData[6][3] = {0};
void loop() {
  if (stringComplete) {
    memcpy(speed_data, speed_buffer, 10);
    controlMotor(speed_data[0], speed_data[1], speed_data[2]);// '1'哪一层  'F'前后哪个电机 '1'方向
    switchPinNum = getSwitchPinNum(speed_data[0], speed_data[1]);
    saveSerialData[switchPinNum-2][0] = speed_data[0];//保存当前触发的电机信息
    saveSerialData[switchPinNum-2][1] = speed_data[1];
    saveSerialData[switchPinNum-2][2] = speed_data[2];
  }
  if(digitalRead(switchPinNum) == HIGH){//限位开关没按下去是高电平，按下是低电平说明需要继续转动
    controlMotor(saveSerialData[switchPinNum-2][0], saveSerialData[switchPinNum-2][1], '0');//停止转动
  }
  if(switchPinNum >= 2){
     Serial.println(digitalRead(switchPinNum));
     delay(50);   
  }
}
