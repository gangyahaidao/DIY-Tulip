#define TURN_DIR 4 // 进行正反转切换引脚
#define SPEED_1 2 // 速度反馈引脚，接外部中断0
#define TURN_PWM 5 // 4.6V~0.8V调速，<=0.4V停止转动，对应PWM值：230~40
#define STOP_LOW 6 // 低电平刹车
#define SPEED_2 3 // 速度反馈引脚，接外部中断1
#define NORMAL_SPEED 180 // 150
#define CIRCLE_RUN_COUNT 640 // 从最左边转到最右边中断计数值 最大540 取530
#define MIDDLE_RUN_COUNT 320 // 从最左边转动到中间位置的中断计数值
#define Sensor_pin 10

#define LEFT_LIMIT 8 // 左边限位开关引脚
#define RIGHT_LIMIT 9
int speed1_count = 0; // 中断计数
int ret = 0;
bool inInitState = false; // 是否在进行初始化
/**
中断绑定的函数
*/
int GlobalDestAngle = 0;
int TurnDir = 0; // 1右转是正转 2左转是反转 0不进行位置检测
void speed1_ISR() { // 速度反馈1中断函数，向右转计数增加，左转计数减少
  if (LOW == digitalRead(SPEED_2)) {
    speed1_count++;
  } else if(HIGH == digitalRead(SPEED_2)) { // 右转
    speed1_count--;
  }  
  if(TurnDir != 0) {
    if(TurnDir == 1) { // 向右转
      if(speed1_count >= GlobalDestAngle) {
        stopRun();
        if(inInitState) {
          inInitState = false;
          Serial.println("#S@"); // 发送电机校准完毕命令
        }
        TurnDir = 0;
      }
    } else if(TurnDir == 2) { // 向左转
      if(speed1_count <= GlobalDestAngle) {
        stopRun();
        TurnDir = 0;
      }
    }
  }
  if(speed1_count >= CIRCLE_RUN_COUNT) {
    stopRun();
    TurnDir = 0;
  }
}
/**
初始化中断，绑定中断函数
*/
void initInterrupt()
{
//  pinMode(SPEED_1, INPUT_PULLUP); // 有时候可以加有时候不需要加，需要根据不同硬件情况测试
//  pinMode(SPEED_2, INPUT_PULLUP);
  attachInterrupt(0, speed1_ISR, RISING); // pin 2，上升沿触发
  // attachInterrupt(1, speed2_ISR, RISING); // pin 3 此引脚产生的波形更好用此中断
}

/**
根据指定的PWM值启动
*/
void startRun(int speedValue) {
  analogWrite(TURN_PWM, speedValue);
  digitalWrite(STOP_LOW, HIGH);    
}
/**
停止运行，进入锁定状态
*/
void stopRun() {
  digitalWrite(STOP_LOW, LOW); // 启动刹车功能
  analogWrite(TURN_PWM, 0);
}

byte inChar;//用于缓存一帧的有用数据
boolean stringComplete = false;
byte recv_index = 0;
byte speed_buffer[3], speed_data[3];//前变量用于接收，后变量用于计算
void serialEvent() {//接收上位机发送的控制命令
  if (stringComplete == false && Serial.available()) {//数据必须要进行矫正，之后回复服务器端
    inChar = (unsigned char)Serial.read();
    // Serial.println((char)inChar);
    if(inChar == '#') // 一帧数据的开始字节
    {
      recv_index = 0;
    }else if(inChar == '@') // 接收到数据尾部
    {
      stringComplete = true; // 用于测试使用
      recv_index = 0;
    }else{ //将数据存储起来      
      if(recv_index >= 3){
        recv_index = 0;
      }else{
        speed_buffer[recv_index] = inChar;
        recv_index++;  
      }
    }
  }
}

/**
从当前位置运动到指定的比例
*/
void runToCustomerRatio(int destAngle) {
  if(speed1_count > destAngle) { // 当前位置在目标位置的右边，向左转
    TurnDir = 2; // 向左转
    GlobalDestAngle = destAngle; // 目标位置
    digitalWrite(TURN_DIR, LOW); // 向左转动
    startRun(NORMAL_SPEED); // 匀速开始转动       
  } else if(speed1_count < destAngle){ // 向右转
    TurnDir = 1; // 向右转
    GlobalDestAngle = destAngle; // 目标位置
    digitalWrite(TURN_DIR, HIGH); // 向右转动
    startRun(NORMAL_SPEED); // 匀速开始转动
  } 
}

long preMillisTime = 0;
int preSenseValue = 0;
void setup() {
  Serial.begin(115200);
  pinMode(Sensor_pin, INPUT); //设置人体红外接口状态
  pinMode(LEFT_LIMIT, INPUT_PULLUP); // 设置碰撞传感器引脚
  pinMode(RIGHT_LIMIT, INPUT_PULLUP);
  pinMode(TURN_DIR, OUTPUT);
  digitalWrite(TURN_DIR, LOW);  // 高电平向右转，低电平向左转
  pinMode(STOP_LOW, OUTPUT);
  digitalWrite(STOP_LOW, HIGH);  
  pinMode(TURN_PWM, OUTPUT);
  initInterrupt();
  stopRun(); // 初始处于锁定状态
  preMillisTime = millis();
}

void loop() {
  // 处理串口接收到的数据
  if(stringComplete) { // 如果收到一帧数据
    Serial.println("#R@"); // 通知上位机接收到了命令
    // Serial.println("#RECV@"); // 通知服务器已经收到控制命令
    memcpy(speed_data, speed_buffer, 3);
    speed_buffer[0] = '\0';
    speed_buffer[1] = '\0';
    speed_buffer[2] = '\0';
    int turnRatio = atoi(speed_data); // 将上位机发送的转动角度百分比转换成整数，上位机发送数据范围：1~99
    if(turnRatio == 0) { // 说明发送的数据是控制命令字符串
      TurnDir = 0;
      if(speed_data[0] == 'R') { // 进行复位操作，先转动到最左边，然后转动到中间位置
        digitalWrite(TURN_DIR, LOW); // 向左转动
        startRun(200); // 速度PWM = 200
        while(true) {
            ret = digitalRead(LEFT_LIMIT);
            if(ret == LOW) { // 如果转动到了最左边，停止转动
              stopRun();
              noInterrupts();
              delay(10);
              speed1_count = 0; // 将转动中断计数重置为0
              interrupts();
              break;
            }
        }
        digitalWrite(TURN_DIR, HIGH); // 开始向右转
        startRun(200); // 速度PWM = 200
        runToCustomerRatio(MIDDLE_RUN_COUNT); // 运动到中间位置停止
        inInitState = true;
      }
    } else { // 接收的是目标角度比例
      int destAngle = CIRCLE_RUN_COUNT*(turnRatio/100.0);
      if(turnRatio >= 1 && turnRatio <= 99) {
        stopRun();
        TurnDir = 0;
        runToCustomerRatio(destAngle);
      }      
    }    
    stringComplete = false;
  }
//
//  if((millis() - preMillisTime) >= 50){
//    int val = digitalRead(Sensor_pin); //定义参数存储人体红外传感器读到的状态
//    if(val != preSenseValue) {
//      preSenseValue = val;  
//      if (val == 1) //如果检测到有动物运动（在检测范围内），蜂鸣器发出警报
//      {
//        Serial.println("#Y@");      
//      } else if(val == 0) {
//        Serial.println("#N@");  
//      }
//    }
//    preMillisTime = millis();
//  }  
  delay(10);
}
