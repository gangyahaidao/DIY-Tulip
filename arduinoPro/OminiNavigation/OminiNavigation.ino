#define FR_1 22//正反转控制引脚  3口
#define FR_2 23
#define FR_3 24
#define PWM_1 2//速度控制引脚  4.6~0.8v  速度低到高   5口
#define PWM_2 3
#define PWM_3 4
#define STOP_1 26//紧急刹车，低电平有效  6口
#define STOP_2 27
#define STOP_3 28

#define AUTO_STOP_INTERVAL 1000
long lastMotorCommand = AUTO_STOP_INTERVAL;//长时间未收到速度消息则停止

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
      if (match[0] == 0xcd) {
        match_flag = 1;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 1) {//接收第二个消息头
      match[1] = (unsigned char)Serial.read();
      if (match[1] == 0xeb) {
        match_flag = 2;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 2) {//接收第三个消息头
      match[2] = (unsigned char)Serial.read();
      if (match[2] == 0xd7) {
        match_flag = 3;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 3) {//接收第四个字节,有用数据长度
      serial_cnt = (unsigned char)Serial.read();
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
  // put your setup code here, to run once:
  pinMode(FR_1, OUTPUT);pinMode(FR_2, OUTPUT);pinMode(FR_3, OUTPUT);
  pinMode(PWM_1, OUTPUT);pinMode(PWM_2, OUTPUT);pinMode(PWM_3, OUTPUT);
  pinMode(STOP_1, OUTPUT);pinMode(STOP_2, OUTPUT);pinMode(STOP_3, OUTPUT);
  startAllMotors();  
}

#define MOVE_PWM 150  //2.0v
#define TURN_PWM 200  //3.5v
void loop() {
  if (stringComplete) {
    lastMotorCommand = millis();
    memcpy(speed_data, speed_buffer, 10);
    if (speed_data[0] == '1') {//表示视觉识别正常      
      if(speed_data[1] == 'L'){//向右转
        setPWMs(TURN_PWM, TURN_PWM, TURN_PWM);
      }else if(speed_data[1] == 'R'){//向左转
        setPWMs(-TURN_PWM, -TURN_PWM, -TURN_PWM);
      }else if(speed_data[1] == 'M'){//直行
        setPWMs(MOVE_PWM, 0, -MOVE_PWM);
      }
      startAllMotors();
    }else if (speed_data[0] == '0'){
      stopAllMotors();  
    }
  }
  //长时间未收到速度命令则停止
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    stopAllMotors();
  }
}

void setPWMs(int pwm1, int pwm2, int pwm3){//设置PWM
  if(pwm1 > 0){
    digitalWrite(FR_1, LOW);    
  }else if(pwm1 == 0){
    digitalWrite(STOP_1, HIGH);  
  }else{
    pwm1 = -pwm1;  
    digitalWrite(FR_1, HIGH);
  }
  //pwm2
  if(pwm2 > 0){
    digitalWrite(FR_2, LOW);    
  }else if(pwm2 == 0){
    digitalWrite(STOP_2, HIGH);  
  }else{
    pwm2 = -pwm2;  
    digitalWrite(FR_2, HIGH);
  }
  //pwm3
  if(pwm3 > 0){
    digitalWrite(FR_3, LOW);    
  }else if(pwm3 == 0){
    digitalWrite(STOP_3, HIGH);  
  }else{
    pwm3 = -pwm3;  
    digitalWrite(FR_3, HIGH);
  }
  analogWrite(PWM_1, pwm1);analogWrite(PWM_2, pwm2); analogWrite(PWM_3, pwm3);
}
void startAllMotors(){
  digitalWrite(STOP_1, HIGH);digitalWrite(STOP_2, HIGH);digitalWrite(STOP_3, HIGH);
}
void stopAllMotors(){
  digitalWrite(STOP_1, LOW);digitalWrite(STOP_2, LOW);digitalWrite(STOP_3, LOW);
}
