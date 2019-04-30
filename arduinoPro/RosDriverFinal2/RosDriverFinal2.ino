#include "RosDriverFinal.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "IMU.h"
#define UPDATE_PID_RATE 50
#define UPDATE_STATUS_RATE 50

#define SOUND 53
#define BUMP_FRONT_LEFT 49
#define BUMP_FRONT_RIGHT 51
#define BUMP_BACK_ONE 47
#define BUMP_WHEEL_LEFT 50
#define BUMP_WHEEL_RIGHT 52

#define AUTO_STOP_INTERVAL 1000
long lastMotorCommand = AUTO_STOP_INTERVAL;//长时间未收到速度消息则停止

int carStatus = 0;//小车状态，0表示未初始化，1表示正常，-1表示error

static uint32_t tTime[5];//定义时间数组，用于分时执行任务
UPLOAD_STATUS upload_status;//上传数据封装结构体

long counterLeft, counterRight;
long preCounterLeft, preCounterRight;//上一次编码器的值
extern SetPointInfo leftMotor, rightMotor;

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

extern float IMU[9];
void serialEvent2() {//接收IMU模块数据
  if (Serial2.available()) {
    uint8_t ch = (unsigned char)Serial2.read();
    Packet_Decode(ch);
  }
}

void resetSystem() {
  resetIMU();
  //重置编码器值
  resetEncoders();
  //重置PID控制结构体
  resePID();
  if(rightMotor.Encoder > 0){
    doSound(5);
  }
}

void doSoundOnce(int millisec){
  long pre_millis = millis();
  while(1){
    if((millis() - pre_millis) >= millisec){break;}
    digitalWrite(SOUND, LOW);
    delay(1);
    digitalWrite(SOUND, HIGH);  
    delay(1);
  }  
}
void doSound(int times){
  for(int i = 0; i < times; i++){
    doSoundOnce(150);
    delay(100);
  }  
}

void resetIMU(){
  char sendData[8] = {'A', 'T', '+', 'R', 'S', 'T', '\r', '\n'};//复位IMU Z轴
  Serial2.write(sendData, 8);  
}

void setup() {
  pinMode(SOUND, OUTPUT);    
  pinMode(BUMP_FRONT_LEFT, INPUT);
  pinMode(BUMP_FRONT_RIGHT, INPUT);
  pinMode(BUMP_BACK_ONE, INPUT);
  pinMode(BUMP_WHEEL_LEFT, INPUT_PULLUP);//使能上拉电阻
  pinMode(BUMP_WHEEL_RIGHT, INPUT_PULLUP);
  doSound(3);
  Serial.begin(115200);
  Serial2.begin(115200);//读取IMU模块消息 50hz  20hz->9600
  Serial3.begin(115200);//调试设置PID
  //testEncoders();//测试编码器
  //初始化电机PID
  resetEncoders();
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  resetIMU();
  carStatus = 1;
}

double leftTargetSpeed, rightTargetSpeed;
double escapeSpeed;//碰撞传感器遇到障碍物两轮后退速度
long frontLeftFlag = HIGH, frontRightFlag = HIGH, backOneFlag = HIGH, wheelLeftFlag = HIGH, wheelRightFlag= HIGH;
bool recvPersonalControlCmd = false;//收到上位机下发的自定义运动命令则为true
void loop() {
  //receive_setPID();//接收设置PID命令
  //1.串口接收到一帧数据  
  if (stringComplete) {
    //将接收到的速度命令赋值给电机，将上位机发送的速度为百分比
    lastMotorCommand = millis();
    memcpy(speed_data, speed_buffer, 10);
    if (speed_data[0] == 't') {
      rightTargetSpeed = MAX_SPEED * speed_data[5]/100.0;
      if (speed_data[1] == 'F') { //右轮前进
        rightMotor.dir = true;
      } else if (speed_data[1] == 'S') {
        rightTargetSpeed = 0;
        rightMotor.dir = true;
      } else if (speed_data[1] == 'B') {
        rightMotor.dir = false;
      }
      //设置左轮目标速度
      leftTargetSpeed = MAX_SPEED * speed_data[6]/100.0;
      if (speed_data[2] == 'F') { //左轮前进
        leftMotor.dir = true;
      } else if (speed_data[2] == 'B') {
        leftMotor.dir = false;
      } else if (speed_data[2] == 'S') {
        leftTargetSpeed = 0;
        leftMotor.dir = true;
      }
      rightMotor.TargetTicksPerFrame = rightTargetSpeed;
      leftMotor.TargetTicksPerFrame = leftTargetSpeed;
    } else if (speed_data[0] == 'I') { //上位机发送的复位命令
      carStatus = 0;//设置小车正在初始化
      resetSystem();      
      doSound(2);
      carStatus = 1;//初始化完毕
    } else if (speed_data[0] == 'C') { //IMU标定命令
      resetIMU();
    }else if(speed_data[0] == 'T'){//机器人前方有障碍物，需要后退，后退速度百分比为speed_data[1]指定
      recvPersonalControlCmd = true;//收到自主运动命令，暂时关闭碰撞传感器，移动以小段距离再开启
      escapeSpeed = MAX_SPEED * speed_data[1] / 100;
      leftMotor.dir = false;//两轮后退
      rightMotor.dir = false;
      rightMotor.TargetTicksPerFrame = escapeSpeed;
      leftMotor.TargetTicksPerFrame = escapeSpeed;
    }else if(speed_data[0] == 'L'){//机器人后方遇到障碍物，需要前进一段距离
      recvPersonalControlCmd = true;
      escapeSpeed = MAX_SPEED * speed_data[1] / 100;
      leftMotor.dir = true;//两轮前进
      rightMotor.dir = true;
      rightMotor.TargetTicksPerFrame = escapeSpeed;
      leftMotor.TargetTicksPerFrame = escapeSpeed;
    }else if(speed_data[0] == 'Y'){
      recvPersonalControlCmd = false;//使能碰撞传感器
    }else if(speed_data[0] == 'P'){//收到紧急停止命令
      rightMotor.TargetTicksPerFrame = 0;
      leftMotor.TargetTicksPerFrame = 0;
      setMotorSpeeds(0, 0);
    }else if(speed_data[0] == 'R'){//收到原地旋转命令
      if(speed_data[1] == 'l'){//向左旋转
        leftMotor.dir = false;
        rightMotor.dir = true;
        leftMotor.TargetTicksPerFrame = MAX_SPEED*speed_data[2]/100.0;
        rightMotor.TargetTicksPerFrame = MAX_SPEED*speed_data[2]/100.0;
      }else if(speed_data[1] == 'r'){//向右旋转
        leftMotor.dir = true;
        rightMotor.dir = false;
        leftMotor.TargetTicksPerFrame = MAX_SPEED*speed_data[2]/100.0;
        rightMotor.TargetTicksPerFrame = MAX_SPEED*speed_data[2]/100.0;
      }
    }
    stringComplete = false;
  }
  //2.更新电机PID控制速度
  if ((millis() - tTime[0]) >= (1000 / UPDATE_PID_RATE)) {
    leftMotor.Encoder = readEncoder(LEFT);
    rightMotor.Encoder = readEncoder(RIGHT);
    updatePIDSpeed();
    tTime[0] = millis();
  }
  //3.发送一帧数据到上位机，频率50hz
  if ((millis() - tTime[1]) >= (1000 / UPDATE_STATUS_RATE)) {
    int millisTime = millis() - tTime[1];
    //memset(upload_status, 0, sizeof(upload_status));
    upload_status.status = carStatus;
    upload_status.power = 12;
    upload_status.theta = IMU[8];//方向角0~360度
    upload_status.encoder_ppr = 2400;

    counterRight = readEncoder(RIGHT); //上位机用于计算CarPos2D位移，有正负
    upload_status.encoder_delta_r = counterRight - preCounterRight;//右轮的编码器增量
    preCounterRight = counterRight;

    counterLeft = readEncoder(LEFT);
    upload_status.encoder_delta_l = counterLeft - preCounterLeft;//左轮的编码器增量
    preCounterLeft = counterLeft;

    upload_status.encoder_delta_car = (upload_status.encoder_delta_r + upload_status.encoder_delta_l) / 2;
    upload_status.omga_r = upload_status.encoder_delta_r * 1000 / millisTime; //右轮转速，个每秒
    upload_status.omga_l = upload_status.encoder_delta_l * 1000 / millisTime; //左轮转速，个每秒
    upload_status.bump_front_left_flag = frontLeftFlag;
    upload_status.bump_front_right_flag = frontRightFlag;
    upload_status.bump_back_one_flag = backOneFlag;
    upload_status.bump_wheel_left_flag = wheelLeftFlag;
    upload_status.bump_wheel_right_flag = wheelRightFlag;
    memcpy((byte*) & (upload_status.IMU[0]), (byte*)&IMU[0], 4 * 9); //将IMU数据复制到结构体
    upload_status.time_stamp = (int)millis();
    send_status_server(&upload_status);//发送到上位机
    tTime[1] = millis();
  }
  //4.长时间未收到速度命令则停止
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    rightMotor.TargetTicksPerFrame = 0;
    leftMotor.TargetTicksPerFrame = 0;
    setMotorSpeeds(0, 0);
  }
  //5.检测碰撞传感器
  frontLeftFlag = digitalRead(BUMP_FRONT_LEFT);  
  frontRightFlag = digitalRead(BUMP_FRONT_RIGHT);
  backOneFlag = digitalRead(BUMP_BACK_ONE);
  wheelLeftFlag = digitalRead(BUMP_WHEEL_LEFT);
  wheelRightFlag = digitalRead(BUMP_WHEEL_RIGHT);
  if((frontLeftFlag == LOW || frontRightFlag == LOW || backOneFlag == LOW || wheelLeftFlag == LOW || wheelRightFlag == LOW) && recvPersonalControlCmd == false){//任何一个触发都会停止    
    rightMotor.TargetTicksPerFrame = 0;
    leftMotor.TargetTicksPerFrame = 0;
    setMotorSpeeds(0, 0);
    //doSound(1);
  }
}

//发送数据到上位机
#define DATA_COUNT 24  //结构体数据成员的个数
byte temp_buffer[4 + 4 * DATA_COUNT], send_buffer[4 + 4*DATA_COUNT + DATA_COUNT];
void send_status_server(UPLOAD_STATUS* data) {
  byte copy_index = 0;
  byte temp_copy_index = 0;
  temp_buffer[0] = FRAME_HEAD_1;//头部
  temp_buffer[1] = FRAME_HEAD_2;
  temp_buffer[2] = FRAME_HEAD_3;
  temp_buffer[3] = 120;//数据长度  4*DATA_COUNT+DATA_COUNT
  memcpy(&temp_buffer[4], (byte*)data, 4*DATA_COUNT);
  //在结构体的每个数据中间插入空格0x20
  memcpy(&send_buffer[copy_index], &temp_buffer[temp_copy_index], 4);
  copy_index += 4;
  temp_copy_index += 4;
  for (int i = 1; i <= DATA_COUNT; i++) {
    memcpy(&send_buffer[copy_index], &temp_buffer[temp_copy_index], 4);
    copy_index += 4;
    temp_copy_index += 4;
    send_buffer[copy_index] = 0x20;//添加空格
    copy_index += 1;
  }
  
  Serial.write(send_buffer, 4 + 4*DATA_COUNT + DATA_COUNT);//发送到上位机
}
