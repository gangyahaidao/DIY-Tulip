#include "RosDriverFinal.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"
#define UPDATE_PID_RATE 50
#define UPDATE_STATUS_RATE 50
#define UPDATE_IMU_RATE 50

#define MAX_SPEED 655.0  // mm/s

static uint32_t tTime[5];//定义时间数组，用于分时执行任务

float ax, ay, az;//加速度值
float gx, gy, gz;//角速度值
float angle[3];
float IMU[9];

UPLOAD_STATUS upload_status;

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

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);//设置PID

  testEncoders();//测试编码器
  
  //初始化电机PID
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  //初始化IMU??????????????????????
  
}

double leftTargetSpeed, rightTargetSpeed;
void loop() {
  receive_setPID();//接收设置PID命令
  
  //1.串口接收到一帧数据
  if (stringComplete) {
    //将接收到的速度命令赋值给电机，将上位机发送的速度为百分比
    memcpy(speed_data, speed_buffer, 10);
    if (speed_data[0] == 't') {
      if (speed_data[1] == 'F') { //右轮前进
        rightTargetSpeed = MAX_SPEED * speed_data[5];
      } else if (speed_data[1] == 'S') {
        rightTargetSpeed = 0;
      } else if (speed_data[1] == 'B') {
        rightTargetSpeed = -MAX_SPEED * speed_data[5];
      }
      //设置左轮目标速度
      if (speed_data[2] == 'F') { //左轮前进
        leftTargetSpeed = MAX_SPEED * speed_data[6];
      } else if (speed_data[2] == 'B') {
        leftTargetSpeed = -MAX_SPEED * speed_data[6];
      } else if (speed_data[2] == 'S') {
        leftTargetSpeed = 0;
      }
      //rightMotor.TargetTicksPerFrame = rightTargetSpeed;
      //leftMotor.TargetTicksPerFrame = leftTargetSpeed;
    }
    stringComplete = false;
  }
  //2.更新电机PID控制速度
  if ((millis() - tTime[0]) >= (1000 / UPDATE_PID_RATE)) {
    updatePIDSpeed();
    tTime[0] = millis();
  }
  if ((millis() - tTime[1]) >= (1000 / UPDATE_STATUS_RATE)) {
    //发送一帧数据到上位机，频率50hz
    int millisTime = millis() - tTime[1];
    //memset(upload_status, 0, sizeof(upload_status));
    upload_status.status = 1;
    upload_status.power = 12;
    upload_status.theta = IMU[8];//方向角0~360度
    upload_status.encoder_ppr = 1200;//??????????????
    
    counterRight = readEncoder(RIGHT); //???????????????????????增量正负问题
    upload_status.encoder_delta_r = abs(counterRight - preCounterRight);//右轮的编码器增量
    preCounterRight = counterRight;
    
    counterLeft = readEncoder(LEFT); 
    upload_status.encoder_delta_l = abs(counterLeft - preCounterLeft);//左轮的编码器增量
    preCounterLeft = counterLeft;
    
    upload_status.encoder_delta_car = (upload_status.encoder_delta_r + upload_status.encoder_delta_l) / 2;
    upload_status.omga_r = upload_status.encoder_delta_r * 1000 / millisTime; //右轮转速，个每秒
    upload_status.omga_l = upload_status.encoder_delta_l * 1000 / millisTime; //左轮转速，个每秒
    memcpy(&(upload_status.IMU[0]), &IMU[0], 9);//将IMU数据复制到结构体
    upload_status.time_stamp = (int)millis();
    send_status_server(&upload_status);//发送到上位机
    tTime[1] = millis();
  }
  //3.更新IMU信息
  if ((millis() - tTime[2]) >= (1000 / UPDATE_IMU_RATE)) {
    //???????????????????????????????????????????????????????????
    tTime[2] = millis();
  }
}

//发送数据到上位机
byte temp_buffer[4 + 4 * 19], send_buffer[99];
void send_status_server(UPLOAD_STATUS* data) {
  byte copy_index = 0;
  byte temp_copy_index = 0;
  temp_buffer[0] = FRAME_HEAD_1;//头部
  temp_buffer[1] = FRAME_HEAD_2;
  temp_buffer[2] = FRAME_HEAD_3;
  temp_buffer[3] = 95;//数据长度
  memcpy(&temp_buffer[4], (byte*)data, 4 * 19);
  //在结构体的每个数据中间插入空格0x20
  memcpy(&send_buffer[copy_index], &temp_buffer[temp_copy_index], 4);
  copy_index += 4;
  temp_copy_index += 4;
  for (int i = 1; i <= 19; i++) {
    memcpy(&send_buffer[copy_index], &temp_buffer[temp_copy_index], 4);
    copy_index += 4;
    temp_copy_index += 4;
    send_buffer[copy_index] = 0x20;//添加空格
    copy_index += 1;
  }
  Serial.write(send_buffer, 99);//发送到上位机
}
