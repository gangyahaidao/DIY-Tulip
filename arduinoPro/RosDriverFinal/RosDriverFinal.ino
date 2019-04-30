#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include "RosDriverFinal.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"

#define ROTATE_SPEED 3.0   //(r/min)/tick
#define UPDATE_PID_RATE 30   //hz
#define UPDATE_BATTERY_RATE 5
#define UPDATE_ODOM_RATE 30 
#define UPDATE_IMU_RATE 50 

static uint32_t tTime[5];//定义时间数组，用于分时执行任务

int aix, aiy, aiz;// 加速度计原始数据
int gix, giy, giz;// 陀螺仪原始数据
float ax, ay, az;//加速度值
float gx, gy, gz;//角速度值
float angle[3], q0, q1, q2, q3;
Madgwick filter;

SRobotOdomData odomData;
SRobotSpeedData speedData;
SRobotBatteryData batteryData;
SRobotAcceData acceData;
SRobotGyroData gyroData;
SRobotYPRData yprData;
RRobotData speed_data;//接收速度缓存
RRobotData speed_motor;//用于给电机赋值

extern Wheel wheelL, wheelR;//编码器参数结构体
float ang[2] = {0};   //由编码器tick数计算得到速度--> r/min
extern SetPointInfo leftMotor, rightMotor;//PID参数结构体

int match_flag = 0;
byte inChar;
byte match[2];
boolean stringComplete = false;
int serial_cnt = 0; //global
byte speed_buffer[20], speed_frame[20];//前变量用于接收，后变量用于计算
void serialEvent() {//接收上位机发送的控制命令
  if (Serial.available()) {
    if (match_flag == 0) {
      match[0] = (unsigned char)Serial.read();
      if (match[0] == 0x55) {
        match_flag = 1;
        speed_buffer[0] = 0x55;
        serial_cnt ++;
        return;
      }
      else {
        match_flag = 0;
        serial_cnt = 0;
        return;
      }
    }
    if (match_flag == 1) {
      match[1] = (unsigned char)Serial.read();
      if (match[1] == 0xAA) {
        match_flag = 2;
        speed_buffer[1] = 0xAA;
        serial_cnt ++;
        return;
      }
      else {
        match_flag = 0;
        serial_cnt = 0;
        return;
      }
    }
    if (match_flag == 2) {
      inChar = (unsigned char)Serial.read();
      speed_buffer[serial_cnt] = inChar;
      serial_cnt ++;
      if (serial_cnt >= 20) {
        stringComplete = true;
        serial_cnt = 0;
        match_flag = 0;
        watch_dog = millis();
        return;
      }
    }
  }
}
float velocity_calculate(Wheel *omni) {//30ms计算一次
  //calculating the speed of wheel, ang[X]:r/min
  float ang = float(omni->count * ROTATE_SPEED);//tick*(r/min/tick)
  if (omni->dir == 1) {
    ang = -ang;
  }
  omni->count = 0;
  return ang;
}

#define SAMPLE_LEN 60
int get_current() {//获取放电电流
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLE_LEN; i ++) {
    sum += analogRead(A0);
    delayMicroseconds(30);
  }
  return (2490 - map(sum / SAMPLE_LEN, 0, 4096, 0, 3300)) / 0.185;
}
int get_voltage() {//获取当前电压
  int sensor_value;
  sensor_value = map(analogRead(A1), 0, 4096, 0, 3300);
  if (sensor_value ) {
    // 12.6v  R1 4.02k , R2 12.1k
    return map(sensor_value - 2740, 0, 560, 1100, 1320);
  }
}

void pack_message_and_send(void* data, msg_type type, byte sub_type = 0) {//将指定数据类型的数据包发送到上位机
  byte a = 0;
  byte buffer[20];
  buffer[0] = FRAME_HEAD_1;
  buffer[1] = FRAME_HEAD_2;
  buffer[2] = sub_type;
  buffer[3] = type;
  memcpy(&buffer[4], data, 12);
  for (int i = 3; i < 16; i++) {//计算校验码
    a += buffer[i];
  }
  buffer[16] = a;
  a = 0;
  buffer[17] = 0;
  buffer[18] = 0;
  buffer[19] = FRAME_END;
  Serial.write(buffer, 20);//发送
}

void setup() {
  Serial.begin(115200);
  //初始化电机PID和编码器
  initEncoders();//初始化编码器中断库对象
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器

  CurieIMU.begin();
  CurieIMU.setAccelerometerRate(200);
  CurieIMU.setAccelerometerRange(8);//参照opencr设置
  CurieIMU.setGyroRate(200);
  CurieIMU.setGyroRange(2000);//(+/-2000°/s)
  filter.begin(200);

  CurieIMU.autoCalibrateGyroOffset();//陀螺仪自动校准
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);//加速度自动计校准
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
}

byte aa = 0;//用于计算校验码
void loop() {
  //1.串口接收到一帧数据
  if (stringComplete) {
    aa = 0;
    for (k = 0; k < 20; k++) {
      speed_frame[k] = speed_buffer[k];
    }
    if (speed_frame[3] == 0x05 && speed_frame[19] == 0x0A) {
      //Software_Reset();
    }else if (speed_frame[3] == MSG_MOVECTRL && speed_frame[19] == 0x0A) {//接收到电机控制命令
      for (int i = 3; i < 16; i++) {
        aa += speed_frame[i];
      }
      if (aa == speed_frame[16]) {//校验码计算正确
        memcpy(&(speed_data.v_motor1), &speed_frame[4], 4);
        memcpy(&(speed_data.v_motor2), &speed_frame[8], 4);
        //memcpy(&(speed_data.v_motor3), &speed_frame[12], 4);
        aa = 0;
        leftMotor.TargetTicksPerFrame = speed_data.v_motor1*1000;//需要修改上位机程序发送速度为：m/s，转换成mm/s
        rightMotor.TargetTicksPerFrame = speed_data.v_motor2*1000;
        //speed_motor.v_motor3 = speed_data.v_motor3;
      }
      else {
        //debug_println("check_sum error!");
        aa = 0;
      }
    }
    stringComplete = false;
  }
  //2.更新电机PID控制值
  if ((millis() - tTime[0]) >= (1000 / UPDATE_PID_RATE)) {
    updatePIDSpeed();
    ang[0] = velocity_calculate(&wheelL);
    ang[1] = velocity_calculate(&wheelR);
    tTime[0] = millis();
  }
  //3.更新电池状态
  if ((millis() - tTime[1]) >= (1000 / UPDATE_BATTERY_RATE)) {
    batteryData.voltage = get_voltage();
    batteryData.current = get_current();
    batteryData.charge = 0;
    pack_message_and_send(&batteryData, MSG_BATTERY);
    tTime[1] = millis();
  }
  //4.更新odom编码器和当前速度信息
   if ((millis() - tTime[2]) >= (1000 / UPDATE_ODOM_RATE)) {
    odomData.odom_motor1 = wheelL.counter;
    odomData.odom_motor2 = wheelR.counter;
    pack_message_and_send(&odomData, MSG_ODOM);//发送编码器值
    
    speedData.v_motor1 = ang[0];
    speedData.v_motor2 = ang[1];
    pack_message_and_send(&speedData, MSG_SPEED);//发送当前线速度：r/min
    tTime[2] = millis();
  }
  //5.更新并发布IMU信息
  if ((millis() - tTime[3]) >= (1000 / UPDATE_IMU_RATE)) {
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    ax = (aix * 8.0) / 32768.0;
    ay = (aiy * 8.0) / 32768.0;
    az = (aiz * 8.0) / 32768.0;
    gx = (gix * 2000.0) / 32768.0;
    gy = (giy * 2000.0) / 32768.0;
    gz = (giz * 2000.0) / 32768.0;
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    //计算欧拉角
    angle[0] = filter.getRoll();
    angle[1] = filter.getPitch();
    angle[2] = filter.getYaw();

    gyroData.gyx = gx;
    gyroData.gyy = gy;
    gyroData.gyz = gz;
    pack_message_and_send(&gyroData, MSG_ACCE, 1);//发送角速度信息

    yprData.yaw = angle[2];
    yprData.pitch = angle[1];
    yprData.roll = angle[0];
    pack_message_and_send(&yprData, MSG_ACCE, 2);//发送角度信息
    tTime[3] = millis();
  }
}
