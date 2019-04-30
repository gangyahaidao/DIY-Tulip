#include "turtlebot3_core_config.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"

#define BUADRATE 576000
/*******************************************************************************
  SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[5];
int aix, aiy, aiz;// 加速度计原始数据
int gix, giy, giz;// 陀螺仪原始数据
float ax, ay, az;//加速度值
float gx, gy, gz;//角速度值
float angle[3], q0, q1, q2, q3;
//the parameter of PID controller
SRobotOdomData odomData;
SRobotSpeedData speedData;
SRobotBatteryData batteryData;
SRobotAcceData acceData;
SRobotGyroData gyroData;
SRobotYPRData yprData;
Madgwick filter;//陀螺仪数据滤波对象
extern SetPointInfo leftMotor, rightMotor;

/*******************************************************************************
  Setup function
*******************************************************************************/
void setup()
{
  //初始化电机PID和编码器
  initEncoders();//初始化编码器中断库对象
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  resetPID();

  Serial.begin(BUADRATE);//ROS

  // Setting for IMU
  //imu.begin();
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

  pinMode(13, OUTPUT);
}

/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{  
  //1.计算更新PID速度控制
  if ((millis() - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD)) {
    updatePID();
    tTime[0] = millis();
  }

  if ((millis() - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD)) {
    
    tTime[1] = millis();
  }

  if ((millis() - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD)) {
    
    tTime[2] = millis();
  }

  if ((millis() - tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD)) {
    
    tTime[3] = millis();
  }

  if ((millis() - tTime[4]) >= (1000 / 200)) {//陀螺仪数据发布频率200hz
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
    //计算四元数
    q0 = filter.q0;
    q1 = filter.q1;
    q2 = filter.q2;
    q3 = filter.q3;
    tTime[4] = millis();
  }
}

int match_flag = 0;
BYTE inChar;
BYTE match[2];
void serialEvent() {//中断方式接收上位机发送的串口数据
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
