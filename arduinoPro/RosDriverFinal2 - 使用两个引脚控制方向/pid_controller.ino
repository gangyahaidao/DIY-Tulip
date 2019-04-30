#include "pid_controller.h"

double Kp = 18.0 / 50;
double Ki = 35.0 / 50;
double Kd = 0.0 / 50;
unsigned char moving = 1; // 默认启动
SetPointInfo leftMotor, rightMotor;//存储左右轮子的状态值
PID myPIDL(&(leftMotor.input), &(leftMotor.output), &(leftMotor.TargetTicksPerFrame), Kp, Ki, Kd, DIRECT);
PID myPIDR(&(rightMotor.input), &(rightMotor.output), &(rightMotor.TargetTicksPerFrame), Kp, Ki, Kd, DIRECT);

void initPID() { //初始化PID控制类
  myPIDL.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);
  leftMotor.TargetTicksPerFrame = 0.0;
  leftMotor.Encoder = readEncoder(LEFT);
  leftMotor.output = 0;
  leftMotor.Encoder = readEncoder(LEFT);
  leftMotor.PrevInput = 0;
  leftMotor.PrevEnc = leftMotor.Encoder;
  leftMotor.dir = true;

  rightMotor.TargetTicksPerFrame = 0.0;
  rightMotor.Encoder = readEncoder(RIGHT);
  rightMotor.output = 0;
  rightMotor.Encoder = readEncoder(RIGHT);
  rightMotor.PrevInput = 0;
  rightMotor.PrevEnc = rightMotor.Encoder;
  rightMotor.dir = true;
}

void doPID() {  
  leftMotor.input = (leftMotor.Encoder - leftMotor.PrevEnc)*19.792034;//当前速度mm/s = s/t = (n*(PI*D/600))/(1000/30)
  rightMotor.input = (rightMotor.Encoder - rightMotor.PrevEnc)*19.792034;
  if(leftMotor.input >= 655){
    leftMotor.input = 655;  
  }
  if(rightMotor.input >= 655){
    rightMotor.input = 655;  
  }
  /*Serial.print('A');//用于PID波形调试
  Serial.print(leftMotor.input);
  Serial.print("; B");
  Serial.println(rightMotor.input);*/
  myPIDL.Compute();//计算PID输出值
  myPIDR.Compute();  
  /*Serial.print(leftMotor.output);
  Serial.print("; ");
  Serial.println(rightMotor.output);*/
  //左轮
  leftMotor.PrevEnc = leftMotor.Encoder;
  leftMotor.PrevInput = leftMotor.input;
  //右轮
  rightMotor.PrevEnc = rightMotor.Encoder;
  rightMotor.PrevInput = rightMotor.input;
}

void updatePIDSpeed() {
  leftMotor.Encoder = readEncoder(LEFT);
  rightMotor.Encoder = readEncoder(RIGHT);
  //Serial.print(leftMotor.Encoder);
  //Serial.println(rightMotor.Encoder);
  if (!moving) {
    if (leftMotor.PrevInput != 0 || rightMotor.PrevInput != 0)
      initPID();
    return;
  }
  doPID();//更新PID
  setMotorSpeeds(leftMotor.output, rightMotor.output);
  //setMotorSpeeds(MAX_PWM, MAX_PWM);
}
