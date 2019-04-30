#include "pid_controller.h"
#define TEST_SPEED 400.0
double Kp = 15.0 / 50;  //15 18 18
double Ki = 38.0 / 50;  //38 60 35
double Kd = 0.0 / 50;
double leftOutputSpeed, rightOutputSpeed;
unsigned char moving = 1; // 默认启动
SetPointInfo leftMotor, rightMotor;//存储左右轮子的状态值
PID myPIDL(&(leftMotor.input), &(leftMotor.output), &(leftMotor.TargetTicksPerFrame), Kp, Ki, Kd, DIRECT);
PID myPIDR(&(rightMotor.input), &(rightMotor.output), &(rightMotor.TargetTicksPerFrame), Kp, Ki, Kd, DIRECT);

void initPID() { //初始化PID控制类
  myPIDL.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);  
  resePID();
}

void resePID(){
  leftMotor.TargetTicksPerFrame = 0.0;
  //leftMotor.TargetTicksPerFrame = TEST_SPEED;
  leftMotor.output = 0;
  leftMotor.Encoder = readEncoder(LEFT);
  leftMotor.PrevInput = 0;
  leftMotor.PrevEnc = leftMotor.Encoder;
  leftMotor.dir = true;

  rightMotor.TargetTicksPerFrame = 0.0;
  //rightMotor.TargetTicksPerFrame = TEST_SPEED;
  rightMotor.output = 0;
  rightMotor.Encoder = readEncoder(RIGHT);
  rightMotor.PrevInput = 0;
  rightMotor.PrevEnc = rightMotor.Encoder;
  rightMotor.dir = true;
}

void doPID() {  
  leftMotor.input = abs(leftMotor.Encoder - leftMotor.PrevEnc)*8.18123086;//当前速度mm/s = s/t = (n*(PI*D/2400))/(1000/50)     D=125   EAI底盘最大速度800 mm/s
  rightMotor.input = abs(rightMotor.Encoder - rightMotor.PrevEnc)*8.18123086; 
  //Serial.println(leftMotor.input);
  if(leftMotor.input > MAX_SPEED){
    leftMotor.input = MAX_SPEED;
  }
  if(rightMotor.input > MAX_SPEED){
    rightMotor.input = MAX_SPEED;  
  }
  /*Serial.print('A');//用于PID波形调试
  Serial.println(leftMotor.input);
  Serial.print("; B");
  Serial.println(rightMotor.input);*/
  myPIDL.Compute();//计算PID输出值
  myPIDR.Compute();  
  /*Serial.print(leftMotor.output);
  Serial.print("; ");
  Serial.print(rightMotor.output);
  Serial.print(" = ");
  Serial.println(leftMotor.output - rightMotor.output);*/
  //左轮
  leftMotor.PrevEnc = leftMotor.Encoder;
  leftMotor.PrevInput = leftMotor.input;
  //右轮
  rightMotor.PrevEnc = rightMotor.Encoder;
  rightMotor.PrevInput = rightMotor.input;
}

void updatePIDSpeed() {
  //Serial.print(leftMotor.Encoder);
  //Serial.println(rightMotor.Encoder);
  if (!moving) {
    if (leftMotor.PrevInput != 0 || rightMotor.PrevInput != 0)
      initPID();
    return;
  }
  doPID();//更新PID
  if(leftMotor.TargetTicksPerFrame == 0){
    leftOutputSpeed = 0;
  }else{
    leftOutputSpeed = leftMotor.output;
  }
  if(rightMotor.TargetTicksPerFrame == 0){
    rightOutputSpeed = 0;
  }else{
    rightOutputSpeed = rightMotor.output;
  }
  setMotorSpeeds(leftOutputSpeed, rightOutputSpeed);  
  //setMotorSpeeds(MAX_PWM, MAX_PWM/2);
}
