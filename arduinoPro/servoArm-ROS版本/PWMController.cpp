#include "common.h"
#include <Servo.h>

uint16 ServoPwmDuty[6] = {1500, 1500, 1500, 1500, 500, 500}; //PWM脉冲宽度，存储上一次转动之后的角度
uint16 ServoPwmDutySet[6] = {1500, 1500, 1500, 1500, 500, 500}; //PWM脉冲宽度,上位机下发的角度
float ServoPwmDutyInc[6];   //为了速度控制，当PWM脉宽发生变化时，每2.5ms或20ms递增的PWM脉宽

bool ServoPwmDutyHaveChange = FALSE;  //脉宽有变化标志位

uint16 ServoTime = 2000;      //舵机从当前角度运动到指定角度的时间，也就是控制速度

Servo myservo[6];  // create servo object to control a servo
bool ExecDone = false;

void ServoSetPluseAndTime(uint8 id, uint16 p, uint16 time) {//在话题接收函数中要依次设置各个舵机的目标值,参考“https://github.com/jesseweisberg/moveo_ros/blob/master/moveo_moveit/moveo_moveit_arduino/moveo_moveit_arduino.ino
  if (id >= 0 && id <= 5 && p >= 500 && p <= 2500)
  {
    if (time < 20)
      time = 20;
    if (time > 30000)
      time = 30000;
    ServoPwmDutySet[id] = p;
    ServoTime = time;
    ServoPwmDutyHaveChange = TRUE;
    ExecDone = false;
  }
}

void ServoPwmDutyCompare(void) { //脉宽变化比较及速度控制,每20ms调用一次
  uint8 i;

  static uint16 ServoPwmDutyIncTimes; //需要递增的次数
  static bool ServoRunning = FALSE; //舵机正在以指定速度运动到指定的脉宽对应的位置
  if (ServoPwmDutyHaveChange) //停止运动并且脉宽发生变化时才进行计算,主要是计算增量次数以及每次的增强值
  {
    ServoPwmDutyHaveChange = FALSE;
    ServoPwmDutyIncTimes = ServoTime / 20; //当每20ms调用一次ServoPwmDutyCompare()函数时用此句
    for (i = 0; i < 6; i++) {
      if (ServoPwmDutySet[i] > ServoPwmDuty[i]){//如果目标角度大于上一次停留的角度，将每次的增量变成负值,主要是跟下面写入的角度值对应
        ServoPwmDutyInc[i] = ServoPwmDutySet[i] - ServoPwmDuty[i];
        ServoPwmDutyInc[i] = -ServoPwmDutyInc[i];
      }else{
        ServoPwmDutyInc[i] = ServoPwmDuty[i] - ServoPwmDutySet[i];
      }
      ServoPwmDutyInc[i] /= ServoPwmDutyIncTimes;//每次递增的脉宽
    }
    ServoRunning = TRUE;  //舵机开始动作
  }
  if (ServoRunning){
    ServoPwmDutyIncTimes--;
    for (i = 0; i < 6; i++){
      if (ServoPwmDutyIncTimes == 0){ //最后一次递增就直接将设定值赋给当前值
        ServoPwmDuty[i] = ServoPwmDutySet[i];
        ServoRunning = FALSE; //到达设定位置，舵机停止运动
        ExecDone = true;
      }else{
        ServoPwmDuty[i] = ServoPwmDutySet[i] + (signed short int)(ServoPwmDutyInc[i] * ServoPwmDutyIncTimes);
      }
      if ((i >= 0) && (i < 6)){//依次写入角度值
        myservo[i].writeMicroseconds(ServoPwmDuty[i]);//写入旋转角度，1500是在中间，与write函数一样，只是参数是微妙us
      }
    }
  }
}

void InitPWM(void)
{
  myservo[0].attach(2, 500, 2500); // 后面两个参数是用来控制转动的最大最小角度  
  myservo[1].attach(3, 500, 2500);
  myservo[2].attach(4, 500, 2500);
  myservo[3].attach(5, 500, 2500);
  myservo[4].attach(6, 500, 2500);
  ServoSetPluseAndTime(4, 500, 2000);
  myservo[5].attach(7, 500, 2500);
  ServoSetPluseAndTime(5, 500, 2000);
}

