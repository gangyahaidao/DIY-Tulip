#ifndef PWMCONTROLLER_H
#define PWMCONTROLLER_H

void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time);//用来设置舵机转动的角度以及时间
void ServoPwmDutyCompare(void);//脉冲变化比较及速度控制
void InitPWM(void);//初始化舵机控制对象

#endif
