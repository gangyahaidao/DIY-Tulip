#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define MAX_PWM        255

#define LEFT_ENA      6  //左边电机PWM使能控制
#define LEFT_IN1      7
#define LEFT_IN2      8
#define RIGHT_ENAB    9  //右边电机PWM使能控制
#define RIGHT_IN3     10
#define RIGHT_IN4     11

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif

