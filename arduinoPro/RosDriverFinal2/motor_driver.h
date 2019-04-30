#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define MAX_PWM        255

#define LEFT_ENA      6  //左边电机PWM使能控制
#define LEFT_IN1      4
#define LEFT_IN2      5
#define RIGHT_ENB     9  //右边电机PWM使能控制
#define RIGHT_IN3     7
#define RIGHT_IN4     8

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif

