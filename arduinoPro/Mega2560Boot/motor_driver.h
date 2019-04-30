#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif
