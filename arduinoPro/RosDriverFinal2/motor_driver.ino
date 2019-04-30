#include "motor_driver.h"

int enA = LEFT_ENA;//左边的轮子等待
int in1 = LEFT_IN1;
int in2 = LEFT_IN2;
int enB = RIGHT_ENB;//右边的轮子
int in3 = RIGHT_IN3;
int in4 = RIGHT_IN4;

extern SetPointInfo leftMotor, rightMotor;

void initMotorController() {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void setMotorSpeed(int i, int spd) {//spd是 >=0 的值
  if (spd > MAX_PWM) {
    spd = MAX_PWM;
  }
  if (i == LEFT) {
    if (leftMotor.dir == true) {//正传      
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, spd);
    } else if (leftMotor.dir == false) {//反转
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, spd);
    }
  } else {
    if (rightMotor.dir == true) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, spd);
    } else if (rightMotor.dir == false) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, spd);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

