#include "commands.h"
#include "motor_driver.h"

int enA = 6;//左边的轮子等待
int in1 = 5;
int in2 = 4;
// motor two
int enB = 9;//右边的轮子
int in3 = 7;
int in4 = 8;

void initMotorController() {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void setMotorSpeed(int i, int spd) {
  if (spd > MAX_PWM) {
    spd = MAX_PWM;
  }
  if (spd < -MAX_PWM) {
    spd = -1 * MAX_PWM;
  }
  if (i == LEFT) {
    if (spd >= 0) {
      digitalWrite(in2, HIGH);
      digitalWrite(in1, LOW);
      analogWrite(enA, spd);
    } else if (spd < 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, -spd);
    }
  } else {
    if (spd >= 0) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, spd);
    } else if (spd < 0) {
      digitalWrite(in4, HIGH);
      digitalWrite(in3, LOW);
      analogWrite(enB, -spd);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
