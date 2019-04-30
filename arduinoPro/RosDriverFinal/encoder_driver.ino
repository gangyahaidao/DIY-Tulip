#include "encoder_driver.h"

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
Wheel wheelL, wheelR;//定义左轮和右轮相关的变量的结构体对象

void initEncoders() {
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP); //int0 as left encoder A
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP); //as left encoder B
  pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);//int1 as right encoder A
  pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);//as right encoder B
  attachInterrupt(0, encoderLeftISR, RISING);  //pin 2
  attachInterrupt(1, encoderRightISR, RISING);//pin 3
  //初始化轮子结构体
  wheelL.counter = 0;
  wheelL.count = 0;
  wheelL.dir = 0;
  wheelL.encoder_a_pin = ENCODER_LEFT_A_PIN;
  wheelL.encoder_b_pin = ENCODER_LEFT_B_PIN;
  wheelR.counter = 0;
  wheelR.count = 0;
  wheelR.dir = 0;
  wheelR.encoder_a_pin = ENCODER_RIGHT_A_PIN;
  wheelR.encoder_b_pin = ENCODER_RIGHT_B_PIN;
}

void encoderLeftISR() {
  wheelL.count++;
  if (HIGH == digitalRead(3)) { // int 1
    wheelL.counter--;
    wheelL.dir = 1;//后退为1
    //Serial.println("L--");
  } else {
    wheelL.counter++;
    wheelL.dir = 0;//前进为0
    //Serial.println("L++");
  }
}

void encoderRightISR() {
  wheelL.count++;
  if (HIGH == digitalRead(18)) { //int 3
    wheelR.counter--;
    wheelR.dir = 1;
    //Serial.println("R--");
  } else {
    wheelR.counter++;
    wheelR.dir = 0;
    //Serial.println(right_enc_pos);
  }
}

long readEncoder(int i) {
  static long encVal = 0L;
  if (i == LEFT)  {
    noInterrupts();
    encVal = wheelL.counter;
    interrupts();
  }
  else {
    noInterrupts();
    encVal = wheelR.counter;
    interrupts();
  }
  return encVal;
}

void resetEncoder(int i) {
  if (i == LEFT) {
    noInterrupts();
    wheelL.counter = 0L;
    interrupts();
    return;
  } else {
    noInterrupts();
    wheelR.counter = 0L;
    interrupts();
    return;
  }
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
