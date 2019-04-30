#include "encoder_driver.h"

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

void encoderLeftISR() {
  if (1 == digitalRead(3)) { // int 1
    left_enc_pos--;
    //Serial.println("L--");
  } else {
    left_enc_pos++;
    //Serial.println("L++");
  }
}

void encoderRightISR() {
  if (1 == digitalRead(18)) { //int 3
    right_enc_pos--;
    //Serial.println("R--");
  } else {
    right_enc_pos++;
    //Serial.println(right_enc_pos);
  }
}

void initEncoders() {
  pinMode(2, INPUT); //int0 BL motor_board_p06 //as left encoder A
  pinMode(3, INPUT); //int1 BR motor_board_p03 //as left encoder B
  pinMode(19, INPUT);//int4 AL motor_board_p09 //as right encoder A
  pinMode(18, INPUT);//int5 AR motor_board_p12 //as right encoder B 

  attachInterrupt(0, encoderLeftISR, RISING);  //pin 2
  attachInterrupt(4, encoderRightISR, RISING);//pin 19  
}

long readEncoder(int i) {
  static long encVal = 0L;
  if (i == LEFT)  {
    noInterrupts();
    encVal = left_enc_pos;
    interrupts();
  }
  else {
    noInterrupts();
    encVal = right_enc_pos;
    interrupts();
  }
  return encVal;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {//暂时不实现重置函数
  if (i == LEFT) {
    noInterrupts();
    left_enc_pos = 0L;
    interrupts();
    return;
  } else {
    noInterrupts();
    right_enc_pos = 0L;
    interrupts();
    return;
  }
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
