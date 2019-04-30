#include "encoder_driver.h"

Encoder knobLeft(ENCODER_LEFT_A, ENCODER_LEFT_B);
Encoder knobRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

long readEncoder(int i) {
  static long encVal = 0L;
  if (i == LEFT)  {
    encVal = knobLeft.read();
  }
  else {
    encVal = knobRight.read();
  }
  return encVal;
}

void resetEncoder(int i) {
  if (i == LEFT) {
    knobLeft.write(0);
  } else {
    knobRight.write(0);
  }
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

long positionLeft  = -999;
long positionRight = -999;
void testEncoders() {
  while (1) {
    long newLeft, newRight;
    newLeft = knobLeft.read();
    newRight = knobRight.read();
    if (newLeft != positionLeft || newRight != positionRight) {
      Serial.print("Left = ");
      Serial.print(newLeft);
      Serial.print(";  Right = ");
      Serial.println(newRight);
      positionLeft = newLeft;
      positionRight = newRight;
    }
    delay(100);
  }
}
