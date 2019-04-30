#include <Servo.h> 

Servo servo[6];
void setup() {  
  servo[0].attach(3, 500, 2500);servo[0].writeMicroseconds(1500);
  servo[1].attach(5, 500, 2500);servo[1].writeMicroseconds(1500);
  servo[2].attach(6, 500, 2500);servo[2].writeMicroseconds(1000);
  servo[3].attach(9, 500, 2500);servo[3].writeMicroseconds(1500);
  servo[4].attach(10, 500, 2500);servo[4].writeMicroseconds(1500);
  servo[5].attach(11, 500, 2500);servo[5].writeMicroseconds(1500);
}

void loop() {
  // put your main code here, to run repeatedly:

}
