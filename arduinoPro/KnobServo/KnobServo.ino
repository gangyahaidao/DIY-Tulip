/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo1;

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  myservo1.attach(8);  // attaches the servo on pin 9 to the servo object
  myservo1.write(90);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);
}

void loop() {
                    // waits for the servo to get there
}

