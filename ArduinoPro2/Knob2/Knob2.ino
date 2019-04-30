/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(8);  // attaches the servo on pin 9 to the servo object  
}

void loop() {
  for(int i = 0; i < 100; i++){
    myservo.write(i*2);
    delay(20);
  }
  delay(1000);                           // waits for the servo to get there
  for(int i = 100; i > 0; i--){
    myservo.write(i*2);
    delay(20);
  }
  delay(1000);
}

