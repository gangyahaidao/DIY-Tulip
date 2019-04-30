/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo1.attach(3);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(5);
  myservo3.attach(6);
  myservo4.attach(9);  
}

void loop() {
    myservo1.write(90);              // tell servo to go to position in variable 'pos'
    myservo2.write(90);
    myservo3.write(90);
    myservo4.write(90);
    delay(15); 
}

