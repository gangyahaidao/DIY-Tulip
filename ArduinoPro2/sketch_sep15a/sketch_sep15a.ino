#include <Servo.h> 
Servo mymotor; 
void setup() 
{ 
mymotor.attach(10); 
} 
void loop() 
{ 
mymotor.writeMicroseconds(500); //motors rotating in clockwise direction with the highest speed; 
delay(5000); //the motors running for 5 seconds. 
mymotor.writeMicroseconds(1000); //motors rotating in clockwise direction with the highest speed; 
delay(5000); //the motors running for 5 seconds. 
mymotor.writeMicroseconds(1500); //motors rotating in clockwise direction with the highest speed; 
delay(5000); //the motors running for 5 seconds. 
mymotor.writeMicroseconds(2000); //motors rotating in clockwise direction with the highest speed; 
delay(5000); //the motors running for 5 seconds. 
mymotor.writeMicroseconds(2500); //motors rotating in clockwise direction with the highest speed; 
delay(5000); //the motors running for 5 seconds. 

} 
