#include <Servo.h>
#include <Pando.h>
#include "Gyro.h"
// #include "DFRobot_HT1632C.h"
#include <Wire.h>
//#include "GoBLE.h"

Pando Pando;  // This is Pando!

byte dato;  // To store the char sent by the app
//---------------------------------------------------------
//-- First step: Make sure the pins for servos are in the right position
/*
         ---------------
        |   |_|   |_|   |
        |---------------|
  YR 3==> |               | <== YL 2
         ---------------
            ||     ||
  RR 9==>   -----   ------  <== RL 8
         |-----   ------|
*/
#define PIN_YL 3 //servo[2]
#define PIN_YR 5 //servo[3]
#define PIN_RL 6 //servo[4]
#define PIN_RR 9 //servo[5]

#define TOUCH_PIN A0
#define NoiseSensor_PIN A1
int touchVal = 0;
int noiseVal = 0;

Gyro gyro;

//GoBLE Goble(Serial);// init the bluetooth Serial port
// Bluno default port - Serial

/*SOUNDS******************
   S_connection  S_disconnection  S_buttonPushed S_mode1 S_mode2 S_mode3 S_surprise S_OhOoh  S_OhOoh2  S_cuddly
   S_sleeping  S_happy S_superHappy S_happy_short S_sad S_confused S_fart1 S_fart2  S_fart3
*/

/*MOVEMENTS LIST**************
   dir=1---> FORWARD/LEFT
   dir=-1---> BACKWARD/RIGTH
   T : amount of movement. HIGHER VALUE SLOWER MOVEMENT usually 1000 (from 600 to 1400)
   h: height of mov. around 20
     jump(steps=1, int T = 2000);
     walk(steps, T, dir);
     turn(steps, T, dir);
     bend (steps, T, dir); //usually steps =1, T=2000
     shakeLeg (steps, T, dir);
     updown(steps, T, HEIGHT);
     swing(steps, T, HEIGHT);
     tiptoeSwing(steps, T, HEIGHT);
     jitter(steps, T, HEIGHT); (small T)
     ascendingTurn(steps, T, HEIGHT);
     moonwalker(steps, T, HEIGHT,dir);
     crusaito(steps, T, HEIGHT,dir);
     flapping(steps, T, HEIGHT,dir);
  /*GESTURES LIST***************
  PandoHappy PandoSuperHappy  PandoSad   PandoSleeping  PandoFart  PandoConfused PandoLove  PandoAngry
  PandoFretful PandoMagic  PandoWave  PandoVictory  PandoFail*/

///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200); //setup your bluetooth module to match this baudrate (or change it here)

  gyro.begin();

  pinMode(TOUCH_PIN, INPUT);

  // Set the servo pins
  //  Pando.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true);
  Pando.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, NoiseSensor_PIN);

  Pando.sing(S_connection); // Pando wake up!
  Pando.home();
  delay(50);
}



///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
   Pando.playGesture(PandoHappy);
   delay(1000);
  
   Pando.playGesture(PandoSuperHappy);
   delay(1000);
  
   Pando.playGesture(PandoSad);
   delay(1000);
  
   Pando.playGesture(PandoSleeping);
   delay(1000);
  
   Pando.playGesture(PandoFart);
   delay(1000);
  
   Pando.playGesture(PandoConfused);
   delay(1000);
  
   Pando.playGesture(PandoLove);
   delay(1000);
  
   Pando.playGesture(PandoAngry);
   delay(1000);
  
   Pando.playGesture(PandoFretful);
   delay(1000);
  
   Pando.playGesture(PandoThinking);
   delay(1000);
}


// test function
void testAllSongs() {
  Pando.sing(S_connection);
  delay(2000);
  Pando.sing(S_disconnection);
  delay(2000);
  Pando.sing(S_buttonPushed);
  delay(2000);
  Pando.sing(S_mode1);
  delay(2000);
  Pando.sing(S_mode2);
  delay(2000);
  Pando.sing(S_mode3);
  delay(2000);
  Pando.sing(S_surprise);
  delay(2000);
  Pando.sing(S_OhOoh);
  delay(2000);
  Pando.sing(S_OhOoh2);
  delay(2000);
  Pando.sing(S_cuddly);
  delay(2000);
  Pando.sing(S_sleeping);
  delay(2000);
  Pando.sing(S_happy);
  delay(2000);
  Pando.sing(S_superHappy);
  delay(2000);
  Pando.sing(S_happy_short);
  delay(2000);
  Pando.sing(S_sad);
  delay(2000);
  Pando.sing(S_confused);
  delay(2000);
  Pando.sing(S_fart1);
  delay(2000);
  Pando.sing(S_fart2);
  delay(2000);
  Pando.sing(S_fart3);
  delay(2000);
}
