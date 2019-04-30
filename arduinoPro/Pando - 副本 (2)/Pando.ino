#include <Servo.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Pando.h>

Pando Pando;  // This is Pando!
#define PIN_YL 3 //servo[2]
#define PIN_YR 5 //servo[3]
#define PIN_RL 6 //servo[4]
#define PIN_RR 9 //servo[5]

////点阵控制引脚
//#define DATA 11
//#define WR 4
//#define CS 12
#define TOUCH_PIN A0
#define NoiseSensor_PIN A1
#define SoundSensor_PIN 10 //蜂鸣器
int touchVal = 0;
int noiseVal = 0;
//播放音乐的变量
int length = 15; // the number of notes
char notes[] = "ccggaagffeeddc "; // a space represents a rest
int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };
int tempo = 110;

int aix, aiy, aiz;// 加速度计原始数据
int gix, giy, giz;// 陀螺仪原始数据
float ax, ay, az;//加速度值
float gx, gy, gz;//角速度值
float angle[3], q0, q1, q2, q3;
Madgwick filter;

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

void setup() {
  Serial.begin(115200); //setup your bluetooth module to match this baudrate (or change it here)
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  CurieIMU.begin();
  CurieIMU.setAccelerometerRate(20);
  CurieIMU.setAccelerometerRange(2);//参照opencr设置
  CurieIMU.setGyroRate(20);
  CurieIMU.setGyroRange(250);//(+/-2000°/s)
  filter.begin(20);
  CurieIMU.autoCalibrateGyroOffset();//陀螺仪自动校准
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);//加速度自动计校准
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  
  pinMode(SoundSensor_PIN, OUTPUT);
  //playABCD();
  pinMode(TOUCH_PIN, INPUT);

  Pando.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, NoiseSensor_PIN, SoundSensor_PIN);
  Pando.sing(S_connection); // Pando wake up!
  Pando.home();
  Pando.gazeAround();
  delay(50);  
}

void loop() {
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    ax = (aix * 8.0) / 32768.0;
    ay = (aiy * 8.0) / 32768.0;
    az = (aiz * 8.0) / 32768.0;
    gx = (gix * 2000.0) / 32768.0;
    gy = (giy * 2000.0) / 32768.0;
    gz = (giz * 2000.0) / 32768.0;
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    //计算欧拉角
    angle[0] = filter.getRoll();
    angle[1] = filter.getPitch();
    angle[2] = filter.getYaw();
    Serial.print("angle:");
    Serial.print(angle[0]); Serial.print(" ");
    Serial.print(angle[1]); Serial.print(" ");
    Serial.print(angle[2]); Serial.println();
    
  //test touch sensor
  touchVal = digitalRead(TOUCH_PIN);//检测到手指是高电平
  if (touchVal == HIGH) {
    Pando.playGesture(PandoLove);
    delay(10);
    touchVal = !touchVal;
  }

  // test noise sensor
    noiseVal = Pando.getNoise();
//  Serial.print("noiseVal = ");
//  Serial.println(noiseVal);
//  delay(100);
  if (noiseVal > 100) {
    Pando.playGesture(PandoSuperHappy);
    noiseVal = 0;
  }

  testAllSongs();
}

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(SoundSensor_PIN, HIGH);
    delayMicroseconds(tone);
    digitalWrite(SoundSensor_PIN, LOW);
    delayMicroseconds(tone);
  }
}
void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };

  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}

void playABCD()
{
  for (int i = 0; i < length; i++) {
    if (notes[i] == ' ') {
      delay(beats[i] * tempo); // rest
    } else {
      playNote(notes[i], beats[i] * tempo);
    }
    // pause between notes
    delay(tempo / 2);
  }
}

void testAllSongs() {
  Pando.sing(S_connection);
  delay(2000);
  Pando.sing(S_disconnection);
  delay(2000);
  Pando.sing(S_buttonPushed);
  delay(2000);
  Pando.sing(S_mode1);
  delay(2000);
  Pando.sing(S_mode2);//11
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
  Pando.sing(S_happy_short);//11
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
