#include <Pando.h>
#include <SoftwareSerial.h>
unsigned char Re_buf[11], counter = 0;
float angle[3];

Pando Pando;  // This is Pando!
SoftwareSerial mySerial(8, 7);//rx tx

#define PIN_YL 3 //servo[2] 机器人正向左上舵机
#define PIN_YR 5 //servo[3]
#define PIN_RL 6 //servo[4]
#define PIN_RR 9 //servo[5]

#define TOUCH_PIN A0
#define NoiseSensor_PIN A1 // 麦克风
#define SoundSensor_PIN 10 //蜂鸣器 10
int touchVal = 0;
int noiseVal = 0;

unsigned long timeRun = 0, preTimeRun = 0;
int secondsCount = 0;
// 14种
int gesture_arr[] = {PandoHappy,PandoSuperHappy,PandoSad,PandoSleeping,PandoFart,PandoConfused,PandoLove,PandoAngry,PandoFretful,PandoMagic,PandoWave,PandoVictory,PandoFail,PandoThinking};
void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  mySerial.begin(9600);
  pinMode(SoundSensor_PIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT); // 触摸引脚
  Pando.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true, NoiseSensor_PIN, SoundSensor_PIN);
  Pando.sing(S_connection); // 播放启动音乐
  Pando.home();
  Pando.gazeAround(); // 显示一种LED效果
  delay(50);
  timeRun = millis(); // 获取当前程序运行的毫秒数
  preTimeRun = millis();  
}

void loop() {
  // testAllSongs();
  while (mySerial.available()) {
    Re_buf[counter] = (unsigned char)mySerial.read();
    if (counter == 0 && Re_buf[0] != 0x55) return; //第0号数据不是帧头
    counter++;
    if (counter == 11)          //接收到11个数据
    {
      counter = 0;             //重新赋值，准备下一帧数据的接收
      if (Re_buf[0] == 0x55)   //检查帧头
      {
        switch (Re_buf [1])
        {
          case 0x53:
            angle[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 180;
            angle[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 180;
            angle[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 180;
//            Serial.print("angle:");
//            Serial.print(angle[0]); Serial.print(" ");//左右  -90 ~ 90
//            Serial.print(angle[1]); Serial.println();//前后
            if (angle[0] >= 60 || angle[0] <= -60)
            {
              Serial.println("--imu left-right");
              Pando.playGesture(PandoFart);
            } else if(angle[1] >= 60 || angle[1] <= -60) {
              Serial.println("--imu front-back");
              Pando.playGesture(PandoFail);
            }
            break;
          default: break;
        }
      }
    }
  }
  touchVal = digitalRead(TOUCH_PIN);//检测到手指是高电平
  if (touchVal == HIGH) {
    Serial.println("--touched");
    Pando.playGesture(PandoLove);
    touchVal = !touchVal;
  }

  delay(20);
  noiseVal = Pando.getNoise();
//  Serial.print("noiseVal = ");
//  Serial.println(noiseVal);
  if (noiseVal > 100) {
    Serial.println("--noise");
    Pando.playGesture(PandoVictory);
    noiseVal = 0;
  }
    
  timeRun = millis();
  if((timeRun - preTimeRun) >= 1000 || (timeRun - preTimeRun) < 0) {
    preTimeRun = timeRun;
    secondsCount += 1; 
    if(secondsCount >= 60*3) {
      randomSeed(timeRun);
      int randomNum = random(14); // 输出0-13
      Serial.println("--timer");
      Pando.playGesture(gesture_arr[randomNum]); 
      secondsCount = 0;
    }
  }
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

