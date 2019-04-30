#ifndef Pando_h
#define Pando_h

#include <Servo.h>
#include "DFRobot_HT1632C.h"
#include <BatReader.h>

#include "Pando_eyes.h"
#include "Pando_sounds.h"
#include "Pando_gestures.h"
#include "common.h"

//-- Constants
#define FORWARD     1
#define BACKWARD    -1
#define LEFT        1
#define RIGHT       -1
#define SMALL       5
#define MEDIUM      15
#define BIG         30

#define PIN_Buzzer  10
#define PIN_NoiseSensor A1

#define SERVO_NUM 4
#define MOVE_TIME 2000
#define UL 0
#define UR 1
#define DL 2
#define DR 3

#ifndef DEG2RAD
  #define DEG2RAD(g) ((g)*M_PI)/180
#endif

class Pando
{
  public:
  
	void attachServos();
	void detachServos();

    //-- Pando initialization
    void init(int YL, int YR, int RL, int RR, bool load_calibration=true, int NoiseSensor=PIN_NoiseSensor, int Buzzer=PIN_Buzzer/*, int USTrigger=PIN_Trigger, int USEcho=PIN_Echo*/);

    //-- Predetermined Motion Functions
    void _moveServos(int time, int  servo_target[]);

    //-- HOME = Pando at rest position
    void home();
    bool getRestState();
    void setRestState(bool state);
    
    //-- Predetermined Motion Functions
    void jump(float steps=1, int T = 2000);

    void walk(float steps=4, int T=1000, int dir = FORWARD);
    void turn(float steps=4, int T=2000, int dir = LEFT);
    void bend (int steps=1, int T=1400, int dir=LEFT);
    void shakeLeg (int steps=1, int T = 2000, int dir=RIGHT);

    void updown(float steps=1, int T=1000, int h = 20);
    void swing(float steps=1, int T=1000, int h=20);
    void tiptoeSwing(float steps=1, int T=900, int h=20);
    void jitter(float steps=1, int T=500, int h=20);
    void ascendingTurn(float steps=1, int T=900, int h=20);

    void moonwalker(float steps=1, int T=900, int h=20, int dir=LEFT);
    void crusaito(float steps=1, int T=900, int h=20, int dir=FORWARD);
    void flapping(float steps=1, int T=1000, int h=20, int dir=FORWARD);

    //-- Sensors functions
    // float getDistance(); //US sensor
    int getNoise();      //Noise Sensor

    //-- Battery
    double getBatteryLevel();
    double getBatteryVoltage();
    
    //-- Mouth & Animations
    // void putMouth(unsigned long int mouth, bool predefined = true);
    // void putAnimationMouth(unsigned long int anim, int index);
    // void clearMouth();

    //-- Eye & Animations
    void print(const char str[], uint16_t speed);

    
    void putEyes(int eyeExpression);

    void smileEyes();
    void happyOpenEyes();
    void angryEyes();

    void sadEyes();
    void sadOpenEyes();
    void sadCloseEyes();

    void fartLeftEyes();
    void fartRightEyes();

    void bigEyes();
    void closeEyes();
    void surpriseEyes();
    void confusedEyes();
    void normalEyes();
    void normalEyesLeft();
    void normalEyesRight();
    void normalEyesUp();
    void normalEyesUpLeft();
    void normalEyesUpRight();
    void smallLoveEyes();
    void loveEyes();

    void blinkEyes();
    void binkLoveEyes();
    void gazeAround();

    //-- Sounds
    void _tone (float noteFrequency, long noteDuration, int silentDuration);
    void bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration);
    void sing(int songName);

    //-- Gestures
    void playGesture(int gesture);
	
	void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time);//用来设置舵机转动的角度以及时间
	void ServoPwmDutyCompare(void);//脉冲变化比较及速度控制
	void InitServos(int YL, int YR, int RL, int RR);//初始化舵机控制对象
	
  private:
    
    // MaxMatrix ledmatrix=MaxMatrix(12,10,11, 1);
    // DFRobot_HT1632C ht1632c = DFRobot_HT1632C(DATA, WR, CS);
    DFRobot_HT1632C ht1632c = DFRobot_HT1632C(11, 12, 4);

    BatReader battery;
	
	uint16 ServoPwmDuty[4]; //PWM脉冲宽度，存储上一次转动之后的角度
	uint16 ServoPwmDutySet[4]; //PWM脉冲宽度,上位机下发的角度
	float ServoPwmDutyInc[4];   //为了速度控制，当PWM脉宽发生变化时，每2.5ms或20ms递增的PWM脉宽
	bool ServoPwmDutyHaveChange;  //脉宽有变化标志位
	uint16 ServoTime;      //舵机从当前角度运动到指定角度的时间，也就是控制速度
	Servo myservo[4];  // create servo object to control a servo
	bool ExecDone;
    // US us;

    int servo_pins[4];
    int servo_trim[4];
    int servo_position[4];

    int pinBuzzer;
    int pinNoiseSensor;
    
    unsigned long final_time;
    unsigned long partial_time;
    float increment[4];

    bool isPandoResting;

    // unsigned long int getMouthShape(int number);
    // unsigned long int getAnimShape(int anim, int index);
    void _execute(int A[4], int O[4], int T, double phase_diff[4], float steps);

};

#endif


