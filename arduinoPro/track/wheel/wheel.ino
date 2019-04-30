#include <avr/wdt.h>
#include <Wire.h>
#include <math.h>

#define TIMEOUT WDTO_8S
#define START_PIN 21
#define LEFT 5 //1-5
#define MIDDLE 11 //6-11
#define RIGHT 16 //12-16
#define FORWARD_SPEED 40
#define SIDE_FLOAT 5.0 
#define SIDE 5
#define RASIO 0.01
#define SLOWDOWN 0.85

int leftEdgePin, rightEdgePin;
int left, middle, right;

int al1 = 4; //左边的电机1方向LOW正转输入IN1
int alc = 3; //ENA

int ar1 = 12; //电机2输入IN3
int arc = 11; //ENB

String words1;
unsigned int state = 0;//0停止;1运动

unsigned int al_wheel; //记录U型测速模块的次数
unsigned int ar_wheel;

double speedAL;
double speedAR;

/**PID定义**/
//double kp=1.5, ki=0.0, kd=0.5; //single wheel test good
double kp = 1.5, ki = 0.00, kd = 1.2;
double speeds = 0; //角度平衡点，PWM差，死区，PWM1，PWM2

unsigned long SampleTime2 = 20; // 采样间隔时间ms
unsigned long preTime;
int pwm_al, pwm_ar, pwm_bl, pwm_br;
unsigned long pid_pre;

double lastErr_AL = 0;
double lastErr_AR = 0;

double errSum_AL = 0;
double errSum_AR = 0;

double Setpoint_AL = 0;//左右轮子的目标速度
double Setpoint_AR = 0;
int lastTime2;

void InitPid();
void go(int d_al, int d_ar);//参数为1或者0，轮子转或者不转
void CalculateByPID();
void pwm_output(int pwm_al, int pwm_ar, int pwm_bl, int pwm_br);
//编码器中断计数函数
void aLCount() {
  al_wheel++;
}
void aRCount() {
  ar_wheel++;
}

void setup ()
{
  Serial.begin(9600);
  Wire.begin();  
  attachInterrupt(0, aLCount, FALLING); // pin 2 al
  attachInterrupt(2, aRCount, FALLING); //pin 3 ar

  pinMode(al1, OUTPUT);
  pinMode(alc, OUTPUT);
  pinMode(ar1, OUTPUT);
  pinMode(arc, OUTPUT);
  //设置磁传感器的端口类型
  for(int i = 1; i <= 16; i++){
    pinMode(START_PIN + i, INPUT_PULLUP);
  }

  InitPid();
  wdt_enable(TIMEOUT);//看门狗8s复位
  Serial.println("setup down...");
}

void loop()
{  
  getLightedPinCount();
//  printSpeed();
//  delay(200);

  if((middle >= 4) && (left == 0) && (right == 0)){
    //直行
    moveForward(FORWARD_SPEED, FORWARD_SPEED);
    CalculateByPID();
    getLightedPinCount();
  }
  if(left > 0){
    if(left <= 3){
      turnLeft(FORWARD_SPEED*SLOWDOWN*(1-RASIO*left), FORWARD_SPEED*SLOWDOWN);
      CalculateByPID();
      getLightedPinCount();
    }else{
      while(left > 3){ 
        turnLeft(FORWARD_SPEED*SLOWDOWN*(1-RASIO*left*6), FORWARD_SPEED*SLOWDOWN);
        CalculateByPID();
        getLightedPinCount();
      } 
    }    
  }
  if(right > 0){
    if(right <= 3){
      turnRight(FORWARD_SPEED*SLOWDOWN, FORWARD_SPEED*SLOWDOWN*(1-RASIO*right));
      CalculateByPID();
      getLightedPinCount();
    }else{
      while(right > 3){ 
        turnRight(FORWARD_SPEED*SLOWDOWN, FORWARD_SPEED*SLOWDOWN*(1-RASIO*right*6)); 
        CalculateByPID();
        getLightedPinCount();             
      }   
    }   
  }
  
  if((left == 0) && (middle == 0) && (right == 0)){
    //turnRound(-FORWARD_SPEED, FORWARD_SPEED);    
    stopMove();
  }
  if((left > 0) && (middle > 0) && (right > 0)){
    //turnRound(-FORWARD_SPEED/2, FORWARD_SPEED/2);
    stopMove();
  }
  if (state == 1) {//如果小车当前处于运行状态则进行pid的调速
    CalculateByPID();
  }
  
  //重置看门狗
  wdt_reset();
}

void printSpeed(){
  Serial.print(left);
  Serial.print(", ");
  Serial.print(middle);
  Serial.print(", ");
  Serial.println(right);  
}

/**
  计算磁检测亮灯的数量
*/
void getLightedPinCount(){
  int readret[16] = {0};
  for(int i = 1; i <= 16; i++){
    readret[i-1] = digitalRead(START_PIN+i); 
    if(i == 1){
      leftEdgePin = readret[i-1];
    }
    if(i == 16){
      rightEdgePin = readret[i-1];  
    }
  }  
  int count = 0;
  for(int i = 1; i <= LEFT; i++){
    if(readret[i-1] == LOW){
      count++;
    }
  }
  left = count;
  count = 0;
  for(int i = LEFT+1; i <= MIDDLE; i++){
    if(readret[i-1] == LOW){
      count++;  
    }  
  }
  middle = count;
  count = 0;
  for(int i = MIDDLE+1; i <= RIGHT; i++){
    if(readret[i-1] == LOW){
      count++;  
    }  
  }
  right = count;
}

//指定特定的速度前进
void moveForward(int speedL, int speedR) {
  state = 1;
  Setpoint_AL = speedL;
  Setpoint_AR = speedR;
  go(1, 1);
}
void moveBack(int speedL, int speedR) {
  state = 1;
  Setpoint_AL = speedL;
  Setpoint_AR = speedR;
  go(-1, -1);
}
void turnLeft(int speedL, int speedR) {
  state = 1;
  Setpoint_AL = speedL;
  Setpoint_AR = speedR;
  go(1, 1);
}
void turnRight(int speedL, int speedR) {
  state = 1;
  Setpoint_AL = speedL;
  Setpoint_AR = speedR;
  go(1, 1);
}
void stopMove() {
  state = 0;
  go(0, 0);
}
//原地转圈
void turnRound(int speedL, int speedR){
  state = 1;
  Setpoint_AL = speedL;
  Setpoint_AR = speedR;
  go(-1, -1);
}

void InitPid()
{
  al_wheel = 0;
  ar_wheel = 0;
  pid_pre = millis();
}
//===============PID=====================
void CalculateByPID()
{
  unsigned long pid_now = millis();
  int pid_time = (pid_now - pid_pre);
  if (pid_time >= SampleTime2)
  { //compute all the working error variables
    pid_pre = pid_now;
    double al_count = (double)al_wheel;
    double ar_count = (double)ar_wheel;

    speedAL =  al_count / pid_time * 40.8854;
    speedAR =  ar_count / pid_time * 40.8854;

    al_wheel = 0;
    ar_wheel = 0;
    double Input_AL = speedAL;
    double Input_AR = speedAR;

    double error_AL = Setpoint_AL - Input_AL;
    double error_AR = Setpoint_AR - Input_AR;

    errSum_AL += error_AL;
    errSum_AR += error_AR;

    double dErr_AL = (error_AL - lastErr_AL);
    double dErr_AR = (error_AR - lastErr_AR);

    double Output_AL = kp * error_AL + ki * errSum_AL + kd * dErr_AL;
    double Output_AR = kp * error_AR + ki * errSum_AR + kd * dErr_AR;

    if (Output_AL > 255) Output_AL = 255;
    if (Output_AR > 255) Output_AR = 255;
    if (Output_AL < -255) Output_AL = -255;
    if (Output_AR < -255) Output_AR = -255;

    pwm_al =  pwm_al  + (int)Output_AL;
    pwm_ar =  pwm_ar  + (int)Output_AR;

    if (pwm_al > 255) pwm_al = 255;
    if (pwm_ar > 255) pwm_ar = 255;
    if (pwm_al < 0) pwm_al = 0;
    if (pwm_ar < 0) pwm_ar = 0;
    lastErr_AL = error_AL;
    lastErr_AR = error_AR;

    //将经过pid计算的PWM信号输入到电机的使能端
    pwm_output(pwm_al, pwm_ar);   
  }
}

void pwm_output(int pwm_al, int pwm_ar)
{
  analogWrite(alc, pwm_al);
  analogWrite(arc, pwm_ar);
  Serial.print(pwm_al);
  Serial.print(",");
  Serial.println(pwm_ar);
}

void go(int d_al, int d_ar)
{
  if (d_al > 0)
  { //正转
    digitalWrite(al1, LOW);
  }
  else if (d_al == 0)
  {
    analogWrite(alc, 0);
  }  else
  {
    digitalWrite(al1, HIGH);
  }
  
  if (d_ar > 0)
  {
    digitalWrite(ar1, LOW);
  }
  else if (d_ar == 0)
  {
    analogWrite(arc, 0);
  }
  else
  {
    digitalWrite(ar1, HIGH);
  }
}
