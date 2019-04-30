#include <PID_v1.h>
#define FORWARD_SPEED 450//速度是mm/s  最大速度993.7896 mm/s
#define TURN_SLOWDOWN_RASIO 0.65
#define INTERVAL_MS 20//pid计算周期ms，用于计算当前转速
//#define DEBUG//开启调试PID模式
#undef DEBUG

#define START_PIN 21//磁传感器起始引脚-1值
#define LEFT 5 //1-5
#define LEFT_COUNT 5  //左边亮灯数量
#define MIDDLE 11 //6-11
#define MIDDLE_COUNT 6
#define RIGHT 16 //12-16
#define RIGHT_COUNT 5
#define HALF_TURN_COUNTS 328

#define TURN_RASIO 1.68
#define ANGLE_DELTA 4

int readret[16] = {0};//存储每个传感器的检测值
int left = 0, middle = 0, right = 0;

int al1 = 4; //左边的电机1方向LOW正转输入IN1
int alc = 3; //ENA
int ar1 = 12; //电机2输入IN3
int arc = 11; //ENB

unsigned int state = 0;//0停止;1运动
unsigned int al_wheel; //记录脉冲次数,一圈是663
unsigned int ar_wheel;
unsigned int al_wheel_turn;
unsigned int ar_wheel_turn;
unsigned long pid_pre;
unsigned long pid_now = 0;

double Setpoint_AL = 0;//左右轮子的目标速度
double Setpoint_AR = 0;
double speedAL = 0;//轮子当前速度 mm/s
double speedAR = 0;
double pwm_AL = 0, pwm_AR = 0;
//设置PID值
//double kp1 = 0.004, ki1 =0.8, kd1 = 0.001;  //800速度
//double kp2 = 2.2, ki2 = 50.0, kd2 = 0.0001;//955~990速度用于返回

double kp3 = 0.058, ki3 = 0.95, kd3 = 0.0008;//600速度用于带路double kp3 = 0.0865, ki3 = 2.1, kd3 = 0.0005;
PID myPID_L(&speedAL, &pwm_AL, &Setpoint_AL, kp3, ki3, kd3, DIRECT);
PID myPID_R(&speedAR, &pwm_AR, &Setpoint_AR, kp3, ki3, kd3, DIRECT);

//PID myPID_L(&speedAL, &pwm_AL, &Setpoint_AL, kp2, ki2, kd2, DIRECT);
//PID myPID_R(&speedAR, &pwm_AR, &Setpoint_AR, kp2, ki2, kd2, DIRECT);
//PID myPID_L(&speedAL, &pwm_AL, &Setpoint_AL, kp1, ki1, kd1, DIRECT);
//PID myPID_R(&speedAR, &pwm_AR, &Setpoint_AR, kp1, ki1, kd1, DIRECT);

boolean outFromMiddle = false, outFromLeft = false, outFromRight = false;
boolean stopL = false, stopR = false, stopBoth = false;
String comdata = "WWO";
String anglestr = "";
int datalength = 3;
int dataindex = -1;
float distanceSR04 = 0;
//串口收到原地转动方向
bool hasReceiveTurnData = false;
bool hasReceiveMoveData = false;
int turnAngle = 0;//转动的角度
int doTurnAngle = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  attachInterrupt(0, aLCount, FALLING); // pin 2 al
  attachInterrupt(2, aRCount, FALLING); //pin 21 ar
  pinMode(al1, OUTPUT);pinMode(alc, OUTPUT);pinMode(ar1, OUTPUT);pinMode(arc, OUTPUT);
  myPID_L.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);
  //设置磁传感器的端口类型
  for (int i = 1; i <= 16; i++) {
    pinMode(START_PIN + i, INPUT_PULLUP);
  }
  al_wheel_turn = 0;
  ar_wheel_turn = 0;
  InitPid();
}

void loop() {   
  #ifdef DEBUG   
  while(1){
    getLightedPinCount();
    if (left == 0 && middle == 0 && right == 0) {
      //stopMove();
    }
    stopBoth = true;
    moveForward(FORWARD_SPEED, FORWARD_SPEED);   
    refreshPWM();//计算pwd，电机使能 
  }  
  #endif
  
  if (Serial1.available() > 0) {//如果串口有数据
    comdata = "";
    datalength = 0;
    anglestr = "";    
    turnAngle = 0;
    char cc;
    while (Serial1.available() > 0) {
      cc = char(Serial1.read());
      if(isdigit(cc)){
        hasReceiveTurnData = true;
        hasReceiveMoveData = false;
        anglestr += cc;
      }else{
        hasReceiveMoveData = true;
        comdata += cc;
      }      
      delay(3);
    }
    Serial1.flush();
    if(hasReceiveTurnData==true && hasReceiveMoveData==false){
      turnAngle = anglestr.toInt();      
      //进行转向，角度<90时向右转doTurnAngle<0，角度>=90向左转doTurnAngle>=0
      doTurnAngle = turnAngle - 90;
      if(doTurnAngle < 0){
        turnRight(FORWARD_SPEED/2, -FORWARD_SPEED/2);
      }else{
        turnLeft(-FORWARD_SPEED/2, FORWARD_SPEED/2);
      }
      //进行转向
      stopBoth = false;
      turnTheCounts((int)(HALF_TURN_COUNTS/90.0*abs(doTurnAngle)));
      stopMove();//停止
    }else{
      datalength = comdata.length();
    }    
  }    

  //delay(6);//延迟一下，等待PID更新
  getLightedPinCount();
  //printLighted();delay(200);
  if (left == 0 && middle > 0 && right == 0 && datalength > 0 && dataindex != -1) {//满足只有中间亮且有收到数据，前进
    moveForward(FORWARD_SPEED, FORWARD_SPEED);
  } else if (left > 0 && datalength > 0 && dataindex != -1) {//小车右边的灯亮且收到数据，右拐
    if(middle == 0){
      turnRight(FORWARD_SPEED, FORWARD_SPEED*(1-sin(ANGLE_DELTA*(LEFT_COUNT-left))));
    }else{
      turnRight(FORWARD_SPEED, FORWARD_SPEED - pow(TURN_RASIO, left));
    }   
  } else if (right > 0 && datalength > 0 && dataindex != -1) {//小车左边的灯亮且收到数据，左拐
    if(middle == 0){
      turnLeft(FORWARD_SPEED*(1-sin(ANGLE_DELTA*(RIGHT_COUNT-right))), FORWARD_SPEED);
    }else{
      turnLeft(FORWARD_SPEED - pow(TURN_RASIO, right), FORWARD_SPEED);
    }
  }
  
  if (left == 0 && middle == 0 && right == 0) { //小车没有在轨道上：1.在行进过程中，跑出了轨道；2.播放通知前到达终点P；3.播放通知后处于终点P，要进行转180度        
    stopMove();
    if (datalength > 1 && dataindex != -1) { //行进中间跑出了轨道，后退          
      stopBoth = false;
      moveBack(FORWARD_SPEED/3, FORWARD_SPEED/3);
      while (1) {
        getLightedPinCount();
        if (middle >= 4) {
          break;
        }
        refreshPWM();
      }
    } else if (datalength == 1 && dataindex > 0) { //达到终点P，停止同时清空方向数据，等待返回命令      
      if(comdata[dataindex+1] == 'P'){//到达目的地停止      
        stopMove(); 
        comdata = "";
        datalength = 0;
        dataindex = -1;        
        Serial1.print('P');//向上位机发送'P'字符，表示机器人已经达到了停靠点
      }else if(comdata[dataindex+1] == 'O'){//返回到达原点，转180度，然后停止等待新的下一条命令
        stopMove();
        stopBoth = false;
        turnLeft(-FORWARD_SPEED / 2, FORWARD_SPEED / 2);
        turnTheCounts(HALF_TURN_COUNTS*2);
        stopMove();
        stopBoth = false;
        while (1) {
          getLightedPinCount();
          if (middle > 0 || right > 0 || left > 0) {//right是左边的灯数量
            stopMove();break;
          }
          moveForward(FORWARD_SPEED/2, FORWARD_SPEED/2);
          refreshPWM();
        }
        comdata = "";
        datalength = 0;
        dataindex = -1;
        Serial1.print('O');
      }      
    } else if (datalength > 0 && dataindex == -1 && comdata[0] == 'H') { //收到返回命令，转180度
      dataindex++;
      datalength--;
      stopBoth = false;
      //进行转弯
      turnLeft(-FORWARD_SPEED/2, FORWARD_SPEED/2);
      turnTheCounts(HALF_TURN_COUNTS*2);
      stopMove();
      stopBoth = false;
      while (1) {
        getLightedPinCount();
        if (middle > 0 || right > 0 || left > 0) {//right是左边的灯数量
          break;
        }
        moveForward(FORWARD_SPEED, FORWARD_SPEED);
        refreshPWM();
      }
    }
  }
  if (left == LEFT_COUNT && middle == MIDDLE_COUNT && right == RIGHT_COUNT && datalength > 0 && dataindex >= 0) { //行进到站点，不包括停止点
    dataindex++;
    datalength--;    
    if (comdata[dataindex] == 'W') {//遇到站点W命令，直接前进
      stopBoth = false;
      moveForward(FORWARD_SPEED, FORWARD_SPEED);
      while (left == LEFT_COUNT && middle == MIDDLE_COUNT && right == RIGHT_COUNT) {
        getLightedPinCount();
        refreshPWM();
      }
    } else {
      stopMove();
      stopBoth = false;
      if (comdata[dataindex] == 'A') {//站点A命令，左拐
        moveForward(FORWARD_SPEED, FORWARD_SPEED);//前进直到所有的灯不是都在磁带上
        while (left == LEFT_COUNT && middle == MIDDLE_COUNT && right == RIGHT_COUNT) {
          getLightedPinCount();
          refreshPWM();
        }
        stopMove();
        stopBoth = false;
        turnLeft(-FORWARD_SPEED / 2, FORWARD_SPEED / 2);
        turnTheCounts(HALF_TURN_COUNTS);
        moveForward(FORWARD_SPEED/2, FORWARD_SPEED/2);//前进直到所有的灯不是都在磁带上，避免在转弯之后检测到全亮
        while (left == LEFT_COUNT && middle == MIDDLE_COUNT && right == RIGHT_COUNT) {
          getLightedPinCount();
          refreshPWM();
        }
      } else if (comdata[dataindex] == 'D') {//站点D命令，右拐
        moveForward(FORWARD_SPEED, FORWARD_SPEED);//前进直到所有的灯不是都在磁带上
        while (left == LEFT_COUNT && middle == MIDDLE_COUNT && right == RIGHT_COUNT) {
          getLightedPinCount();
          refreshPWM();
        }
        stopMove();
        stopBoth = false;
        turnRight(FORWARD_SPEED / 2, -FORWARD_SPEED / 2);
        turnTheCounts(HALF_TURN_COUNTS);
      }
    }
  }else if(datalength > 0 && dataindex == -1 && (left > 0 || middle > 0 || right > 0) && comdata[0] == 'W'){//在原点等待第一条命令
      //判断是否经过唤醒有转动      
      if(hasReceiveTurnData){        
        stopBoth = false;//使能转动
        if(doTurnAngle < 0){
          turnLeft(-FORWARD_SPEED/2, FORWARD_SPEED/2);
        }else{
          turnRight(FORWARD_SPEED/2, -FORWARD_SPEED/2);
        }
        //进行转向
        turnTheCounts((int)(HALF_TURN_COUNTS/90.0*abs(doTurnAngle)));
        doTurnAngle = 0;
        hasReceiveTurnData = false;
        stopMove();//停止    
      }
      dataindex++;
      datalength--;
      stopBoth = false;
      moveForward(FORWARD_SPEED, FORWARD_SPEED);
  }
  refreshPWM();//计算pwd，电机使能
}

  //编码器中断计数函数
  void aLCount() {
    al_wheel++;
    al_wheel_turn++;
  }
  void aRCount() {
    ar_wheel++;
    ar_wheel_turn++;
  }
  //初始化计算PID相关的变量
  void InitPid()
  {
    al_wheel = 0;
    ar_wheel = 0;
    pid_pre = millis();
  }
  //计算转速，作为PID输入参数，并更新PWM
  int cc = 0;
  int dd = 0;
  void refreshPWM() {
    pid_now = millis();
    int pid_time = (pid_now - pid_pre);
    if (pid_time >= INTERVAL_MS) {
      pid_pre = pid_now;
      double al_count = (double)al_wheel;
      double ar_count = (double)ar_wheel;
      speedAL =  al_count / pid_time * 615.998;//将波形次数与时间相除，每毫秒的转数*每毫秒移动的距离mm = 当前速度mm/ms*1000
      speedAR =  ar_count / pid_time * 615.998;
      //#ifdef DEBUG
      Serial1.print('A');//用于PID波形调试
      Serial1.println(speedAL);
      Serial1.print('B');
      Serial1.println(speedAR);
      //#endif
      //进行一次PID计算
      myPID_L.Compute();
      myPID_R.Compute();
      //设置PWM
      if (stopBoth) {
        analogWrite(alc, 0);
        analogWrite(arc, 0);
      } else if (stopL) {
        analogWrite(alc, 0);
        analogWrite(arc, pwm_AR);
      } else if (stopR) {
        analogWrite(alc, pwm_AL);
        analogWrite(arc, 0);
      } else {
        analogWrite(alc, pwm_AL);
        analogWrite(arc, pwm_AR);
      }
      al_wheel = 0;//圈数清零
      ar_wheel = 0;
    }
  }

  //指定特定的速度前进
  void moveForward(int speedL, int speedR) {
    Setpoint_AL = speedL;
    Setpoint_AR = speedR;
    go(1, 1);
  }
  void moveBack(int speedL, int speedR) {
    Setpoint_AL = speedL;
    Setpoint_AR = speedR;
    go(-1, -1);
  }
  void turnLeft(int speedL, int speedR) {//左轮速度比右轮速度慢
    if (speedL < 0) {
      Setpoint_AL = abs(speedL);
      Setpoint_AR = speedR;
      go(-1, 1);
    } else {
      Setpoint_AL = speedL;
      Setpoint_AR = speedR;
      go(1, 1);
    }
  }
  void turnRight(int speedL, int speedR) {
    if (speedR < 0) {
      Setpoint_AL = speedL;
      Setpoint_AR = abs(speedR);
      go(1, -1);
    } else {
      Setpoint_AL = speedL;
      Setpoint_AR = speedR;
      go(1, 1);
    }
  }
  void stopMove() {
    stopBoth = true;
    go(0, 0);
  }
  //原地转圈
  void turnRound(int speedL, int speedR) {
    Setpoint_AL = speedL;
    Setpoint_AR = speedR;
    go(-1, 1);
  }

  void go(int d_al, int d_ar)
  {
    if (d_al > 0) { //正转
      digitalWrite(al1, LOW);
    } else if (d_al == 0) {
      analogWrite(alc, 0);
    } else {
      digitalWrite(al1, HIGH);
    }
    if (d_ar > 0) {
      digitalWrite(ar1, LOW);
    } else if (d_ar == 0) {
      analogWrite(arc, 0);
    } else {
      digitalWrite(ar1, HIGH);
    }
  }

  /**
    计算磁检测亮灯的数量
  */
  void getLightedPinCount() {
    left = 0; middle = 0; right = 0;
    for (int i = 1; i <= 16; i++) {
      readret[i - 1] = digitalRead(START_PIN + i);
      if (i >= 1 && i <= LEFT && readret[i - 1] == LOW) {
        left++;
      } else if (i >= LEFT + 1 && i <= MIDDLE && readret[i - 1] == LOW) {
        middle++;
      } else if (i >= MIDDLE + 1 && i <= RIGHT && readret[i - 1] == LOW) {
        right++;
      }
    }
  }

  void myPrint(String name, int value) {
    Serial.print(name);
    Serial.print(" = ");
    Serial.println(value);
  }
  void printLighted() {
    Serial.print(left);
    Serial.print(", ");
    Serial.print(middle);
    Serial.print(", ");
    Serial.println(right);
  }

  void turnTheCounts(int n) { //转动指定的圈数
    if(n == 0){return;}
    int destL = al_wheel_turn + n;//目标圈数，转半圈
    int destR = ar_wheel_turn + n;
    if (destL < al_wheel_turn) { //说明到达了uint最大值
      al_wheel_turn = 0;
      destL = al_wheel_turn + n;
    }
    if (destR < ar_wheel_turn) { //说明到达了uint最大值
      ar_wheel_turn = 0;
      destR = ar_wheel_turn + n;
    }
    while (destL > al_wheel_turn || destR > ar_wheel_turn) { //循环直到转够圈数
      refreshPWM();
    }
  }
 
