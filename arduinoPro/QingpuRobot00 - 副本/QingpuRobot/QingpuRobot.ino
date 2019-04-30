#define BAUDRATE     115200
#define MAX_PWM        255

#include "Arduino.h"
#include "commands.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "encoder_driver.h"

#define PID_RASIO 50.0
#define PID_RATE           50     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;
#define AUTO_STOP_INTERVAL 5000
long lastMotorCommand = AUTO_STOP_INTERVAL;
extern unsigned char moving;
extern SetPointInfo leftMotor, rightMotor;
extern double Kp;
extern double Ki;
extern double Kd;
extern PID myPIDL, myPIDR;
/* Variable initialization */
int arg = 0;
int index0 = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;
unsigned long time = 0, old_time = 0;
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index0 = 0;
}
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case READ_PIDIN:
      Serial.print( readPidIn(LEFT));
      Serial.print(" ");
      Serial.println( readPidIn(RIGHT));
      break;
    case READ_PIDOUT:
      Serial.print( readPidOut(LEFT));
      Serial.print(" ");
      Serial.println( readPidOut(RIGHT));
      break;
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        moving = 0;
      }else{
        moving = 1;  
      }
      //需要将接收的每帧tick数转换为mm/s
      leftMotor.TargetTicksPerFrame = arg1*19.792034;//向全局对象变量赋值,最大支持输入33
      rightMotor.TargetTicksPerFrame = arg2*19.792034;
      Serial.println("OK");
      Serial.println(leftMotor.TargetTicksPerFrame);
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0]/PID_RASIO;
      Ki = pid_args[1]/PID_RASIO;
      Kd = pid_args[2]/PID_RASIO;
      myPIDL.SetTunings(Kp, Ki, Kd);
      myPIDR.SetTunings(Kp, Ki, Kd);
      Serial.println("PID OK");
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}
/***********************************************
 * 程序初始化setup()
 ***********************************************/
void setup() {  
  Serial.begin(BAUDRATE);//连接到上位机
  Serial2.begin(BAUDRATE);//连接到陀螺仪

  //初始化电机PID和编码器
  initEncoders();//初始化编码器中断库对象
  initMotorController();//初始化电机控制引脚
  initPID();//初始化PID控制器
  resetPID();
  
}

void loop() {
  while (Serial.available() > 0) {//直接接收串口的命令
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[index0] = NULL;
      else if (arg == 2) argv2[index0] = NULL;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index0] = NULL;
        arg = 2;
        index0 = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index0] = chr;
        index0++;
      }
      else if (arg == 2) {
        argv2[index0] = chr;
        index0++;
      }
    }
  }
  if (millis() > nextPID) {//执行pid更新函数
    updatePID();
    nextPID += PID_INTERVAL;
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {//转动指定的时间
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}

