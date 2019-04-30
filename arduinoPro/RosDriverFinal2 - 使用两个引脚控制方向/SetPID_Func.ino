#include "SetPID_Func.h"

extern double Kp;
extern double Ki;
extern double Kd;
extern PID myPIDL, myPIDR;

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
      Serial1.println("PID OK");
      break;
    default:
      Serial1.println("Invalid Command");
      break;
  }
}

void receive_setPID(){
  while (Serial1.available() > 0) {//串口1接收设置PID命令
    chr = Serial1.read();
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
}
