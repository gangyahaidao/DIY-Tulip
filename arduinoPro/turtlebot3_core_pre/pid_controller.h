#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <PID_v1.h>
typedef struct {
  double TargetTicksPerFrame;    // 一帧的目标脉冲数
  double input;                    //1/30秒内转的脉冲数
  double output;                   // pwm值
  long Encoder;                  //电机当前的转脉冲数
  long PrevEnc;                  //电机前一帧的脉冲数
  int PrevInput;
} SetPointInfo;

void initPID();
void resetPID();
void doPID();
void updatePID();
long readPidIn(int i);
long readPidOut(int i);

#endif
