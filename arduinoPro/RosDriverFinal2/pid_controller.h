#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define MAX_SPEED 550.0  // mm/s

#include <PID_v1.h>
typedef struct {
  double TargetTicksPerFrame;    // 大于0的目标速度 mm/s
  double input;                    //1/UPDATE秒计算出的速度 mm/s
  double output;                   // pwm值
  long Encoder;                  //电机当前的转脉冲数
  long PrevEnc;                  //电机前一帧的脉冲数
  int PrevInput;
  bool dir;//true正传  false反转
} SetPointInfo;

void initPID();
void resePID();
void doPID();
void updatePIDSpeed();

#endif

