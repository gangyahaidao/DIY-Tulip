#include "common.h"
#include "PWMController.h"

u32 gSystemTickCount = 0;  //系统从启动到现在的毫秒数
u32 preTimeCount = 0;

void flash(){
  gSystemTickCount++;
}

void InitTimer2(void){
    FlexiTimer2::set(1, 1.0/1000, flash);//1毫秒调用一次
    FlexiTimer2::start();
}

void TaskTimeHandle(void) {
  if(gSystemTickCount - preTimeCount >= 20){    
    preTimeCount = gSystemTickCount;
    ServoPwmDutyCompare();
  }
}
