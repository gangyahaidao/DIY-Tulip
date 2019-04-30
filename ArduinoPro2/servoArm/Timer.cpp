#include "common.h"
#include "PWMController.h"

u32 gSystemTickCount = 0;  //系统从启动到现在的毫秒数

void InitTimer2(void)    //100us@12.000MHz
{
  TCCR2A = 0;
  TCCR2B = _BV(CS21) | _BV(CS20);
  TIMSK2 = _BV(TOIE2);
  TCNT2 = 206; //(256-206)/500000=100us
  sei();
}

ISR(TIMER2_OVF_vect)
{
  TCNT2 = 206; //定时器3中断  100us
  static uint16 time = 0;
  if (++time >= 10) {
    time = 0;
    gSystemTickCount++;
  }
}

void TaskTimeHandle(void) {
  static uint32 time = 10;
  static uint32 times = 0;
  if (gSystemTickCount > time) {
    time += 10;
    times++;
    if (times % 2 == 0) //20ms
    {
      ServoPwmDutyCompare();
    }
  }
}
