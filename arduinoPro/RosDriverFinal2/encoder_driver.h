#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#define  ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define ENCODER_LEFT_A    21   //int2
#define ENCODER_LEFT_B    20   //int3
#define ENCODER_RIGHT_A   19   //int4
#define ENCODER_RIGHT_B   18   //int5

#define LEFT              0
#define RIGHT             1

void testEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#endif

