#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#endif
