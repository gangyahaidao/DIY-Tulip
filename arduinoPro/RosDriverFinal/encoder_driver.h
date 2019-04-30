#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#define ENCODER_LEFT_A_PIN    2//中断0引脚
#define ENCODER_LEFT_B_PIN    4
#define ENCODER_RIGHT_A_PIN   3//中断1引脚
#define ENCODER_RIGHT_B_PIN   5

#define LEFT              0
#define RIGHT             1

typedef struct{
  long counter;//编码器计数值
  int count;//帧计数，用来计算旋转的角度
  char dir;//旋转方向
  char encoder_a_pin;//编码器A引脚
  char encoder_b_pin;//编码器B引脚
}Wheel;

void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#endif
