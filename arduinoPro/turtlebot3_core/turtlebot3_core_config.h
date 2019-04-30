#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <math.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

//frame head
#define FRAME_HEAD_1    0x55
#define FRAME_HEAD_2    0xAA
//frame end
#define FRAME_END       0x0A

//frame type
typedef enum{
  MSG_ODOM = 1,
  MSG_SPEED = 2,
  MSG_BATTERY = 3,
  MSG_ACCE  = 4,
  MSG_WHEEL  = 5,
  MSG_MOVECTRL = 0x81,
  MSG_SERVOCTRL = 0x82
}msg_type;

typedef struct
{
  long counter;
  int count;
  char dir;
  char encoder_a;
  char encoder_b;
}Wheel;

typedef struct
{
  long counter1;
  long counter2;
  long counter3;
}SRobotWheelData;

typedef struct
{

    int odom_motor1;
    int odom_motor2;
    int odom_motor3;

}SRobotOdomData;

typedef struct
{

    float v_motor1;
    float v_motor2;
    float v_motor3;

}SRobotSpeedData;

typedef struct
{

    float v_motor1;
    float v_motor2;
    float v_motor3;

}RRobotData;

typedef struct
{

float voltage;
float current;
float charge;

}SRobotBatteryData;

typedef struct
{

float acx;
float acy;
float acz;

}SRobotAcceData;

typedef struct
{

float gyx;
float gyy;
float gyz;

}SRobotGyroData;

typedef struct
{

float yaw;
float pitch;
float roll;

}SRobotYPRData;

#endif // TURTLEBOT3_CORE_CONFIG_H_
