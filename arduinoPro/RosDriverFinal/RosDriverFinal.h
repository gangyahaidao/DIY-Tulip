#ifndef ROSDRIVERFINAL_H
#define ROSDRIVERFINAL_H

#define FRAME_HEAD_1      0x55
#define FRAME_HEAD_2      0xAA
#define FRAME_END         0x0A

typedef enum { //发送到上位机的数据类型标识
  MSG_ODOM = 1,
  MSG_SPEED = 2,
  MSG_BATTERY = 3,
  MSG_ACCE  = 4,
  MSG_WHEEL  = 5,
  MSG_MOVECTRL = 0x81,
  MSG_SERVOCTRL = 0x82
} msg_type;

typedef struct { //发送odom电机编码器的值
  int odom_motor1;
  int odom_motor2;
} SRobotOdomData;

typedef struct { //发送电机当前速度
  float v_motor1;
  float v_motor2;
} SRobotSpeedData;

typedef struct { //接收电机的目标速度
  float v_motor1;
  float v_motor2;
} RRobotData;

typedef struct { //电池状态
  float voltage;
  float current;
  float charge;
} SRobotBatteryData;

typedef struct { //加速度信息
  float acx;
  float acy;
  float acz;
} SRobotAcceData;

typedef struct { //角速度信息
  float gyx;
  float gyy;
  float gyz;
} SRobotGyroData;

typedef struct { //角度信息
  float yaw;
  float pitch;
  float roll;
} SRobotYPRData;

#endif
