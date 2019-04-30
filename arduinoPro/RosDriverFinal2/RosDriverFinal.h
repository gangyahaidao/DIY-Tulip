#ifndef ROSDRIVERFINAL_H
#define ROSDRIVERFINAL_H

#define FRAME_HEAD_1      0xcd
#define FRAME_HEAD_2      0xeb
#define FRAME_HEAD_3      0xd7

//数据帧格式：消息头 + 内容长度 + 内容，每个内容数据是四个字节，中间使用空格0x20分隔
typedef struct {
    long status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【9 13】v
    float theta;//方位角，【0 360】°
    long encoder_ppr;//车轮1转对应的编码器个数
    long encoder_delta_r;//右轮编码器增量， 个为单位
    long encoder_delta_l;//左轮编码器增量， 个为单位
    long encoder_delta_car;//两车轮中心位移，个为单位
    long omga_r;//右轮转速 个每秒
    long omga_l;//左轮转速 个每秒
    long bump_front_left_flag;//左前方碰撞传感器标志位，为LOW低电平时候表示触发碰撞
    long bump_front_right_flag;//右前方
    long bump_back_one_flag;//后方
    long bump_wheel_left_flag;//左轮旁边的防碰撞传感器
    long bump_wheel_right_flag;
    float IMU[9];//mpu9250 9轴数据  acc angular oula
    unsigned long time_stamp;//时间戳
}UPLOAD_STATUS; 

#endif
