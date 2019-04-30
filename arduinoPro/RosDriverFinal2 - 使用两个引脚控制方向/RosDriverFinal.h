#ifndef ROSDRIVERFINAL_H
#define ROSDRIVERFINAL_H

#define FRAME_HEAD_1      0xcd
#define FRAME_HEAD_2      0xeb
#define FRAME_HEAD_3      0xd7

//数据帧格式：消息头 + 内容长度 + 内容，每个内容数据是四个字节，中间使用空格0x20分隔
typedef struct {
    int status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【9 13】v
    float theta;//方位角，【0 360】°
    int encoder_ppr;//车轮1转对应的编码器个数
    int encoder_delta_r;//右轮编码器增量， 个为单位
    int encoder_delta_l;//左轮编码器增量， 个为单位
    int encoder_delta_car;//两车轮中心位移，个为单位
    int omga_r;//右轮转速 个每秒
    int omga_l;//左轮转速 个每秒
    /*暂时不使用超声波
    float distance1;//第一个超声模块距离值 单位cm
    float distance2;//第二个超声模块距离值 单位cm
    float distance3;//第三个超声模块距离值 单位cm
    float distance4;//第四个超声模块距离值 单位cm
    */
    float IMU[9];//mpu9250 9轴数据
    unsigned int time_stamp;//时间戳
}UPLOAD_STATUS; 

#endif
