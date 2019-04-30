#include "common.h"
#include "Timer.h"
#include "PWMController.h"
#include <math.h>
#include <ros.h>
#include <rosrobot_description/ArmJointState.h>
#include <Servo.h> 

#define LED     13
#define PI 3.1415926
#define RAD2DEG(x) (int)(x/PI*180)
#define MOVE_TIME 1200

int joint_step[6] = {0};
int recv_step[6] = {0};
char joint_status = 0;
ros::NodeHandle nh;
void arm_cb(const rosrobot_description::ArmJointState& cmd_arm){  
  digitalWrite(LED, HIGH);  
  recv_step[5] = cmd_arm.position1;//肩部的坐标
  recv_step[4] = cmd_arm.position2;
  recv_step[3] = cmd_arm.position3;
  recv_step[2] = cmd_arm.position4;
  recv_step[1] = cmd_arm.position5;
  recv_step[0] = cmd_arm.position6;

  if(recv_step[5] == 0){//肩部
    joint_step[5] = 500;
  }else{
    joint_step[5] = abs(recv_step[5]);
  }  

  if(recv_step[4] == 0){
    joint_step[4] = 500;
  }else{
    joint_step[4] = abs(recv_step[4]);
  }  

  if(recv_step[3] == 0){
    joint_step[3] = 1500;
  }else{
    joint_step[3] = 90 - recv_step[3];
  }  

  if(recv_step[2] == 0){
    joint_step[2] = 1500;
  }else{
    joint_step[2] = 90 - recv_step[2];
  }  

  if(recv_step[1] == 0){
    joint_step[1] = 1500;
  }else{
    joint_step[1] = 90 - recv_step[1];
  }  

  if(recv_step[0] == 0){
    joint_step[0] = 1500;
  }else{
    joint_step[0] = 45 - recv_step[0];
  }  
  
  joint_status = 1;
}
/*void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}*/
//订阅手臂六个舵机的旋转角度
ros::Subscriber<rosrobot_description::ArmJointState> arm_sub("/joint_steps",arm_cb); // /joint_states   subscribes to joint_steps on arm
//订阅钳子的旋转角度
//ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position

void setup() {
  Serial.begin(57600);
  pinMode(LED,OUTPUT);  
  digitalWrite(LED, LOW);
  InitTimer2();
  InitPWM();  
  //初始化ros订阅器
  nh.initNode();
  nh.subscribe(arm_sub);
  //nh.subscribe(gripper_sub);
}

void loop() {
  TaskTimeHandle();//每20ms调用一次舵机
  
  if(joint_status == 1){
    for(int i = 0; i < 6; i++){
      ServoSetPluseAndTime(i, map(joint_step[i], 0, 180, 500, 2500), MOVE_TIME);//转变成us表示的角度范围,并设置时间
    }       
    joint_status = 0;
  }  
  nh.spinOnce();
}
