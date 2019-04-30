#include "common.h"
#include "Timer.h"
#include "PWMController.h"
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>

#define LED     8
#define BUZZER  9
#define KEY     10
#define PI 3.1415926
#define RAD2DEG(x) (int)(x/PI*180)

int joint_step[6] = {0};
int convert_step[6] = {0};
char joint_status = 0;
ros::NodeHandle nh;
std_msgs::Int16 msg;
void arm_cb(const sensor_msgs::JointState& cmd_arm){
  joint_status = 1;
  joint_step[0] = cmd_arm.position[0];
  joint_step[1] = cmd_arm.position[1];
  joint_step[2] = cmd_arm.position[2];
  joint_step[3] = cmd_arm.position[3];
  joint_step[4] = cmd_arm.position[4];
  joint_step[5] = cmd_arm.position[5]; //gripper position <0-180>
}
void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}
//订阅手臂五个舵机的旋转角度
ros::Subscriber<sensor_msgs::JointState> arm_sub("/move_group/fake_controller_joint_states",arm_cb); //subscribes to joint_steps on arm
//订阅钳子的旋转角度
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER,OUTPUT);
  pinMode(LED,OUTPUT);
  InitPWM();
  InitTimer2();
  //初始化ros订阅器
  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
}

void loop() {
  TaskTimeHandle();//每20ms调用一次舵机
  if(joint_state == 1){
    for(int i = 0; i < 6; i++){
      convert_step[i] = map(joint_step[i], 1000, 2000, 0, 180);  
      ServoSetPluseAndTime(i, convert_step[i], 3000);
    }       
    joint_status = 0;
  }  
  nh.spinOnce();
}
