#include "common.h"
#include "Timer.h"
#include "PWMController.h"
#include <math.h>
#include <Servo.h> 

#define LED     13
#define PI 3.1415926
#define RAD2DEG(x) (int)(x/PI*180)
#define MOVE_TIME 300

#define RECV_BYTES_LEN 8
int match_flag = 0;
byte inChar;//用于缓存一帧的有用数据
byte match[4];//存储三个字节头部
boolean stringComplete = false;
byte serial_cnt; //global
byte recv_index = 0;
byte speed_buffer[RECV_BYTES_LEN];//前变量用于接收，后变量用于计算
void serialEvent() {//接收上位机发送的控制命令
  if (Serial.available()) {
    if (match_flag == 0) {//接收第一个消息头
      match[0] = (unsigned char)Serial.read();
      if (match[0] == 0xcd) {
        match_flag = 1;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 1) {//接收第二个消息头
      match[1] = (unsigned char)Serial.read();
      if (match[1] == 0xeb) {
        match_flag = 2;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 2) {//接收第三个消息头
      match[2] = (unsigned char)Serial.read();
      if (match[2] == 0xd7) {
        match_flag = 3;
        return;
      } else {
        match_flag = 0;
        return;
      }
    }
    if (match_flag == 3) {//接收第四个字节,有用数据长度
      serial_cnt = (unsigned char)Serial.read();
      match_flag = 4;
      return;
    }
    if (match_flag == 4) {
      inChar = (unsigned char)Serial.read();//接收帧数据有用字节
      speed_buffer[recv_index] = inChar;
      recv_index++;//接收的数据长度
      serial_cnt--;
      if (serial_cnt <= 0) {
        stringComplete = true;
        recv_index = 0;
        serial_cnt = 0;
        match_flag = 0;
        return;
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(LED,OUTPUT);  
  digitalWrite(LED, LOW);
  InitTimer2();  
  InitPWM();  
}

void loop() {
  TaskTimeHandle();//每20ms调用一次舵机
  
  //1.串口接收到一帧数据
  if (stringComplete) {
    ServoSetPluseAndTime(0, map(speed_buffer[1], 0, 180, 500, 2500), MOVE_TIME);
    ServoSetPluseAndTime(1, map(speed_buffer[0], 0, 180, 500, 2500), MOVE_TIME);
    /*Serial.print("x_center: ");
    Serial.print(personPos.x_center);
    Serial.print(", y_center: ");
    Serial.println(personPos.y_center); */
    stringComplete = false;
  }
}
