#include <NewPing.h>
#include "Sonar.h"

#define SONAR_NUM     6 // Number of sensors.
#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 35 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(22, 23, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(24, 25, MAX_DISTANCE),
  NewPing(26, 27, MAX_DISTANCE),
  NewPing(28, 29, MAX_DISTANCE),
  NewPing(30, 31, MAX_DISTANCE),
  NewPing(32, 33, MAX_DISTANCE),  
};
SonarDistance sonarDistance;//创建sonar结构体

void setup() {
  Serial.begin(115200);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // Other code that *DOESN'T* analyze ping results can go here.
  
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

byte temp_buffer[4 + 4*6];
byte send_buffer[4 + 4*6 + 6];//四字节头部+数据
void oneSensorCycle() { // 将数据上传到上位机
  byte copy_index = 0;
  byte temp_copy_index = 0;    
  temp_buffer[0] = 0xcd;
  temp_buffer[1] = 0xeb;
  temp_buffer[2] = 0xd7;
  temp_buffer[3] = 4*6+6;
  sonarDistance.sonar1 = cm[3];
  sonarDistance.sonar2 = cm[4];
  sonarDistance.sonar3 = cm[5];
  sonarDistance.sonar4 = cm[0];
  sonarDistance.sonar5 = cm[1];
  sonarDistance.sonar6 = cm[2];
  memcpy(&temp_buffer[4], (byte*)&sonarDistance, 4*6);
  //增加空格
  memcpy(&send_buffer[copy_index], &temp_buffer[temp_copy_index], 4);
  copy_index += 4;
  temp_copy_index += 4;
  for (int i = 1; i <= 6; i++) {
    memcpy(&send_buffer[copy_index], &temp_buffer[temp_copy_index], 4);
    copy_index += 4;
    temp_copy_index += 4;
    send_buffer[copy_index] = 0x20;//添加空格
    copy_index += 1;
  }
  Serial.write(send_buffer, 4+4*6+6);
  
  /*for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();*/
}
