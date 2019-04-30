#include <Servo.h>
char buffer[10];
Servo servo1;
Servo servo2;

void setup() {
  // put your setup code here, to run once:
  servo1.attach(5);
  servo2.attach(6);
  Serial.begin(115200);
  Serial.flush();
  servo1.write(90);
  servo2.write(90);
  Serial.println("Starting...");
}

String receiveStr;
char ch;
void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
      ch = (char)Serial.read();
      if(ch == 'x'){
        receiveStr += ch;
        while(Serial.available() > 0){
          ch = (char)Serial.read();
          if(ch == ';'){
            processCmd(receiveStr);
            receiveStr = "";
          }else{
            receiveStr += ch;  
          }
        }        
      }
  }
}

void processCmd(String str){
  int len=str.length()+1;
  char buf[len];
  str.toCharArray(buf, len);
  char* parameter = strtok(buf, ",");
  while(parameter != NULL){
    setServo(parameter);
    parameter = strtok(NULL, ",");
  }    
}

void setServo(char* data){
  if((data[0] == 'x')){
    int firstVal = strtol(data+1, NULL, 10);
    firstVal = map(firstVal, 0, 640, 0, 180);
    servo1.write(firstVal);
  }
  if((data[0] == 'y')){
    int firstVal = strtol(data+1, NULL, 10);
    firstVal = map(firstVal, 0, 480, 0, 180);
    servo1.write(firstVal);
  }
}
