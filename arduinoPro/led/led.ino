#define LED 13
#define MYPIN1 21
#define MYPIN2 22
#define MYPIN3 23
#define MYPIN4 24
#define MYPIN5 25
#define MYPIN6 26
#define MYPIN7 27
#define MYPIN8 28
#define MYPIN9 29
#define MYPIN10 30
#define MYPIN11 31
#define MYPIN12 32
#define MYPIN13 33
#define MYPIN14 34
#define MYPIN15 35
#define MYPIN16 36
int leftCount, middleCount, rightCount;
int motor1Count = 0;
void Motor1InitFunc(){
  motor1Count++;
  Serial1.println("motor1Count1 = " + motor1Count);  
}
void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(MYPIN1, INPUT_PULLUP);
  pinMode(MYPIN2, INPUT_PULLUP);
  pinMode(MYPIN3, INPUT_PULLUP);
  pinMode(MYPIN4, INPUT_PULLUP);
  pinMode(MYPIN5, INPUT_PULLUP);
  pinMode(MYPIN6, INPUT_PULLUP);
  pinMode(MYPIN7, INPUT_PULLUP);
  pinMode(MYPIN8, INPUT_PULLUP);
  pinMode(MYPIN9, INPUT_PULLUP);
  pinMode(MYPIN10, INPUT_PULLUP);
  pinMode(MYPIN11, INPUT_PULLUP);
  pinMode(MYPIN12, INPUT_PULLUP);
  pinMode(MYPIN13, INPUT_PULLUP);
  pinMode(MYPIN14, INPUT_PULLUP);
  pinMode(MYPIN15, INPUT_PULLUP);
  pinMode(MYPIN16, INPUT_PULLUP);
  Serial.begin(115200);
  attachInterrupt(2, Motor1InitFunc, LOW);
  Serial.println("attachInterrupt");
}
void writeStringToSerial(String data){
  int len=data.length()+1;
  char buf[len];
  data.toCharArray(buf, len);
  Serial.write(buf);
  Serial.flush();
}
//turn 
void turnLeft(int turnSpeedLeft, int turnSpeedRight){
  int len = 0;
  char buf[64] = {0};
  String str = "";
  str = str + "1,SVR,1," + turnSpeedRight + "\r\n";
  writeStringToSerial(str);  
  str = "";
  str = str + "1,SVR,2," + turnSpeedLeft + "\r\n";
  writeStringToSerial(str);
  str = "";
  str += "1,AVR,1,1\r\n";
  writeStringToSerial(str);  
  }
//turn right
void turnRight(int turnSpeedLeft, int turnSpeedRight){
  String str = "";
  str = str + "1,SVR,1," + turnSpeedRight + "\r\n";
  writeStringToSerial(str);
  str = "";
  str = str + "1,SVR,2," + turnSpeedLeft + "\r\n";
  writeStringToSerial(str);
  str = "";
  str = str + "1,AVR,1,1\r\n";
  writeStringToSerial(str);
  }
void moveStraight(int leftSpeed, int rightSpeed){
  String str = "";
  str = str + "1,SVR,1," + leftSpeed + "\r\n";
  writeStringToSerial(str);
  str = "";
  str = str + "1,SVR,2," + rightSpeed + "\r\n";
  writeStringToSerial(str);
  str = "";
  str += "1,AVR,1,1\r\n";
  writeStringToSerial(str);
  }
void moveBack(int leftSpeed, int rightSpeed){
  String str = "";
  str = str + "1,SVR,1," + leftSpeed + "\r\n";
  writeStringToSerial(str);
  str = "";
  str = str + "1,SVR,2," + rightSpeed + "\r\n";
  writeStringToSerial(str);
  str = "";
  str += "1,AVR,1,1\r\n";
  writeStringToSerial(str);
}  
void stopGo(){
  String str = "";
  str += "1,SVR,1,0\r\n";
  writeStringToSerial(str);
  str = "";
  str += "1,SVR,2,0\r\n";
  writeStringToSerial(str);
  str = "";
  str += "1,AVR,1,1\r\n";
  writeStringToSerial(str);
  }
    
int getLightedPinCount(int from, int to){
  int count = 0;
  for(int i = from; i <= to; i++){
    if(digitalRead(i+21) == LOW){
      count++;
      }
    }
  return count;
  }  

void loop() {    
  
//  leftCount = getLightedPinCount(1, 5);
//  middleCount = getLightedPinCount(6, 11);
//  rightCount = getLightedPinCount(12, 16);
//  if(middleCount < 4){
//    if(leftCount > 0){
//      turnRight(500, 0);
//      while(leftCount > 0){        
//        leftCount = getLightedPinCount(1, 5);
//      }
//      moveStraight(500, 500);           
//    }else if(rightCount > 0){
//      turnLeft(0, 500);
//      while(rightCount > 0){        
//        rightCount = getLightedPinCount(12, 16);
//      }
//      moveStraight(500, 500);                  
//    }
//  }else if(middleCount >= 4){
//    moveStraight(500, 500);
//    while(middleCount >= 4){
//      middleCount = getLightedPinCount(6, 11);
//    }
//  }
//  if(middleCount == 0 && leftCount == 0 && rightCount == 0){
//    moveBack(-500, -500);
//    while(middleCount == 0 && leftCount == 0 && rightCount == 0){
//        leftCount = getLightedPinCount(1, 5);
//        middleCount = getLightedPinCount(6, 11);
//        rightCount = getLightedPinCount(12, 16);
//    }
//  }
}
