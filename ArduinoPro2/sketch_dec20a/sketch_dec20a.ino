#define ENCODER_LEFT_A_PIN    2//中断引脚
#define ENCODER_LEFT_B_PIN    3
#define ENCODER_RIGHT_A_PIN   21//中断引脚
#define ENCODER_RIGHT_B_PIN   20

#define LEFT              0
#define RIGHT             1

typedef struct{
  long counter;//编码器计数值
  int count;//帧计数，用来计算旋转的角度
  char dir;//旋转方向
  char encoder_a_pin;//编码器A引脚
  char encoder_b_pin;//编码器B引脚
}Wheel;

void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

Wheel wheelL, wheelR;//定义左轮和右轮相关的变量的结构体对象

void initEncoders() {
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP); //int0 as left encoder A
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP); //as left encoder B
  pinMode(ENCODER_RIGHT_A_PIN, INPUT);//int1 as right encoder A
  pinMode(ENCODER_RIGHT_B_PIN, INPUT);//as right encoder B
  attachInterrupt(0, encoderLeftISR, RISING);  //pin 2
  attachInterrupt(2, encoderRightISR, RISING);//pin 21
  //初始化轮子结构体
  wheelL.counter = 0;
  wheelL.count = 0;
  wheelL.dir = 0;
  wheelL.encoder_a_pin = ENCODER_LEFT_A_PIN;
  wheelL.encoder_b_pin = ENCODER_LEFT_B_PIN;
  wheelR.counter = 0;
  wheelR.count = 0;
  wheelR.dir = 0;
  wheelR.encoder_a_pin = ENCODER_RIGHT_A_PIN;
  wheelR.encoder_b_pin = ENCODER_RIGHT_B_PIN;
}

void encoderLeftISR() {
  wheelL.count++;
  if (HIGH == digitalRead(ENCODER_LEFT_B_PIN)) { // int 3
    wheelL.counter--;
    //Serial.println("L--");
  } else {
    wheelL.counter++;
    //Serial.println("L++");
  }
  Serial.print("L:");
  Serial.println(wheelL.counter);
}

void encoderRightISR() {
  wheelL.count++;
  if (HIGH == digitalRead(ENCODER_RIGHT_B_PIN)) { //int 20
    wheelR.counter--;
    //Serial.println("R--");
  } else {
    wheelR.counter++;
    //Serial.println("R++");
  }
  Serial.print("R:");
  Serial.println(wheelR.counter);
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  initEncoders();//初始化编码器中断库对象
  while(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(1);
}
