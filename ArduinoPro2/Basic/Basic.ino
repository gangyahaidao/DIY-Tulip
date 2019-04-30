#define ENCODER_LEFT_A_PIN    2//中断0引脚
#define ENCODER_LEFT_B_PIN    3
#define ENCODER_RIGHT_A_PIN   4//中断1引脚
#define ENCODER_RIGHT_B_PIN   5

#define LEFT              0
#define RIGHT             1

typedef struct{
  long counter;//编码器计数值
  int count;//帧计数，用来计算旋转的角度
  char dir;//旋转方向
  char encoder_a_pin;//编码器A引脚
  char encoder_b_pin;//编码器B引脚
}Wheel;

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
Wheel wheelL, wheelR;//定义左轮和右轮相关的变量的结构体对象

void initEncoders() {
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP); //int0 as left encoder A
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP); //as left encoder B
  //pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);//int1 as right encoder A
  //pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);//as right encoder B
  attachInterrupt(0, encoderLeftISR, FALLING);  //pin 2
  //attachInterrupt(1, encoderRightISR, RISING);//pin 3
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
  if (HIGH == digitalRead(3)) { // int 1
    wheelL.counter--;
    wheelL.dir = 1;//后退为1
    Serial.println("L--");
  } else {
    wheelL.counter++;
    wheelL.dir = 0;//前进为0
    Serial.println("L++");
  }
}

void encoderRightISR() {
  wheelL.count++;
  if (HIGH == digitalRead(18)) { //int 3
    wheelR.counter--;
    wheelR.dir = 1;
    Serial.println("R--");
  } else {
    wheelR.counter++;
    wheelR.dir = 0;
    Serial.println(right_enc_pos);
  }
}

void setup()
{
  initEncoders();
  Serial.begin(115200); //setup our serial 初始化Arduino串口
}

void loop() {
  Serial.println(wheelL.counter);
  delay(100);
}
