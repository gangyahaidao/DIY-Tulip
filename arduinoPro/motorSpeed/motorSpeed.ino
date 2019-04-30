#define MOTOR1 2

long motor1Step = 0;
void setup() {
  // put your setup code here, to run once:
  //pinMode(MOTOR1, INPUT);
  Serial.begin(115200);
  attachInterrupt(0, motor1_int_func, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  int ret = digitalRead(MOTOR1);
  if(ret == HIGH){
    Serial.println("HIGH level");  
  }else if(ret == LOW){
    Serial.println("LOW level"); 
  }
  delay(800);
}

void motor1_int_func(){
   motor1Step++;
   Serial.print("motor1Step = ");
   Serial.println(motor1Step);
}
