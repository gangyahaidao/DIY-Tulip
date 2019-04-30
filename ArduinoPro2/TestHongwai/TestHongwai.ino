int Sensor_pin = 3;
int Buzzerpin = 7;

void Beep()
{
  digitalWrite(Buzzerpin, HIGH);  
  delay(1);
  digitalWrite(Buzzerpin, LOW);
  delay(1);
}
void setup()
{
  pinMode(Sensor_pin, INPUT); //设置人体红外接口为输入状态
  pinMode(Buzzerpin, OUTPUT); //设置蜂鸣器接口为输出状态
  Serial.begin(115200);
}
void loop()
{
  int val = digitalRead(Sensor_pin); //定义参数存储人体红外传感器读到的状态
  Serial.print(millis());
  Serial.print(':');
  Serial.println(val);
  if (val == 1) //如果检测到有动物运动（在检测范围内），蜂鸣器发出警报
  {
    Beep();
  }
}
