void blink()//中断函数
{
  Serial.println("interr");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setup");
  attachInterrupt(0, blink, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

}
