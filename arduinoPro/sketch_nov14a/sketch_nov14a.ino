void setup() {
  // put your setup code here, to run once:
  pinMode(9, INPUT);
  Serial.begin(115200);
  while (!Serial) {;}
}

void loop() {
  // put your main code here, to run repeatedly:
  int ret = digitalRead(9);
  Serial.println(ret);
  delay(200);
}
