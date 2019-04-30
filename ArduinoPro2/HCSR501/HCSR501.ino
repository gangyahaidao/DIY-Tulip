void setup() {
  pinMode(2, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  int ret = digitalRead(2);
  if(ret == HIGH) {
    Serial.println("--has people");
  } else {
    Serial.println("--no people");  
  }
  delay(100);
}
