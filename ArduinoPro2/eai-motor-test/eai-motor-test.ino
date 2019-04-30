void setup() {
  // put your setup code here, to run once:
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(6, 0);
}
