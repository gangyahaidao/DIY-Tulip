#define Phase_A 2
#define Phase_B 3

unsigned long time;
long Position = 0;
boolean A = false;
boolean B = false;

void setup()
{
  pinMode(Phase_A, INPUT);//PULL_UP avoid signal interference
  pinMode(Phase_B, INPUT);

  attachInterrupt(2, Interrupt_A, CHANGE);//LOW,CHANGE,RISING,FALLING available.
  attachInterrupt(3, Interrupt_B, CHANGE);//LOW,CHANGE,RISING,FALLING available.
  Serial.begin(115200); //setup our serial 初始化Arduino串口
}

void loop() {
  Serial.println(Position);
  delay(100);
}

void Interrupt_A() {
  if (A == false) {
    if ( B == false) {
      Position = Position + 1;
      A == true;
    }
    else {
      Position = Position - 1;
      A == true;
    }
  }
  else {
    if ( B == false) {
      Position = Position - 1;
      A == false;
    }
    else {
      Position = Position + 1;
      A == false;
    }
  }

}
void Interrupt_B() {
  if (B == false) {
    if ( A == false) {
      Position = Position - 1;
      B == true;
    }
    else {
      Position = Position + 1;
      A == true;
    }
  }
  else {
    if ( A == false) {
      Position = Position + 1;
      B == false;
    }
    else {
      Position = Position - 1;
      B == false;
    }
  }
}
