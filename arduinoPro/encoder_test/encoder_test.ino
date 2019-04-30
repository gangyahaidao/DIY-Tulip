#include <quadrature.h>

Quadrature_encoder<0,1> encoder(Board::due);

void setup() {
  Serial.begin(115200);  
  encoder.begin();
//  encoder.reverse();
}

void loop() {
  //read the encoder
  long ct = encoder.count();
  
  //format the output for printing 
  char buf[50];
  sprintf(buf, "enc1 count is: %d", ct);
  Serial.println(buf);
  
  //check the motion
  Motion::motion m = encoder.motion();
  Serial.println(text(m));

  delay(100);
}
