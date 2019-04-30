String comdata = "";
float gAngle;
float gRate;
float gX_acc;
float gY_acc;
float gZ_acc;
byte recv_array[15];//存储接收的字节
byte tmp0, tmp1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}
void loop() {  
  if(Serial1.available() > 0){
    tmp0 = Serial1.read();    
    if(tmp0 == 0xAA){
      delay(2);
      tmp1 = Serial1.read();
      if(tmp1 == 0x00){//说明数据是需要接收处理的
        recv_array[0] = tmp0;
        recv_array[1] = tmp1;
        //直接读取剩余13个字节到缓存数组中
        Serial1.readBytes(recv_array+2, 13);
        parse_data(recv_array);//调用函数进行处理        
      }
    }
  }
}

bool parse_data(uint8_t* data_string){
  uint8_t index;
  uint16_t angle;
  uint16_t rate;
  uint16_t x_acc;
  uint16_t y_acc;
  uint16_t z_acc;
  uint8_t check_sum;
  
  index = data_string[2];
  rate = (data_string[3] & 0xFF) | ((data_string[4] << 8) & 0xFF00);  
  angle = (data_string[5] & 0xFF) | ((data_string[6] << 8) & 0XFF00);  
  x_acc = (data_string[7] & 0xFF) | ((data_string[8] << 8) & 0xFF00);  
  y_acc = (data_string[9] & 0xFF) | ((data_string[10] << 8) & 0XFF00);  
  z_acc = (data_string[11] & 0xFF) | ((data_string[12] << 8) & 0xFF00);  
  //reserved = data_string[13];  
  //Verify checksum    
  check_sum = data_string[2] + data_string[3] + data_string[4] + data_string[5]   
    + data_string[6] + data_string[7] + data_string[8] + data_string[9]   
    + data_string[10] + data_string[11] + data_string[12] + data_string[13];    
  if(check_sum != data_string[14]){  
    Serial.println("Checksum mismatch error");
    return false;
  }  
  //Scale and store data   
  gRate = rate / 100.0; 
  gAngle = angle / 100.0; 
  gX_acc = x_acc; 
  gY_acc = y_acc; 
  gZ_acc = z_acc; 
  print_result();
  return true;
}

void print_result(){
  Serial.print("gAngle = ");
  Serial.print(gAngle);
  Serial.print("; gRate = ");
  Serial.print(gRate);
  Serial.print("; gX_acc = ");
  Serial.print(gX_acc);
  Serial.print("; gY_acc = ");
  Serial.print(gY_acc);
  Serial.print("; gZ_acc = ");
  Serial.println(gZ_acc);
}
