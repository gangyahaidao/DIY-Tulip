#include "IMU.h"

/* 最终数据，存储解析之后的值 */
int16_t AccRaw[3];  /* 原始化速度 */
int16_t GyoRaw[3];
float Eular[3];   /* 欧拉角 */
float IMU[9];

static Packet_t RxPkt;

/* CRC16 */
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
  uint32_t crc = *currectCrc;
  uint32_t j;
  for (j = 0; j < lengthInBytes; ++j)
  {
    uint32_t i;
    uint32_t byte = src[j];
    crc ^= byte << 8;
    for (i = 0; i < 8; ++i)
    {
      uint32_t temp = crc << 1;
      if (crc & 0x8000)
      {
        temp ^= 0x1021;
      }
      crc = temp;
    }
  }
  *currectCrc = crc;
}

/* packet decode */
uint32_t Packet_Decode(uint8_t c)
{
  static uint16_t CRCReceived = 0; /* CRC value received from a frame */
  static uint16_t CRCCalculated = 0;  /* CRC value caluated from a frame */
  static uint8_t status = kStatus_Idle; /* state machine */
  static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

  switch (status)
  {
    case kStatus_Idle:
      if (c == 0x5A)
        status = kStatus_Cmd;
      break;
    case kStatus_Cmd:
      if (c == 0xA5)
        status = kStatus_LenLow;
      break;
    case kStatus_LenLow:
      RxPkt.payload_len = c;
      crc_header[2] = c;
      status = kStatus_LenHigh;
      break;
    case kStatus_LenHigh:
      RxPkt.payload_len |= (c << 8);
      crc_header[3] = c;
      status = kStatus_CRCLow;
      break;
    case kStatus_CRCLow:
      CRCReceived = c;
      status = kStatus_CRCHigh;
      break;
    case kStatus_CRCHigh:
      CRCReceived |= (c << 8);
      RxPkt.ofs = 0;
      CRCCalculated = 0;
      status = kStatus_Data;
      break;
    case kStatus_Data:
      RxPkt.buf[RxPkt.ofs++] = c;
      if (RxPkt.ofs >= RxPkt.payload_len)
      {
        /* calculate CRC */
        crc16_update(&CRCCalculated, crc_header, 4);
        crc16_update(&CRCCalculated, RxPkt.buf, RxPkt.ofs);

        /* CRC match */
        if (CRCCalculated == CRCReceived)
        {
          /* 成功接收到一帧数据，开始显示 */
          DispData(&RxPkt);
        }
        status = kStatus_Idle;
      }
      break;
    default:
      status = kStatus_Idle;
      break;
  }
}

float yaw = 0;
/* 成功解析到一帧数据后的处理 */
void DispData(Packet_t *pkt)
{
  if(pkt->buf[0] == kItemAccRaw)  /* Acc raw value */
  {
    IMU[0] = ((float)(int16_t)(pkt->buf[1] + (pkt->buf[2]<<8)))*9.8/1000; //转换成m/s2
    IMU[1] = ((float)(int16_t)(pkt->buf[3] + (pkt->buf[4]<<8)))*9.8/1000;
    IMU[2] = ((float)(int16_t)(pkt->buf[5] + (pkt->buf[6]<<8)))*9.8/1000;
  }
  
  if(pkt->buf[7] == kItemGyoRaw)  /* gyro raw value */
  {
    IMU[3] = (float)(int16_t)(pkt->buf[8] + (pkt->buf[9]<<8))/10.0;
    IMU[4] = (float)(int16_t)(pkt->buf[10] + (pkt->buf[11]<<8))/10.0;
    IMU[5] = (float)(int16_t)(pkt->buf[12] + (pkt->buf[13]<<8))/10.0;
  }
  if(pkt->buf[14] == kItemAtdE)  /* atd E */
  {
    IMU[6] = ((float)(int16_t)(pkt->buf[15] + (pkt->buf[16]<<8)))/100.0;
    IMU[7] = ((float)(int16_t)(pkt->buf[17] + (pkt->buf[18]<<8)))/100.0;
    yaw = ((float)(int16_t)(pkt->buf[19] + (pkt->buf[20]<<8)))/10.0;
    if (yaw < 0) { //-180~0  -->  180~360
      yaw = 360 + yaw;
    }
    IMU[8] = yaw;
  }
  
  /* 打印姿态角 */
  /*Serial.print(AccRaw[0]);
  Serial.print("; ");
  Serial.print(AccRaw[1]);
  Serial.print("; ");
  Serial.println(AccRaw[2]);*/
  
  /*Serial.print("Pitch:");
  Serial.print(Eular[0]);
  Serial.print("  ");
  Serial.print("Roll:");
  Serial.print(Eular[1]);
  Serial.print("  ");

  Serial.print("Yaw:");
  Serial.print(Eular[2]);
  Serial.print("  ");
  Serial.print("\r\n");*/
}
