/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 This is the library for Veyron Brushed motor controller.
 See the example for details.
 
 Created 2014-8-28
 By Angelo qiao <Angelo.qiao@dfrobot.com>
 Modified 2014-8-29
 By Angelo qiao Angelo.qiao@dfrobot.com>
 Modified 2014-9-30
 By Angelo qiao Angelo.qiao@dfrobot.com>
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <>
 2.This code is tested on Arduino Leonardo, Mega boards.
 If you have to use the UNO, please set a 1K pull-down resister on RX pin (D0 pin) of the Uno.
 ****************************************************/

#ifndef DFRobotVeyronBrushed_h
#define DFRobotVeyronBrushed_h

#include "Arduino.h"


#define DIRECTION_MODE_UNIDIRECTION (0)
#define DIRECTION_MODE_BIDIRECTION (1)

#define H_BRIDGE_MODE_HPWM_LON (0)
#define H_BRIDGE_MODE_HPWM_LPWM_ON_OFF (1)
#define H_BRIDGE_MODE_HPWM_LPWM_ON_OFF_ON (2)

#define DEFAULT_TIME_OUT_DURATION 500

#define RECEIVED_BUFFER_SIZE (40)
#define SEND_BUFFER_SIZE (40)

#define BROADCAST_ID (0)

#define RECEIVED_BUFFER_FULL (RECEIVED_BUFFER_SIZE-1)
#define RECEIVED_BUFFER_TIME_OUT (0xff)

class DFRobotVeyronBrushed {
  
  char _receivedBuffer[RECEIVED_BUFFER_SIZE];
  //  uint8_t _receivedBufferLength;
  
  Stream* _serial;
  
  char* parseFrame();
  
  unsigned long _timeOutDuration;
  unsigned long _timeOutTimer;
  
  void sendFrame();
  
  void sendFrame(uint8_t id, const char* command_P);
  void sendFrame(uint8_t id, const char* command_P, int content1);
  void sendFrame(uint8_t id, const char* command_P, int content1, int content2);
  void sendFrame(uint8_t id, const char* command_P, int content1, int content2, int content3);
  
  void sendFrame(uint8_t id, const char* command_P, float content1, float content2);
  
  char* floatToString(float theFloat, char* theBuffer, const char* thePostfix_P);
  
  char* intToString(int theInt, char* theBuffer, const char* thePostfix_P);
  
  void beginTransmission(uint8_t id);
  void write(const __FlashStringHelper* command);
  void write(int content);
  void write(long content);
  void write(float content);
  void endTransmission();
  
  
public:
  DFRobotVeyronBrushed();
  
  void begin(Stream& theSerial);
  
  void setTimeOut(unsigned long theTimeOutDuration);
  
  char* readType(uint8_t id);
  void readConfig(uint8_t id, Stream& theSerial=Serial);
  void factoryReset(uint8_t id);
  void readHelp(uint8_t id, Stream& theSerial=Serial);
  void storeConfig(uint8_t id);
  void setBaudrate(uint8_t id, unsigned long theBaudrate);
  void setNewId(uint8_t theNewId);
  char* readEncoderPulses(uint8_t id, uint8_t theMotorNumber);
  void clearEncoderPulses(uint8_t id, uint8_t theMotorNumber);
  void setEncoderPulsePerRevolution(uint8_t id, int theEncoderPulsePerRevolutionM1, int theEncoderPulsePerRevolutionM2);
  void setGearRatio(uint8_t id, float theGearRatioM1, float theGearRatioM2);
  
  void setDirectionMode(uint8_t id, uint8_t theDirectionMode);
  void setHBridgeMode(uint8_t id, uint8_t theHBridgeMode);
  void setSlowStartMode(uint8_t id, boolean theSlowStartMode);
  
  char* readSpeed(uint8_t id, uint8_t theMotorNumber);
  char* readMaxSpeed(uint8_t id, uint8_t theMotorNumber);
  
  void setSpeed(uint8_t id, uint8_t theMotorNumber, int theSpeed);
  
  void setConditionalSpeed(uint8_t id, uint8_t theMotorNumber, int theConditionalSpeed);
  
  void startConditionalSpeed(uint8_t id, boolean isStartConditionalSpeedM1,  boolean isStartConditionalSpeedM2);
  
  char* readAcceleration(uint8_t id, uint8_t theMotorNumber);
  
  char* readDeacceleration(uint8_t id, uint8_t theMotorNumber);
  
  void setAcceleration(uint8_t id, uint8_t theMotorNumber, int theAcceleration, int theDeacceleration);
  
  char* readMaxPPM(uint8_t id);
  
  char* readMinPPM(uint8_t id);
  
  void setPPMRange(uint8_t id, int theMinPPM, int theMaxPPM);
  
  char* readCyclicalSignalMode(uint8_t id);
  
  void setCyclicalSignalMode(uint8_t id, boolean isCyclicalSignalMode);
  void setCyclicalSignalModeTimeOut(uint8_t id, int theCyclicalSignalModeTimeOut);
  
  char* readKp(uint8_t id, uint8_t theMotorNumber);
  char* readTi(uint8_t id, uint8_t theMotorNumber);
  char* readTd(uint8_t id, uint8_t theMotorNumber);
  
  void setPID(uint8_t id, uint8_t theMotorNumber, float theKp, float theTi, float theTd);
  
  char* readMaxContinuousCurrent(uint8_t id);
  char* readMaxPeakyCurrent(uint8_t id);
  char* readCurrent(uint8_t id, uint8_t theMotorNumber);
  
  void setMaxCurrent(uint8_t id, unsigned long theMaxContinuousCurrent, unsigned long theMaxPeakyCurrent);
  
  char* readMaxTemperature(uint8_t id);
  char* readTemperature(uint8_t id);
  void setMaxTemperature(uint8_t id, int theMaxTemperature);
  
  char* readVoltage(uint8_t id);
  void setVoltageRange(uint8_t id, float theMinVoltage, float theMaxVoltage);
  
  char* readError(uint8_t id);
  void clearError(uint8_t id);
  
  void startSpeedMonitor(uint8_t id, uint8_t theMotorNumber);
  void endSpeedMonitor();
};



#endif
