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

#include "DFRobotVeyronBrushed.h"
#include <pgmspace.h>

DFRobotVeyronBrushed::DFRobotVeyronBrushed()
{
  _timeOutDuration=DEFAULT_TIME_OUT_DURATION;
}

void DFRobotVeyronBrushed::begin(Stream& theSerial)
{
  _serial = &theSerial;
}

void DFRobotVeyronBrushed::setTimeOut(unsigned long theTimeOutDuration)
{
  _timeOutDuration=theTimeOutDuration;
}

char* DFRobotVeyronBrushed::parseFrame()
{
  uint8_t receivedBufferIndex=0;
  
  _timeOutTimer=millis();
  while (1) {
    if (_serial->available()) {
      _receivedBuffer[receivedBufferIndex]= _serial->read();
      if (_receivedBuffer[receivedBufferIndex]=='\n') {
        _receivedBuffer[receivedBufferIndex+1] = 0;
        break;
      }
      receivedBufferIndex++;
      if (receivedBufferIndex==(RECEIVED_BUFFER_SIZE-1)) {
        strcpy_P(_receivedBuffer, PSTR("Buffer Full!\r\n"));
        break;
      }
      
      _timeOutTimer=millis();
    }
    else{
      if (millis()-_timeOutTimer>=_timeOutDuration) {
        strcpy_P(_receivedBuffer, PSTR("Time Out!\r\n"));
        break;
      }
    }
  }
  
  return _receivedBuffer;
}

char* DFRobotVeyronBrushed::readType(uint8_t id)
{
  while(Serial.read() != -1);
  _serial->println();
  _serial->print(id);
  _serial->print(F(",PING"));
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::readConfig(uint8_t id, Stream& theSerial)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",RCONFIG"));
  _serial->println();
  
  _timeOutTimer=millis();
  while (1) {
    if (_serial->available()) {
      theSerial.write(_serial->read());
      
      if (!_serial->available()) {
        _timeOutTimer=millis();
      }
    }
    else{
      if (millis()-_timeOutTimer> _timeOutDuration) {
        theSerial.println(F("Read Config Finished\r\n"));
        break;
      }
    }
  }
}
void DFRobotVeyronBrushed::factoryReset(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",FCONFIG"));
  _serial->println();
}

void DFRobotVeyronBrushed::readHelp(uint8_t id, Stream& theSerial)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",HELP"));
  _serial->println();
  
  _timeOutTimer=millis();
  while (1) {
    if (_serial->available()) {
      theSerial.write(_serial->read());
      
      if (!_serial->available()) {
        _timeOutTimer=millis();
      }
    }
    else{
      if (millis()-_timeOutTimer> _timeOutDuration) {
        theSerial.println(F("Read Help Finished\r\n"));
        break;
      }
    }
  }
}

void DFRobotVeyronBrushed::storeConfig(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",EEPSAV"));
  _serial->println();
}

void DFRobotVeyronBrushed::setBaudrate(uint8_t id, unsigned long theBaudrate)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SBR,"));
  _serial->print(theBaudrate);
  _serial->println();
}
void DFRobotVeyronBrushed::setNewId(uint8_t theNewId)
{
  _serial->println();
  _serial->print(BROADCAST_ID);
  _serial->print(F(",SNA,"));
  _serial->print(theNewId);
  _serial->println();
}
char* DFRobotVeyronBrushed::readEncoderPulses(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GEP,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}
void DFRobotVeyronBrushed::clearEncoderPulses(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",CEP,"));
  _serial->print(theMotorNumber);
  _serial->println();
}
void DFRobotVeyronBrushed::setEncoderPulsePerRevolution(uint8_t id, int theEncoderPulsePerRevolutionM1, int theEncoderPulsePerRevolutionM2)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SER,"));
  _serial->print(theEncoderPulsePerRevolutionM1);
  _serial->print(',');
  _serial->print(theEncoderPulsePerRevolutionM2);
  _serial->println();
}
void DFRobotVeyronBrushed::setGearRatio(uint8_t id, float theGearRatioM1, float theGearRatioM2)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SGR,"));
  _serial->print(theGearRatioM1);
  _serial->print(',');
  _serial->print(theGearRatioM2);
  _serial->println();
}

void DFRobotVeyronBrushed::setDirectionMode(uint8_t id, uint8_t theDirectionMode)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SDIR,"));
  _serial->print(theDirectionMode);
  _serial->println();
}
void DFRobotVeyronBrushed::setHBridgeMode(uint8_t id, uint8_t theHBridgeMode)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SPWM,"));
  _serial->print(theHBridgeMode);
  _serial->println();
}
void DFRobotVeyronBrushed::setSlowStartMode(uint8_t id, boolean theSlowStartMode)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SSD,"));
  _serial->print(theSlowStartMode);
  _serial->println();
}

char* DFRobotVeyronBrushed::readSpeed(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GVC,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}
char* DFRobotVeyronBrushed::readMaxSpeed(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GVM,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::setSpeed(uint8_t id, uint8_t theMotorNumber, int theSpeed)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SVS,"));
  _serial->print(theMotorNumber);
  _serial->print(',');
  _serial->print(theSpeed);
  _serial->println();
}

void DFRobotVeyronBrushed::setConditionalSpeed(uint8_t id, uint8_t theMotorNumber, int theConditionalSpeed)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SVR,"));
  _serial->print(theMotorNumber);
  _serial->print(',');
  _serial->print(theConditionalSpeed);
  _serial->println();
}

void DFRobotVeyronBrushed::startConditionalSpeed(uint8_t id, boolean isStartConditionalSpeedM1,  boolean isStartConditionalSpeedM2)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",AVR,"));
  _serial->print(isStartConditionalSpeedM1);
  _serial->print(',');
  _serial->print(isStartConditionalSpeedM2);
  _serial->println();
}

char* DFRobotVeyronBrushed::readAcceleration(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GACC,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}

char* DFRobotVeyronBrushed::readDeacceleration(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GDEC,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::setAcceleration(uint8_t id, uint8_t theMotorNumber, int theAcceleration, int theDeacceleration)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SAD,"));
  _serial->print(theMotorNumber);
  _serial->print(',');
  _serial->print(theAcceleration);
  _serial->print(',');
  _serial->print(theDeacceleration);
  _serial->println();
}

char* DFRobotVeyronBrushed::readMaxPPM(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GPA"));
  _serial->println();
  
  return parseFrame();
}

char* DFRobotVeyronBrushed::readMinPPM(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GPI"));
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::setPPMRange(uint8_t id, int theMinPPM, int theMaxPPM)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SPPM,"));
  _serial->print(theMinPPM);
  _serial->print(',');
  _serial->print(theMaxPPM);
  _serial->println();
}

char* DFRobotVeyronBrushed::readCyclicalSignalMode(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GSM"));
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::setCyclicalSignalMode(uint8_t id, boolean isCyclicalSignalMode)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SSM,"));
  _serial->print(isCyclicalSignalMode);
  _serial->println();
}
void DFRobotVeyronBrushed::setCyclicalSignalModeTimeOut(uint8_t id, int theCyclicalSignalModeTimeOut)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SDCT,"));
  _serial->print(theCyclicalSignalModeTimeOut/10);
  _serial->println();
}

char* DFRobotVeyronBrushed::readKp(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GKP,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}
char* DFRobotVeyronBrushed::readTi(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GTI,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}
char* DFRobotVeyronBrushed::readTd(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GTD,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::setPID(uint8_t id, uint8_t theMotorNumber, float theKp, float theTi, float theTd)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SPID,"));
  _serial->print(theMotorNumber);
  _serial->print(',');
  _serial->print(theKp);
  _serial->print(',');
  _serial->print(theTi);
  _serial->print(',');
  _serial->print(theTd);
  _serial->println();
}

char* DFRobotVeyronBrushed::readMaxContinuousCurrent(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GMCC"));
  _serial->println();
  
  return parseFrame();
}
char* DFRobotVeyronBrushed::readMaxPeakyCurrent(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GMPC"));
  _serial->println();
  
  return parseFrame();
}

char* DFRobotVeyronBrushed::readCurrent(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GCC,"));
  _serial->print(theMotorNumber);
  _serial->println();
  
  return parseFrame();
}


void DFRobotVeyronBrushed::setMaxCurrent(uint8_t id, unsigned long theMaxContinuousCurrent, unsigned long theMaxPeakyCurrent)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SMLC,"));
  _serial->print(theMaxContinuousCurrent);
  _serial->print(',');
  _serial->print(theMaxPeakyCurrent);
  _serial->println();
}

char* DFRobotVeyronBrushed::readMaxTemperature(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GMT"));
  _serial->println();
  
  return parseFrame();
}
char* DFRobotVeyronBrushed::readTemperature(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GCT"));
  _serial->println();
  
  return parseFrame();
}
void DFRobotVeyronBrushed::setMaxTemperature(uint8_t id, int theMaxTemperature)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SMT,"));
  _serial->print(theMaxTemperature);
  _serial->println();
}

char* DFRobotVeyronBrushed::readVoltage(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GSV"));
  _serial->println();
  
  return parseFrame();
}
void DFRobotVeyronBrushed::setVoltageRange(uint8_t id, float theMinVoltage, float theMaxVoltage)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",SLV,"));
  _serial->print(theMinVoltage);
  _serial->print(',');
  _serial->print(theMaxVoltage);
  _serial->println();
}

char* DFRobotVeyronBrushed::readError(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",GER"));
  _serial->println();
  
  return parseFrame();
}

void DFRobotVeyronBrushed::clearError(uint8_t id)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",CER"));
  _serial->println();
}

void DFRobotVeyronBrushed::startSpeedMonitor(uint8_t id, uint8_t theMotorNumber)
{
  _serial->println();
  _serial->print(id);
  _serial->print(F(",OUT,"));
  _serial->print(theMotorNumber);
  _serial->println();
}
void DFRobotVeyronBrushed::endSpeedMonitor()
{
  _serial->println();
}
