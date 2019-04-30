/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 This demo is only the explanation of every function and command.
 It is not a good idea to be uploaded to arduino.
 
 Created 2014-8-28
 By Angelo qiao <Angelo.qiao@dfrobot.com>
 Modified 2014-8-29
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
 3.This Example is the first step of the cofiguration of the Veyron brushed motor controller.
 ****************************************************/

#include "DFRobotVeyronBrushed.h"
#include <stdlib.h>

DFRobotVeyronBrushed VeyronBrushed;

void setup()
{
  Serial1.begin(57600);
  VeyronBrushed.begin(Serial1);
  
  Serial.begin(115200);
  
  int id = 1;   //id address of the Veyron
  int motorNumber = 1;     //show which motor is selected. 1 for M1 and 2 for M2.
  int speed = 50;   //the speed of the motor

  
  //set the serial time Out Duration to 500ms
  unsigned long timeOutDuration = 500;
  VeyronBrushed.setTimeOut(timeOutDuration);

  //read the type of Veyron.
  String type = VeyronBrushed.readType(id);
  
  //read the configuration. use Serial to print the result
  VeyronBrushed.readConfig(id, Serial);
  
  //factory reset
  VeyronBrushed.factoryReset(id);
  
  //read help. use Serial to print the result
  VeyronBrushed.readHelp(id, Serial);

  //After configuration changed, let Veyron to store it.
  VeyronBrushed.storeConfig(id);
  
  //set the baudrate of serial.
  unsigned long baudrate = 57600;
  VeyronBrushed.setBaudrate(id, baudrate);
  
  //set the new id.
  int newID = 1;
  VeyronBrushed.setNewId(newID);
  
  //read the accumulate number of the Encoder Pulses
  long encoderPulses = atoi(VeyronBrushed.readEncoderPulses(id, motorNumber));
  
  //clear the accumulate number of the Encoder Pulses
  VeyronBrushed.clearEncoderPulses(id, motorNumber);
  
  //set the Pulse Per Revolution of the encoder
  int encoderPulsePerRevolutionM1 = 13; //Your Pulse Per Revolution of the encoder M1
  int encoderPulsePerRevolutionM2 = 13; //Your Pulse Per Revolution of the encoder M2
  VeyronBrushed.setEncoderPulsePerRevolution(id, encoderPulsePerRevolutionM1, encoderPulsePerRevolutionM2);
  
  //set the gear ratio
  float gearRatioM1 = 51.0;   //Your gear ratio of the motor on M1
  float gearRatioM2 = 51.0;   //Your gear ratio of the motor on M2
  VeyronBrushed.setGearRatio(id, gearRatioM1, gearRatioM2);
  
  //set direction mode. UNIDIRECTION or BIDIRECTION
  int directionMode = DIRECTION_MODE_BIDIRECTION;
  /***************
  int directionMode = DIRECTION_MODE_UNIDIRECTION;
   ***************/
  VeyronBrushed.setDirectionMode(id, directionMode);

  //set the h Bridge Mode
  int hBridgeMode = H_BRIDGE_MODE_HPWM_LPWM_ON_OFF_ON;
  /***************
  int hBridgeMode = H_BRIDGE_MODE_HPWM_LON;
  int hBridgeMode = H_BRIDGE_MODE_HPWM_LPWM_ON_OFF;
   ***************/
  VeyronBrushed.setHBridgeMode(id, hBridgeMode);

  //whether to use slow start mode on ppm and analog mode
  boolean isSlowStartMode = true;
  /***************
  boolean isSlowStartMode = false;
   ***************/
  VeyronBrushed.setSlowStartMode(id, isSlowStartMode);

  //read current speed of motor
  long currentSpeed = atoi( VeyronBrushed.readSpeed(id, motorNumber));
  
  //read max speed of motor
  long maxSpeed = atoi(VeyronBrushed.readMaxSpeed(id, motorNumber));
  
  //set the speed of motor
  VeyronBrushed.setSpeed(id, motorNumber, speed);
  
  //set conditional speed
  VeyronBrushed.setConditionalSpeed(id, motorNumber, speed);

  //use conditional speed to run the motor
  boolean isStartConditionalSpeedM1 = true;
  boolean isStartConditionalSpeedM2 = true;
  VeyronBrushed.startConditionalSpeed(id, isStartConditionalSpeedM1,  isStartConditionalSpeedM2);
  
  //read Acceleration
  long acceleration = atoi(VeyronBrushed.readAcceleration(id, motorNumber));
  
  //read deacceleration
  long deacceleration = atoi(VeyronBrushed.readDeacceleration(id, motorNumber));
  
  //set Acceleration and Deacceleration
  VeyronBrushed.setAcceleration(id, motorNumber, acceleration, deacceleration);
  
  //read max PPM settings
  long maxPPM = atoi(VeyronBrushed.readMaxPPM(id));
  
  //read min PPM settings
  long minPPM = atoi(VeyronBrushed.readMinPPM(id));
  
  //set PPM range
  VeyronBrushed.setPPMRange(id, minPPM, maxPPM);

  //read Cyclical Signal Mode
  boolean isCyclicalSignalMode = atoi(VeyronBrushed.readCyclicalSignalMode(id));
  
  //set Cyclical Signal Mode
  VeyronBrushed.setCyclicalSignalMode(id, isCyclicalSignalMode);
  
  //set Cyclical Signal Mode time out duration
  unsigned long CyclicalSignalModeTimeOut = 1000;   //set time out to 1000ms
  VeyronBrushed.setCyclicalSignalModeTimeOut(id, CyclicalSignalModeTimeOut);
  
  //read PID
  float Kp = atof(VeyronBrushed.readKp(id, motorNumber));
  float Ti = atof(VeyronBrushed.readTi(id, motorNumber));
  float Td = atof(VeyronBrushed.readTd(id, motorNumber));
  
  //set PID
  VeyronBrushed.setPID(id, motorNumber, Kp, Ti, Td);

  //read max Continuous Current
  long maxContinuousCurrent = atoi(VeyronBrushed.readMaxContinuousCurrent(id));
  
  //read max Reaky Current
  long maxReakyCurrent = atoi(VeyronBrushed.readMaxPeakyCurrent(id));
  
  //read Current of each motor
  long current = atoi(VeyronBrushed.readCurrent(id, motorNumber));
  
  //set max current
  VeyronBrushed.setMaxCurrent(id, maxContinuousCurrent, maxReakyCurrent);

  //read max temperature
  long maxTemperature = atoi(VeyronBrushed.readMaxTemperature(id));
  
  //read temperature
  long temperature = atoi(VeyronBrushed.readTemperature(id));
  
  //set max warining tempature
  VeyronBrushed.setMaxTemperature(id, maxTemperature);

  //read voltage unit: V
  float voltage = atoi(VeyronBrushed.readVoltage(id)) / 10.0;
  
  //set voltage range unit: V
  int minVoltage = 10.5;
  int maxVoltage = 24.3;
  VeyronBrushed.setVoltageRange(id, minVoltage, maxVoltage);
  
  //read error
  String error = VeyronBrushed.readError(id);

  //clear error
  VeyronBrushed.clearError(id);

  //Start speed monitor for PID tunning
  VeyronBrushed.startSpeedMonitor(id, motorNumber);

  //end speed monitor
  VeyronBrushed.endSpeedMonitor();

}

void loop()
{
  
}
