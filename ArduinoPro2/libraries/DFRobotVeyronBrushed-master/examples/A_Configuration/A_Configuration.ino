/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 This example config the parameter of Gear ratio and encoder
 so that workflow like PID autoTune and direction detection can be done by Veyron.
 
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

DFRobotVeyronBrushed VeyronBrushed;

void setup()
{
  Serial1.begin(57600);
  VeyronBrushed.begin(Serial1);
  
  int id = 1;             //The default id address of the Veyron
  
  //Check the datasheet of motor to get the right configuration
  //The following configuration is based on this datasheet:
  //<http://www.dfrobot.com/image/data/FIT0277/mechanical.jpg>
  
  float gearRatioM1 = 51.0;   //Your gear ratio of the motor on M1
  float gearRatioM2 = 51.0;   //Your gear ratio of the motor on M2
  VeyronBrushed.setGearRatio(id, gearRatioM1, gearRatioM2);  //set the gear ratio
  
  int encoderPulsePerRevolutionM1 = 13; //Your Pulse Per Revolution of the encoder M1
  int encoderPulsePerRevolutionM2 = 13; //Your Pulse Per Revolution of the encoder M2
  //set the Pulse Per Revolution of the encoder
  VeyronBrushed.setEncoderPulsePerRevolution(id, encoderPulsePerRevolutionM1, encoderPulsePerRevolutionM2);
  
  VeyronBrushed.storeConfig(1);  //Let Veyron to store the cofiguration.
}

void loop()
{
  
}
