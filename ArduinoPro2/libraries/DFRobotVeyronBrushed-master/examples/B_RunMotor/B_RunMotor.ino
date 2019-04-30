/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 After run the workflow like PID autoTune and direction detection.
 You can run this code to control your motor
 
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
 3.If the motor doesn't run, please run the C_CheckLink example.
 ****************************************************/

#include "DFRobotVeyronBrushed.h"

DFRobotVeyronBrushed VeyronBrushed;

void setup()
{
  Serial1.begin(57600);
  VeyronBrushed.begin(Serial1);
}

void loop()
{
  int id = 1;             //The ID of the Veyron
  int motorNumber = 1;    //The motor number: 1 for M1 and 2 for M2
  int speed;              //Store the speed of the motor in RPM (Revolutions Per Minute)
  
  speed=80;               //80 RPM
  VeyronBrushed.setSpeed(id, motorNumber, speed);//Run the motor 1 in 80 RPM
  delay(2000);
  
  speed=0;                //0 RPM
  VeyronBrushed.setSpeed(id, motorNumber, speed);//Stop the motor 1
  delay(2000);
  
  speed=-80;                //-80 RPM
  VeyronBrushed.setSpeed(id, motorNumber, speed);//Run the motor 1 revesely in 80 RPM
  delay(2000);
  
  speed=0;                //0 RPM
  VeyronBrushed.setSpeed(id, motorNumber, speed);//Stop the motor 1
  delay(2000);
}
