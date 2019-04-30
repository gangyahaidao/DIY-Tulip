/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 This example factory Reset all the Veyron devices connected on the UART bus.
 
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
 And change Serial1 to Serial
 ****************************************************/

#include "DFRobotVeyronBrushed.h"

DFRobotVeyronBrushed VeyronBrushed;

void setup()
{
  Serial1.begin(115200);
  VeyronBrushed.begin(Serial1);
  
  //The value of BROADCAST_ID is "0".
  //All the Veyron devices connected on the UART bus will accept the command.
  int id = BROADCAST_ID;
  
  //use all kinds of baudrate to reset the Veyron.
  unsigned long baudrateArray[10]={1200,2400,4800,9600,14400,19200,28800,38400,57600,115200};
  
  for (int baudrateIndex=0; baudrateIndex<10; baudrateIndex++) {
    Serial1.begin(baudrateArray[baudrateIndex]);    //begin the Serial1 use different Baudrate
    delay(50);
    VeyronBrushed.factoryReset(id);     //factory Reset
    Serial1.end();
    delay(50);
  }
}

void loop()
{
  
}
