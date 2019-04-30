/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 This example show how to set new id address for Veyron device
 
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
  
  int newId = 1;                 //The new id address of Veyron you want to set.
  int broadcastId = BROADCAST_ID;   //The Broadcast id for store the cofiguration.
  
  VeyronBrushed.setNewId(newId);
  VeyronBrushed.storeConfig(broadcastId);  //Let Veyron to store the cofiguration.
}

void loop()
{
  
}
