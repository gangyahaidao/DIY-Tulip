/***************************************************
 Veyron Brushed motor controller
 <http://www.dfrobot.com/index.php?route=product/product&path=48&product_id=912#.UniMMJH7k8M>
 
 ***************************************************
 This example check whether the serial communication is working.
 
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
 ****************************************************/

#include "DFRobotVeyronBrushed.h"

DFRobotVeyronBrushed VeyronBrushed;

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  
  Serial1.begin(57600);
  VeyronBrushed.begin(Serial1);
}

void loop()
{
  int id = 1;       //The id of the Veyron module
  String VeyronType = VeyronBrushed.readType(id);   //Read the type of the Veyron
  Serial.println();
  Serial.print(VeyronType);
  
  //Open the Serial monitor.
  if (VeyronType.startsWith("BDC")) {
    //If "BDC" is received, the connection is working.
    Serial.println("Connection Good!");
  }
  else{
    //If "Time Out!" is received, please check the connection.
    Serial.println("Connection Wrong!");
    Serial.println("Recheck the electrical connection?");
    Serial.println("Change the baudrate?");
    Serial.println("Change the ID?");
    Serial.println("This is not DFRobot Veyron Brushed motor controller?");
  }

  delay(2000);
}
