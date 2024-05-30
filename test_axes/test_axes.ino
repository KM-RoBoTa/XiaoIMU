/*****************************************************************************/
//  HighLevelExample.ino
//  Hardware:      Grove - 6-Axis Accelerometer&Gyroscope
//	Arduino IDE:   Arduino-1.65
//	Author:	       Lambor
//	Date: 	       Oct,2015
//	Version:       v1.0
//
//  Modified by:
//  Data:
//  Description:
//
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include "LSM6DS3.h"
#include "Wire.h"


// Acc: lin acc (duh). If sudden dash towards 1 direction: big peak in the movement's direction (acc), then huge of opposite sign (huge decc to bring to stop)
// Gyro: angular velocity

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

void setup() {
    // put your setup code here, to run once:
    Serial.begin(230400);
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        //Serial.println("Device error");
    }
}

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// DEBUG
int ctr = 1;

void loop() {
  accX =  myIMU.readFloatAccelX();
  accY =  myIMU.readFloatAccelY();
  accZ =  myIMU.readFloatAccelZ();

  gyroX = myIMU.readFloatGyroX();
  gyroY = myIMU.readFloatGyroY();
  gyroZ = myIMU.readFloatGyroZ(); 


  // DEBUG testing
  /*accX =  ctr;
  accY =  ctr;
  accZ =  ctr;
  gyroX = ctr;
  gyroY = ctr;
  gyroZ = ctr;
  ctr++;

  if (ctr > 30) 
    ctr = 1; */

  // DEbug 2
  /*accX =  1;
  accY =  2;
  accZ =  3;
  gyroX = 4;
  gyroY = 5;
  gyroZ = 6;*/


  // Send values to the plotter
  Serial.print('\1'); // start of heading SOH
  Serial.print(accX);
  Serial.print(","); 
  Serial.print(accY);
  Serial.print(","); 
  Serial.print(accZ);
  Serial.print(","); 
  Serial.print(gyroX);
  Serial.print(","); 
  Serial.print(gyroY);
  Serial.print(","); 
  Serial.println(gyroZ);
  //Serial.println('\0');

  // Only accs
  /*Serial.print(accX);
  Serial.print(","); 
  Serial.print(accY);
  Serial.print(","); 
  Serial.println(accZ);*/


  // Only gyro
  /*Serial.print(gyroX);
  Serial.print(","); 
  Serial.print(gyroY);
  Serial.print(","); 
  Serial.println(gyroZ);*/

  //delayMicroseconds(50);
}













