/**
 ****************************************************************************
 * @file        IMU_arduinoSketch.ino
 * @brief       Sketch for the Seeed Studio's XIAO nRF52840 Sense IMU
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  kamilo.melo@km-robota.com, 10/2023
 * @authors  katarina.lichardova@km-robota.com, 10/2023
 * @authors  melike.cezayirlioglu@km-robota.com, 10/2023
 ****************************************************************************
 */

#include "LSM6DS3.h"
#include "Wire.h"

// Acc: lin acc (duh). If sudden dash towards 1 direction: big peak in the movement's direction (acc),
//      then huge of opposite sign (huge decc to bring to stop)
// Gyro: angular velocity


float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// DEBUG
int ctr = 1;

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

void setup() {

    // Open serial port
    Serial.begin(230400);
    while (!Serial);


    if (myIMU.begin() != 0) {
        //Serial.println("Device error");
    }
}


void loop() {
  accX =  myIMU.readFloatAccelX();
  accY =  myIMU.readFloatAccelY();
  accZ =  myIMU.readFloatAccelZ();

  gyroX = myIMU.readFloatGyroX();
  gyroY = myIMU.readFloatGyroY();
  gyroZ = myIMU.readFloatGyroZ(); 

  // Debug: hardcoding values
  /*accX =  1;
  accY =  2;
  accZ =  3;
  gyroX = 4;
  gyroY = 5;
  gyroZ = 6;*/

  // DEBUG 2 testing
  /*accX =  ctr;
  accY =  ctr;
  accZ =  ctr;
  gyroX = ctr;
  gyroY = ctr;
  gyroZ = ctr;
  ctr++;

  if (ctr > 30) 
    ctr = 1; */

  // Send values through serial
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

  //delayMicroseconds(50);
}













