/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

//-CUSTOM BIAIS-------------------------------------------
float gyroBiais[3] = { 0.01 , 0.04 , -0.00 };
float accelBiais[3] = { 0.12 , 0.08 , 0.54 };
float accelScales[3] = { 1.00 , 1.00 , 0.99 };
float magBiais[3] = { 7.56 , 44.27 , -42.80 };
float magScale[3] = { 0.99 , 0.99 , 1.03 };
//---------------------------------------------------------


void setup() {
  // serial to display data
  Serial.begin(9600);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  //calibrate();
  //printBiais();
  printBiais();
  loadBias();
  printBiais();
  
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  printDatas();
  delay(100);
}

void printDatas(){
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
  
}

void printBiais(){

 Serial.println("//-CUSTOM BIAIS-------------------------------------------");
   Serial.print("float gyroBiais[3] = { ");Serial.print(IMU.getGyroBiasX_rads());Serial.print(" , ");Serial.print(IMU.getGyroBiasY_rads());Serial.print(" , ");Serial.print(IMU.getGyroBiasZ_rads());Serial.println(" };");

  Serial.print("float accelBiais[3] = { ");Serial.print(IMU.getAccelBiasX_mss());Serial.print(" , ");Serial.print(IMU.getAccelBiasY_mss());Serial.print(" , ");Serial.print(IMU.getAccelBiasZ_mss());Serial.println(" };");
  Serial.print("float accelScales[3] = { ");Serial.print(IMU.getAccelScaleFactorX());Serial.print(" , ");Serial.print(IMU.getAccelScaleFactorY());Serial.print(" , ");Serial.print(IMU.getAccelScaleFactorZ());Serial.println(" };");
  Serial.print("float magBiais[3] = { ");Serial.print(IMU.getMagBiasX_uT());Serial.print(" , ");Serial.print(IMU.getMagBiasY_uT());Serial.print(" , ");Serial.print(IMU.getMagBiasZ_uT());Serial.println(" };");
  Serial.print("float magScale[3] = { ");Serial.print(IMU.getMagScaleFactorX());Serial.print(" , ");Serial.print(IMU.getMagScaleFactorY());Serial.print(" , ");Serial.print(IMU.getMagScaleFactorZ());Serial.println(" };");
  Serial.println("//---------------------------------------------------------");

  
}



void calibrate(){
  Serial.println("\nStarting Gyro Calibration");
  Serial.println("leave the object flat");
  IMU.calibrateGyro();

Serial.println("\nStarting Accelerometer Calibration");
  IMU.calibrateAccel();
  Serial.println("Switch Axe");
  delay(5000);
  IMU.calibrateAccel();
  Serial.println("Switch Axe");
  delay(5000);
  IMU.calibrateAccel();
  Serial.println("Switch Axe");
  delay(5000);
  IMU.calibrateAccel();
  Serial.println("Switch Axe");
  delay(5000);
  IMU.calibrateAccel();
  Serial.println("Switch Axe");
  delay(5000);
  IMU.calibrateAccel();
  Serial.println("Done");
  Serial.println("Starting Magnetometer Calibration");
  Serial.println("move with a 8 shape");
  delay(4000);
  // calibrating magnetometer
  IMU.calibrateMag();
  Serial.println("Done\n");

  printBiais();
}


void loadBias(){
IMU.setGyroBiasX_rads(gyroBiais[0]);
IMU.setGyroBiasY_rads(gyroBiais[1]);
IMU.setGyroBiasZ_rads(gyroBiais[2]);
IMU.setAccelCalX(accelBiais[0],accelScales[0]);
IMU.setAccelCalY(accelBiais[1],accelScales[1]);
IMU.setAccelCalZ(accelBiais[2],accelScales[2]);
IMU.setMagCalX(magBiais[0],magScale[0]);
IMU.setMagCalY(magBiais[1],magScale[1]);
IMU.setMagCalZ(magBiais[2],magScale[2]);
}
