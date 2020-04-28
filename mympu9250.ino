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
#include "quaternionFilters.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

float pitch, yaw, roll;
float ax, ay, az, gx, gy, gz, mx, my, mz;

float avrpitch, avryaw, avrroll;


uint32_t count = 0, sumCount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

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

  printBiais();
  loadBias();
  printBiais();
  
}

void loop() {
  // read the sensor
  IMU.readSensor();
  GetYRP();
  // printIMU();
  printAVRYRP();
  delay(100);
}

void printIMU(){
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

void printYRP(){
Serial.print("YRP: ");
Serial.print(yaw);
Serial.print("\t");
Serial.print(roll);
Serial.print("\t");
Serial.println(pitch);
}

void printAVRYRP(){
Serial.print("AVRYRP: ");
Serial.print(avryaw);
Serial.print("\t");
Serial.print(avrroll);
Serial.print("\t");
Serial.println(avrpitch);

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


//QUATERNIONS CALCULS

float avr(float avr, float new_sample) {
    int N=20;   
    avr -= avr / N;
    avr += new_sample / N;
    return avr;
}

void updateTime()
{
  Now = micros();

  // Set integration time by time elapsed since last filter update
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
}

void GetYRP(){
  updateTime();

ax=IMU.getAccelX_mss();
ay=IMU.getAccelY_mss();
az=IMU.getAccelZ_mss();
gx=IMU.getGyroX_rads();
gy=IMU.getGyroY_rads();
gz=IMU.getGyroZ_rads();
mx=IMU.getMagX_uT();
my=IMU.getMagY_uT();
mz=IMU.getMagZ_uT();

//MadgwickQuaternionUpdate(ax, ay, az, gx, gy , gz , my, mx, mz, deltat);
MahonyQuaternionUpdate(ax, ay, az, gx, gy , gz , my, mx, mz, deltat);

                         // Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      pitch *= RAD_TO_DEG;
      yaw   *= RAD_TO_DEG;

      // Declination Latitude:  48° 0' 29" N  Longitude:  0° 11' 50" E 
      //  0° 30' E ± 0° 22' changing by 0° 11' E per year
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      
      yaw  -= 0.3;
      roll *= RAD_TO_DEG;

      avryaw=avr(avryaw,yaw);
      avrroll=avr(avrroll,roll);
      avrpitch=avr(avrpitch,pitch);

}
