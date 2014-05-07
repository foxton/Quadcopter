/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include <Servo.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

Servo engine1;
Servo engine2;
Servo engine3;
Servo engine4;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

int i, j, k, l, m, n, o, p, q; //These variables are used for sending the start signal to the ESC
int engineX, engineY;
/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
//double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
int kalAngleX, kalAngleY;
int thrust1 = 65,thrust2 = 65,thrust3 = 65,thrust4 = 65;
int isflying = 0;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() 
{  
 
  delay(5000);
	for(m = 0; m <= 2; m += 1) 
{
engine1.attach(5); // ESC engine 1 connected to pin 5
delay(50);
engine2.attach(6); // ESC engine 2 connected to pin 6
delay(50);
engine3.attach(9); // ESC engine 3 connected to pin 9
delay(50);
engine4.attach(10); // ESC engine 4 connected to pin 10
delay(100);

for(i = 5; i <= 15; i += 1) //To start the ESC, the "throttle" must be in the bottom location.  The correct value for starting the ESC is somewhere between 5 and 15 (Range 0-180)
{							
	engine1.write(i);							
	delay(20);
}

delay(100);
for(j = 5; j <= 15; j += 1) //Start ESC
{												
	engine2.write(j);
	delay(20);
}
delay(100);
for(k = 0; k <= 20; k += 1) //Start ESC
  {												
    engine3.write(k);
	delay(20);
  }
delay(100);
	for(l = 0; l <= 20; l += 1) //Start ESC
  {
    engine4.write(l);
	delay(20);
  }
}

  Serial.begin(9600);
  Wire.begin();

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 
  
  while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
  }
  
  delay(100); // Wait for sensor to stabilize
  
  /* Set kalman and gyro starting angle */
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;
  
  timer = micros();
 
}

void loop() 
{
  /*
  if(isflying == 0)
  {
 for(thrust1 = 63, thrust2 = 63, thrust3 = 63, thrust4 = 63; thrust1 <= 90, thrust2 <= 90, thrust3 <= 90, thrust4 <= 90; thrust1 += 1, thrust2 += 1, thrust3 += 1, thrust4 += 1 )
{
engine1.write(thrust1);

engine2.write(thrust2);

engine3.write(thrust3);

engine4.write(thrust4);
delay(1000); //Delay so that the engines aren't accelerating too fast
Serial.print("Accelerating");                  //For debugging
 Serial.print("Thrust");Serial.print("\n");    //Print engine thrust values
Serial.print(thrust1);Serial.print("\t");
Serial.print(thrust2);Serial.print("\t");
Serial.print(thrust3);Serial.print("\t");
Serial.print(thrust4);Serial.print("\n");
gyro();
}
isflying = 1;
  }
*/
  
 gyro();
 idle(); 
  /* Print Data */
  

  Serial.print("kalAngleX \t");                //Print Gyro angles with kalman filter.
  Serial.print("kalAngleY \t");
  Serial.print("accX \t");
  Serial.print("accY \t");
  Serial.print("accZ \n");
  Serial.print(kalAngleX);Serial.print("\t");
  Serial.print("\t");
  Serial.print(kalAngleY);Serial.print("\t");
 Serial.print("\n");



 Serial.print(accX);Serial.print("\t");
 Serial.print("\t");
 Serial.print(accY);Serial.print("\t");
 Serial.print("\t");
 Serial.print(accZ);Serial.print("\n");
 Serial.print("\t");
 
  

 Serial.print("Thrust");Serial.print("\n");    //Print engine thrust values
Serial.print(thrust1);Serial.print("\t");
Serial.print(thrust2);Serial.print("\t");
Serial.print(thrust3);Serial.print("\t");
Serial.print(thrust4);Serial.print("\n");

  
   
  Serial.print("\r\n");
  delay(50);
 
 






}

void idle()
{
engine1.write(thrust1);

engine2.write(thrust2);

engine3.write(thrust3);

engine4.write(thrust4);
}
void gyro()
{
  /* Update all the values */  
   while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
  
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  timer = micros();
  
  temp = ((double)tempRaw + 12412.0) / 340.0;
  
 if(kalAngleX > 185)            //Keep the quadcopter in the air
{
engine1.write(thrust1 += 1); 
engine2.write(thrust2 -= 1); 
engine3.write(thrust3 += 1); 
engine4.write(thrust4 -= 1);
} 

if(kalAngleY > 185)
{
engine1.write(thrust1 -= 1); 
engine2.write(thrust2 -= 1); 
engine3.write(thrust3 += 1); 
engine4.write(thrust4 += 1); 
}

if(kalAngleX > 175 && kalAngleX < 185 && kalAngleY > 175 && kalAngleY < 185)
{
engine1.write(thrust1); 
engine2.write(thrust2); 
engine3.write(thrust3); 
engine4.write(thrust4); 
}

if(kalAngleX < 175)
{
engine1.write(thrust1 -= 1); 
engine2.write(thrust2 += 1); 
engine3.write(thrust3 -= 1); 
engine4.write(thrust4 += 1); 
}

if(kalAngleY < 175)
{
engine1.write(thrust1 += 1); 
engine2.write(thrust2 += 1); 
engine3.write(thrust3 -= 1); 
engine4.write(thrust4 -= 1); 
} 
}
