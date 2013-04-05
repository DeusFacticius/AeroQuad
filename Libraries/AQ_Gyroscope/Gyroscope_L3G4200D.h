/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_GYROSCOPE_L3G4200D_H_
#define _AEROQUAD_GYROSCOPE_L3G4200D_H_

#include <Gyroscope.h>
#include <SensorsStatus.h>
#include <Device_I2C.h>

#define GYRO_CALIBRATION_THRESHOLD 32

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define TEMP_OUT	0x26
#define STATUS_REG	0x27
#define X_OUT_L		0x28
#define X_OUT_H		0x29
#define Y_OUT_L		0x2A
#define Y_OUT_H		0x2B
#define Z_OUT_L		0x2C
#define Z_OUT_H		0x2D

#define SEQ_READ_FLG	0x80

#define XEN_FLG 	0x01
#define YEN_FLG		0x02
#define ZEN_FLG		0x04
#define PD_FLG		0x08
#define DR1_FLG		0x80
#define DR0_FLG		0x40
#define BW1_FLG		0x20
#define BW0_FLG		0x10
#define NO_PD			(PD_FLG | XEN_FLG | YEN_FLG | ZEN_FLG)

#define LE_FLG				0x00
#define BE_FLG				0x40
#define GYRO_FS_250		0x00
#define GYRO_FS_500		0x10
#define GYRO_FS_2000 	0x30

//#define GYRO_RATE GYRO_FS_2000
//#define GYRO_RATE		GYRO_FS_500
#define GYRO_RATE		GYRO_FS_250

#define HPEN_FLG	0x10

#define GYRO_BUF_SIZE 6

#define L3G4200D_IDENTITY   0xD3
#define GYRO_ADDR			0x68

//int GyroAddr = 105;                 // I2C address of gyro
float gyroTempBias[3] = {0.0, 0.0, 0.0};
int gyroTemperature = 0;
float gyroTempBiasSlope[3] = { 0.0, 0.0, 0.0 };
float gyroTempBiasIntercept[3] = { 0.0, 0.0, 0.0 };

int gyroOrientation[3] = { -1, 1, -1 };

void measureGyroSum();
void evaluateGyroRate();
void initializeGyro();
void measureGyro();
boolean calibrateGyro();
void readGyroTemp();

int readGyroReg(byte subaddr) {
	//Serial.println("Beginning transmission...");
  Wire.beginTransmission(GYRO_ADDR);
  //Serial.println("Writing subaddr...");
  Wire.write((byte)subaddr);
  //Serial.println("Ending transmission...");
  Wire.endTransmission();
  //Serial.println("Delaying...");
  delay(100);
  //Serial.println("Requesting from gyro...");
  Wire.requestFrom(GYRO_ADDR, 1);
  //Serial.println("Reading from gyro...");
  return Wire.read();
}

void initializeGyro() {
	//Serial.println("Detecting gyro...");
	if((readGyroReg(0x0F) & 0xFF) == L3G4200D_IDENTITY)
	  vehicleState |= GYRO_DETECTED;
	
  //Serial.println("Updating reg1...");
	// Turn on all axes, disable power down,
	// set output data rate to 400Hz, and
	// bandwidth to 25 cutoff
  updateRegisterI2C(GYRO_ADDR, CTRL_REG1, 
  	(DR1_FLG | PD_FLG | XEN_FLG | YEN_FLG | ZEN_FLG));
  delay(5);
  
  //Serial.println("Updating reg2....");
  // Disable high-pass filter
  updateRegisterI2C(GYRO_ADDR, CTRL_REG2, 0);
  delay(5);
  
  //Serial.println("Updating reg3...");
  // Disable interrupts, etc.
  updateRegisterI2C(GYRO_ADDR, CTRL_REG3, 0);
  delay(5);
  
  //Serial.println("Updating reg4....");
  if(GYRO_RATE == GYRO_FS_250) {
  	updateRegisterI2C(GYRO_ADDR, CTRL_REG4, (BE_FLG | GYRO_FS_250));
  	gyroScaleFactor = radians(8.75 / 1000.0);
  } else if(GYRO_RATE == GYRO_FS_500) {
  	updateRegisterI2C(GYRO_ADDR, CTRL_REG4, (BE_FLG | GYRO_FS_500));
  	gyroScaleFactor = radians(17.50 / 1000.0);
  } else if(GYRO_RATE == GYRO_FS_2000) {
  	updateRegisterI2C(GYRO_ADDR, CTRL_REG4, (BE_FLG | GYRO_FS_2000));
  	gyroScaleFactor = radians(70.00 / 1000.0);
  }
  
  // Enable high pass filter w/ default values
  //updateRegisterI2C(GYRO_ADDR, CTRL_REG5, HPEN_FLG);
  
  //Serial.println("Syncronizing...");
  // Wait for synchronization
  delay(100);
  
  //gyroScaleFactor = radians(17.50 / 1000.0);
}

void computeGyroTCBias() {
	readGyroTemp();
	
	for(byte axis = XAXIS; axis <= ZAXIS; axis++)
		gyroTempBias[axis] = gyroTempBiasSlope[axis] * gyroTemperature + gyroTempBiasIntercept[axis];
}

void measureGyroADC(int *gyroADC) {
	sendByteI2C(GYRO_ADDR, (X_OUT_L | SEQ_READ_FLG));
	Wire.requestFrom(GYRO_ADDR, GYRO_BUF_SIZE);
	gyroADC[XAXIS] = gyroOrientation[XAXIS]*readShortI2C();
	gyroADC[YAXIS] = gyroOrientation[YAXIS]*readShortI2C();
	gyroADC[ZAXIS] = gyroOrientation[ZAXIS]*readShortI2C();
}

void orientGyroValues(int *gyroADC) {
	gyroADC[XAXIS] = gyroADC[XAXIS] - gyroZero[XAXIS];
	gyroADC[YAXIS] = gyroADC[YAXIS] - gyroZero[YAXIS];
	gyroADC[ZAXIS] = gyroADC[ZAXIS] - gyroZero[ZAXIS];
}

boolean calibrateGyro() {

  int findZero[FINDZERO];
  int diff = 0;
    
  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      sendByteI2C(GYRO_ADDR, ((axis * 2) + X_OUT_L) | SEQ_READ_FLG);
      findZero[i] = gyroOrientation[axis]*readShortI2C(GYRO_ADDR);
      delay(10);
    }
    int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
    //if(diff <= GYRO_CALIBRATION_THRESHOLD) {
    	gyroZero[axis] = tmp;
    //} else {
    //	return false;
    //}
  }
  return true;
}

void measureGyro() {
  int gyroADC[3];
  
  measureGyroADC(gyroADC);
  
  orientGyroValues(gyroADC);
  
  computeGyroTCBias();
  
  for(byte axis = XAXIS; axis <= ZAXIS; axis++) {
  	//gyroRate[axis] = filterSmooth((gyroADC[axis] * gyroScaleFactor) - gyroTempBias[axis], 
  	//		gyroRate[axis], gyroSmoothFactor);
  	gyroRate[axis] = gyroADC[axis] * gyroScaleFactor;
  }
  
  // measure gyro heading
  long int currentTime = micros();
	if(gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0))
		gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
	
	gyroLastMesuredTime = currentTime;
}

void evaluateGyroRate() {
	int gyroADC[3];
	gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount);
	gyroADC[YAXIS] = (gyroSample[YAXIS] / gyroSampleCount);
	gyroADC[ZAXIS] = (gyroSample[ZAXIS] / gyroSampleCount);
	
	gyroSample[XAXIS] = gyroSample[YAXIS] = gyroSample[ZAXIS] = 0;
	gyroSampleCount = 0;
	
	orientGyroValues(gyroADC);
	
	computeGyroTCBias();
	
	for(byte axis = XAXIS; axis <= ZAXIS; axis++) {
		gyroRate[axis] = gyroADC[axis] * gyroScaleFactor;
		//gyroRate[axis] = filterSmooth((gyroADC[axis] * gyroScaleFactor) - gyroTempBias[axis],
		//		gyroRate[axis], gyroSmoothFactor);
	}
	
	long int currentTime = micros();
	if(gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0))
		gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
	
	gyroLastMesuredTime = currentTime;
}

void readGyroTemp() {
	sendByteI2C(GYRO_ADDR, TEMP_OUT);
	gyroTemperature = readByteI2C(GYRO_ADDR);
}

void measureGyroSum() {
	int gyroADC[3];
	
	measureGyroADC(gyroADC);
	
	gyroSample[XAXIS] += gyroADC[XAXIS];
	gyroSample[YAXIS] += gyroADC[YAXIS];
	gyroSample[ZAXIS] += gyroADC[ZAXIS];
	
	gyroSampleCount++;
}

#endif