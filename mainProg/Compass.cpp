#include "Compass.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

compass::compass(int sda, int sdc, int addr)
{
    sdaPin = sda;
    sdcPin = sdc;
    i2cAddress = addr;
    i2cTimeout = 15;
    currentWaitTime = 0;
    bearingOffset = 0;
    compassData[0] = 0;
    compassData[1] = 0;
    compassData[2] = 0;
    bias[0] = 0;
    bias[1] = 0;
    bias[2] = 0;
    scaleFactor[0] = 0;
    scaleFactor[1] = 0;
    scaleFactor[2] = 0;
}

void compass::getCalibrationValues(int* _bias, float* _scaleFactor)
{
    for (int i = 0; i < 3; i++)
    {
        _bias[i] = bias[i];
        _scaleFactor[i] = scaleFactor[i];
    }
}

void compass::setCalibrationValues(int* _bias, float* _scaleFactor)
{
    for (int i = 0; i < 3; i++)
    {
        bias[i] = _bias[i];
        scaleFactor[i] = _scaleFactor[i];
    }
}

void compass::init()
{   
    //set sensitivity range for +-1.3Ga
    Wire.beginTransmission(i2cAddress);
    Wire.write((byte) 0x01);
    Wire.write((byte) 0x20);
    Wire.endTransmission();
    
    delay(2);
    
    //set register a for 8x oversampling
    Wire.beginTransmission(i2cAddress);
    Wire.write((byte) 0x00);
    Wire.write((byte) 0x70);
    Wire.endTransmission();
    
    delay(2);
    
    //set continuous operation mode  
    Wire.beginTransmission(i2cAddress);
    Wire.write((byte) 0x02);
    Wire.write((byte) 0x00);
    Wire.endTransmission();
    
    delay(2);
}

void compass::calibrate()
{
   const int numSamples = 20;
   const int waitTime = 500;
   int minX = 0, maxX = 0,
       minY = 0, maxY = 0,
       minZ = 0, maxZ = 0;
   
   for (int i = 0; i < numSamples; i++)
   {
       getRawData();
       minX = min(minX, compassData[0]);
       maxX = max(maxX, compassData[0]);
       minY = min(minY, compassData[1]);
       maxY = max(maxY, compassData[1]);
       minZ = min(minZ, compassData[2]);
       maxZ = max(maxZ, compassData[2]);
       delay(waitTime);
   }
   bias[0] = int((maxX+minX)/2);
   bias[1] = int((maxY+minY)/2);
   bias[2] = int((maxZ+minZ)/2);
   
   scaleFactor[0] = 500.0/(maxX - minX);
   scaleFactor[1] = 500.0/(maxY - minY);
   scaleFactor[2] = 500.0/(maxZ - minZ);
   
   Serial.println(maxX - minX);
   Serial.println(maxY - minY);
   Serial.println(maxZ - minZ);
}

void compass::debug()
{
    Serial.println("compass calibration stuff");
    Serial.println(bias[0]);
    Serial.println(bias[1]);
    Serial.println(bias[2]);
    Serial.println(scaleFactor[0]);
    Serial.println(scaleFactor[1]);
    Serial.println(scaleFactor[2]);
}

void compass::applyCompensation()
{
    compassData[0] = (compassData[0]-bias[0]) * scaleFactor[0];
    compassData[1] = (compassData[1]-bias[1]) * scaleFactor[1];
    compassData[2] = (compassData[2]-bias[2]) * scaleFactor[2];
}

void compass::getRawData()
{
    currentWaitTime = 0;
    //move register pointer to start of X reg
    Wire.beginTransmission(i2cAddress);
    Wire.write((byte) 0x03);
    Wire.endTransmission();

    Wire.requestFrom(i2cAddress, 6);
    while (Wire.available()<6 && currentWaitTime<=i2cTimeout)
    {
        currentWaitTime++;
        delay(1);
    }
    compassData[0] = Wire.read() << 8;
    compassData[0] |= Wire.read();
    compassData[2] = Wire.read() << 8;
    compassData[2] |= Wire.read();
    compassData[1] = Wire.read() << 8;
    compassData[1] |= Wire.read();
}

int compass::getRealBearing()
{
    getRawData();
    applyCompensation();
    //angle returned in radians
    float angle = SENSOR_FLIPPED? atan2(compassData[0], compassData[1]): atan2(compassData[1], compassData[0]);
    return angle*(180.0/3.141592);
}

int compass::getRelativeBearing()
{
    int relBearing = getRealBearing() - bearingOffset;
    //make sure we stay within (-180,180)
    if (relBearing < -180)
    {
        relBearing = 180 - (-relBearing % 180);
    }
    else if (relBearing > 180)
    {
        relBearing = -180 + (relBearing % 180);
    }
    return relBearing;
}

void compass::setZeroBearing()
{
    bearingOffset = getRealBearing();
}
