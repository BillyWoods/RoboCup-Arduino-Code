#ifndef COMPASS_SENSOR_H
#define COMPASS_SENSOR_H

/*
BILLY WOODS 20/08/16
--------------------
COMPASS LIBRARY FOR THE HONEYWELL HMC5883L
--------------------
NOTES:

For HMC5883L:
  -read addr        0x3D (00111101)
  -write addr       0x3C (00111100)
  -root 7 bit addr  0x1E (0011110 )  <---- used by Wire.h library

Arduino pinouts:
 _______________________________
| i2c function  | UNO   | MEGA  |
|---------------+-------+-------|
| SDA           | 4     | 20    |
| SDC           | 5     | 21    |
 -------------------------------
*/


// this ensures that turning CW gives + and CCW gives -
#define SENSOR_FLIPPED false 

class compass
{
    private:
        int sdaPin, sdcPin, i2cAddress, i2cTimeout, currentWaitTime,
            bearingOffset;
        int bias[3];
        float scaleFactor[3];
        
        void getRawData(); // misleading, as does not return data, writes it to compassData
        void applyCompensation(); //applies vals from calibration to compassData

    public:
        int compassData[3];
        
        compass(int sda, int sdc, int addr);

        void getCalibrationValues(int* _bias, float* _scaleFactor);
        void setCalibrationValues(int* _bias, float* _scaleFactor);

        void init(); // configures continuous operation, 8x sampling
        void calibrate(); // wave the compass round for 10 seconds after calling this
        void debug(); // prints debug info

        // bearings from following two functions are in range (-180, 180)
        int getRealBearing(); 
        int getRelativeBearing();
        void setZeroBearing(); //set zero bearing to current angle
};

#endif
