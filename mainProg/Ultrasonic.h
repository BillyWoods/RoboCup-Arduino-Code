#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>


//an object detected obstructing the ultrasonic array
/*class obstruct
{
    public:
        obstruct();
        int pos[2]; //absolute position
};*/

//a single ultrasonic sensor
class ultrasonicSensor
{
    private:
        uint8_t trigPin, echoPin;
    public:
        ultrasonicSensor(uint8_t trigger, uint8_t echo);
        ultrasonicSensor();
        void setPins(uint8_t trigger, uint8_t echo);
        void init(); //will init the pins
        int ping(); //returns distance in cm, or -1 if timeout
};

//an array of 4 sensors
class ultrasonicArray
{
    private:
        int fieldSize[2]; //width and length
        int sensorSpacing; //distance between opposing sensors
        int fieldSizeTolerance;
        uint32_t lastPingTime; //timestamp in milliseconds
        void defaultVarInit(); //called by constructors, sets default vals for vars above
    public:
        ultrasonicSensor front,
                         right,
                         back,
                         left;
    
        ultrasonicArray(int fTrig, int fEcho, int rTrig, int rEcho, 
                        int bTrig, int bEcho, int lTrig, int lEcho);
        void init();
        void setFieldSize(int X, int Y);
        void setSensorSpacing(int spacing);
        // writes all measured distances to array passed to it
        void pingAllSensors(int* data); 
        // pass it an array of two ints: [x,y]
        // heading in range [-180,180], last position can also be estimated position
        void getPosition(int* positionData, int heading, int* lastPosition);
        int getTimeSinceLastPing();
};

#endif // ULTRASONIC_H
