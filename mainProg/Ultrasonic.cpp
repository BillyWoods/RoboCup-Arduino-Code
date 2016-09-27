#include "Ultrasonic.h"



/* --------------------------
   SINGLE ULTRASONIC SENSOR
-----------------------------*/

void ultrasonicSensor::init()
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT );
    digitalWrite(trigPin, LOW);
}

ultrasonicSensor::ultrasonicSensor(uint8_t trigger, uint8_t echo)
{
    trigPin = trigger;
    echoPin = echo;
}

ultrasonicSensor::ultrasonicSensor()
{
    // empty
}

void ultrasonicSensor::setPins(uint8_t trigger, uint8_t echo)
{
    trigPin = trigger;
    echoPin = echo;
}

int ultrasonicSensor::ping()
{
    int duration;
    int cm;
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    
    // 15ms timeout gives max range of 2.5m
    duration = pulseIn(echoPin, HIGH, 15000);
    cm = duration/29/2;
    if (cm < 300) return cm;
    else           return -1;
}


/* -----------------------
   ULTRASONIC ARRAY PART
-------------------------*/

void ultrasonicArray::defaultVarInit()
{
    setSensorSpacing(17);
    fieldSizeTolerance = 45;
    //setFieldSize(100, 140);
}

ultrasonicArray::ultrasonicArray(int fTrig, int fEcho, int rTrig, int rEcho, 
                                 int bTrig, int bEcho, int lTrig, int lEcho)
{
    front.setPins(fTrig, fEcho);
    right.setPins(rTrig, rEcho);
    back.setPins(bTrig, bEcho);
    left.setPins(lTrig, lEcho);
    defaultVarInit();
    lastPingTime = 0;
}

void ultrasonicArray::init()
{
    front.init();
    right.init();
    back.init();
    left.init();

    defaultVarInit();
}

void ultrasonicArray::setFieldSize(int X, int Y)
{
    fieldSize[0] = X;
    fieldSize[1] = Y;
}

void ultrasonicArray::setSensorSpacing(int spacing)
{
    sensorSpacing = spacing;
}

void ultrasonicArray::pingAllSensors(int* data)
{
    data[0] = front.ping();
    data[1] = right.ping();
    data[2] = back.ping();
    data[3] = left.ping();
    
    lastPingTime = millis(); 
}

void ultrasonicArray::getPosition(int* positionData, int _heading, int* lastPosition) //obstruct* obstructTracker)
{
    // tolerance of being orthogonal to walls
    const int angleTol = 20;

    // convert heading of [-180,180] to [0,360]
    int heading = _heading >= 0? _heading : 360 + _heading;

    int retVal[2] = {-1, -1};

    // ultrasonics can only be used to find position when we are roughly orthogonal to walls
    if ((heading + angleTol) % 90 < (angleTol * 2))
    {
        int rotations = (heading+angleTol)/90;
        int readings[4];
        pingAllSensors(readings);
        //this is the tolerance that the last position and reading have to be
        //within otherwise the reading will be considered an obstruction
        int deltaTol = 50;

        //correct directions for heading
        if (rotations)
        {
            int tempReadings[4];
            tempReadings[0] = readings[(4 - rotations + 0)%4];
            tempReadings[1] = readings[(4 - rotations + 1)%4];
            tempReadings[2] = readings[(4 - rotations + 2)%4];
            tempReadings[3] = readings[(4 - rotations + 3)%4];
            for(int i = 0; i<4; i++)
                readings[i] = tempReadings[i];
        }
        //from last position, try and figure out what the previous reading was
        //this is done so that any crazy deltas can be attributed to an obstruction
        int prevReading[4];
        prevReading[0] = fieldSize[1] - lastPosition[1] - sensorSpacing/2;
        prevReading[1] = fieldSize[0] - lastPosition[0] - sensorSpacing/2;
        prevReading[2] = lastPosition[1] - sensorSpacing/2;
        prevReading[3] = lastPosition[0] - sensorSpacing/2;
        
        int measuredWidth  = readings[3] + readings[1] + sensorSpacing,
            measuredLength = readings[0] + readings[2] + sensorSpacing;
        
        // ------------- calculating x position ----------
        //there is an obstruction either on the left, right, or both
        if (abs(measuredWidth - fieldSize[0]) >  fieldSizeTolerance)
        {
            int deltaLeft = abs(readings[3] - prevReading[3]),
                deltaRight = abs(readings[1] - prevReading[1]);
            //obstruction on the right
            if (deltaRight > deltaLeft && deltaLeft < deltaTol)
            {
                retVal[0] = readings[3] + sensorSpacing/2;
            }
            //obstruction on the left
            else if (deltaLeft > deltaRight && deltaRight < deltaTol)
            {
                retVal[0] = fieldSize[0] - readings[1] - sensorSpacing/2; 
            }
            //left and right sensors obstructed or our prev reading is way out of date
            else
            {
                retVal[0] = -1;
            }
        }
        // left and right unobstructed
        else 
        {
            retVal[0] = readings[3] + sensorSpacing/2;
        }

        // ------------- calculating y position ----------
        //there is an obstruction either in front, behind or both
        if (abs(measuredLength - fieldSize[1]) >  fieldSizeTolerance)
        {
            int deltaFront = abs(readings[0] - prevReading[0]),
                deltaBack = abs(readings[2] - prevReading[2]);
            //obstruction in front, use back reading
            if (deltaFront > deltaBack && deltaBack < deltaTol)
            {
                retVal[1] = readings[2] + sensorSpacing/2;
            }
            //obstruction behind
            else if (deltaBack > deltaFront && deltaFront < deltaTol)
            {
                retVal[1] = fieldSize[1] - readings[0] - sensorSpacing/2; 
            }
            //blocked on both front and behind or prev reading is far out
            else
            {
                retVal[1] = -1;
            }
        }
        // front and back unobstructed
        else
        {
            retVal[1] = readings[2] + sensorSpacing/2;
        }
    }
    positionData[0] = retVal[0];
    positionData[1] = retVal[1];
}

int ultrasonicArray::getTimeSinceLastPing()
{
    return millis() - lastPingTime;
}



//for the obstruction object
/*obstruct::obstruct()
{
    //empty
}*/
