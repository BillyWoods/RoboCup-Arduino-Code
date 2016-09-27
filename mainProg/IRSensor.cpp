#include "IRSensor.h"
#include "Robot.h"

void initIRs()
{
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(CENTRE_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);
}

void getIRReadings(int* IRArray)
{
    IRArray[0] = analogRead(LEFT_IR_PIN);  
    IRArray[1] = analogRead(CENTRE_IR_PIN);
    IRArray[2] = analogRead(RIGHT_IR_PIN); 
    // second sample, so averaging can be done
    IRArray[0] += analogRead(LEFT_IR_PIN);
    IRArray[1] += analogRead(CENTRE_IR_PIN);
    IRArray[2] += analogRead(RIGHT_IR_PIN);
    // divide by two
    IRArray[0] >>= 1;
    IRArray[1] >>= 1;
    IRArray[2] >>= 1;
    // invert
    IRArray[0] = 1024 - IRArray[0];
    IRArray[1] = 1024 - IRArray[1];
    IRArray[2] = 1024 - IRArray[2];
}

uint32_t lastLockOnBallTime = 0;
int bAngle = -180;
int getBallAngle()
{
    const int threshold = 130;

    int IRReadings[3];
    getIRReadings(IRReadings);

    int totalReadings = 0;

    bool anyIRDetected = false;
    for (int i = 0; i<NUM_IRS; i++)
    {
        if (IRReadings[i] >= threshold)
        {
            anyIRDetected = true;
            totalReadings += IRReadings[i];
        }
        else
        {
            // stop non-detections affecting angle calculations
            //   by setting them to zero
            IRReadings[i] = 0;
        }
    }
    // no IRs have detected anything
    if (!anyIRDetected)
    {
        // been too long since last saw ball to guess at likely position
        if (millis() - lastLockOnBallTime > 2000)
            bAngle = bAngle < 0? -180:180;
        // probably a good assumption that the ball went farther along
        //   its path
        else
            bAngle *= 2;
    }
    else
    {
        // we have a valid lock on ball; an IR has made a detection
        lastLockOnBallTime = millis();
        // commented out below is the unsimplified derivation
        //angle += -IR_SPACING_ANGLE * (IRReadings[0]/totalReadings);
        //angle +=  0                * (IRReadings[1]/totalReadings);
        //angle +=  IR_SPACING_ANGLE * (IRReadings[2]/totalReadings);
        bAngle = IR_SPACING_ANGLE * (IRReadings[2] - IRReadings[0]);
        bAngle /= totalReadings;
    }

    return bAngle;
}
