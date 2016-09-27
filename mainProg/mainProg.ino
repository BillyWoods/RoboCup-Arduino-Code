/*
-------------------
Billy Woods 12/8/16
-------------------
Main program which ties everything together
*/

#include <Wire.h> //used for compass sensor
#include <math.h>
#include <EEPROM.h>

#include "Compass.h"
#include "Ultrasonic.h"
#include "Stepper.h"
#include "Robot.h"
#include "Tasks.h"
#include "IRSensor.h"
#include "ColourSensor.h"
#include "ControlSwitch.h"


//define class instances for sensors being used
ultrasonicArray ultrasonics(30,31,32,33,34,35,36,37);
compass HMC5883L(20,21,0x1E);
drive_train platform(3.1415*6.5, int(17.5 * 3.1415 + 0.5), 
                     400, 43, 45, 47, 49, 51, 53);

struct robotState state =
{
    NONE,
    ATTACK,  // strategy

    false,
    180,
    0,
    false,
    false,

    {STARTUP_X_POS, STARTUP_Y_POS}, // put the robot down at this position on startup
    0,
    0,
    0,
    0,
    0
};

String pendingSerialCommand = "";
bool serialCommandComplete = false;

void setup()
{
    // adc prescaler stuff
    ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
    // uncomment as required
    ADCSRA |= bit (ADPS2);                               //  16 
    //  ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32 
    //  ADCSRA |= bit (ADPS1) | bit (ADPS2);                 //  64 
    //  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128

    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);

    initIRs();
    initControlSwitch();

    pinMode(ROLLER_PIN1, OUTPUT);
    pinMode(ROLLER_PIN2, OUTPUT);
    pinMode(ROLLER_SENSE_PIN, INPUT);
    ballRollerOff();

    Serial.begin(9600);
    Wire.begin();
    HMC5883L.init();
    platform.init();
    ultrasonics.init();
    initColourSensor();

    ultrasonics.setFieldSize(FIELD_X, FIELD_Y);
    getCompassCalibrationFromEeprom();
    // roller has to be calibrated to account for falling battery voltage
    //   when trying to use motor current to detect load (ball)
    calibrateBallRoller();

    // on startup set zero bearing to current bearing, so position 
    //   robot to face opposition goals on reset/startup
    delay(250);
    HMC5883L.setZeroBearing();

    // dynamic memory allocation for these queues
    movement_queue.init();

    Serial.println("started");
}


void loop()
{
    static int loopCount = 0;
    static uint16_t lastTime = 0;

    if (pendingSerialCommand != "" && serialCommandComplete)
    {
        handleSerialCommand(pendingSerialCommand);
        pendingSerialCommand = "";
        serialCommandComplete = false;
    }

    updateFromSwitchState();
    updateState();
    taskArbiter();
    movementQueueCheck();

    /*
    loopCount++;
    if (loopCount >= 10)
    {
        loopCount = 0;
        Serial.println("10x: " + String((millis() & 0xFFFF)-lastTime));
        lastTime = millis();
    }
    */
}

void serialEvent()
{
    while(Serial.available() > 0)
    {
        char byteIn = Serial.read();
        if (byteIn  == '\n' || byteIn == '\r')
        {
            serialCommandComplete = true;
        }
        else
        {
            pendingSerialCommand += byteIn;
        }
    }
}
