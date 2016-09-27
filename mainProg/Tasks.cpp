#include "Tasks.h"

#include <EEPROM.h>

#include "Robot.h"
#include "Stepper.h"
#include "Compass.h"
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "ColourSensor.h"

extern ultrasonicArray ultrasonics;
extern compass HMC5883L;
extern drive_train platform;
extern robotState state;

/*--------------------------------------------------------------
 the following are vars for checkForBall() and calibrateBallRoller()
---------------------------------------------------------------*/
// an expected analog value which would be read if roller was on with no ball
//   this value is calibrated by calibrateBallRoller()
int rollerRunningNoBall = 738;
// how far from rollerRunningNoBall a value must be to be
//   considered a detection
const int rollerBallDetectionThreshold = 6;


void handleSerialCommand(String _cmd)
{
    if (_cmd == "stop")
    {
        state.currentTask = NONE;
        state.strategy = NONE;
        clearMovementQueue();
        ballRollerOff();
    }

    else if (_cmd == "debug ir")
        state.currentTask = DEBUG_IR;
    else if (_cmd == "debug compass")
        state.currentTask = DEBUG_COMPASS;
    else if (_cmd == "debug ultrasonics")
        state.currentTask = DEBUG_ULTRASONICS;
    else if (_cmd == "debug ball roller")
        state.currentTask = DEBUG_BALL_ROLLER;
    else if (_cmd == "debug state")
        state.currentTask = DEBUG_MACHINE_STATE;


    else if (_cmd == "calibrate compass")
        state.currentTask = CALIBRATE_COMPASS;
    else if (_cmd == "zero compass")
        HMC5883L.setZeroBearing();
    else if (_cmd == "save config")
        saveCompassCalibrationToEeprom();
    else if (_cmd == "load config")
        getCompassCalibrationFromEeprom();
    else if (_cmd == "calibrate ball roller")
        state.currentTask = CALIBRATE_BALL_ROLLER;

    else if (_cmd == "test pattern 1")
        state.currentTask = TEST_PATTERN_1;

    else if (_cmd == "find ball")
        state.currentTask = FIND_BALL;
    else if (_cmd == "move attack side")
        state.currentTask = MOVE_ATTACK_SIDE;
    else if (_cmd == "shoot ball")
        state.currentTask = SHOOT_BALL;

    else if (_cmd == "update position")
        state.currentTask = FORCE_UPDATE_POSITION;
    else if (_cmd == "ball roller on")
        ballRollerOn();
    else if (_cmd == "ball roller off")
        ballRollerOff();

    else if (_cmd == "attack program")
        state.strategy = ATTACK;
    else if (_cmd == "defence program")
        state.strategy = DEFENCE;

    else if (_cmd == "ultrasonic bounding on")
        USE_ULTRASONIC_BOUNDING = true;

    else
        Serial.println("unrecognised command");
}

void taskArbiter()
{
    if (state.strategy == ATTACK)
        attackProgram();
    else if (state.strategy == DEFENCE)
        defenceProgram();

    switch (state.currentTask)
    {
        case (DEBUG_IR):
            debugIR();
            break;
        case (DEBUG_COMPASS):
            debugCompass();
            break;
        case (DEBUG_ULTRASONICS):
            debugUltrasonics();
            break;
        case (DEBUG_BALL_ROLLER):
            debugBallRoller();
            break;
        case (DEBUG_MACHINE_STATE):
            debugMachineState();
            break;
        
        case (CALIBRATE_COMPASS):
            calibrateCompass();
            break;
        case (CALIBRATE_BALL_ROLLER):
            calibrateBallRoller();
            break;

        case (TEST_PATTERN_1):
            testPattern1();
            break;

        case(FIND_BALL):
            findBall();
            break;
        case(MOVE_ATTACK_SIDE):
            moveAttackSide();
            break;
        case(SHOOT_BALL):
            shootBall();
            break;

        case(FORCE_UPDATE_POSITION):
            forceUpdatePosition();
            break;
    }
}



//debug functions
void debugUltrasonics()
{
    int timeFromLastPing = ultrasonics.getTimeSinceLastPing();
    int rawResult[4];
    ultrasonics.pingAllSensors(rawResult);
    int supposedPosition[2]; 
    int lastPos[2] = {40,40};
    ultrasonics.getPosition(supposedPosition, HMC5883L.getRelativeBearing(), lastPos);

    Serial.println("---- Ultrasonic Debug Data ----");
    Serial.println("Time elapsed since last ping: " + String(timeFromLastPing));
    Serial.println("(F, R, B, L): (" + String(rawResult[0]) + ", " + String(rawResult[1]) +
            ", " + String(rawResult[2]) + ", " + String(rawResult[3]) + ")");
    Serial.println("Position (x,y): " + String(supposedPosition[0]) + ", " +
            String(supposedPosition[1]));
    delay(1000);
}

void debugCompass()
{
    Serial.println("---- Compass Debug Data ----");
    int realFacing = HMC5883L.getRealBearing();
    int relativeFacing = HMC5883L.getRelativeBearing();
    Serial.println("Absolute bearing: " + String(realFacing));
    Serial.println("Relative bearing: " + String(relativeFacing));
    HMC5883L.debug();
    delay(1000);
}

void debugIR()
{
    // commented out as was for IRSensor.h.unused
    //int IRs[8] = {0,0,0,0,0,0,0,0};
    //getIRReadings(IRs);

    int l = analogRead(LEFT_IR_PIN);  // 1000 - IRs[0];
    int c = analogRead(CENTRE_IR_PIN);  // 1000 - IRs[1];
    int r = analogRead(RIGHT_IR_PIN); // 1000 - IRs[2];
        r += analogRead(RIGHT_IR_PIN);
        c += analogRead(CENTRE_IR_PIN);
        l += analogRead(LEFT_IR_PIN);
        l >>= 1;
        c >>= 1;
        r >>= 1;

    int rawColourSensorData = analogRead(COLOUR_SENSOR_PIN);

    Serial.println("---- IR Debug Data ----");
    Serial.println("left: " + String(l));
    Serial.println("centre: " + String(c));
    Serial.println("right: " + String(r));
    Serial.println("angle to ball: " + String(state.ballAngle));
    Serial.println("raw colour underneath: " + String(rawColourSensorData));

    delay(1000);
}

void debugBallRoller()
{
    Serial.println("------ debug ball roller -----");

    // do some averaging of the roller sense reading
    int readTot = 0;
    const int numSamples = 3;
    for (int i=0; i<numSamples; i++)
        Serial.println("motor current draw: " + String(analogRead(ROLLER_SENSE_PIN)));

    Serial.println("ball state: " + String(state.hasBall));
    Serial.println("tolerance: " + String(rollerBallDetectionThreshold));
    Serial.println("no ball supposed val: " + String(rollerRunningNoBall));

    delay(1000);
}

void debugMachineState()
{
    updateState();
    Serial.println("has ball: " + String(state.hasBall));
    Serial.println("ball angle: " + String(state.ballAngle));
    Serial.println("last had ball: " + String(state.lastHadBall));
    Serial.println("position (x,y): " + String(state.position[0]) + " , " + state.position[1]);
    Serial.println("facing: " + String(state.facing));
    Serial.println("heading: " + String(state.heading));
    Serial.println("speed: " + String(state.speed));
    Serial.println("strategy? : " + String(state.strategy));
    Serial.println("ultrasonic bounding? : " + String(USE_ULTRASONIC_BOUNDING));
    Serial.println("colour underneath? : " + String(state.colourUnderneath));
    delay(1000);
}

void calibrateCompass()
{
    platform.rotate(370, 37);
    HMC5883L.calibrate();
    state.currentTask = NONE;
}

void saveCompassCalibrationToEeprom()
{
    int biases[3];
    float scaleFactors[3];
    HMC5883L.getCalibrationValues(biases, scaleFactors);

    uint16_t addr = COMPASS_EEPROM_BEGIN;

    for (int i = 0; i < 3; i++)
    {
        byte toWriteH = (biases[i] & 0xFF00) >> 8,
             toWriteL = (biases[i] & 0x00FF);
        EEPROM.write(addr, toWriteH);
        addr += 1;
        EEPROM.write(addr, toWriteL);
        addr += 1;
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 3; j >= 0; j--)
        {
            byte toWrite = *((byte*)(scaleFactors + i) + j);
            EEPROM.write(addr, toWrite);
            addr += 1;
        }
    }
}

void getCompassCalibrationFromEeprom()
{
    int biases[3] = {0,0,0};
    float scaleFactors[3];
    uint16_t addr = COMPASS_EEPROM_BEGIN;

    for (int i = 0; i < 3; i++)
    {
        biases[i] = EEPROM.read(addr) << 8;
        addr += 1;
        biases[i] |= EEPROM.read(addr);
        addr += 1;
    }

    for (int i = 0; i < 3; i++)
    {
        *((uint32_t*)(scaleFactors + i)) = 0;
        for (int j = 3; j >= 0; j--)
        {
            *((byte*)(scaleFactors + i) + j) = EEPROM.read(addr);
            addr += 1;
        }
    }

    HMC5883L.setCalibrationValues(biases, scaleFactors);
}

void calibrateBallRoller()
{
    ballRollerOn();
    delay(1500);
    int total = 0;
    const int numSamples = 10;
    for (int i = 0; i<numSamples; i++)
    {
        total += analogRead(ROLLER_SENSE_PIN);
        delay(1);
    }

    ballRollerOff();
    rollerRunningNoBall = total/numSamples - 2;

    if (state.currentTask == CALIBRATE_BALL_ROLLER)
        // if called by a serial command, clear the task, so it 
        //   does not repeat
        state.currentTask = NONE;
}


void testPattern1()
{
    const int speed = 15; // cm/s
    Serial.println("Test pattern 1 in 10 seconds");
    delay(10000);

    lodgeNewMovement(movement{STRAIGHT_LINE, 0, speed, 30});
    lodgeNewMovement(movement{STRAIGHT_LINE, 90, speed, 30});
    lodgeNewMovement(movement{STRAIGHT_LINE, 180, speed, 30});
    lodgeNewMovement(movement{STRAIGHT_LINE, -90, speed, 30});

    state.currentTask = NONE;
}

/*--------------------------------------------------------------
Note that code which has been commented out of findBall, updateState,
attackProgram and defenceProgram was commented out during the 
competition when the ultrasonics and ball possession detection
proved unrealiable due to electronics issues.
---------------------------------------------------------------*/

void findBall()
{
    //checkForBall();
    //ballRollerOn();
    // we've found the ball and have it, our job is done
    /*if (state.hasBall)
    {
        state.currentTask = NONE;
        return;
    }*/

    const int maxRotSpeed = 90;
    const int slowRotSpeed = 30;
    const int maxSpeed = 20;
    const int slowSpeed = 10;
    
    const int ballAngleTol = 2;
    const int bigAngle = 10;

    bool pointingAtBall = abs(state.ballAngle) <= ballAngleTol;

    // stop any rotation if we are pointing at the ball
    if (pointingAtBall)
    {
        if(state.angularVelocity != 0)
        {
            clearMovementQueue();
            platform.stop();
            state.angularVelocity = 0;
        }
        if (platform.readyForNextMove())
            lodgeNewMovement(movement{STRAIGHT_LINE, 0, maxSpeed, 20});
    }
    
    else if (platform.readyForNextMove())
    {
        // if there is a big difference between sensors, rotate quickly, big angle
        if (abs(state.ballAngle) > bigAngle)
        {
            lodgeNewMovement(movement{ROTATION, state.ballAngle, maxRotSpeed, 0});
        }
        // small difference, so slowly rotate out a small angle
        else
        {
            lodgeNewMovement(movement{ROTATION, state.ballAngle, slowRotSpeed, 0});
        }
    }
}

void moveAttackSide()
{
    const int alignmentTol = 25;
    if (platform.readyForNextMove() && abs(state.facing) > alignmentTol)
    {
        // have to set steps for each motor directly, as this is an
        //   odd, crescent-shaped move which doesn't fit either straightLine
        //   or rotate
        platform.move(250,1,250,0,1800,0,2600);
    }
    // we are now pretty much aligned with enemy goals, just a tiny
    //   movement now to bring us into line
    else if (abs(state.facing) < alignmentTol)
    {
        platform.stop();
        clearMovementQueue();
        state.currentTask = NONE;
        platform.move(25,0,25,1,180,1,260);
    }
}

void dribbleBall()
{
    // head towards enemy goals, evade robots perhaps
    // change state.currentTask to shoot ball if lined up to shoot
}

void shootBall()
{
    static bool calledAlready = false;
    if (!calledAlready)
    {
        straightMovement(0, 30, 35);
        calledAlready = true;
    }
    if (!platform.currentlyMoving() && calledAlready)
    {
        calledAlready = false;
        state.currentTask = NONE;
    }
    ballRollerOff();
}

void forceUpdatePosition()
{
    int angleOffOrthog = (state.heading + 180) % 90;
    rotateMovement(-angleOffOrthog, 0);
}

void ballRollerOn()
{
    digitalWrite(ROLLER_PIN1, HIGH);
    digitalWrite(ROLLER_PIN2, LOW);
    state.rollerOn = true;
}

void ballRollerOff()
{
    digitalWrite(ROLLER_PIN1, LOW);
    digitalWrite(ROLLER_PIN2, LOW);
    state.rollerOn = false;
}

void updateState()
{
    state.colourUnderneath = getLineColour();

    // extrapolated position update, update state needs to be 
    //   called regularly for this to be effective. Extrapolating
    //   a position before getting position from the ultrasonics
    //   allows the ultrasonics able to guess where obstructions are
    //   the ultrasonics, if they can get a good enough reading,
    //   will update the position to be more accurate than the rough
    //   guess which has been extrapolated
    /*
    if (platform.currentlyMoving())
    {
        extrapolatePosition();
    }
    else 
    {
        state.speed = 0;
        state.heading = 0;
        state.angularVelocity = 0;
    }
    */

    state.facing = HMC5883L.getRelativeBearing();

    /*
    int position[2];
    ultrasonics.getPosition(position, state.facing, state.position);

    // only update position if a valid position has been found

    if (position[0] != -1)
        state.position[0] = position[0];
    if (position[1] != -1)
        state.position[1] = position[1];

    checkForBall();
    if (state.hasBall) 
        state.ballAngle = 0;
    else
    */
          state.ballAngle = getBallAngle();
}

void attackProgram()
{
    static uint32_t timeLastBack = 0;
    
    bool outOfBounds = state.colourUnderneath == WHITE; //|| state.position[1] < 20; //state.colourUnderneath == BLACK ||  

    // keep us in bounds
    if (outOfBounds)
    {
        // de bounce the reversal, only want to do it once
        if (millis() - timeLastBack > 1000)
        {
            timeLastBack = millis();
            platform.stop();
            clearMovementQueue();
            int reverseDir = state.heading + 180 - state.facing;
            // keep angle in range [-180,180]
            if (reverseDir < -180) reverseDir += 360;
            else if (reverseDir > 180) reverseDir -= 360;

            lodgeNewMovement(movement{STRAIGHT_LINE, reverseDir, 20, 10});
        }
    }    

    /*---------------------------------------------------
    the commented out stuff below is how the attack program 
    is meant to operate, but was reduced down to only findBall
    during the competition to simplify the program (which has bugs)
    and to work around broken modules, such as detecting when
    the ball was in the roller.
    ----------------------------------------------------*/
    findBall();

    //else if(platform.readyForNextMove())
    //{
        //const int enemyGoalAngleTol = 20;
        //if (!state.hasBall)
        //    state.currentTask = FIND_BALL;
        //else
       // {
            /* --- for future ----
            when we have ball: 
            
            if too far from enemy goals/not aligned:
              dribble it towards enemy goals, perhaps
              backwards to shield from enemy bot IR receivers

            else:
              we are close enough to shoot, so turn around and shoot
            ---------------------*/
       //     if (abs(state.facing) < enemyGoalAngleTol)
         //       state.currentTask = SHOOT_BALL;
        //    else
          //      state.currentTask = MOVE_ATTACK_SIDE;
       // }
       // if (abs(state.ballAngle) >= 180)
       // {
       //     lodgeNewMovement(movement{STRAIGHT_LINE, 180, 150, 30});
       //     state.currentTask = NONE;
       // }
       // else
       // {
            //state.currentTask = FIND_BALL;
            //Serial.println(String(state.ballAngle));
        // }

    //}


    /*------------------------------------------------------
    with the robot not being able to detect when it had possession
    of the ball, it was thought best to make the robot periodically
    re-align with the enemy goals to stop own goals being accidentally
    shot while looking for the ball. Due to some bug, this bit of 
    code didn't appear to have much effect.
    ------------------------------------------------------*/
    static uint32_t timeLastHeadingCorrection = 0;
    // check alignment with enemy goals, if out of tolerance, correct this
    if (state.facing > 10 || state.facing < -10)
    {
        if ((millis() - timeLastHeadingCorrection) > 5000)
        {
            lodgeNewMovement(movement{ROTATION, -state.facing, 140, 0});
            timeLastHeadingCorrection = millis();
        }
    }

}

void defenceProgram()
{
    const int maxRotSpeed = 160;
    const int slowRotSpeed = 30;
    const int fastMoveSpeed = 30;
    const int slowMoveSpeed = 10;
    const int smallMoveDist = 5;
    const int bigMoveDist = 15;

    const int ballAngleTol = 2;
    // angles smaller than this are considered small enough that
    //   rotations should be slowed down
    const int smallBallAngle = 5; 

    // check to see if on black line
    //   if so, hard stop, move a small distance back the way we came
    //   otherwise, move to side to be in line with ball

    static uint32_t timeLastBack = 0;
    
    bool outOfBounds = state.colourUnderneath == BLACK || state.colourUnderneath == WHITE;

    // keep us in bounds
    if (outOfBounds)
    {
        // de bounce the reversal, only want to do it once
        if (millis() - timeLastBack > 1000)
        {
            timeLastBack = millis();
            platform.stop();
            clearMovementQueue();
            int reverseDir = state.heading + 180 - state.facing;
            if (reverseDir < -180) reverseDir += 360;
            else if (reverseDir > 180) reverseDir -= 360;

            lodgeNewMovement(movement{STRAIGHT_LINE, reverseDir, 20, 10});
        }
    }    
    // if not out of bounds, do goalkeeping
    else if (platform.readyForNextMove())
    {
        int angleToBall = state.ballAngle;

        bool pointingAtBall = angleToBall < ballAngleTol && angleToBall > -ballAngleTol;

        if (pointingAtBall)
        {
            // the true parameter indicates it should be a decelerated move rather 
            //   than instant stop
            platform.stop(true);
            clearMovementQueue();
        }
        // ball not in view
        else if (abs(angleToBall) >= 180)
        {
            lodgeNewMovement(movement{ROTATION, angleToBall, maxRotSpeed, 0});
        }
        else
        {
            int speed = abs(angleToBall) > smallBallAngle? fastMoveSpeed : slowMoveSpeed;
            int distance = abs(angleToBall) > smallBallAngle? bigMoveDist : smallMoveDist;

            if (angleToBall < 0)
            {
                // go left
                lodgeNewMovement(movement{STRAIGHT_LINE, -90, speed, distance});
            }
            else
            {
                // go right
                lodgeNewMovement(movement{STRAIGHT_LINE, 90, speed, distance});
            }
        }
    }

    static uint32_t timeLastHeadingCorrection = 0;
    // check alignment with enemy goals, if out of tolerance, correct this
    if (!(state.facing < 5 && state.facing > -5))
    {
        if ((millis() - timeLastHeadingCorrection) > 5000)
        {
            lodgeNewMovement(movement{ROTATION, -state.facing, slowRotSpeed, 0});
            timeLastHeadingCorrection = millis();
        }
    }
}


// related subroutines/dependencies

void checkForBall()
{
    const int ballTimeout = 1500;
    bool gotBall = true;
    uint32_t timeNow = millis();

    // measure motor current, compare to a calibrated value for motor under no load
    const int numSamples = 4;
    for (int i=0; i<numSamples; i++)
    {
        // ALL samples must be significant for a positive
        int reading = analogRead(ROLLER_SENSE_PIN);
        if (!(rollerRunningNoBall - reading > rollerBallDetectionThreshold) || reading < 700)
            gotBall = false;
    }
        
    // will only update to not having the ball if we haven't made contact
    // for a certain amount of time, this is a kind of debounce
    if (gotBall)
    {
        state.lastHadBall = timeNow;
        state.hasBall = gotBall;
    }
    else if (timeNow - state.lastHadBall > ballTimeout)
    {
        // don't have ball and have timed out
        state.hasBall = false;
    }
}

void extrapolatePosition()
{
    uint32_t timeNow = millis();
    int dx = int(float(state.speed) * sin(float(state.heading)/57.2958) * (float(timeNow - state.lastPosUpdateTime)/1000.0) + 0.5),
        dy = int(float(state.speed) * cos(float(state.heading)/57.2958) * (float(timeNow - state.lastPosUpdateTime)/1000.0) + 0.5);

    state.position[0] += dx;
    state.position[1] += dy;

    state.lastPosUpdateTime = timeNow;
}
