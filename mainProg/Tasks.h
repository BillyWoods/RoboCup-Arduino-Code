#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>

enum tasks
{
    NONE,

    DEBUG_IR, // also gives info on ball detection using roller/microswitch
    DEBUG_COMPASS,
    DEBUG_ULTRASONICS,
    DEBUG_BALL_ROLLER,
    DEBUG_MACHINE_STATE,

    CALIBRATE_COMPASS,
    CALIBRATE_BALL_ROLLER,

    TEST_PATTERN_1,

    FIND_BALL,        // rotate to find ball, then move to it, with updates along the way
    MOVE_ATTACK_SIDE, // move around the ball so that we are now facing the enemy goals, with the ball
    DRIBBLE_BALL,
    SHOOT_BALL,       // dribbles towards the other team's goals, will use current position to angle correctly, also stops when close to goals

    FORCE_UPDATE_POSITION   // will force a position update by aligning with field, normally position is updated opportunistically
};


/* ----------------------------------------------
    tasks available over serial and also used by
    attack program, no need to call these directly
    most times
------------------------------------------------*/
// sensor debuggers
void debugIR();
void debugCompass();
void debugUltrasonics();
void debugBallRoller();
void debugMachineState();

void calibrateCompass();
void saveCompassCalibrationToEeprom();
void getCompassCalibrationFromEeprom();
void calibrateBallRoller();

void testPattern1();

// used by attack program
void findBall();
void moveAttackSide();
void dribbleBall();
void shootBall();

void forceUpdatePosition();
void ballRollerOn();
void ballRollerOff();

// ----------------------------
// subroutines not part of the current task system and not
//   available over serial
//-----------------------------
// note that check for ball will detect false positives on startup as motor
//   current draw is large
void checkForBall();
void extrapolatePosition();
// sets state.currentTask based on serial commands or just runs smaller
//   functions which do not require being looped
void handleSerialCommand(String _cmd);
// call this in main loop, will run correct functions for current state
void taskArbiter();

void updateState();
void attackProgram();
void defenceProgram();

#endif
