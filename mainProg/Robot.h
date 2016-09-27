#ifndef ROBOT_H
#define ROBOT_H

#include "Compass.h"
#include "Ultrasonic.h"
#include "Stepper.h"
#include "Queue.h"

#include "EEPROM.h"

#define ROLLER_SENSE_PIN A2

#define ROLLER_PIN1 6
#define ROLLER_PIN2 7

#define FIELD_X 182
#define FIELD_Y 240
#define INNER_FIELD_X 125
#define INNER_FIELD_Y 180
// a bounding box, used for goalie at present, only used if
//   USE_ULTRASONIC_BOUNDING is true and strategy is DEFENCE. 
// Note that the bounding box will take precendence over 
//   INNER_FIELD_[XY] bounding
#define GOAL_X_MIN 30
#define GOAL_X_MAX 150
#define GOAL_Y_MIN 30
#define GOAL_Y_MAX 60

#define STARTUP_X_POS 90
#define STARTUP_Y_POS 40

// defined in Robot.cpp, but declared here for other files which may
//  include this header
extern bool USE_ULTRASONIC_BOUNDING;
extern queue<struct movement> movement_queue;

// this queue is defined in Robot.cpp but never needs to be directly
//   accessed
#define MOVE_QUEUE_SIZE 4


enum EEPROM_MEMORY_MAP
{
    COMPASS_EEPROM_BEGIN = 0x00,
    COMPASS_EEPROM_END   = 0x00 + 6 + 12 - 1  // 3 ints stored for bias, 3 floats for scale factor
};

enum strategies
{
    DO_NOTHING,
    ATTACK,
    DEFENCE
};

struct robotState
{
    volatile int currentTask;
    int strategy;

    bool hasBall;
    int ballAngle;
    unsigned long int lastHadBall;
    bool rollerOn;
    bool canShoot;

    int position[2]; //x and y (cm) from bottom left corner of field (near our goals)
    int facing;      //-180 to 180 deg, 0 deg = pointed at opposition goals
    int heading;     //angle robot is currently moving along, y axis = 0 deg
    int speed;       //speed of linear motion in cm/s
    int angularVelocity; // degrees per second, can be positive or negative
    unsigned long int lastPosUpdateTime; // used for position extrapolation

    int colourUnderneath;
};

enum movementType
{
    STRAIGHT_LINE,
    ROTATION
};

struct movement
{
    movementType type;
    int angle;
    uint8_t speed;

    // only applies to straight movements
    uint8_t distance;
};

// adds a movement to the movement queue, returns true on success
bool lodgeNewMovement(struct movement move);
void clearMovementQueue();
// this function checks whether robot is ready to start next movement
//   and is meant to be called regularly
void movementQueueCheck();

// wrap stepper movement modules so that the state of the robot is
//   automatically updated and so allow position to be extrapolated, etc.
// also, knowing the speed, direction, etc. of robot through state, we can
//   string together similar moves so they avoid deccelerating and
//   re-accelertaing unecessarily.
// should only be called directly if you wish to jump the movement queue,
//   otherwise use lodgeMovement
// will return true if the move is accepted, false if rejected, e.g. 
//   if called before previous move has been finished or the
//   movement cannot extend the current move due to a large angle mismatch.
bool straightMovement(int angle, unsigned int distance, unsigned int speed);
bool rotateMovement(int angle, unsigned int speed);

#endif
