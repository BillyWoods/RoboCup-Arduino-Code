#include "Robot.h"

extern drive_train platform;
extern robotState state;

bool USE_ULTRASONIC_BOUNDING = false;

queue<struct movement> movement_queue(MOVE_QUEUE_SIZE);

bool lodgeNewMovement(struct movement move)
{
    return movement_queue.append(&move);
}

void clearMovementQueue()
{
    movement_queue.reset();
    state.speed = 0;
    platform.stop();
}

void movementQueueCheck()
{
    struct movement* newMove = movement_queue.peek();
    if (newMove)
    {
        if (platform.readyForNextMove())
        {
            switch(newMove->type)
            {
                case STRAIGHT_LINE:
                    if (straightMovement(newMove->angle, newMove->distance, newMove->speed))
                        movement_queue.getNext(); // clears current move off, as we will actually use it
                    break;
                case ROTATION:
                    if (rotateMovement(newMove->angle, newMove->speed))
                        movement_queue.getNext(); // clears current move off, as we will actually use it
                    break;
            }
        }
    }
}

// angle is relative to front of robot, i.e. 0 is straight ahead
bool straightMovement(int angle, unsigned int distance, unsigned int speed)
{
    const int angleTol = 20; // decides if moving in same direction, i.e. an extension move
    int tempHeading = angle + state.facing; 
    bool extensionMove = platform.readyForNextMove() && abs(tempHeading - state.heading) < angleTol;

    if (tempHeading < -180)
       tempHeading += 360;
    else if (tempHeading > 180)
        tempHeading -= 360;

    // make sure we won't go out of bounds
    const int xFieldMin = (FIELD_X - INNER_FIELD_X)/2, 
              xFieldMax = (FIELD_X + INNER_FIELD_X)/2,
              yFieldMin = (FIELD_Y - INNER_FIELD_Y)/2, 
              yFieldMax = (FIELD_Y + INNER_FIELD_Y)/2;

    bool useGoalieBounding = state.strategy == DEFENCE;

    int xMin = useGoalieBounding ? GOAL_X_MIN : xFieldMin,
        xMax = useGoalieBounding ? GOAL_X_MAX : xFieldMax,
        yMin = useGoalieBounding ? GOAL_Y_MIN : yFieldMin,
        yMax = useGoalieBounding ? GOAL_Y_MAX : yFieldMax;
              
    int xFinish = state.position[0] + int(float(distance) * sin(float(tempHeading)/57.2958)),
        yFinish = state.position[1] + int(float(distance) * cos(float(tempHeading)/57.2958));

    // our move will finish inside bounds, so execute it
    //  or we're not using ultrasonic bounding, so go right ahead
    if(!(USE_ULTRASONIC_BOUNDING) ||
       (xFinish >= xMin && xFinish <= xMax &&
       yFinish >= yMin && yFinish <= yMax) )
    {
        if (extensionMove || !platform.currentlyMoving())
        {
            state.heading = tempHeading;
            state.speed = speed;
            state.angularVelocity = 0;

            platform.straightLine(angle, distance, speed, extensionMove);
            return true;
        }
    }
    // returning false indicates that the requested movement could not be made
    return false;
}

bool rotateMovement(int angle, unsigned int speed)
{
    const int angularSpeedTol = 40;
    int angularVelocity = (angle/abs(angle)) * speed;
    bool extensionMove = platform.readyForNextMove() && 
                         abs(angularVelocity - state.angularVelocity) < angularSpeedTol;
    
    if (extensionMove || !platform.currentlyMoving())
    {
        // no linear speed, only angular
        state.speed = 0;
        state.heading = 0;
        state.angularVelocity = angularVelocity;

        platform.rotate(angle, speed, extensionMove);
        return true;
    }
    return false;
}
