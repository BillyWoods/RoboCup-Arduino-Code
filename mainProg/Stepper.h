#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

#define NUM_AXES 3
#define CLOCK_SPEED 16.0f // in megahertz

#define ACCELERATION 30 // reduction in period time on major axis, per step
#define INITIAL_PERIOD 8000

// a hacky way of making the interrupt go faster, will break 
//  portability and ability to simply change step pins in constructor
//  see quickStepPin(int pin) and quickStepPinsLow() in the cpp file 
//  to see which step pins are being accessed
#define USE_DIRECT_PORT_ACCESS

// only make one instance of this class, also make sure movements are such
// that at least one stepper receives a pulse every 4 seconds
class drive_train
{
    private:
        const float wheelCircumference;
        const unsigned int bodyCircumference;
        const int stepsRev;
        int stepsPerCM;

        // [left, right, back]
        const int stepPins[NUM_AXES];
        const int dirPins[NUM_AXES];

        //---- for plain movements (no acceleration)----
        // us between steps on these axes, is used to select major axis to drive
        unsigned long int periods[NUM_AXES]; 
        // keep track of this so the interrupt knows when a step is overdue on the non-major axes
        volatile unsigned long int lastStepTime[NUM_AXES];
        // need to keep track of remaining steps so timer interrupt knows when to disable itself
        volatile unsigned int stepsRemaining[NUM_AXES];
        uint8_t majorAxis;

        /*-------------------------
        vars used for acceleration
        --------------------------*/
        // if the speed of movement > min initial velocity, this is set
        bool useAcceleration;
        volatile bool onRamp, offRamp;
        volatile unsigned long int curPeriods[NUM_AXES];
        unsigned int periodDeltas[NUM_AXES];
        volatile unsigned int rampSteps; // number of steps spent in acceleration, used so we know when to start deceleration

        // for timer setup, decides on a prescaler and number of counts
        //   so that the desired period is achieved. Will set the prescaler 
        //   bits in the timerBConfig passed to it
        uint16_t getCompCount(unsigned long int period, uint8_t* timerConfigB);

    public:
        // interrupt-called stuff
        static drive_train* self;
        // send a 1us long (min) pulse to the corresponding axes' pins
        // does this based off time elapsed since last step, this is also the TIMER1_COMPA handler
        void doStep();

        /*-------------------
            USER FUNCTIONS
        ---------------------*/

        // wheel circumference in cm, steps per revolution
        drive_train(const float _wheelCircumference, const unsigned int _bodyCircumference, const int _stepsRev, 
                    const int _ls, const int _ld, const int _rs, const int _rd, const int _bs, const int _bd);

        // call this in the setup loop to set pin modes
        void init();

        bool currentlyMoving();
        // goes to true shortly before off ramp starts, useful for trying to extend moves
        bool readyForNextMove();

        // moves all motors a certain number of steps in a certian time
        // time period is time for whole move in ms, this is safe to call in the middle of other moves
        // to override them, a dir of 0 makes the motors rotate clockwise, and 1 makes it move anticlockwise
        // isExtension is set to avoid the acceleration on ramp, this is useful when 'extending' a current
        // movement, i.e. lodging a new movement in a similar direction and speed to the present one
        void move(unsigned int lsteps, bool ldir, unsigned int rsteps, bool rdir, 
                  unsigned int bsteps, bool bdir, unsigned long int time, bool isExtension = false);

        // angle in degrees (-180, 180], distance in cm, moves are relative, speed in cm/second
        void straightLine(int angle, unsigned int distance, unsigned int speed, bool isExtension = false);
        // speed in degrees per second
        void rotate(int angle, unsigned int speed, bool isExtension = false);
        
        void stop(bool withDeccel = false);
};

#endif
