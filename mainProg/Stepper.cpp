#include "Stepper.h"

#include <math.h>

drive_train* drive_train::self;

drive_train::drive_train(const float _wheelCircumference, const unsigned int _bodyCircumference, const int _stepsRev, 
    const int _ls, const int _ld, const int _rs, const int _rd, const int _bs, 
    const int _bd)
    
    :wheelCircumference(_wheelCircumference), bodyCircumference(_bodyCircumference), stepsRev(_stepsRev), 
    stepPins{_ls, _rs, _bs}, dirPins{_ld, _rd, _bd}
{
    stepsPerCM = int(float(stepsRev)/wheelCircumference + 0.5);
    stepsRemaining = {0,0,0};
    majorAxis = 0;
    self = this;
}

void drive_train::init()
{
    for(int i = 0; i < NUM_AXES; i++)
    {
        pinMode(stepPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);
    }
}

bool drive_train::currentlyMoving()
{
    // steps remaining and interrupt stil enabled
    return stepsRemaining[majorAxis] > 0 && (TIMSK1 & _BV(OCIE1A));
}

bool drive_train::readyForNextMove()
{
    return ((stepsRemaining[majorAxis] <= rampSteps + 20) && useAcceleration) 
            || (stepsRemaining[majorAxis] <= 20) || !currentlyMoving();
}

inline void quickStepPin(int pin)
{
    if (pin == 0) // left stepper
        PORTL |= B01000000; // PL6
    else if (pin == 1) // right stepper
        PORTL |= B00000100; // PL2
    else if (pin == 2) // back stepper
        PORTB |= B00000100; // PB2
}

inline void quickStepPinsLow()
{
    PORTL &= ~B01000100;
    PORTB &= ~B00000100;
}

// will perform stepping based off the values of periods and lastStepTime
void drive_train::doStep()
{
    unsigned long int timeNow = micros();
    for (int i = 0; i < NUM_AXES; i++)
    {
        // check if axis is overdue for a step, also have a catch for if
        // period = 0, which is intended to indicate no stepping, rather
        // than stepping every cycle, the +50 is a tolerance value
        if (curPeriods[i] != 0 && (timeNow - lastStepTime[i] + 50 >= curPeriods[i]))
        {
            #if defined USE_DIRECT_PORT_ACCESS
                quickStepPin(i);
            #else
                digitalWrite(stepPins[i], HIGH);
            #endif
            lastStepTime[i] += curPeriods[i];
            stepsRemaining[i] -= 1;
        }
    }

    if (useAcceleration)
    {
        offRamp = stepsRemaining[majorAxis] <= rampSteps &&
                  curPeriods[majorAxis] < INITIAL_PERIOD;

        // for onRamp, it is important to check that more than rampSteps steps 
        //   are remaining, otherwise may begin accelerating again
        onRamp = !(stepsRemaining[majorAxis] <= rampSteps) && 
                 curPeriods[majorAxis] > periods[majorAxis];

        if (onRamp)
            for (int j = 0; j < NUM_AXES; j++)
                curPeriods[j] -= periodDeltas[j]; 
        else if (offRamp)
            for (int j = 0; j < NUM_AXES; j++)
                curPeriods[j] += periodDeltas[j];
    }
    
    // disable interrupt if move is complete
    if (stepsRemaining[majorAxis] <= 0)
    {
        TIMSK1 &= ~_BV(OCIE1A);
    }

    #if defined USE_DIRECT_PORT_ACCESS
        quickStepPinsLow();
    #else
        for (int i = 0; i < NUM_AXES; i++)
        {
            digitalWrite(stepPins[i], LOW);
        }
    #endif
}

uint16_t drive_train::getCompCount(unsigned long int period, uint8_t* timerConfigB)
{
    const uint8_t prescalerBitmasks[5] = 
    {
        0x01,  // /1 (clock freq)
        0x02,  // /8
        0x03,  // /64
        0x04,  // /256
        0x05   // /1024
    };
    const float usPerClock[5] = 
    {
        (1   ) / CLOCK_SPEED,
        (8   ) / CLOCK_SPEED,
        (64  ) / CLOCK_SPEED,
        (256 ) / CLOCK_SPEED,
        (1024) / CLOCK_SPEED
    };

    // see if period will fit within smallest clock increments first
    int chosenPrescaler = 0;
    for (; chosenPrescaler < 5;)
    {
        // max count for 16 bit counter is 0xFFFF
        if ( !(period > (uint32_t)((usPerClock[chosenPrescaler])*0xFFFF + 0.5)) )
        {
            break;
        }
        chosenPrescaler++;
    }

    (*timerConfigB) |= prescalerBitmasks[chosenPrescaler];
    return (uint16_t)( period / usPerClock[chosenPrescaler] + 0.5);
}

void drive_train::move(unsigned int lsteps, bool ldir, unsigned int rsteps, bool rdir, 
    unsigned int bsteps, bool bdir, unsigned long int time, bool isExtension)
{
    cli();
    
    periods[0] = (lsteps > 0)? (uint32_t)((time*1000.0)/lsteps) : 0;
    periods[1] = (rsteps > 0)? (uint32_t)((time*1000.0)/rsteps) : 0;
    periods[2] = (bsteps > 0)? (uint32_t)((time*1000.0)/bsteps) : 0;

    stepsRemaining[0] = lsteps;
    stepsRemaining[1] = rsteps;
    stepsRemaining[2] = bsteps;

    unsigned long int shortestPeriod = 0xFFFFFFFF;
    majorAxis = 0;
    for (int i = 0; i<NUM_AXES; i++)
    {
        if (periods[i] != 0 && periods[i] < shortestPeriod)
        {
            shortestPeriod = periods[i];
            majorAxis = i;
        }
    }

    // set direction pins accordingly
    digitalWrite(dirPins[0], ldir);
    digitalWrite(dirPins[1], rdir);
    digitalWrite(dirPins[2], bdir);

    offRamp = false;
    onRamp = false;
    // use acceleration only on fast movements
    useAcceleration = shortestPeriod < INITIAL_PERIOD;
    if (useAcceleration)
    {
        // no on ramp for extension moves, though off ramp is ok
        onRamp = !(isExtension);
        rampSteps = int(float(INITIAL_PERIOD - shortestPeriod)/float(ACCELERATION) + 0.5);

        for (int i = 0; i<NUM_AXES; i++)
        {
            float periodRatio = float(periods[i])/float(shortestPeriod);
            curPeriods[i] = uint32_t( periodRatio * float(INITIAL_PERIOD) + 0.5);
            periodDeltas[i] = uint32_t(float(curPeriods[i] - periods[i])/float(rampSteps) + 0.5 );

            // full speed, no on ramp acceleration if this is an extension
            // (still needed the period deltas calculated for the off ramp, hence why
            //  curPeriod was initially calculated as if for acceleration)
            if (isExtension)
                curPeriods[i] = periods[i];
        }
    }
    else
    {
        for (int i = 0; i<NUM_AXES; i++)
        {
            curPeriods[i] = periods[i];
            periodDeltas[i] = 0;
        }
    }

    unsigned long int timeNow = micros();
    for (int i = 0; i<NUM_AXES; i++)
        lastStepTime[i] = timeNow;

    // setup timer interrupt for shortest period, this interrupt will call step()
    // turn off timer's clock and also put it into CTC mode
    TCCR1B = 0x08;
    // reset counter
    TCNT1 = 0;
    // attach no pins to counter 1's triggering and disable waveform generation
    TCCR1A = 0;
    uint8_t timerClockBits = 0;
    // set prescale bits on config reg B and also set comp A counter
    OCR1A = getCompCount(shortestPeriod, &timerClockBits);
    // set config register B
    TCCR1B |= timerClockBits;
    // enable timer 1 interrupt based off comp a
    TIMSK1 |= _BV(OCIE1A);
    sei();
}

// the magic (not really) interrupt routine, runs in about 75us
//   that may be due mostly to 6 digitalWrite() calls
ISR(TIMER1_COMPA_vect)
{
    drive_train::self->doStep();
}

// solves for the three wheel holonomic drive
void drive_train::straightLine(int angle, unsigned int distance, unsigned int speed, bool isExtension)
{
    // vector for movement in forwards direction
    float a = sin(1.0472+(float(angle)/57.2958)) * 1.1547;
    // vector for movement at 120 degrees
    float b = sin(float(angle)/57.2958) * 1.1547;

    // speeds in cm/s at the tyre circumference
    // LSpeed = 1.1547 * a * speed;
    // RSpeed = 1.1547 * (a - b) * speed;
    // BSpeed = 1.1547 * b * speed;

    // anti clockwise if a > 0
    bool LDir = a > 0? 1:0;
    // clockwise when positive a is dominant, anti-clockwise when b is dominant
    bool RDir = a - b > 0? 0:1;
    // clockwise if b > 0
    bool BDir = b > 0? 0:1;

    unsigned long int time = int(float(distance*1000.0)/float(speed) + 0.5);
   
    unsigned int LSteps = int(abs(a) * 1.1547 * stepsPerCM * distance + 0.5);
    unsigned int RSteps = int(abs(a - b) * 1.1547 * stepsPerCM * distance + 0.5);
    unsigned int BSteps = int(abs(b) * 1.1547 * stepsPerCM * distance + 0.5);

    move(LSteps, LDir, RSteps, RDir, BSteps, BDir, time, isExtension);
}

void drive_train::rotate(int angle, unsigned int speed, bool isExtension)
{
    unsigned int numSteps = int(float(abs(angle)/360.0) * bodyCircumference * stepsPerCM + 0.5);
    unsigned int time = int( (1000 * float(abs(angle)) )/speed + 0.5);//in ms
    bool dir = angle > 0? 1:0;
    move(numSteps, dir, numSteps, dir, numSteps, dir, time, isExtension);
}

void drive_train::stop(bool withDeccel)
{
    cli();
    if (withDeccel && useAcceleration)
    {
        // put us straight onto an off ramp to deccelerate
        stepsRemaining[majorAxis] = rampSteps;
        offRamp = true;
    }
    else
    {
        // come to a hard stop
        // disable timer 1 interrupt off of comp A
        TIMSK1 &= ~_BV(OCIE1A);
        stepsRemaining[majorAxis] = 0;
    }
    sei();
}
