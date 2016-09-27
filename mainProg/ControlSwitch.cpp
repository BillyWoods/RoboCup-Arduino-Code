#include "ControlSwitch.h"
#include "Robot.h"
#include "Tasks.h"

#include <Arduino.h>

extern robotState state;

void initControlSwitch()
{
    pinMode(CONTROL_SWITCH_PIN, INPUT);
    // enable pullup
    digitalWrite(CONTROL_SWITCH_PIN, HIGH);
}

void updateFromSwitchState()
{
    bool currentState = digitalRead(CONTROL_SWITCH_PIN);

    // only change state stuff on changing, stops constant interference
    // stop when low 
    if (currentState == LOW && (state.strategy == DEFENCE || state.strategy == ATTACK))
    {
        state.strategy = DO_NOTHING;
        state.currentTask = NONE;
        clearMovementQueue();
        ballRollerOff();
        Serial.println("Doing nothing now");
    }
    // when high, play soccer
    else if (currentState == HIGH)
    {
#ifdef IS_DEFENCE_BOT
        //state.strategy = DEFENCE;
#else
        state.strategy = ATTACK;
#endif
    }
}
