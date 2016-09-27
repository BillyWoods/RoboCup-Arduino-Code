#ifndef CONTROL_SWITCH_H
#define CONTROL_SWITCH_H

#define CONTROL_SWITCH_PIN 13

// if left commented out, implies this is the attack robot
//#define IS_DEFENCE_BOT

void initControlSwitch();
void updateFromSwitchState();

#endif
