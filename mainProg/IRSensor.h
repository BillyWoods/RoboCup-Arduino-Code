#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

#define NUM_IRS 3
#define IR_SPACING_ANGLE 25

#define LEFT_IR_PIN A0
#define CENTRE_IR_PIN A3
#define RIGHT_IR_PIN A1

void initIRs();
void getIRReadings(int* IRArray);

// angle is relative to front of robot and is in range [-180,180]
//   will return a greater angle than the last known angle if ball not detected
int getBallAngle();

#endif
