#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H

#define COLOUR_SENSOR_PIN A8

#include <Arduino.h>

enum linecolours
{
    GREEN = 0,
    BLACK = 1,
    WHITE = 2
};

const int GREEN_MIN = 69,
          GREEN_MAX = 350,
         
          BLACK_MIN = 351,
          BLACK_MAX = 1000,
         
          WHITE_MIN = 10,
          WHITE_MAX = 68;

void initColourSensor();
int getLineColour();

#endif
