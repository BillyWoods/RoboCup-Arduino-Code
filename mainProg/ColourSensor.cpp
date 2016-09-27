#include "ColourSensor.h"

const int lineTols[6] =
{
    GREEN_MIN,
    GREEN_MAX,
        
    BLACK_MIN,
    BLACK_MAX,
        
    WHITE_MIN,
    WHITE_MAX    
};

void initColourSensor()
{
    pinMode(COLOUR_SENSOR_PIN, INPUT);
    digitalWrite(COLOUR_SENSOR_PIN, HIGH);
}

inline bool readingIsInRange(int reading, int colour)
{
    return (reading >= lineTols[2 * colour] && reading <= lineTols[2 * colour + 1]);
}

int getLineColour()
{
    // average the reading
    int reading = analogRead(COLOUR_SENSOR_PIN);
    //reading += analogRead(COLOUR_SENSOR_PIN);
    //reading >>= 1;

    if (readingIsInRange(reading, GREEN))
        return GREEN;
    else if (readingIsInRange(reading, BLACK))
        return BLACK;
    else if (readingIsInRange(reading, WHITE))
        return WHITE;
    else
        return -1;
}
