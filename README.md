# RoboCup-Arduino-Code
The Arduino (C++) code which I ran on my soccer robot at the 2016 Australian national RoboCup competition.

# Modules which might be of interest for your own robotics projects:
Honeywell HMC5883L compass library with calibration - makes it possible to use seemingly-misbehaving compasses.

Stepper motor driving - drive three (extendable with a little work) stepper motors simultaneously, non-blocking, can do poorly-aproximated realtime acceleration

IR ball finding - get the direction of the ball as an angle relative to your robot, have a version which uses analog reading of IR demodulators with a low pass filter on their outputs, and another version which measures digital pulse output length from IR demodulators with pin change interrupts.
