#ifndef INCL_ROBOTLIBRARY_H
#define INCL_ROBOTLIBRARY_H

#define numServos 6
#include "string_functions.h"
#include "arduino.h"
#define DO 0
#define DI 1
#define AO 2
#define AI 3
#define PWM 4
#define SERVO 5
#define numPins 28

extern int servoPins[numServos];

int getPinMateType(int pin);
int getPinType(int pin);
int setPWMFrequency(int pin, long frequency);
long getPWMFrequency(int pin);
void robotSetup();
void setPinMode(int pin, int mode);
void setPWM(int pin, int duty);

#endif
