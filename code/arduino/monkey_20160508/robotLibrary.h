#ifndef INCL_ROBOTLIBRARY_H
#define INCL_ROBOTLIBRARY_H

#define numServos 6
#include "string_functions.h"
#define dataCommand "DATA"
#define dataRequest "GET"
#define uiRequest "UI_DESCRIPTION"
#include "arduino.h"
#define DO 0
#define DI 1
#define AO 2
#define AI 3
#define PWM 4
#define SERVO 5
#define numPins 28
#define NUM_DATA_OUTPUTS 8
#define DATA_OUTDEGREE 1

extern int servoPins[numServos];

bool bluetoothAvailable();
bool getBluetoothData();
bool isBluetoothConnected();
bool readAI(int pin, int threshold);
bool readDI(int pin);
bool setAngle(int servoNum, int angle);
char getBluetoothChar();
char* getData(int sourceID);
char* getData(int sourceID, int destID);
int angleToDuty(double angle, int pin);
int getPinMateType(int pin);
int getPinType(int pin);
int readAI(int pin);
int setPWMFrequency(int pin, long frequency);
long getPWMFrequency(int pin);
void clear(int pin);
void processBluetoothData();
void processData();
void processData(const char* data, int destID);
void processData(const char* data, int sourceID, int destID);
void processData(const char* data, int sourceID, int* destIDs, int numDestIDs);
void robotLoop();
void robotSetup();
void sendBluetoothChar(char toSend);
void sendBluetoothData(const char* data);
void set(int pin);
void setDO(int pin, bool setHigh);
void setPinMode(int pin, int mode);
void setPWM(int pin, int duty);

#endif
