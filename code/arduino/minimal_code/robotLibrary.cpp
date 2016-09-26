#include "robotLibrary.h"

int servoPins[numServos];
long PWMFrequency[3];
int controllerPins[] = {-1, 3, -1, -1, -1, -1, -1, -1, 5, -1, -1, -1, -1,
                        -1, -1, 6, -1, -1, -1, -1, -1, -1, 9, 10, 11, -1, -1, -1};
const char* pinTypes[] = {"DataInputPort", "ServoInputPort", "PowerInputPort", "Ground", "DataInputPort",
                          "DataOutputPort", "DataOutputPort", "DataInputPort", "ServoInputPort", "PowerInputPort",
                          "Ground", "DataInputPort", "DataOutputPort", "DataOutputPort", "DataInputPort",
                          "ServoInputPort", "PowerInputPort", "Ground", "DataInputPort", "DataOutputPort",
                          "DataOutputPort", "DataInputPort", "ServoInputPort",  "ServoInputPort",  "ServoInputPort",
                          "DataInputPort", "DataOutputPort", "DataOutputPort"};


int getPinMateType(int pin)
{
  if(contains(pinTypes[pin], "DigitalOutput"))
    return DI;
  else if(contains(pinTypes[pin], "DigitalInput"))
    return DO;
  else if(contains(pinTypes[pin], "PWMOutput"))
    return DI;
  else if(contains(pinTypes[pin], "AnalogOutput"))
    return AI;
  else if(contains(pinTypes[pin], "AnalogInput"))
    return PWM;
  else if(contains(pinTypes[pin], "ServoInput"))
    return SERVO;
  return -1;
}

int getPinType(int pin)
{
  if(contains(pinTypes[pin], "DigitalOutput"))
    return DO;
  else if(contains(pinTypes[pin], "DigitalInput"))
    return DI;
  else if(contains(pinTypes[pin], "PWMOutput"))
    return PWM;
  else if(contains(pinTypes[pin], "AnalogOutput"))
    return AO;
  else if(contains(pinTypes[pin], "AnalogInput"))
    return AI;
  else if(contains(pinTypes[pin], "Servo"))
    return SERVO;
  return -1;
}

// Set the PWM Frequency for the given pin
// Will return the frequency achieved, or -1 if arguments are invalid
int setPWMFrequency(int pin, long frequency)
{
  byte mode;
  long baseFrequency;
  if(pin == 3 || pin == 9 || pin == 10 || pin == 11)
      baseFrequency = 31250;
  if(pin == 5 || pin == 6)
      baseFrequency = 62500;
  long error = baseFrequency;
  int divisor = 1;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    int divisors[] = {1,8,64,256,1024};
    for(int i = 0; i < 5; i++)
    {
      long newError = frequency - baseFrequency / divisors[i];
      newError *= newError < 0 ? -1 : 1;
      if(newError < error)
      {
        error = newError;
        divisor = divisors[i];
      }
    }
    Serial.print("divisor: "); Serial.println(divisor);
    Serial.print("frequency: "); Serial.println(baseFrequency / divisor);
    Serial.print("error: "); Serial.println(error);
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return -1;
    }
    if(pin == 5 || pin == 6)
      TCCR0B = TCCR0B & 0b11111000 | mode;
    else
      TCCR1B = TCCR1B & 0b11111000 | mode;
  }
  else if(pin == 3 || pin == 11)
  {
    int divisors[] = {1,8,32,64,128,256,1024};
    for(int i = 0; i < 7; i++)
    {
      long newError = frequency - baseFrequency / divisors[i];
      newError *= newError < 0 ? -1 : 1;
      if(newError < error)
      {
        error = newError;
        divisor = divisors[i];
      }
    }
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return -1;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
  return baseFrequency / divisor;
}

long getPWMFrequency(int pin)
{
  switch(controllerPins[pin])
  {
    case 5:
    case 6:
      return PWMFrequency[0];
    case 9:
    case 10:
      return PWMFrequency[1];
    case 3:
    case 11:
      return PWMFrequency[2];
  }
}

void robotSetup()
{
  // Set each pin to the correct mode
  for(int pinIndex = 0; pinIndex < numPins; pinIndex++)
  {
    if(controllerPins[pinIndex] >= 0 && getPinType(pinIndex) >= 0)
      setPinMode(controllerPins[pinIndex], getPinMateType(pinIndex));
  }
  servoPins[0] = 1;
  setPWM(servoPins[0], 128); 
  servoPins[1] = 8;
  setPWM(servoPins[1], 128); 
  servoPins[2] = 15;
  setPWM(servoPins[2], 128); 
  servoPins[3] = 22;
  setPWM(servoPins[3], 128); 
  servoPins[4] = 23;
  setPWM(servoPins[4], 128); 
  servoPins[5] = 24;
  setPWM(servoPins[5], 128); 
}

void setPinMode(int pin, int mode)
{
  switch(mode)
  {
    case DO: pinMode(pin, OUTPUT); break;
    case DI: pinMode(pin, INPUT); break;
    case AO:
    case PWM: pinMode(pin, OUTPUT); break;
    case SERVO:
      pinMode(pin, OUTPUT);
      switch(pin)
      {
        case 5:
        case 6:
          PWMFrequency[0] = setPWMFrequency(pin, 980); break;
        case 9:
        case 10:
          PWMFrequency[1] = setPWMFrequency(pin, 480); break;
        case 3:
        case 11:
          PWMFrequency[2] = setPWMFrequency(pin, 480); break;
      }
      break;
    case AI: break;
  }
}

void setPWM(int pin, int duty)
{
  analogWrite(controllerPins[pin], duty);
}
