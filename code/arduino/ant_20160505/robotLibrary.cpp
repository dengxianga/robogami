#include "robotLibrary.h"

int servoPins[numServos];
char bluetoothData[50];
bool bluetoothConnected;
char* UI_DESCRIPTIONS[] = {"UI$front$UISlider$$curPosition,6$0,100,50", "UI$right$UISlider$$curPosition,13$0,100,50", "UI$back$UISlider$$curPosition,20$0,100,50", "UI$left$UISlider$$curPosition,27$0,100,50"};
bool validBluetoothData;










long PWMFrequency[3];
int controllerPins[] = {-1, 3, -1, -1, -1, -1, -1, -1, 5, -1, -1, -1, -1, -1, -1, 6, -1, -1, -1, -1, -1, -1, 9, 10, 11, -1, -1, -1};
const char* pinTypes[] = {"DataInputPort", "ServoInputPort", "PowerInputPort", "Ground", "DataInputPort", "DataOutputPort", "DataOutputPort", "DataInputPort", "ServoInputPort", "PowerInputPort", "Ground", "DataInputPort", "DataOutputPort", "DataOutputPort", "DataInputPort", "ServoInputPort", "PowerInputPort", "Ground", "DataInputPort", "DataOutputPort", "DataOutputPort", "DataInputPort", "ServoInputPort",  "ServoInputPort",  "ServoInputPort", "DataInputPort", "DataOutputPort", "DataOutputPort"};
const char* protocol[] = {"direct", "direct", "direct", "direct", "direct", "direct", "bluetooth", "direct", "direct", "direct", "direct", "dirnect", "direct", "bluetooth", "direct", "direct", "direct", "direct", "direct", "direct", "bluetooth", "direct", "direct", "direct", "direct", "direct", "direct", "bluetooth"};
int dataMapping[NUM_DATA_OUTPUTS][DATA_OUTDEGREE] = {{0}, {4}, {7}, {11}, {14}, {18}, {21}, {25}};
int dataOutputIDs[NUM_DATA_OUTPUTS] = {5, 6, 12, 13, 19, 20, 26, 27};
bool autoPoll[NUM_DATA_OUTPUTS][DATA_OUTDEGREE] = {{false}, {false}, {false}, {false}, {false}, {false}, {false}, {false}};
char outputData[50];
bool validGetData;


bool bluetoothAvailable()
{
  return bluetoothConnected;
}

bool getBluetoothData()
{
  if(!bluetoothAvailable())
    return false;
  if(validBluetoothData)
  {
    bluetoothData[0] = '\0';
    validBluetoothData = false;
  }
  int index = length(bluetoothData);
  while(bluetoothAvailable() && !validBluetoothData)
  {
    bluetoothData[index++] = getBluetoothChar();
    validBluetoothData = (bluetoothData[index-1] == '\0');
  }
  if(validBluetoothData)
  {
    // If it is a heartbeat, respond to it now
    if(equals(bluetoothData, "?"))
    {
      sendBluetoothData("?");
      bluetoothData[0] = '\0';
      validBluetoothData = false;
      return false;
    }
    Serial.print("Got BT data <"); Serial.print(bluetoothData); Serial.println(">");
  }
  return validBluetoothData;
}

bool isBluetoothConnected()
{
  return bluetoothConnected;
}

bool readAI(int pin, int threshold)
{
  return readAI(pin) > threshold;
}

bool readDI(int pin)
{
  return digitalRead(controllerPins[pin]);
}

bool setAngle(int servoNum, int angle) 
{
  setPWM(servoPins[servoNum], angleToDuty(angle, servoPins[servoNum]));
  //Serial.print("Set servo ");
  //Serial.print(servoNum);
  //Serial.print(" to angle "); Serial.println(angle);
  return true;
}

char getBluetoothChar()
{
}

char* getData(int sourceID)
{
  return getData(sourceID, -1);
}

char* getData(int sourceID, int destID)
{
  if(equalsIgnoreCase(protocol[sourceID], "bluetooth"))
  {
    // TODO control frequency of polling in a way that is fair to all polled inputs
    // and that also does not limit frequency of incoming requests
  
    // Request data from bluetooth
    // use format GET$outputPortID$inputPortID
    char toSend[50]; toSend[0] = '\0';
    strcpy(dataRequest, toSend, 50);
    concat(toSend, "$", toSend, 50);
    concatInt(toSend, sourceID, toSend, 50);
    concat(toSend, "$", toSend, 50);
    sendBluetoothData(toSend);
    // For now, pretend like it was an invalid getData
    // When the bluetooth controller responds with DATA command, it will be processed then
    outputData[0] = '\0';
    validGetData = false;
    return outputData;
  }
  if(sourceID == 5)
  {
    int input = (int) atof(getData(6));
    if(!validGetData)
    {
      outputData[0] = '\0';
      return outputData;
    }
    validGetData = true;
    itoa((int) (input), outputData, 10);
    return outputData;
  }
  if(sourceID == 12)
  {
    int input = (int) atof(getData(13));
    if(!validGetData)
    {
      outputData[0] = '\0';
      return outputData;
    }
    validGetData = true;
    itoa((int) (input), outputData, 10);
    return outputData;
  }
  if(sourceID == 19)
  {
    int input = (int) atof(getData(20));
    if(!validGetData)
    {
      outputData[0] = '\0';
      return outputData;
    }
    validGetData = true;
    itoa((int) (input), outputData, 10);
    return outputData;
  }
  if(sourceID == 26)
  {
    int input = (int) atof(getData(27));
    if(!validGetData)
    {
      outputData[0] = '\0';
      return outputData;
    }
    validGetData = true;
    itoa((int) (input), outputData, 10);
    return outputData;
  }
  outputData[0] = '\0';
  validGetData = false;
  return outputData;
}

int angleToDuty(double angle, int pin)
{
  double pwmPeriod = 1000.0/(double)getPWMFrequency(pin);
  double pulseWidth = angle/180.0 * (1.2 - 0.3) + 0.3;
  return pulseWidth / pwmPeriod * 255;
}

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

int readAI(int pin)
{
  return analogRead(controllerPins[pin]);
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

void clear(int pin)
{
  setDO(pin, 0);
}

void processBluetoothData()
{
  if(!getBluetoothData())
    return;
  if(indexOf(bluetoothData, uiRequest) >= 0)
  {
    for(int i = 0; i < 4; i++)
      sendBluetoothData(UI_DESCRIPTIONS[i]);
    sendBluetoothData("MAP$5-0$6-4$12-7$13-11$19-14$20-18$26-21$27-25$");
    return;
  }
  // Parse data
  bool isRequest = false;
  char data[10];  data[0] = '\0';
  if(indexOf(bluetoothData, dataCommand) >= 0)
    isRequest = false;
  else if(indexOf(bluetoothData, dataRequest) >= 0)
    isRequest = true;
  else
    return;
  bluetoothConnected = true;
  char outputIDChar[3]; outputIDChar[0] = '\0';
  char inputIDChar[3]; inputIDChar[0] = '\0';
  int index = 0;
  int btLength = length(bluetoothData);
  for(; index < btLength && bluetoothData[index] != '$'; index++);
  index++; // get passed first dollar sign
  if(!isRequest)
  {
    int dataIndex = 0;
    while(index < btLength && bluetoothData[index] != '$')
      data[dataIndex++] = bluetoothData[index++];
    data[dataIndex] = '\0';
    index++; // get passed second dollar sign
  }
  int idIndex = 0;
  while(index < btLength && bluetoothData[index] != '$')
    outputIDChar[idIndex++] = bluetoothData[index++];
  outputIDChar[idIndex] = '\0';
  index++;
  idIndex = 0;
  while(index < btLength && bluetoothData[index] != '$')
    inputIDChar[idIndex++] = bluetoothData[index++];
  inputIDChar[idIndex] = '\0';

  int outputID = length(outputIDChar) > 0 ? atoi(outputIDChar) : -1;
  int inputID = length(inputIDChar) > 0 ? atoi(inputIDChar) : -1;

  Serial.print("\tgot data <"); Serial.print(data); Serial.println(">");
  Serial.print("\tgot output <"); Serial.print(outputIDChar); Serial.print("> -> "); Serial.println(outputID);
  Serial.print("\tgot input <"); Serial.print(inputIDChar); Serial.print("> -> "); Serial.println(inputID);

  if(isRequest)
  {
    getData(outputID);
    if(validGetData)
    {
      char toSend[50]; toSend[0] = '\0';
      strcpy(dataCommand, toSend, 50);
      concat(toSend, "$", toSend, 50);
      concat(toSend, outputData, toSend, 50);
      concat(toSend, "$", toSend, 50);
      concatInt(toSend, outputID, toSend, 50);
      concat(toSend, "$", toSend, 50);
      concat(toSend, inputIDChar, toSend, 50);
      sendBluetoothData(toSend);
    }
  }
  else
  {
    if(inputID >= 0 && outputID < 0)
      processData(data, inputID);
    else if(inputID >= 0 && outputID >= 0)
      processData(data, outputID, inputID);
    else if(outputID >= 0)
    {
      // Find index in array of this output ID
      for(int i = 0; i < NUM_DATA_OUTPUTS; i++)
      {
        if(dataOutputIDs[i] == outputID)
          processData(data, outputID, dataMapping[i], DATA_OUTDEGREE);
      }
    }
  }
}

void processData()
{
  for(int dataOutput = 0; dataOutput < NUM_DATA_OUTPUTS; dataOutput++)
  {
    // If any of its inputs are set to autoPolling, get data from it and send to those inputs
    validGetData = false;
    for(int dataInput = 0; dataInput < DATA_OUTDEGREE && dataMapping[dataOutput][dataInput] >= 0; dataInput++)
    {
      if(autoPoll[dataOutput][dataInput])
      {
        if(!validGetData)
          getData(dataOutputIDs[dataOutput]);
        if(validGetData)
          processData(outputData, dataMapping[dataOutput][dataInput]);
      }

    }
  }
}

void processData(const char* data, int destID)
{
  return processData(data, -1, destID);
}

void processData(const char* data, int sourceID, int destID)
{
  if(destID == 0)
  {
    int angle = (int) atof(data);
    setAngle(0, angle);
  }
  if(equalsIgnoreCase(protocol[destID], "bluetooth"))
  {
    // TODO control frequency of polling in a way that is fair to all polled inputs
    // and that also does not limit frequency of incoming requests
  
    // Send data to bluetooth
    // use format DATA$data$outputPortID$inputPortID
    char toSend[50]; toSend[0] = '\0';
    strcpy(dataCommand, toSend, 50);
    concat(toSend, "$", toSend, 50);
    concat(toSend, data, toSend, 50);
    concat(toSend, "$", toSend, 50);
    concatInt(toSend, sourceID, toSend, 50);
    concat(toSend, "$", toSend, 50);
    concatInt(toSend, destID, toSend, 50);
    sendBluetoothData(toSend);
  }
  if(destID == 4)
  {
    int input = (int) atof(data);
    char outputData[10];
    itoa((int) (input), outputData, 10);
    for(int dataOutput = 0; dataOutput < NUM_DATA_OUTPUTS; dataOutput++)
    {
      if(dataOutputIDs[dataOutput] == 5)
        processData(outputData, dataOutput, dataMapping[dataOutput], DATA_OUTDEGREE);
    }
  }
  if(destID == 7)
  {
    int angle = (int) atof(data);
    setAngle(1, angle);
  }
  if(destID == 11)
  {
    int input = (int) atof(data);
    char outputData[10];
    itoa((int) (input), outputData, 10);
    for(int dataOutput = 0; dataOutput < NUM_DATA_OUTPUTS; dataOutput++)
    {
      if(dataOutputIDs[dataOutput] == 12)
        processData(outputData, dataOutput, dataMapping[dataOutput], DATA_OUTDEGREE);
    }
  }
  if(destID == 14)
  {
    int angle = (int) atof(data);
    setAngle(2, angle);
  }
  if(destID == 18)
  {
    int input = (int) atof(data);
    char outputData[10];
    itoa((int) (input), outputData, 10);
    for(int dataOutput = 0; dataOutput < NUM_DATA_OUTPUTS; dataOutput++)
    {
      if(dataOutputIDs[dataOutput] == 19)
        processData(outputData, dataOutput, dataMapping[dataOutput], DATA_OUTDEGREE);
    }
  }
  if(destID == 21)
  {
    int angle = (int) atof(data);
    setAngle(3, angle);
  }
  if(destID == 25)
  {
    int input = (int) atof(data);
    char outputData[10];
    itoa((int) (input), outputData, 10);
    for(int dataOutput = 0; dataOutput < NUM_DATA_OUTPUTS; dataOutput++)
    {
      if(dataOutputIDs[dataOutput] == 26)
        processData(outputData, dataOutput, dataMapping[dataOutput], DATA_OUTDEGREE);
    }
  }
  Serial.print("Finished ProcessData <");
  Serial.print(data);
  Serial.print(">");
  Serial.print(" for ");
  Serial.println(destID);
  Serial.println();
}

void processData(const char* data, int sourceID, int* destIDs, int numDestIDs)
{
  for(int i = 0; i < numDestIDs; i++)
  {
    if(destIDs[i] >= 0)
      processData(data, sourceID, destIDs[i]);
  }
}

void robotLoop()
{
  processBluetoothData();
  processData();
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
  setAngle(0, 0);
  bluetoothConnected = false;
  validBluetoothData = false;
  bluetoothData[0] = '\0';
  servoPins[1] = 8;
  setAngle(1, 0);
  servoPins[2] = 15;
  setAngle(2, 0);
  servoPins[3] = 22;
  setAngle(3, 0);
  servoPins[4] = 23;
  setAngle(4, 0);
  servoPins[5] = 24;
  setAngle(5, 0);
}

void sendBluetoothChar(char toSend)
{
}

void sendBluetoothData(const char* data)
{
  int index = 0;
  for(; index < length(data); index++)
    sendBluetoothChar(data[index]);
  if(data[index-1] != '\0')
    sendBluetoothChar('\0');
  Serial.print("Sent BT data <"); Serial.print(data); Serial.println(">");
}

void set(int pin)
{
  setDO(pin, 1);
}

void setDO(int pin, bool setHigh)
{
  digitalWrite(controllerPins[pin], setHigh);
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
