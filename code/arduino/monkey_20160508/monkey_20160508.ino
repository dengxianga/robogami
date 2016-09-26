#include "arduino.h"
#include "string_functions.h"
#include "robotLibrary.h"

#define controt   4

#define numUsedServos 6
#define numPhase      4

#define SINGLE   0
#define DOUBLE  1
#define WHEEL    2

#define BACKMOVE  0
#define FORWMOVE  1
#define CONTMOVE  2
#define PAUSE1    3
#define PAUSE2    4

int PHASE;
int valback[numUsedServos];
int servoPot[numUsedServos];
int phases[numUsedServos] = {0, 1, 2, 2, 3, 3};

int forpos[numUsedServos]  = {100, 100, 150, 100, 150, 100};
int backpos[numUsedServos] = {150, 150, 200, 100, 200, 100};
int midpos[numUsedServos]  = {125, 125, 175, 200, 175, 200};
int retrpos[numUsedServos] = {100, 100,  80, 200,  80, 200};
int TYPE[numUsedServos]    = {SINGLE, SINGLE, DOUBLE, DOUBLE, DOUBLE, DOUBLE};
int STATE[numUsedServos]   = {BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE};


void setup()
{
  Serial.begin(9600);
  
  // setup servos:
  // servo0: pin 3
  // servo1: pin 5
  // servo2: pin 6
  // servo3: pin 9
  // servo4: pin 10
  // servo5: pin 11
  robotSetup();
  
  //setup servo potentiometer analog input
  pinMode(A0, INPUT);
  servoPot[0] = A0;
  pinMode(A1, INPUT);
  servoPot[1] = A1;
  pinMode(A2, INPUT);
  servoPot[2] = A2;
  pinMode(A3, INPUT);
  servoPot[3] = A3;
  pinMode(A4, INPUT);
  servoPot[4] = A4;
  pinMode(A5, INPUT);
  servoPot[5] = A5;
  
  PHASE = numPhase;
  
  for (int i = 0; i < numUsedServos; ++i) {
    if (i == 1 || i == 2) {
      setPWM(servoPins[i], backpos[i]);
    } else {
      setPWM(servoPins[i], backpos[i]/2);
    }
  }
}

void loop()
{
  robotLoop();
  
  /*****************************
  // TEST 0: output analog readings
  for (int i = 0; i < numUsedServos; ++i) {
    valback[i] = analogRead(servoPot[i]);
    Serial.print(valback[i]);
    Serial.print("\t");
  }
  Serial.println("");
    
    delay(100);
  //*****************************/
  
  
  /*****************************
  // TEST 1: all servos should move to a position and stop
  setPWM(servoPins[0], backpos/2);
  setPWM(servoPins[1], backpos);
  setPWM(servoPins[2], backpos);
  setPWM(servoPins[3], backpos/2);
  setPWM(servoPins[4], backpos/2);
  setPWM(servoPins[5], backpos/2);
  delay(1000);
  
  setPWM(servoPins[0], forpos/2);
  setPWM(servoPins[1], forpos);
  setPWM(servoPins[2], forpos);
  setPWM(servoPins[3], forpos/2);
  setPWM(servoPins[4], forpos/2);
  setPWM(servoPins[5], forpos/2);
  delay(1000);
  //*****************************/
  
  /*****************************
  // TEST 2: all servos should rotate continuously
  setPWM(servoPins[0], controt);
  setPWM(servoPins[1], controt);
  setPWM(servoPins[2], controt);
  setPWM(servoPins[3], controt);
  setPWM(servoPins[4], controt);
  setPWM(servoPins[5], controt);
  
  //*****************************/
  
    
  /*****************************
  // TEST 3: one servo gait
  PHASE = 1;
  valback[PHASE] = analogRead(servoPot[PHASE]);
  if (PHASE == 1 || PHASE == 2) {
    setPWM(servoPins[PHASE], forpos);
  } else {
    setPWM(servoPins[PHASE], forpos/2);
  }
  delay(1000);
  
  int val = analogRead(servoPot[PHASE]);
  while (val < valback[PHASE]){
    setPWM(servoPins[PHASE], controt);
    val = analogRead(servoPot[PHASE]);
    delay(50);
  }
  delay(200);
  if (PHASE == 1 || PHASE == 2) {
    setPWM(servoPins[PHASE], backpos);
  } else {
    setPWM(servoPins[PHASE], backpos/2);
  }
  delay(1000);
  
  //*****************************/
  
  /*****************************
  // TEST 4: wavegait
  if (PHASE == numUsedServos) {
    valback[0] = analogRead(servoPot[0]);
    delay(50);
    valback[1] = analogRead(servoPot[1]);
    delay(50);
    valback[2] = analogRead(servoPot[2]);
    delay(50);
    valback[3] = analogRead(servoPot[3]);
    delay(50);
    valback[4] = analogRead(servoPot[4]);
    delay(50);
    valback[5] = analogRead(servoPot[5]);
    delay(50);
  
    setPWM(servoPins[0], forpos/2);
    setPWM(servoPins[1], forpos);
    setPWM(servoPins[2], forpos);
    setPWM(servoPins[3], forpos/2);
    setPWM(servoPins[4], forpos/2);
    setPWM(servoPins[5], forpos/2);
  
    delay(1000);
    PHASE = 0;
  } else {
    int val = analogRead(servoPot[PHASE]);
    while (val < valback[PHASE]) {
      if (PHASE == 1 || PHASE == 2) {
        setPWM(servoPins[PHASE], controt);
      } else {
        setPWM(servoPins[PHASE], controt);
      }
      val = analogRead(servoPot[PHASE]);
      delay(50);
    }
    delay(200);
    
    switch (PHASE) {
      case 0:
        setPWM(servoPins[PHASE], backpos/2);
        break;
      case 1:
        setPWM(servoPins[PHASE], backpos);
        break;
      case 2:
        setPWM(servoPins[PHASE], backpos);
        break;
      case 3:
        setPWM(servoPins[PHASE], backpos/2);
        break;
      case 4:
        setPWM(servoPins[PHASE], backpos/2);
        break;
      case 5:
        setPWM(servoPins[PHASE], backpos/2);
        break;
    }
    delay(800);
    
    PHASE = PHASE + 1;
  }
  //*****************************/
  
  //*****************************
  // TEST 5: designed gait
  if (PHASE == numPhase) {
    
    for (int i = 0; i < numUsedServos; ++i) {
      valback[i] = analogRead(servoPot[i]);
      if (i == 1 || i == 2) {
        setPWM(servoPins[i], forpos[i]);
      } else {
        setPWM(servoPins[i], forpos[i]/2);
      }
      STATE[i] = FORWMOVE;
      
      delay(50);
    }
      
    delay(100);
    PHASE = 0;
  } else {
    boolean done = true;
    
    for (int i = 0; i < numUsedServos; ++i) {
      if (phases[i] != PHASE) {
        continue;
      }
            
      switch (STATE[i]) {
        case FORWMOVE: {
          if (i == 1 || i == 2) {
            setPWM(servoPins[i], retrpos[i]);
          } else {
            setPWM(servoPins[i], retrpos[i]/2);
          }
          STATE[i] = PAUSE1;
          done = false;
          } break;
        case PAUSE1: {
          STATE[i] = CONTMOVE;
          done = false;
          } break;
        case CONTMOVE: {
          if (TYPE[i] == SINGLE) {
            setPWM(servoPins[i], controt);
            delay(50);
            int val = analogRead(servoPot[i]);
            if (val > valback[i]) {
              STATE[i] = BACKMOVE;
            }
          } else {
            if (i == 1 || i == 2) {
              setPWM(servoPins[i], midpos[i]);
            } else {
              setPWM(servoPins[i], midpos[i]/2);
            }
            STATE[i] = PAUSE2;
          }
          done = false;
          } break;
        case PAUSE2: {
          STATE[i] = BACKMOVE;
          done = false;
          } break;
        case BACKMOVE: {
          if (i==1 || i==2) {
            setPWM(servoPins[i], backpos[i]);
          } else {
            setPWM(servoPins[i], backpos[i]/2);
          }
          } break;
      }
    }
    delay(150);
    if (done) {
      delay(100);
      PHASE = PHASE + 1;
    }
  }
  //*****************************/
  
}
