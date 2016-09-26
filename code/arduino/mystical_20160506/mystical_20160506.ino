#include "arduino.h"
#include "string_functions.h"
#include "robotLibrary.h"

#define forpos  100
#define backpos 150
#define controt   4

#define numUsedServos 5
#define numPhase  3

#define BACKMOVE  0
#define FORWMOVE  1
#define CONTMOVE  2

int PHASE;
int valback[numUsedServos];
int servoPot[numUsedServos];
int phases[numUsedServos] = {0,1,2,2,0};
int STATE[numUsedServos] = {BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE};


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
  
  setPWM(servoPins[0], backpos/2);
  setPWM(servoPins[1], backpos);
  setPWM(servoPins[2], backpos);
  setPWM(servoPins[3], backpos/2);
  setPWM(servoPins[4], backpos/2);
  setPWM(servoPins[5], backpos/2);
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
        setPWM(servoPins[i], forpos);
      } else {
        setPWM(servoPins[i], forpos/2);
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
          STATE[i] = CONTMOVE;
          done = false;
          } break;
          
        case CONTMOVE: {
          setPWM(servoPins[i], controt);
          delay(50);
          int val = analogRead(servoPot[i]);
          if (val > valback[i]) {
            STATE[i] = BACKMOVE;
          }
          done = false;
          } break;
          
        case BACKMOVE: {
          if (i==1 || i==2) {
            setPWM(servoPins[i], backpos);
          } else {
            setPWM(servoPins[i], backpos/2);
          }
          } break;
      }
    }
    delay(100);
    if (done) {
      delay(100);
      PHASE = PHASE + 1;
    }
  }
  //*****************************/
  
}
