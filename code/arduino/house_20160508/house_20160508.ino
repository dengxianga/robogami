#include "arduino.h"
#include "string_functions.h"
#include "robotLibrary.h"

#define controt   4
#define rconrot 250

#define numUsedServos 4
#define numPhase      2
#define numWheelPose  6

#define SINGLE   0
#define DOUBLE   1
#define WHEEL    2

#define BACKMOVE  0
#define FORWMOVE  1
#define CONTMOVE  2
#define PAUSE1    3
#define PAUSE2    4

int PHASE;
int val;
int valback[numUsedServos];
int whichWheelPose;
const int servoPot[numServos] = {A0, A1, A4, A3, A2, A5};

const int phases[numUsedServos] = {2, 0, 1, 2};
const int wheelposes[numWheelPose] = {155, 131, 107, 83, 59, 35};

const int forpos[numUsedServos]  = {100, 100, 100, 100};
const int backpos[numUsedServos] = {150, 150, 150, 150};
const int midpos[numUsedServos]  = {125, 125, 125, 125};
const int retrpos[numUsedServos] = {100, 100, 100, 100};
const int TYPE[numUsedServos]    = {WHEEL, SINGLE, SINGLE, WHEEL};
int STATE[numUsedServos]   = {BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE};


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
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  
  for (int i = 0; i < numUsedServos; ++i) {
    if (i == 1 || i == 2) {
      setPWM(servoPins[i], backpos[i]);
    } else {
      setPWM(servoPins[i], backpos[i]/2);
    }
   }
  
  
  PHASE = numPhase;  
  whichWheelPose = 0;
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
  // TEST 3a: one servo single leg gait
  PHASE = 2;
  valback[PHASE] = analogRead(servoPot[PHASE]);
  Serial.println("forward");
  if (PHASE == 1 || PHASE == 2) {
    setPWM(servoPins[PHASE], forpos[PHASE]);
  } else {
    setPWM(servoPins[PHASE], forpos[PHASE]/2);
  }
  delay(100);
  
  int val = analogRead(servoPot[PHASE]);
  while (val < valback[PHASE]){
    Serial.print(val) ;
    Serial.print(" ");
    Serial.println(valback[PHASE]);
    setPWM(servoPins[PHASE], controt);
    val = analogRead(servoPot[PHASE]);
    delay(50);
  }
  delay(1000);
  Serial.println("back");
  if (PHASE == 1 || PHASE == 2) {
    setPWM(servoPins[PHASE], backpos[PHASE]);
  } else {
    setPWM(servoPins[PHASE], backpos[PHASE]/2);
  }
  delay(1000);
  
  //*****************************/
  
  /*****************************
  // TEST 1b: 1 servo iterate through wheel positions
  PHASE = 0;
  Serial.println(whichWheelPose);
  
  if (whichWheelPose == numWheelPose-1) {
      valback[PHASE] = analogRead(servoPot[PHASE]);
  }

  if (whichWheelPose == 0) {
    int val = analogRead(servoPot[PHASE]);
    while (val < valback[PHASE]){
      setPWM(servoPins[PHASE], controt);
      val = analogRead(servoPot[PHASE]);
      Serial.print(val);
      Serial.print(" < ");
      Serial.println(valback[PHASE]);
      delay(50);
    }
  }

  setPWM(servoPins[PHASE], wheelposes[whichWheelPose]);
  delay(5000);

  whichWheelPose = (whichWheelPose + 1) % numWheelPose;  
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
  boolean done = true;

  Serial.println(PHASE);
  
  if (PHASE == numPhase) {
    
    for (int i = 0; i < numUsedServos; ++i) {
      if (TYPE[i] == SINGLE || TYPE[i] == DOUBLE) {
        if (STATE[i] == FORWMOVE) {
          continue;
        }
      
        valback[i] = analogRead(servoPot[i]);
        if (i == 1 || i == 2) {
          Serial.println("setting servos");
          setPWM(servoPins[i], forpos[i]);
        } else {
          setPWM(servoPins[i], forpos[i]/2);
        }
        STATE[i] = FORWMOVE;
      } else {
        if (whichWheelPose == numWheelPose - 1) {
          valback[i] = analogRead(servoPot[i]);
        }

        val = analogRead(servoPot[i]);
        if ((whichWheelPose == 0) && (val < valback[i])) {
          setPWM(servoPins[i], controt);
          done = false;
        } else {
          setPWM(servoPins[i], wheelposes[whichWheelPose]);
          STATE[i] = FORWMOVE;
        }
      }      
    }

    if (done) {      
      whichWheelPose = (whichWheelPose + 1) % numWheelPose;
      delay(100);
      PHASE = 0;
    }
  } else {    
    for (int i = 0; i < numUsedServos; ++i) {
      if (phases[i] != PHASE) {
        continue;
      }
            
      switch (STATE[i]) {
        case FORWMOVE: {
          if (TYPE[i] == DOUBLE) {
            if (i == 1 || i == 2) {
              setPWM(servoPins[i], retrpos[i]);
            } else {
              setPWM(servoPins[i], retrpos[i]/2);
            }
            STATE[i] = PAUSE1;
          } else {
            STATE[i] = CONTMOVE;
          }
          done = false;
          } break;
        case PAUSE1: {
          STATE[i] = CONTMOVE;
          done = false;
          } break;
        case CONTMOVE: {
          if (TYPE[i] == SINGLE) {
            setPWM(servoPins[i], controt);
            val = analogRead(servoPot[i]);
            if (val > valback[i]) {
              STATE[i] = BACKMOVE;
            }
          } else if (TYPE[i] == DOUBLE) {
            if (i == 1 || i == 2) {
              setPWM(servoPins[i], midpos[i]);
            } else {
              setPWM(servoPins[i], midpos[i]/2);
            }
            STATE[i] = PAUSE2;
          } else {
            STATE[i] = BACKMOVE;
          }
          done = false;
          } break;
        case PAUSE2: {
          STATE[i] = BACKMOVE;
          done = false;
          } break;
        case BACKMOVE: {
          if (TYPE[i] == SINGLE || TYPE[i] == DOUBLE) {
            if (i==1 || i==2) {
              setPWM(servoPins[i], backpos[i]);
            } else {
              setPWM(servoPins[i], backpos[i]/2);
            }
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
