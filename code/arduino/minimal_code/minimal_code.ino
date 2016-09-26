#include "arduino.h"
#include "robotLibrary.h"

// gait and trajectory definitions
#include "gaitinfo.h"
#include "gaitdef.h"

int iGAIT; // index in trajectory
int GAIT;  // id of gait currectly executing
int PHASE; // phase of gait
int val;   // temp variable, value read from analog pin
int whichWheelPose; // current position of wheels

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
  
  // initialize vars keeping track of gait and trajectory
  iGAIT = numGait-1;
  GAIT = trajectory[iGAIT];
  PHASE = numPhase[GAIT];  
  whichWheelPose = 0;

  // intialize servo positions
  for (int i = 0; i < numUsedServos; ++i) {
    if (i == 1 || i == 2) {
      setPWM(servoPins[i], backpos[GAIT][i]);
    } else {
      setPWM(servoPins[i], backpos[GAIT][i]/2);
    }
  }  
}

// FOLLOW DESIGNED GAIT
void loop()
{
  boolean done = true;
  
  if (PHASE == numPhase[GAIT]) {
    // LAST PHASE, SHIFT ALL LIMBS
    
    for (int i = 0; i < numUsedServos; ++i) {
      if (TYPE[i] == SINGLE || TYPE[i] == DOUBLE) {
        // legs
        if (STATE[i] == FORWMOVE) {
          continue;
        }
      
        valback[i] = analogRead(servoPot[i]);
        if (i == 1 || i == 2) {
          setPWM(servoPins[i], forpos[GAIT][i]);
        } else {
          setPWM(servoPins[i], forpos[GAIT][i]/2);
        }
        STATE[i] = FORWMOVE;
      } else {
        // wheels
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
      iGAIT = (iGAIT + 1) % trajSize;
      GAIT = trajectory[iGAIT];
      PHASE = 0;
    }
  } else {    
    // MOVE THE LIMBS IN THE STEP
    for (int i = 0; i < numUsedServos; ++i) {
      if (phases[GAIT][i] != PHASE) {
        continue;
      }
            
      switch (STATE[i]) {
        case FORWMOVE: {
          if (TYPE[i] == DOUBLE) {
            if (i == 1 || i == 2) {
              setPWM(servoPins[i], retrpos[GAIT][i]);
            } else {
              setPWM(servoPins[i], retrpos[GAIT][i]/2);
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
              STATE[i] = PAUSE2;
            }
          } else if (TYPE[i] == DOUBLE) {
            if (i == 1 || i == 2) {
              setPWM(servoPins[i], midpos[GAIT][i]);
            } else {
              setPWM(servoPins[i], midpos[GAIT][i]/2);
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
              setPWM(servoPins[i], backpos[GAIT][i]);
            } else {
              setPWM(servoPins[i], backpos[GAIT][i]/2);
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
}
