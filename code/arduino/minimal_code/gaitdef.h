// GAIT-SPECIFIC VARS
#define numUsedServos 4
#define numGait       1
#define trajSize      3
#define numWheelPose  6

const int TYPE[numUsedServos]   = {WHEEL, SINGLE, SINGLE, WHEEL};

const int numPhase[numGait]     = {2};
const int phases[numGait][numUsedServos] = {{2, 0, 1, 2}};
volatile int STATE[numUsedServos]        = {BACKMOVE, BACKMOVE, BACKMOVE, BACKMOVE};
volatile int valback[numUsedServos];

const int forpos[numGait][numUsedServos]  = {{100, 100, 100, 100}};
const int backpos[numGait][numUsedServos] = {{150, 150, 150, 150}};
const int midpos[numGait][numUsedServos]  = {{125, 125, 125, 125}};
const int retrpos[numGait][numUsedServos] = {{100, 100, 100, 100}};
const int wheelposes[numWheelPose]        = {155, 131, 107, 83, 59, 35};

const int trajectory[trajSize] = {0,0,0};
