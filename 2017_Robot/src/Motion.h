#ifndef __Motion
#define __Motion

#include "base.h"

extern li cmdPos[8];
extern li curVel[8];
extern uc closeLoopFlag[8];
extern int axiaGain [8];
extern int axiaGainI[8];
extern int axiaGainD[8];
extern int axiaGainF[8];

extern li cVal[40];

extern int cValIndex;

void closeLoopService(void);
extern uc BlockCloseLoopOff;

void Motion  (uc i,li dist,int vel,int accel,void (*motionFinished)(uc));
void MotionRL(li distR,li distL,int vel,int accel,void (*motionFinished)(uc));
void MotionAbs(uc i,li dist,int vel,int accel,void (*motionFinished)(uc));

void timedMotionRL(li distR,li distL,int time,int accel,void (*motionFinished)(uc));


// RL Motion Support Functions

int moveTime ( int time );
int moveSpeed( int Rate,int Dist);

void whenFinished( void (*motionFinished)(uc),int timeOut);

void MotionForward(uc i,int time); //,void(*ForceMotionForwardEnd)(uc));
void MotionReverse(uc i,int time); //,void(*ForceMotionReverseEnd)(uc));

//Open Loop Motion Timers (TIME=seconds*100)

/*
void OpenGrip   (int Time);
void CloseGrip  (int Time);

void WristUp    (int Time);
void WristDown  (int Time);

void Retract    (int Time);
void Extend     (int Time) ;

void RotateR    (int Time);
void RotateL    (int Time);

void RotateUp   (int Time);
void RotateDown (int Time);
*/
void MoveForward(int Time);
void MoveBack   (int Time);

void MoveRight  (int Time);
void MoveLeft   (int Time);










#endif
