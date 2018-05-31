#ifndef __iTimer
#define __iTimer

#include "base.h"

//Timers use table

// Timer 00 -  right
// Timer 01 -  left
// Timer 02 -  AxisT
// Timer 03 -  AxisU
// Timer 04 -  AxisE
// Timer 05 -  AxisW
// Timer 06 -
// Timer 07 - DualTrack LR
// Timer 08 - whenFinished
// Timer 09 - ArmDoneService
// Timer 10 -
// Timer 11 - DisableWatch
// Timer 12 - DriveAwayWatch
// Timer 13 -
// Timer 14 -
// Timer 15 - Camera Tracking



// *** Timer Variables

extern ui  masterClock;

extern int newClock;

extern ui  countDown[20];

extern uc countDownTripped[20];

extern void (*countDownService[20])(uc);
extern void (*countDownFinish [20])(uc);

void startTimer( uc num ,int count, void (*serviceAddress)(uc), void (*finishAddress)(uc));
void timerService( uc num ,void (*serviceAddress)(uc));

void clearAllTimers(void);
void resetTimers(void);
void setupTimers(void);
void nullService(void);
void nullServiceInt(uc value);
void fgTimerService(void);
void stopTimer(uc l );
void tripTimer(uc i);
void WhenDone(void (*finishAddress)(uc));
void whenArmDone(void (*FinishAddress)(uc));

int timerActive(uc num);

#endif
