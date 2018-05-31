#ifndef __Motor
#define __Motor

#include "base.h"

extern float pwm[21];
extern float dac[21];

#define motorZero 128

int Limit255(int value);


 void SetToroUpSpeed      (float rate);
 void SetToroDownSpeed    (float rate);
 void SetBumperUpSpeed    (float rate);
 void SetBumperDownSpeed  (float rate);
 void SetCatcherOpenSpeed (float rate);
 void SetCatcherCloseSpeed(float rate);
















#endif
