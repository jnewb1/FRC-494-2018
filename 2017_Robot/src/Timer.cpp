#include "iTimer.h"
#include "JoyStick.h"
#include "driving.h"
#include "aton.h"
#include "base.h"
#include "Timer.h"

//#include <moduleLib.h>
//#include <taskLib.h>
//#include <unldLib.h>
#include <stdio.h>

// *** Timer Variables

ui  masterClock = 0;
ui  countDown[20];

double finishTime[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

uc countDownTripped[20];

void(*countDownService[20])(uc);
void(*countDownFinish[20])(uc);

void nullService(void) {}
void nullServiceInt(uc value) {}

void setupTimers(void)
{
	clearAllTimers();
}

void stopTimer(uc l)
{
	if (l < (uc)20)
	{
		countDown[l] = 0;
		countDownTripped[l] = 0;
		countDownService[l] = nullServiceInt;
		countDownFinish[l] = nullServiceInt;
	}
}

void clearAllTimers(void)
{
	for (int L = 0; L < 20; L++) if (L != 11) stopTimer(L);
}

int newClock = 0;

int LastTimer = 0;

void startTimer(uc num, int count, void(*serviceAddress)(uc), void(*finishAddress)(uc))
{
	if (num > (uc)19) return;

	// tprintf("startTimer %d \n",(int)num);

	LastTimer = num;

	if (count < 1) count = 1;

	if (serviceAddress == 0) serviceAddress = nullServiceInt;
	if (finishAddress == 0) finishAddress = nullServiceInt;

	countDown[num] = 0;

	finishTime[num] = 0;

	countDownTripped[num] = 0;
	countDownService[num] = serviceAddress;
	countDownFinish[num] = finishAddress;

	countDown[num] = count;

	finishTime[num] = Timer::GetFPGATimestamp() + count / 100.0;
}

void WhenDone(void(*finishAddress)(uc))
{
	countDownFinish[LastTimer] = finishAddress;
}

void timerService(uc num, void(*serviceAddress)(uc))
{
	if (num < (uc)20) countDownService[num] = serviceAddress;
}

void tripTimer(uc i)
{
	//printf("tripTimer %4d\n",(int)i);

	countDownTripped[i] = 1;
	countDown[i] = 0;

	finishTime[i] = 0;
}

int timerActive(uc num)
{
	if (num < 0 || num>19) return 0;

	return (int)countDown[num];
}

void fgTimerService(void)
{
	if (atonActive) atonActive--;

	for (int L = 0; L < (uc)20; L++)
	{

		if (countDownTripped[L])
		{

			//tprintf("TimerFinished %d \n",L);

			countDownTripped[L] = 0;
			countDown[L] = 0;

			if (countDownFinish[L]) countDownFinish[L](L);


		}

		else

			// if (countDown[L]>0) {countDownService[L](L);countDown[L]--;if (countDown[L]==0) tripTimer(L);}
		{

			if (countDown[L] > 0)
			{
				double curTime = Timer::GetFPGATimestamp();

				if (curTime < finishTime[L]) {

					if (countDownService[L]) countDownService[L](L);

					countDown[L]--;

					if (countDown[L] == 0) tripTimer(L);

				}

				else { countDown[L] = 0; tripTimer(L); }

			}

		}

	}

}


#define timeLimit(limit)                \
    static double lastTime=0;            \
	double curTime=GetClock();           \
	if (curTime-lastTime<limit) return; \
	lastTime=curTime;

int blockStick2 = 0;

void updateCountDownVariables()
{
	//if (steeringTimeOut>0) steeringTimeOut--; else steeringTimeOut=0;

	if (blockStick2 > 0) blockStick2--; else blockStick2 = 0;
}

void backProcess();

void IFC_Local_Loop_Service(void)
{

	static double lastTime2 = 0;

	double curTime = Timer::GetFPGATimestamp();

	double dif2 = curTime - lastTime2;

	if (dif2 < 0.01) return;

	lastTime2 = curTime;

	int count = 1;

	for (int L = 0; L < count; L++)
	{
		newClock++;

		//drivingService();

		trigService();

		keyService();

		hatService();

		atonService();

		fgTimerService();

		backProcess();

		updateCountDownVariables();
	}

}



//void IFC_Local_Loop_Service(void)
//{
//  static double lastTime=0;
//
//  double curTime=GetClock();
//
//  float dif=curTime-lastTime;
//
//  if ( dif>0.1 || dif<-0.1) lastTime=curTime;
//
//  if (dif<0.01) return; //
//
//  lastTime=curTime;
//
//  int loops=(int)(dif/0.01+0.5);
//
//  if (loops>10) loops=10;
//
//  for (int L=0; L<loops; L++ )
//  {
//    newClock++;
//
//   // drivingService();
//
//    trigService();
//
//    keyService();
//
//    hatService();
//
//    atonService();
//
//    fgTimerService();
//
//    updateCountDownVariables();
//  }
//
//}


