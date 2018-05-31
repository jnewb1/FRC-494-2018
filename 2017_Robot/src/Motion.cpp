//#include <iostream.h>
#include "WPILib.h"
#include "motion.h"
#include "iencoder.h"
#include "itimer.h"
#include "motor.h"
#include "cal.h"
//#include "eeprom.h"
//#include "user_routines.h"

 //#define debug
//#define  aRight    0
//#define  aLeft      1
//#define  aTurret    2
//#define  aElev     3
//#define  aExtend    4
//#define  aWrist     5
//#define  aGriper   6
//#define  aCaster    7
//uc dragFlag=0;

extern li cmdPos[8];
extern li setPos[8];
extern li cmdVel[8];
extern li cmdAcc[8];
extern li curVel[8];
extern li endPos[8];
extern li curError[8];
extern li lastLoc[8];
extern li curVelError[8];
extern li curVelErrorI[8];
extern li curErrorI[8];

extern uc closeLoopFlag[8];

extern int axiaGain[8];



li pid_time = 1;

uc allFinished = 1;

// RL Motion Support Functions

int moveTime(int time)
{
	if (time < 1) time = 1;

	return 10000 / time;
}

int moveSpeed(int Rate, int Dist)
{
	return moveTime((Dist * 10) / Rate);
}


li DivTen(li Value)
{
	li tmp;

	tmp = Value / (li)10;

	if (tmp < (li)1) tmp = (li)1;

	return tmp;
}

void begMotion(uc i);
void endMotion(uc i);
void begMotionN(uc i);
void endMotionN(uc i);

//********************************************************
//Forced Motion Full ( Forward-Reverse ) arm motions only!
//********************************************************

void MotionForwardService(uc i) {
	//printf("service %4d\r",(int)i);
	closeLoopFlag[i] = 4;

} //Force Speed Forward

void MotionForward(uc i, int time) //,void(*MotionForwardEnd)(uc))
{
	if (time > 1000) time = 1000; //ten sec safety
	startTimer(i, time, MotionForwardService, 0); //MotionForwardEnd);
}

void MotionReverseService(uc i)
{
	// printf("service %4d\n",(int)i);

	closeLoopFlag[i] = 5;

} //Force Speed Reverse

void MotionReverse(uc i, int time) //,void(*MotionReverseEnd)(uc))
{
	//  printf("motionReverse %4d\n",(int)i);

	if (time > 1000) time = 1000; //ten sec safety
	startTimer(i, time, MotionReverseService, 0); //MotionReverseEnd);
}


// use When Done to setup next action


//***********************************************************
//***********************************************************

li labss(li value) { if (value < (li)0) return -value; return value; }


// distR,distL distance in encoder counts
// vel is feet/sec * 10  example 10fps = 100
// accel fixed ramp in encoder counts

void MotionRL(li distR, li distL, int vel, int accel, void(*motionFinished)(uc))
{

	li avgDist;
	li time;
	li rate;

#ifdef debug
	printf("MotionRL\n");
#endif

	avgDist = ((li)10 * (labss(distR) + labss(distL)) / (li)2) / (li)countsPerFootRight;

	time = (li)100 * avgDist / vel;

	rate = (li)100000 / time;

	if (motionFinished == 0) motionFinished = nullServiceInt;

	endPos[0] = distR; //used as travel distance
	endPos[1] = distL;

	cmdVel[0] = vel;
	cmdAcc[0] = accel;
	curVel[0] = 0;

	setPos[0] = cmdPos[0]; //start of motion location
	setPos[1] = cmdPos[1];
	//printf("rate %5d  \r",(int)rate);

	endPos[7] = 10000;
	cmdVel[7] = rate;
	cmdAcc[7] = accel;
	curVel[7] = 0;

	setPos[7] = 0;
	cmdPos[7] = 0;

	allFinished = 0;

	closeLoopFlag[0] = 1;
	closeLoopFlag[1] = 1;

	startTimer(7, 12000, begMotion, motionFinished);

}

void timedMotionRL(li distR, li distL, int time, int accel, void(*motionFinished)(uc))
{

#ifdef debug
	printf("MotionRL\n");
#endif


	if (motionFinished == 0) motionFinished = nullServiceInt;

	endPos[0] = distR; //used as travel distance
	endPos[1] = distL;

	cmdVel[0] = 250;
	cmdAcc[0] = accel;
	curVel[0] = 0;

	setPos[0] = cmdPos[0]; //start of motion location
	setPos[1] = cmdPos[1];

	endPos[7] = 10000;
	cmdVel[7] = (li)100000 / time;
	cmdAcc[7] = accel;
	curVel[7] = 0;

	setPos[7] = 0;
	cmdPos[7] = 0;

	allFinished = 0;

	closeLoopFlag[0] = 1;
	closeLoopFlag[1] = 1;

	startTimer(7, 12000, begMotion, motionFinished);

}

void calcRL(void)
{

	cmdPos[0] = setPos[0] + (endPos[0] * cmdPos[7]) / (li)10000;
	cmdPos[1] = setPos[1] + (endPos[1] * cmdPos[7]) / (li)10000;

	// closeLoopFlag[0]=1;
   //  closeLoopFlag[1]=1;

	 // printf("cmdPos %5d %5d \n",(int)cmdPos[0],(int)cmdPos[1]);

}


void Motion(uc i, li dist, int vel, int accel, void(*motionFinished)(uc))
{

#ifdef debug
	printf("Motion %4d\n", (int)i);
#endif



	MotionAbs(i, cmdPos[i] + dist, vel, accel, motionFinished);

}


void MotionAbs(uc i, li dist, int vel, int accel, void(*motionFinished)(uc))
{

#ifdef debug
	printf("%5d %5d %5d %5d\r", (int)i, (int)dist, (int)vel, (int)accel);
#endif

	if (motionFinished == 0) motionFinished = nullServiceInt;

	allFinished = 0;

	endPos[i] = dist;
	cmdVel[i] = vel;
	cmdAcc[i] = accel;
	curVel[i] = 0;

	setPos[i] = cmdPos[i];


	// printf("Beg %5d End %5d\n",(int)cmdPos[i],(int)endPos[i]);
#ifdef debug
	printf("Beg %5d End  %5d\r", (int)cmdPos[i], (int)endPos[i]);
#endif

	if (endPos[i] > cmdPos[i]) startTimer(i, 12000, begMotion, motionFinished);
	else        startTimer(i, 12000, begMotionN, motionFinished);
}

void begMotion(uc i)
{
#ifdef debug
	printf("begMotion %2d %4d %4d\n", (int)i, (int)curVel[i], (int)cmdPos[i]);
#endif
	closeLoopFlag[i] = 1;

	if (cmdVel[i])
	{
		//Check for past midpoint

		if (cmdPos[i] > setPos[i] + (endPos[i] - setPos[i]) / 2)
		{

			timerService(i, endMotion);

			setPos[i] = cmdPos[i];

			endMotion(i);

			return;

		}

		curVel[i] += cmdAcc[i];

		if (curVel[i] > cmdVel[i]) {

			curVel[i] = cmdVel[i];

			setPos[i] = endPos[i] - (cmdPos[i] - setPos[i]);

			timerService(i, endMotion);

		}

		cmdPos[i] += DivTen(curVel[i]);
	}

	if (i == (uc)7) calcRL();

}

void endMotion(uc i)
{

#ifdef debug
	printf("EndMotion %2d %4d %4d\n", (int)i, (int)curVel[i], (int)cmdPos[i]);
#endif

	closeLoopFlag[i] = 1;

	cmdPos[i] += DivTen(curVel[i]);

	if (cmdPos[i] > setPos[i])
	{
		// if (i!=(uc)AxisW)

		curVel[i] -= cmdAcc[i];

		if (curVel[i] < cmdAcc[i] * 2) curVel[i] = cmdAcc[i] * 2;
	}

	//printf("cmdPos[i] %5d endPos[i] %5d\n",cmdPos[i],endPos[i]);

	//if (i!=AxisW)
	if (cmdPos[i] > endPos[i])
	{
		cmdPos[i] = endPos[i];

		closeLoopFlag[i] = 0;

		if (i == (uc)7) closeLoopFlag[AxisR] = closeLoopFlag[AxisL] = 0;

		tripTimer(i);

	}

	// if (i==AxisW)
	// if (loc[i]>endPos[i])
	// {
   //     cmdPos[i]=endPos[i];

   //     tripTimer(i);

   //     closeLoopFlag[i]=0;
   //  }

	if (i == (uc)7) calcRL();
}

void begMotionN(uc i)
{
#ifdef debug
	printf("nbeg: %6d %6d\n", (int)cmdPos[i], (int)curVel[i]);
#endif
	//Check for past midpoint

	closeLoopFlag[i] = 1;

	if (cmdPos[i] < setPos[i] + (endPos[i] - setPos[i]) / 2)
	{

		timerService(i, endMotionN);

		setPos[i] = cmdPos[i];

		endMotionN(i);

		return;

	}

	curVel[i] += cmdAcc[i];

	if (curVel[i] > cmdVel[i]) {

		curVel[i] = cmdVel[i];

		setPos[i] = endPos[i] - (cmdPos[i] - setPos[i]);

		timerService(i, endMotionN);

	}

	cmdPos[i] -= DivTen(curVel[i]);
}

void endMotionN(uc i)
{
#ifdef debug
	printf("nEnd:%6d %6d\n", (int)cmdPos[i], (int)curVel[i]);
#endif

	closeLoopFlag[i] = 1;

	cmdPos[i] -= DivTen(curVel[i]);

	if (cmdPos[i] < setPos[i])
	{
		// if (i!=(uc)AxisW)

		curVel[i] -= cmdAcc[i];

		if (curVel[i] < cmdAcc[i] * 2) curVel[i] = cmdAcc[i] * 2;
	}

	// if (i!=AxisW)
	if (cmdPos[i] < endPos[i])
	{
		closeLoopFlag[i] = 0;

		if (i == (uc)7) closeLoopFlag[AxisR] = closeLoopFlag[AxisL] = 0;

		cmdPos[i] = endPos[i];

		tripTimer(i);
	}

	// if (i==AxisW)
	// if (loc[i]<endPos[i])
	// {
	//   closeLoopFlag[i]=0;

	//   cmdPos[i]=endPos[i];

	//   tripTimer(i);
   //  }

}

void ShowCodeV(uc index, int itime, uc value, int vtime);
void ShowCodeI(uc index, int itime, uc value, int vtime);

uc BlockCloseLoopOff = 0;
li cVal[40] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

int cValIndex = 0;

void IndexPos()
{
	//axiaGain[AxisR]++;
	//axiaGain[AxisL]++;


	cValIndex++;

	if (cValIndex < 0) cValIndex = 0;
	if (cValIndex > 39) cValIndex = 39;

	if (cVal[cValIndex] < 0) cVal[cValIndex] = 0;
	if (cVal[cValIndex] > 250) cVal[cValIndex] = 0;

	// printf("[%4d] %4d\r",(int) cValIndex, (int) cVal[cValIndex]);

	// DiplayCalValName( cValIndex );

	// user1=cValIndex;
	// user2=cVal[cValIndex];

	// ShowCodeI( user1,100, user2, 100);

	// EEPROM_Write(9,  cValIndex );
}

void IndexNeg()
{


	cValIndex--;

	if (cValIndex < 0) cValIndex = 0;
	if (cValIndex > 39) cValIndex = 39;

	if (cVal[cValIndex] < 0) cVal[cValIndex] = 0;
	if (cVal[cValIndex] > 250) cVal[cValIndex] = 0;

	//printf("[%4d] %4d\r",(int) cValIndex, (int) cVal[cValIndex]);

   // DiplayCalValName( cValIndex );

   // user1=cValIndex;
  //  user2=cVal[cValIndex];

   // ShowCodeI( user1,100, user2, 100);

   // EEPROM_Write(9,  cValIndex );

	 //if (axiaGain[AxisR]>1) axiaGain[AxisR]--;
	 //if (axiaGain[AxisL]>1) axiaGain[AxisL]--;
}




void ValuePos()
{

	//	if (!disabled_mode)
	//   {
	//	   printf("Must Be Disabled to Write\r");
	//
	//	   return;
	//   }

	cVal[cValIndex]++;

	if (cVal[cValIndex] < 0) cVal[cValIndex] = 0;
	if (cVal[cValIndex] > 250) cVal[cValIndex] = 0;

	//printf("[%4d] %4d\r",(int) cValIndex, (int) cVal[cValIndex]);

   // DiplayCalValName( cValIndex );

  //  user1=cValIndex;
  //  user2=cVal[cValIndex];

  //  ShowCodeV( user1,50, user2, 100);

  //  EEPROM_Write(10+cValIndex, cVal[cValIndex]);

	 //axiaGainI[AxisR]++;
	 //axiaGainI[AxisL]++;
}
void ValueNeg()
{

	//	if (!disabled_mode)
	 //   {
	//	   printf("Must Be Disabled to Write\r");
	//
	//	   return;
	 //   }

	cVal[cValIndex]--;

	if (cVal[cValIndex] < 0) cVal[cValIndex] = 0;
	if (cVal[cValIndex] > 250) cVal[cValIndex] = 0;

	//printf("[%4d] %4d\r",(int) cValIndex, (int) cVal[cValIndex]);

   // DiplayCalValName( cValIndex );

  //  user1=cValIndex;
  //  user2=cVal[cValIndex];

  //  ShowCodeV( user1,50, user2, 100);

   // EEPROM_Write(10+cValIndex, cVal[cValIndex]);


	 //if (axiaGainI[AxisR]>1) axiaGainI[AxisR]--;
	 //if (axiaGainI[AxisL]>1) axiaGainI[AxisL]--;
}

void locStoreSet()
{

	//	if (!disabled_mode)
	 //   {
	//	   printf("Must Be Disabled to Write\r");
	//
	//	   return;
	//    }

	if (cValIndex < 20) return;
	if (cValIndex > 29) return;

	if (cValIndex == 20) { cVal[20] = (uc)(loc[AxisU] / 10); printf("Store U Set\r"); }
	if (cValIndex == 21) { cVal[21] = (uc)(loc[AxisW] / 10); printf("Store W Set\r"); }

	if (cValIndex == 22) { cVal[22] = (uc)(loc[AxisU] / 10); printf("PickUp U Set\r"); }
	if (cValIndex == 23) { cVal[23] = (uc)(loc[AxisW] / 10); printf("PickUp W Set\r"); }

	if (cValIndex == 24) { cVal[24] = (uc)(loc[AxisU] / 10); printf("Carry U Set\r"); }
	if (cValIndex == 25) { cVal[25] = (uc)(loc[AxisW] / 10); printf("Carry W Set\r"); }

	if (cValIndex == 26) { cVal[26] = (uc)(loc[AxisU] / 10); printf("Ram U Set\r"); }
	if (cValIndex == 27) { cVal[27] = (uc)(loc[AxisW] / 10); printf("Ram W Set\r"); }

	if (cValIndex == 28) { cVal[28] = (uc)(loc[AxisU] / 10); printf("Hurdle U Set\r"); }
	if (cValIndex == 29) { cVal[29] = (uc)(loc[AxisW] / 10); printf("Hurdle W Set\r"); }

	//DiplayCalValName( cValIndex );

 //   user1=cValIndex;
 //   user2=cVal[cValIndex];

 //   ShowCodeV( user1,50, user2, 100);

 //   EEPROM_Write(10+cValIndex, cVal[cValIndex]);

}

/*
void dGainPos()
{
	axiaGainD[AxisR]++;
	axiaGainD[AxisL]++;
}
void dGainNeg()
{
	if (axiaGainD[AxisR]>0) axiaGainD[AxisR]--;
	if (axiaGainD[AxisL]>0) axiaGainD[AxisL]--;
}

void fGainPos()
{
	axiaGainF[AxisR]++;
	axiaGainF[AxisL]++;
}
void fGainNeg()
{
	if (axiaGainF[AxisR]>1) axiaGainF[AxisR]--;
	if (axiaGainF[AxisL]>1) axiaGainF[AxisL]--;
}
*/

void clReset(uc i)
{
	cmdPos[i] = (long int)loc[i];
	lastLoc[i] = (long int)loc[i];
	curError[i] = 0;
	curErrorI[i] = 0;
	curVelError[i] = 0;
	curVelErrorI[i] = 0;
	pid_time = 1;
}

void closeLoop(uc i)
{

	return;
/*
	li error;
	li lastError;
	li rate;
	li pos_last;
	li pos_error_last;
	li vel_last;
	li vel_error_last;
	li vel_ff;

	if (i > (uc)6) return;  //7-sonar 8-sonar 9-gyro

	pos_last = lastLoc[i]; //motor_info[motor].pos;
	pos_error_last = curError[i]; //motor_info[motor].pos_error;
	vel_last = curVel[i]; //motor_info[motor].vel;
	vel_error_last = curVelError[i]; //motor_info[motor].vel_error;

	// if (closeLoopFlag[i]) printf(" %3d \r",i);

	if (closeLoopFlag[i] == (uc)0)
	{
		clReset(i);

		return;
	}

	if (closeLoopFlag[i] == (uc)4)
	{
		//  pwm[i]=255;

		closeLoopFlag[i] = 0;

		clReset(i);

		return;
	}

	if (closeLoopFlag[i] == (uc)5)
	{
		//pwm[i]=0;

		closeLoopFlag[i] = 0;

		clReset(i);

		return;
	}

	//if (!BlockCloseLoopOff) closeLoopFlag[i]=0;

	allFinished = 0;

	lastError = curError[i];

	error = (long int)(loc[i] - cmdPos[i]);

	curError[i] = error;

	//curErrorI[i]+=error;

	//if (curErrorI[i]> 100000) curErrorI[i]= 100000;
	//if (curErrorI[i]<-100000) curErrorI[i]=-100000;

	if (error < 0) curErrorI[i] += 100;
	if (error > 0) curErrorI[i] -= 100;


	vel_ff = (cmdPos[i] - lastLoc[i]);


	lastLoc[i] = cmdPos[i];

	// rate=error/(li)axiaGain[i];            //p

	// rate+=(curErrorI[i]/pid_time)/(li)100; //i
	// rate+=(pos_error_last-error )/(li)100; //d

   //  rate=-vel_ff*axiaGainF[i];                    //ff
	rate = 0;
	rate += error*axiaGain[i];                     //p
	// rate+=(pos_error_last-error) *axiaGainD[i]; //d
	rate -= (curErrorI[i] * axiaGainI[i]);           //i

	if (i == (uc)AxisU)
	{
		rate = -rate;

		printf("cmd %5d Act %5d error %5d\r", (int)cmdPos[AxisU], (int)loc[AxisU], (int)rate);
	}

	rate = rate / (li)1000;

	if (i == (uc)AxisU)
	{

		if (abs(error) < 10) rate = 0;
		else
		{
			if (rate > 0)
			{
				if (rate < 25) rate = 25;
			}
			else
			{
				if (rate > -25) rate = -25;
			}

		}

	}

*/
	//if (i==(uc)0) printf("right %5d \r",(int)loc[0]);

  /*
  long int pos_last = motor_info[motor].pos;
  long int pos_error_last = motor_info[motor].pos_error;
  long int vel_last = motor_info[motor].vel;
  long int vel_error_last = motor_info[motor].vel_error;


  motor_info[motor].pos = Get_Encoder_Count(motor);

  if (PID_POS == motor_info[motor].pid_mode)
	{
	//  Position control
	motor_info[motor].pos_error = motor_info[motor].pos_cmd -
								  motor_info[motor].pos;
	motor_info[motor].pos_error_i += motor_info[motor].pos_error;
	motor_info[motor].pos_error_d = motor_info[motor].pos_error -
									pos_error_last;
	motor_info[motor].pwm = ( KP_P * motor_info[motor].pos_error) +
							((KI_P * motor_info[motor].pos_error_i)/pid_time) +
							( KD_P * motor_info[motor].pos_error_d);
	motor_info[motor].pwm /= DIV_P;




  //DeadBand Calculation

	if (rate < 0) rate -= 7;
	if (rate > 0) rate += 7;

	//clamp output range

	//if (rate<    -motorZero) rate=    -motorZero;
	//if (rate> 255-motorZero) rate= 255-motorZero;

	if (i == (uc)AxisR)
	{
		// pwm[12]=Limit255(motorZero+rate); //drill
		// pwm[13]=Limit255(motorZero+rate); //chip
		 //pwm[ 0]=Limit255(motorZero+rate); //bigwheel

		// printf("[0] rate:%4d \n",(int)rate);

		goto cExit;
	}

	if (i == (uc)AxisL)
	{
		//  pwm[14]=Limit255(motorZero+rate); //drill
		//  pwm[15]=Limit255(motorZero+rate); //chip
		//  pwm[ 1]=Limit255(motorZero+rate); //bigwheel

		 // printf("[1] rate:%4d \n",(int)Limit255(motorZero-rate));

		  //printf("pwm[1] %3d \n",pwm[1]);

		goto cExit;
	}

	//  if (i==AxisT || i==AxisU || i==aExtend) rate=-rate;

	 // pwm[i]=Limit255(motorZero+rate);

cExit:

	lastLoc[i] = (long int)loc[i];

	// printf("i:%4d rate:%4d",(int)i,(int)rate);

	*/
}




void closeLoopService(void)
{
	uc i;

	allFinished = 1;

	for (i = 0; i < (uc)7; i++) closeLoop(i);

	pid_time++;

}


uc skipOnce = 0;

void checkIfFinished(uc i)
{
	if (allFinished && !skipOnce) { tripTimer(i); BlockCloseLoopOff = 0; }

	skipOnce = 0;
}


void whenFinished(void(*motionFinished)(uc), int timeOut)
{
	BlockCloseLoopOff = 1;

	if (timeOut == 0) timeOut = 6000;

	skipOnce = 1;

	startTimer(8, timeOut, checkIfFinished, motionFinished);
}

