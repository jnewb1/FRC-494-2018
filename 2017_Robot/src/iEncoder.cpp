//
// BS file
//

#include "iEncoder.h"
#include "itimer.h"
#include "cal.h"

#include "WPILib.h"


#include "signal.h"
//#include "sigLib.h"		// for signal
#include <string>
#include "time.h"
//#include "usrLib.h"
#include "motor.h"

#include <stdio.h>

//extern int testFlag;
//extern int testCount;
//extern float tableSize;

// extern float shotErrorLimit;

// extern float toroIsDown;
// extern float toroIsUp;
// extern float toroIsCollecting;
 //extern float toroIsSpitting;
 //extern float toroInManual;

int forceFrontCamera = 0;

int resetArmTimeOut = 0;

float curToroRate = 0;

extern int blockToroAuto;

extern int forceSpeedZero;
extern int forceSpeedZeroBlock;

void setSpike(int index, int value);

double             timeOutBeginTime = 0;
double			   armMoveBaseTime = 0;
double			   armMoveBaseTime2 = 0;
float			   armMoveBaseLoc = 0;
float			   armMoveDestLoc = 0;
float			   armMoveTime = 0;
//float              armMoveTimeOut =0;
float			   armMoveFinished = 0;
float              armMoveFlag = 0;
float              armRateBegin = 0;
float              armRateEnd = 0;

double timeOutBaseTime = 0;

void  TimeOutReset()
{
	timeOutBaseTime = Timer::GetFPGATimestamp();

}
int  TimeOut(float time)
{
	if (Timer::GetFPGATimestamp() > timeOutBaseTime + time) return 1;

	return 0;
}


float armRate1 = 0;
float armAngle1 = 0;
float armRate2 = 0;
float armAngle2 = 0;

int skipCount = 0;

int armMoveTimeOut = 0;

double lastWantedArmAngleTime = 0;
float  lastWantedArmAngle = 0;

float armLastTime = 0;
float armLastLoc = 0;

float elevCmd = 30;
float elevDest = 30;

int shiftDelay = 0;


void GetSpeedHoodOffset(float Dist, float &Speed, float &Hood, float &Offset);

int forceWheel = 0;

float forceWheelCount = 0;

extern  float spare3;
extern  float spare4;


extern float wantedOffset;
extern float currentOffset;


extern float armRes;
extern float armGain;

extern int goalCountt;

extern int  goalColor[100];
extern int  goalX1[100];
extern int  goalX2[100];
extern int  goalY1[100];
extern int  goalY2[100];
extern int  goalCX[100];
extern int  goalCY[100];
extern int  goalWide[100];
extern int  goalHigh[100];
extern int  goalWideC[100];
extern int  goalHighC[100];
extern int  goalWideM[100];
extern int  goalHighM[100];
extern int  goalGap[100];
extern int  goalDistH[100];
extern int  goalDistY[100];


double lastShooterTime = 0;

float miniBot = 0;
float preDeploy = 0;

float shiftRight = 0;
float shiftLeft = 0;

//float deadZone( float ,float );

//extern SpeedController *jMotor[21];

float topRoller = 0;
float botRoller = 0;

//extern Servo *sServo[21];


//extern Gyro *gyro1;

//Analog Sample Variables

extern float dac[21];

int revCount[20] = { 0,0,0,0,0,0,0,0 };
int encNoWrap[20] = { 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 }; //{0,0,0,0,1,0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0};

AnalogInput *pot[20];

float rawAnalogSample[20];

float AnalogSample[20];

float loc[21];
float rawLoc[21];

float encZero[20];

int zeroResetFlag[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

void resetEncoder(uc i)
{
	return;

	if (i < 1) return;

	zeroResetFlag[i] = -1;

	revCount[i] = 0;

	loc[i] = 0;

	//  int ii=potSel[i]; if (ii==0) return;

	 // float newSample=pot[ii]->GetVoltage(); dac[i]=newSample;
	float newSample = pot[i]->GetAverageVoltage(); dac[i] = newSample;

	if (newSample < 0) newSample = 0;
	if (newSample > 5) newSample = 5;

	rawLoc[i] = newSample / 5;

	/*
	if (encNoWrap[i] || i==aArmElevation)
	{
	   encZero[i]=0;

	   AnalogSample[i]=newSample;
	}
	else
	{
	  encZero[i]=newSample;

	  AnalogSample[i]=newSample;
	}
	*/

	zeroResetFlag[i] = 5;
}

float distScale = 0.16978837945671410;


void setEncoder(int i, float newLoc)
{

	return;

	float curLoc = loc[i]; //if (i==aLeftDist) {curLoc=-curLoc; newLoc=-newLoc; }

	float curOff = encZero[i];

	float newOff = (curLoc - newLoc)*distScale * 5 / 1.33333333333333 + curOff;

	encZero[i] = newOff;

	loc[i] = newLoc;

}


void resetEncoders(void)
{
	for (int i = 1; i < 6; i++)
	{
		resetEncoder(i);
	}
}


float robotSpeed = 0;


float steerScale = -1;

double lastRevPos[20] = { 0,0,0,0,0,0,0,0,0,0,0 };
double lastRevNeg[20] = { 0,0,0,0,0,0,0,0,0,0,0 };

float shooterLastLoc = 0;
float shooterLastTime = 0;

/*
void ReadEncoder(int i)
{
	 if (i<1 || i>7 ) return;

	 double curTime=0;

	 //if (i==1) {loc[1]=getGyroAngle();return;}

	 if (zeroResetFlag[i]==-1) return;

	// int ii=potSel[i];if (ii==0) return;

	// float newSample=pot[i]->GetVoltage(); dac[i]=newSample;

	 float newSample=pot[i]->GetAverageVoltage(); dac[i]=newSample;

	 if (newSample<0) newSample=0;
	 if (newSample>5) newSample=5;

	 rawLoc[i]=newSample/5;

	 float sampleDelta=AnalogSample[i]-newSample;

	 AnalogSample [i]=newSample;

	 if (encNoWrap[i])
	 {
		revCount[i]=0;
	 }
	 else
	 {
		if (sampleDelta> 2.5) {curTime=frc::GetClock();printf("Pos %f\n",curTime-lastRevPos[i]);lastRevPos[i]=curTime;revCount[i]++;} // check for underflow wraparound
		if (sampleDelta<-2.5) {curTime=frc::GetClock();printf("Neg %f\n",curTime-lastRevNeg[i]);lastRevNeg[i]=curTime;revCount[i]--;} // check for overflow  wraparound
	 }

	 if (zeroResetFlag[i]>0) {

		 revCount[i]=0;

		 //if (i==aArmElevation && AnalogSample[aArmElevation]>4) revCount[aArmElevation]=-1;

		 zeroResetFlag[i]--;

	 }

	// if (i==aArmElevation)
	// {
	//    if (revCount[i]<-1) revCount[i]=-1;
	//    if (revCount[i]> 1) revCount[i]=1;
	// }

	// if (i==aAzimuth)
	// {
	//   if (revCount[i]> 3) revCount[i]=3;
	//   if (revCount[i]<-3) revCount[i]=-3;
	// }

	 float tmp=((float)revCount[i]+(AnalogSample[i]/5.0 ))-encZero[i]/5;

	  if (i==2 || i==3)

		 tmp/=distScale;

	  if (i==5) tmp*=-1.4619*2; //hood

	  if (i==aRightDist) tmp*= (12.0/9);
	  if (i==aLeftDist ) tmp*= (12.0/9);

	//  if (i==aArmElevation) tmp*=90;

	//  if (i==aAzimuth) tmp*=-37.0;

	  if (i==7) //calculate rpm - one rev is 1.0
				  {

					 // float shooterLastLoc=0;
					 // float shooterLastTime=0;



				//	  if (shooterCount==0)  shooterLastLoc=tmp;
				//	  if (shooterCount==9)  shooterRPM=(tmp-shooterLastLoc)*60*20;

				//	  shooterCount++;

				//	  if (shooterCount>9) shooterCount=0;

				//	  tmp=shooterRPM;


				  }


	  loc[i]=tmp;

}

*/

float lastSample1[20] = { 0,0,0,0,0,0,0,0,0 };
float lastSample2[20] = { 0,0,0,0,0,0,0,0,0 };

float sampleTrack[20] = { 0,0,0,0,0,0,0 };

void ReadEncoderSlow(int i, float minLevel, float delay)
{
	return;

	if (i < 3 || i>5) return;

	if (zeroResetFlag[i] == -1) return;

	float newSample = pot[i]->GetAverageVoltage();

	if (newSample < 0) newSample = 0;
	if (newSample > 5) newSample = 5;

	//if ( i==aAzimuth ) newSample=5-newSample; //|| i==aHood

	dac[i] = newSample;

	if (newSample == sampleTrack[i]) return;

	sampleTrack[i] = newSample;

	rawLoc[i] = newSample / 5;

	float sampleDelta = AnalogSample[i] - newSample;

	AnalogSample[i] = newSample;

	if (1) //encNoWrap[i] || 1  )
	{
		revCount[i] = 0;
	}
	else
	{

		if (sampleDelta > minLevel)  // check for underflow wraparound
		{
			double curTime = Timer::GetFPGATimestamp();

			if (curTime - lastRevPos[i] > delay)
			{
				lastRevNeg[i] = 0;

				lastRevPos[i] = curTime;

				revCount[i]++;

			}
		}

		if (sampleDelta < -minLevel)  // check for overflow  wraparound
		{
			float curTime = Timer::GetFPGATimestamp();

			if (curTime - lastRevNeg[i] > delay)
			{
				lastRevPos[i] = 0;

				lastRevNeg[i] = curTime;

				revCount[i]--;

			}
		}
	}

	if (zeroResetFlag[i] > 0)
	{
		revCount[i] = 0;

		zeroResetFlag[i]--;
	}


	float tmp = ((float)revCount[i] + (AnalogSample[i] / 5.0)) - encZero[i] / 5;

	// if (i==aAzimuth) tmp=tmp*-5;
	// if (i==aHood   ) tmp=(tmp-bumperIsDown)*1000.0 ;
	// if (i==aArm    ) tmp=tmp* armRes;

	loc[i] = tmp; sendBack(i, loc[i]);

}

void ReadEncoder(int i, float minLevel)
{
	return;

	if (i < 3 || i>6) return;

	if (zeroResetFlag[i] == -1) return;

	float newSample = pot[i]->GetAverageVoltage();

	// float newSample=pot[i]->GetVoltage();

	if (newSample < 0) newSample = 0;
	if (newSample > 5) newSample = 5;

	//if ( i==aAzimuth ) newSample=5-newSample; //|| i==aHood

	dac[i] = newSample;

	if (newSample == sampleTrack[i]) return;

	sampleTrack[i] = newSample;

	rawLoc[i] = newSample / 5;



	AnalogSample[i] = newSample;


	revCount[i] = 0;




	if (zeroResetFlag[i] > 0)
	{
		revCount[i] = 0;

		zeroResetFlag[i]--;
	}


	// float tmp=((float)revCount[i]+(AnalogSample[i]/5.0 ))-encZero[i]/5;

	float tmp = AnalogSample[i];

	// if (i==aArm-1) tmp=5.0-tmp;

	// if (i==aAzimuth) tmp=tmp*-5;
	// if (i==aHood   ) tmp=(tmp-bumperIsDown)*1000.0 ;
	// if (i==aArm    ) tmp=tmp* armRes;

	// if (i==aFeeder) tmp=(tmp*feederRes)+toroBlockAfterShotTime;
	// if (i==aElev  ) tmp=(tmp*elevRes  )+elevOff;
	// if (i==aArm   ) tmp=(tmp*armRes)+armOff;
	// if (i==aArm-1 ) tmp=(tmp*armRes)+armOff;

	loc[i] = tmp;

	// if (useLeftPot==0) loc[aArm]=loc[aArm-1];

	sendBack(i, loc[i]);

}


void ReadSonar(int i)
{
	return;

	if (i < 1 || i>8) return;

	float newSample = pot[i]->GetAverageVoltage();

	dac[i] = newSample;

	AnalogSample[i] = newSample;

	float tmp = AnalogSample[i];

	loc[i] = tmp * 4 * 12;

	sendBack(i, loc[i]);
}

float sQue[200];
int   sQueIndex = 0;




//void closeLoopPWM(int L);
void setPwm(int L, float val);
int setServo(int L, float val);

void Wait(double seconds);

//static void swap(int &A,int &B) {int tmp=A; A=B; B=tmp;}

double waitTime = 0;

//int spitIt=0;
//int rollIt=0;

int tripShift = 0;
int tripStopDrop = 0;

float suckSpeed = -1;
float spitSpeed = 1;

float shiftServoBeg = 1;
float shiftServoEnd = -1;

float stopDropServoBeg = -1;
float stopDropServoEnd = 1;

float reverseSkid = 0;

float disableGyro = 0;

int autoKick = 0;
//int liftFlag=0;

int gyroSetup = 300;

float gyroBase = 2.35; //2.5;
float gyroMin = 2.45;
float gyroMax = 2.45;

extern float gyroAng;

int gyroFault = 1;

int windItUp = 0;

int winchTimeOut = 0;

int newGyro = 1;
int gyroInRange = 0;

int gyroCount = 0;

float totalGyro = 0;

float gMin = 5.0;
float gMax = 0.0;

int newDone = 0;

int blockArmUser = 0;
int blockAzimthUser = 0;
int blockHoodUser = 0;


void resetGyro();

extern DigitalOutput *lidarSelectPtr1;
extern DigitalOutput *lidarSelectPtr2;

//extern DigitalInput *ballDetect1Ptr;			// digital inputs for line tracking sensors
//extern DigitalInput *ballDetect2Ptr;
//extern DigitalInput *ballDetect3Ptr;
//extern DigitalInput *ballDetect4Ptr;

extern DigitalInput *noRobotPtr;

extern  DigitalInput *encoder1A;
extern  DigitalInput *encoder1B;

extern  DigitalInput *encoder2A;
extern  DigitalInput *encoder2B;

extern  DigitalInput *encoder3A;
extern  DigitalInput *encoder3B;

extern Encoder *encoder1;
extern Encoder *encoder2;
extern Encoder *encoder3;

extern DigitalInput *shooterRevPtr;

//extern    DigitalInput *ballDetect1Ptr_Back;
//extern    DigitalInput *ballDetect2Ptr_Back;
//extern    DigitalInput *ballDetect3Ptr_Back;
//extern    DigitalInput *ballDetect4Ptr_Back;

extern    DigitalInput *noRobotPtr_Back;

extern    DigitalInput *encoder1A_Back;
extern    DigitalInput *encoder1B_Back;

extern    DigitalInput *encoder2A_Back;
extern    DigitalInput *encoder2B_Back;

extern    DigitalInput *encoder3A_Back;
extern    DigitalInput *encoder3B_Back;

extern    DigitalInput *shooterRevPtr_Back;


float ENC1A = 0;
float ENC1B = 0;
float ENC2A = 0;
float ENC2B = 0;

float shooterRev = 0;
float armDownLimit = 0;

float robotTime = 0;
double robotTimeBase = 0;

float noRobot = 0;

int lineIo = 0;

float eLoopCount = 0;

int rollUsed = 0;

double armUpStartTime = 0;
double armDownStartTime = 0;


extern int goalFrame;

float fmax(float val, float max)
{
	if (val < -max) return -max;
	if (val > max) return  max;

	return val;

}

extern float spare5;

int lastGoalFrame = 0;


float getShooterRPMBasedOnDistance()
{
	return 1000;
}

float getHoodBasedOnDistance()
{
	return 90;

}

double appQueEventTime[20];
int   appQueEvent[20];
int   appQueSelect[20];

int appQueBeg = 0;
int appQueEnd = 0;

#define appReadyEvent 1
#define appOpenEvent  0
#define appBot 0
#define appMid 1
#define appTop 2

void addAppQue(double time, int app, int event)
{
	appQueSelect[appQueEnd] = app;
	appQueEvent[appQueEnd] = event;
	appQueEventTime[appQueEnd] = time;

	appQueEnd++; if (appQueEnd > 19) appQueEnd = 0;
}

extern double imageAge;

float RPMoff = 0;

double shootBaseTime = 0;

float lastValue[20] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
float lastValueCount[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float lastGoodValue[20] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };

float filter(int index, float newValue, int samples, int &changed)
{
	if (newValue == lastValue[index]) lastValueCount[index]++; else lastValueCount[index] = 0;

	lastValue[index] = newValue;

	changed = 0;

	if (lastGoodValue[index] == -1) lastGoodValue[index] = newValue;

	if (lastValueCount[index] == samples)
	{
		if (newValue != lastGoodValue[index]) { lastGoodValue[index] = newValue; changed = 1; }
	}
	return lastGoodValue[index];
}

float filter(int index, float newValue, int samples)
{
	if (newValue == lastValue[index]) lastValueCount[index]++; else lastValueCount[index] = 0;

	lastValue[index] = newValue;

	if (lastGoodValue[index] == -1) lastGoodValue[index] = newValue;

	if (lastValueCount[index] == samples)
	{
		if (newValue != lastGoodValue[index]) { lastGoodValue[index] = newValue; }
	}
	return lastGoodValue[index];
}

float filter(int index, float newValue, int samples0, int samples1)
{
	if (newValue == lastValue[index]) lastValueCount[index]++; else lastValueCount[index] = 0;

	lastValue[index] = newValue;

	if (lastGoodValue[index] == -1) lastGoodValue[index] = newValue;

	if (newValue == 0 && lastValueCount[index] == samples0)
	{
		if (newValue != lastGoodValue[index]) { lastGoodValue[index] = newValue; }
	}
	else
		if (newValue && lastValueCount[index] == samples1)
		{
			if (newValue != lastGoodValue[index]) { lastGoodValue[index] = newValue; }
		}

	return lastGoodValue[index];
}

void grabFris()
{

}

float scaleRate(float rate)
{
	if (rate > 100) rate = 100;
	if (rate < -100) rate = -100;

	rate = rate / 100.0;

	return rate;
}

void setArmRate(float rate)
{
	wantedArmAngle = scaleRate(rate);
}

void armMove(float endAng, float rate, float maxTime)
{
	tprintf("ang %4.1f rate %4.1f mTime %4.1f \n", endAng, rate, maxTime);

	armMoveBaseTime = Timer::GetFPGATimestamp();

	armMoveTime = maxTime;

	//  armMoveTimeOut=armMoveBaseTime+armMoveTime;

	armMoveBaseLoc = loc[5];
	armMoveDestLoc = endAng;

	armRateBegin = rate;
	armRateEnd = rate;

	armMoveFinished = 0;
	armMoveFlag = 120;

	// float dist=armMoveDestLoc-armMoveBaseLoc;
}

void armMove(float ang1, float rate1, float ang2, float rate2)
{
	if (rate1 <= 100)
	{
		armMove(ang2, rate2, 1.5);

		return;
	}

	tprintf("a %4.1f r %4.1f a %4.1f r %4.1f\n", ang1, rate1, ang2, rate2);

	if (rate1 <= 0) rate1 = 10;
	if (rate2 <= 0) rate2 = 10;

	armMoveBaseTime = Timer::GetFPGATimestamp();

	armMoveBaseLoc = ang1;
	armMoveDestLoc = ang2;

	armRateBegin = rate1;
	armRateEnd = rate2;

	armMoveFinished = 0;
	armMoveFlag = 111;

	float dist = armMoveDestLoc - armMoveBaseLoc;

	float rate = (armRateBegin + (armRateEnd - armRateBegin) / 2);

	if (rate <= 10) rate = 10;

	armMoveTime = fabs(dist / rate);
}



void armMove(float dest, float rate)
{
	if (fabs(dest - loc[5]) <= 1.0)
	{
		armMoveFinished = 1;

		return;

	}



	if (rate <= 10) rate = 10;

	armMoveBaseTime = Timer::GetFPGATimestamp();
	armMoveBaseLoc = loc[5];
	armMoveDestLoc = dest;
	armMoveFinished = 0;
	armMoveFlag = 1;

	armMoveTime = fabs((dest - loc[5]) / rate);
}


void armSweep(float ang1, float rate1, float ang2, float rate2)
{
	armAngle1 = ang1;
	armAngle2 = ang2;

	armRate1 = rate1;
	armRate2 = rate2;

	armMoveFinished = 0;
	armMoveFlag = 130;
}

void ToroUpCmd();
void ToroDownCmd();

extern int blockToroUp;

//static int skipMe=0;

int lightCount = 0;

DriverStation *dsv = 0;

double bmTimeBeg = 0;
// double bmTimeEnd=0;
double bmTimeLast = 0;
double bmDeltaTime = 0;
double bmNextTime = 0;

double baseTestTime = 0;
float baseTestValue = 0;

extern float potValue1;
//extern float potValue2;


float azmAngle = 0;
float elboAngle = 0;

extern AnalogPotentiometer *pot1, *pot2;



int initEncoderTask()
{
	// printf(" Exit Encoder Task\n");

	return 0;

	printf("initEncoderTask\n");

	//encoderInit();



	//pot1= new AnalogPotentiometer(2);
   // pot2= new AnalogPotentiometer(2);
   // pot3= new AnalogPotentiometer(3);

//return 0;

	for (;;)
	{

		// float potValue1=pot1->Get(); dac[1]=potValue1;
		// potValue2=pot2->Get(); dac[2]=potValue2;
		// potValue3=pot3->Get(); dac[3]=potValue3;

		// azmAngle =potValue1*360.0;
		// elboAngle=potValue2*360.0;

		// printf("pot1 %f pot2 %f\n",azmAngle,elboAngle);
		// printf("pot1 %f\n",potValue1);
	  // bmTimeBeg=frc::GetClock();

	  //if (bmNextTime==0) bmNextTime=bmTimeBeg; else bmNextTime+=0.005;

	  //tprintf("time %f\n",1.0/(bmTimeBeg-bmTimeLast));


	 // bmTimeLast=bmTimeBeg;


	  //bmDeltaTime=bmTimeBeg-bmTimeLast;

	  //bmTimeLast=bmTimeBeg;

	 // eLoopCount++;

	//  ReadEncoder  (aFeeder ,1   );
	//  ReadEncoder  (aElev   ,1   );
	 // ReadEncoder  (aArm    ,1   );
	//  ReadEncoder  (aArm-1    ,1   );

	//  ReadSonar( aSonar );


	 // ReadSonar();

//extern int testFlag;
//extern int testCount;
//extern float tableSize;

		 /*

	  if (testFlag)
	  {
		  if (testCount==0) { baseTestTime=frc::GetClock(); baseTestValue=loc[5]; }

		  table1[testCount]=testCount;
		  table2[testCount]=loc[5];
		  table3[testCount]=frc::GetClock()-baseTestTime;

		  tableSize=testCount;

		  testCount++;

		  if (testCount>199) testFlag=0;

	  }



	  if (dsv==0) dsv = DriverStation::GetInstance();

	  if (dsv) dac[8] = dsv->GetBatteryVoltage();

	  getGyroAngle();

	  int inZone=0;

	  if (A_ShotDistBeg ) if(A_ShotDistBeg <=loc[3] && (A_ShotDistBeg + A_ShotDistZone)>=loc[3]) inZone=1;
	  if (B_ShotDistBeg ) if(B_ShotDistBeg <=loc[3] && (B_ShotDistBeg + B_ShotDistZone)>=loc[3]) inZone=2;
	  if (X_ShotDistBeg ) if(X_ShotDistBeg <=loc[3] && (X_ShotDistBeg + X_ShotDistZone)>=loc[3]) inZone=3;
	  if (Y_ShotDistBeg ) if(Y_ShotDistBeg <=loc[3] && (Y_ShotDistBeg + Y_ShotDistZone)>=loc[3]) inZone=4;
	  if (RS_ShotDistBeg) if(RS_ShotDistBeg<=loc[3] && (RS_ShotDistBeg+RS_ShotDistZone)>=loc[3]) inZone=5;
	  if (RH_ShotDistBeg) if(RH_ShotDistBeg<=loc[3] && (RH_ShotDistBeg+RH_ShotDistZone)>=loc[3]) inZone=6;
	  if (LH_ShotDistBeg) if(LH_ShotDistBeg<=loc[3] && (LH_ShotDistBeg+LH_ShotDistZone)>=loc[3]) inZone=7;

	  extern float spare3;


	  spare3=inZone;


	  lightCount++;

	  if (inZone)
	  {
			 if (lightCount>lightsOffTimeInZone+lightsOnTimeInZone) lightCount=0;

			 if (lightCount<lightsOnTimeInZone) setSpike(1,1); //turn lights on

			  else setSpike(1,0); //turn lights off


	  }
	  else
	  {

	  if (lightCount>lightsOffTime+lightsOnTime) lightCount=0;

	  if (lightCount<lightsOnTime) setSpike(1,1); //turn lights on

			  else setSpike(1,0); //turn lights off

	  }


	  if (lineIo)
	  {
	   if (noRobotPtr) noRobot = noRobotPtr ->Get() ? 0 : 1;

	   //if (noRobot && lineIo) lineIo--;

	  // if (!noRobot)
	   {
		if (encoder1A) ENC1A      = encoder1A    ->Get() ? 1 : 0;
		if (encoder1B) ENC1B      = encoder1B    ->Get() ? 1 : 0;

		if (encoder2A) ENC2A     = encoder2A    ->Get() ? 1 : 0;
		if (encoder2B) ENC2B     = encoder2B    ->Get() ? 1 : 0;

		if (shooterRevPtr) shooterRev    = shooterRevPtr   ->Get() ? 1 : 0;



		if (ballDetect1Ptr) ballDetect1=filter(1,ballDetect1Ptr->Get() ? 0 : 1,30,3);
		if (ballDetect2Ptr) ballDetect2=filter(2,ballDetect2Ptr->Get() ? 0 : 1,30,3);
		if (ballDetect3Ptr) ballDetect3=filter(3,ballDetect3Ptr->Get() ? 0 : 1,30,3);
		if (ballDetect4Ptr) ballDetect4=filter(4,ballDetect4Ptr->Get() ? 0 : 1,30,3);

		// filter test

	   // int changed=0;

	   // float fBallDetect1=filter(1,ballDetect1,3,changed);


	   // if (changed && fBallDetect1==0) minHotRatio+=0.001;


		//-----------------

	   }
	 }


	  if (forceFrontCamera)
	  {
		  cameraSelect=0;

		  forceFrontCamera--;
	  }
	  else
	  if (cameraSelect<2)
	  {
	  if (joyY[2]> 0.1) cameraSelect=0;
	  if (joyY[2]<-0.1) cameraSelect=1;

	  if (joyY[3]> 0.1) cameraSelect=0;
	  if (joyY[3]<-0.1) cameraSelect=1;
	  }



	 if (holdStraight && fabs(joyX[1])>0.05) targetOffsetX+=joyX[1]/8;

	 if (holdStraight && gyroShift && joyY[1]> 0.5) gyroShiftState=1;
	 if (holdStraight && gyroShift && joyY[1]<-0.5) {gyroShiftState=2; }
	 if ( gyroShift && joyY[1]<-0.5) {holdStraight=1;}

	 if (fabs(joyY[2])>0.5 || fabs(joyY[2])>0.5) { blockDriving=0;DriveModeFlag=1; }

	// if (fabs(joyY[2])>0.7 ) { holdStraight=0;}

	 if (fabs(joyY[2]+joyY[3])>0.05 && fabs(joyX[2]+joyX[3])>0.01) { holdStraight=0;}

	 if (fabs(joyY[2]+joyY[3])>0.075) resetArmTimeOut=1;

	 if (joyY[2]+joyY[3]>0.3 && toroIsDown && !blockToroAuto && fabs(curToroRate)<0.2) if (!blockToroUp) {if (loc[5]<50)ToroUpCmd();} else { if (blockToroUp>0) blockToroUp--; }

	// if (loc[5]<42) blockToroUp =0;

	// extern int driverSelect;

	// if (driverSelect==0)
	// {
	//
	//	 if (fabs(joyY[1])>0.5) driverSelect=1;
	//     if (fabs(joyY[2])>0.5) driverSelect=2;
	//
	// }


	 drivingService();

	 // --- Shooter Conntol Code --- 0:shooter Off 1:shooter Auto [2..100]:shooter fixed volts /100  [>100] RPM

	 //static float trackShooterVolts=0;
	 //
	 //float wantedShooterVolts=shooterSpeed;
	 //
	 //if (wantedShooterRPM==0) {wantedShooterVolts=0; WheelAtSpeed=0;}
	 //
	 //if (wantedShooterRPM>-100 && wantedShooterRPM<=100) wantedShooterVolts=wantedShooterRPM/100.0;
	 //
	 //if (wantedShooterRPM>100 || wantedShooterRPM==1 )
	 //{
		//   RPMoff=0;
		//
	 //      float wRPM=wantedShooterRPM+RPMoff+moveBackRate;
	 //
	 //      if (wRPM==1) wRPM=getShooterRPMBasedOnDistance()+RPMoff+moveBackRate;
	 //
	 //      float deltaTime=frc::GetClock()-lastShooterTime;
  //
	 //      if (deltaTime>1) shooterRPM=0; //something is wrong
	 //
	 //      float errorRPM=wRPM-shooterRPM;
	 //
	 //      float pulseRate=(wRPM/ShooterPulseMaxRPM+ShooterPulseOffset); // 2000.0);
	 //
	 //      if (pulseRate>0.999) pulseRate=0.999;
	 //      if (pulseRate<0.08 ) pulseRate=0.08;
	 //
	 //      if (errorRPM>0) wantedShooterVolts=pulseRate; else wantedShooterVolts=pulseRate*sweepRate;
	 //
	 //      if (errorRPM<-100) wantedShooterVolts=-0.1;
	 //
	 //      if (errorRPM>-150  && errorRPM<100) WheelAtSpeed=1; else WheelAtSpeed=0;
	 //
	 //      if (spare4>0 && errorRPM>spare4)
	 //               wantedShooterVolts=1.0;
	 //}
	 //
	 //if (lightsOnTime<=0) shooterSpeed=wantedShooterVolts;
	 //else
	 //{
	 //      	if (wantedShooterVolts>trackShooterVolts) {trackShooterVolts+=lightsOnTime/2; if (trackShooterVolts>wantedShooterVolts) trackShooterVolts=wantedShooterVolts; }
	 //   	if (wantedShooterVolts<trackShooterVolts) {trackShooterVolts-=lightsOnTime/2; if (trackShooterVolts<0                 ) trackShooterVolts=0; }
	 //}
	 //
	 //if (wantedShooterVolts<trackShooterVolts) shooterSpeed=wantedShooterVolts;
	 //else shooterSpeed=trackShooterVolts;
	 //
	 //if (wantedShooterRPM>-100 && wantedShooterRPM<=100) if(fabs(wantedShooterVolts-trackShooterVolts)<0.05) WheelAtSpeed=1; else WheelAtSpeed=0;
	 //
	 //if (shooterSpeed>0 && forceWheel>0) {shooterSpeed=1.0; forceWheel--;}

	// if (toroIsSpitting && shooterSpeed>toroIsSpitting)
	//	 setPwm(pwmShooter ,toroIsSpitting);
	 /else
	 //setPwm(pwmShooter ,shooterSpeed);
	// setPwm(pwmShooter2,shooterSpeed);

	 //---- get encoder info for right and left drive trains -------

	// float res=ShiftLowRes;

	// if (ShiftModeFlag==1) res=ShiftHighRes;

	 // need to detect change is shift to adjust count (maybe zero count?)

	 if (ShiftHighRes==0) ShiftHighRes=49.0;
	 if (ShiftLowRes ==0) ShiftLowRes =49.0;

	 if (encoder1) loc[1]= (float) encoder1->GetRaw() /ShiftLowRes;    else loc[1]=0;
	 if (encoder2) loc[2]= (float) encoder2->GetRaw() /ShiftHighRes;   else loc[2]=0;

	// if (encoder1) minHotRatio=(float)encoder1->GetRaw()/1000.0;

	 //-----------------------------------------------------------------

	// static int tgl=0;
	 //static int tgl2=0;

	// int    cenX  =0;
	// double ispeed=0;

	 //if (fastVideo || imageAge>1.0) {setPwm(pwmAzimuth  ,fmax(-(joyX2[2]+joyX2[1]),1.0));goto skip;}

	 //if (goalCountt==0){setPwm(pwmAzimuth,fmax(-(joyX2[2]+joyX2[1]),1.0));goto skip;}
		 //sCARD goalCX   [100];
		 //sCARD goalCY   [100];

	 //if (goalFrame==lastGoalFrame) goto skip;
	 //
	 //lastGoalFrame=goalFrame;
	 //
	 //ispeed=0;

	 //int X=goalCX[0];
	 ////  int Y=goalCY[0];

	 //cenX=(int)crossHairX;//+(joyX2[2]+joyX2[1])*bumperDownRate;
	 //
	 //ispeed=(X-cenX)*3; //+I/2;
	 //
	 //float spd=ispeed*turretGain;
	 //
	 //spd=spd*spd;
	 //
	 ////if (spd) spd+=0.05;
	 //
	 //if (ispeed<0) spd=-spd;
	 //
	 //if (fabs(X-cenX)>0)
	 //{
	 //   if (spd<0 && spd>-useLeftPot) spd=-useLeftPot;
	 //   if (spd>0 && spd< useLeftPot) spd= useLeftPot;
	 //}
	 //spd=fmax(spd,1.0);
	 //
	 ////if (  fabs( (fabs(spd)-useLeftPot) )<useLeftPot )
	 ////{
	 ////    if (tgl==0)  setPwm(pwmAzimuth,spd);  else setPwm(pwmAzimuth,0);
	 ////}
	 ////else
	 ////{
	 ////  	 setPwm(pwmAzimuth,spd);
	 ////}
	 //
	 //tgl++; if (tgl>2) tgl=0;
	 //
	 //skip:
	 //
	 //
	 //-------- Shifter Control   0:low 1:high

	 if (ShiftModeFlag==0) {setPwm(pwmRightShifter, rightShiftLow ); setPwm(pwmLeftShifter, leftShiftLow ); }
	 if (ShiftModeFlag==1) {setPwm(pwmRightShifter, rightShiftHigh); setPwm(pwmLeftShifter, leftShiftHigh); }


	 // Shooting control (shooterState<200) (original shooting control)

	 if (shooterState>=100 && shooterState<200)
	 {
		   if (shooterState==100) //move to load location
		   {
			   tprintf("--------------------------------------\n");
			   tprintf("MoveTo Ball Load Location\n");

			   armManualFlag=0; //this is auto only

			   armMove(ballLoadLoc,ballLoadRate);

			   if ((wantedArmAngle==0 && !IsDisabled()) || fabs(ballLoadLoc-loc[5]) > 10 )
			   {
				  // if (inAton) {tprintf("inAton\n"); }
				   //if (!IsDisabled()) {tprintf("!IsDisabled\n"); }

				   if (inAt && !IsDisabled())
				   {
					   shooterState=101;//tprintf("101\n");
				   }

					   else
					   {

						   //tprintf("104\n");
				   shooterState=104; // setup for load only
				}
			   }

			   else

				   shooterState=101;
		   }


		   if (shooterState==101 && armMoveFinished)
		   {
			//   tprintf("---> Shot First Motion\n");

			//   armMove(fShotAngle1,fShotRate1,fShotAngle2,fShotRate2);

			 //  shooterState=102;

			  if (sweepRate>0)
			  {

			  armSweep(sShotAngle1,sweepRate,sShotAngle2,sShotRate2);

			  shooterState=103;

			  }
			  else
			  {
				 armMove(fShotAngle1,fShotRate1,fShotAngle2,fShotRate2);

				 shooterState=102;


			  }

		   }

		   if (shooterState==102 && armMoveFinished)
		   {


			   tprintf("---> Shot Second Motion\n");

			   armMove(sShotAngle1,sShotRate1,sShotAngle2,sShotRate2);

			   shooterState=103;
			}

			if (shooterState==103 && armMoveFinished)
			{

			   tprintf("---> Move To Ball Load Location\n");

			   armMove(sShotAngle2,ballLoadRate,ballLoadLoc,ballLoadRate);

			   shooterState=104;
			}

			if (shooterState==104 && armMoveFinished)
			{
			   shooterState= 0;

			   tprintf("---> [Shot Finished] -----------------\n");

			  //blockToroUp =0;

			   if (IsDisabled()) wantedArmAngle = 0;

			   blockToroUp =toroBlockAfterShotTime*80; //240
			}

	   }







	   // Actual arm shots 1=simple 111=complex 120=fast



	   if (armMoveFlag==1 || armMoveFlag==2)
	   {
		   if (armMoveFlag==1)
		   {

			  tprintf("Simple %5.2f -> %5.2f @t %f\n",armMoveBaseLoc,armMoveDestLoc,armMoveTime);

			  armMoveFlag=2;
		   }

		   double curTime=frc::GetClock();

		   double deltaTime=curTime-armMoveBaseTime;

		   float error=fabs(loc[5]-wantedArmAngle);

		  // tprintf("dt %5.2f err %5.2f amt %5.2f \n",deltaTime,error,armMoveTime);


		   if (deltaTime>armMoveTime && (fabs(error)<shotErrorLimit*4 || IsDisabled() || deltaTime>armMoveTime+5)) {

			   deltaTime=armMoveTime; armMoveFinished=1; armMoveFlag=0;//iArm=0;

			//   tprintf("[sEnd] cmd%4.1f act%4.1f err%4.1f t%5.2f \n",wantedArmAngle,loc[5],loc[5]-wantedArmAngle,deltaTime);

		   }

		   float armMovePercent=deltaTime/armMoveTime;

		   if (armMovePercent>1.0) armMovePercent=1.0;

		   float newLoc=armMoveBaseLoc+armMovePercent*(armMoveDestLoc-armMoveBaseLoc);

		   wantedArmAngle=newLoc;



		   //tprintf("angle %f\n",wantedArmAngle);

	//	   if (skipMe==0) {tprintf("ANGLe %f\n",wantedArmAngle);}

	//	   skipMe+=1; if (skipMe>10) skipMe=0;


		   float dTime=curTime-armLastTime;
		   float dLoc =loc[5]-armLastLoc;

		  // float rate=dLoc/dTime;

		  // if (dLoc) tprintf("rate %f %f %f\n",rate,dTime,dLoc);

		   armLastTime=curTime;
		   armLastLoc =loc[5];


	   }

	   if (armMoveFlag==111 || armMoveFlag==112)
		   {
			  if (armMoveFlag==111)
			  {

				tprintf("Complex %5.2f -> %5.2f @t %f\n",armMoveBaseLoc,armMoveDestLoc,armMoveTime);

				armMoveFlag=112;
			  }


			   double deltaTime=frc::GetClock()-armMoveBaseTime;

			   float error=fabs(loc[5]-wantedArmAngle);

			   if (deltaTime>armMoveTime  && (fabs(error)<shotErrorLimit || IsDisabled() || deltaTime>armMoveTime+5))
			   {
				   deltaTime=armMoveTime;

				   armMoveFinished=1;

				   armMoveFlag=0;

				   wantedArmAngle=armMoveDestLoc;

				//   tprintf("[End] cmd%4.1f act%4.1f err%4.1f t%4.2f \n",wantedArmAngle,loc[5],loc[5]-wantedArmAngle,deltaTime);


			   }
			   else
			   {
			   float dTime=fabs(armMoveTime);

			   if (dTime<0.001) dTime=0.001;

			   if (deltaTime>armMoveTime) deltaTime=armMoveTime;

			   float a =deltaTime;
			   float aa=deltaTime/dTime;
			   float b=armRateBegin;
			   float c=armRateEnd;
			   float dist=a*( b+(c-b)/2*aa );

			   float newLoc=armMoveBaseLoc+ dist;

			   if (armMoveDestLoc<armMoveBaseLoc) newLoc=armMoveBaseLoc-dist;

			   wantedArmAngle=newLoc;

	// 		   if (skipMe==0) {
	 //
	// 			  tprintf("t%4.1f b %4.1f e %4.1 f d%4.1f l%4.1f\n",a,armRateBegin,armRateEnd,dist,newLoc);
	// 			  tprintf("ANGLe %f\n",wantedArmAngle);
	// 					   }
	//
	// 		   skipMe+=1; if (skipMe>10) skipMe=0;


			   }

	   }

	  if (armMoveFlag==120 || armMoveFlag==121 || armMoveFlag==122 || armMoveFlag==123)
	  {
		  if (armMoveFlag==120)
		  {

			tprintf("Fast %5.2f -> %5.2f @ %f\n",armMoveBaseLoc,armMoveDestLoc,armRateBegin);

			armMoveFlag=121;
		  }

		  if (armMoveFlag==122)
		  {
			 //tprintf("armMoveFlag==122\n");

			 armMoveBaseTime2=frc::GetClock();

			 armMoveFlag=123;

			 wantedArmAngle=-0.99;



		  }

		  if (armMoveFlag==123)
		  {
			  float deltaTime=frc::GetClock()-armMoveBaseTime2;

				 wantedArmAngle=-0.9; //-0.99;

				 if (loc[5]<armMoveDestLoc || deltaTime>0.2)
				 {

				 //    tprintf("BackTime %5.2f\n",deltaTime);
					 armMoveFinished=1;

					 armMoveFlag=0;

					 wantedArmAngle=armMoveDestLoc;

					 forceSpeedZero=0;
					 forceSpeedZeroBlock=0;

				//   tprintf("[fEnd] cmd%4.1f act%4.1f err%4.1f  \n",wantedArmAngle,loc[5],loc[5]-wantedArmAngle);
				 }






		  }

		  if (armMoveFlag==121)
		  {

			//  tprintf("armMoveFlag==121\n");
		  if (armRateBegin>100) armRateBegin=100;

		  wantedArmAngle=armRateBegin/100;

		  double deltaTime=frc::GetClock()-armMoveBaseTime;

		  forceSpeedZero=10;

		  if (deltaTime>armMoveTime  ||  loc[5]>armMoveDestLoc)
		  {
			   deltaTime=armMoveTime;

			   //armMoveFinished=1;

			   armMoveFlag=122;

			  // wantedArmAngle=armMoveDestLoc;

			  // tprintf("[fEnd] cmd%4.1f act%4.1f err%4.1f t%4.2f \n",wantedArmAngle,loc[5],loc[5]-wantedArmAngle,deltaTime);
		   }
		  }

	   }

	  // --------------------------------
	  // armMoveFlag  = 130 sweep shot
	  //---------------------------------

	  if (armMoveFlag>=130 && armMoveFlag<=133)
	  {
		  if (armMoveFlag==130)
		  {
			tprintf("Sweep %5.2f @ %5.2f -> %5.2f @ %5.2f\n",armAngle1,armRate1,armAngle2,armRate2);

			tprintf("Sweep %5.2f -> %5.2f @ %5.2f\n",loc[5],armAngle1,armRate1);

			TimeOutReset();

			armMoveFlag=131;
		  }

		  if (armMoveFlag==131)
		  {
			setArmRate(armRate1);

			if (!forceSpeedZeroBlock) forceSpeedZero=10;

			if (TimeOut(2)  ||  loc[5]>armAngle1)
			{
			   armMoveFlag=132;

			   tprintf("Sweep1 %5.2f ->%5.2f @ %5.2f\n",loc[5],armAngle2,armRate2);

			   TimeOutReset();
			}
		  }

		  if (armMoveFlag==132)
		  {
			// float delta=armAngle2-loc[5];

			// if (delta<5)
			// {
			//	 if (delta<0) delta=0;

			//	 setArmRate(armRate2*(5-delta)/5.0);
			// }
			// else

			setArmRate(armRate2);

			forceSpeedZero=10;

			if (TimeOut(2)  ||  loc[5]>armAngle2)
			{
				tprintf("SweepStop %5.2f @ -99\n",loc[5]);

				 TimeOutReset();

				 armMoveFlag=133;

				setArmRate(-99);
			 }
		  }

		  if (armMoveFlag==133)
		  {
			  if (loc[5]<armAngle2 || TimeOut(2))
			  {
				  tprintf("SweepDone %5.2f\n",loc[5]);

				 armMoveFinished=1;

				 armMoveFlag=0;

				 wantedArmAngle=armAngle2;

				 forceSpeedZero=0;
				 forceSpeedZeroBlock=0;
			  }
		  }
	   }


	  // closed loop arm control wantedArmAngle>1  or  open loop current wantedArmAngle<1

	   if (wantedArmAngle)
	   {

		   float rate=0;

		   if (wantedArmAngle>1)
		   {

		   double curTime=GetTime();

		   bool disabled=IsDisabled();

		   if (disabled || fabs(wantedArmAngle-lastWantedArmAngle)>3 || resetArmTimeOut)
		   {
			   lastWantedArmAngle=wantedArmAngle;

			   lastWantedArmAngleTime=curTime;

			   resetArmTimeOut=0;

		   }

		   double timeOutDelta=curTime-lastWantedArmAngleTime;

		   if (timeOutDelta>66 && armMoveTimeOut==0)  lastWantedArmAngleTime=curTime;

		   if ( armMoveTimeOut==0 &&  timeOutDelta>60 && timeOutDelta<66)
		   {
			   if (!disabled)
			   {
				  tprintf("Shooting Arm Idle[ %f ]\n",curTime-lastWantedArmAngleTime);

				  armMoveTimeOut=1; timeOutBeginTime=curTime;
			   }
		   }

		   //tprintf("Wanted %f\n",wantedArmAngle);

		   if (wantedArmAngle>195) wantedArmAngle=195;
		   if (wantedArmAngle<1  ) wantedArmAngle=1;

		   float error=wantedArmAngle-loc[5];

		   if (error<-0.5) iArm-=iArmGain/100;
		   if (error> 0.5) iArm+=iArmGain/100;

		   if (iArm<iArmMin) iArm=iArmMin;
		   if (iArm>iArmMax) iArm=iArmMax;

		   float volts=iArm+error*armGain;

		   if (volts>1) volts=1;
		   if (volts<-1) volts=-1;

		   rate=volts;

		   if (armMoveTimeOut)
		   {
			   if ((curTime-timeOutBeginTime)>2) { tprintf("Idle Arm Reset\n"); armMoveTimeOut=0; lastWantedArmAngleTime=curTime; }

			   rate/=1.1;

			   wantedArmAngle=loc[5];

			   armMoveFinished=1;

			   armMoveFlag=0;

			   shooterState=0;

			   iArm=0;
		   }
		   }
		   else
		   {
			   rate=wantedArmAngle;
		   }

		   setPwm(pwmShooterR1,rate);
		   setPwm(pwmShooterR2,rate);
		   setPwm(pwmShooterL1,rate);
		   setPwm(pwmShooterL2,rate);
	   }



	   {

			//extern float toroIsDown;
			//extern float toroIsUp;
			//extern float toroIsCollecting;
			//extern float toroIsSpitting;
			//extern float toroInManual;


			  static float lastRate=0;

			  float rate=-joyT[2]-joyT[1]; //-joyT[3];

			  if (fabs(rate)>0.1)
			  {
				  if (rate>0)
				  {
					  rate=rate*(1+fabs(joyY[2]+joyY[3])); if (rate>2) rate=2;
				  }
				  else
				  {
					  rate=rate*(1+fabs(joyY[2]+joyY[3])); if (rate<-2) rate=-2;
				  }


			  }

			  curToroRate=rate;

			 // tprintf("Rate %f\n",rate);

			  if (fabs(rate-lastRate)>0.1 || toroInManual)
			  {

				  resetArmTimeOut=1;

				if (rate> 0.1) toroIsCollecting=1; else toroIsCollecting=0;
				if (rate<-0.1) toroIsSpitting  =1; else toroIsSpitting  =1;

				toroInManual    =1;

			  setPwm(pwmRoller1,rate/2+0.05);
			  setPwm(pwmRoller2,rate/2+0.05);

			  lastRate=rate;

			  }







	   }


	   if (fabs(joyY2[2])>0.1 && !toroIsDown) ToroDownCmd();

	if (armManualFlag==0)
	{
	   if (IsDisabled()) wantedArmAngle=loc[5];

		  if (shooterState==0 && wantedArmAngle==0)
		  {
			  wantedArmAngle=loc[5]; if (wantedArmAngle==0) wantedArmAngle=0.1;
		  }
		  else
		  {
			  if (fabs(joyY2[2])>0.1)
			  {
				armMoveFlag=0;
				shooterState=0;
				elevFlag=0;
			  }

			  if (shooterState==0 && wantedArmAngle>0 && fabs(joyY2[2])>0.05)
			  {
				 if (joyY2[2]<0) wantedArmAngle+=(joyY2[2]/1.5+0.05)*(joyY2[2]/1.5+0.05)*-5;
				 else wantedArmAngle+=(joyY2[2]/1.5-0.05)*(joyY2[2]/1.5-0.05)*5;

				if (wantedArmAngle>195) wantedArmAngle=195;
				if (wantedArmAngle<ballLoadLoc) wantedArmAngle=ballLoadLoc;

			  }
		  }
	}

	if (armManualFlag)
	{
	   if (shooterState==0)
			  {
				  setPwm(pwmShooterR1,joyY2[2]);
				  setPwm(pwmShooterR2,joyY2[2]);
				  setPwm(pwmShooterL1,joyY2[2]);
				  setPwm(pwmShooterL2,joyY2[2]);
			  }
			  else

				  if (fabs(joyY2[2])>0.3) {shooterState=0; elevFlag=0;}
	 }


	// bmTimeEnd=frc::GetClock();

	//if (bmDeltaTime<0.006)

		//taskDelay(5); //printf(" %f\n",bmDeltaTime);

	//printf(" serviceTime %f\n",1.0/(bmTimeEnd-bmTimeBeg) );



	//printf(" %f\n",1.0/bmDeltaTime);

	  bmDeltaTime=bmNextTime-frc::GetClock();

	  if (bmDeltaTime<-0.1 || bmDeltaTime>0.1)
	  {
		  bmNextTime=0;

		  tprintf("timing error %f\n",bmDeltaTime);

		  Wait(0.005);

	  }
	  else
	  {
		  int delayTicks=bmDeltaTime*1000;

		  if (delayTicks>0) Wait(bmDeltaTime); //(delayTicks);

	  }

*/
//		 Wait(0.005);

	}

	return 1;
}

int blockToroUp = 0;

void SendTorrowsDown(void(*serviceAddress)());

void makeTheShot()
{
	tprintf("makeTheShot\n");

	//if (shooterState) { tprintf("shooterState %f\n", shooterState); return; }

	//shooterState = 100;

	forceFrontCamera = 300;

	blockToroUp = -1;
}

extern int toroDownWhileShooting;


void ShootBall(float ShotAngBeg, float ShotAngEnd, float ShotSpeed)
{

	//if (shooterState) { tprintf("Shot Reject [%5.2f]\n", shooterState); return; }
/*
	fShotAngle1 = ShotAngBeg;
	fShotRate1 = ShotSpeed;

	fShotAngle2 = ShotAngBeg;
	fShotRate2 = ShotSpeed;

	sShotAngle1 = ShotAngBeg;
	sShotRate1 = ShotSpeed;

	sShotAngle2 = ShotAngEnd;
	sShotRate2 = ShotSpeed;
*/

	if (ShotSpeed <= 100)
	{
		//fShotAngle1 = ballLoadLoc;
		//fShotRate1 = ballLoadRate;
		//fShotRate2 = ballLoadRate;
	}


	// shooterState=100;

	 //	toroDownWhileShooting=1;


	SendTorrowsDown(makeTheShot);

	forceFrontCamera = 300;



}



void ShootBallNT(float ShotAngBeg, float ShotAngEnd, float ShotSpeed)
{
/*
	fShotAngle1 = ShotAngBeg;
	fShotRate1 = ShotSpeed;

	fShotAngle2 = ShotAngBeg;
	fShotRate2 = ShotSpeed;

	sShotAngle1 = ShotAngBeg;
	sShotRate1 = ShotSpeed;

	sShotAngle2 = ShotAngEnd;
	sShotRate2 = ShotSpeed;
*/
	//shooterState = 100;

	forceFrontCamera = 300;


}

void forceShot()
{

	makeTheShot();

}


void closeLoopPWM(int L, int pwmSel)
{
	return;

	float error = (cmdLoc[L] - cmdOff[L]) - loc[L];
	float rate = error*cmdGain[L];

	if (rate < 0) rate -= cmdDead[L];
	if (rate > 0) rate += cmdDead[L];

	if (rate < -127) rate = -127;
	if (rate > 127) rate = 127;

	// pwm[L]=(unsigned char) (128+rate);

	float speed = rate / 128.0;

	// if (steeringOff && (L==aFrontSteer || L==aBackSteer)) speed=0;

	if (forceCount[pwmSel] > 0)
	{
		speed = forceValue[pwmSel];

		forceCount[pwmSel]--;
	}

	setPwm(pwmSel, speed);
}


int setMotor(int L, float speed);

void setPwm(int L, float val)
{
	if (L <= 0) return;

	if (val > 1) val = 1;
	if (val < -1) val = -1;

	if (forceCount[L] > 0)
	{
		val = forceValue[L];

		forceCount[L]--;
	}

	if (!setMotor(L, val)) setServo(L, val);
}

void setPwmFg(int L, float val)
{
	if (L <= 0) return;

	if (val > 1) val = 1;
	if (val < -1) val = -1;

	if (forceCount[L] > 0)
	{
		val = forceValue[L];

		forceCount[L]--;
	}

	if (!setMotor(L, val)) setServo(L, val);
}


/*
void setServo(int L,float val)
{

	if (val> 1) val= 1;
	if (val<-1) val=-1;

	if (forceCount[L]>0)
	{
	   val=forceValue[L];

	   forceCount[L]--;
	}

   if (sServo[L]) {sServo[L]->Set(val);sendBack(20+L,val);}
   else sendBack(20+L,-2);
}
*/

float DeadZone(float base, float zone)
{

	//if (fabs(base)<zone) return 0;


	if (base < 0)
	{

		if (-base < zone) return 0;

		return -(-base - zone) / (1.0 - zone);

	}
	else
	{

		if (base < zone) return 0;

		return (base - zone) / (1.0 - zone);

	}


	return base;


}

//Encoder rightEncoder;
//Encoder leftEncoder;

float GetAnalog0()
{

	return pot[0]->GetAverageVoltage();
}



void encoderInit(void)
{
	return;

	printf("encoderInit\n");

	for (int L = 1; L <= 3; L++)
	{
		pot[L] = new AnalogInput(L);

		//if (L!=aArmElevation)

			 // resetEncoder(L);
	}



	//pot[8]= new AnalogChannel(1,8);

	//if (!encoderTask.Start()) printf("Encoder Failed to Start");

	//encoderTask.SetPriority(50);

	//rightEncoder=encoder(10,11, true);
	//leftEncoder =encoder(12,13, true);


}



void shooterInitService(uint32_t interruptAssertedMask, void *param)
{

	//static int   notNext=0;

	double curTime = Timer::GetFPGATimestamp();

	float deltaTime = curTime - lastShooterTime;

	lastShooterTime = curTime;


	/*

	if (notNext) {notNext=0; return;}

	if (deltaTime!=0)
	{
	  float newShooterRPM=1.0/deltaTime*60.0; //calculate RevPerSecond

	  if ( (newShooterRPM>100 ) && (newShooterRPM<2200))  shooterRPM=(shooterRPM*9+newShooterRPM)/10.0; else notNext=1;



	}
	*/

	//static float lastShooterRPM=0;


	if (deltaTime != 0)
	{
		float newShooterRPM = 1.0 / deltaTime*60.0; //calculate RevPerSecond

		if (newShooterRPM < 0 || newShooterRPM>6000) return;

		//if (fabs(newShooterRPM-lastShooterRPM)<100 ) shooterRPM=(shooterRPM*4+newShooterRPM)/5.0;

		//lastShooterRPM=(lastShooterRPM*4+newShooterRPM)/5.0;

		shooterRPM = newShooterRPM;

	}

	// loc[8]=shooterRPM;





}
