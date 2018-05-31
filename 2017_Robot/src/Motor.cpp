
#include "motor.h"
#include "iencoder.h"
#include "itimer.h"
#include "motion.h"
#include "base.h"
#include "cal.h"
#include "WPILib.h"
#include "relay.h"

int forceSpeedZero = 0;
int forceSpeedZeroBlock = 0;

float pwm[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float dac[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

SpeedController *jMotor[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
SpeedController *jMotorBack[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

Relay *spike[8] = { 0,0,0,0,0,0,0,0 };

int motorVal[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

Servo *sServo[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
Servo *sServoBack[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

void sendBack(int index, float value);

extern int steeringOff;

void setRightDriveTrain(int val, int shift)
{
	return;

	//	pwm[pwmRightMotor1 ]=val;
	//	pwm[pwmRightMotor2 ]=val;

		//if (shift==-1) return;

		//              pwm[pwmRightShifter]=shiftFast;
		//if (shift==1) pwm[pwmRightShifter]=shiftSlow;
		//if (shift==2) pwm[pwmRightShifter]=shiftNetural;
}

void setLeftDriveTrain(int val, int shift)
{
	return;

	//pwm[pwmLeftMotor1 ]=val;
	//pwm[pwmLeftMotor2 ]=val;

	//if (shift==-1) return;

	//              pwm[pwmLeftShifter]=shiftFast;
	//if (shift==1) pwm[pwmLeftShifter]=shiftSlow;
	//if (shift==2) pwm[pwmLeftShifter]=shiftNetural;

}



int setMotor(int L, float speed)
{
	//if (forceSpeedZero && L!=6 && L!=7 && L!=16 && L!=17) speed=0;


	static int inMotor = 0;

	if (inMotor)    return 0;

	if (!jMotor[L]) return 0;

	if (jMotor[L] != jMotorBack[L])

		return 0;

	inMotor = 1;

	if (speed < -1) speed = -1;
	if (speed > 1) speed = 1;

	if (jMotor[L]) {

		//if (L==pwmRoller1 || L==pwmRoller2) {tprintf(" %d %f \n",L,speed);}

		jMotor[L]->Set(speed); sendBack(20 + L, speed);
	}

	motorVal[L] = speed;

	// tprintf("motion %d value %f\n",L,speed);

	inMotor = 0;

	return 1;

}

int setServo(int L, float speed)
{
	static int inMotor = 0;

	if (inMotor)    return 0;

	if (!sServo[L]) return 0;

	if (sServo[L] != sServoBack[L])

		return 0;

	inMotor = 1;

	if (speed < -1) speed = -1;
	if (speed > 1) speed = 1;

	sServo[L]->Set(speed); sendBack(20 + L, speed);

	motorVal[L] = speed;

	inMotor = 0;

	return 1;

}


float currentToroUpDownSpeed = 0;

void SetToroUpSpeed(float rate)
{
	setMotor(pwmToroUpDown, -rate);

	currentToroUpDownSpeed = -rate;

}

void SetToroDownSpeed(float rate)
{
	setMotor(pwmToroUpDown, rate);

	currentToroUpDownSpeed = rate;
}


void SetBumperUpSpeed(float rate)
{
	setMotor(pwmBumperUpDown, -rate);
}

void SetBumperDownSpeed(float rate)
{
	setMotor(pwmBumperUpDown, rate);
}

void SetCatcherOpenSpeed(float rate)
{
	setMotor(pwmCatcher, rate);
}

void SetCatcherCloseSpeed(float rate)
{
	setMotor(pwmCatcher, -rate);
}


extern float toroIsDown;
extern float toroIsUp;
extern float toroIsCollecting;
extern float toroIsSpitting;
extern float toroInManual;

/*
void  ToroCollectCmd()
{
	tprintf("ToroCollectCmd\n");

	stopTimer(6);

	setMotor(pwmRoller1, rollerCollectRate + 0.05);
	setMotor(pwmRoller2, rollerCollectRate + 0.05);

	toroIsCollecting = 1;
	toroIsSpitting = 0;
	toroInManual = 0;

}
*/
/*
void  SetToroRate(float rate)
{
	setMotor(pwmRoller1, rate + 0.05);
	setMotor(pwmRoller2, rate + 0.05);

}
*/
/*
void  ToroSpitCmd()
{
	tprintf("ToroSpitCmd\n");

	stopTimer(6);

	setMotor(pwmRoller1, -rollerSpitRate + 0.05);
	setMotor(pwmRoller2, -rollerSpitRate + 0.05);

	toroIsCollecting = 0;
	toroIsSpitting = 1;
	toroInManual = 0;
}
*/


void  ToroOffCmd()
{
	tprintf("ToroOffCm\n");

	stopTimer(6);

	setMotor(pwmRoller1, 0 + 0.05);
	setMotor(pwmRoller2, 0 + 0.05);

	toroIsCollecting = 0;
	toroIsSpitting = 0;
	toroInManual = 0;
}



void stopTheRollers(unsigned char i)
{
	ToroOffCmd();
}

void stopCollectIn(float  stopTime)
{
	if (stopTime <= 0) stopTime = 3.0;

	startTimer(6, stopTime * 100, 0, stopTheRollers);
}

float holdBase = 0;
float holdLast = -1;


float baseAngle = 0;




void turn(float ang)
{
	tprintf("turn to %6.2f\n", ang);

	if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

	holdStraight = 1;
	holdLast = 1;
	//targetOffsetX = ang;
	holdBase = GyroLoc;
	gyroShiftState = 0;

	baseAngle = holdBase;

}

void nextTurn(float ang)
{
	tprintf("turnNext to %6.2f\n", ang);

	if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

	holdStraight = 1;
	holdLast = 1;
	//targetOffsetX = ang;
	holdBase = baseAngle;
	gyroShiftState = 0;

}

extern float gErrorLimit;

void updatePWM(int L)
{
	return;

	//if (holdStraight != holdLast) { targetOffsetX = 0; holdBase = GyroLoc - targetOffsetX; gyroShiftState = 0; }

	holdLast = holdStraight;


	float speed = (pwm[L] - 128.0) / 128.0;

	if (speed > 0)
	{

		if (L == pwmRightMotor1 || L == pwmRightMotor2) if (rightDriveAdj > 0.1) speed *= rightDriveAdj;
		if (L == pwmLeftMotor1 || L == pwmLeftMotor2) if (leftDriveAdj > 0.1) speed *= leftDriveAdj;
	}
	else
	{
		if (L == pwmRightMotor1 || L == pwmRightMotor2) if (rightDriveAdjN > 0.1) speed *= rightDriveAdjN;
		if (L == pwmLeftMotor1 || L == pwmLeftMotor2) if (leftDriveAdjN > 0.1) speed *= leftDriveAdjN;
	}

	if (holdStraight)
	{
		//float gError = (GyroLoc - holdBase + targetOffsetX); if (gyroShiftState == 1) gError -= gyroShift;

		//float gErrorLevel = gError*holdGain / 100.0;

		float gErrorLimitTmp = gErrorLimit;

		// tprintf("gError         %6.3f\n",gError );
		// tprintf("gErrorLevel    %6.3f\n",gErrorLevel);
		// tprintf("gErrorLevelTmp %6.3f\n",gErrorLimitTmp);


		// if (fabs(speed)>gErrorLimit) gErrorLimitTmp=1;

		if (!spinFlag)

			//if (gErrorLimit)
			//if (fabs(gErrorLevel) > gErrorLimitTmp) { if (gErrorLevel > 0) gErrorLevel = gErrorLimitTmp; else gErrorLevel = -gErrorLimitTmp; }

		if (spinFlag) speed = 0;


		//if (L == pwmRightMotor1 || L == pwmRightMotor2) speed += gErrorLevel;
		//if (L == pwmLeftMotor1 || L == pwmLeftMotor2) speed -= gErrorLevel;
	}

	if (speed < -1.0) speed = -1.0;
	if (speed > 1.0) speed = 1.0;


	if (forceCount[L] > 0)
	{
		speed = forceValue[L];

		forceCount[L]--;
	}

	if (forceSpeedZero > 0) { setMotor(L, 0); forceSpeedZero--; }
	else

		setMotor(L, speed);

}

extern int shiftDelay;

int adjustIt(int value, int adjust)
{
	adjust /= 2;

	value -= 128;

	if (adjust > 50)
	{

		float scale = (adjust - 50) * 2 / 100.0;

		return 128 + value*scale;

	}

	float scale = (50 - adjust) * 2 / 100.0;

	return 128 + value*scale;


}


void updatePWM(void)
{

	return;

	if (shiftDelay > 0)
	{
		shiftDelay--;

		//	pwm[pwmRightMotor1]=adjustIt(pwm[pwmRightMotor1],shiftDelay);
		//	pwm[pwmRightMotor2]=adjustIt(pwm[pwmRightMotor2],shiftDelay);
		//	pwm[pwmLeftMotor1 ]=adjustIt(pwm[pwmLeftMotor1],shiftDelay);
		//	pwm[pwmLeftMotor2 ]=adjustIt(pwm[pwmLeftMotor2],shiftDelay);
	}

	//updatePWM(pwmRightMotor1);
	//updatePWM(pwmRightMotor2);
	//updatePWM(pwmLeftMotor1 );
	//updatePWM(pwmLeftMotor2 );


} //updatePWM

//static int slot(int val) { return val/10 +1; }
//static int port(int val) { return val % 10;  }

int talonFlag = 1;

void setSpike(int index, int value)
{

	//	if (value==0) spike[index]->Set(Relay::kOff);
	//	else          spike[index]->Set(Relay::kForward);

}

void motorInit()
{

	printf("motorInit\n");
	/*
		 if (pwmRightMotor1 >0)  jMotor[pwmRightMotor1  ]= new Talon(port(pwmRightMotor1  ) ); //pwmRightMotor1
		 if (pwmRightMotor2 >0)  jMotor[pwmRightMotor2  ]= new Talon(port(pwmRightMotor2  ) ); //pwmRightMotor2
		 if (pwmLeftMotor1  >0)  jMotor[pwmLeftMotor1   ]= new Talon(port(pwmLeftMotor1   ) ); //pwmLeftMotor1
		 if (pwmLeftMotor2  >0)  jMotor[pwmLeftMotor2   ]= new Talon(port(pwmLeftMotor2   ) ); //pwmLeftMotor2

		 if (pwmShooterR1   >0)  jMotor[pwmShooterR1    ]= new Talon(port(pwmShooterR1    ) ); //pwmShooterR1
		 if (pwmShooterL1   >0)  jMotor[pwmShooterL1    ]= new Talon(port(pwmShooterL1    ) ); //pwmShooterL1
		 if (pwmShooterR2   >0)  jMotor[pwmShooterR2    ]= new Talon(port(pwmShooterR2    ) ); //pwmShooterR2
		 if (pwmShooterL2   >0)  jMotor[pwmShooterL2    ]= new Talon(port(pwmShooterL2    ) ); //pwmShooterL2

		 if (pwmRoller1     >0)  jMotor[pwmRoller1      ]= new Talon(port(pwmRoller1      ) ); //pwmRoller1
		 if (pwmRoller2     >0)  jMotor[pwmRoller2      ]= new Talon(port(pwmRoller2      ) ); //pwmRoller2

		 if (pwmRightShifter>0)  sServo[pwmRightShifter ]= new Servo (port(pwmRightShifter ) ); //pwmRightShifter
		 if (pwmLeftShifter >0)  sServo[pwmLeftShifter  ]= new Servo (port(pwmLeftShifter  ) ); //pwmLeftShifter

		 if (pwmToroUpDown  >0)  jMotor[pwmToroUpDown   ]= new Talon(port(pwmToroUpDown   ) );
		 if (pwmCatcher     >0)  jMotor[pwmCatcher      ]= new Talon(port(pwmCatcher      ) );

		 if (pwmBumperUpDown>0)  jMotor[pwmBumperUpDown ]= new Talon(port(pwmBumperUpDown ) );
	*/
	/*
	spike[0]=new Relay(0);
	spike[1]=new Relay(1);
	spike[2]=new Relay(2);
	spike[3]=new Relay(3);
	spike[4]=new Relay(4);
	spike[5]=new Relay(5);
	spike[6]=new Relay(6);
	spike[7]=new Relay(7);
*/


	for (int l = 0; l < 20; l++)
	{
		jMotorBack[l] = jMotor[l];
		sServoBack[l] = sServo[l];
		motorVal[l] = -10;
	}

	// setPwm(pwmRoller1,0.05);
	// setPwm(pwmRoller2,0.05);



} //motorInit



