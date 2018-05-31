#include "WPILib.h"
#include "aton.h"
#include "base.h"
#include "cal.h"
#include "motion.h"
#include "iencoder.h"
#include "motion.h"
#include "itimer.h"
#include "driving.h"
#include "joystick.h"
#include "motor.h"
#include "math.h"
#include "base.h"

extern int inAtonRotate;
void Set_CubeIntake(float right, float left);

extern int joyOverride;
extern float joyXOverride;
extern float joyYOverride;
extern float joyZOverride;
extern int ForceCubeIntake;

int atonActive = 0;
int atonActivee = 0;

extern float atonNum;
int stationNum = 0;
int blockAton = 0;
int atonMode = 0;

int atonSpin = 0;

void aton1(void);
void aton2(void);
void aton3(void);
void aton4(void);
void aton5(void);
void aton6(void);
void aton7(void);
void aton8(void);
void aton9(void);

extern float gyroZeroField;
bool isCubeUltrasonicValid();


//void aton1HG(void);
//void aton2HG(void);
//void aton3HG(void);
//void aton4HG(void);
//void aton5HG(void);
//void aton6HG(void);

void WaitFor(float time, void (*serviceAddress)(uc),
		void (*finishAddress)(uc)) {
	startTimer(3, time * 100.0, serviceAddress, finishAddress);
}

void WaitFor(float time, void (*finishAddress)(uc)) {
	startTimer(3, time * 100.0, 0, finishAddress);
}

void WaitWith(float time, void (*finishAddress)(uc)) {
	startTimer(4, time * 100.0, 0, finishAddress);
}

double beginAtonTime = 0;

static int once = 0;
void stopaton(unsigned char i);
void stopAton(uc i) {
	printf("Stop Aton\n");

	clearAllTimers();
	stopaton('u');

	drivingOff = 0;

	if (!inAton)
		once = 0;
}

void atonQuickStop(uc i);
void driveForwardOnly();

void StartWatchingForDriver(unsigned char);

void atonService() {

	if (!inAton && once > 0)
		once--;

	if (!inAton)
		blockAton = 0;

	if (inDisable || blockAton)
		return;
	if (once)
		return;

	if (inAton) {
		printf("Starting Service\n");
		printf("Start Aton %d\n", (int) atonNum);

		lastClock = 0; //reset loging clock

		once = 1500;

		startTimer(14, 1500, 0, StartWatchingForDriver);

		matchTimeBase = Timer::GetFPGATimestamp();
		matchTime = 0;
		matchStatus = 1;

		beginAtonTime = Timer::GetFPGATimestamp();

		switch ((int) atonNum) {
		case (uc) 1:
			aton1();
			break;
		case (uc) 2:
			aton2();
			break;
		case (uc) 3:
			aton3();
			break;
		case (uc) 4:
			aton4();
			break;
		case (uc) 5:
			aton5();
			break;
		case (uc) 6:
			aton6();
			break;
		case (uc) 7:
			aton7();
			break;
		case (uc) 8:
			aton8();
			break;
		case (uc) 9:
			aton9();
			break;
		default:
			driveForwardOnly();
			break;
		}
	}

}

void startAton(int atonNum) {

	if (atonActive) {
		tprintf("Aton Active (request blocked)\n");
		return;
	}

	lastClock = 0; //reset loging clock

	once = 2000;

	startTimer(14, 1500, 0, StartWatchingForDriver);

	matchTimeBase = (double) Timer::GetFPGATimestamp();
	matchTime = 0;
	matchStatus = 1;

	beginAtonTime = (double) Timer::GetFPGATimestamp();

	switch (atonNum) {
	case (uc) 1:
		aton1();
		break;
	case (uc) 2:
		aton2();
		break;
	case (uc) 3:
		aton3();
		break;
	case (uc) 4:
		aton4();
		break;
	case (uc) 5:
		aton5();
		break;
	case (uc) 6:
		aton6();
		break;

	default:
		;
	}
}

std::string gameData;
void getGameData() {
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

}

/*          --2017 Auton --                        */


void stopaton(unsigned char i) {
	printf("stop_aton\n");
	joyXOverride = 0;
	joyYOverride = 0;
	joyZOverride = 0;
	joyOverride = 0;

}

extern float wheelZero;
void aton_move(float speed, float strafespeed, float time, float x = 0) {
	printf("aton_moveforward\n");
	joyOverride = 1;
	joyXOverride = strafespeed;
	joyYOverride = -speed;
	joyZOverride = 0;

	startTimer(9, time, 0, stopaton);

}

float Get_DistanceEncoderInches();
float Get_DistanceEncoder();

float Get_DistanceEncoderVelocity();

extern float cam1DriveMode;
extern float selectCamera;
extern float gyroAngle;
extern float gyroZero;
int firstRun = 1;
int movePosGoodTimes = 0;
int firstRunAngle = 1;
int moveAngleGoodTimes = 0;

int badEncoder = 0;
float lastEncoder = 0;

float atonMoveTimeDone = 0;

int isFirstAtonMoveTime = 1;

void setAtonMoveTime(unsigned char i){
	atonMoveTimeDone = 1;
}


float aton_move_time(float x, float y, float turn, float seconds){
	inAtonRotate=0;
	if(isFirstAtonMoveTime){
		startTimer(11, seconds*100, 0, setAtonMoveTime);
		isFirstAtonMoveTime = 0;
	}
	joyXOverride = x;
	joyYOverride = y;
	joyZOverride = turn;
	joyOverride = 1;

	if(atonMoveTimeDone==1){

		joyXOverride = 0;
		joyYOverride = 0;
		joyZOverride = 0;
		joyOverride = 0;
		atonMoveTimeDone = 0;
		isFirstAtonMoveTime=1;
		return 1;
	}
	else{
		return 0;
	}

}
float startMoveTime = 0;
float lastMoveTime = 0;
float getTime(){
	return Timer::GetFPGATimestamp();
}
float Get_UltrasonicInches();
float Get_Ultrasonic2Inches();
void enableStrafeUltrasonic();
void disableCubeUltrasonic();
void disableStrafeUltrasonic();
void enableCubeUltrasonic();


float aton_strafe_pos(float pos) {
	//enableStrafeUltrasonic();
	//disableCubeUltrasonic();
	inAtonRotate = 0;

	float yError = (Get_UltrasonicInches() - pos);
	printf("yError: %f", yError);
	float curSpeed = yError * (.1);
	float maxSpeed=0.5;

	if (fabs(curSpeed) > maxSpeed) {
		curSpeed = maxSpeed * (curSpeed / (fabs(curSpeed)));
		}
	if (fabs(curSpeed) > 1) {
		curSpeed = 1 * (curSpeed / (fabs(curSpeed)));
	}
	if (fabs(curSpeed) < .20){
		curSpeed = .20 * (curSpeed / (fabs(curSpeed)));
	}

	//ToolBarDest = yError;
	//printf("curSpeed%f yerror %f\n", curSpeed, yError);
	joyXOverride = curSpeed;
	joyYOverride = 0;
	joyZOverride = 0;
	joyOverride = 1;


	if (fabs(yError) < 3) {
		joyOverride=0;
		return 1;
	}
	return 0;
}
int ultrasonicTimeout=0;
float aton_move_cube(float pos) {
	disableStrafeUltrasonic();
	enableCubeUltrasonic();
	inAtonRotate = 0;

	float yError = (Get_Ultrasonic2Inches() - pos);
	printf("yError: %f", yError);
	float curSpeed = yError * (-.1);
	float maxSpeed=0.3;
	if(isCubeUltrasonicValid() == false){
			curSpeed=-maxSpeed;
			yError=5;
			ultrasonicTimeout++;
	}
	if (fabs(curSpeed) > maxSpeed) {
		curSpeed = maxSpeed * (curSpeed / (fabs(curSpeed)));
		}
	if (fabs(curSpeed) > 1) {
		curSpeed = 1 * (curSpeed / (fabs(curSpeed)));
	}
	if (fabs(curSpeed) < .20){
		curSpeed = .20 * (curSpeed / (fabs(curSpeed)));
	}

	//ToolBarDest = yError;
	//printf("curSpeed%f yerror %f\n", curSpeed, yError);
	joyXOverride = 0;
	joyYOverride = curSpeed;
	joyZOverride = 0;
	joyOverride = 1;


	if (fabs(yError) < 3 || ultrasonicTimeout>10) {
		joyOverride=0;
		return 1;
	}
	return 0;
}

float aton_move_pos(float pos, float strafe=0, float check=1, float rampTime=0.2) {
	//printf("aton_move_pos\n");

	if (firstRun) {
			startMoveTime = getTime();
			movePosGoodTimes=0;
			//gyroZero = gyroAngle;
			firstRun = 0;
			wheelZero = Get_DistanceEncoder();
			badEncoder=0;
			lastEncoder=0;
		}
	inAtonRotate = 0;
	if(Get_DistanceEncoder()==lastEncoder){
		badEncoder++;
		//printf("badEncoder %d", badEncoder);
	}
	lastEncoder = Get_DistanceEncoder();
	float maxSpeed=(1/rampTime)*(getTime()-startMoveTime);
	//printf("MaxSpeed %f\n", maxSpeed);

	if(badEncoder>20){
		joyXOverride = 0;
		joyYOverride = 0;
		joyZOverride = 0;
		joyOverride = 0;
		if(!aton_move_time(0, -0.35, 0, pos*0.0105)){
			return 0;
		}
		else{
			joyOverride = 0;
			firstRun = 1;
			return 1;
		}

	}

	float yError = (-(Get_DistanceEncoder() - wheelZero) - (pos * -EncoderGain));
	float curSpeed = yError * -(.350/10000);

	if (fabs(curSpeed) > maxSpeed) {
			curSpeed = maxSpeed * (curSpeed / (fabs(curSpeed)));
		}
	if (fabs(curSpeed) > 1) {
		curSpeed = 1 * (curSpeed / (fabs(curSpeed)));
	}
	if (fabs(curSpeed) < .30){
		curSpeed = .30 * (curSpeed / (fabs(curSpeed)));
	}

	ToolBarDest = yError;
	//printf("curSpeed%f yerror %f\n", curSpeed, yError);
	joyXOverride = strafe;
	joyYOverride = -curSpeed;
	joyZOverride = 0;
	joyOverride = 1;


	if (fabs(yError) < 1000) {
		if(check){
			movePosGoodTimes+=1;
			//joyOverride=0;
		}
		else{
			joyOverride = 0;
			firstRun = 1;
			return 1;
		}
	}
	if(movePosGoodTimes>=3){
		joyOverride = 0;
		firstRun = 1;
		return 1;
	} else {
		return 0;
	}

}
float curEncoder;
float aton_move_pos2(float pos) {
	if (firstRun) {
		wheelZero = Get_DistanceEncoder();
		firstRun = 0;
		badEncoder=0;
		lastEncoder=0;
	}
	curEncoder=Get_DistanceEncoder();
	if(curEncoder==lastEncoder){
			badEncoder++;
			//printf("badEncoder %d", badEncoder);
	}
	lastEncoder = curEncoder;
	inAtonRotate = 0;
	float yError = (-(Get_DistanceEncoder() - wheelZero) - (pos * -EncoderGain));

	if(badEncoder>20){
		joyXOverride = 0;
		joyYOverride = 0;
		joyZOverride = 0;
		joyOverride = 0;
		if(!aton_move_time(0, -0.35, 0, (pos*0.0105))){
			return 0;
		}
		else{
			joyOverride = 0;
			firstRun = 1;
			return 1;
		}
	}



	float curSpeed = yError * -(.350/1000);

	if (fabs(curSpeed) > 10) {
		curSpeed = 10 * (curSpeed / (fabs(curSpeed)));
	}
	if (fabs(curSpeed) < 1){
		curSpeed = 1 * (curSpeed / (fabs(curSpeed)));
	}

	//ToolBarDest = yError;
	//printf("curSpeed%f yerror %f\n", curSpeed, yError);
	joyXOverride = 0;
	joyYOverride = -curSpeed;
	joyZOverride = 0;
	joyOverride = 2;


	if (fabs(yError) < 500) {
		joyOverride = 0;
		firstRun = 1;
		return 1;
	}

	return 0;
}

PIDController * rotatePID;

float aton_move_angle(float angle, float check=1) {
	//printf("aton_move_angle\n");
	inAtonRotate = 1;
	float AngleError = (gyroAngle - gyroZero) - angle;
	float curSpeed = AngleError * 0.015;
	if (firstRunAngle) {
				moveAngleGoodTimes=0;
				firstRunAngle = 0;
			}

	if (fabs(curSpeed) < 0.3) {
		curSpeed = 0.3 * (curSpeed / (fabs(curSpeed)));
	}

	if (fabs(curSpeed) > 1.00) {
		curSpeed = 1.00 * (curSpeed / (fabs(curSpeed)));
	}
	ToolBarDest = AngleError;
	//printf("curSpeed%f yerror %f\n", curSpeed, AngleError);
	joyOverride = 1;
	joyXOverride = 0;
	joyYOverride = 0;
	joyZOverride = -curSpeed;


	if (fabs(AngleError) < 2) {
		if(check){
			//joyOverride = 0;
			moveAngleGoodTimes+=1;
		}
		else{

			joyOverride = 0;
			firstRunAngle = 1;
			gyroZero = gyroZero + angle;
			return 1;
		}
		}
	if(moveAngleGoodTimes>=2){

			joyOverride = 0;
			firstRunAngle = 1;
			gyroZero = gyroZero + angle;
			return 1;
		} else {
			return 0;
		}

}
float aton_move_angle_abs(float angle, float check){
	angle = (gyroZero-gyroZeroField) + angle;
	printf("angle: %f\n", angle);
	return (aton_move_angle(angle, check));
}

void StopCube(unsigned char i) {
	ForceCubeIntake = 0;
	Set_CubeIntake(0, 0);
}

int move_toApproachSwitch;
int move_toCenterofSwitch; //aton1
int move_toTouchSwitch;

int move_toSideCenterofSwitch;
int move_toBehindSwitch;        //aton2 and 3
int move_toSwitchfromBehind;

int move_toFaceScale; //435
int move_toScalefromBehind;

int move_CrossCubeDrop;
int move_CornerCubeDrop;


void atonStart() {
	enableStrafeUltrasonic();
	selectCamera = 1;
	cam1DriveMode = 1;

	wheelZero = Get_DistanceEncoder();
	gyroZero = gyroAngle;

	getGameData();
	startZero();

	Set_CubeIntake(-0.3,-0.3);

	move_toApproachSwitch = move_ApproachSwitch;
	move_toCenterofSwitch = move_CenterofSwitch; //aton1
	move_toTouchSwitch = move_TouchSwitch;

	move_toSideCenterofSwitch = move_SideCenterofSwitch;
	move_toBehindSwitch = move_BehindSwitch;        //aton2 and 3
	move_toSwitchfromBehind = move_SwitchfromBehind;

	move_toFaceScale = move_FaceScale; //435
	move_toScalefromBehind = move_ScalefromBehind;

}
void startZeroWait(unsigned char i){
	startToolbarZero();
}

int crossScaleRight = 0; //flag if on right side (reverse rotations)
int crossScaleSign = 1; //used to reverse rotation for opposide side
void crossScale(int right) {
	printf("crossoverScale\n");
	atonStart();
	crossScaleRight = right;

	if(crossScaleRight){ //Side Scale from Right Side
		crossScaleSign = -1;
	}
	else{               //Side Scale from Left Side
		crossScaleSign = 1;
	}

	startTimer(19, 0, 0, crossScale_2);
}

void crossScale_2(unsigned char i){
	if (!aton_move_pos(move_toBehindSwitch, 0, 1, 1)) {
		startTimer(19, 0, 0, crossScale_2);
	} else {
		startTimer(19, 0, 0, crossScale_3);
	}
}
void crossScale_3(unsigned char i) {
	printf("crossScale_3\n");
	if (!aton_move_angle(crossScaleSign * 85, 0)) {
		startTimer(19, 0, 0, crossScale_3);
	} else {
		startTimer(19, 0, 0, crossScale_4);
	}
}
void crossScale_4(unsigned char i) {
	printf("crossScale_4");
	if(int(Crossover)==1){
		if (!aton_move_pos(move_toScalefromBehind, 0, 1, 1)) {
			startTimer(19, 0, 0, crossScale_4);
		} else {
			startTimer(19, 0, 0, crossScale_5);
		}
	}
	else{
		stopDrive();
		/*if (!aton_move_pos(100, 0, 0, 0.1)) {
			startTimer(19, 0, 0, crossScale_4);
		}
		*/

	}
}


void crossScale_5(unsigned char i) {
	if (!aton_move_angle(crossScaleSign * -85, 0)) {
		startTimer(19, 0, 0, crossScale_5);
	}
	else{
		startLiftScaleHigh();
		startTimer(19, 50, 0, crossScale_4_2);
	}
}

void crossScale_4_2(unsigned char i) {

	if (!aton_move_pos(move_CrossDropCube, 0, 0)) {
		startTimer(19, 0, 0, crossScale_4_2);
	} else {
		dropCubeScaleHigh();
		startTimer(19, 150, 0, crossScale_4_3);
		startTimer(18, 150, 0, startZeroWait);
	}
}
void crossScale_4_3(unsigned char i){
	if (!aton_move_pos(-move_CrossDropCube, 0, 0)) {
		startTimer(19, 0, 0, crossScale_4_3);
	}
	else{
		startPickup();
	}
}

int sideScaleRight = 0;//flag if on right side (reverse rotations)
int sideScaleSign = 1; //used to reverse rotation for opposide side
void sideScale(int right) {
	printf("sideScale\n");
	atonStart();
	startLiftScaleMid();
	sideScaleRight = right;

	if(sideScaleRight){ //Side Scale from Right Side
		sideScaleSign = -1;
	}
	else{               //Side Scale from Left Side
		sideScaleSign = 1;
	}
	startTimer(19, 0, 0, sideScale_2);
}
void sideScale_2(unsigned char i) {
	printf("sideScale_2\n");
	if (!aton_move_pos(move_toFaceScale, 0, 0, 2)) {
		startTimer(19, 0, 0, sideScale_2);
	} else {
		startTimer(19, 0, 0, sideScale_3);
	}
}
void sideScale_3(unsigned char i) {
	printf("sideScale_3\n");
	if (!aton_move_angle(sideScaleSign * 85, 0)) {
		startTimer(19, 0, 0, sideScale_3);
	} else {
		startTimer(19, 0, 0, sideScale_4);
	}
}
void sideScale_4(unsigned char i) {
	printf("sideScale_4\n");
	dropCubeScale();
	startTimer(19, 200, 0, sideScale_5);
}
void sideScale_5(unsigned char i){
	printf("sideScale_5\n");
	if (!aton_move_pos(-30, 0, 0)) {
		startTimer(19, 0, 0, sideScale_5);
		}
	else{
		if(int(TwoCube)==1){
			startPickup();
			startTimer(19, 0, 0, sideScale_6);
		}
	}
}
void sideScale_6(unsigned char i){
	printf("sideScale_6\n");
	if (!aton_move_angle(sideScaleSign * 65, 0)) {
		startTimer(19, 0, 0, sideScale_6);
		}
	else{
		startTimer(19, 100, 0, sideScale_7);
	}
}
void sideScale_7(unsigned char i){
	if (!aton_move_pos(120, 0, 0)) {
		startTimer(19, 0, 0, sideScale_7);
	} else {
		pickupCube();
	}
}
int backScaleRight = 0;//flag if on right back (reverse rotations)
int backScaleSign = 1; //used to reverse rotation for oppoback back
void backScale(int right) {
	printf("backScale\n");
	atonStart();
	backScaleRight = right;
	if(backScaleRight){ //back Scale from Right back
		backScaleSign = -1;
	}
	else{               //back Scale from Left back
		backScaleSign = 1;
	}

	startTimer(19, 0, 0, backScale_2);
}
void backScale_2(unsigned char i) {
	printf("backScale_2\n");
	if (!aton_move_pos(-move_toBehindSwitch, 0, 0, 1)) {
		startTimer(19, 0, 0, backScale_2);
	} else {
		startTimer(19, 0, 0, backScale_3);
	}
}
void backScale_3(unsigned char i) {
	printf("backScale_3\n");
	startScaleBehind();
	if (!aton_strafe_pos(backScaleSign * strafe_behindHeadDropoff)) { //70
		startTimer(19, 0, 0, backScale_3);
	} else {
		startTimer(19, 0, 0, backScale_4);

	}
}
void backScale_4_2_3(unsigned char i){
	startToolbarZero();
	startTimer(19, 20, 0, backScale_4_2);
}
void backScale_4_2_2(unsigned char i){
	if (fabs(Get_ToolbarError())>1000) {
		startTimer(19, 0, 0, backScale_4_2_2);
	}
	else{
		startDropCubeSlow();
		startTimer(19, 100, 0, backScale_4_2_3);
	}
}
void backScale_4(unsigned char i) {
	printf("backScale_4\n");
	startScaleBehind();
	if (!aton_move_pos(-move_OverHeadDroppoff, 0, 0, 0)) {
		startTimer(19, 0, 0, backScale_4);
	}
	else{
		startTimer(19, 50, 0, backScale_4_2_2);
	}
}

void backScale_4_2(unsigned char i){
	if (fabs(Get_ToolbarError())>1000) {
		startTimer(19, 0, 0, backScale_4_2);
	}
	else{
		startLiftPickup();
		startToolbarPickup();
		startTimer(19, 0, 0, backScale_5);
	}
}
void backScale_5(unsigned char i){
	printf("backScale_5\n");
	if (!aton_move_pos(70, 0, 0)) {
		startLoadCube();
		startTimer(19, 0, 0, backScale_5);
	}
	else{
		//startTimer(19, 0, 0, backScale_6);
	}
}
/*
void backScale_6(unsigned char i){
	if (!aton_move_angle(backScaleSign * -15)) { //80
		startTimer(19, 0, 0, backScale_6);
	} else {
		startTimer(19, 0, 0, backScale_7);
	}
}
*/

void backScale_6(unsigned char i) {
	if (!aton_strafe_pos(backScaleSign * strafe_firstCubePickup)) { //80
			startTimer(19, 0, 0, backScale_6);
		} else {
			startTimer(19, 0, 0, backScale_7);
		}

}

void backScale_7(unsigned char i){
	printf("backScale_7\n");

	if (!aton_move_pos(20, 0, 0)) {
		startTimer(19, 0, 0, backScale_7);
	}
	else{
		stopCube5();
		startTimer(19, 0, 0, backScale_8);
	}

}
/*
void backScale_8(unsigned char i){
	if (!aton_move_angle(backScaleSign * 15)) { //80
		startTimer(19, 0, 0, backScale_8);
	} else {
		startScaleBehind();
		startTimer(19, 0, 0, backScale_3);
	}
}
*/
void backScale_8(unsigned char i){
	printf("backScale_8\n");
	if (!aton_strafe_pos(backScaleSign*strafe_behindHeadDropoff)) { //70
		startTimer(19, 0, 0, backScale_8);
	}
	else{
		stopCube5();
		startTimer(19, 0, 0, backScale_9);
	}


}

void backScale_9(unsigned char i){
	printf("backScale_9\n");

	if (!aton_move_pos(-20, 0, 0)) {
		startTimer(19, 0, 0, backScale_9);
	}
	else{
		startTimer(19, 0, 0, backScale_3);
	}

}
int pickupCubeStrafeRight = 0;//flag if on right back (reverse rotations)
int pickupCubeStrafeSign = 1; //used to reverse rotation for oppoback back
void pickupCubeStrafe(int right) {
	printf("pickupCubeStrafe\n");
	pickupCubeStrafeRight = right;
	if(pickupCubeStrafeRight){ //back Scale from Right back
		pickupCubeStrafeSign = -1;
	}
	else{               //back Scale from Left back
		pickupCubeStrafeSign = 1;
	}
	startTimer(19, 0, 0, pickupCubeStrafe_2);
}
void pickupCubeStrafe_2(unsigned char i) { //Initial Move, move behind switch,
	printf("pickupCubeStrafe_2\n");
	if (!aton_move_angle(135)) {
		startTimer(19, 0, 0, pickupCubeStrafe_2);
	} else {
		startTimer(19, 0, 0, pickupCubeStrafe_3);
	}
}


void pickupCubeStrafe_2_2(unsigned char i) { //Initial Move, move behind switch,
	printf("cornerScale_2\n");
	if (!aton_move_pos(40, 0, 0, 0)) {
		startTimer(19, 0, 0, pickupCubeStrafe_2_2);
	} else {
		startTimer(19, 0, 0, pickupCubeStrafe_3);

	}
}
void pickupCubeStrafe_3(unsigned char i){
	if (!aton_strafe_pos(strafe_firstCubePickup)) {
		startTimer(19, 0, 0, pickupCubeStrafe_3);
	} else {
		startTimer(19, 0, 0, pickupCubeStrafe_4);
		startLoadCube();
	}
}
void pickupCubeStrafe_4(unsigned char i){
	if (!aton_move_cube(10)) {
		startTimer(19, 0, 0, pickupCubeStrafe_4);
	} else {
		stopCube5();
		startTimer(19, 0, 0, pickupCubeStrafe_5);
	}
}
void pickupCubeStrafe_5(unsigned char i){
	if (!aton_move_pos(-10)) {
		startTimer(19, 0, 0, pickupCubeStrafe_5);
	} else {
		startTimer(19, 0, 0, backScale_3);
	}
}

int cornerScaleRight = 0; //flag if on right side (reverse rotations)
int cornerScaleSign = 1;  //used to reverse rotation for opposide side
void cornerScale(int right){
	printf("cornerScale\n");
	atonStart();
	startLiftScaleMid();
	cornerScaleRight = right;

	if(cornerScaleRight){ //Corner Scale from Right Side
		cornerScaleSign = -1;
	}
	else{                //Corner Scale from Left Side
		cornerScaleSign = 1;
	}
	startTimer(19, 0, 0, cornerScale_2);
}

void cornerScale_2(unsigned char i) { //Initial Move, move behind switch,
	printf("cornerScale_2\n");
	if (!aton_move_pos(move_CornerToScale, 0, 0, 2)) {
		startTimer(19, 0, 0, cornerScale_2);
	} else {
		startTimer(19, 0, 0, cornerScale_3);
		startToolbarScaleMid();
	}
}
void cornerScale_3(unsigned char i){ // Rotate to angle to hit corner of switch
	if (!aton_move_angle(cornerScaleSign * 35, 0)) {
		startTimer(19, 0, 0, cornerScale_3);
	} else {
		startTimer(19, 0, 0, cornerScale_4);
	}
}
void cornerScale_4(unsigned char i){ //Start toolbar down, and move forward. once there, drop the cube slowly

	/*
	if (!aton_move_pos(move_CornerDropCube, 0, 0, 0.1)) {
		startTimer(19, 0, 0, cornerScale_4);
	} else {
		startDropCubeSlow();
		startTimer(19, 50, 0, cornerScale_4_2);
	}
	*/
	startShootCube();
	if(int(TwoCube)==1){
		startTimer(19, 100, 0, cornerScale_5); //If two cube is enabled, start two cube auto stuff
	}
	else{
		startTimer(19, 0, 0, cornerScale_4_3);
	}

}
void cornerScale_4_2(unsigned char i){ //Reverse those movements to get back to behind switch
	startToolbarZero();
	if (!aton_move_pos(-move_CornerDropCube, 0, 0, 0.1)) {
		startTimer(19, 0, 0, cornerScale_4_2);
	} else {
		if(int(TwoCube)==1){
			startTimer(19, 0, 0, cornerScale_5); //If two cube is enabled, start two cube auto stuff
		}
		else{
			startTimer(19, 0, 0, cornerScale_4_3);
		}
	}

}
void cornerScale_4_3(unsigned char i){ //Reverse those movements to get back to behind switch
	if (!aton_move_pos(-25, 0, 0, 0.1)) {
		startTimer(19, 0, 0, cornerScale_4_3);
	}
	else{
		startLiftPickup();
	}
}
void cornerScale_5(unsigned char i){ //move toolbar and lift to pickup mode and rotate to pickup the next cube
	startLiftPickup();
	startToolbarPickup();
	//pickupCubeStrafe(0);

	if (!aton_move_angle(cornerScaleSign * TwoCubePickupAngle, 0)) { //80
		startTimer(19, 0, 0, cornerScale_5);
	} else {
		startLoadCube();
		startTimer(19, 0, 0, cornerScale_6);
	}


}
void cornerScale_6(unsigned char i){ //move forward to be touching the cube
	/*if (!aton_move_pos(move_TwoCubeDistance, 0, 0, 0.1)) {
		startTimer(19, 0, 0, cornerScale_6);
	} else {
		stopCube5();
		startTimer(19, 150, 0, cornerScale_7);
	}*/
	if(!aton_move_pos(move_TwoCubeDistance, 0)){
		startTimer(19, 0, 0, cornerScale_6);
	}
	else{
		startTimer(19, 50, 0, cornerScale_6_2);
	}

}
float getAverageIntakeAmps(){
	return (IntakeLeftAmps+IntakeRightAmps)/2.0;
}
void closeOnCube(unsigned char i){
	stopCube5();
}
void slowDrive2(unsigned char i){
	slowDrive();
}
void cornerScale_6_1(unsigned char i){ //Reverse those movements to get back to behind switch
	if (!aton_move_pos(-20, 0, 0, 0.1)) {
		startTimer(19, 0, 0, cornerScale_6_1);
	}
	else{
		if(getAverageIntakeAmps()>1){
			startTimer(19, 0, 0, backScale_4);
		}
		else{
			startLoadCube();
			startTimer(17, 50, 0, slowDrive2);
			startTimer(18, 150, 0, closeOnCube);
			startTimer(19, 200, 0, cornerScale_6_1);
		}

	}
}
void cornerScale_6_cube(unsigned char i);
void cornerScale_6_2(unsigned char i){ //a little rotation to ensure cube is in grasp, when done close gripper
	/*if (!aton_move_angle(cornerScaleSign * 15, 0)) {
		startTimer(19, 0, 0, cornerScale_6_2);
	} else {

		startTimer(19, 0, 0, cornerScale_6_3);
	}*/
	if (!aton_move_angle(cornerScaleSign * (130-TwoCubePickupAngle), 0)) { //50
		startTimer(19, 0, 0, cornerScale_6_2);
	} else {
		slowDrive();
		startTimer(18, 100, 0, closeOnCube);
		startTimer(19, 150, 0, cornerScale_6_1);
	}
}
void cornerScale_6_cube(unsigned char i){
	if (!aton_move_cube(10)) {
		startTimer(19, 0, 0, cornerScale_6_cube);
	} else {
		slowDrive();
			startTimer(18, 50, 0, closeOnCube);
			startTimer(19, 100, 0, cornerScale_6_1);
		}
}
void cornerScale_6_3(unsigned char i){ //rotate back to original angle and bring toolbar back up
	if (!aton_move_angle(cornerScaleSign * -15, 0)) {
		startTimer(19, 0, 0, cornerScale_6_3);
	} else {
		startTimer(19, 0, 0, cornerScale_7);
	}
}
void cornerScale_7(unsigned char i){ //move back to position to score
	startToolbarZero();
	if (!aton_move_pos(-move_TwoCubeDistance, 0, 0, 0.1)) {
		startTimer(19, 0, 0, cornerScale_7);
	} else {
		startTimer(19, 0, 0, cornerScale_8);
	}
}
void cornerScale_8(unsigned char i){ //rotate back to angle to score
	startLiftScaleMid();
	startToolbarScaleMid();
	if (!aton_move_angle(cornerScaleSign * -105, 0)) {
		startTimer(19, 0, 0, cornerScale_8);
	} else {
		startTimer(19, 100, 0, cornerScale_4); //repeat to step 4 (will continue in loop)
	}
}
void driveForwardOnly2(unsigned char i);
void driveForwardOnly(){
	startTimer(19, 100, 0, driveForwardOnly2);
}
void driveForwardOnly2(unsigned char i){
	if (!aton_move_pos(240)){
		startTimer(19, 0, 0, driveForwardOnly2);
	}
	else{

	}
}


void aton1() { //Auton 1 starts in center and goes left or right depending on switch side
	atonStart();
	startLiftSwitch();
	startTimer(19, 0, 0, aton1_2);
}
void aton1_2(unsigned char i) { //Initial Move
	if (!aton_move_pos(move_toApproachSwitch, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton1_2);
	} else {
		startTimer(19, 0, 0, aton1_3);
	}
}
void aton1_3(unsigned char i) { //Move 45 degrees in direction of the switch
	if (gameData[0] == 'R') {
		if (!aton_move_angle(45, 0)) {
			startTimer(19, 0, 0, aton1_3);
		} else {
			startTimer(19, 0, 0, aton1_4);
		}
	}
	if (gameData[0] == 'L') {
		if (!aton_move_angle(-45, 0)) {
			startTimer(19, 0, 0, aton1_3);
		} else {
			startTimer(19, 0, 0, aton1_4);
		}
	}

}
void aton1_4(unsigned char i) { //move forward in direction of switch
	if (!aton_move_pos(move_toCenterofSwitch, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton1_4);
	} else {
		startTimer(19, 0, 0, aton1_5);
	}
}
void aton1_5(unsigned char i) { //rotate to face the switch
	if (gameData[0] == 'R') {
		if (!aton_move_angle(-45, 0)) {
			startTimer(19, 0, 0, aton1_5);
		} else {
			startTimer(19, 0, 0, aton1_6);
		}
	}
	if (gameData[0] == 'L') {
		if (!aton_move_angle(45, 0)) {
			startTimer(19, 0, 0, aton1_5);
		} else {
			startTimer(19, 0, 0, aton1_6);
		}
	}

}
void aton1_6(unsigned char i) { //move to get against the switch and drop cube
		dropCube();
		startTimer(19,200,0,aton1_7);
}
void aton1_7(unsigned char i) { //move back to prepare reverse
	startZero();
	if (!aton_move_pos(-20, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton1_7);
	} else {
		startTimer(19, 0, 0, aton1_8);
	}
}
void aton1_8(unsigned char i) { //Move 45 degrees in direction of the switch
	if (gameData[0] == 'R') {
		if (!aton_move_angle(45, 0)) {
			startTimer(19, 0, 0, aton1_8);
		} else {
			startTimer(19, 0, 0, aton1_9);
		}
	}
	if (gameData[0] == 'L') {
		if (!aton_move_angle(-45, 0)) {
			startTimer(19, 0, 0, aton1_8);
		} else {
			startTimer(19, 0, 0, aton1_9);
		}
	}
}
void aton1_9(unsigned char i) { //move backwards in direction of switch
	if (!aton_move_pos(-move_toCenterofSwitch, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton1_9);
	} else {
		startTimer(19, 0, 0, aton1_10);
	}
}
void aton1_10(unsigned char i) { //Move 45 degrees in direction of the switch
	if (gameData[0] == 'R') {
		if (!aton_move_angle(-45, 0)) {
			startTimer(19, 0, 0, aton1_10);
		} else {
			startTimer(19, 0, 0, aton1_11);
		}
	}
	if (gameData[0] == 'L') {
		if (!aton_move_angle(45, 0)) {
			startTimer(19, 0, 0, aton1_10);
		} else {
			startTimer(19, 0, 0, aton1_11);
		}
	}
}
void aton1_11(unsigned char i){ //pickup the center cube on staack
	pickupCube();
}
void pickupCubeUltra_2(unsigned char i);
void pickupCubeUltra(unsigned char i){
	if(!aton_move_cube(10)){
		startTimer(19, 0, 0, pickupCubeUltra);
	}
	else{
		stopCube5();
		printf("PickupCubeUltra\n");
		startTimer(19, 100, 0, pickupCubeUltra_2);
	}
}
void pickupCubeUltra_2(unsigned char i){
	if(!aton_move_pos(-30)){
		startTimer(19, 0, 0, pickupCubeUltra_2);
	}
	else{
		printf("PickupCubeUltra_2\n");
	}
}
/*
void aton2(){
	startLoadCube();
	startTimer(19, 0, 0, pickupCubeUltra);
}
*/

/*
void aton2(){
	printf("aton2\n");
	atonStart();
	if (gameData[1] == 'L') {
		backScale(0);
	}
	else{
		crossScale(0);
	}
}
*/

/*
void aton3(){
	printf("aton3\n");
	atonStart();
	if (gameData[1] == 'R') {
		backScale(1);
	}
	else{
		crossScale(1);
	}
}
*/

void aton2() {
	printf("aton2\n");
	atonStart();
	startLiftSwitch();
	startTimer(19, 0, 0, aton2_2);
}


void aton3() {
	printf("aton3\n");
	atonStart();
	startLiftSwitch();
	startTimer(19, 0, 0, aton3_2);
}

void aton4() {
	printf("aton4\n");
	atonStart();
	if (gameData[1] == 'L') {
		sideScale(0);
	}
	else{
		crossScale(0);
	}
}

void aton5() {
	printf("aton5\n");
	atonStart();
	if (gameData[1] == 'R') {
		sideScale(1);
	}
	else{
		crossScale(1);
	}
}

void aton6() {
	printf("aton6\n");
	atonStart();

	if(int(favorScale)==0){
		if(gameData[0] == 'L'){
			aton2();
		}
		else
		if(gameData[1] == 'L'){
			aton8();
		}
		else
		if(gameData[0] == 'R'){
			aton2();
		}
	}
	else{
		if(gameData[1] == 'L'){
			aton8();
		}
		else
		if(gameData[0] == 'L'){
			aton2();
		}
		else
		if(gameData[0] == 'R'){
			aton2();
		}
	}
}

void aton7(){
	printf("aton7\n");
	atonStart();

	if(int(favorScale)==0){
		if(gameData[0] == 'R'){
			aton3();
		}
		else
		if(gameData[1] == 'R'){
			aton9();
		}
		else
		if(gameData[0] == 'L'){
			aton3();
		}
	}
	else{
		if(gameData[1] == 'R'){
			aton9();
		}
		else
		if(gameData[0] == 'R'){
			aton3();
		}
		else
		if(gameData[0] == 'L'){
			aton3();
		}
	}
}

void aton8(){
	printf("aton8\n");
	atonStart();
	if(gameData[1]=='L'){
		cornerScale(0);
	}
	else{
		crossScale(0);
	}
}

void aton9(){
	printf("aton9\n");
	atonStart();
	if(gameData[1]=='R'){
		cornerScale(1);
	}
	else{
		crossScale(1);
	}
}




void aton2_2(unsigned char i) {
	printf("aton2_2\n");
	if (gameData[0] == 'L') {
		if (!aton_move_pos(move_toSideCenterofSwitch)) {
			startTimer(19, 0, 0, aton2_2);
		} else {
			startTimer(19, 0, 0, aton2_3);
		}
	}
	if (gameData[0] == 'R') {
		if (!aton_move_pos(move_toBehindSwitch)) {
			startTimer(19, 0, 0, aton2_2);
		} else {
			startTimer(19, 0, 0, aton2_3);
		}
	}
}

void aton2_3(unsigned char i) {
	printf("aton2_3\n");
	if (!aton_move_angle(85)) {
		startTimer(19, 0, 0, aton2_3);
	} else {
		if (gameData[0] == 'L') {
			startTimer(19, 0, 0, aton2_4);
		} else {
			startTimer(19, 0, 0, aton2_4_2);
		}
	}
}
void aton2_4_2(unsigned char) {
	if (!aton_move_pos(move_toSwitchfromBehind)) {
		startTimer(19, 0, 0, aton2_4_2);
	} else {
		startTimer(19, 0, 0, aton2_5);
	}
}
void aton2_4(unsigned char i) {
	printf("aton2_4\n");
	dropCube();
}
void aton2_5(unsigned char i) {
	if (!aton_move_angle(95)) {
		startTimer(19, 0, 0, aton2_5);
	} else {
		startTimer(19, 0, 0, aton2_4);
	}
}
void aton3_2(unsigned char i) {
	printf("aton3_2\n");
	if (gameData[0] == 'R') {
		if (!aton_move_pos(move_toSideCenterofSwitch)) {
			startTimer(19, 0, 0, aton3_2);
		} else {
			startTimer(19, 0, 0, aton3_3);
		}
	}
	if (gameData[0] == 'L') {
		if (!aton_move_pos(move_toBehindSwitch)) {
			startTimer(19, 0, 0, aton3_2);
		} else {
			startTimer(19, 0, 0, aton3_3);
		}
	}
}

void aton3_3(unsigned char i) {
	printf("aton3_3\n");
	if (!aton_move_angle(-85)) {
		startTimer(19, 0, 0, aton3_3);
	} else {
		if (gameData[0] == 'R') {
			startTimer(19, 0, 0, aton3_4);
		} else {
			startTimer(19, 0, 0, aton3_4_2);
		}
	}
}
void aton3_4_2(unsigned char) {
	if (!aton_move_pos(move_toSwitchfromBehind)) {
		startTimer(19, 0, 0, aton3_4_2);
	} else {
		startTimer(19, 0, 0, aton3_5);
	}
}
void aton3_4(unsigned char i) {
	printf("aton3_4\n");

	dropCube();
}
void aton3_5(unsigned char i) {
	if (!aton_move_angle(-95)) {
		startTimer(19, 0, 0, aton3_5);
	} else {
		startTimer(19, 0, 0, aton3_4);
	}
}


void aton3_7(unsigned char i){

	/*
	if (!aton_move_time(1, 0.40, -0.20, 1.0)) {
			startTimer(19, 0, 0, aton3_7);
		} else {
			pickupCube();
			startTimer(19, 300, 0, aton3_8);
		}
		*/
	if (!aton_move_pos(-10)) {
		startTimer(19, 0, 0, aton3_7);
	} else {
		startTimer(19, 0, 0, aton3_8);
	}
}
void aton3_8(unsigned char i){
	if (!aton_move_time(1,0.05,0, 1)){
		startTimer(19, 0, 0, aton3_8);
		} else {
			startTimer(19, 0, 0, aton3_9);
		}
}
void aton3_9(unsigned char i){
	if (!aton_move_angle(-15)) {
				startTimer(19, 0, 0, aton3_9);
			} else {
				pickupCube();
				startTimer(19, 300, 0, aton3_10);
			}
}
void aton3_10(unsigned char i){
	dropCube();
}


void aton5_2(unsigned char i) {
	printf("aton5_2\n");
	if (gameData[1] == 'R') {
		if (!aton_move_pos(move_toFaceScale, 0, 0, 2)) {
			startTimer(19, 0, 0, aton5_2);
		} else {
			startTimer(19, 0, 0, aton5_3);
		}
	}
	if (gameData[1] == 'L') {
		if (!aton_move_pos(move_toBehindSwitch)) {
			startTimer(19, 0, 0, aton5_2);
		} else {
			startTimer(19, 0, 0, aton5_3);
		}
	}

}
void aton5_3(unsigned char i) {
	printf("aton5_3\n");
	if (!aton_move_angle(-85)) {
		startTimer(19, 0, 0, aton5_3);
	} else {
		if (gameData[1] == 'R') {
			startTimer(19, 0, 0, aton5_4);
		}
		if (gameData[1] == 'L') {
			startTimer(19, 0, 0, aton5_4_2);
		}
	}
}
void aton5_4(unsigned char i) {
	printf("aton5_4\n");
	dropCubeScale();
	startTimer(19, 200, 0, aton5_6_1);
}
void aton5_4_2(unsigned char i) {
	printf("aton5_4_2");
	if(int(Crossover)==1){
		if (!aton_move_pos(move_toScalefromBehind, 0, 1)) {
			startTimer(19, 0, 0, aton5_4_2);
		} else {
			startTimer(19, 0, 0, aton5_5);
		}
	}
}
void aton5_5(unsigned char i) {
	if (!aton_move_angle(85)) {
		startTimer(19, 0, 0, aton5_5);
	}
	else{
		startTimer(19, 0, 0, aton5_5_2);
	}
}
void aton5_5_2(unsigned char i){
	startLiftScaleMid();
	if (!aton_move_pos(20, 0, 0)) {
		startTimer(19, 0, 0, aton5_5_2);
		}
	else{
		dropCubeScale();
	}
}
void aton5_6_1(unsigned char i){
	printf("aton4_5_1\n");
	if (!aton_move_pos(-30, 0, 0)) {
		startTimer(19, 0, 0, aton5_6_1);
		}
	else{
		startPickup();
		if(int(TwoCube)==1){
			startTimer(19, 0, 0, aton5_6);
		}
	}
}
void aton5_6(unsigned char i){
	printf("aton5_6\n");
	if (!aton_move_angle(-65)) {
		startTimer(19, 0, 0, aton5_6);
		}
	else{
		startTimer(19, 0, 0, aton5_7);
	}
}
void aton5_7(unsigned char i){
	if (!aton_move_pos(120, 0, 0)) {
			startTimer(19, 0, 0, aton5_7);
		} else {
			pickupCube();
		}
}

void aton8_2(unsigned char i) {
	printf("aton8_2");
	if (!aton_move_pos(move_toBehindSwitch, 0, 0, 1)) {
		startTimer(19, 0, 0, aton8_2);
	} else {
		startTimer(19, 0, 0, aton8_3);
	}
}
void aton8_3(unsigned char i){
	if (!aton_move_angle(35, 0)) {
		startTimer(19, 0, 0, aton8_3);
	} else {
		startTimer(19, 0, 0, aton8_4);
	}
}
void aton8_4(unsigned char i){
	startScaleMid();
	if (!aton_move_pos(35, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton8_4);
	} else {
		startDropCubeSlow();
		startTimer(19, 100, 0, aton8_4_2);
	}

}
void aton8_4_2(unsigned char i){
	if (!aton_move_pos(-35, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton8_4_2);
	} else {
		if(int(TwoCube)==1){
			startTimer(19, 0, 0, aton8_5);
		}
	}

}
void aton8_5(unsigned char i){
	startLiftPickup();
	startToolbarPickup();
	if (!aton_move_angle(100, 0)) {
		startTimer(19, 0, 0, aton8_5);
	} else {
		startTimer(19, 0, 0, aton8_6);
		startLoadCube();
	}

}
void aton8_6(unsigned char i){
	if (!aton_move_pos(70, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton8_6);
	} else {
		stopCube4();
		startTimer(19, 50, 0, aton8_7);
	}
}
void aton8_7(unsigned char i){
	if (!aton_move_pos(-70, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton8_7);
	} else {
		startTimer(19, 0, 0, aton8_8);
	}
}
void aton8_8(unsigned char i){
	if (!aton_move_angle(-100, 0)) {
		startTimer(19, 0, 0, aton8_8);
	} else {
		startTimer(19, 0, 0, aton8_4);
	}
}
void aton9_2(unsigned char i) {
	printf("aton9_2");
	if (!aton_move_pos(move_toBehindSwitch, 0,1, 1)) {
		startTimer(19, 0, 0, aton9_2);
	} else {
		startTimer(19, 0, 0, aton9_3);
	}
}
void aton9_3(unsigned char i){
	if (!aton_move_angle(-45, 0)) {
		startTimer(19, 0, 0, aton9_3);
	} else {
		startTimer(19, 0, 0, aton9_4_3);
	}
}
void aton9_4_3(unsigned char i){
	startToolbarScaleMid();
	startTimer(18, 25, 0, startDropCube);
	startTimer(19, 100, 0, aton9_5);
}
void aton9_4(unsigned char i){
	startLiftScaleMid();
	if (!aton_move_pos(35, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton9_4);
	} else {
		startToolbarScaleMid();
		//startShootCube();
		startDropCubeSlow();
		startTimer(19, 100, 0, aton9_4_2);
	}

}
void aton9_4_2(unsigned char i){
	if (!aton_move_pos(-35, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton9_4_2);
	} else {
		if(int(TwoCube)==1){
			startTimer(19, 0, 0, aton9_5);
		}
	}

}
void aton9_5(unsigned char i){
	startPickup();
	if (!aton_move_angle(-105, 0)) {
		startTimer(19, 0, 0, aton9_5);
	} else {
		startTimer(19, 0, 0, aton9_6);
		startLoadCube();
	}

}
void aton9_6(unsigned char i){
	if (!aton_move_pos(70, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton9_6);
	} else {
		stopCube4();
		startTimer(19, 100, 0, aton9_6_2);
	}
}
void aton9_6_2(unsigned char i){
	if (!aton_move_angle(-10, 0)) {
		startTimer(19, 0, 0, aton9_6_2);
	} else {
		startTimer(19, 0, 0, aton9_7);
	}
}
void aton9_7(unsigned char i){
	if (!aton_move_pos(-70, 0, 0, 0.1)) {
		startTimer(19, 0, 0, aton9_7);
	} else {
		startTimer(19, 0, 0, aton9_8);
	}
}
void aton9_8(unsigned char i){
	if (!aton_move_angle(110, 0)) {
		startTimer(19, 0, 0, aton9_8);
	} else {
		startTimer(19, 0, 0, aton9_4_3);
	}
}
void aton4_2(unsigned char i) {
	printf("aton4_2\n");
	if (gameData[1] == 'L') {
		if (!aton_move_pos(move_toFaceScale, 0, 0, 2)) {
			startTimer(19, 0, 0, aton4_2);
		} else {
			startTimer(19, 0, 0, aton4_3);
		}
	}
	if (gameData[1] == 'R') {
		if (!aton_move_pos(move_toBehindSwitch)) {
			startTimer(19, 0, 0, aton4_2);
		} else {
			startTimer(19, 0, 0, aton4_3);
		}
	}

}
void aton4_3(unsigned char i) {
	printf("aton4_3\n");
	if (!aton_move_angle(85)) {
		startTimer(19, 0, 0, aton4_3);
	} else {
		if (gameData[1] == 'L') {
			startTimer(19, 0, 0, aton4_4);
		}
		if (gameData[1] == 'R') {
			startTimer(19, 0, 0, aton4_4_2);
		}
	}
}
void aton4_4(unsigned char i) {
	printf("aton4_4\n");
	dropCubeScale();
	startTimer(19, 200, 0, aton4_6_1);
}
void aton4_4_2(unsigned char i) {
	printf("aton4_4_2");
	if(int(Crossover)==1){
		if (!aton_move_pos(move_toScalefromBehind)) {
			startTimer(19, 0, 0, aton4_4_2);
		} else {
			startTimer(19, 0, 0, aton4_5);
		}
	}
}
void aton4_5(unsigned char i) {
	if (!aton_move_angle(-85)) {
		startTimer(19, 0, 0, aton4_5);
	}
	else{
		dropCubeScaleFar();
	}
}
void aton4_6_1(unsigned char i){
	printf("aton4_6_1\n");
	if (!aton_move_pos(-30, 0, 0)) {
		startTimer(19, 0, 0, aton4_6_1);
		}
	else{
		startPickup();
		if(int(TwoCube)==1){
			startTimer(19, 0, 0, aton4_6);
		}
	}
}
void aton4_6(unsigned char i){
	printf("aton4_6\n");
	if (!aton_move_angle(65)) {
		startTimer(19, 0, 0, aton4_6);
		}
	else{
		startTimer(19, 0, 0, aton4_7);
	}
}
void aton4_7(unsigned char i){
	if (!aton_move_pos(120, 0, 0)) {
			startTimer(19, 0, 0, aton4_7);
		} else {
			pickupCube();
		}
}

void WatchForDriver(unsigned char i) {
	if (joyY[2] < -0.2 || joyY[2] > 0.2 || joyY[3] < -0.2 || joyY[3] > 0.2) {
		stopAton(0);
	}
}

void StartWatchingForDriver(unsigned char i) {
	tprintf("AtonFinished\n");
	tprintf("\n");

	//rollerDeadZone    =3; // only collect balls at the finish of aton

	DriveModeFlag = 1;

	startTimer(14, 1500, WatchForDriver, 0);
}

void atonQuickStop(uc i) {
	clearAllTimers();

	drivingOff = 0;
}

void ShotWatch(uc i);
void ShotWatchEnd(uc i);

void ShootBall(float ShotAngBeg, float ShotAngEnd, float ShotSpeed);

void (*ShotFinished)() = 0;

void SendTorrowsDown(void (*serviceAddress)());

float barWatchStartLoc = 0;
float barWatchDistance = 0;

int appLock = 0;

float ShotAngBegTmp = 0;
float ShotAngEndTmp = 0;
float ShotSpeedTmp = 0;

void ShootBallB();

extern int toroDownWhileShooting;

void ShootBall(float ShotAngBeg, float ShotAngEnd, float ShotSpeed,
		void (*serviceAddress)()) {

	tprintf("--------------------------\n");
	pprintf("------- Shoot Ball -------\n");
	pprintf("--------------------------\n");

	ShotAngBegTmp = ShotAngBeg;
	ShotAngEndTmp = ShotAngEnd;
	ShotSpeedTmp = ShotSpeed;

	ShotFinished = serviceAddress;

	SendTorrowsDown(ShootBallB);

}

void ShootBallB() {

	ShootBall(ShotAngBegTmp, ShotAngEndTmp, ShotSpeedTmp);

	startTimer(15, 1000, ShotWatch, ShotWatchEnd);
}

void ShootBallNT(float ShotAngBeg, float ShotAngEnd, float ShotSpeed);

void ShootBallNT(float ShotAngBeg, float ShotAngEnd, float ShotSpeed,
		void (*serviceAddress)()) {

	tprintf("--------------------------\n");
	pprintf("------- Shoot Ball -------\n");
	pprintf("--------------------------\n");

	ShotAngBegTmp = ShotAngBeg;
	ShotAngEndTmp = ShotAngEnd;
	ShotSpeedTmp = ShotSpeed;

	ShotFinished = serviceAddress;

	ShootBallNT(ShotAngBegTmp, ShotAngEndTmp, ShotSpeedTmp);

	startTimer(15, 1000, ShotWatch, ShotWatchEnd);

}

void ShootBallNTT(float ShotAngBeg, float ShotAngEnd, float ShotSpeed,
		void (*serviceAddress)()) {

	tprintf("--------------------------\n");
	pprintf("------- Shoot Ball -------\n");
	pprintf("--------------------------\n");

	ShotAngBegTmp = ShotAngBeg;
	ShotAngEndTmp = ShotAngEnd;
	ShotSpeedTmp = ShotSpeed;

	ShotFinished = serviceAddress;

	ShootBallNT(ShotAngBegTmp, ShotAngEndTmp, ShotSpeedTmp);

	startTimer(14, 1000, ShotWatch, ShotWatchEnd);

}
void ShotWatch(uc i) {
	/*
	 if (shooterState == 0 || shooterState == 104)
	 {
	 shooterState = 0;

	 tripTimer(i);

	 return;
	 }

	 if (inDisable)
	 {
	 tprintf("inDisable (Abort)\n");
	 tripTimer(i);
	 }
	 */
}

void ShotWatchEnd(uc i) {

	pprintf("--------------------------\n");
	pprintf("-------Shot Finished------\n");
	tprintf("--------------------------\n");

	// blockDriving=0;

	// setRightDriveTrain(128,0);
	// setLeftDriveTrain (128,0);

	safe(ShotFinished);
}

int watchTripTime = 0;

float baseLoc = 0;

void driveForward(uc i) {
	int speed = 30;

	float dist = fabs(loc[1] - baseLoc);

	if (dist > 60) {
		tripTimer(i);

		return;
	}

	blockDriving = 1;

	setRightDriveTrain(128 + speed, 0);
	setLeftDriveTrain(128 + speed, 0);

}

//--------------------------------------------------------------------------------------------

void driveFinish(uc i) {
	blockDriving = 0;
}

void safe(void (*ptrAdr)()) {
	if (ptrAdr)
		ptrAdr();
}

//--------------------------------

void (*moveForwardFinished)() = 0;
int moveForwardDist = 0;
int moveForwardSpeed = 0;

void driveForward2(uc i) {
	//steeringTimeOut=400;

	int speed = moveForwardSpeed;

	if (speed < 0)
		speed = -speed;

	float curLoc = loc[aLeftDist];
	if (!useLeftDist)
		curLoc = loc[aRightDist];

	float dist = fabs(curLoc - baseLoc);

	if (dist > moveForwardDist) {

		tripTimer(i);

		return;
	}

	blockDriving = 1;

	setRightDriveTrain(128 + speed, 0);
	setLeftDriveTrain(128 + speed, 0);

	if (inDisable) {
		tprintf("inDisable (Abort)\n");
		tripTimer(i);
	}
}

void driveFinish2(uc i) {
	tprintf("moveFinished\n");

	blockDriving = 0;

	setRightDriveTrain(128, 0);
	setLeftDriveTrain(128, 0);

	safe(moveForwardFinished);
}

void moveBackward(float dist, float speed, void (*serviceAddress)());

void moveForward(float dist, float speed, void (*serviceAddress)()) {
	if (dist < 0) {
		moveBackward(dist, speed, serviceAddress);
		return;
	}

	tprintf("moveForward to %6.2f at %6.2f\n", dist, speed);

	moveForwardFinished = serviceAddress;

	if (dist == 0) {
		tprintf("delay %6.3f sec\n", speed / 100.0);
		startTimer(15, int(speed), 0, driveFinish2);
		return;
	}

	if (speed == 0)
		speed = 100;

	if (dist < 0) {
		dist = -dist;
		speed = -speed;
	}

	moveForwardDist = (int) dist;

	moveForwardSpeed = (int) speed;

	float curLoc = loc[aLeftDist];
	if (!useLeftDist)
		curLoc = loc[aRightDist];

	baseLoc = curLoc;

	startTimer(15, 700, driveForward2, driveFinish2);
}

//-----------------------------------------------------

void (*moveBackwardFinished)() = 0;
int moveBackwardDist = 0;
int moveBackwardSpeed = 0;

void driveBackward2(uc i) {
	//steeringTimeOut=400;

	int speed = moveBackwardSpeed;

	if (speed > 0)
		speed = -speed;

	float curLoc = loc[aLeftDist];
	if (!useLeftDist)
		curLoc = loc[aRightDist];

	float dist = fabs(curLoc - baseLoc);

	if (dist > moveBackwardDist) {

		tripTimer(i);

		return;
	}

	blockDriving = 1;

	setRightDriveTrain(128 + speed, 0);
	setLeftDriveTrain(128 + speed, 0);

	if (inDisable) {
		tprintf("inDisable (Abort)\n");
		tripTimer(i);
	}
}

void driveFinish2b(uc i) {
	tprintf("moveFinished\n");

	blockDriving = 0;

	setRightDriveTrain(128, 0);
	setLeftDriveTrain(128, 0);

	safe(moveBackwardFinished);
}

void moveBackward(float dist, float speed, void (*serviceAddress)()) {
	tprintf("moveBackward to %6.2f at %6.2f\n", dist, speed);

	moveBackwardFinished = serviceAddress;

	if (dist == 0) {
		tprintf("delay %6.3f sec\n", speed / 100.0);
		startTimer(15, speed, 0, driveFinish2b);
		return;
	}

	if (speed == 0)
		speed = 100;

	if (dist < 0) {
		dist = -dist;
		speed = -speed;
	}

	moveBackwardDist = (int) dist;

	moveBackwardSpeed = (int) speed;

	float curLoc = loc[aLeftDist];
	if (!useLeftDist)
		curLoc = loc[aRightDist];

	baseLoc = curLoc;

	startTimer(11, 1000, driveBackward2, driveFinish2b);
}
//-------------------------------------------------

void (*moveElevFinished)() = 0;
int moveElevDest = 0;
float moveElevSpeed = 0;

void moveElev2(uc i) {
	//    float distToGo=moveElevDest-loc[aElev];

	//    if (moveElevSpeed<0) distToGo=-distToGo;

	// if (distToGo<5) {float scale=distToGo/5.0; if (scale<0.2) scale=0.2;RS_ShotAngEnd=moveElevSpeed*scale; }

	//     if (distToGo<5) {float scale=distToGo/5.0; if (scale<elevBrake) scale=elevBrake;RS_ShotAngEnd=moveElevSpeed*scale; }

	//    if (distToGo<=0) {

	//    	 tprintf("ElevMoveFinished\n");

	//   	 tripTimer(i);

	//   	 return;
	//   }

	//  if (inDisable)
	//  {
	//  	tprintf("inDisable (Abort)\n");
	//  	tripTimer(i);
	// }
}

void ElevFinish2(uc i) {
	tprintf("ElevFinished\n");

	//RS_ShotAngBeg = 2; //Stop Elev motion

	safe(moveElevFinished);
}

void XmoveElev(float dest, float speed, void (*serviceAddress)()) {

}

//------------------------------------------------------------

extern float elevCmd;

void mmoveElev2(uc i) {
	// float distToGo=fabs(moveElevDest-loc[aElev]);

	// elevCmd=moveElevDest;

	/*
	 if (elevCmd<moveElevDest)
	 {
	 elevCmd+=1; if (elevCmd>moveElevDest) elevCmd=moveElevDest;
	 }
	 else
	 if (elevCmd>moveElevDest)
	 {

	 elevCmd-=1; if (elevCmd<moveElevDest) elevCmd=moveElevDest;

	 }


	 if (distToGo<=0.1) {

	 tprintf("ElevMoveFinished\n");

	 tripTimer(i);

	 return;
	 }

	 if (inDisable)
	 {
	 tprintf("inDisable (Abort)\n");
	 tripTimer(i);
	 }*/
}

void eElevFinish2(uc i) {
	tprintf("ElevFinished\n");

	elevFlag = 0; //Stop CloseLoop
	//RS_ShotAngBeg = 2; //Stop Elev motion

	safe(moveElevFinished);
}

void moveElev(float dest, float speed, void (*serviceAddress)()) {
	/*
	 tprintf("mmoveElev to %6.2f at %6.2f\n",dest,speed);

	 moveElevFinished=serviceAddress;

	 moveElevDest =(int)dest;

	 elevCmd=loc[aElev];

	 elevFlag=20; //start closeloop


	 startTimer(aElev ,500,mmoveElev2, eElevFinish2);
	 */
}

//-------------------------------------------------

void (*moveFeederFinished)() = 0;
int moveFeederDest = 0;
float moveFeederSpeed = 0;

void moveFeeder2(uc i) {
	/*
	 float distToGo=moveFeederDest-loc[aFeeder];

	 if (moveFeederSpeed<0) distToGo=-distToGo;

	 if (distToGo<=0) {

	 tprintf("FeederMoveFinished\n");

	 tripTimer(i);

	 return;
	 }

	 if (inDisable)
	 {
	 tprintf("inDisable (Abort)\n");
	 tripTimer(i);
	 }
	 */
}

void FeederFinish2(uc i) {
	//tprintf("FeederFinished\n");

	//rollerSpitRate=0; //Stop Feeder motion

	// safe(moveFeederFinished);
}

void moveFeeder(float dest, float speed, void (*serviceAddress)()) {
	//tprintf("moveFeeder to %6.2f at %6.2f\n",dest,speed);
	//
	//moveFeederFinished=serviceAddress;
	//
	//moveFeederDest =(int)dest;
	//
	//if (dest<loc[aFeeder]) speed=-speed;
	//
	//moveFeederSpeed= speed;
	//
	//rollerSpitRate=1;
	//curHotRatio =speed;
	//
	//startTimer(aFeeder ,500,moveFeeder2, FeederFinish2);
}

//-------------------------------------------------

void (*moveArmFinished)() = 0;
int moveArmDest = 0;
float moveArmSpeed = 0;

void moveArm2(uc i) {
	/*
	 float distToGo=moveArmDest-loc[aArm];

	 if (moveArmSpeed<0) distToGo=-distToGo;

	 if (distToGo<10) {float scale=distToGo/10.0; if (scale<0.2) scale=0.2; armRate=moveArmSpeed*scale; }

	 if (distToGo<=0) {

	 tprintf("armMoveFinished\n");

	 tripTimer(i);

	 return;
	 }

	 if (inDisable)
	 {
	 tprintf("inDisable (Abort)\n");
	 tripTimer(i);
	 }
	 */
}

void ArmFinish2(uc i) {
	tprintf("ArmFinished\n");

	//armState = 2; //Stop Arm motion

	safe(moveArmFinished);
}

void moveArm(float dest, float speed, void (*serviceAddress)()) {
	/*
	 tprintf("moveArm to %6.2f at %6.2f\n",dest,speed);

	 moveArmFinished=serviceAddress;

	 moveArmDest =(int)dest;

	 if (dest<loc[aArm]) speed=-speed;

	 moveArmSpeed= speed;

	 armState=1;
	 armRate =speed;

	 startTimer(aArm ,350,moveArm2, ArmFinish2);
	 */
}

//-------------------------------------------------

//extern int lineLeft  ;
//extern int lineCenter;
//extern int lineRight ;

void (*lineTrackFinished)() = 0;
int lineTrackDist = 0;
float lineTrackSpeed = 0;

int lastTrackDir = 0;

int ltOnce = 0;

//int slowDown=0;

int stopAtLine = 0;
float trackType = 0;

int lineTrackDir = 0;

void lineTrack2(uc i) {
	//steeringTimeOut=400;
	/*
	 int speed=(int)lineTrackSpeed;

	 // if (speed<0)  speed=-speed;

	 if (speed> 120) speed= 120;
	 if (speed<-120) speed=-120;

	 if (speed==0) speed=10;

	 float dist=fabs(loc[aRightDist]-baseLoc);

	 if (dist>lineTrackDist) {

	 tripTimer(15);

	 return;
	 }

	 blockDriving=1;

	 lastTrackDir=0;

	 if (lineTrackDir==0)
	 {
	 if (lineRight ) lastTrackDir= 1;
	 if (lineLeft  ) lastTrackDir=-1;
	 }
	 else
	 {
	 // sensors in back of robot

	 if (armUpLimit  ) lastTrackDir= 1;
	 if (armDownLimit) lastTrackDir=-1;
	 }


	 //if (!lineRight && !lineLeft && !lineCenter) lastTrackDir=3;

	 float rtAdj=1;
	 float ltAdj=1;

	 if (trackType==0) //Track Line
	 {
	 if (lastTrackDir==-1) {rtAdj=1.0;          ltAdj=lineTrackAdj;}
	 if (lastTrackDir== 1) {rtAdj=lineTrackAdj; ltAdj=1.0;}
	 }

	 if (trackType==2) //gyro Track
	 {
	 //	float g=loc[1];

	 //	if (lineTrackDir==0) g*=gyroForwardGain; else g*=gyroReverseGain; //forward

	 //	float gg=g; if (gg<0) gg=-gg;

	 //	if (gg>0.9) gg=0.9;
	 //	if (gg<0.0) gg=0.0;

	 //	if (g>0) rtAdj=1.0-gg; else ltAdj=1.0-gg;

	 //	if (lineTrackDir==0) {float tmp=rtAdj; rtAdj=ltAdj; ltAdj=tmp; }
	 }

	 // tprintf("speed %6d\n",speed);


	 setRightDriveTrain(128+speed*rtAdj,0);
	 setLeftDriveTrain (128+speed*ltAdj,0);



	 //  if (inDisable)

	 //	tripTimer(15);

	 // if (trackType==2)
	 // if (lineRight && lineLeft && lineCenter) //we have hit the v?

	 // tripTimer(15);

	 */
}

void lineTrackFinish(uc i) {

	blockDriving = 0;
	//sideDrive   =0;

	setRightDriveTrain(128, 0);
	setLeftDriveTrain(128, 0);

	stopTimer(15);

	tprintf("[lineTrack Finished]\n");

	if (ltOnce) {
		ltOnce = 0;
		safe(lineTrackFinished);
	};
}

void lineTrack(float dist, float speed, float maxTime, float trackTypeP,
		void (*serviceAddress)()) {

	trackType = trackTypeP;

	if (trackTypeP == 0) {
		tprintf("[lineTrack Start] -> %6.3f\n", dist);
	}
	if (trackTypeP == 1) {
		tprintf("[lineTrack (no Line) Start] -> %6.3f\n", dist);
	}
	if (trackTypeP == 2) {
		tprintf("[lineTrack (gyro) Start] -> %6.3f\n", dist);
	}

	ltOnce = 1;

	lineTrackFinished = serviceAddress;

	lineTrackDir = 0;

	if (dist < 0) {
		dist = -dist;
		speed = -speed;
		lineTrackDir = 1;
	}

	lineTrackDist = (int) dist;

	lineTrackSpeed = speed;

	baseLoc = loc[aRightDist];

	if (maxTime < 0)
		maxTime = 12;

	startTimer(15, (int) (maxTime * 100), lineTrack2, lineTrackFinish);
}

//--------------------------------------

void (*rotateRobotFinished)() = 0;
float rotateRobotDest = 0;
float rotateRobotSpeed = 0;

float finishRotateIfGreaterThan = 0;

int rtOnce = 0;

void rotateRobot2(uc i) {
	//steeringTimeOut=400;

	int speed = (int) rotateRobotSpeed;

	// if (speed<0)  speed=-speed;

	if (speed > 120)
		speed = 120;
	if (speed < -120)
		speed = -120;

	if (speed == 0)
		speed = 10;

	// float dist=fabs(loc[1]-rotateRobotDest);

	float rtAdj = 1;
	float ltAdj = 1;

	if (finishRotateIfGreaterThan) {
		if (loc[1] > rotateRobotDest) {
			tripTimer(12);
			return;
		}

		rtAdj = -1;

	} else {
		if (loc[1] < rotateRobotDest) {
			tripTimer(12);
			return;
		}

		ltAdj = -1;
	}

	blockDriving = 1;

	setRightDriveTrain(128 + speed * rtAdj, 0);
	setLeftDriveTrain(128 + speed * ltAdj, 0);

}

void rotateRobotFinish(uc i) {

	blockDriving = 0;

	setRightDriveTrain(128, 0);
	setLeftDriveTrain(128, 0);

	stopTimer(12);

	tprintf("[rotateRobot Finished]\n");

	if (rtOnce) {
		ltOnce = 0;
		safe(rotateRobotFinished);
	};
}

void rotateRobot(float dest, float speed, float maxTime,
		void (*serviceAddress)()) {

	if (fabs(loc[1] - dest) < 2) {
		safe(serviceAddress);
		return;
	}

	if (loc[1] < dest) {
		finishRotateIfGreaterThan = 1;

		tprintf("[pos rotateRobot Start] %6.3f -> %6.3f\n", loc[1], dest);

	} else {
		finishRotateIfGreaterThan = 0;

		tprintf("[neg rotateRobot Start] %6.3f -> %6.3f\n", loc[1], dest);

	}

	rtOnce = 1;

	rotateRobotFinished = serviceAddress;

	rotateRobotDest = dest;

	rotateRobotSpeed = speed;

	if (maxTime < 0)
		maxTime = 12;

	startTimer(12, (int) (maxTime * 100), rotateRobot2, rotateRobotFinish);
}

void setAtonArm(int arm) {
	if (arm != 0 && arm != 4 && arm != 5 && arm != 6) {
		tprintf("Arm Fault\n");
		arm = 0;
		return;
	}

	armFlag = arm;

	if (arm == 0) {
		tprintf("Arm Off\n");
		return;
	}
	if (arm == 4) {
		tprintf("Arm To Floor \n");
		return;
	}
	if (arm == 5) {
		tprintf("Arm To Safe  \n");
		return;
	}
	if (arm == 6) {
		tprintf("Arm To Elev  \n");
		return;
	}
}

//-------------------------------------------------------

void atonDelayFinish(uc i);

void (*atonDelayFinishAdr)() = 0;

void atonDelay(float delayTime, void (*finishAdr)()) {
	if (delayTime <= 0 || inDisable) {
		safe(finishAdr);
		return;
	}

	atonDelayFinishAdr = finishAdr;

	stopAllDriveTrain = 1;

	startTimer(15, (int) (delayTime * 100), 0, atonDelayFinish);
}

void atonDelayFinish(uc i) {
	stopAllDriveTrain = 0;
	safe(atonDelayFinishAdr);
}

//void a1DropBridge();
void a2DropBridge();
void a3DropBridge();
void a4DropBridge();
void a5DropBridge();
void a6DropBridge();

void ShootBall2(unsigned char i) {

}

void ShootBall1(unsigned char i) {

}

//float toroIsDown=0;
// float toroIsUp=0;
// float toroIsCollecting=0;
// float toroIsSpitting=0;
// float toroInManual=0;

float toroIsGoingUp = 0;
float toroIsGoingDown = 0;

void SetToroUpSpeed(float rate);
void SetToroDownSpeed(float rate);

void (*toroFinishedAddress)() = 0;
void (*toroFinishedAddressTmp)() = 0;
extern float currentToroUpDownSpeed;

void ToroDownFinished(unsigned char i) {
	// if (currentToroUpDownSpeed>0.1) {}

	SetToroDownSpeed(0);
	toroIsDown = 1;
	toroIsUp = 0;

	toroIsGoingDown = 0;

	if (toroFinishedAddress)
		toroFinishedAddress();
}

double pulseEndTime = 0;

double GetTime();

int toroDownWhileShooting = 0;

double toroDownStartTime = 0;

void ToroDownSerivce(unsigned char i) {
	double curTime = Timer::GetFPGATimestamp();
	// tprintf("--- ToroDown Service   %6.3f  %3d  \n",float(curTime-toroDownStartTime),toroDownWhileShooting );

	if (curTime < pulseEndTime)
		SetToroDownSpeed(1.0);
	else

	//SetToroDownSpeed(toroDownRate);

	if (curTime - toroDownStartTime > 0.2 && toroDownWhileShooting) {
		//tprintf("------ ToroDown Finished -------\n");
		//tprintf("--------------------------------\n");

		toroDownWhileShooting = 0;

		toroFinishedAddressTmp = toroFinishedAddress;

		toroFinishedAddress = 0;

		if (toroFinishedAddressTmp)
			toroFinishedAddressTmp();

	}

}

void SendTorrowsDown(void (*serviceAddress)()) {
	toroDownStartTime = Timer::GetFPGATimestamp();

	// tprintf("SendTorrowsDown\n");

	toroFinishedAddress = serviceAddress;

	//SetToroDownSpeed(toroDownRate);

	toroIsGoingDown = 1;

	// if (toroIsDown) {toroFinishedAddress=0; serviceAddress();}

	//pulseEndTime = frc::GetClock() + toroUpTime / 4;

	toroIsGoingUp = 0;

	for (int L = 4; L < 9; L++) {
		if (!timerActive(L)) {
			//startTimer(L, int(toroDownTime * 100), ToroDownSerivce, ToroDownFinished);

			return;

		}

	}

}

void ToroUpFinished(unsigned char i) {
	//if (currentToroUpDownSpeed<-0.1) {}

	SetToroUpSpeed(0);
	toroIsDown = 0;
	toroIsUp = 1;

	toroIsGoingUp = 0;

	if (toroFinishedAddress)
		toroFinishedAddress();
}

/*
 void ToroUpSerivce(unsigned char i)
 {
 double curTime = frc::GetClock();

 if (curTime < pulseEndTime) SetToroUpSpeed(1.0);
 else

 //SetToroUpSpeed(toroUpRate);



 }
 */

/*
 void SendTorrowsUp(void(*serviceAddress)())
 {
 //tprintf("SendTorrowsUp\n");

 toroFinishedAddress = serviceAddress;

 SetToroUpSpeed(toroUpRate);

 toroIsGoingUp = 1;

 // if (toroIsUp ) {toroFinishedAddress=0; serviceAddress();}

 toroIsGoingDown = 0;

 pulseEndTime = frc::GetClock() + toroUpTime / 4;

 for (int L = 4; L < 9; L++)
 {
 if (!timerActive(L))
 {
 startTimer(L, int(toroUpTime * 100), ToroUpSerivce, ToroUpFinished);

 return;

 }

 }



 }
 */

// float bumperIsDown=0;
int bumperIsGoingDown = 0;
int bumperIsGoingUp = 0;

int catcherIsGoingOpen = 0;
//int catcherIsGoingClose=0;

void SetBumperUpSpeed(float rate);
void SetBumperDownSpeed(float rate);

void (*bumperFinishedAddress)() = 0;

// float bumperUpRate  =1;
// float ArmRes=1;

void BumperDownFinished(unsigned char i) {
	SetBumperDownSpeed(0);

	bumperIsDown = 1;
	bumperIsUp = 0;

	bumperIsGoingDown = 0;
	bumperIsGoingUp = 0;

	if (bumperFinishedAddress)
		bumperFinishedAddress();
}

/*
 void SendBumperDown(void(*serviceAddress)())
 {
 tprintf("SendBumperDown\n");

 bumperFinishedAddress = serviceAddress;

 SetBumperDownSpeed(bumperDownRate);

 bumperIsGoingDown = 1;
 bumperIsGoingUp = 0;

 startTimer(8, int(bumperDownTime * 100), 0, BumperDownFinished);
 }

 void BumperUpFinished(unsigned char i)
 {
 SetBumperUpSpeed(0);

 bumperIsDown = 0;
 bumperIsUp = 1;

 bumperIsGoingDown = 0;
 bumperIsGoingUp = 0;


 if (bumperFinishedAddress) bumperFinishedAddress();
 }


 void SendBumperUp(void(*serviceAddress)())
 {
 tprintf("SendBumperUp\n");

 bumperFinishedAddress = serviceAddress;

 SetBumperUpSpeed(bumperUpRate);

 bumperIsGoingDown = 0;
 bumperIsGoingUp = 1;


 startTimer(8, int(bumperUpTime * 100), 0, BumperUpFinished);
 }

 void SetCatcherOpenSpeed(float rate);
 void SetCatcherCloseSpeed(float rate);

 void(*catcherFinishedAddress)() = 0;

 //float catcherIsOpen=0;
 //float catcherIsClosed=0;

 //int catcherIsGoingOpen  =0;
 int catcherIsGoingClosed = 0;


 void CatcherOpenFinished(unsigned char i)
 {
 SetCatcherOpenSpeed(0);

 catcherIsOpen = 1;
 catcherIsClosed = 0;

 catcherIsGoingOpen = 0;
 catcherIsGoingClosed = 0;

 if (bumperFinishedAddress) catcherFinishedAddress();
 }

 void OpenCatcher(void(*serviceAddress)())
 {
 tprintf("OpenCatcher\n");

 catcherFinishedAddress = serviceAddress;

 SetCatcherOpenSpeed(catcherOpenRate);

 catcherIsGoingOpen = 1;
 catcherIsGoingClosed = 0;

 startTimer(7, int(catcherOpenTime * 100), 0, CatcherOpenFinished);
 }

 void CatcherCloseFinished(unsigned char i)
 {
 SetCatcherCloseSpeed(0);

 catcherIsOpen = 0;
 catcherIsClosed = 1;

 catcherIsGoingOpen = 0;
 catcherIsGoingClosed = 0;

 if (bumperFinishedAddress) catcherFinishedAddress();
 }

 void CloseCatcher(void(*serviceAddress)())
 {
 tprintf("CloseCatcher\n");

 catcherFinishedAddress = serviceAddress;

 SetCatcherCloseSpeed(catcherCloseRate);

 catcherIsGoingOpen = 0;
 catcherIsGoingClosed = 1;

 startTimer(7, int(catcherCloseTime * 100), 0, CatcherCloseFinished);
 }


 */

void TorrowsOff() {
	tprintf("TorrowsOff\n");
}

void TorrowsCollect() {
	tprintf("TorrowsCollect\n");
}

void TorrowsSpit() {
	tprintf("TorrowsSpitn");
}

float fabs(float);

void finishLink(unsigned char i);

int linkTmp = 0;

void AtonLink(int from, int link, float lTime) {
	if (from == link)
		return;

	linkTmp = link;

	if (lTime <= 0)
		finishLink(0);

	else

		startTimer(15, int(lTime * 100), 0, finishLink);
}

void finishLink(unsigned char i) {
	if (linkTmp == 1)
		aton1();
	if (linkTmp == 2)
		aton2();
	if (linkTmp == 3)
		aton3();
	if (linkTmp == 4)
		aton4();
	if (linkTmp == 5)
		aton5();
	if (linkTmp == 6)
		aton6();
}

void stopCollectIn(float stopTime);

void ToroOffCmd();

void CatcherOpenCmd();

//---------- Aton Variables -------------

float atonDelayTime = 0;

int inAt = 0;

#define tI  unsigned char i

//----------------------------
//---------- ATON 1 ----------
//----------------------------

void a1MoveSecond(unsigned char i);
void a1MoveFirst();
void a1Finished();
void a1FinishShot1(unsigned char i);

void ToroCollectCmd();

void a1MoveToCanDrop1(tI);
void a1DropCan(tI);
void a1MoveToTote2(tI);
void a1GrabTote2AndCan(tI);
void a1MoveToCanDrop2(tI);
void a1DropCan2(tI);
void a1MoveToTote3(tI);
void a1GrabTote3AndCan(tI);
void a1MoveToFinalLocation(tI);
void a1Finish(tI);

extern float autoDriveY;
extern float autoDriveX;
extern float autoDriveR;

/*

 void aton1xx( )
 {
 tprintf("Aton1\n");

 if (inDisable) {tprintf("inDisable (Abort)\n");return;}

 ttprintf("Lift Tote and Can\n");

 startTimer(3,100,0,a1MoveToCanDrop1);
 }


 void a1MoveToCanDrop1     (tI)
 {
 ttprintf("a1MoveToCanDrop1\n");

 autoDriveY=0.0;
 autoDriveX=0.5;

 startTimer(3,200,0,a1DropCan);
 }

 void a1DropCan            (tI)
 {
 ttprintf("a1DropCan\n");

 autoDriveY=0.0;
 autoDriveX=0.0;

 startTimer(3,100,0,a1MoveToTote2);
 }

 void a1MoveToTote2        (tI)
 {
 ttprintf("a1MoveToTote2\n");

 autoDriveY=0.0;
 autoDriveX=-0.5;


 startTimer(3,200,0,a1GrabTote2AndCan);
 }

 void a1GrabTote2AndCan    (tI)
 {
 ttprintf("a1GrabTote2AndCan\n");

 autoDriveY=0.5;
 autoDriveX=0.0;

 startTimer(3,100,0,a1MoveToCanDrop2);
 }

 void a1MoveToCanDrop2     (tI)
 {
 ttprintf("a1MoveToCanDrop2\n");

 autoDriveY=0.0;
 autoDriveX=0.5;

 startTimer(3,200,0,a1DropCan2);

 }

 void a1DropCan2           (tI)
 {
 ttprintf("a1DropCan2\n");

 autoDriveY=0.0;
 autoDriveX=0.0;

 startTimer(3,100,0,a1MoveToTote3);
 }

 void a1MoveToTote3        (tI)
 {
 ttprintf("a1MoveToTote3\n");

 autoDriveY=0.0;
 autoDriveX=-0.5;

 startTimer(3,200,0,a1GrabTote3AndCan);

 }
 void a1GrabTote3AndCan    (tI)
 {
 ttprintf("a1GrabTote3AndCan\n");

 autoDriveY=0.5;
 autoDriveX=0.0;

 startTimer(3,100,0,a1MoveToFinalLocation);
 }

 void a1MoveToFinalLocation(tI)
 {
 ttprintf("a1MoveToFinalLocation\n");

 autoDriveY=0.0;
 autoDriveX=0.5;

 startTimer(3,200,0,a1Finish);
 }

 void a1Finish             (tI)
 {
 ttprintf("a1Finish\n");

 autoDriveY=0.0;
 autoDriveX=0.0;


 }

 */

void a1FinishShot1(unsigned char i) {
	//ShootBall(a1_BegA, a1_EndA, a1_Rate, 0); //a1MoveSecond);

	startTimer(3, 150, 0, a1MoveSecond);
}

//void aton1HG() {if (int(a1_Cmd) & 2) GetHotGoal(aton1); else aton1();  }

void a1MoveFinishedB(unsigned char t) {

	//ShootBall(a1_BegA, a1_EndA, a1_Rate, a1Finished);
}

void a1MoveFinished() {
	tprintf("a1MoveFinished\n");

	if (cameraSelect < 2)
		cameraSelect = 0;

	if (int(a1_Cmd) & 2) {
		double dTime = Timer::GetFPGATimestamp() - beginAtonTime;

		if (hotGoal == 0 && dTime < 5.0) {
			SendTorrowsDown(0);

			startTimer(3, int((5.0 - dTime) * 100), 0, a1MoveFinishedB);

			return;
		}
	}

	//ShootBall(a1_BegA, a1_EndA, a1_Rate, a1Finished);
}

void a1MoveFirst() {
	tprintf("a1MoveFirst\n");

	holdStraight = 1;

	//if (cameraSelect < 2 && a1_Dist < 0) cameraSelect = 1;

	//moveForward(a1_Dist, a1_Speed, a1MoveFinished);

}

void ToroCollectCmd();

void a1MoveSecond(unsigned char i) {
	tprintf("a1MoveSecond\n");

	tprintf("Move Backward to Pickup Ball\n");

	holdStraight = 1;

	//ToroCollectCmd();

	//if (cameraSelect < 2 && a1_Dist < 0) cameraSelect = 1;

	//moveForward(a1_Dist, a1_Speed, a1Finished);

}

void a1Finished() {
	tprintf("a1Finished (Aton1 Finished)\n");

	holdStraight = 0;

	//if (a1_Coll == 0) ToroOffCmd();

	//if (cameraSelect < 2 && a1_Coll == 0) { cameraSelect = 0; inAt = 0; }

	ttprintf("Aton1 Finished\n")
;
	//AtonLink(1, a1_Coll, a1_cTime);

}

//----------------------------
//---------- ATON 2 ----------
//----------------------------

void turn(float ang);
void nextTurn(float ang);

int reverseDrive = 0;

void ReverseDrive(int v) {
	reverseDrive = v;
}

void ReverseDriveToggle() {
	if (reverseDrive)
		reverseDrive = 0;
	else
		reverseDrive = 1;

	tprintf("ReverseDrive %d\n", reverseDrive);
}

void ReverseDriveToggleNoHold() {
	if (reverseDrive)
		reverseDrive = 0;
	else
		reverseDrive = 1;

	holdStraight = 0;

	tprintf("ReverseDrive %d\n", reverseDrive);
}

void Turn180Finish(unsigned char i) {
	drivingOff = 0;
}

void Turn180() {
	drivingOff = 1;

	//turn(180);

	startTimer(13, 100, 0, Turn180Finish);
}

int spinFlag = 0;

void Spin180Finish(unsigned char i) {
	ReverseDriveToggle();

	drivingOff = 0;

	spinFlag = 0;

}

void Spin180() {
	tprintf("Spin 180\n");

	drivingOff = 1;

	spinFlag = 1;

	//turn(180);

	startTimer(13, 100, 0, Spin180Finish);
}

void fullStop() {

}

//----------------------------------------------------
void testMoveFinished() {
	holdStraight = 0;

	tprintf("Test Move Finished\n");
}

void TestAtonOneMove() {

	tprintf("Test Aton One Move\n");

	//float testDist = a1_Dist;
	//float testSpeed = a1_Speed;

	//if (testDist == 0) testDist = 10;
	//if (testSpeed == 0) testSpeed = 10;

	holdStraight = 1;

	//moveForward(testDist, testSpeed, testMoveFinished);

}

void TestAtonTwoMove() {

	tprintf("Test Aton Two Move\n");

	//float testDist = a2_Dist;
	//float testSpeed = a2_Speed;

	//if (testDist == 0) testDist = 10;
	//if (testSpeed == 0) testSpeed = 10;

	holdStraight = 1;

	//moveForward(testDist, testSpeed, testMoveFinished);

}

void TestAtonThreeMove() {

	tprintf("Test Aton Three Move\n");

	//float testDist = a3_Dist;
	//float testSpeed = a3_Speed;

	//if (testDist == 0) testDist = 10;
	//if (testSpeed == 0) testSpeed = 10;

	holdStraight = 1;

	//moveForward(testDist, testSpeed, testMoveFinished);

}
void TestAtonFourMove() {

	tprintf("Test Aton Four Move\n");

	//float testDist = a4_Dist;
	//float testSpeed = a4_Speed;

	//if (testDist == 0) testDist = 10;
	//if (testSpeed == 0) testSpeed = 10;

	holdStraight = 1;

	//moveForward(testDist, testSpeed, testMoveFinished);

}
void TestAtonFiveMove() {

	tprintf("Test Aton Five Move\n");

	//float testDist = a5_Dist;
	//float testSpeed = a5_Speed;

	//if (testDist == 0) testDist = 10;
	//if (testSpeed == 0) testSpeed = 10;

	holdStraight = 1;

	//moveForward(testDist, testSpeed, testMoveFinished);

}
void TestAtonSixMove() {

	tprintf("Test Aton Six Move\n");

	//float testDist = a6_Dist;
	//float testSpeed = a6_Speed;

	//if (testDist == 0) testDist = 10;
	//if (testSpeed == 0) testSpeed = 10;

	holdStraight = 1;

	//moveForward(testDist, testSpeed, testMoveFinished);

}

void TestAtonOneShoot() {
	tprintf("Test Aton One Shoot\n");

	//ShootBall(a1_BegA, a1_EndA, a1_Rate, 0);
}

void TestAtonTwoShoot() {
	tprintf("Test Aton Two Shoot\n");

	//ShootBall(a2_BegA, a2_EndA, a2_Rate, 0);
}

void TestAtonThreeShoot() {
	tprintf("Test Aton Three Shoot\n");

	//ShootBall(a3_BegA, a3_EndA, a3_Rate, 0);
}

void TestAtonFourShoot() {
	tprintf("Test Aton Four Shoot\n");

	//ShootBall(a4_BegA, a4_EndA, a4_Rate, 0);
}

void TestAtonFiveShoot() {
	tprintf("Test Aton Five Shoot\n");

	//ShootBall(a5_BegA, a5_EndA, a5_Rate, 0);
}

void TestAtonSixShoot() {
	tprintf("Test Aton Six Shoot\n");

	//ShootBall(a6_BegA, a6_EndA, a6_Rate, 0);
}

void GyroTestFinish() {
	tprintf("[Gyro Test Finished]\n\n");

	blockGyroReset = 0;
	atonActive = 0;
	inAt = 0;
}

void GyroTest() {
	/*

	 tprintf("GyroTest\n");

	 ZeroAll();

	 inAt=1;

	 atonActive=15000;

	 resetGyro();blockGyroReset=-1500;

	 resetDist();

	 lineTrack (a1Dist,a1Rate,a1Parm3,2,GyroTestFinish);
	 */
}

void (*hotGoalFinishedAddress)() = 0;

//int hotGoal=0;
//int lookForHotGoal=0;

extern float hotGoal;
extern float lookForHotGoal;

void GetHotGoalFinished(unsigned char i) {
	return;

	blackAndWhite = 8;
	//fastVideo=1;

	lookForHotGoal = 0;

	if (hotGoalFinishedAddress)
		hotGoalFinishedAddress();
}

double hotGoalStartTime = 0;

void GetHotGoalService(unsigned char i) {

	return;

	if (Timer::GetFPGATimestamp() - hotGoalStartTime < 0.25) {
		hotGoal = 0;
		return;
	}

	if (hotGoal) {
		blackAndWhite = 8;
		//	fastVideo=1;

		lookForHotGoal = 0;

		tripTimer(i);
	}

}

void GetHotGoal(void (*serviceAddress)()) {
	hotGoalFinishedAddress = serviceAddress;

	blackAndWhite = 0;
	//fastVideo    =0;

	hotGoalStartTime = Timer::GetFPGATimestamp();

	lookForHotGoal = 1;

	startTimer(4, 110, GetHotGoalService, GetHotGoalFinished);
}

//----------------------
//---- Two Ball Aton
//----------------------

void MoveBackToGetBall();
void DragBallToShootingLocation(unsigned char i);
void ReleaseBallAndShoot();
void WaitLoadPosition();
void MoveToCollectSecondBall();
void WaitForBallTwoCollect();
void MoveForwardWithBallTwo(unsigned char i);
void WaitToShootBallTwo();
void ShootTheSecondBall(unsigned char i);
void TwoBallAtonFinished();
void WaitForBallCollect();
void DropTheBall();

void SetToroRate(float rate);

void ShootFirstBall(unsigned char i);

double atonTime0 = 0;
double atonTime1 = 0;
double atonTime2 = 0;
double atonTime3 = 0;
double atonTime4 = 0;
double atonTime5 = 0;
double atonTime6 = 0;
double atonTime7 = 0;
double atonTime8 = 0;
double atonTime9 = 0;
double atonTime10 = 0;
double atonTime11 = 0;

void TwoBallAton() {
	//ttprintf("TwoBallAton Begin\n");

	atonTime0 = Timer::GetFPGATimestamp();

	if (inDisable) {
		ttprintf("inDisable (Abort)\n")
;
		return;
	}

	ZeroAll();

	ReverseDrive(0);

	inAt = 1;

	ShiftModeFlag = 0;

	atonActive = 1000;

	resetDist();

	//CatcherOpenCmd();

	cameraSelect = 0;

	toroInManual = 0;

	//SetToroRate(a4_Rate / 100.0);

	SendTorrowsDown(MoveBackToGetBall);
}

void MoveBackToGetBall() {
	//ttprintf("MoveBackToGetBall\n");
	atonTime1 = Timer::GetFPGATimestamp();

	holdStraight = 1;

	//moveForward(-fabs(a4_Dist), a4_Speed, WaitForBallCollect);  //C1: dist,speed
}

void WaitForBallCollect() {
	atonTime2 = Timer::GetFPGATimestamp();

	//wantedArmAngle = ballLoadLoc;

	//ttprintf("WaitForBallCollect\n");

	//startTimer(3, int(a4_cTime * 100), 0, DragBallToShootingLocation); //C1: delay
}

void DragBallToShootingLocation(unsigned char i) {
	atonTime3 = Timer::GetFPGATimestamp();

	//ttprintf("DragBallToShootingLocation\n");

	//SetToroRate(a5_Coll / 100.0);  //M1: toro rate

	cameraSelect = 0;

	holdStraight = 1;

	//moveForward(a5_Dist, a5_Speed, DropTheBall); //M1: dist,speed
}

void DropTheBall() {
	atonTime4 = Timer::GetFPGATimestamp();

	//ttprintf("DropTheBall\n");

	//SetToroRate(-fabs(a4_BegA) / 100.0); //DR: toro rate

	//startTimer(3, int(a4_EndA*100.0), 0, ShootFirstBall); //DR: delay

}

void ShootFirstBall(unsigned char i) {
	atonTime5 = Timer::GetFPGATimestamp();

	//ttprintf("ShootFirstBall\n");

	//SetToroRate(0);

	//ShootBallNT(a5_BegA, a5_EndA, a5_Rate, MoveToCollectSecondBall); //S1: beg,end,rate
}

void MoveToCollectSecondBall() {
	atonTime6 = Timer::GetFPGATimestamp();

	//ttprintf("MoveToCollectSecondBall\n");

	cameraSelect = 0;

	//SetToroRate(a4_Cmd / 100.0); // C2: toro rate

	holdStraight = 1;

	moveForward(-fabs(a5_Cmd), a6_Cmd, WaitForBallTwoCollect); // C2: dist,speed
}

void WaitForBallTwoCollect() {
	atonTime7 = Timer::GetFPGATimestamp();

	//ttprintf("WaitForBallTwoCollect\n");

	//startTimer(3, int(a4_Coll * 100), 0, MoveForwardWithBallTwo); // C2: delay
}

void MoveForwardWithBallTwo(unsigned char i) {
	atonTime8 = Timer::GetFPGATimestamp();

	//ttprintf("MoveForwardWithBallTwo\n");

	cameraSelect = 0;

	holdStraight = 1;

	//moveForward(a6_Dist, a6_Speed, WaitToShootBallTwo); //M2: dist,speed
}

void WaitToShootBallTwo() {
	atonTime9 = Timer::GetFPGATimestamp();

	//ttprintf("WaitToShootBallTwo\n");

	//SetToroRate(0);

	//startTimer(3, int(a6_Coll * 100), 0, ShootTheSecondBall); // M2: delay
}

void ShootTheSecondBall(unsigned char i) {
	atonTime10 = Timer::GetFPGATimestamp();

	//ttprintf("ShootTheSecondBall\n");

	//ShootBallNT(a6_BegA, a6_EndA, a6_Rate, TwoBallAtonFinished);
}

void TwoBallAtonFinished() {
	holdStraight = 0;

	cameraSelect = 0;

	toroInManual = 1;

	inAt = 0;

	atonTime11 = Timer::GetFPGATimestamp();

	// ttprintf("Two Ball Aton Finished\n");
	printf("%5.2f : TwoBallAton Begin      \n",
			(float) (atonTime0 - atonTime0));
	printf("%5.2f : MoveBackToGetBall      \n",
			(float) (atonTime1 - atonTime0));
	printf("%5.2f : WaitForBallCollect     \n",
			(float) (atonTime2 - atonTime0));
	printf("%5.2f : DragBallToShootingLocation \n",
			(float) (atonTime3 - atonTime0));
	printf("%5.2f : DropTheBall            \n",
			(float) (atonTime4 - atonTime0));
	printf("%5.2f : ShootFirstBall         \n",
			(float) (atonTime5 - atonTime0));
	printf("%5.2f : MoveToCollectSecondBall\n",
			(float) (atonTime6 - atonTime0));
	printf("%5.2f : WaitForBallTwoCollect  \n",
			(float) (atonTime7 - atonTime0));
	printf("%5.2f : MoveForwardWithBallTwo \n",
			(float) (atonTime8 - atonTime0));
	printf("%5.2f : WaitToShootBallTwo     \n",
			(float) (atonTime9 - atonTime0));
	printf("%5.2f : ShootTheSecondBall     \n",
			(float) (atonTime10 - atonTime0));
	printf("%5.2f : Two Ball Aton Finished \n",
			(float) (atonTime11 - atonTime0));

}

//--------------------------------------------------

extern float holdBase;
extern float holdLast;
extern float targetOffsetX;

//---------------------------
//-- AUTO SHOOT WITH SONAR --
//---------------------------

void autoShot1(int index);

void sonarFinished();
void watchForShot(unsigned char i);

void ShotACmd();

int tDown = 0;

float distToGo = 0;
float shootAtDist = 0;

void sonarShot(int index) {
	if (index > 0) {
		autoShot1(index);
		return;
	}

	tprintf("sonarShot\n");

	if (inDisable) {
		tprintf("inDisable (Abort)\n");
		return;
	}

	ttprintf("SonarShot Begin\n")
;
	if (int(a2_Cmd) & 1) {
		if (fabs(joyY[2]) > 0.3 || fabs(joyY[3]) > 0.3 || fabs(joyX[2]) > 0.3
				|| fabs(joyX[3]) > 0.3) {
			return;
		};

		//if (loc[3]>57 && loc[3]<120) {ShotACmd(); return; }

		//if (loc[3]>15 && loc[3]<58) { return; }

	}

	ZeroAll();

	//resetGyro();

	ReverseDrive(0);

	inAt = 1;

	ShiftModeFlag = 0;

	atonActive = 1000;

	resetDist();

	//CatcherOpenCmd();

	cameraSelect = 0;

	//SendTorrowsDown(0);

	//SendTorrowsUp(0);

	holdLast = -1;
	holdStraight = 1;
	holdLast = -1;

	//distToGo = a2_Dist + 20;

	//shootAtDist = a2_Dist;

	//  printf("a2_Cmd %f sonar %f\n",a2_Cmd,loc[aSonar]);

	if (int(a2_Cmd) & 1) {
		if (loc[aSonar] > 16) {
			distToGo = loc[aSonar] - 100;

			shootAtDist = loc[aSonar] - 120;

		}
	}

	//	printf("shootAtDist %f\n",shootAtDist);
	//	printf("distToGo    %f\n",distToGo);

	//moveForward(distToGo, a2_Speed, sonarFinished);

	startTimer(3, 500, watchForShot, 0);

	tDown = 0;

}

extern int forceSpeedZeroBlock;

void watchForShot(unsigned char i) {

	holdStraight = 1;
	//targetOffsetX = 0;
	holdBase = 0;
	gyroShiftState = 0;

	float curLoc = loc[aLeftDist];
	if (!useLeftDist)
		curLoc = loc[aRightDist];

	if (curLoc > shootAtDist) // || loc[aSonar]<a3_Dist)
			{
		stopTimer(3);

		forceSpeedZeroBlock = 1;

		//ShootBallNT(a2_BegA, a2_EndA, a2_Rate, sonarFinished);
	}

	//printf("dToGo %5.2f cLoc %5.2f sAtDist %5.2f\n",distToGo,curLoc,shootAtDist);

	if (curLoc > shootAtDist - 10 && !tDown) {

		tDown = 1;

		SendTorrowsDown(0);

	}

	if (joyY[2] < -0.3 || joyY[3] < -0.3 || fabs(joyX[2]) > 0.3
			|| fabs(joyX[3]) > 0.3) {
		sonarFinished();
	};

}

void sonarFinished() {
	tprintf("sonarFinished \n");

	stopTimer(3);

	stopTimer(15);

	blockDriving = 0;

	setRightDriveTrain(128, 0);
	setLeftDriveTrain(128, 0);

	holdStraight = 0;

	cameraSelect = 0;

	forceSpeed = 0;
	atonActive = 0;
	stopAllDriveTrain = 0;
	blockDriving = 0;

	inAt = 0;

}

//---------------------------
//-- AUTO SHOOT            --
//---------------------------

void autoFinished1();
void watchForShot1(unsigned char i);

int autoState = 0;

float trussShotShootDistance = 0;
float trussShotMaxDistance = 0;

void autoShot1(int index) {
	tprintf("autoShot1\n");

	if (inDisable) {
		tprintf("inDisable (Abort)\n");
		return;
	}

	ttprintf("AutoShot1 Begin\n")
;
	ZeroAll();

	//resetGyro();

	ReverseDrive(0);

	inAt = 1;

	ShiftModeFlag = 0;

	atonActive = 1000;

	resetDist();

//	CatcherOpenCmd();

	cameraSelect = 0;

	//SendTorrowsDown(0);

	//SendTorrowsUp(0);

	holdLast = -1;
	holdStraight = 1;
	holdLast = -1;

	//distToGo = a3_Dist + 20;

	//trussShotMaxDistance = a3_Dist;

	trussShotShootDistance = trussShotMaxDistance / 3;

	if (trussShotDist > 0)
		trussShotShootDistance = trussShotDist;

	//moveForward(distToGo, a3_Speed, autoFinished1);

	startTimer(3, 500, watchForShot1, 0);

	tDown = 0;

	autoState = 0;

}

extern int forceSpeedZeroBlock;

void watchForShot1(unsigned char i) {
	holdStraight = 1;
	//targetOffsetX = 0;
	holdBase = 0;
	gyroShiftState = 0;

	float curLoc = loc[aLeftDist];
	if (!useLeftDist)
		curLoc = loc[aRightDist];

	if (curLoc > trussShotShootDistance && autoState == 0) // || loc[aSonar]<a3_Dist)
			{

		forceSpeedZeroBlock = 1;

		//ShootBallNTT(a3_BegA, a3_EndA, a3_Rate, 0);

		autoState = 1;

	}

	//printf("dToGo %5.2f cLoc %5.2f sAtDist %5.2f\n",distToGo,curLoc,shootAtDist);

	if (curLoc > trussShotShootDistance - 10 && !tDown && autoState == 0) {

		tDown = 1;

		SendTorrowsDown(0);

	}

	if (curLoc > trussShotMaxDistance - 20)
		if (moveForwardSpeed > 30)
			moveForwardSpeed -= 1;

	if (curLoc > trussShotMaxDistance) // || loc[aSonar]<a3_Dist)
			{
		stopTimer(3);

		autoFinished1();
	}

	if (joyY[2] < -0.3 || joyY[3] < -0.3 || fabs(joyX[2]) > 0.3
			|| fabs(joyX[3]) > 0.3) {
		autoFinished1();
	};

}

void autoFinished1() {
	tprintf("autoFinished1 \n");

	stopTimer(3);

	stopTimer(15);

	blockDriving = 0;

	setRightDriveTrain(128, 0);
	setLeftDriveTrain(128, 0);

	//SendTorrowsUp(0);

	holdStraight = 0;

	cameraSelect = 0;

	forceSpeed = 0;
	atonActive = 0;
	stopAllDriveTrain = 0;
	blockDriving = 0;

	inAt = 0;

	//AtonLink(3, a3_Coll, a3_cTime);
}

void BinArmAzm(float speed);
void BinJaw(float speed);
void BinArmElbow(float speed);
void BinArmWrist(float speed);
void ToteForkRetract(float speed);
void ToteForkIntake(float speed);
void ToteLift(float speed);
void BinLift(float speed);

// Basic Tote Lift Motion

void (*tMoveFinishedAddress)() = 0;

float tFinishLoc = 0;

int tAuto = 0;

void tMovePosService(uc i) {
	/*
	 tAuto=1;

	 float curLoc=loc[pwmToteLift1];

	 float speed = tMaxUpSpeed;

	 if (speed<=0) speed=-0.5;

	 if (!atonActivee)
	 {
	 float dist=fabs(curLoc-tFinishLoc);


	 //if (dist<6) speed/=2;
	 //if (dist<3 ) speed/=2;

	 }

	 //ToteLift(-speed);



	 if (curLoc >= tFinishLoc) tripTimer(i);


	 */

}

void tMoveNegService(uc i) {
	/*
	 tAuto=1;

	 float curLoc = loc[pwmToteLift1];

	 float speed = tMaxDnSpeed;

	 if (speed<=0.0) speed = 0.5;

	 float dist=fabs(curLoc-tFinishLoc);

	 //if (dist<6) speed/=2;
	 //if (dist<3 ) speed/=2;

	 //ToteLift(speed);



	 if (curLoc <= tFinishLoc) tripTimer(i);

	 */

}

void tMoveFinished(uc i) {

	tAuto = 0;

	if (tMoveFinishedAddress)
		tMoveFinishedAddress();
}

void tMove(float dest, void (*finished)(), float maxTime) {
	/*
	 if (dest<1) dest=1;

	 tMoveFinishedAddress = finished;

	 tFinishLoc=dest;

	 if (loc[pwmToteLift1]<tFinishLoc)

	 startTimer(5, maxTime*100,tMovePosService,tMoveFinished );

	 else

	 startTimer(5, maxTime*100, tMoveNegService, tMoveFinished);
	 */
}

// Basic Bin Lift Motion

void (*bMoveFinishedAddress)() = 0;

float bFinishLoc = 0;

int bAuto = 0;

void bMovePosService(uc i) {
	/*

	 bAuto = 1;
	 float curLoc=loc[pwmBinLift1];

	 float speed = bMaxUpSpeed;

	 if (speed<=0) speed=0.5;



	 float dist=fabs(curLoc-bFinishLoc);

	 if (dist<12) speed/=2;
	 //if (dist<6 ) speed/=2;



	 //	BinLift(speed);


	 if (curLoc >= bFinishLoc) tripTimer(i);
	 */
}

void bMoveNegService(uc i) {
	/*

	 bAuto = 1;

	 float curLoc = loc[pwmBinLift1];


	 float speed = bMaxDnSpeed;

	 if (speed<=0.0) speed = 0.5;


	 float dist=fabs(curLoc-bFinishLoc);

	 if (dist<12) speed/=2;
	 if (dist<6 ) speed/=2;

	 //	BinLift(-speed);

	 if (curLoc <= bFinishLoc) tripTimer(i);
	 */
}

void bMoveFinished(uc i) {

	bAuto = 0;

	if (bMoveFinishedAddress)
		bMoveFinishedAddress();
}

void bMove(float dest, void (*finished)(), float maxTime) {
	/*
	 bMoveFinishedAddress = finished;

	 bFinishLoc=dest;

	 if (loc[pwmBinLift1]<bFinishLoc)

	 startTimer(6, maxTime*100,bMovePosService,bMoveFinished );

	 else

	 startTimer(6, maxTime*100, bMoveNegService, bMoveFinished);
	 */
}

// Basic Azm rotation Motion

void (*aMoveFinishedAddress)() = 0;

float aFinishLoc = 0;

int aAuto = 0;

void aMovePosService(uc i) {

}

void aMoveNegService(uc i) {
	aAuto = 1;

}

void aMoveFinished(uc i) {
	//if (loc[pwmBinArmAzm])
	//	aCmdPos = loc[pwmBinArmAzm];
	//else

	aAuto = 0;

	if (aMoveFinishedAddress)
		aMoveFinishedAddress();
}

void aMove(float dest, void (*finished)(), float maxTime) {
	aMoveFinishedAddress = finished;

	aFinishLoc = dest;

	//	if (loc[pwmBinArmAzm]<aFinishLoc)

	//	startTimer(7, maxTime*100,aMovePosService,aMoveFinished );

	//	else

	//startTimer(7, maxTime*100, aMoveNegService, aMoveFinished);
}

void aaMovePosService(uc i) {
	aAuto = 1;

	float speed = 1;

	if (speed <= 0)
		speed = 0.5;

	//	BinArmAzm(speed);

	//float curLoc=loc[pwmBinArmAzm];

	//if (curLoc >= aFinishLoc) tripTimer(i);
}

void aaMoveNegService(uc i) {
	aAuto = 1;

	float speed = 1;

	if (speed <= 0.0)
		speed = 0.5;

	//	BinArmAzm(-speed);

	//float curLoc = loc[pwmBinArmAzm];

	//if (curLoc <= aFinishLoc) tripTimer(i);
}

void aaMoveFinished(uc i) {
	//if (loc[pwmBinArmAzm])
	//	aCmdPos = loc[pwmBinArmAzm];
	//else

	aAuto = 0;

	if (aMoveFinishedAddress)
		aMoveFinishedAddress();
}

void aaMove(float dest, void (*finished)(), float maxTime) {
	aMoveFinishedAddress = finished;

	aFinishLoc = dest;

	//if (loc[pwmBinArmAzm]<aFinishLoc)

	//startTimer(7, maxTime*100,aaMovePosService,aaMoveFinished );

	//else

	//startTimer(7, maxTime*100,aaMoveNegService, aaMoveFinished);
}

void aaMove(float dest) {
	aaMove(dest, 0, 10);
}

// Basic elbo rotation Motion

void (*eMoveFinishedAddress)() = 0;

float eFinishLoc = 0;

int eAuto = 0;

void eMovePosService(uc i) {
	eAuto = 1;

	//	float curLoc=loc[pwmBinArmElbow];

	//	float dist=fabs(curLoc-eFinishLoc);

	//	if (dist<40) speed/=2;
	//	if (dist<20 ) speed/=2;

	//	BinArmElbow(-speed);

	//	if (curLoc >= eFinishLoc) tripTimer(i);
}

void eMoveNegService(uc i) {
	eAuto = 1;

	//	float curLoc = loc[pwmBinArmElbow];
	//

	//	float dist=fabs(curLoc-eFinishLoc);

	//	if (dist<40) speed/=2;
	//	if (dist<20 ) speed/=2;

	//	BinArmElbow(speed);

	//	if (curLoc <= eFinishLoc) tripTimer(i);
}

void eMoveFinished(uc i) {
	//eCmdPos = loc[pwmBinArmElbow];

	eAuto = 0;

	if (eMoveFinishedAddress)
		eMoveFinishedAddress();
}

void eMove(float dest, void (*finished)(), float maxTime) {
	eMoveFinishedAddress = finished;

	eFinishLoc = dest;

	//	if (loc[pwmBinArmElbow]<eFinishLoc)

	//	startTimer(8, maxTime*100,eMovePosService,eMoveFinished );

	//	else

	//	startTimer(8, maxTime*100, eMoveNegService, eMoveFinished);
}

// move the lidar servo

void sMove(float dest, void (*finished)(), float maxTime) {
	//	sCmdPos = loc[pwmLidarServo];

	if (finished)
		finished();
}

//--------------------- move to levels ------------------

// LoadLevel    location for loaded tote
// PickupLevel  location to pickup tote

void (*LoadOnceLowFinishAddress)() = 0;
void LoadOnceLowEnd() {
	ttprintf("LoadOnceLowEnd\n")
;
	if (LoadOnceLowFinishAddress)
		LoadOnceLowFinishAddress();
}

void (*LoadOnceHighFinishAddress)() = 0;
void LoadOnceHighEnd() {
	ttprintf("LoadOnceHighEnd\n")
;
	if (LoadOnceHighFinishAddress)
		LoadOnceHighFinishAddress();
}

//----------------------------
//---------- ATON 1 ----------
//----------------------------

void donothing(unsigned char i);

void a1Finisheded(tI);

void a1_JawsAreNowClosed(tI);
void a1_BinIsUp(tI);
void a1_ShootTheBall(tI);

void MoveToolBarLowCmd();

void MoveShoeToMid();

void a1_MoveForward(tI);

/*
 void stopaton(unsigned char i){
 joyOverride = 0;
 joyXOverride = 0;
 joyYOverride = 0;
 joyZOverride = 0;

 }
 int aton1donePegAim = 0;
 void aton1(){
 printf("aton1\n");

 joyOverride = 1;
 joyXOverride = 0;
 joyYOverride = maxPegDriveSpeed;
 joyZOverride = 0;
 while( aton1donePegAim!=1)
 {
 PegAim();

 float delta = PegGoalX - PegGoalX1;
 float ydelta = PegGoalY - PegGoalY1;
 if((delta<5) && (ydelta<5)){
 dropPeg();
 startTimer(6, 200, 0, aton1_2);
 }
 }


 }
 void aton1_2(unsigned char i){
 while(true){
 GoalAim();
 }


 }


 */
/*
 void aton1()
 {
 printf("aton1\n");

 joyOverride = 1;
 joyXOverride = 0;
 joyYOverride = 0.6;
 joyZOverride = 0;

 startTimer(5, 300, 0, aton1_2);
 }

 void aton1_2(unsigned char){
 joyOverride = 1;
 joyXOverride = 0.8;
 joyYOverride = 0;
 joyZOverride = 0;

 startTimer(6, 200, 0, stopaton);
 }


 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

 atonActive = 1;

 //MoveToolBarLowCmd();

 //MoveShoeToMid();

 //autoDriveY =1.0;
 //autoDriveX = 0;
 //autoDriveR = 0;

 WaitFor(1.0, a1_MoveForward);

 */
void a1_FindTheGoal(tI);

void a1_MoveForward(tI) {

	autoDriveY = 0.6;
	autoDriveX = 0;
	autoDriveR = 0;

	WaitFor(a1_MoveTime1, a1_FindTheGoal);

}

void a1_TrackGoal(tI);

void a1_ShootTheBall(tI);

void MoveElevToHigh();

extern int gTimeOut;

extern float gyroZero;

void MoveShoeToLow();

void a1_FindTheGoal(tI) {
	autoDriveY = 0.0;
	autoDriveX = 0;
	autoDriveR = 0;

	//xDrift = 0;
	//xDriftCount = 0;

	//RaiseWheels();

	//Set_ShooterWheel(1.0);

	//MoveElevToHigh();
	/*
	 if (a1_ShotDist == 1) MoveElevToMid2(); else
	 if (a1_ShotDist == 2) MoveElevToMid(); else
	 MoveElevToHigh();


	 ToolBarAuto = 0;

	 MoveShoeToLow();

	 gTimeOut = -1000;

	 gyroZero += a1_Rotate;

	 startTimer(3, aShootTime + 100 - a1_MoveTime1 * 100, a1_TrackGoal, a1_ShootTheBall);

	 */
	//need to rotate using gyro
}

void TrackGoal2();

void a1_TrackGoal(tI) {

	LookForGoal = 1;

	//if (a1_ShotDist == 1) TrackGoal4(); else
	//	if (a1_ShotDist == 2) TrackGoal3(); else
	//		TrackGoal2();

}

void RotateToDefences(tI);

void StopTheBall(tI);

void a1_ShootTheBall(tI) {

	if (1) //GoalFound)
	{
		//Set_BallFeed(1.0);
	}

	//startTimer(1, 50, 0, StopTheBall);

	autoDriveY = 0;
	autoDriveX = 0;
	autoDriveR = 0;

	LookForGoal = 0;

	//GoalFound=0;

	GoalH = 0;
	GoalW = 0;
	GoalX = 160;
	GoalY = 120;

	autoDriveR = 0;
	autoDriveY = 0;
	atonActive = 0;

	if (1) //GoalFound)
	{
		//startTimer(1, 100, 0, RotateToDefences);
	}

	GoalFound = 0;

}

void a1xx_ShootTheBall(tI) {

	autoDriveY = 0.0;
	autoDriveX = 0;
	autoDriveR = 0;

	atonActive = 0;

}

void a1_JawsAreNowClosed(tI) {
	//bMove(a1BinLevel);

	//tMove(1);

	WaitFor(2.0, a1_BinIsUp);
}

void BinToCenter();

void a1_BinIsUp(tI) {

	ttprintf("Move forward\n")
;
	autoDriveY = a1DriveSpeed;
	autoDriveX = 0;
	autoDriveR = 0;

	//BinToCenter();

	startTimer(3, a1DriveTime * 100, 0, a1Finisheded);

}

void a1Finisheded(tI) {
	autoDriveY = 0;
	autoDriveX = 0;
	autoDriveR = 0;
}

//----------------------------
//---------- ATON 2 ----------
//----------------------------

/*
 void a2Finished();
 void MoveToCanDrop1();
 void DropCan(); //tI);
 void MoveLeftToBeInlineWithTote2(tI);
 void GrabTote2AndCan2(tI);
 void DriveIntoTote2(tI);
 void MoveToCanDrop2(tI);
 void DropCan2(tI);
 void MoveToTote3(tI);
 void GrabTote3AndCan(tI);
 void MoveToFinalLocation(tI);
 void GetReadyToLoadToteAndBin2(tI);
 void PickupToteAndBin2(tI);
 void PickupCan2(tI);
 void MoveRightToDropBin2(tI);
 void MoveLeftToBeInlineWithTote3(tI);
 void GetReadyToLoadToteAndBin3(tI);
 void MoveRightToDropBin2();
 void PickupToteAndBin2();
 void a2Finish(tI);
 void a2Finished(tI);
 void MoveRightToDropBin3();
 void PickupToteAndBin3();
 void DropCan3(tI);
 void DriveIntoTote3(tI);
 void LiftBin2AndRotateToDrop(tI);
 void RotateBin2OutOfWay();
 void PickupTote2();
 */
extern float autoDriveY;
extern float autoDriveX;

// aton pickup bins,totes and them to center of field

void JawsAreNowClosed(tI);
void WatchForTote2WithLidar(tI);
void PickupTote2(tI);
void ToteAndBinUp(tI);
void liftTote2(tI);
void hitFinished();
void StartMoveingToTote3(tI);
void GrabBin3(tI);
void PickupThirdTote(tI);
void PickupThirdToteB();
void PickupTote2Lift(tI);
void MoveToCenter(tI);
void MoveToCenter(tI);
void DropToteStack(tI);
void a2Finished(tI);
void TotesAreDown();
void StopAllThings();
void SuckStop();
void zeroToteLift();
void zeroBinLift();

void a2_ShootTheBall(tI);
void a2_FindTheGoal(tI);

//#1
/*
 void aton2()
 {
 ttprintf("#1 Aton2\n");

 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

 atonActive = 1;

 autoDriveY = 1.0;
 autoDriveX = 0;
 autoDriveR = 0;

 WaitFor(a2_MoveTime1, a2_FindTheGoal);

 }


 void a2_TrackGoal(tI);

 void MoveShoeToLow();

 void a2_FindTheGoal(tI)
 {
 autoDriveY = 0.0;
 autoDriveX = 0;
 autoDriveR = 0;

 //RaiseWheels();

 //Set_ShooterWheel(1.0);

 //MoveElevToHigh();

 if (a2_ShotDist == 1) MoveElevToMid2(); else
 if (a2_ShotDist == 2) MoveElevToMid(); else
 MoveElevToHigh();



 MoveShoeToLow();

 gTimeOut = -1000;

 xDrift = 0;
 xDriftCount = 0;

 if (fabs(a2_Rotate) > 1) gyroZero += a2_Rotate;

 //else { xDrift = a2_Rotate; xDriftCount = 100; RaiseWheels(); }

 startTimer(3, aShootTime - a2_MoveTime1 * 100, a2_TrackGoal, a2_ShootTheBall);


 //need to rotate using gyro
 }

 void TrackGoal2();

 void a2_TrackGoal(tI)
 {

 LookForGoal = 1;


 //if (a2_ShotDist == 1) TrackGoal4(); else
 //	if (a2_ShotDist == 2) TrackGoal3(); else
 ///		TrackGoal2();


 }

 void StopTheBall(unsigned char i);

 void a2_ShootTheBall(tI)
 {

 if (1) //GoalFound)
 {
 //Set_BallFeed(1.0);
 }

 //startTimer(1, 50, 0, StopTheBall);
 2017
 autoDriveY = 0;
 autoDriveX = 0;
 autoDriveR = 0;

 LookForGoal = 0;

 //GoalFound=0;

 GoalH = 0;
 GoalW = 0;
 GoalX = 160;
 GoalY = 120;

 autoDriveR = 0;
 autoDriveY = 0;
 atonActive = 0;

 if (1) //GoalFound)
 {
 //startTimer(1, 100, 0, RotateToDefences);
 }

 GoalFound = 0;

 }












 */

//#2
void JawsAreNowClosed(tI) {
	ttprintf("#2 JawsAreNowClosed\n")
;
	autoDriveY = a2ForwardMoveSpeedB2;

	//bMove(28+2);
	//tMove(17);
	//aMove(180);

	WaitFor(2, ToteAndBinUp);
}

void ToteAndBinUp(tI) {
	WaitFor(2, PickupTote2);
}

//#8

void PickupTote2(tI) {
	ttprintf("#8 PickkupTote2\n")
;
	//aaMove(269,hitFinished,3);

	autoDriveY = 0.0;

	//SuckStop();

	//  tMove(0);

	WaitFor(1.0, liftTote2);
}

void alignSpeed() {
	autoDriveY = 0.4;

}
void alignStop() {

	autoDriveY = 0.0;

}

void hitFinished() {
	aaMove(270);

	eMove(170);
}

void liftTote2(tI) {
	//tMove(17);

//	Spit();

	WaitFor(1.5, StartMoveingToTote3);
}

void WatchForTote3WithLidar(tI) {
	if (lidarValue < 14)
		tripTimer(3);
	//	if (lidarValue<40) Suck();
}

void StartSuckIt(tI) {
	//	Suck();
}

void StartLidarWatch(tI);

void StartMoveingToTote3(tI) {
	autoDriveY = a2ForwardMoveSpeedB2;

	//WaitWith(2.0,StartSuckIt);

	WaitFor(2.0, 0, StartLidarWatch);
}

void StartLidarWatch(tI) {
	//Suck();

	WaitFor(2.0, PickupThirdTote);
}

void PickupThirdTote(tI) {
	autoDriveY = 0;

	//	SuckStop();

	//tMove(0,PickupThirdToteB);

	//}

	//void PickupThirdToteB()
	//{
	//	tMove(15);

	//bMove(15);

	autoDriveX = a2RightMoveSpeedB3;

	WaitFor(1.0, MoveToCenter);
}

//#12
void MoveToCenter(tI) {
	ttprintf("#12 MoveToCenter\n")
;
	autoDriveY = 0;
	autoDriveX = a2RightMoveSpeedB3;

	//bMove(20);

	//eMove(150);

	//aMove(270);

	WaitFor(a2RightMoveTimeB3 - 1, DropToteStack);
}

//#13
void DropToteStack(tI) {
	ttprintf("#13 DropToteStack\n")
;
	// Spit();

	tMove(0, TotesAreDown);
}

void TotesAreDown() {
	autoDriveY = -1.0;
	autoDriveX = 0.0;

	WaitFor(1.0, a2Finished);
}

//#14

void a2Finished(tI) {
	ttprintf("#14 a2Finished\n")
;
	StopAllThings();
}

float getBinLiftAct();
float getToteLiftAct();

void SuckStop();

void StopAllThings() {
	for (int L = 0; L < 20; L++)
		stopTimer(L);

	autoDriveY = 0;
	autoDriveX = 0;

	aAuto = 0;
	bAuto = 0;
	tAuto = 0;
	eAuto = 0;

	//	SuckStop();

	//BinJaw (0);

	//Cop(0.0);

	atonActive = 0;

}

void StoreArm();
void StoreArmService() {
}
;
void StopStoreArm();

int topLoadTrip = 0;

void StopArm() {
	for (int L = 0; L < 20; L++)
		stopTimer(L);

	aAuto = 0;
	bAuto = 0;
	eAuto = 0;

	//	BinJaw (0);

	atonActive = 0;

}

//float aActPos=loc[pwmBinArmAzm];
//float bActPos=getBinLiftAct();
//float tActPos=getToteLiftAct();
//float eActPos=loc[pwmBinArmElbow];

float wantedA = 0;
float wantedE = 0;

void StoreArm() {
	ttprintf("StoreArm\n")
;
	wantedA = 270;
	wantedE = 10;

	aMove(wantedA);
	eMove(wantedE);

	//CloseJaws();
}

void StartingArm() {
	ttprintf("StartingArm\n")
;
	wantedA = 246;
	wantedE = 267;

	aMove(wantedA);
	eMove(wantedE);

	//OpenJaws();
}

void LongArm() {
	ttprintf("LongArm\n")
;
	wantedA = 180;
	wantedE = 180;

	aMove(wantedA);
	eMove(wantedE);

	//OpenJaws();

}

void ArmService() {

}

void topLoadArmService() {
	if (topLoadTrip) {
		//float bActPos = getBinLiftAct();
		/*
		 //if (bActPos > 50)
		 {
		 wantedA = 233;
		 wantedE = 315;

		 aMove(wantedA);
		 eMove(wantedE);

		 topLoadTrip = 0;
		 }
		 */
	} else {

	}
}

void topLoadArm() {

	wantedA = 0;
	wantedE = 0;

	topLoadTrip = 1;
}

void midLoadArmWrap() {
	wantedA = 233;
	wantedE = 315;

	aMove(wantedA);
	eMove(wantedE);
}

//--- move to mid location to load bin on tote lift rails

void midLoadArm() {
	wantedA = 0;
	wantedE = 0;

	bMove(24, midLoadArmWrap);
}

//--- Move Bin to load location over totes

void binLoadArm() {
	ttprintf("binLoadArm\n")
;
	wantedA = 233;
	wantedE = 315;

	aMove(wantedA);
	eMove(wantedE);

}

//----------------------------
//---------- ATON 4 ----------
//----------------------------

void a4Finisheded(tI);

void a4_JawsAreNowClosed(tI);
void a4_BinIsUp(tI);
void a4_DropToteStack(tI);
void a4Finished(tI);

void a4_ShootTheBall(tI);

//#1
void a4_ShootTheBall(tI);
void a4_FindTheGoal(tI);

//#1
/*
 void aton4()
 {
 ttprintf("#1 Aton4\n");

 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

 atonActive = 1;

 autoDriveY = 1.0;
 autoDriveX = 0;
 autoDriveR = 0;

 WaitFor(a4_MoveTime1, a4_FindTheGoal);

 }

 void MoveElevToHigh();

 extern int gTimeOut;

 extern float gyroZero;

 void a4_TrackGoal(tI);

 void MoveShoeToLow();

 void a4_FindTheGoal(tI)
 {

 autoDriveY = 0.0;
 autoDriveX = 0;
 autoDriveR = 0;

 //Set_ShooterWheel(1.0);

 //RaiseWheels();

 //MoveElevToHigh();

 if (a4_ShotDist == 1) MoveElevToMid2(); else
 if (a4_ShotDist == 2) MoveElevToMid(); else
 MoveElevToHigh();


 MoveShoeToLow();

 gTimeOut = -1000;

 //gyroZero+=a4_Rotate;

 xDrift = 0;
 xDriftCount = 0;

 if (fabs(a4_Rotate) > 1) gyroZero += a4_Rotate;

 else { xDrift = a4_Rotate; xDriftCount = 100; RaiseWheels(); }


 startTimer(3, aShootTime - a4_MoveTime1 * 100, a4_TrackGoal, a4_ShootTheBall);

 //need to rotate using gyro

 }

 void TrackGoal2();

 void a4_TrackGoal(tI)
 {

 LookForGoal = 1;


 //if (a4_ShotDist == 1) TrackGoal4(); else
 //if (a4_ShotDist == 2) TrackGoal3(); else
 //TrackGoal2();


 }

 void StopTheBall(unsigned char i);

 void a4_ShootTheBall(tI)
 {

 if (1) //GoalFound)
 {
 //Set_BallFeed(1.0);
 }

 //startTimer(1, 50, 0, StopTheBall);

 autoDriveY = 0;
 autoDriveX = 0;
 autoDriveR = 0;

 LookForGoal = 0;

 //GoalFound=0;

 GoalH = 0;
 GoalW = 0;
 GoalX = 160;
 GoalY = 120;

 autoDriveR = 0;
 autoDriveY = 0;
 atonActive = 0;


 if (1) //GoalFound)
 {

 //startTimer(1, 100, 0, RotateToDefences);
 }

 GoalFound = 0;
 }

 void a4_JawsAreNowClosed(tI)
 {
 bMove(a4BinLevel);

 //tMove(1);

 WaitFor(2.0, a4_BinIsUp);
 }

 void a4_BinIsUp(tI)
 {

 ttprintf("Move forward\n");

 autoDriveY = 0;
 autoDriveX = a4DriveSpeed;

 //startTimer(3, a4DriveTime * 100, 0, a4_DropToteStack);

 }

 void a4_DropToteStack(tI)
 {

 //Spit();

 autoDriveY = -1.0;
 autoDriveX = 0.0;

 //WaitFor(1.0, a4Finished);
 }

 void a4Finished(tI)
 {
 StopAllThings();
 }



 */

void copFinished(tI);

void CopAction() {
	//Cop(0.8) ;

	WaitFor(5, copFinished);

}
//
void CopStop() {

	//Cop(0.0);

}

void copFinished(tI) {
	//Cop(0.0);
}

//-------------------------------------------------------------
//
//  Load Tote auto
//
//

void IamDown();
void IamUp();
void IamDownService(tI);
void IamDownFault(tI);

void LoadOnceCycle() {
	tMove(2, IamDown, 5);

	WaitFor(2, IamDownService, IamDownFault);
}

void IamDownService(tI) {

}

void IamDownFault(tI) {

	stopTimer(3);

}

void IamDown() {

	tMove(22, IamUp, 5);

}

void IamUp() {

}

//------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

void JawsAreNowClosed_a3(tI);
void WatchForTote2WithLidar_a3(tI);
void PickupTote2_a3(tI);
void ToteAndBinUp_a3(tI);
void liftTote2_a3(tI);
void hitFinished_a3();
void StartMoveingToTote3_a3(tI);
void GrabBin3_a3(tI);
void PickupThirdTote_a3(tI);
void PickupThirdToteB_a3();
void PickupTote2Lift_a3(tI);
void MoveToCenter_a3(tI);
void MoveToCenter_a3(tI);
void DropToteStack_a3(tI);
void a3Finished(tI);
void TotesAreDown_a3();

//----------------------------
//---------- ATON 3 ----------
//----------------------------

/*

 float lidarSucks=1;

 void SuckSlow();

 void aton3()
 {
 tprintf("Aton3\n");

 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

 ttprintf("Lift Tote and Can\n");

 if (loc[pwmToteLift1]<0) zeroToteLift();

 if (loc[pwmBinLift1]<0) zeroBinLift();

 atonActive = 1;

 CloseJaws();

 SuckSlow();

 WaitFor(0.50, JawsAreNowClosed_a3);
 }

 //#2
 void JawsAreNowClosed_a3(tI)
 {
 ttprintf("#2 JawsAreNowClosed\n");

 autoDriveY = a2ForwardMoveSpeedB2;

 bMove(28 + 2+6+2);
 tMove(17);
 aMove(180+45);

 WaitFor(2.0, ToteAndBinUp_a3);
 }

 void ToteAndBinUp_a3(tI)
 {
 WaitFor(4.5 - 2.0, WatchForTote2WithLidar_a3, PickupTote2_a3);
 }

 void WatchForTote2WithLidar_a3(tI)
 {

 if (lidarSucks) return;

 if (getToteLiftAct()>12 && lidarValue<12) tripTimer(3);
 }

 void PickupTote2_a3(tI)
 {
 ttprintf("PickkupTote2\n");

 autoDriveY = 0.0;

 //SuckStop();

 SpitRight();

 tMove(0);

 WaitFor(1.0, liftTote2_a3);
 }

 void liftTote2_a3(tI)
 {
 tMove(17);

 SpitRight();

 WaitFor(1.0, StartMoveingToTote3_a3);
 }

 void WatchForTote3WithLidar_a3(tI)
 {
 if (lidarSucks) return;

 if (lidarValue<6) tripTimer(3);
 if (lidarValue<40) Suck();
 }

 void StartSuckIt_a3(tI)
 {
 SuckSlow();
 }

 void StartLidarWatch_a3(tI);

 void StartMoveingToTote3_a3(tI)
 {
 autoDriveY = a2ForwardMoveSpeedB2;

 //WaitWith(2.0,StartSuckIt);

 WaitFor(2.0, 0, StartLidarWatch_a3);
 }

 void StartLidarWatch_a3(tI)
 {
 Suck();

 WaitFor(2.0, WatchForTote3WithLidar_a3, PickupThirdTote_a3);
 }

 void PickupThirdTote_a3(tI)
 {
 autoDriveY = -0.2;

 SpitRight();

 autoDriveX = a2RightMoveSpeedB3;

 WaitFor(1.0, MoveToCenter_a3);
 }

 //#12
 void MoveToCenter_a3(tI)
 {
 ttprintf("MoveToCenter\n");

 autoDriveY = 0;
 autoDriveX = a2RightMoveSpeedB3;

 eMove(150);
 aMove(270);

 WaitFor(a2RightMoveTimeB3 - 1, DropToteStack_a3);
 }


 //#13
 void DropToteStack_a3(tI)
 {
 ttprintf("DropToteStack\n");

 Spit();

 tMove(0, TotesAreDown_a3);
 }

 void TotesAreDown_a3()
 {
 autoDriveY = -1.0;
 autoDriveX = 0.0;

 WaitFor(1.0, a3Finished);
 }

 //#14



 void a3Finished(tI)
 {
 ttprintf("a3Finished\n");

 //StopAllThings();
 autoDriveY = 0.0;
 autoDriveX = 0.0;


 bMove(19);
 aMove(246);
 eMove(286);

 SuckStop();



 }

 void BinToCenter()
 {


 aMove(246);
 eMove(286);


 }

 */

//----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

//----------------------------
//---------- ATON 5 ----------
//----------------------------
void JawsAreNowClosed_a5(tI);

void azmFinished();
void a5NextStep();

void a5_ShootTheBall(tI);

//#1
void a5_ShootTheBall(tI);
void a5_FindTheGoal(tI);

//#1
/*
 void aton5()
 {
 ttprintf("#1 Aton5\n");

 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

 atonActive = 1;

 autoDriveY = 1.0;
 autoDriveX = 0;
 autoDriveR = 0;

 WaitFor(a5_MoveTime1, a5_FindTheGoal);

 }

 void MoveElevToHigh(){};

 extern int gTimeOut;

 extern float gyroZero;

 void a5_TrackGoal(tI);

 void MoveShoeToLow(){};

 void a5_FindTheGoal(tI)
 {

 autoDriveY = 0.0;
 autoDriveX = 0;
 autoDriveR = 0;

 //Set_ShooterWheel(1.0);

 RaiseWheels();

 //MoveElevToHigh();

 if (a5_ShotDist == 1) MoveElevToMid2(); else
 if (a5_ShotDist == 2) MoveElevToMid(); else
 MoveElevToHigh();


 MoveShoeToLow();

 gTimeOut = -1000;

 // gyroZero+=a5_Rotate;

 xDrift = 0;
 xDriftCount = 0;

 if (fabs(a5_Rotate) > 1) gyroZero += a5_Rotate;

 else { xDrift = a5_Rotate; xDriftCount = 100; RaiseWheels(); }


 //startTimer(3, aShootTime - a5_MoveTime1 * 100, a5_TrackGoal, a5_ShootTheBall);

 //need to rotate using gyro

 }

 void TrackGoal2();

 void a5_TrackGoal(tI)
 {

 LookForGoal = 1;


 if (a5_ShotDist == 1) TrackGoal4(); else
 if (a5_ShotDist == 2) TrackGoal3(); else
 //TrackGoal2();


 }
 */

void StopTheBall(unsigned char i);

void a5_ShootTheBall(tI) {

	if (1) //GoalFound)
	{
		//Set_BallFeed(1.0);
	}

	//startTimer(1, 50, 0, StopTheBall);

	autoDriveY = 0;
	autoDriveX = 0;
	autoDriveR = 0;

	LookForGoal = 0;

	//	GoalFound=0;

	GoalH = 0;
	GoalW = 0;
	GoalX = 160;
	GoalY = 120;

	autoDriveR = 0;
	autoDriveY = 0;

	atonActive = 0;

	if (1) //GoalFound)
	{

		//startTimer(1, 100, 0, RotateToDefences);
	}

	GoalFound = 0;
}

void xxaton5() {
	tprintf("Aton5\n");

	if (inDisable) {
		tprintf("inDisable (Abort)\n");
		return;
	}

	atonActive = 1;

	//OpenJaws();

	bMove(12);

	aMove(220, a5NextStep);
	eMove(128);

}

void a5NextStep() {

	aMove(a5Amz, azmFinished);
	eMove(a5Elbo);

}

void GrabTheBin(tI);

void azmFinished() {
	autoDriveX = 0.7;

	WaitFor(0.50, GrabTheBin);

}

void GrabTheBin(tI) {

	autoDriveX = 0.0;

	//CloseJaws();

	WaitFor(0.50, JawsAreNowClosed_a5);

}

void NowInCenter(tI);

void binIsHigh();

void JawsAreNowClosed_a5(tI) {
	ttprintf("JawsAreNowClosed\n")
;
	bMove(35, binIsHigh);

}

void binIsHigh() {

	aMove(250);
	eMove(270);

	autoDriveX = -1.0;

	WaitFor(2.0, NowInCenter);

	//autoDriveY = a2ForwardMoveSpeedB2;

	//bMove(28 + 2+6+2);
	//tMove(17);
	//aMove(180+45);

	//WaitFor(2.0, ToteAndBinUp_a3);
}

void NowInCenter(tI) {

	autoDriveX = 0;

}

bool MoveTo(double NewLoc, double NewRate, double NewAccel, double NewJerk,
		double MaxTime);

//--------------------------------------------------------------
//--------------------  tote profile commands ------------------
//--------------------------------------------------------------

float tCmdLoc = 0;
float tBeginLoc = 0;
float tEndLoc = 0;
float tTime = 0;

float tVel = 0;
//float tAccel=0;

void tMoveP(float dest, float speed, float accel, void (*finished)(),
		float maxTime) {

	tMoveFinishedAddress = finished;

	MoveTo(dest, speed, accel, 100.0 * 2.75, maxTime);

}

#define CARD int
#define REAL double
#define BOOL bool
#define bPROCEDURE bool
//#define TRUE true
//#define FALSE false
#define BEGIN {
#define END   }
#define IF if
#define AND &&
#define OR ||
#define PROCEDURE void
#define ELSE }else{
#define THEN {
#define RETURN return
#define Null 0
#define sPROCEDURE static void
#define rPROCEDURE double

PROCEDURE SetupMove(CARD Axis);

int MaxInt = 10;
bool ActiveFlag[20] = { false, false, false, false, false };

int RapidFlag[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float RapidVel[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float RapidVelz[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float FeedScale[20] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
float MaxJerk[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float MaxAccel[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float MaxVel[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float FinalPos[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float CmdPos[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float CurVell[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float CurVel[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float BeginMove[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float DeltaMove[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

float MaxTimeLimit[20] = { 0, 0, 0, 0, 0, 0, 0, 0 };

float mTime[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float wTime[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float mLock[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float MoveRamp[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

bool ProfileAct[20] = { false, false, false, false, false };

bool OKK = false;

bool AxisActive[20] = { false, false, false, false, false };

bool ProfileEndFlag[20] = { false, false, false, false, false };

PROCEDURE StopBackGround(CARD Which);

double TaperRamp1(double mTime, double Ramp);

PROCEDURE StartMove(CARD Axis);

PROCEDURE StartBackGround(CARD Which, void (*Where)(tI), CARD Rate);

sPROCEDURE nMoveForward(tI); //CARD ServiceNumber);

float getToteLiftAct();

bPROCEDURE MoveToxx(REAL NewLoc, REAL NewRate, REAL NewAccel, REAL NewJerk,
		REAL MaxTime)
BEGIN

	int Axis = 0;

//CmdPos[Axis] = getToteLiftAct();

	if (NewLoc < CmdPos[Axis])
		CmdPos[Axis] += 2;
	else
		CmdPos[Axis] -= 2;

	StartBackGround(Axis, 0, 0);

	CurVell[Axis] = 0.0;
	CurVel[Axis] = 0.0;
	MaxVel[Axis] = 0.0;
	MaxAccel[Axis] = 0.0;
	ProfileAct[Axis] = FALSEE;
	ProfileEndFlag[Axis] = FALSEE;
	AxisActive[Axis] = TRUEE;
	mTime[Axis] = 0.0;
	wTime[Axis] = 0.0;
	mLock[Axis] = -1;

	MaxTimeLimit[Axis] = MaxTime;

	MaxJerk[Axis] = NewJerk;
	MaxAccel[Axis] = NewAccel;
	MaxVel[Axis] = NewRate;
	FinalPos[Axis] = NewLoc;

	IF(CmdPos[Axis] != NewLoc) THEN

				StartMove(Axis);

			ELSE

				AxisActive[Axis] = FALSEE;

				ProfileEndFlag[Axis] = TRUEE;

				StopBackGround(Axis);

				if (tMoveFinishedAddress)
					tMoveFinishedAddress();

			END;

			RETURN TRUEE;

		END
		;
		/* MoveTo */

		PROCEDURE StartMove(CARD Axis)
		BEGIN

			SetupMove(Axis);

			StartBackGround(Axis, nMoveForward, 0);

		END
		;

		PROCEDURE StopBackGround(CARD Which)
		BEGIN

		IF(	(Which >= 0) AND (Which < MaxInt)
						) ActiveFlag[Which] = FALSEE;

					stopTimer(13);

				END
				;

				PROCEDURE StartBackGround(CARD Which, void (*Where)(tI),
						CARD Rate)
				BEGIN

//Schedule(Which, Where);

					startTimer(13, MaxTimeLimit[Which] * 100, Where, 0);

				END
				;

#define AxisCount 9

				REAL ccMoveTime = 10.0;

//! REAL Ratiox = 10.0;

				REAL mVelMax[AxisCount + 1], mAccelMax[AxisCount + 1],

				mVel[AxisCount + 1], mAccel[AxisCount + 1],
						mJerk[AxisCount + 1];

//    MoveSetPointA[AxisCount+1],
//    MoveSetPoint[AxisCount+1];

				PROCEDURE SetupMove(CARD Axis)
				BEGIN

//REAL AccelTime = 0.0;

					ProfileEndFlag[Axis] = FALSEE;

					mTime[Axis] = 0.0;
					mVel[Axis] = 0.0;
					mAccel[Axis] = 0.0;

					BeginMove[Axis] = CmdPos[Axis];

					DeltaMove[Axis] = FinalPos[Axis] - BeginMove[Axis];

					mVelMax[Axis] = 0.1;

					REAL TotalDist = fabs(DeltaMove[Axis]);

// SendBack(360,TotalDist);

					REAL MaxV = MaxVel[Axis];
					IF(MaxV < 0.000001
								) MaxV = 0.000001;
							REAL Accel = MaxAccel[Axis];
							IF(Accel < 0.000001
										) Accel = 0.000001;

									REAL RampTime = MaxV / Accel;

//SendBack(361,RampDist);

									REAL RampDist = RampTime * MaxV / 2.0;

									IF(RampDist > TotalDist / 2.0) THEN

												RampTime = sqrt(
														TotalDist * Accel)
														/ Accel;

												RampDist = TotalDist / 2.0;

											END;

											REAL DistLeft = TotalDist / 2.0
													- RampDist;

											REAL TimeLeft = DistLeft / MaxV;

											REAL TotalTime = 2.0
													* (RampTime + TimeLeft);

											mVelMax[Axis] = 1.0 / (TotalTime)
													* 0.01;

											MoveRamp[Axis] = RampDist
													/ TotalDist;

											IF(MoveRamp[Axis] < 0.5) THEN

														ccMoveTime = TotalDist
																/ MaxV;

														mVelMax[Axis] = 1.0
																/ ccMoveTime
																* 0.01;

														REAL Rtmp =
																1.0
																		+ MoveRamp[Axis];

														mVelMax[Axis] =
																mVelMax[Axis]
																		/ Rtmp;

														MoveRamp[Axis] =
																MoveRamp[Axis]
																		/ Rtmp;

													END;

												END
												;
												// SetupMove

												PROCEDURE Tick(REAL Time,
														CARD ServiceNumber)
												BEGIN

													CmdPos[ServiceNumber] =
															BeginMove[ServiceNumber]
																	+ DeltaMove[ServiceNumber]
																			* Time;

												END
												;

												sPROCEDURE nMoveForward(tI)
												BEGIN

												CARD ServiceNumber = 0;

													CARD LockIndex =
															mLock[ServiceNumber];

													IF(LockIndex >= 0) THEN

																wTime[ServiceNumber] =
																		wTime[LockIndex];

															ELSE

																wTime[ServiceNumber] =
																		TaperRamp1(
																				mTime[ServiceNumber],
																				MoveRamp[ServiceNumber]);

															END;

															Tick(
																	wTime[ServiceNumber],
																	ServiceNumber);

															IF(mTime[ServiceNumber]
																			>= 1.0) THEN

																		AxisActive[ServiceNumber] =
																				FALSEE;

																		ProfileEndFlag[ServiceNumber] =
																				TRUEE;

																		StopBackGround(
																				ServiceNumber);

																		if (tMoveFinishedAddress)
																			tMoveFinishedAddress();

																		mLock[ServiceNumber] =
																				-1;

																		RETURN;

																	END;

																	IF(LockIndex
																					>= 0) THEN

																				mTime[ServiceNumber] =
																						mTime[LockIndex];

																			ELSE

																				mTime[ServiceNumber] =
																						mTime[ServiceNumber]
																								+ mVelMax[ServiceNumber]; // * FeedScale[ServiceNumber] * FeedHoldScale;

																			END;

																			IF(mTime[ServiceNumber]
																							> 1.0) THEN mTime[ServiceNumber] =
																								1.0;
																					END;

																				END
																				;
																				// MoveForward

																				rPROCEDURE TaperRamp1(
																						REAL mTime,
																						REAL Ramp)
																				BEGIN

																				REAL RampRatio =
																							1.0
																									/ (1.0
																											- Ramp);

																					IF(mTime
																									< Ramp) THEN

																							RETURN(mTime
																										* mTime
																										/ Ramp)
																										/ 2.0
																										* RampRatio;

																							END;

																							IF(mTime
																											> (1.0
																													- Ramp)) THEN

																									RETURN((1.0
																												- Ramp)
																												- (1.0
																														- mTime)
																														* (1.0
																																- mTime)
																														/ Ramp
																														/ 2.0)
																												* RampRatio;

																									END;

																									RETURN(Ramp
																											/ 2.0
																											+ (mTime
																													- Ramp))
																											* RampRatio;

																								END
																								;
																								// TaperRamp

																								void BinToCenter() {
																									aMove(
																											246);
																									eMove(
																											286);
																								}

//----------------------------
//---------- ATON 3 ----------
//----------------------------

																								void SuckFast();
																								void SuckSlow();

																								float a3DriveTime =
																										3.75;

																								int brokenWing =
																										1;

																								int armDown =
																										0;

																								void aton3Core() {
																									/*
																									 tprintf("Aton3\n");

																									 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

																									 if (loc[pwmToteLift1]<0) zeroToteLift();
																									 if (loc[pwmBinLift1 ]<0) zeroBinLift();

																									 atonActive = 1;

																									 CloseJaws();

																									 //SuckSlow();

																									 //if (brokenWing)

																									 //SpitRight();

																									 WaitFor(0.50, JawsAreNowClosed_a3);
																									 */
																								}

																								void a3_ShootTheBall(
																										tI);
																								void a3_FindTheGoal(
																										tI);

//#1
																								/*
																								 void aton3()
																								 {
																								 ttprintf("#1 Aton3\n");

																								 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

																								 atonActive = 1;

																								 autoDriveY = 1.0;
																								 autoDriveX = 0;
																								 autoDriveR = 0;

																								 WaitFor(a3_MoveTime1, a3_FindTheGoal);

																								 }
																								 */
																								/*
																								 void MoveElevToHigh();

																								 extern int gTimeOut;

																								 extern float gyroZero;

																								 void a3_TrackGoal(tI);

																								 void MoveShoeToLow();

																								 void a3_FindTheGoal(tI)
																								 {

																								 autoDriveY = 0.0;
																								 autoDriveX = 0;
																								 autoDriveR = 0;

																								 RaiseWheels();

																								 //Set_ShooterWheel(1.0);

																								 //MoveElevToHigh();

																								 if (a3_ShotDist == 1) MoveElevToMid2(); else
																								 if (a3_ShotDist == 2) MoveElevToMid(); else
																								 //MoveElevToHigh();


																								 //MoveShoeToLow();

																								 gTimeOut = -1000;

																								 //gyroZero+=a3_Rotate;

																								 xDrift = 0;
																								 xDriftCount = 0;

																								 if (fabs(a3_Rotate) > 1) gyroZero += a3_Rotate;

																								 else { xDrift = a3_Rotate; xDriftCount = 100; RaiseWheels(); }

																								 //startTimer(3, aShootTime - a3_MoveTime1 * 100, a3_TrackGoal, a3_ShootTheBall);

																								 //need to rotate using gyro

																								 }
																								 */
																								void TrackGoal2();
																								/*
																								 void a3_TrackGoal(tI)
																								 {

																								 LookForGoal = 1;


																								 if (a3_ShotDist == 1) TrackGoal4(); else
																								 if (a3_ShotDist == 2) TrackGoal3(); else
																								 //TrackGoal2();


																								 }
																								 */
																								void StopTheBall(
																										unsigned char i);

																								void a3_ShootTheBall(
																										tI) {

																									if (1) //GoalFound)
																									{
																										//Set_BallFeed(1.0);
																									}

																									//startTimer(1, 50, 0, StopTheBall);

																									autoDriveY =
																											0;
																									autoDriveX =
																											0;
																									autoDriveR =
																											0;

																									LookForGoal =
																											0;

																									//GoalFound=0;

																									GoalH =
																											0;
																									GoalW =
																											0;
																									GoalX =
																											160;
																									GoalY =
																											120;

																									autoDriveR =
																											0;
																									autoDriveY =
																											0;

																									atonActive =
																											0;

																									if (1) //GoalFound)
																									{

																										//startTimer(1, 100, 0, RotateToDefences);
																									}

																									GoalFound =
																											0;
																								}

																								/*
																								 //----------------------------
																								 //---------- ATON 6 ----------
																								 //----------------------------

																								 void a6_ShootTheBall(tI);
																								 void a6_FindTheGoal(tI);

																								 //#1
																								 void aton6()
																								 {
																								 ttprintf("#1 Aton6\n");

																								 if (inDisable) { tprintf("inDisable (Abort)\n"); return; }

																								 atonActive = 1;

																								 autoDriveY = 1.0;
																								 autoDriveX = 0;
																								 autoDriveR = 0;

																								 ////WaitFor(a6_MoveTime1, a6_FindTheGoal);

																								 }

																								 void MoveElevToHigh();

																								 extern int gTimeOut;

																								 extern float gyroZero;

																								 void a6_TrackGoal(tI);

																								 void MoveShoeToLow();

																								 void a6_FindTheGoal(tI)
																								 {

																								 autoDriveY = 0.0;
																								 autoDriveX = 0;
																								 autoDriveR = 0;

																								 RaiseWheels();

																								 //Set_ShooterWheel(1.0);

																								 if (a6_ShotDist == 1) MoveElevToMid2(); else
																								 if (a6_ShotDist == 2) MoveElevToMid(); else
																								 MoveElevToHigh();

																								 MoveShoeToLow();

																								 gTimeOut = -1000;

																								 //	gyroZero+=a6_Rotate;

																								 xDrift = 0;
																								 xDriftCount = 0;

																								 if (fabs(a6_Rotate) > 1) gyroZero += a6_Rotate;

																								 else { xDrift = a6_Rotate; xDriftCount = 100; RaiseWheels(); }


																								 startTimer(3, aShootTime - a6_MoveTime1 * 100, a6_TrackGoal, a6_ShootTheBall);

																								 //need to rotate using gyro

																								 }

																								 void TrackGoal2();

																								 void a6_TrackGoal(tI)
																								 {
																								 LookForGoal = 1;

																								 //if (a6_ShotDist == 1) TrackGoal4(); else
																								 //	if (a6_ShotDist == 2) TrackGoal3(); else
																								 //	TrackGoal2();
																								 }
																								 */

																								void StopTheBall(
																										unsigned char i);

																								void a6_ShootTheBall(
																										tI) {

																									if (1) //GoalFound)
																									{
																										//Set_BallFeed(1.0);
																									}

																									//startTimer(1, 50, 0, StopTheBall);

																									autoDriveY =
																											0;
																									autoDriveX =
																											0;
																									autoDriveR =
																											0;

																									LookForGoal =
																											0;

																									GoalH =
																											0;
																									GoalW =
																											0;
																									GoalX =
																											160;
																									GoalY =
																											120;

																									autoDriveR =
																											0;
																									autoDriveY =
																											0;

																									atonActive =
																											0;

																									if (1) //GoalFound)
																									{

																										//startTimer(1, 100, 0, RotateToDefences);
																									}

																									GoalFound =
																											0;

																								}
																								void xxaton6() {
																									//tprintf("Aton6\n");

																									//armDown=1;

																									//aton3Core();
																								}

																								void StartSuckItBW(
																										tI) {

																									//	 SuckSlow();

																									//	tMove(17);

																								}

//#2
																								void JawsAreNowClosed_a3(
																										tI) {
																									autoDriveY =
																											a2ForwardMoveSpeedB2;

																									bMove(
																											28
																													+ 10
																													+ 12);

																									tMove(
																											17);

																									WaitFor(
																											a3DriveTime,
																											PickupTote2_a3);

																									WaitWith(
																											1.5,
																											StartSuckItBW);
																								}

																								void PickupTote2_a3(
																										tI) {
																									autoDriveY =
																											0.0;

																									//	SpitRight();

																									tMove(
																											0);

																									WaitFor(
																											1.0,
																											liftTote2_a3);
																								}

																								void liftTote2_a3(
																										tI) {
																									//tMove(17);

																									tMove(
																											17);

																									WaitFor(
																											1.0,
																											StartMoveingToTote3_a3);
																								}

																								void SwitchToSuck(
																										tI);

																								void StartMoveingToTote3_a3(
																										tI) {
																									autoDriveY =
																											a2ForwardMoveSpeedB2;

																									WaitFor(
																											1.5,
																											0,
																											SwitchToSuck);
																								}

																								void SwitchToSuck(
																										tI) {
																									//	Suck();

																									tMove(
																											17);

																									WaitFor(
																											a3DriveTime
																													- 1.5,
																											0,
																											PickupThirdTote_a3);
																								}

																								extern float binDropLoc;

																								void PickupThirdTote_a3(
																										tI) {
																									autoDriveY =
																											-0.2;

																									autoDriveX =
																											a2RightMoveSpeedB3;

																									//	SuckFast();

																									//if (binDropLoc) OpenJaws();

																									WaitFor(
																											1.0,
																											MoveToCenter_a3);
																								}

//#12
																								void MoveToCenter_a3(
																										tI) {
																									autoDriveY =
																											0;
																									autoDriveX =
																											a2RightMoveSpeedB3;

																									//eMove(150);
																									//aMove(270);

																									if (binDropLoc) {
																										eMove(
																												180);
																										aMove(
																												180);
																										bMove(
																												12);
																									}

																									WaitFor(
																											a2RightMoveTimeB3
																													- 1,
																											DropToteStack_a3);
																								}

																								void DropToteStack_a3(
																										tI) {
																									Spit();

																									tMove(
																											0,
																											TotesAreDown_a3);

																									//if (armDown) { OpenJaws(); bMove(12); }

																								}

																								void TotesAreDown_a3() {
																									autoDriveY =
																											-1.0;
																									autoDriveX =
																											0.0;

																									WaitFor(
																											1.0,
																											a3Finished);
																								}

																								void a3Finished(
																										tI) {
																									autoDriveY =
																											0.0;
																									autoDriveX =
																											0.0;

																									//bMove(19);
																									aMove(
																											246);
																									eMove(
																											286);

																									//	SuckStop();
																								}

//----------------- motion profile

																								int
																										ACC_RAMP_UP =
																												0,
																										CONSTANT_POSITIVE_ACC =
																												1,
																										ACC_MIRROR =
																												2,
																										CONSTANT_VELOCITY =
																												3;

																								static double acceleration[10000];
																								static double jerk[10000];
																								static double position[10000];
																								static double velocity[10000];
																								static int p_ptr =
																										0;

																								int maxElement =
																										0;

																								static double maxTime =
																										10.0;
																								static double dt =
																										0.025;

																								void generate(
																										double displacement,
																										double max_velocity,
																										double max_acceleration,
																										double max_jerk) {
																									//the profile will be a mirror image, so we should find the halfway point to know where to stop
																									double midpoint =
																											displacement
																													/ 2;
																									//also need to know the velocity midpoint
																									double velocityMidpoint =
																											max_velocity
																													/ 2;

																									int atMidpoint =
																											false;
																									int stage =
																											0;
																									int j =
																											0; //mirror counter
																									int jSet =
																											false;

																									int p =
																											0;

																									acceleration[0] =
																											0;
																									jerk[0] =
																											0;
																									position[0] =
																											0;
																									velocity[0] =
																											0;

																									p_ptr =
																											1;

																									//using max time, calculate the max number of times the generator should run before giving up.
																									int maxSteps =
																											(int) (maxTime
																													/ dt);

																									//loop for generating profile
																									for (int i =
																											0;
																											i
																													< maxSteps;
																											i++) {
																										//check to see if we've gotten to the midpoint
																										if (position[i]
																												> midpoint) {
																											//if so, remember that we got to the midpoint
																											atMidpoint =
																													true;
																											//and exit the loop
																											break;
																										}

																										//once our acceleration reaches maximum, move on
																										if (acceleration[i]
																												>= max_acceleration) {
																											stage =
																													CONSTANT_POSITIVE_ACC;
																										}

																										//one our velocity reaches halfway, move on
																										if (velocity[i]
																												>= velocityMidpoint) {
																											stage =
																													ACC_MIRROR;
																											//configure mirror counter
																											if (!jSet) {
																												j =
																														i;
																												jSet =
																														true;
																											}

																										}

																										//once we've finished mirroring the acc. trapezoid, move on.
																										if (j
																												< 0) {
																											stage =
																													CONSTANT_VELOCITY;
																										}
																										//

																										//			if(p.get(i).velocity >= max_velocity){
																										//				stage = CONSTANT_VELOCITY;
																										//			}

																										//create empty element to add

																										//check to see if we're in the ramp up acc. stage
																										if (stage
																												== ACC_RAMP_UP) {
																											//if so, do math
																											jerk[i
																													+ 1] =
																													max_jerk;
																											acceleration[i
																													+ 1] =
																													acceleration[i]
																															+ max_jerk
																																	* dt;
																											velocity[i
																													+ 1] =
																													velocity[i]
																															+ acceleration[i
																																	+ 1]
																																	* dt;
																											position[i
																													+ 1] =
																													position[i]
																															+ velocity[i
																																	+ 1]
																																	* dt;
																										}
																										//check to see if we're in the positive, but constant, acc. stage
																										if (stage
																												== CONSTANT_POSITIVE_ACC) {
																											//if so, do math
																											jerk[i
																													+ 1] =
																													0;
																											acceleration[i
																													+ 1] =
																													max_acceleration;
																											velocity[i
																													+ 1] =
																													velocity[i]
																															+ acceleration[i
																																	+ 1]
																																	* dt;
																											position[i
																													+ 1] =
																													position[i]
																															+ velocity[i
																																	+ 1]
																																	* dt;
																										}

																										//check to see if we're in the process of mirroring acc.
																										if (stage
																												== ACC_MIRROR) {
																											//if so do math
																											jerk[i
																													+ 1] =
																													-fabs(
																															jerk[i]);
																											acceleration[i
																													+ 1] =
																													acceleration[i]
																															+ jerk[i
																																	+ 1]
																																	* dt;
																											velocity[i
																													+ 1] =
																													velocity[i]
																															+ acceleration[i
																																	+ 1]
																																	* dt;
																											position[i
																													+ 1] =
																													position[i]
																															+ velocity[i
																																	+ 1]
																																	* dt;
																											//decrement mirror counter
																											j--;
																											//System.out.println(j + "    " + -p.get(j).jerk);
																										}
																										//check to see if we're in constant velocity
																										if (stage
																												== CONSTANT_VELOCITY) {
																											//if so, do math
																											jerk[i
																													+ 1] =
																													0;
																											acceleration[i
																													+ 1] =
																													0;
																											velocity[i
																													+ 1] =
																													velocity[i];
																											position[i
																													+ 1] =
																													position[i]
																															+ velocity[i
																																	+ 1]
																																	* dt;
																										}

																										p =
																												i
																														+ 1;
																									}

																									//if we break from the loop and we haven't reached the midpoint, something really has gone wrong.
																									if (!atMidpoint) {
																										// ("Profile Generation Failed!");
																										return;
																									}
																									//mirror everything else

																									p--;

																									int size =
																											p
																													+ 1;

																									maxElement =
																											0;

																									for (int i =
																											0;
																											i
																													< p;
																											i++) {

																										acceleration[size
																												+ i] =
																												-acceleration[size
																														- i
																														- 1];
																										velocity[size
																												+ i] =
																												velocity[size
																														- i
																														- 1];
																										position[size
																												+ i] =
																												position[size
																														- 1]
																														+ (position[size
																																- 1]
																																- position[size
																																		- i
																																		- 2]);

																										maxElement =
																												size
																														+ i;

																										if (position[size
																												+ i]
																												>= displacement) {
																											position[size
																													+ i] =
																													displacement;
																											break;
																										}

																									}

																									position[maxElement] =
																											displacement;

																									return;
																								}

																								void testProfile() {
																									generate(
																											100.0,
																											10,
																											100,
																											100);
																								}

																								int trackBeg =
																										0;
																								int trackEnd =
																										0;
																								int trackIndex =
																										0;

																								double tBegLoc =
																										0;

																								void trackFinished(
																										tI) {

																									stopTimer(
																											13);

																									// tCmdPos=0;

																									if (tMoveFinishedAddress)
																										tMoveFinishedAddress();

																								}

																								int trackDown =
																										0;

																								sPROCEDURE trackProfile(
																										tI) {

																									//tCmdPos=tBegLoc+position[trackEnd];

																									//return;

																									trackIndex++;

																									if (trackIndex
																											>= trackEnd) {
																										trackFinished(
																												13);
																									}

																								}

																								bPROCEDURE MoveTo(
																										REAL NewLoc,
																										REAL NewRate,
																										REAL NewAccel,
																										REAL NewJerk,
																										REAL MaxTime)
																								BEGIN

//tBegLoc = getToteLiftAct();

																									double dist =
																											NewLoc
																													- tBegLoc;

																									generate(
																											fabs(
																													dist),
																											NewRate,
																											NewAccel,
																											NewJerk);

																									if (dist
																											< 0)
																										trackDown =
																												1;
																									else
																										trackDown =
																												0;

//  static double position[10000];

																									// int maxElement = 0;

																									trackBeg =
																											0;
																									trackEnd =
																											maxElement;

																									trackIndex =
																											0;

																									startTimer(
																											13,
																											1000,
																											trackProfile,
																											0);

																									return 0;

																								END
																								;
																								//MoveTo

