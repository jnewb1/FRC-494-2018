#include "UserCode.h"

#include <stdio.h>
//#include <taskLib.h>
//#include <iostream.h>
#include <math.h>
#include "WPILib.h"

#include "JoyStick.H"
#include "itimer.h"
#include "iencoder.h"
#include "motor.h"
#include "aton.h"
#include "driving.h"
//#include "DriverStationLCD.h"
#include "base.h"



//DefineVariables

float move_OverHeadDroppoff = 0;
float TwoCubePickupAngle = 0;
float move_TwoCubeDistance = 0;
float move_CornerToScale = 0;
float strafe_behindHeadDropoff = 0;
float strafe_secondCubePickup = 0;
float strafe_firstCubePickup = 0;
float LiftScaleBehind = 0;
float ToolBarScaleBehind = 0;
float ultra3Distance = 0;
float ultra2Distance = 0;
float ultra1Distance = 0;
float move_CornerDropCube = 0;
float move_CrossDropCube = 0;
float HoldCube = 0;
float TwoCube = 0;
float LiftClimbSide = 0;
float ToolBarClimbSide = 0;
float Crossover = 0;
float LiftClimb = 0;
float ToolBarClimb = 0;
float favorScale = 0;

float ToolBarAngle = 0;

float move_ScalefromBehind = 0;
float move_FaceScale = 0;
float move_SwitchfromBehind = 0;
float move_BehindSwitch = 0;
float move_SideCenterofSwitch = 0;
float move_TouchSwitch = 0;
float move_CenterofSwitch = 0;
float move_ApproachSwitch = 0;



float EncoderGain = -273;
float LiftGain = -128.5;
float ToolBarGain = 58.6;

float ToolBarScaleHigh = 0;
float ToolBarScaleMid = 0;
float ToolBarScaleLow = 0;
float ToolBarSwitch = 0;
float LiftScaleHigh = 0;
float LiftScaleMid = 0;
float LiftScaleLow = 0;
float LiftSwitch = 0;
float LiftLow = 0;
float ToolBarPickup = 80;



float matchNumber = 0;
float matchTimeLeft = 0;
float robotWatts = 0;
float CubeIntakeAngle = 0;
float PegGoalAngle = 0;
float TuningVar5 = 0;
float TuningVar4 = 0;
float TuningVar3 = 0;
float TuningVar2 = 0;
float TuningVar1 = 0;
float LiftDest = 0;
float ToolBarSpeed = 0;
float Lift2Volts = 0;
float Lift1Volts = 0;
float Lift2Amps = 0;
float Lift1Amps = 0;
float Lift2Speed = 0;
float Lift1Speed = 0;
float LiftHeight = 0;
float BatteryVoltage = 0;
float RBD2Amps = 0;
float RBD1Amps = 0;
float RFD2Amps = 0;
float RFD1Amps = 0;
float LBD2Amps = 0;
float LBD1Amps = 0;
float LFD2Amps = 0;
float LFD1Amps = 0;
float LFD1Speed = 0;
float RBD2Speed = 0;
float RBD1Speed = 0;
float LBD2Speed = 0;
float LBD1Speed = 0;
float RFD2Speed = 0;
float RFD1Speed = 0;
float LFD2Speed = 0;
float EncoderVelocity = 0;
float EncoderDistance = 0;


float aton_HopperRPM = 0;
float aton_ShooterRPM = 0;
float atonNum = 1;
float maxGoalAimSpeed = 0.3;
float GoalAimGain = 0.02;
float hopperShootRPM = -140;
float gearDropoffSpeed = 0;
float gearPickupSpeed = 0;
float maxPegAimSpeed = 0.5;
float maxPegDriveSpeed = 0.6;
float pegGoalDriveScale = 0.05;
float pegGoalAimScale = 0.04;
float camerafps = 58;
float PegGoalY1 = 90;
float PegGoalX1 = 150;
float BoilerGoalY3 = 1;
float BoilerGoalY2 = 2;
float BoilerGoalY1 = 185;
float BoilerGoalX3 = 4;
float BoilerGoalX2 = 5;
float BoilerGoalX1 = 160;

float BoilerErrorScale = -0.05;
float cam1dr = 0;
float cam2dr = -17;
float cam2dg = 8;
float cam1dg = 4;
float ShooterShootRPM = 3350;
float HopperShootSpeed = 0;
float ToolBarGearPickup = 0;
float ToolBarGearDropoff = 0;
float PegTargetGot = 0;
float PegGoalY = 0;
float PegGoalX = 0;
float BoilerTargetGot = 0;
float TurretScale = 275;
float TurretPOS = 0;
float HopperRPM = 0;
float HopperPos = 0;
float selectCamera = 1;
float BoilerGoalY = 0;
float BoilerGoalX = 0;
float GyroZ = 0;
float GyroY = 0;
float GyroX = 0;
float HopperSpeed = 0;
float HopperVolts = 0;
float HopperAmps = 0;
float BallIntakeAmps = 0;
float BallIntakeVolts = 0;
float BallIntakeSpeed = 0;
float IntakeRightAmps = 0;
float IntakeLeftVolts = 0;
float IntakeLeftAmps = 0;
float IntakeRightVolts = 0;
float IntakeRightSpeed = 0;
float IntakeLeftSpeed = 0;
float Winch2Volts = 0;
float Winch2Speed = 0;
float Winch2Amps = 0;
float Winch1Amps = 0;
float Winch1Volts = 0;
float Winch1Speed = 0;
float ShooterDest = 0;
float ShooterAmps = 0;
float ShooterVolts = 0;
float ToolBarAmps = 0;
float sendBackValues[420];

float AtonFinalRotate = 180;
float CrossY4 = 100;
float CrossX4 = 100;
float CreapSpeed = 0.16;
float a6_Rotate = 0;
float a5_Rotate = 0;
float a4_Rotate = 0;
float a3_Rotate = 0;
float a2_Rotate = 0;
float a1_Rotate = 0;
float a6_MoveTime1 = 0;
float a5_MoveTime1 = 6000;
float a4_MoveTime1 = -6000;
float a3_MoveTime1 = -5000;
float a2_MoveTime1 = -5000;
float TrackingGainY = 0.5;
float TrackingY = 100;
float TrackingX = 160;
float TrackingGain = 1.0;
float gyro_Zero = 0;
float RotateLeftAngle = 15;
float RotateRightAngle = 15;
float CrossY3 = 160;
float CrossY2 = 160;
float CrossY1 = 160;
float CrossX3 = 160;
float CrossX2 = 160;
float CrossX1 = 160;
float GyroGain = 0.018518518518;
float a1_MoveTime1 = 0;
float ToolBarLoc = 0;
float ToolBarError = 0;
float ToolBarVolts = 0;
float ToolBarDest = 0;
float ToolBarAuto = 0;
float ToolBarScale = -296;
float ToolBarHigh = 0;
float ToolBarMid = 10;
float ToolBarLow = 90;
float GyroLoc = 0;
float UseGyro = 1;
float LookForGoal = 0;


//---------------

//int FindGreenTargets(char* x,int y,int z)
//{
 // printf("FindGreenTargets\n");



//}

//float tableSize=0;
float table1[200];
float table2[200];
float table3[200];
float table4[200];

extern int   rawImageCount;
extern char  imgWork[320 * 242 * 4];

extern  int camRes;
extern  int camWide;
extern  int camHigh;
extern  int camByteCount;

extern char TeamPiIP[20];

#define PORTfrompi 1137

extern int goalCountt;

extern int teamNum;

int FindGreenTargets(char *imgBuf, int wide, int high);

void  processRawImage();

int alive = 0;

int driverSelect = 0;

int fastGyro = 0;

int gyroInitFinished = 0;

float rollerState = 0;
float shotErrorLimit = 5;

float minDisplayLine = 0;
float maxDisplayLine = 70;
float blackAndWhite = 8;


float armGain = 0.01;


float  armRes = 36;
float  toroBlockAfterShotTime = 3.0;



float bumperIsDown = 0;
float bumperIsUp = 1;

float holdGain = 8.3;
float holdGainR = 8.3;
float turretGain = 0.02;


float shotBlocker = 0;
float teamNumber = 14; //494;
float minShootTime = 0;
float moveBackRate = 90;



float goalPixHigh = 0;
float goalPixWide = 0;

float wantedOffset = 0;
float currentOffset = 0;

//---------------- load/save variables --------------

 // float offsetFront     =0.0;
 // float offsetBack      =0.0;

//  float deadFront       =10;
//  float deadBack        =10;
//  float gainFront       =1200;
// float gainBack        =1200;


int slowVideoMode = 2;

extern float robotTime;
extern double robotTimeBase;


//------------- 2015 variables -------------



					  // Aton 1 --> drive forward and stop
float a1DriveTime = 0;   // Drive forward time in seconds
float a1DriveSpeed = 0;   // Drive forward speed in +-1 units
float a1BinLevel = 0;

float a4DriveTime = 0;   // Drive forward time in seconds
float a4DriveSpeed = 0;   // Drive forward speed in +-1 units
float a4BinLevel = 0;

float a5Amz = 200;
float a5Elbo = 120;
float atonSuckSpeed = 0.4;
float userSuckSpeed = 1.0;

float a5MoveSpeed = 200;
float a5MoveTime = 200;
float binDropLoc = 0;


// Aton 2 --> Pickup three totes and bins and travel to aton zone
float a2GrabAndLiftTime = 0;   //Time  to grab and lift bin and tote before moving
float a2DropOffTime = 0;   //Time  to grab and lift bin and tote before moving
float a2RightMoveTimeB1 = 0;   //time  to move right with bin 1
float a2RightMoveSpeedB1 = 0;   //speed to move right with bin 1
float a2LeftMoveTimeB1 = 0;   //time  to move left  after dropping bin 1
float a2LeftMoveSpeedB1 = 0;   //speed to move left  after dropping bin 1
float a2ForwardMoveTimeB2 = 0;   //time  to move forward to grab bin 2
float a2ForwardMoveSpeedB2 = 0;   //speed to move forward to grab bin 2
float a2RightMoveTimeB2 = 0;   //time  to move right with bin 2
float a2RightMoveSpeedB2 = 0;   //speed to move right with bin 2
float a2LeftMoveTimeB2 = 0;   //time  to move left after dropping bin 2
float a2LeftMoveSpeedB2 = 0;   //speed to move left after dropping bin 2
float a2ForwardMoveTimeB3 = 0;   //time  to move forward to grab bin 3
float a2RightMoveTimeB3 = 0;   //time  to move right with third tote and bin (finial)
float a2RightMoveSpeedB3 = 0;   //speed to move right with third tote and bin (finial)
					  // Aton 3 --> Pickup three totes and bins and travel to aton zone
float a3ShoulderOutAngle = 0;   //Angle of shoulder for bin pickup
float a2ElboOutAngle = 0;   //Angle of elbo    for bin pickup
float a3InToGrapTime = 0;   //Time  to move in to grab tote
float a3InToGrapSpeed = 0;   //Speed to move in to grab tote
float a3OutToGrapTime = 0;   //Time  to move out to grab tote
float a3OutInToGrapSpeed = 0;   //Speed to move out to grab tote
float a3ForawrdMoveTime = 0;   //Time  to move forward to next bin
float a3ForawrdMoveSpeed = 0;   //Speed to move forward to next bin
float a3BinDropOffTime = 0;   //Time to drop bin

float a2DropAngleAmz = 180;  // 
float a2DropAngleElbo = 180;  // 
float a2PickupAngleAmz = 270;  // 
float a2PickupAngleElbo = 200;  // 
float a2DropCanTime = 1.0;  // 
float lidarDisable = 0;  // 
float lidarValue = 0;
float jawsClosed = 0;
float newImageCount = 0;

float blockCanIO = 0;

float tDisable = 0;
float bDisable = 0;
float aDisable = 0;
float eDisable = 0;
float sDisable = 0;


float lidarSelect = 0;

float atonLimit = 0;

//---------- Level Parameters -----------------

// float gyroShift     =0;
float midLevel = 35;

float DistanceH = 0;
float DistanceY = 0;
float FilterHYFlag = 0;
float FilterHYLimit = 40;
float PickupRollerFlag = 0;
float DebugRobotVideo = 0;
float PickupRollerSpeed = 0;

float toroInManual = 0;
float catcherIsOpen = 0;
float catcherIsClosed = 1;

float WheelAtSpeed = 0;
float ShiftModeFlag = 0;
float ShiftLowRes = 49;
float ShiftHighRes = 49;



float GoalFound = 0;
float	GoalX = 0;
float	GoalY = 0;
float	GoalW = 0;
float	GoalH = 0;






float shoesOut2 = 0.7;
float shoesIn2 = -0.7;

float curHotRatio = 1;
float minHotRatio = 0.05;







float A_ShotDistBeg = 0;
float A_ShotDistZone = 0;
float B_ShotDistBeg = 0;
float B_ShotDistZone = 0;
float X_ShotDistBeg = 0;
float X_ShotDistZone = 0;
float Y_ShotDistBeg = 0;
float Y_ShotDistZone = 0;
float RS_ShotDistBeg = 0;
float RS_ShotDistZone = 0;
float LH_ShotDistBeg = 0;
float LH_ShotDistZone = 0;
float RH_ShotDistBeg = 0;
float RH_ShotDistZone = 0;


float armManualFlag = 0;
float rapidShootElevSpeed = 40;
float gyroShift = 0;
float gyroShiftState = 0;
float elevBrake = 0.2;

float pauseVideo = 0;


//---------- Roller Parameters ---------------

//float lowerSuckRate=0.25;
float wantedShooterRPM = 0;

float shooterRPM = 0;
float shooterSpeed = 0;

//float lowerRollRate=-0.25;
// float upperRollRate= 0.25;

 //------------ Aton Parameters ----------------

float a1_Cmd = 0;
float a2_Cmd = 0;
float a3_Cmd = 0;
float a4_Cmd = 0;
float a5_Cmd = 0;
float a6_Cmd = 0;

float cameraSelect = 2;
//-------------------

float elevUpRate = 1.0;
float elevDownRate = 1.0;
float DriveModeFlag = 1;


//------------------------------

// --- 2014 ball shooter





float shooterEndDuration = 0.2;


// ---- Drive Train Adjustment ---

float rightDriveAdj = 1.0;
float leftDriveAdj = 1.0;
float rightDriveAdjN = 1.0;
float leftDriveAdjN = 1.0;


// --- Line Track Adjustment ---

float lineTrackAdj = 0.5;

//--- Roller Hold Speed ---

float packetSize = 0.15;

// -- auto deploy control

float autoDeployTime = 124.75;

// -- gyro adjustment

float hoodGain = 6;
float hoodMin = 1.5;
float hoodMax = 1.9;

float spare1 = 0;
float spare2 = 0;
float spare3 = 0;
float spare4 = 0;
float spare5 = 0;

//----------------------------------------------------

float cmdPosElevation = 0;
float cmdPosAzimuth = 0;
float cmdPosFrontSteer = 0;
float cmdPosBackSteer = 0;

float matchTime = 0;
double matchTimeBase = 0;
float matchStatus = 0;

int currentTeam = 0; // 0: no teams 1: red team 2: blue Team 3: both teams

int trackRedTeam = 0;
int trackBlueTeam = 0;

//---------------------- tracking ---------------------

float trackTimeOut = 0;
float trackLockMin = 0;
float trackCenterY = 0;
float trackCenterX = 0;
float trackState = 0;
float gfMinLevel = 50;
float bfMaxWide = 10;
float bfMaxHigh = 40;
float bfMaxRatio = 0.5;
float trackX = 0;
float trackY = 0;
float trackZ = 0;


// float shoesFlag       =0;
float ballDetect2 = 0;
float armFlag = 0;
float elevFlag = 0;
float armSpeed = 0;
float lookForHotGoal = 0;
float hotGoal = 0;

float crossHairOffset = 0;
float lightsOffTime = 0;
float lightsOnTime = 1;


float sSpeed[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float sHood[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
//float aCmd [20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float tCX1 = 0;
float tCX2 = 0;
float tCX3 = 0;
float tCX4 = 0;
float tCY1 = 0;
float tCY2 = 0;
float tCY3 = 0;
float tCY4 = 0;
float tH1 = 0;
float tH2 = 0;
float tH3 = 0;
float tH4 = 0;
float tW1 = 0;
float tW2 = 0;
float tW3 = 0;
float tW4 = 0;







float targetOffsetY = 0;

float minWheelShootSpeed = 4000;


float toroIsSpitting = 0;
float toroIsCollecting = 0;
float toroIsDown = 0;
float toroIsUp = 1;

float quickShotAngle = 0;
float longShotDelay = 0;
float longShotAngle = 0;

float shotDelay = 0;


//494 Default Crosshairs


// 70 Default Crosshairs








extern float mTargetA;
extern float crossHairX;











float useLeftPot = 0;

float wantedArmAngle = 0;

float crossHair2X = 80;
float crossHair2Y = 240;

float sweepRate = 40;
float ShooterPulseMaxRPM = 2000;
float ShooterPulseOffset = -0.3;


float tIndex = 0;
float tValue1 = 0;
float tValue2 = 0.25;
float tValue3 = 0.75;
float holdStraight = 0;
float tableSize = 0;
float liftFlag = 0;
float liftTime = 1;


float DistAdjH1 = -1;
float DistAdjH2 = -1;
float DistAdjY1 = -2;
float DistAdjY2 = -0.967;
float DistAdjDistH1 = 27;
float DistAdjDistH2 = 16;
float DistAdjDistY1 = 25;
float DistAdjDistY2 = 60;
float ShooterModeFlag = 0;


float RH_ShotAngBeg = 0;
float RH_ShotAngEnd = 0;
float RH_ShotSpeed = 0;
float trussShotDist = 0;



float lightsOffTimeInZone = 0;
float lightsOnTimeInZone = 0.5;
float AppReady = 1.0;

float AppStateTop = 0;
float AppStateMid = 0;
float AppStateBot = 0;

//---------  auto hang --------

// rotate to arm angle, spit, level down, arm Up, stop spit

float  ahArmAngle = 45;
float  ahStartSpitDelay = 0.00;
float  ahStopSpitDelay = 0.25;
float  ahStartDownDelay = 0.01;
float  ahStartArmUpDelay = 0.2;

float  maxManualSpitTime = 0.2;

float deployReady = 0.0;

float useLeftDist = 1;







int exitSystem = 0;

int exitSys = 0;

extern Joystick *jStick1;
extern Joystick *jStick2;
extern Joystick *jStick3;
extern Joystick *jStick4;



//Gyro *gyro1=0;

//DriverStationLCD *dsLCD=NULL;

// DigitalInput *ballDetect1Ptr=0;
// DigitalInput *ballDetect2Ptr=0;
 //DigitalInput *ballDetect3Ptr=0;
 //DigitalInput *ballDetect4Ptr=0;

DigitalInput *noRobotPtr = 0;

// DigitalInput *encoder1A=0;
// DigitalInput *encoder1B=0;

// DigitalInput *encoder2A=0;
// DigitalInput *encoder2B=0;

// DigitalInput *encoder3A=0;
// DigitalInput *encoder3B=0;

// Encoder *encoder1=0;
// Encoder *encoder2=0;
// Encoder *encoder3=0;

// DigitalInput *shooterRevPtr=0;

 //DigitalOutput *lidarSelectPtr1=0;
//  DigitalOutput *lidarSelectPtr2=0;

   // DigitalInput *ballDetect1Ptr_Back=0;
  //  DigitalInput *ballDetect2Ptr_Back=0;
   // DigitalInput *ballDetect3Ptr_Back=0;
  //  DigitalInput *ballDetect4Ptr_Back=0;

DigitalInput *noRobotPtr_Back = 0;

DigitalInput *encoder1A_Back = 0;
DigitalInput *encoder1B_Back = 0;

DigitalInput *encoder2A_Back = 0;
DigitalInput *encoder2B_Back = 0;

DigitalInput *encoder3A_Back = 0;
DigitalInput *encoder3B_Back = 0;

DigitalInput *shooterRevPtr_Back = 0;















extern int lineIo;

void UDCReceiverTestInit();

void UDCReceiverTest();

void ZeroEncoder(int index)
{
	//  if (index==1 && encoder1) encoder1->Reset();
	//  if (index==2 && encoder2) encoder2->Reset();
	//  if (index==3 && encoder3) encoder3->Reset();

}

void shooterInitService(uint32_t interruptAssertedMask, void *param);


extern float sendBackValues[420];

void userInit()
{
	robotTimeBase = (double)Timer::GetFPGATimestamp(); //Time();

	for (int L = 0; L < 420; L++) sendBackValues[L] = 0; //0.001*L;

	LoadGlobalData();

	printf("userInit\n");

	motorInit();

	setupTimers();

	encoderInit();

	jStick1 = new Joystick(0);
	jStick2 = new Joystick(1);
	//jStick3= new Joystick(2);
	//jStick4= new Joystick(3);

   // lineLeftPtr   = new DigitalInput(1);
   // lineCenterPtr = new DigitalInput(2);
   // lineRightPtr  = new DigitalInput(3);
  // UpDownLimitPtr= new DigitalInput(4);

   // encoder1A_Back=encoder1A      = new DigitalInput(1);
   // encoder1B_Back=encoder1B      = new DigitalInput(2);

   // encoder2A_Back=encoder2A      = new DigitalInput(3);
   // encoder2B_Back=encoder2B      = new DigitalInput(4);

   // encoder3A_Back=encoder3A      = new DigitalInput(5);
   // encoder3B_Back=encoder3B      = new DigitalInput(6);

   // encoder1=new Encoder(encoder1A,encoder1B);
   // encoder2=new Encoder(encoder2A,encoder2B);
   // encoder3=new Encoder(encoder3A,encoder3B);

   /* encoder1->Start(); */ // encoder1->Reset();
   /* encoder2->Start(); */ //encoder2->Reset();
   // encoder3->Start(); encoder3->Reset();

   // shooterRevPtr_Back=shooterRevPtr= new DigitalInput(11);

  // shooterRevPtr->RequestInterrupts(shooterInitService,(void*)0);

  // shooterRevPtr->SetUpSourceEdge(false,true);

 //  shooterRevPtr->EnableInterrupts (                  );

  // ballDetect1Ptr= new DigitalInput(7);
 //  ballDetect2Ptr= new DigitalInput(8);
 //  ballDetect3Ptr= new DigitalInput(9);
  // ballDetect4Ptr= new DigitalInput(10);

   //noRobotPtr     = new DigitalInput(14);

  // lidarSelectPtr1 = new DigitalOutput(0);
  //  lidarSelectPtr1 = new DigitalOutput(1);

  // lidarSelectPtr1->Set(1);
 //  lidarSelectPtr2->Set(0);


   //encoder2->Start();
   //encoder3->Start();




	lineIo = 2;

	// if (fastGyro)
	// gyro1 = new Gyro((int32_t)1); //aGyroPort+1);

	gyroInitFinished = 100;

	// dsLCD = DriverStationLCD::GetInstance();

	setupDriving();

	// UDCReceiverTestInit();



} //userInit


float lidarSelectBack = -1;

void setLidarSelect(float var)
{
	/*
	if (lidarSelectPtr1==0 || lidarSelectPtr2==0)  return;

	static int delay=0;

	delay++;

	if (var==lidarSelectBack && delay<10) return;

	delay=0;

	lidarSelect=var;

	if (lidarSelect==0)
	{
	lidarSelectPtr1->Set(1);
	lidarSelectPtr2->Set(0);
	}
	else
	{
	 lidarSelectPtr1->Set(0);
	 lidarSelectPtr2->Set(1);
	}

	lidarSelectBack=var;
	*/
}




void sendBack(int index, float value)
{
	if (index < 0 || index>400) return;

	sendBackValues[index + 2] = value;
}

float fabs(float val)
{
	if (val < 0) return -val;

	return val;
}

int  SumBritePixels(char *imgBuf, int wide, int high);

char imgSend[640 * 480 * 4];

int  imgSendCnt;

int  imgGone;

float offsetAdjust = 0;

double updateRate = 0;

//void UDCSendTest();

double imageAge = 0;
double lastImageTime = 0;

//void RightToro (float speed);
//void LeftToro (float speed);

#define PROCEDURE void
#define CARD int
#define CHAR char

PROCEDURE greenFilter(CHAR *imgBuf, CARD wide, CARD high);


float ffabs(float val)
{
	if (val > 0) return val;

	return -val;
}

extern float CustomDriveFlag;

void  customJoySticks();

void userFastLoop(void)  //200HZ
{
	//printf("userFastLoop\n");

	  //return;

	UDCReceiverTest();

	/// UDCSendTest();

	atonSpin = 0;

	trackRedTeam = redTeam; trackBlueTeam = blueTeam;

	IFC_Local_Loop_Service();

	customJoySticks();

	// static int lastImage=0;

	// static int goalCount=0;

	 // int newImage=0;

	 // Check For New Image

	 //static double lastTime=0;

	double nTime = (double)Timer::GetFPGATimestamp(); //GetTime();

	//double dTime=nTime-lastTime;

	//lastTime=nTime;

	//printf("dTime %f\n",(float) dTime);

	robotTime = nTime - robotTimeBase;

	//robotTime=123.123;

	sendBack(309, robotTime);

	//static int lastImage = 0;

	imageAge = lastImageTime - nTime;

	// if (rawImageCount!=lastImage)
	{

		lastImageTime = nTime;

		//  processRawImage();

		//  {
			//goalCount=0;
			//goalCountt=0;

			//mTargetX=0;

		//	if (imgGone) imgGone=0;
		//  }
		//  else
		if (LookForGoal)
		{

			//    int goalCountt=FindGreenTargets((char*)&imgWork[0],320,240);

						  //  printf("goalCount %d\n",goalCountt);

						  //if (goalCountt==0) mTargetX=0;

						  //  if (imgGone)

						  // for (int L=0; L<camByteCount;L++) imgWork[L]=100;

						  //  greenFilter(imgWork, 320,240);

			 //   memcpy(imgSend,imgWork,camByteCount);

						  //  printf("sendBack %d\n",imgGone);

			imgGone = 0;

		}

		imgSendCnt++;

		//newImage=1;
	}

	//lastImage=rawImageCount;





	//float steer= joyX[1]/4;  //2

	//if (steer>0.25) steer=0.25; if (steer<-0.25) steer=-0.25;

  //  float front    =1.0-(steer+0.5);
  //  float back     =0.5; //1.0-front;

	//float front    =1.0-(steer+0.5);
   // float back     =1.0-front;

  //  if (!crabDrive)

	  //  front=0.5;

   //if (backDrive) back=1.0-front;

	//updateSteering( front, back,crabDrive,0 );


	updateDashboardLights();

	teamNumber = teamNum;

	//shooterRPM = Get_Speed_ShooterWheel();

	GlobalSendBack();

	//getGyroAngle();

	//updatePWM();



} //userFastLoop

void FinishProcessing();

void greenFilter(char *imgBuf, int wide, int high);
void lightsFilter(char *imgBuf, int wide, int high);

void backProcess()
{
	return;

	static int lastImage = 0;

	double nTime = Timer::GetFPGATimestamp();

	// if (spare4<0.05) spare4=0.05;



	if (rawImageCount != lastImage)
	{

		// printf("dtime %f %f\n",(float) (nTime-lastImageTime),spare4);

		if (1) //nTime-lastImageTime>spare4 ) //|| blackAndWhite>6)
		{

			// tprintf("process Raw Image\n");

		   // printf("process Raw Image\n");

			lastImageTime = nTime;

			processRawImage();

			//  {

			//	  goalCountt=0;

			//	  mTargetX=0;

			//  }
			//  else
			{

				//  goalCountt=0;

				//  lightsFilter(imgWork,320,240/8);

				FinishProcessing();

				//  mTargetX=0;

			}

			if (imgGone) imgGone = 0;

			imgSendCnt++;
		}

	}

	lastImage = rawImageCount;
}

void backProcessx()
{
	static double nTime1 = 0;
	static double nTime2 = 0;

	nTime1 = Timer::GetFPGATimestamp();

	double dTime = 1.0 / (nTime1 - nTime2); spare2 = dTime;

	nTime2 = Timer::GetFPGATimestamp();

	//static int lastImage = 0;

	// static int goalCount=0;

	// int newImage=0;

	double nTime = Timer::GetFPGATimestamp();

	imageAge = lastImageTime - nTime;

	//  if (rawImageCount!=lastImage)
	{
		lastImageTime = nTime;

		//  processRawImage();

		//  {
			//goalCount=0;
			//goalCountt=0;

		//	mTargetX=0;
//
		//	if (imgGone) imgGone=0;
		 // }
		//  else
		{

			//	goalCountt=goalCount=FindGreenTargets(imgWork,160,120);

			 //   if (goalCount==0) mTargetX=0;

		//	    if (imgGone) memcpy(imgSend,imgWork,camByteCount);

			imgGone = 0;

		}

		imgSendCnt++;

		//newImage=1;
	}

	//  lastImage=rawImageCount;
}


float gyroAng = 0;

extern float holdBase;
extern float holdLast;

int gyroResetCount = 0;

void resetGyro();

float gyroAdjust = 1; //1.0/12.38; //90.0/48.0;

float getGyroAngle()
{
	return 0.0;

	//if (gyro1==0) return 0;

	// float value=gyro1->GetAngle()*gyroAdjust;

	//float value=gyroAng*gyroAdjust;

	//if (gyro1)  value= gyro1->GetAngle()*gyroAdjust;

	 //if (value>10000 || value<-10000)

	 //	value=0; //gyroFault=1;

	//if (gyroResetCount>0) {gyroResetCount--;value=0;}

	//loc[7]=value;

	//if (disableGyro || gyroFault) return 0;

	//return value;
}




int blockGyroReset = 0;

void lockStraightDrive(int timeOut)
{
	blockGyroReset = timeOut;

	//driveStraightGyro=getGyroAngle();
}



int newDrivingInfo = 0;

//GetRawAxis

float limit1(float val)
{
	if (val > 1) return  1;
	if (val < -1) return -1;

	return val;
}

float adjustStick(float val);
float adjustStick2(float val);
float adjustStick3(float val);
float adjustStick3nd(float val);

extern int reverseDrive;

void userNewDriverStationPacket()
{



	//float value=(float)jStick2->GetPOV(0);

	//printf("%1.0f\n",value);




//printf("joyY2[1] %f joyY2[2] %f\n",joyY2[1],joyY2[2]);

	newDrivingInfo = 1;

	if (blockCanIO != 2)
	{

		if (jStick2)
		{
			//if (fabs(jStick2->GetRawAxis(1)) > 0.5) { inAt = 0; inAton = 0; }
			//if (fabs(jStick2->GetRawAxis(0)) > 0.5) { inAt = 0; inAton = 0; }
		}

		if (jStick1) joyX[1] = jStick1->GetRawAxis(0); else joyX[1] = 0;
		if (jStick2) joyX[2] = jStick2->GetRawAxis(0); else joyX[2] = 0;

		// printf("joyX[1] %6.2f \n",(float)joyX[1]);

		  //  joyX[3]=jStick3->GetRawAxis(0);
		  //  joyX[4]=jStick4->GetRawAxis(0);

		if (jStick1)	joyY[1] = jStick1->GetRawAxis(1); else joyY[1] = 0;
		if (jStick2) joyY[2] = jStick2->GetRawAxis(1); else joyY[2] = 0;
		//  joyY[3]=jStick3->GetRawAxis(1);
		//   joyY[4]=jStick4->GetRawAxis(1);

		if (jStick1)	joyX2[1] = jStick1->GetRawAxis(4); else joyX2[1] = 0;
		if (jStick2) joyX2[2] = jStick2->GetRawAxis(4); else joyX2[2] = 0;
		//  joyX2[3]=jStick3->GetRawAxis(4);
		//   joyX2[4]=jStick4->GetRawAxis(4);

		if (jStick1)	 joyY2[1] = jStick1->GetRawAxis(5); else joyY2[1] = 0;
		if (jStick2)  joyY2[2] = jStick2->GetRawAxis(5); else joyY2[2] = 0;
		//   joyY2[3]=jStick3->GetRawAxis(5);
		//   joyY2[4]=jStick4->GetRawAxis(5);

		//  printf("joyY2[1] %f joyY2[2] %f\n",joyY2[1],joyY2[2]);

		if (jStick1)	 hatX[1] = jStick1->GetRawAxis(2); else hatX[1] = 0;
		if (jStick2)  hatX[2] = jStick2->GetRawAxis(2); else hatX[2] = 0;

		//   hatX[3]=jStick3->GetRawAxis(2);
		//   hatX[4]=jStick4->GetRawAxis(2);

		if (jStick1) hatY[1] = jStick1->GetRawAxis(3); else hatY[1] = 0;
		if (jStick2) hatY[2] = jStick2->GetRawAxis(3); else hatY[1] = 0;

		// printf("HatX %f HatY %f\n",hatX[2],hatY[2]);

		//   hatY[3]=jStick3->GetRawAxis(3);
		//   hatY[4]=jStick4->GetRawAxis(3);

		joyT[1] = hatX[1] - hatY[1];
		joyT[2] = hatX[2] - hatY[2];
		//    joyT[3]=hatX[3]+hatY[3];
		//    joyT[4]=hatX[4]+hatY[4];


	  //loc[15]=hatX[1];
	   //loc[16]=hatX[2];
	  // loc[17]=0;
	  // loc[18]=0;




	  //float x=joyX2[1];// float xabs=fabs(x);
	  //float y=joyY2[1];// float yabs=fabs(y);

	 // joyT[arm]=adjustStick3nd(jStick1->GetRawAxis(2));
	 // joyT[drv]=jStick2->GetRawAxis(2);
	 // joyT[3]=jStick3->GetRawAxis(2);
	 // joyT[4]=jStick4->GetRawAxis(2);


	 // loc[19]=joyT[2];



		for (int m = 1; m < 11; m++)
		{
			if (jStick1)  p_sw[1][m] = jStick1->GetRawButton(m); else p_sw[1][m] = 0;
			if (jStick2)  p_sw[2][m] = jStick2->GetRawButton(m); else p_sw[2][m] = 0;

			//  if (p_sw[1][m] || p_sw[2][m]) printf(" m %d  v %d v %d\n",m,p_sw[1][m],p_sw[2][m]);
			 //  p_sw[3][m]=jStick3->GetRawButton(m);
			 //    p_sw[4][m]=jStick4->GetRawButton(m);
		}

		int pov1 = jStick1->GetPOV(0);
		int pov2 = jStick2->GetPOV(0);


		if (pov1 == 0) { p_sw[1][11] = 1; p_sw[1][12] = 0; p_sw[1][13] = 0; p_sw[1][14] = 0; }
		else
			if (pov1 == 90) { p_sw[1][11] = 0; p_sw[1][12] = 1; p_sw[1][13] = 0; p_sw[1][14] = 0; }
			else
				if (pov1 == 180) { p_sw[1][11] = 0; p_sw[1][12] = 0; p_sw[1][13] = 1; p_sw[1][14] = 0; }
				else
					if (pov1 == 270) { p_sw[1][11] = 0; p_sw[1][12] = 0; p_sw[1][13] = 0; p_sw[1][14] = 1; }
					else
					{
						p_sw[1][11] = 0; p_sw[1][12] = 0; p_sw[1][13] = 0; p_sw[1][14] = 0;
					}



		if (pov2 == 0) { p_sw[2][11] = 1; p_sw[2][12] = 0; p_sw[2][13] = 0; p_sw[2][14] = 0; }
		else
			if (pov2 == 90) { p_sw[2][11] = 0; p_sw[2][12] = 1; p_sw[2][13] = 0; p_sw[2][14] = 0; }
			else
				if (pov2 == 180) { p_sw[2][11] = 0; p_sw[2][12] = 0; p_sw[2][13] = 1; p_sw[2][14] = 0; }
				else
					if (pov2 == 270) { p_sw[2][11] = 0; p_sw[2][12] = 0; p_sw[2][13] = 0; p_sw[2][14] = 1; }
					else
					{
						p_sw[2][11] = 0; p_sw[2][12] = 0; p_sw[2][13] = 0; p_sw[2][14] = 0;
					}


		//printf("%d %d %d %d\n",p_sw[2][11],p_sw[2][12],p_sw[2][13],p_sw[2][14]);

	}
}

int Limit255(int value)
{
	if (value < 0) return 0;
	if (value > 255) return 255;

	return value;
}

float adjustStick(float val)
{

	if (val > -0.15 && val < 0.15) return 0;

	if (val < 0) val = val + 0.15;
	else
		val = val - 0.15;

	val = val / 0.85;

	return val;

}

float adjustStick2(float val)
{

	if (val > -0.15 && val < 0.15) return 0;

	if (val < 0) val = val + 0.15;
	else
		val = val - 0.15;

	val = val / 0.85;

	if (val > 0) val = val*val;
	else       val = -(val*val);

	return val;


}

float adjustStick3(float val)
{

	if (val > -0.15 && val < 0.15)return 0;

	if (val < 0) val = val + 0.15;
	else
		val = val - 0.15;

	val = val / 0.85;

	val = val*val*val;

	return val;

}

float adjustStick3nd(float val)
{
	val = val*val*val;

	return val;

}
void eol(FILE * pFile) { putc((char)13, pFile); putc((char)10, pFile); }

int getFloat(FILE *pFile, float &val)
{
	float f = 0;

	int result = fscanf(pFile, "%f", &f); if (result < 0) return 0;

	val = f;

	return 1;
}

FILE *LoadFile = NULL;
int  endOfFile = 0;
char token[100];

int getToken(void)
{
	int result2 = fscanf(LoadFile, "%s", token); if (result2 < 0) return 0;

	// printf("-> %s",token);

	return 1;
}

int   assignFlag = 0;
float assignVal = 0;

int match(const char *str, float &val)
{
	if (strcmp(token, str)) return 0;

	if (assignFlag) //mess to keep # of tables down
	{
		  printf(str);
			  printf(" = %5.2f\n",assignVal);


		val = assignVal;

		assignFlag = 0;

		return 1;
	}

	float ftmp = 0;

	if (fscanf(LoadFile, "%f", &ftmp) < 0) return 0;

	val = ftmp;

	return 1;
}

int checkForMatch()
{

   if (match("a1_Rotate",a1_Rotate)) return 1;
   if (match("a2_Rotate",a2_Rotate)) return 1;
   if (match("a3_Rotate",a3_Rotate)) return 1;
   if (match("a4_Rotate",a4_Rotate)) return 1;
   if (match("a5_Rotate",a5_Rotate)) return 1;
   if (match("a6_Rotate",a6_Rotate)) return 1;
   if (match("selectCamera"        ,selectCamera)) return 1;

   //Match

if (match("move_OverHeadDroppoff",move_OverHeadDroppoff)) return 1;
if (match("TwoCubePickupAngle"  ,TwoCubePickupAngle)) return 1;
if (match("move_TwoCubeDistance",move_TwoCubeDistance)) return 1;
if (match("move_CornerToScale"  ,move_CornerToScale)) return 1;
if (match("strafe_behindHeadDropoff",strafe_behindHeadDropoff)) return 1;
if (match("strafe_secondCubePickup",strafe_secondCubePickup)) return 1;
if (match("strafe_firstCubePickup",strafe_firstCubePickup)) return 1;
if (match("LiftScaleBehind"     ,LiftScaleBehind)) return 1;
if (match("ToolBarScaleBehind"  ,ToolBarScaleBehind)) return 1;
if (match("ultra3Distance"      ,ultra3Distance)) return 1;
if (match("ultra2Distance"      ,ultra2Distance)) return 1;
if (match("ultra1Distance"      ,ultra1Distance)) return 1;
if (match("move_CornerDropCube" ,move_CornerDropCube)) return 1;
if (match("move_CrossDropCube"  ,move_CrossDropCube)) return 1;
if (match("HoldCube"            ,HoldCube)) return 1;
if (match("TwoCube"             ,TwoCube)) return 1;
if (match("LiftClimbSide"       ,LiftClimbSide)) return 1;
if (match("ToolBarClimbSide"    ,ToolBarClimbSide)) return 1;
if (match("Crossover"           ,Crossover)) return 1;
if (match("LiftClimb"           ,LiftClimb)) return 1;
if (match("ToolBarClimb"        ,ToolBarClimb)) return 1;
if (match("favorScale"          ,favorScale)) return 1;
if (match("move_ScalefromBehind",move_ScalefromBehind)) return 1;
if (match("move_FaceScale"      ,move_FaceScale)) return 1;
if (match("move_SwitchfromBehind",move_SwitchfromBehind)) return 1;
if (match("move_BehindSwitch"   ,move_BehindSwitch)) return 1;
if (match("move_SideCenterofSwitch",move_SideCenterofSwitch)) return 1;
if (match("move_TouchSwitch"    ,move_TouchSwitch)) return 1;
if (match("move_CenterofSwitch" ,move_CenterofSwitch)) return 1;
if (match("move_ApproachSwitch" ,move_ApproachSwitch)) return 1;
if (match("matchNumber"         ,matchNumber)) return 1;
if (match("matchTimeLeft"       ,matchTimeLeft)) return 1;
if (match("ToolBarPickup"       ,ToolBarPickup)) return 1;
if (match("robotWatts"          ,robotWatts)) return 1;
if (match("EncoderGain"         ,EncoderGain)) return 1;
if (match("LiftGain"            ,LiftGain)) return 1;
if (match("ToolBarGain"         ,ToolBarGain)) return 1;
if (match("TuningVar5"          ,TuningVar5)) return 1;
if (match("TuningVar4"          ,TuningVar4)) return 1;
if (match("TuningVar3"          ,TuningVar3)) return 1;
if (match("TuningVar2"          ,TuningVar2)) return 1;
if (match("TuningVar1"          ,TuningVar1)) return 1;
if (match("ToolBarScaleHigh"    ,ToolBarScaleHigh)) return 1;
if (match("ToolBarScaleMid"     ,ToolBarScaleMid)) return 1;
if (match("ToolBarScaleLow"     ,ToolBarScaleLow)) return 1;
if (match("ToolBarSwitch"       ,ToolBarSwitch)) return 1;
if (match("LiftScaleHigh"       ,LiftScaleHigh)) return 1;
if (match("LiftScaleMid"        ,LiftScaleMid)) return 1;
if (match("LiftScaleLow"        ,LiftScaleLow)) return 1;
if (match("LiftSwitch"          ,LiftSwitch)) return 1;
if (match("LiftLow"             ,LiftLow)) return 1;
if (match("LiftDest"            ,LiftDest)) return 1;
if (match("ToolBarSpeed"        ,ToolBarSpeed)) return 1;
if (match("Lift2Volts"          ,Lift2Volts)) return 1;
if (match("Lift1Volts"          ,Lift1Volts)) return 1;
if (match("Lift2Amps"           ,Lift2Amps)) return 1;
if (match("Lift1Amps"           ,Lift1Amps)) return 1;
if (match("Lift2Speed"          ,Lift2Speed)) return 1;
if (match("Lift1Speed"          ,Lift1Speed)) return 1;
if (match("ToolBarAngle"        ,ToolBarAngle)) return 1;
if (match("CubeIntakeAngle"     ,CubeIntakeAngle)) return 1;
if (match("LiftHeight"          ,LiftHeight)) return 1;
if (match("BatteryVoltage"      ,BatteryVoltage)) return 1;
if (match("RBD2Amps"            ,RBD2Amps)) return 1;
if (match("RBD1Amps"            ,RBD1Amps)) return 1;
if (match("RFD2Amps"            ,RFD2Amps)) return 1;
if (match("RFD1Amps"            ,RFD1Amps)) return 1;
if (match("LBD2Amps"            ,LBD2Amps)) return 1;
if (match("LBD1Amps"            ,LBD1Amps)) return 1;
if (match("LFD2Amps"            ,LFD2Amps)) return 1;
if (match("LFD1Amps"            ,LFD1Amps)) return 1;
if (match("LFD1Speed"           ,LFD1Speed)) return 1;
if (match("RBD2Speed"           ,RBD2Speed)) return 1;
if (match("RBD1Speed"           ,RBD1Speed)) return 1;
if (match("LBD2Speed"           ,LBD2Speed)) return 1;
if (match("LBD1Speed"           ,LBD1Speed)) return 1;
if (match("RFD2Speed"           ,RFD2Speed)) return 1;
if (match("RFD1Speed"           ,RFD1Speed)) return 1;
if (match("LFD2Speed"           ,LFD2Speed)) return 1;
if (match("EncoderVelocity"     ,EncoderVelocity)) return 1;
if (match("EncoderDistance"     ,EncoderDistance)) return 1;
if (match("aton_HopperRPM"      ,aton_HopperRPM)) return 1;
if (match("aton_ShooterRPM"     ,aton_ShooterRPM)) return 1;
if (match("maxGoalAimSpeed"     ,maxGoalAimSpeed)) return 1;
if (match("GoalAimGain"         ,GoalAimGain)) return 1;
if (match("hopperShootRPM"      ,hopperShootRPM)) return 1;
if (match("gearDropoffSpeed"    ,gearDropoffSpeed)) return 1;
if (match("gearPickupSpeed"     ,gearPickupSpeed)) return 1;
if (match("maxPegAimSpeed"      ,maxPegAimSpeed)) return 1;
if (match("maxPegDriveSpeed"    ,maxPegDriveSpeed)) return 1;
if (match("pegGoalDriveScale"   ,pegGoalDriveScale)) return 1;
if (match("pegGoalAimScale"     ,pegGoalAimScale)) return 1;
if (match("camerafps"           ,camerafps)) return 1;
if (match("PegGoalY1"           ,PegGoalY1)) return 1;
if (match("PegGoalX1"           ,PegGoalX1)) return 1;
if (match("BoilerGoalY3"        ,BoilerGoalY3)) return 1;
if (match("BoilerGoalY2"        ,BoilerGoalY2)) return 1;
if (match("BoilerGoalY1"        ,BoilerGoalY1)) return 1;
if (match("BoilerGoalX3"        ,BoilerGoalX3)) return 1;
if (match("BoilerGoalX2"        ,BoilerGoalX2)) return 1;
if (match("BoilerGoalX1"        ,BoilerGoalX1)) return 1;
if (match("BoilerErrorScale"    ,BoilerErrorScale)) return 1;
if (match("cam1dr"              ,cam1dr)) return 1;
if (match("cam2dr"              ,cam2dr)) return 1;
if (match("cam2dg"              ,cam2dg)) return 1;
if (match("cam1dg"              ,cam1dg)) return 1;
if (match("ShooterShootRPM"     ,ShooterShootRPM)) return 1;
if (match("HopperShootSpeed"    ,HopperShootSpeed)) return 1;
if (match("ToolBarGearPickup"   ,ToolBarGearPickup)) return 1;
if (match("ToolBarGearDropoff"  ,ToolBarGearDropoff)) return 1;
if (match("PegTargetGot"        ,PegTargetGot)) return 1;
if (match("PegGoalY"            ,PegGoalY)) return 1;
if (match("PegGoalX"            ,PegGoalX)) return 1;
if (match("BoilerTargetGot"     ,BoilerTargetGot)) return 1;
if (match("TurretScale"         ,TurretScale)) return 1;
if (match("TurretPOS"           ,TurretPOS)) return 1;
if (match("HopperRPM"           ,HopperRPM)) return 1;
if (match("HopperPos"           ,HopperPos)) return 1;

if (match("a6_MoveTime1", a6_MoveTime1)) return 1;
if (match("a5_MoveTime1", a5_MoveTime1)) return 1;
if (match("a4_MoveTime1", a4_MoveTime1)) return 1;
if (match("a3_MoveTime1", a3_MoveTime1)) return 1;
if (match("a2_MoveTime1", a2_MoveTime1)) return 1;

if (match("a1_MoveTime1", a1_MoveTime1)) return 1;
float tmp = 0;

if (match("atonNum", tmp)) { atonNum = (int)tmp; printf("setting atonnum\n"); return 1; }
   return 0;

	//  if (match("offsetFront"        ,offsetFront   )) return 1;
   //   if (match("offsetBack"         ,offsetBack    )) return 1;

	//  if (match("deadFront"        ,deadFront     )) return 1;
	//  if (match("deadBack"         ,deadBack      )) return 1;
	//  if (match("gainFront"        ,gainFront     )) return 1;
	//  if (match("gainBack"         ,gainBack      )) return 1;

	//------------- 2015 variables -------------

	if (match("tLoadedCount", tLoadedCount)) return 1;
	if (match("tPickupLevel", tPickupLevel)) return 1;
	if (match("tFloorLevel", tFloorLevel)) return 1;
	if (match("tCarryLevel", tCarryLevel)) return 1;
	if (match("tLoadLevel", tLoadLevel)) return 1;
	if (match("tStepLevel", tStepLevel)) return 1;
	if (match("tStepLevel1", tStepLevel1)) return 1;
	if (match("tStepLevel2", tStepLevel2)) return 1;
	if (match("tStepLevel3", tStepLevel3)) return 1;
	if (match("tGain", tGain)) return 1;
	if (match("tResolution", tResolution)) return 1;
	if (match("tMaxUpSpeed", tMaxUpSpeed)) return 1;
	if (match("tMaxDnSpeed", tMaxDnSpeed)) return 1;
	if (match("tMode", tMode)) return 1;
	if (match("tTimeout", tTimeout)) return 1;
	if (match("tZero", tZero)) return 1;

	if (match("tCmdPos", tCmdPos)) return 1;
	if (match("bCmdPos", bCmdPos)) return 1;

	if (match("aCmdPos", aCmdPos)) return 1;
	if (match("eCmdPos", eCmdPos)) return 1;
	if (match("sCmdPos", sCmdPos)) return 1;


	if (match("bPickupLevel", bPickupLevel)) return 1;
	if (match("bFloorLevel", bFloorLevel)) return 1;
	if (match("bCarryLevel", bCarryLevel)) return 1;
	if (match("bLoadLevel", bLoadLevel)) return 1;
	if (match("bStepLevel", bStepLevel)) return 1;
	if (match("bStepLevelUp", bStepLevelUp)) return 1;
	if (match("bToteLevel1", bToteLevel1)) return 1;
	if (match("bToteLevel2", bToteLevel2)) return 1;
	if (match("bToteLevel3", bToteLevel3)) return 1;
	if (match("bToteLevel4", bToteLevel4)) return 1;
	if (match("bToteLevel5", bToteLevel5)) return 1;
	if (match("bToteLevel6", bToteLevel6)) return 1;
	if (match("bToteTopClr", bToteTopClr)) return 1;
	if (match("bGain", bGain)) return 1;
	if (match("bResolution", bResolution)) return 1;
	if (match("bMaxUpSpeed", bMaxUpSpeed)) return 1;
	if (match("bMaxDnSpeed", bMaxDnSpeed)) return 1;
	if (match("bMode", bMode)) return 1;
	if (match("bTimeout", bTimeout)) return 1;
	if (match("bZero", bZero)) return 1;
	if (match("aPickupAngle", aPickupAngle)) return 1;
	if (match("aFloorAngle", aFloorAngle)) return 1;
	if (match("aCarryAngle", aCarryAngle)) return 1;
	if (match("aLoadAngle", aLoadAngle)) return 1;
	if (match("aStepAngle", aStepAngle)) return 1;
	if (match("aStepAngleUp", aStepAngleUp)) return 1;
	if (match("aToteAngle1", aToteAngle1)) return 1;
	if (match("aToteAngle2", aToteAngle2)) return 1;
	if (match("aToteAngle3", aToteAngle3)) return 1;
	if (match("aToteAngle4", aToteAngle4)) return 1;
	if (match("aToteAngle5", aToteAngle5)) return 1;
	if (match("aToteAngle6", aToteAngle6)) return 1;
	if (match("aToteTopClr", aToteTopClr)) return 1;
	if (match("aGain", aGain)) return 1;
	if (match("aResolution", aResolution)) return 1;
	if (match("aMaxCWSpeed", aMaxCWSpeed)) return 1;
	if (match("aMaxCCWSpeed", aMaxCCWSpeed)) return 1;
	if (match("aMode", aMode)) return 1;
	if (match("aTimeout", aTimeout)) return 1;
	if (match("ePickupAngle", ePickupAngle)) return 1;
	if (match("eFloorAngle", eFloorAngle)) return 1;
	if (match("eCarryAngle", eCarryAngle)) return 1;
	if (match("eLoadAngle", eLoadAngle)) return 1;
	if (match("eStepAngle", eStepAngle)) return 1;
	if (match("eStepAngleUp", eStepAngleUp)) return 1;
	if (match("eToteAngle1", eToteAngle1)) return 1;
	if (match("eToteAngle2", eToteAngle2)) return 1;
	if (match("eToteAngle3", eToteAngle3)) return 1;
	if (match("eToteAngle4", eToteAngle4)) return 1;
	if (match("eToteAngle5", eToteAngle5)) return 1;
	if (match("eToteAngle6", eToteAngle6)) return 1;
	if (match("eToteTopClr", eToteTopClr)) return 1;
	if (match("eGain", eGain)) return 1;
	if (match("eResolution", eResolution)) return 1;
	if (match("eMaxCWSpeed", eMaxCWSpeed)) return 1;
	if (match("eMaxCCWSpeed", eMaxCCWSpeed)) return 1;
	if (match("eMode", eMode)) return 1;
	if (match("eTimeout", eTimeout)) return 1;
	if (match("a1DriveTime", a1DriveTime)) return 1;
	if (match("a1BinLevel", a1BinLevel)) return 1;
	if (match("a1DriveSpeed", a1DriveSpeed)) return 1;

	if (match("a4DriveTime", a4DriveTime)) return 1;
	if (match("a4BinLevel", a4BinLevel)) return 1;
	if (match("a4DriveSpeed", a4DriveSpeed)) return 1;

	if (match("a5Amz", a5Amz)) return 1;
	if (match("a5Elbo", a5Elbo)) return 1;
	if (match("atonSuckSpeed", atonSuckSpeed)) return 1;
	if (match("userSuckSpeed", userSuckSpeed)) return 1;

	if (match("a5MoveSpeed", a5MoveSpeed)) return 1;
	if (match("a5MoveTime", a5MoveTime)) return 1;

	if (match("binDropLoc", binDropLoc)) return 1;


	if (match("a2GrabAndLiftTime", a2GrabAndLiftTime)) return 1;
	if (match("a2DropOffTime", a2DropOffTime)) return 1;
	if (match("a2RightMoveTimeB1", a2RightMoveTimeB1)) return 1;
	if (match("a2RightMoveSpeedB1", a2RightMoveSpeedB1)) return 1;
	if (match("a2LeftMoveTimeB1", a2LeftMoveTimeB1)) return 1;
	if (match("a2LeftMoveSpeedB1", a2LeftMoveSpeedB1)) return 1;
	if (match("a2ForwardMoveTimeB2", a2ForwardMoveTimeB2)) return 1;
	if (match("a2ForwardMoveSpeedB2", a2ForwardMoveSpeedB2)) return 1;
	if (match("a2RightMoveTimeB2", a2RightMoveTimeB2)) return 1;
	if (match("a2RightMoveSpeedB2", a2RightMoveSpeedB2)) return 1;
	if (match("a2LeftMoveTimeB2", a2LeftMoveTimeB2)) return 1;
	if (match("a2LeftMoveSpeedB2", a2LeftMoveSpeedB2)) return 1;
	if (match("a2ForwardMoveTimeB3", a2ForwardMoveTimeB3)) return 1;
	if (match("a2RightMoveTimeB3", a2RightMoveTimeB3)) return 1;
	if (match("a2RightMoveSpeedB3", a2RightMoveSpeedB3)) return 1;
	if (match("a3ShoulderOutAngle", a3ShoulderOutAngle)) return 1;
	if (match("a2ElboOutAngle", a2ElboOutAngle)) return 1;
	if (match("a3InToGrapTime", a3InToGrapTime)) return 1;
	if (match("a3InToGrapSpeed", a3InToGrapSpeed)) return 1;
	if (match("a3OutToGrapTime", a3OutToGrapTime)) return 1;
	if (match("a3OutInToGrapSpeed", a3OutInToGrapSpeed)) return 1;
	if (match("a3ForawrdMoveTime", a3ForawrdMoveTime)) return 1;
	if (match("a3ForawrdMoveSpeed", a3ForawrdMoveSpeed)) return 1;
	if (match("a3BinDropOffTime", a3BinDropOffTime)) return 1;

	if (match("blockCanIO", blockCanIO)) return 1;
	if (match("lidarSelect", lidarSelect)) return 1;

	if (match("tDisable", tDisable)) return 1;
	if (match("bDisable", bDisable)) return 1;
	if (match("aDisable", aDisable)) return 1;
	if (match("eDisable", eDisable)) return 1;
	if (match("sDisable", sDisable)) return 1;

	if (match("a2DropAngleAmz", a2DropAngleAmz)) return 1;
	if (match("a2DropAngleElbo", a2DropAngleElbo)) return 1;
	if (match("a2PickupAngleAmz", a2PickupAngleAmz)) return 1;
	if (match("a2PickupAngleElbo", a2PickupAngleElbo)) return 1;
	if (match("a2DropCanTime", a2DropCanTime)) return 1;

	if (match("lidarDisable", lidarDisable)) return 1;
	if (match("lidarValue", lidarValue)) return 1;
	if (match("jawsClosed", jawsClosed)) return 1;
	if (match("newImageCount", newImageCount)) return 1;

	if (match("atonLimit", atonLimit)) return 1;

	//-------------------------------------------


	if (match("slowVideoMode", tmp)) { slowVideoMode = (int)tmp; return 1; }

	if (match("currentTeam", tmp)) { currentTeam = (int)tmp; return 1; }

	//if (match("stationNum"     ,tmp)) {stationNum   =(int)tmp;return 1;}

	if (match("atonDelayTime", atonDelayTime)) return 1;


	if (match("armGain", armGain)) return 1;



	if (match("armRes", armRes)) return 1;

	if (match("toroBlockAfterShotTime", toroBlockAfterShotTime)) return 1;


	if (match("bumperIsDown", bumperIsDown)) return 1;
	if (match("bumperIsUp", bumperIsUp)) return 1;

	if (match("holdGain", holdGain)) return 1;
	if (match("holdGainR", holdGainR)) return 1;
	if (match("turretGain", turretGain)) return 1;


	if (match("shotBlocker", shotBlocker)) return 1;
	if (match("teamNumber", teamNumber)) return 1;

	if (match("minShootTime", minShootTime)) return 1;
	if (match("moveBackRate", moveBackRate)) return 1;






	//------------------------------------------------

	if (match("DistanceH", DistanceH)) return 1;
	if (match("DistanceY", DistanceY)) return 1;
	if (match("FilterHYFlag", FilterHYFlag)) return 1;
	if (match("FilterHYLimit", FilterHYLimit)) return 1;
	if (match("PickupRollerFlag", PickupRollerFlag)) return 1;
	if (match("DebugRobotVideo", DebugRobotVideo)) return 1;

	if (match("GoalFound", GoalFound)) return 1;
	if (match("GoalX", GoalX)) return 1;
	if (match("GoalY", GoalY)) return 1;
	if (match("GoalW", GoalW)) return 1;
	if (match("GoalH", GoalH)) return 1;

	if (match("autoDriveX", autoDriveX)) return 1;
	if (match("autoDriveY", autoDriveY)) return 1;
	if (match("autoDriveR", autoDriveR)) return 1;

	if (match("PickupRollerSpeed", PickupRollerSpeed)) return 1;
	if (match("toroInManual", toroInManual)) return 1;
	if (match("catcherIsOpen", catcherIsOpen)) return 1;
	if (match("catcherIsClosed", catcherIsClosed)) return 1;


	if (match("WheelAtSpeed", WheelAtSpeed)) return 1;
	if (match("ShiftModeFlag", ShiftModeFlag)) return 1;




	if (match("shoesOut2", shoesOut2)) return 1;
	if (match("shoesIn2", shoesIn2)) return 1;


	//if (match("A_ShotDistBeg" 	     ,A_ShotDistBeg  	       )) return 1;
	//if (match("A_ShotDistZone" 	     ,A_ShotDistZone  	       )) return 1;
	//if (match("B_ShotDistBeg" 	     ,B_ShotDistBeg  	       )) return 1;
	//if (match("B_ShotDistZone" 	     ,B_ShotDistZone  	       )) return 1;
	//if (match("X_ShotDistBeg" 	     ,X_ShotDistBeg  	       )) return 1;
	//if (match("X_ShotDistZone" 	     ,X_ShotDistZone  	       )) return 1;
	//if (match("Y_ShotDistBeg" 	     ,Y_ShotDistBeg  	       )) return 1;
	//if (match("Y_ShotDistZone" 	     ,Y_ShotDistZone  	       )) return 1;
	//if (match("RS_ShotDistBeg" 	     ,RS_ShotDistBeg 	       )) return 1;
	//if (match("RS_ShotDistZone" 	     ,RS_ShotDistZone 	       )) return 1;
	//if (match("LH_ShotDistBeg" 	     ,LH_ShotDistBeg 	       )) return 1;
	//if (match("LH_ShotDistZone" 	     ,LH_ShotDistZone 	       )) return 1;
	//if (match("RH_ShotDistBeg" 	     ,RH_ShotDistBeg 	       )) return 1;
	//if (match("RH_ShotDistZone" 	     ,RH_ShotDistZone 	       )) return 1;



	if (match("curHotRatio", curHotRatio)) return 1;


	if (match("minHotRatio", minHotRatio)) return 1;


	if (match("ShiftLowRes", ShiftLowRes)) return 1;
	if (match("ShiftHighRes", ShiftHighRes)) return 1;

	//if (match("drivingArmAngle",drivingArmAngle)) return 1;
	//if (match("pickupArmAngle",pickupArmAngle )) return 1;

	if (match("armManualFlag", armManualFlag)) return 1;
	if (match("rapidShootElevSpeed", rapidShootElevSpeed)) return 1;
	if (match("gyroShift", gyroShift)) return 1;
	if (match("gyroShiftState", gyroShiftState)) return 1;
	if (match("elevBrake", elevBrake)) return 1;

	if (match("pauseVideo", pauseVideo)) return 1;



	if (match("wantedShooterRPM", wantedShooterRPM)) return 1;

	if (match("shooterRPM", shooterRPM)) return 1;
	if (match("shooterSpeed", shooterSpeed)) return 1;


	if (match("minDisplayLine", minDisplayLine)) return 1;
	if (match("maxDisplayLine", maxDisplayLine)) return 1;
	if (match("blackAndWhite", blackAndWhite)) return 1;


	if (match("a1_Cmd", a1_Cmd)) return 1;
	if (match("a2_Cmd", a2_Cmd)) return 1;
	if (match("a3_Cmd", a3_Cmd)) return 1;
	if (match("a4_Cmd", a4_Cmd)) return 1;
	if (match("a5_Cmd", a5_Cmd)) return 1;
	if (match("a6_Cmd", a6_Cmd)) return 1;



	if (match("cameraSelect", cameraSelect)) return 1;


	if (match("elevUpRate", elevUpRate)) return 1;
	if (match("elevDownRate", elevDownRate)) return 1;
	if (match("DriveModeFlag", DriveModeFlag)) return 1;




	if (match("rollerState", rollerState)) return 1;
	if (match("shotErrorLimit", shotErrorLimit)) return 1;

	if (match("trackTimeOut", trackTimeOut)) return 1;
	if (match("trackLockMin", trackLockMin)) return 1;
	if (match("trackCenterY", trackCenterY)) return 1;
	if (match("trackCenterX", trackCenterX)) return 1;
	if (match("trackState", trackState)) return 1;
	if (match("gfMinLevel", gfMinLevel)) return 1;
	if (match("bfMaxWide", bfMaxWide)) return 1;
	if (match("bfMaxHigh", bfMaxHigh)) return 1;
	if (match("bfMaxRatio", bfMaxRatio)) return 1;
	if (match("trackX", trackX)) return 1;
	if (match("trackY", trackY)) return 1;
	if (match("trackZ", trackZ)) return 1;

	//  if (match("shoesFlag"         ,shoesFlag         )) return 1;


	if (match("ballDetect2", ballDetect2)) return 1;
	if (match("armFlag", armFlag)) return 1;
	if (match("armSpeed", armSpeed)) return 1;
	if (match("lookForHotGoal", lookForHotGoal)) return 1;
	if (match("hotGoal", hotGoal)) return 1;
	if (match("crossHairX", crossHairX)) return 1;
	if (match("crossHairOffset", crossHairOffset)) return 1;
	//  if (match("mTargetA"             ,mTargetA             )) return 1;

	if (match("elevFlag", elevFlag)) return 1;

	if (match("lightsOffTime", lightsOffTime)) return 1;
	if (match("lightsOnTime", lightsOnTime)) return 1;


	if (match("useLeftPot", useLeftPot)) return 1;

	if (match("wantedArmAngle", wantedArmAngle)) return 1;

	if (match("crossHair2X", crossHair2X)) return 1;
	if (match("crossHair2Y", crossHair2Y)) return 1;

	if (match("sweepRate", sweepRate)) return 1;
	if (match("ShooterPulseMaxRPM", ShooterPulseMaxRPM)) return 1;
	if (match("ShooterPulseOffset", ShooterPulseOffset)) return 1;

	//  if (match("lightsOffTimeInZone"        ,lightsOffTimeInZone       )) return 1;
   //   if (match("lightsOnTimeInZone"       ,lightsOnTimeInZone      )) return 1;
	if (match("AppReady", AppReady)) return 1;

	if (match("AppStateTop", AppStateTop)) return 1;
	if (match("AppStateMid", AppStateMid)) return 1;
	if (match("AppStateBot", AppStateBot)) return 1;

	if (match("DistAdjH1", DistAdjH1)) return 1;
	if (match("DistAdjH2", DistAdjH2)) return 1;
	if (match("DistAdjY1", DistAdjY1)) return 1;
	if (match("DistAdjY2", DistAdjY2)) return 1;
	if (match("DistAdjDistH1", DistAdjDistH1)) return 1;
	if (match("DistAdjDistH2", DistAdjDistH2)) return 1;
	if (match("DistAdjDistY1", DistAdjDistY1)) return 1;
	if (match("DistAdjDistY2", DistAdjDistY2)) return 1;



	if (match("tIndex", tIndex)) return 1;
	if (match("tValue1", tValue1)) return 1;
	if (match("tValue2", tValue2)) return 1;
	if (match("tValue3", tValue3)) return 1;
	if (match("holdStraight", holdStraight)) return 1;
	if (match("tableSize", tableSize)) return 1;
	if (match("liftFlag", liftFlag)) return 1;
	if (match("liftTime", liftTime)) return 1;



	if (match("ShooterModeFlag", ShooterModeFlag)) return 1;

	if (match("RH_ShotAngBeg", RH_ShotAngBeg)) return 1;
	if (match("RH_ShotAngEnd", RH_ShotAngEnd)) return 1;
	if (match("RH_ShotSpeed", RH_ShotSpeed)) return 1;
	if (match("trussShotDist", trussShotDist)) return 1;

	if (match("ahArmAngle", ahArmAngle)) return 1;
	if (match("ahStartSpitDelay", ahStartSpitDelay)) return 1;
	if (match("ahStopSpitDelay", ahStopSpitDelay)) return 1;
	if (match("ahStartDownDelay", ahStartDownDelay)) return 1;
	if (match("ahStartArmUpDelay", ahStartArmUpDelay)) return 1;

	if (match("maxManualSpitTime", maxManualSpitTime)) return 1;
	if (match("deployReady", deployReady)) return 1;

	if (match("crossHairX", crossHairX)) return 1;

	//if (match("mTargetA",mTargetA)) return 1;

	if (match("useLeftDist", useLeftDist)) return 1;

	if (match("rightDriveAdj", rightDriveAdj)) return 1;
	if (match("leftDriveAdj", leftDriveAdj)) return 1;

	if (match("lineTrackAdj", lineTrackAdj)) return 1;

	if (match("rightDriveAdjN", rightDriveAdjN)) return 1;
	if (match("leftDriveAdjN", leftDriveAdjN)) return 1;

	if (match("packetSize", packetSize)) return 1;

	if (match("autoDeployTime", autoDeployTime)) return 1;

	if (match("hoodGain", hoodGain)) return 1;
	if (match("hoodMin", hoodMin)) return 1;
	if (match("hoodMax", hoodMax)) return 1;




	if (match("spare1", spare1)) return 1;
	if (match("spare2", spare2)) return 1;
	if (match("spare3", spare3)) return 1;
	if (match("spare4", spare4)) return 1;
	if (match("spare5", spare5)) return 1;


	if (match("inAton", inAton)) return 1;
	if (match("inTele", inTele)) return 1;
	if (match("inDisable", inDisable)) return 1;






	if (match("sSpeed0", sSpeed[0])) return 1;
	if (match("sSpeed1", sSpeed[1])) return 1;
	if (match("sSpeed2", sSpeed[2])) return 1;
	if (match("sSpeed3", sSpeed[3])) return 1;
	if (match("sSpeed4", sSpeed[4])) return 1;
	if (match("sSpeed5", sSpeed[5])) return 1;
	if (match("sSpeed6", sSpeed[6])) return 1;
	if (match("sSpeed7", sSpeed[7])) return 1;
	if (match("sSpeed8", sSpeed[8])) return 1;
	if (match("sSpeed9", sSpeed[9])) return 1;
	if (match("sSpeed10", sSpeed[10])) return 1;
	if (match("sSpeed11", sSpeed[11])) return 1;
	if (match("sSpeed12", sSpeed[12])) return 1;
	if (match("sSpeed13", sSpeed[13])) return 1;
	if (match("sSpeed14", sSpeed[14])) return 1;
	if (match("sSpeed15", sSpeed[15])) return 1;
	if (match("sSpeed16", sSpeed[16])) return 1;
	if (match("sSpeed17", sSpeed[17])) return 1;
	if (match("sSpeed18", sSpeed[18])) return 1;
	if (match("sSpeed19", sSpeed[19])) return 1;
	if (match("sHood0", sHood[0])) return 1;
	if (match("sHood1", sHood[1])) return 1;
	if (match("sHood2", sHood[2])) return 1;
	if (match("sHood3", sHood[3])) return 1;
	if (match("sHood4", sHood[4])) return 1;
	if (match("sHood5", sHood[5])) return 1;
	if (match("sHood6", sHood[6])) return 1;
	if (match("sHood7", sHood[7])) return 1;
	if (match("sHood8", sHood[8])) return 1;
	if (match("sHood9", sHood[9])) return 1;
	if (match("sHood10", sHood[10])) return 1;
	if (match("sHood11", sHood[11])) return 1;
	if (match("sHood12", sHood[12])) return 1;
	if (match("sHood13", sHood[13])) return 1;
	if (match("sHood14", sHood[14])) return 1;
	if (match("sHood15", sHood[15])) return 1;
	if (match("sHood16", sHood[16])) return 1;
	if (match("sHood17", sHood[17])) return 1;
	if (match("sHood18", sHood[18])) return 1;
	if (match("sHood19", sHood[19])) return 1;






	if (match("tCX1", tCX1)) return 1;
	if (match("tCX2", tCX2)) return 1;
	if (match("tCX3", tCX3)) return 1;
	if (match("tCX4", tCX4)) return 1;
	if (match("tCY1", tCY1)) return 1;
	if (match("tCY2", tCY2)) return 1;
	if (match("tCY3", tCY3)) return 1;
	if (match("tCY4", tCY4)) return 1;
	if (match("tH1", tH1)) return 1;
	if (match("tH2", tH2)) return 1;
	if (match("tH3", tH3)) return 1;
	if (match("tH4", tH4)) return 1;
	if (match("tW1", tW1)) return 1;
	if (match("tW2", tW2)) return 1;
	if (match("tW3", tW3)) return 1;
	if (match("tW4", tW4)) return 1;

	if (match("targetOffsetY", targetOffsetY)) return 1;
	if (match("minWheelShootSpeed", minWheelShootSpeed)) return 1;


	if (match("toroIsSpitting", toroIsSpitting)) return 1;

	if (match("toroIsCollecting", toroIsCollecting)) return 1;

	if (match("toroIsDown", toroIsDown)) return 1;
	if (match("toroIsUp", toroIsUp)) return 1;


	if (match("quickShotAngle", quickShotAngle)) return 1;
	if (match("longShotDelay", longShotDelay)) return 1;
	if (match("longShotAngle", longShotAngle)) return 1;

	if (match("shotDelay", shotDelay)) return 1;








	if (match("shooterEndDuration", shooterEndDuration)) return 1;





if (match("BoilerGoalY"         ,BoilerGoalY)) return 1;
if (match("BoilerGoalX"         ,BoilerGoalX)) return 1;
if (match("GyroZ"               ,GyroZ)) return 1;
if (match("GyroY"               ,GyroY)) return 1;
if (match("GyroX"               ,GyroX)) return 1;
if (match("HopperSpeed"         ,HopperSpeed)) return 1;
if (match("HopperVolts"         ,HopperVolts)) return 1;
if (match("HopperAmps"          ,HopperAmps)) return 1;
if (match("BallIntakeAmps"      ,BallIntakeAmps)) return 1;
if (match("BallIntakeVolts"     ,BallIntakeVolts)) return 1;
if (match("BallIntakeSpeed"     ,BallIntakeSpeed)) return 1;
if (match("IntakeRightAmps"     ,IntakeRightAmps)) return 1;
if (match("IntakeLeftVolts"     ,IntakeLeftVolts)) return 1;
if (match("IntakeLeftAmps"      ,IntakeLeftAmps)) return 1;
if (match("IntakeRightVolts"    ,IntakeRightVolts)) return 1;
if (match("IntakeRightSpeed"    ,IntakeRightSpeed)) return 1;
if (match("IntakeLeftSpeed"     ,IntakeLeftSpeed)) return 1;
if (match("Winch2Volts"         ,Winch2Volts)) return 1;
if (match("Winch2Speed"         ,Winch2Speed)) return 1;
if (match("Winch2Amps"          ,Winch2Amps)) return 1;
if (match("Winch1Amps"          ,Winch1Amps)) return 1;
if (match("Winch1Volts"         ,Winch1Volts)) return 1;
if (match("Winch1Speed"         ,Winch1Speed)) return 1;
if (match("ShooterDest"         ,ShooterDest)) return 1;
if (match("ShooterAmps"         ,ShooterAmps)) return 1;
if (match("ShooterVolts"        ,ShooterVolts)) return 1;
if (match("ToolBarAmps"         ,ToolBarAmps)) return 1;
	if (match("AtonFinalRotate", AtonFinalRotate)) return 1;
	if (match("CrossY4", CrossY4)) return 1;
	if (match("CrossX4", CrossX4)) return 1;
	if (match("CreapSpeed", CreapSpeed)) return 1;
	if (match("a6_Rotate", a6_Rotate)) return 1;
	if (match("a5_Rotate", a5_Rotate)) return 1;
	if (match("a4_Rotate", a4_Rotate)) return 1;
	if (match("a3_Rotate", a3_Rotate)) return 1;
	if (match("a2_Rotate", a2_Rotate)) return 1;
	if (match("a1_Rotate", a1_Rotate)) return 1;

	if (match("TrackingGainY", TrackingGainY)) return 1;
	if (match("TrackingY", TrackingY)) return 1;
	if (match("TrackingX", TrackingX)) return 1;
	if (match("TrackingGain", TrackingGain)) return 1;
	if (match("gyro_Zero", gyro_Zero)) return 1;
	if (match("RotateLeftAngle", RotateLeftAngle)) return 1;
	if (match("RotateRightAngle", RotateRightAngle)) return 1;
	if (match("CrossY3", CrossY3)) return 1;
	if (match("CrossY2", CrossY2)) return 1;
	if (match("CrossY1", CrossY1)) return 1;
	if (match("CrossX3", CrossX3)) return 1;
	if (match("CrossX2", CrossX2)) return 1;
	if (match("CrossX1", CrossX1)) return 1;
	if (match("GyroGain", GyroGain)) return 1;
	if (match("ToolBarLoc", ToolBarLoc)) return 1;
	if (match("ToolBarError", ToolBarError)) return 1;
	if (match("ToolBarVolts", ToolBarVolts)) return 1;
	if (match("ToolBarDest", ToolBarDest)) return 1;
	if (match("ToolBarAuto", ToolBarAuto)) return 1;
	if (match("ToolBarScale", ToolBarScale)) return 1;
	if (match("ToolBarHigh", ToolBarHigh)) return 1;
	if (match("ToolBarMid", ToolBarMid)) return 1;
	if (match("ToolBarLow", ToolBarLow)) return 1;
	if (match("GyroLoc", GyroLoc)) return 1;

	//float beforeUpdateUseGyro = UseGyro;

	if (match("UseGyro", UseGyro))
	{

		//if (UseGyro!=beforeUpdateUseGyro) resetGyro();

		return 1;
	}


	if (match("LookForGoal", LookForGoal)) return 1;



	//---------------



	return 0;

}

void LoadGlobalData()
{
	//tprintf("LoadGlobalData\n");

	LoadFile = fopen("/home/lvuser/global.dat", "r");

	//printf("LoadFile %d\n",LoadFile);

	if (LoadFile) {

		for (int L = 0; L < 1000; L++)
		{
			if (!getToken()) break;

			checkForMatch();
		}

		fclose(LoadFile);

		LoadFile = 0;
	}
	else
	{
		printf("global.dat Missing\n");

	}

	// printf("Global.Dat Loaded\n");
}

int assignValue(char *varName, float newValue)
{
	strcpy(token, varName);

	assignVal = newValue;

	assignFlag = 1;

	printf("Assign: ");
	printf(token);
	printf(" = %f",newValue);
	printf("\n");


	return checkForMatch();
}


void SaveGlobalData()
{

	printf("SaveGlobalData\n");

	FILE * pFile;

	pFile = fopen("/home/lvuser/global.dat", "w");    //

	if (pFile == 0) { printf("SaveGlobaData File Error\n"); return; }

	//  printf("pFile %d\n",(int)pFile);

	rewind(pFile);



	fprintf(pFile, "blockCanIO      %f ", blockCanIO); eol(pFile);

	fprintf(pFile, "tDisable      %f ", tDisable); eol(pFile);
	fprintf(pFile, "bDisable      %f ", bDisable); eol(pFile);
	fprintf(pFile, "aDisable      %f ", aDisable); eol(pFile);
	fprintf(pFile, "eDisable      %f ", eDisable); eol(pFile);
	fprintf(pFile, "sDisable      %f ", sDisable); eol(pFile);

	fprintf(pFile, "a2DropAngleAmz       %f ", a2DropAngleAmz); eol(pFile);
	fprintf(pFile, "a2DropAngleElbo      %f ", a2DropAngleElbo); eol(pFile);
	fprintf(pFile, "a2PickupAngleAmz     %f ", a2PickupAngleAmz); eol(pFile);
	fprintf(pFile, "a2PickupAngleElbo    %f ", a2PickupAngleElbo); eol(pFile);
	fprintf(pFile, "a2DropCanTime        %f ", a2DropCanTime); eol(pFile);

	fprintf(pFile, "lidarDisable        %f ", lidarDisable); eol(pFile);

	fprintf(pFile, "atonLimit           %f ", atonLimit); eol(pFile);



	//----------- 2015 variables ----------


	// fprintf(pFile, "tZero                 %f ", tZero); eol(pFile);

	// fprintf(pFile, "bPickupLevel          %f ", bPickupLevel); eol(pFile);
	// fprintf(pFile, "bFloorLevel           %f ", bFloorLevel); eol(pFile);
	// fprintf(pFile, "bCarryLevel           %f ", bCarryLevel); eol(pFile);
	// fprintf(pFile, "bLoadLevel            %f ", bLoadLevel); eol(pFile);
	// fprintf(pFile, "bStepLevel            %f ", bStepLevel); eol(pFile);
	// fprintf(pFile, "bStepLevelUp          %f ", bStepLevelUp); eol(pFile);
	// fprintf(pFile, "bToteLevel1           %f ", bToteLevel1); eol(pFile);
	// fprintf(pFile, "bToteLevel2           %f ", bToteLevel2); eol(pFile);
	// fprintf(pFile, "bToteLevel3           %f ", bToteLevel3); eol(pFile);
	// fprintf(pFile, "bToteLevel4           %f ", bToteLevel4); eol(pFile);
	// fprintf(pFile, "bToteLevel5           %f ", bToteLevel5); eol(pFile);
	// fprintf(pFile, "bToteLevel6           %f ", bToteLevel6); eol(pFile);
	// fprintf(pFile, "bToteTopClr           %f ", bToteTopClr); eol(pFile);
   //  fprintf(pFile, "bGain                 %f ", bGain); eol(pFile);
   //  fprintf(pFile, "bResolution           %f ", bResolution); eol(pFile);
   //  fprintf(pFile, "bMaxUpSpeed           %f ", bMaxUpSpeed); eol(pFile);
   //  fprintf(pFile, "bMaxDnSpeed           %f ", bMaxDnSpeed); eol(pFile);
   //  fprintf(pFile, "bMode                 %f ", bMode); eol(pFile);
	// fprintf(pFile, "bTimeout              %f ", bTimeout); eol(pFile);

   //  fprintf(pFile, "aPickupAngle          %f ", aPickupAngle); eol(pFile);
   //  fprintf(pFile, "aFloorAngle           %f ", aFloorAngle); eol(pFile);
   //  fprintf(pFile, "aCarryAngle           %f ", aCarryAngle); eol(pFile);
   //  fprintf(pFile, "aLoadAngle            %f ", aLoadAngle); eol(pFile);
   //  fprintf(pFile, "aStepAngle            %f ", aStepAngle); eol(pFile);
   //  fprintf(pFile, "aStepAngleUp          %f ", aStepAngleUp); eol(pFile);
   //  fprintf(pFile, "aToteAngle1           %f ", aToteAngle1); eol(pFile);
   //  fprintf(pFile, "aToteAngle2           %f ", aToteAngle2); eol(pFile);
   //  fprintf(pFile, "aToteAngle3           %f ", aToteAngle3); eol(pFile);
   //  fprintf(pFile, "aToteAngle4           %f ", aToteAngle4); eol(pFile);
   //  fprintf(pFile, "aToteAngle5           %f ", aToteAngle5); eol(pFile);
   //  fprintf(pFile, "aToteAngle6           %f ", aToteAngle6); eol(pFile);
   //  fprintf(pFile, "aToteTopClr           %f ", aToteTopClr); eol(pFile);
	// fprintf(pFile, "aGain                 %f ", aGain); eol(pFile);
	// fprintf(pFile, "aResolution           %f ", aResolution); eol(pFile);
	// fprintf(pFile, "aMaxCWSpeed           %f ", aMaxCWSpeed); eol(pFile);
	// fprintf(pFile, "aMaxCCWSpeed          %f ", aMaxCCWSpeed); eol(pFile);
	// fprintf(pFile, "aMode                 %f ", aMode); eol(pFile);
	// fprintf(pFile, "aTimeout              %f ", aTimeout); eol(pFile);

	// fprintf(pFile, "ePickupAngle          %f ", ePickupAngle); eol(pFile);
	// fprintf(pFile, "eFloorAngle           %f ", eFloorAngle); eol(pFile);
   //  fprintf(pFile, "eCarryAngle           %f ", eCarryAngle); eol(pFile);
	// fprintf(pFile, "eLoadAngle            %f ", eLoadAngle); eol(pFile);
   //  fprintf(pFile, "eStepAngle            %f ", eStepAngle); eol(pFile);
   //  fprintf(pFile, "eStepAngleUp          %f ", eStepAngleUp); eol(pFile);
	// fprintf(pFile, "eToteAngle1           %f ", eToteAngle1); eol(pFile);
	// fprintf(pFile, "eToteAngle2           %f ", eToteAngle2); eol(pFile);
	// fprintf(pFile, "eToteAngle3           %f ", eToteAngle3); eol(pFile);
	// fprintf(pFile, "eToteAngle4           %f ", eToteAngle4); eol(pFile);
	// fprintf(pFile, "eToteAngle5           %f ", eToteAngle5); eol(pFile);
	// fprintf(pFile, "eToteAngle6           %f ", eToteAngle6); eol(pFile);
	// fprintf(pFile, "eToteTopClr           %f ", eToteTopClr); eol(pFile);
	// fprintf(pFile, "eGain                 %f ", eGain); eol(pFile);
	// fprintf(pFile, "eResolution           %f ", eResolution); eol(pFile);
	// fprintf(pFile, "eMaxCWSpeed           %f ", eMaxCWSpeed); eol(pFile);
	// fprintf(pFile, "eMaxCCWSpeed          %f ", eMaxCCWSpeed); eol(pFile);
	// fprintf(pFile, "eMode                 %f ", eMode); eol(pFile);
	// fprintf(pFile, "eTimeout              %f ", eTimeout); eol(pFile);

	fprintf(pFile, "a1DriveTime           %f ", a1DriveTime); eol(pFile);
	fprintf(pFile, "a1DriveSpeed          %f ", a1DriveSpeed); eol(pFile);
	fprintf(pFile, "a1BinLevel            %f ", a1BinLevel); eol(pFile);

	fprintf(pFile, "a4DriveTime           %f ", a4DriveTime); eol(pFile);
	fprintf(pFile, "a4DriveSpeed          %f ", a4DriveSpeed); eol(pFile);
	fprintf(pFile, "a4BinLevel            %f ", a4BinLevel); eol(pFile);


	fprintf(pFile, "a5Amz                 %f ", a5Amz); eol(pFile);
	fprintf(pFile, "a5Elbo                %f ", a5Elbo); eol(pFile);
	fprintf(pFile, "atonSuckSpeed         %f ", atonSuckSpeed); eol(pFile);
	fprintf(pFile, "userSuckSpeed         %f ", userSuckSpeed); eol(pFile);

	fprintf(pFile, "a5MoveSpeed           %f ", a5MoveSpeed); eol(pFile);
	fprintf(pFile, "a5MoveTime            %f ", a5MoveTime); eol(pFile);

	fprintf(pFile, "binDropLoc            %f ", binDropLoc); eol(pFile);


	fprintf(pFile, "a2GrabAndLiftTime     %f ", a2GrabAndLiftTime); eol(pFile);
	fprintf(pFile, "a2DropOffTime         %f ", a2DropOffTime); eol(pFile);
	fprintf(pFile, "a2RightMoveTimeB1     %f ", a2RightMoveTimeB1); eol(pFile);
	fprintf(pFile, "a2RightMoveSpeedB1    %f ", a2RightMoveSpeedB1); eol(pFile);
	fprintf(pFile, "a2LeftMoveTimeB1      %f ", a2LeftMoveTimeB1); eol(pFile);
	fprintf(pFile, "a2LeftMoveSpeedB1     %f ", a2LeftMoveSpeedB1); eol(pFile);
	fprintf(pFile, "a2ForwardMoveTimeB2   %f ", a2ForwardMoveTimeB2); eol(pFile);
	fprintf(pFile, "a2ForwardMoveSpeedB2  %f ", a2ForwardMoveSpeedB2); eol(pFile);
	fprintf(pFile, "a2RightMoveTimeB2     %f ", a2RightMoveTimeB2); eol(pFile);
	fprintf(pFile, "a2RightMoveSpeedB2    %f ", a2RightMoveSpeedB2); eol(pFile);
	fprintf(pFile, "a2LeftMoveTimeB2      %f ", a2LeftMoveTimeB2); eol(pFile);
	fprintf(pFile, "a2LeftMoveSpeedB2     %f ", a2LeftMoveSpeedB2); eol(pFile);
	fprintf(pFile, "a2ForwardMoveTimeB3   %f ", a2ForwardMoveTimeB3); eol(pFile);
	fprintf(pFile, "a2RightMoveTimeB3     %f ", a2RightMoveTimeB3); eol(pFile);
	fprintf(pFile, "a2RightMoveSpeedB3    %f ", a2RightMoveSpeedB3); eol(pFile);

	fprintf(pFile, "a3ShoulderOutAngle    %f ", a3ShoulderOutAngle); eol(pFile);
	fprintf(pFile, "a2ElboOutAngle       %f ", a2ElboOutAngle); eol(pFile);
	fprintf(pFile, "a3InToGrapTime        %f ", a3InToGrapTime); eol(pFile);
	fprintf(pFile, "a3InToGrapSpeed       %f ", a3InToGrapSpeed); eol(pFile);
	fprintf(pFile, "a3OutToGrapTime       %f ", a3OutToGrapTime); eol(pFile);
	fprintf(pFile, "a3OutInToGrapSpeed    %f ", a3OutInToGrapSpeed); eol(pFile);
	fprintf(pFile, "a3ForawrdMoveTime     %f ", a3ForawrdMoveTime); eol(pFile);
	fprintf(pFile, "a3ForawrdMoveSpeed    %f ", a3ForawrdMoveSpeed); eol(pFile);
	fprintf(pFile, "a3BinDropOffTime      %f ", a3BinDropOffTime); eol(pFile);



	//-------------------------------------










	fprintf(pFile, "atonNum        %f ", atonNum); eol(pFile);

	fprintf(pFile, "slowVideoMode  %d ", slowVideoMode); eol(pFile);
	fprintf(pFile, "atonDelayTime %f ", atonDelayTime); eol(pFile);


	//fprintf (pFile, "bumperIsDown %f ",bumperIsDown);eol(pFile);
	//fprintf (pFile, "bumperIsUp %f ",bumperIsUp);eol(pFile);
	fprintf(pFile, "armGain %f ", armGain); eol(pFile);




	fprintf(pFile, "armRes    %f ", armRes); eol(pFile);

	fprintf(pFile, "toroBlockAfterShotTime %f ", toroBlockAfterShotTime); eol(pFile);


	fprintf(pFile, "holdGain %f ", holdGain); eol(pFile);

	fprintf(pFile, "holdGainR %f ", holdGainR); eol(pFile);


	fprintf(pFile, "shotBlocker %f ", shotBlocker); eol(pFile);
	fprintf(pFile, "TEAMNUMBER  %f ", teamNumber); eol(pFile);

	fprintf(pFile, "minShootTime %f ", minShootTime); eol(pFile);
	fprintf(pFile, "moveBackRate    %f ", moveBackRate); eol(pFile);

	fprintf(pFile, "turretGain %f ", turretGain); eol(pFile);

	//---------- Level Parameters -----------------

  //fprintf (pFile, "gyroShift       %f ",gyroShift        ); eol(pFile);
	fprintf(pFile, "midLevel        %f ", midLevel); eol(pFile);

	fprintf(pFile, "DistanceH         %f ", DistanceH); eol(pFile);
	fprintf(pFile, "DistanceY         %f ", DistanceY); eol(pFile);
	fprintf(pFile, "FilterHYFlag      %f ", FilterHYFlag); eol(pFile);
	fprintf(pFile, "FilterHYLimit     %f ", FilterHYLimit); eol(pFile);
	// fprintf (pFile, "PickupRollerFlag  %f ",PickupRollerFlag ); eol(pFile);
	fprintf(pFile, "PickupRollerSpeed %f ", PickupRollerSpeed); eol(pFile);
	  //fprintf (pFile, "WheelAtSpeed      %f ",WheelAtSpeed     ); eol(pFile);

	/*  fprintf(pFile, "GoalFound       %f ", GoalFound); eol(pFile);
	  fprintf(pFile, "GoalX		      %f ", GoalX    ); eol(pFile);
	  fprintf(pFile, "GoalY		      %f ", GoalY    ); eol(pFile);
	  fprintf(pFile, "GoalW		      %f ", GoalW    ); eol(pFile);
	  fprintf(pFile, "GoalH		      %f ", GoalH); eol(pFile);*/




	  // fprintf (pFile, "armManualFlag       %f ",armManualFlag      ); eol(pFile);
	fprintf(pFile, "rapidShootElevSpeed  %f ", rapidShootElevSpeed); eol(pFile);
	fprintf(pFile, "gyroShift       %f ", gyroShift); eol(pFile);
	//fprintf (pFile, "gyroShiftState    %f ",gyroShiftState   ); eol(pFile);
	fprintf(pFile, "elevBrake       %f ", elevBrake); eol(pFile);


	fprintf(pFile, "shoesOut2         %f ", shoesOut2); eol(pFile);
	fprintf(pFile, "shoesIn2         %f ", shoesIn2); eol(pFile);





	 // fprintf (pFile, "RH_ShotAngBeg      %f ",RH_ShotAngBeg      ); eol(pFile);
	 // fprintf (pFile, "RH_ShotAngEnd      %f ",RH_ShotAngEnd       ); eol(pFile);
	 // fprintf (pFile, "RH_ShotSpeed       %f ",RH_ShotSpeed        ); eol(pFile);

	//  fprintf (pFile, "A_ShotDistBeg       %f ",A_ShotDistBeg       ); eol(pFile);
	//  fprintf (pFile, "A_ShotDistZone       %f ",A_ShotDistZone       ); eol(pFile);
	//  fprintf (pFile, "B_ShotDistBeg       %f ",B_ShotDistBeg       ); eol(pFile);
	//  fprintf (pFile, "B_ShotDistZone       %f ",B_ShotDistZone       ); eol(pFile);
	//  fprintf (pFile, "X_ShotDistBeg       %f ",X_ShotDistBeg       ); eol(pFile);
	//  fprintf (pFile, "X_ShotDistZone       %f ",X_ShotDistZone       ); eol(pFile);
	//  fprintf (pFile, "Y_ShotDistBeg       %f ",Y_ShotDistBeg       ); eol(pFile);
	//  fprintf (pFile, "Y_ShotDistZone       %f ",Y_ShotDistZone       ); eol(pFile);
	//  fprintf (pFile, "RS_ShotDistBeg      %f ",RS_ShotDistBeg      ); eol(pFile);
	//  fprintf (pFile, "RS_ShotDistZone      %f ",RS_ShotDistZone      ); eol(pFile);
	//  fprintf (pFile, "LH_ShotDistBeg      %f ",LH_ShotDistBeg      ); eol(pFile);
	//  fprintf (pFile, "LH_ShotDistZone      %f ",LH_ShotDistZone      ); eol(pFile);
	 // fprintf (pFile, "RH_ShotDistBeg      %f ",RH_ShotDistBeg      ); eol(pFile);
	 // fprintf (pFile, "RH_ShotDistZone      %f ",RH_ShotDistZone      ); eol(pFile);





	fprintf(pFile, "minWheelShootSpeed %f ", minWheelShootSpeed); eol(pFile);


	fprintf(pFile, "toroIsSpitting %f ", toroIsSpitting); eol(pFile);

	fprintf(pFile, "toroIsCollecting %f ", toroIsCollecting); eol(pFile);
	fprintf(pFile, "minHotRatio %f ", minHotRatio); eol(pFile);


	// fprintf( pFile, "toroIsCollecting %f ",toroIsCollecting); eol(pFile);
	// fprintf( pFile, "toroIsUp %f ",toroIsUp); eol(pFile);



	fprintf(pFile, "quickShotAngle %f ", quickShotAngle); eol(pFile);
	fprintf(pFile, "longShotDelay %f ", longShotDelay); eol(pFile);
	fprintf(pFile, "longShotAngle %f ", longShotAngle); eol(pFile);

	fprintf(pFile, "shotDelay %f ", shotDelay); eol(pFile);



	fprintf(pFile, "ShiftLowRes       %f ", ShiftLowRes); eol(pFile);
	fprintf(pFile, "ShiftHighRes      %f ", ShiftHighRes); eol(pFile);

	//---------- Roller Parameters ---------------


	 //fprintf (pFile, "wantedShooterRPM    %f ",wantedShooterRPM     ); eol(pFile);

	fprintf(pFile, "shooterRPM         %f ", shooterRPM); eol(pFile);
	fprintf(pFile, "shooterSpeed       %f ", shooterSpeed); eol(pFile);

	fprintf(pFile, "minDisplayLine      %f ", minDisplayLine); eol(pFile);
	fprintf(pFile, "maxDisplayLine      %f ", maxDisplayLine); eol(pFile);
	fprintf(pFile, "blackAndWhite      %f ", blackAndWhite); eol(pFile);



	//------------ Aton Parameters ----------------

	fprintf(pFile, "a1_Cmd       %f ", a1_Cmd); eol(pFile);
	fprintf(pFile, "a2_Cmd       %f ", a2_Cmd); eol(pFile);
	fprintf(pFile, "a3_Cmd       %f ", a3_Cmd); eol(pFile);
	fprintf(pFile, "a4_Cmd       %f ", a4_Cmd); eol(pFile);
	fprintf(pFile, "a5_Cmd       %f ", a5_Cmd); eol(pFile);
	fprintf(pFile, "a6_Cmd       %f ", a6_Cmd); eol(pFile);



	// fprintf (pFile, "cameraSelect    %f ",cameraSelect   ); eol(pFile);

	fprintf(pFile, "elevUpRate   %f ", elevUpRate); eol(pFile);
	fprintf(pFile, "elevDownRate %f ", elevDownRate); eol(pFile);

	// fprintf (pFile, "DriveModeFlag %f ",DriveModeFlag); eol(pFile);


	fprintf(pFile, "shotErrorLimit %f ", shotErrorLimit); eol(pFile);



	fprintf(pFile, "trackTimeOut %f ", trackTimeOut); eol(pFile);
	fprintf(pFile, "trackLockMin %f ", trackLockMin); eol(pFile);
	fprintf(pFile, "trackCenterY %f ", trackCenterY); eol(pFile);
	fprintf(pFile, "trackCenterX %f ", trackCenterX); eol(pFile);
	fprintf(pFile, "trackState   %f ", trackState); eol(pFile);
	fprintf(pFile, "gfMinLevel   %f ", gfMinLevel); eol(pFile);
	fprintf(pFile, "bfMaxWide    %f ", bfMaxWide); eol(pFile);
	fprintf(pFile, "bfMaxHigh    %f ", bfMaxHigh); eol(pFile);
	fprintf(pFile, "bfMaxRatio   %f ", bfMaxRatio); eol(pFile);
	//  fprintf(pFile,"trackX       %f ",trackX      ); eol(pFile);
	//  fprintf(pFile,"trackY       %f ",trackY      ); eol(pFile);
	 // fprintf(pFile,"trackZ       %f ",trackZ      ); eol(pFile);


	fprintf(pFile, "useLeftPot   %f ", useLeftPot); eol(pFile);

	//  fprintf(pFile,"wantedArmAngle    %f ",wantedArmAngle   ); eol(pFile);
	fprintf(pFile, "crossHair2X %f ", crossHair2X); eol(pFile);
	fprintf(pFile, "crossHair2Y %f ", crossHair2Y); eol(pFile);


	fprintf(pFile, "sweepRate %f ", sweepRate); eol(pFile);
	fprintf(pFile, "ShooterPulseMaxRPM   %f ", ShooterPulseMaxRPM); eol(pFile);
	fprintf(pFile, "ShooterPulseOffset   %f ", ShooterPulseOffset); eol(pFile);

	fprintf(pFile, "DistAdjH1       %f ", DistAdjH1); eol(pFile);
	fprintf(pFile, "DistAdjH2       %f ", DistAdjH2); eol(pFile);
	fprintf(pFile, "DistAdjY1       %f ", DistAdjY1); eol(pFile);
	fprintf(pFile, "DistAdjY2       %f ", DistAdjY2); eol(pFile);
	fprintf(pFile, "DistAdjDistH1   %f ", DistAdjDistH1); eol(pFile);
	fprintf(pFile, "DistAdjDistH2   %f ", DistAdjDistH2); eol(pFile);
	fprintf(pFile, "DistAdjDistY1   %f ", DistAdjDistY1); eol(pFile);
	fprintf(pFile, "DistAdjDistY2   %f ", DistAdjDistY2); eol(pFile);

	//  fprintf(pFile,"tIndex   %f ",tIndex  ); eol(pFile);
	//  fprintf(pFile,"tValue1   %f ",tValue1  ); eol(pFile);
	fprintf(pFile, "tValue2   %f ", tValue2); eol(pFile);
	fprintf(pFile, "tValue3   %f ", tValue3); eol(pFile);
	// fprintf(pFile,"holdStraight   %f ",holdStraight  ); eol(pFile);
   //  fprintf(pFile,"tableSize   %f ",tableSize  ); eol(pFile);
   //  fprintf(pFile,"liftFlag   %f ",liftFlag  ); eol(pFile);
	fprintf(pFile, "liftTime   %f ", liftTime); eol(pFile);


	fprintf(pFile, "lightsOffTimeInZone         %f ", lightsOffTimeInZone); eol(pFile);
	fprintf(pFile, "lightsOnTimeInZone        %f ", lightsOnTimeInZone); eol(pFile);
	fprintf(pFile, "AppReady        %f ", AppReady); eol(pFile);

	fprintf(pFile, "AppStateTop        %f ", AppStateTop); eol(pFile);
	fprintf(pFile, "AppStateMid        %f ", AppStateMid); eol(pFile);
	fprintf(pFile, "AppStateBot        %f ", AppStateBot); eol(pFile);

	fprintf(pFile, "RH_ShotAngBeg          %f ", RH_ShotAngBeg); eol(pFile);
	fprintf(pFile, "RH_ShotAngEnd       %f ", RH_ShotAngEnd); eol(pFile);
	fprintf(pFile, "RH_ShotSpeed      %f ", RH_ShotSpeed); eol(pFile);
	fprintf(pFile, "trussShotDist %f ", trussShotDist); eol(pFile);


	// fprintf(pFile,"shoesFlag         %f ",shoesFlag     ); eol(pFile);

	//   fprintf(pFile,"ballDetect2       %f ",ballDetect2      ); eol(pFile);
	//   fprintf(pFile,"armFlag           %f ",armFlag          ); eol(pFile);
	fprintf(pFile, "armSpeed          %f ", armSpeed); eol(pFile);
	// fprintf(pFile,"lookForHotGoal        %f ",lookForHotGoal       ); eol(pFile);
	 //fprintf(pFile,"hotGoal       %f ",hotGoal      ); eol(pFile);
	fprintf(pFile, "crossHairX        %f ", crossHairX); eol(pFile);
	fprintf(pFile, "crossHairOffset   %f ", crossHairOffset); eol(pFile);
	// fprintf(pFile,"mTargetA          %f ",mTargetA         ); eol(pFile);
	fprintf(pFile, "lightsOffTime    %f ", lightsOffTime); eol(pFile);

	fprintf(pFile, "lightsOnTime      %f", lightsOnTime); eol(pFile);




	fprintf(pFile, "ahArmAngle          %f ", ahArmAngle); eol(pFile);
	fprintf(pFile, "ahStartSpitDelay    %f ", ahStartSpitDelay); eol(pFile);
	fprintf(pFile, "ahStopSpitDelay     %f ", ahStopSpitDelay); eol(pFile);
	fprintf(pFile, "ahStartDownDelay    %f ", ahStartDownDelay); eol(pFile);
	fprintf(pFile, "ahStartArmUpDelay   %f ", ahStartArmUpDelay); eol(pFile);

	fprintf(pFile, "maxManualSpitTime   %f ", maxManualSpitTime); eol(pFile);

	fprintf(pFile, "crossHairX          %f ", crossHairX); eol(pFile);

	fprintf(pFile, "useLeftDist         %f ", useLeftDist); eol(pFile);

	fprintf(pFile, "rightDriveAdj       %f ", rightDriveAdj); eol(pFile);
	fprintf(pFile, "leftDriveAdj        %f ", leftDriveAdj); eol(pFile);

	fprintf(pFile, "lineTrackAdj        %f ", lineTrackAdj); eol(pFile);

	fprintf(pFile, "rightDriveAdjN      %f ", rightDriveAdjN); eol(pFile);
	fprintf(pFile, "leftDriveAdjN       %f ", leftDriveAdjN); eol(pFile);

	fprintf(pFile, "packetSize     %f ", packetSize); eol(pFile);

	fprintf(pFile, "autoDeployTime      %f ", autoDeployTime); eol(pFile);

	fprintf(pFile, "hoodGain     %f ", hoodGain); eol(pFile);
	fprintf(pFile, "hoodMin      %f ", hoodMin); eol(pFile);
	fprintf(pFile, "hoodMax      %f ", hoodMax); eol(pFile);

	fprintf(pFile, "spare1       %f ", spare1); eol(pFile);
	fprintf(pFile, "spare2       %f ", spare2); eol(pFile);
	fprintf(pFile, "spare3       %f ", spare3); eol(pFile);
	fprintf(pFile, "spare4       %f ", spare4); eol(pFile);
	fprintf(pFile, "spare5       %f ", spare5); eol(pFile);




	fprintf(pFile, "sSpeed0     %f ", sSpeed[0]); eol(pFile);
	fprintf(pFile, "sSpeed1     %f ", sSpeed[1]); eol(pFile);
	fprintf(pFile, "sSpeed2     %f ", sSpeed[2]); eol(pFile);
	fprintf(pFile, "sSpeed3     %f ", sSpeed[3]); eol(pFile);
	fprintf(pFile, "sSpeed4     %f ", sSpeed[4]); eol(pFile);
	fprintf(pFile, "sSpeed5     %f ", sSpeed[5]); eol(pFile);
	fprintf(pFile, "sSpeed6     %f ", sSpeed[6]); eol(pFile);
	fprintf(pFile, "sSpeed7     %f ", sSpeed[7]); eol(pFile);
	fprintf(pFile, "sSpeed8     %f ", sSpeed[8]); eol(pFile);
	fprintf(pFile, "sSpeed9     %f ", sSpeed[9]); eol(pFile);
	fprintf(pFile, "sSpeed10    %f ", sSpeed[10]); eol(pFile);
	fprintf(pFile, "sSpeed11    %f ", sSpeed[11]); eol(pFile);
	fprintf(pFile, "sSpeed12    %f ", sSpeed[12]); eol(pFile);
	fprintf(pFile, "sSpeed13    %f ", sSpeed[13]); eol(pFile);
	fprintf(pFile, "sSpeed14    %f ", sSpeed[14]); eol(pFile);
	fprintf(pFile, "sSpeed15    %f ", sSpeed[15]); eol(pFile);
	fprintf(pFile, "sSpeed16    %f ", sSpeed[16]); eol(pFile);
	fprintf(pFile, "sSpeed17    %f ", sSpeed[17]); eol(pFile);
	fprintf(pFile, "sSpeed18    %f ", sSpeed[18]); eol(pFile);
	fprintf(pFile, "sSpeed19    %f ", sSpeed[19]); eol(pFile);
	fprintf(pFile, "sHood0      %f ", sHood[0]); eol(pFile);
	fprintf(pFile, "sHood1      %f ", sHood[1]); eol(pFile);
	fprintf(pFile, "sHood2      %f ", sHood[2]); eol(pFile);
	fprintf(pFile, "sHood3      %f ", sHood[3]); eol(pFile);
	fprintf(pFile, "sHood4      %f ", sHood[4]); eol(pFile);
	fprintf(pFile, "sHood5      %f ", sHood[5]); eol(pFile);
	fprintf(pFile, "sHood6      %f ", sHood[6]); eol(pFile);
	fprintf(pFile, "sHood7      %f ", sHood[7]); eol(pFile);
	fprintf(pFile, "sHood8      %f ", sHood[8]); eol(pFile);
	fprintf(pFile, "sHood9      %f ", sHood[9]); eol(pFile);
	fprintf(pFile, "sHood10     %f ", sHood[10]); eol(pFile);
	fprintf(pFile, "sHood11     %f ", sHood[11]); eol(pFile);
	fprintf(pFile, "sHood12     %f ", sHood[12]); eol(pFile);
	fprintf(pFile, "sHood13     %f ", sHood[13]); eol(pFile);
	fprintf(pFile, "sHood14     %f ", sHood[14]); eol(pFile);
	fprintf(pFile, "sHood15     %f ", sHood[15]); eol(pFile);
	fprintf(pFile, "sHood16     %f ", sHood[16]); eol(pFile);
	fprintf(pFile, "sHood17     %f ", sHood[17]); eol(pFile);
	fprintf(pFile, "sHood18     %f ", sHood[18]); eol(pFile);
	fprintf(pFile, "sHood19     %f ", sHood[19]); eol(pFile);


	fprintf(pFile, "curHotRatio %f ", curHotRatio); eol(pFile);





	fprintf(pFile, "shooterEndDuration   %f ", shooterEndDuration); eol(pFile);

	//SaveValues

fprintf (pFile, "move_OverHeadDroppoff %f ",move_OverHeadDroppoff); eol(pFile);
fprintf (pFile, "TwoCubePickupAngle %f ",TwoCubePickupAngle); eol(pFile);
fprintf (pFile, "move_TwoCubeDistance %f ",move_TwoCubeDistance); eol(pFile);
fprintf (pFile, "move_CornerToScale %f ",move_CornerToScale); eol(pFile);
fprintf (pFile, "strafe_behindHeadDropoff %f ",strafe_behindHeadDropoff); eol(pFile);
fprintf (pFile, "strafe_secondCubePickup %f ",strafe_secondCubePickup); eol(pFile);
fprintf (pFile, "strafe_firstCubePickup %f ",strafe_firstCubePickup); eol(pFile);
fprintf (pFile, "LiftScaleBehind %f ",LiftScaleBehind); eol(pFile);
fprintf (pFile, "ToolBarScaleBehind %f ",ToolBarScaleBehind); eol(pFile);
fprintf (pFile, "ultra3Distance %f ",ultra3Distance); eol(pFile);
fprintf (pFile, "ultra2Distance %f ",ultra2Distance); eol(pFile);
fprintf (pFile, "ultra1Distance %f ",ultra1Distance); eol(pFile);
fprintf (pFile, "move_CornerDropCube %f ",move_CornerDropCube); eol(pFile);
fprintf (pFile, "move_CrossDropCube %f ",move_CrossDropCube); eol(pFile);
fprintf (pFile, "HoldCube   %f ",HoldCube); eol(pFile);
fprintf (pFile, "TwoCube    %f ",TwoCube); eol(pFile);
fprintf (pFile, "LiftClimbSide %f ",LiftClimbSide); eol(pFile);
fprintf (pFile, "ToolBarClimbSide %f ",ToolBarClimbSide); eol(pFile);
fprintf (pFile, "Crossover  %f ",Crossover); eol(pFile);
fprintf (pFile, "LiftClimb  %f ",LiftClimb); eol(pFile);
fprintf (pFile, "ToolBarClimb %f ",ToolBarClimb); eol(pFile);
fprintf (pFile, "favorScale %f ",favorScale); eol(pFile);
fprintf (pFile, "move_ScalefromBehind %f ",move_ScalefromBehind); eol(pFile);
fprintf (pFile, "move_FaceScale %f ",move_FaceScale); eol(pFile);
fprintf (pFile, "move_SwitchfromBehind %f ",move_SwitchfromBehind); eol(pFile);
fprintf (pFile, "move_BehindSwitch %f ",move_BehindSwitch); eol(pFile);
fprintf (pFile, "move_SideCenterofSwitch %f ",move_SideCenterofSwitch); eol(pFile);
fprintf (pFile, "move_TouchSwitch %f ",move_TouchSwitch); eol(pFile);
fprintf (pFile, "move_CenterofSwitch %f ",move_CenterofSwitch); eol(pFile);
fprintf (pFile, "move_ApproachSwitch %f ",move_ApproachSwitch); eol(pFile);
fprintf (pFile, "matchNumber %f ",matchNumber); eol(pFile);
fprintf (pFile, "matchTimeLeft %f ",matchTimeLeft); eol(pFile);
fprintf (pFile, "ToolBarPickup %f ",ToolBarPickup); eol(pFile);
fprintf (pFile, "robotWatts %f ",robotWatts); eol(pFile);
fprintf (pFile, "EncoderGain %f ",EncoderGain); eol(pFile);
fprintf (pFile, "LiftGain   %f ",LiftGain); eol(pFile);
fprintf (pFile, "ToolBarGain %f ",ToolBarGain); eol(pFile);
fprintf (pFile, "TuningVar5 %f ",TuningVar5); eol(pFile);
fprintf (pFile, "TuningVar4 %f ",TuningVar4); eol(pFile);
fprintf (pFile, "TuningVar3 %f ",TuningVar3); eol(pFile);
fprintf (pFile, "TuningVar2 %f ",TuningVar2); eol(pFile);
fprintf (pFile, "TuningVar1 %f ",TuningVar1); eol(pFile);
fprintf (pFile, "ToolBarScaleHigh %f ",ToolBarScaleHigh); eol(pFile);
fprintf (pFile, "ToolBarScaleMid %f ",ToolBarScaleMid); eol(pFile);
fprintf (pFile, "ToolBarScaleLow %f ",ToolBarScaleLow); eol(pFile);
fprintf (pFile, "ToolBarSwitch %f ",ToolBarSwitch); eol(pFile);
fprintf (pFile, "LiftScaleHigh %f ",LiftScaleHigh); eol(pFile);
fprintf (pFile, "LiftScaleMid %f ",LiftScaleMid); eol(pFile);
fprintf (pFile, "LiftScaleLow %f ",LiftScaleLow); eol(pFile);
fprintf (pFile, "LiftSwitch %f ",LiftSwitch); eol(pFile);
fprintf (pFile, "LiftLow    %f ",LiftLow); eol(pFile);
fprintf (pFile, "LiftDest   %f ",LiftDest); eol(pFile);
fprintf (pFile, "ToolBarSpeed %f ",ToolBarSpeed); eol(pFile);
fprintf (pFile, "Lift2Volts %f ",Lift2Volts); eol(pFile);
fprintf (pFile, "Lift1Volts %f ",Lift1Volts); eol(pFile);
fprintf (pFile, "Lift2Amps  %f ",Lift2Amps); eol(pFile);
fprintf (pFile, "Lift1Amps  %f ",Lift1Amps); eol(pFile);
fprintf (pFile, "Lift2Speed %f ",Lift2Speed); eol(pFile);
fprintf (pFile, "Lift1Speed %f ",Lift1Speed); eol(pFile);
fprintf (pFile, "ToolBarAngle %f ",ToolBarAngle); eol(pFile);
fprintf (pFile, "CubeIntakeAngle %f ",CubeIntakeAngle); eol(pFile);
fprintf (pFile, "LiftHeight %f ",LiftHeight); eol(pFile);
fprintf (pFile, "BatteryVoltage %f ",BatteryVoltage); eol(pFile);
fprintf (pFile, "RBD2Amps   %f ",RBD2Amps); eol(pFile);
fprintf (pFile, "RBD1Amps   %f ",RBD1Amps); eol(pFile);
fprintf (pFile, "RFD2Amps   %f ",RFD2Amps); eol(pFile);
fprintf (pFile, "RFD1Amps   %f ",RFD1Amps); eol(pFile);
fprintf (pFile, "LBD2Amps   %f ",LBD2Amps); eol(pFile);
fprintf (pFile, "LBD1Amps   %f ",LBD1Amps); eol(pFile);
fprintf (pFile, "LFD2Amps   %f ",LFD2Amps); eol(pFile);
fprintf (pFile, "LFD1Amps   %f ",LFD1Amps); eol(pFile);
fprintf (pFile, "LFD1Speed  %f ",LFD1Speed); eol(pFile);
fprintf (pFile, "RBD2Speed  %f ",RBD2Speed); eol(pFile);
fprintf (pFile, "RBD1Speed  %f ",RBD1Speed); eol(pFile);
fprintf (pFile, "LBD2Speed  %f ",LBD2Speed); eol(pFile);
fprintf (pFile, "LBD1Speed  %f ",LBD1Speed); eol(pFile);
fprintf (pFile, "RFD2Speed  %f ",RFD2Speed); eol(pFile);
fprintf (pFile, "RFD1Speed  %f ",RFD1Speed); eol(pFile);
fprintf (pFile, "LFD2Speed  %f ",LFD2Speed); eol(pFile);
fprintf (pFile, "EncoderVelocity %f ",EncoderVelocity); eol(pFile);
fprintf (pFile, "EncoderDistance %f ",EncoderDistance); eol(pFile);
fprintf (pFile, "aton_HopperRPM %f ",aton_HopperRPM); eol(pFile);
fprintf (pFile, "aton_ShooterRPM %f ",aton_ShooterRPM); eol(pFile);
fprintf (pFile, "maxGoalAimSpeed %f ",maxGoalAimSpeed); eol(pFile);
fprintf (pFile, "GoalAimGain %f ",GoalAimGain); eol(pFile);
fprintf (pFile, "hopperShootRPM %f ",hopperShootRPM); eol(pFile);
fprintf (pFile, "gearDropoffSpeed %f ",gearDropoffSpeed); eol(pFile);
fprintf (pFile, "gearPickupSpeed %f ",gearPickupSpeed); eol(pFile);
fprintf (pFile, "maxPegAimSpeed %f ",maxPegAimSpeed); eol(pFile);
fprintf (pFile, "maxPegDriveSpeed %f ",maxPegDriveSpeed); eol(pFile);
fprintf (pFile, "pegGoalDriveScale %f ",pegGoalDriveScale); eol(pFile);
fprintf (pFile, "pegGoalAimScale %f ",pegGoalAimScale); eol(pFile);
fprintf (pFile, "camerafps  %f ",camerafps); eol(pFile);
fprintf (pFile, "PegGoalY1  %f ",PegGoalY1); eol(pFile);
fprintf (pFile, "PegGoalX1  %f ",PegGoalX1); eol(pFile);
fprintf (pFile, "BoilerGoalY3 %f ",BoilerGoalY3); eol(pFile);
fprintf (pFile, "BoilerGoalY2 %f ",BoilerGoalY2); eol(pFile);
fprintf (pFile, "BoilerGoalY1 %f ",BoilerGoalY1); eol(pFile);
fprintf (pFile, "BoilerGoalX3 %f ",BoilerGoalX3); eol(pFile);
fprintf (pFile, "BoilerGoalX2 %f ",BoilerGoalX2); eol(pFile);
fprintf (pFile, "BoilerGoalX1 %f ",BoilerGoalX1); eol(pFile);
fprintf (pFile, "BoilerErrorScale %f ",BoilerErrorScale); eol(pFile);
fprintf (pFile, "cam1dr     %f ",cam1dr); eol(pFile);
fprintf (pFile, "cam2dr     %f ",cam2dr); eol(pFile);
fprintf (pFile, "cam2dg     %f ",cam2dg); eol(pFile);
fprintf (pFile, "cam1dg     %f ",cam1dg); eol(pFile);
fprintf (pFile, "ShooterShootRPM %f ",ShooterShootRPM); eol(pFile);
fprintf (pFile, "HopperShootSpeed %f ",HopperShootSpeed); eol(pFile);
fprintf (pFile, "ToolBarGearPickup %f ",ToolBarGearPickup); eol(pFile);
fprintf (pFile, "ToolBarGearDropoff %f ",ToolBarGearDropoff); eol(pFile);
fprintf (pFile, "PegTargetGot %f ",PegTargetGot); eol(pFile);
fprintf (pFile, "PegGoalY   %f ",PegGoalY); eol(pFile);
fprintf (pFile, "PegGoalX   %f ",PegGoalX); eol(pFile);
fprintf (pFile, "BoilerTargetGot %f ",BoilerTargetGot); eol(pFile);
fprintf (pFile, "TurretScale %f ",TurretScale); eol(pFile);
fprintf (pFile, "TurretPOS  %f ",TurretPOS); eol(pFile);
fprintf (pFile, "HopperRPM  %f ",HopperRPM); eol(pFile);
fprintf (pFile, "HopperPos  %f ",HopperPos); eol(pFile);
fprintf (pFile, "BoilerGoalY %f ",BoilerGoalY); eol(pFile);
fprintf (pFile, "BoilerGoalX %f ",BoilerGoalX); eol(pFile);
fprintf (pFile, "GyroZ      %f ",GyroZ); eol(pFile);
fprintf (pFile, "GyroY      %f ",GyroY); eol(pFile);
fprintf (pFile, "GyroX      %f ",GyroX); eol(pFile);
fprintf (pFile, "HopperSpeed %f ",HopperSpeed); eol(pFile);
fprintf (pFile, "HopperVolts %f ",HopperVolts); eol(pFile);
fprintf (pFile, "HopperAmps %f ",HopperAmps); eol(pFile);
fprintf (pFile, "BallIntakeAmps %f ",BallIntakeAmps); eol(pFile);
fprintf (pFile, "BallIntakeVolts %f ",BallIntakeVolts); eol(pFile);
fprintf (pFile, "BallIntakeSpeed %f ",BallIntakeSpeed); eol(pFile);
fprintf (pFile, "IntakeRightAmps %f ",IntakeRightAmps); eol(pFile);
fprintf (pFile, "IntakeLeftVolts %f ",IntakeLeftVolts); eol(pFile);
fprintf (pFile, "IntakeLeftAmps %f ",IntakeLeftAmps); eol(pFile);
fprintf (pFile, "IntakeRightVolts %f ",IntakeRightVolts); eol(pFile);
fprintf (pFile, "IntakeRightSpeed %f ",IntakeRightSpeed); eol(pFile);
fprintf (pFile, "IntakeLeftSpeed %f ",IntakeLeftSpeed); eol(pFile);
fprintf (pFile, "Winch2Volts %f ",Winch2Volts); eol(pFile);
fprintf (pFile, "Winch2Speed %f ",Winch2Speed); eol(pFile);
fprintf (pFile, "Winch2Amps %f ",Winch2Amps); eol(pFile);
fprintf (pFile, "Winch1Amps %f ",Winch1Amps); eol(pFile);
fprintf (pFile, "Winch1Volts %f ",Winch1Volts); eol(pFile);
fprintf (pFile, "Winch1Speed %f ",Winch1Speed); eol(pFile);
fprintf (pFile, "ShooterDest %f ",ShooterDest); eol(pFile);
fprintf (pFile, "ShooterAmps %f ",ShooterAmps); eol(pFile);
fprintf (pFile, "ShooterVolts %f ",ShooterVolts); eol(pFile);
fprintf (pFile, "ToolBarAmps %f ",ToolBarAmps); eol(pFile);
	fprintf(pFile, "AtonFinalRotate %f ", AtonFinalRotate); eol(pFile);
	fprintf(pFile, "CrossY4    %f ", CrossY4); eol(pFile);
	fprintf(pFile, "CrossX4    %f ", CrossX4); eol(pFile);
	fprintf(pFile, "CreapSpeed %f ", CreapSpeed); eol(pFile);
	fprintf(pFile, "a6_Rotate  %f ", a6_Rotate); eol(pFile);
	fprintf(pFile, "a5_Rotate  %f ", a5_Rotate); eol(pFile);
	fprintf(pFile, "a4_Rotate  %f ", a4_Rotate); eol(pFile);
	fprintf(pFile, "a3_Rotate  %f ", a3_Rotate); eol(pFile);
	fprintf(pFile, "a2_Rotate  %f ", a2_Rotate); eol(pFile);
	fprintf(pFile, "a1_Rotate  %f ", a1_Rotate); eol(pFile);
	fprintf(pFile, "a6_MoveTime1 %f ", a6_MoveTime1); eol(pFile);
	fprintf(pFile, "a5_MoveTime1 %f ", a5_MoveTime1); eol(pFile);
	fprintf(pFile, "a4_MoveTime1 %f ", a4_MoveTime1); eol(pFile);
	fprintf(pFile, "a3_MoveTime1 %f ", a3_MoveTime1); eol(pFile);
	fprintf(pFile, "a2_MoveTime1 %f ", a2_MoveTime1); eol(pFile);
	fprintf(pFile, "TrackingGainY %f ", TrackingGainY); eol(pFile);
	fprintf(pFile, "TrackingY  %f ", TrackingY); eol(pFile);
	fprintf(pFile, "TrackingX  %f ", TrackingX); eol(pFile);
	fprintf(pFile, "TrackingGain %f ", TrackingGain); eol(pFile);
	fprintf(pFile, "RotateLeftAngle %f ", RotateLeftAngle); eol(pFile);
	fprintf(pFile, "RotateRightAngle %f ", RotateRightAngle); eol(pFile);
	fprintf(pFile, "CrossY3    %f ", CrossY3); eol(pFile);
	fprintf(pFile, "CrossY2    %f ", CrossY2); eol(pFile);
	fprintf(pFile, "CrossY1    %f ", CrossY1); eol(pFile);
	fprintf(pFile, "CrossX3    %f ", CrossX3); eol(pFile);
	fprintf(pFile, "CrossX2    %f ", CrossX2); eol(pFile);
	fprintf(pFile, "CrossX1    %f ", CrossX1); eol(pFile);
	fprintf(pFile, "GyroGain   %f ", GyroGain); eol(pFile);
	fprintf(pFile, "a1_MoveTime1 %f ", a1_MoveTime1); eol(pFile);
	fprintf(pFile, "ToolBarScale %f ", ToolBarScale); eol(pFile);
	fprintf(pFile, "ToolBarHigh %f ", ToolBarHigh); eol(pFile);
	fprintf(pFile, "ToolBarMid %f ", ToolBarMid); eol(pFile);
	fprintf(pFile, "ToolBarLow %f ", ToolBarLow); eol(pFile);
	//fprintf (pFile, "UseGyro    %f ",UseGyro); eol(pFile);


	   //------------



	fclose(pFile);

	// printf("Global.Dat Saved\n");

}





//float tableIndex=0;
//float table1[200];
//float table2[200];
//float table3[200];
//float table4[200];

int tableTrack = -1;

void GlobalSendBack(void)
{
	//	tableTrack++;

	//	if (tableTrack>=tableSize) tableTrack=0;

	if (tableSize <= 0)
	{
		tIndex = 0;
		tValue1 = 0;
		tValue2 = 0;
		tValue3 = 0;
	}
	else
	{
		tIndex = tableTrack;
		tValue1 = table1[tableTrack];
		tValue2 = table2[tableTrack];
		tValue3 = table3[tableTrack];
	}

	sendBack(398, 398);

	//for (int L=0; L<8; L++) dac[L]=L;

	for (int L = 1; L < 20; L++) sendBack(L, loc[L]);
	for (int L = 20; L < 39; L++) sendBack(L, pwm[L - 20]);
	for (int L = 40; L < 49; L++) sendBack(L, dac[L - 40]); //

	//printf("loc[pwmBinLift1] %f\n",pwmBinLift1);




	sendBack(50, DistanceH);
	sendBack(51, DistanceY);
	sendBack(52, FilterHYFlag);
	sendBack(53, FilterHYLimit);
	sendBack(54, DebugRobotVideo);

	sendBack(55, GoalFound);
	sendBack(56, GoalX);
	sendBack(57, GoalY);
	sendBack(58, GoalW);
	sendBack(59, GoalH);


	/*  sendBack( 55,PickupRollerSpeed    );
	  sendBack( 56,WheelAtSpeed         );
	  sendBack( 57,ShiftModeFlag        );
	  sendBack( 58,ShiftLowRes          );
	  sendBack( 59,ShiftHighRes        );
  */





	sendBack(62, blackAndWhite);
	sendBack(63, wantedShooterRPM);

	sendBack(64, shooterRPM);
	sendBack(65, shooterSpeed);

	sendBack(66, minDisplayLine);
	sendBack(67, maxDisplayLine);


	sendBack(68, autoDriveX);
	sendBack(69, autoDriveY);
	sendBack(70, autoDriveR);



	sendBack(88, elevUpRate);
	sendBack(89, elevDownRate);
	sendBack(90, DriveModeFlag);

	sendBack(91, inAton);
	sendBack(92, inTele);
	sendBack(93, inDisable);

	sendBack(94, eLoopCount);


	sendBack(95, joyX[1]);
	sendBack(96, joyY[1]);
	sendBack(97, joyX2[1]);
	sendBack(98, joyY2[1]);

	sendBack(99, joyX[2]);
	sendBack(100, joyY[2]);
	sendBack(101, joyX2[2]);
	sendBack(102, joyY2[2]);



	sendBack(107, gyroShift);
	sendBack(108, gyroShiftState);
	sendBack(109, elevBrake);


	static float rr = 0;

	rr += 0.01;

	//for (int L=90; L<100; L++) sendBack(L,rr);

   // printf("rr %f\n",rr);













	sendBack(111, matchStatus);

	sendBack(112, matchTime);

	sendBack(113, atonDelayTime);


	sendBack(114, armRes);
	sendBack(115, armGain);

	sendBack(118, bumperIsDown);
	sendBack(119, bumperIsUp);

	sendBack(120, holdGain);
	sendBack(121, turretGain);
	sendBack(122, holdGainR);


	sendBack(125, shotBlocker);
	sendBack(126, teamNumber);
	sendBack(127, minShootTime);
	sendBack(128, moveBackRate);

	sendBack(130, shotErrorLimit);
	sendBack(131, rollerState);

	sendBack(133, trackTimeOut);
	sendBack(134, trackLockMin);
	sendBack(135, trackCenterY);
	sendBack(136, trackCenterX);
	sendBack(137, trackState);
	sendBack(138, gfMinLevel);
	sendBack(139, bfMaxWide);
	sendBack(140, bfMaxHigh);
	sendBack(141, bfMaxRatio);
	sendBack(142, trackX);
	sendBack(143, trackY);
	sendBack(144, trackZ);


	// sendBack( 147,shoesFlag      );

	sendBack(148, curHotRatio);
	sendBack(149, minHotRatio);

	sendBack(150, armFlag);
	sendBack(151, armSpeed);
	sendBack(156, lookForHotGoal);
	sendBack(157, hotGoal);


	sendBack(158, crossHairX);
	sendBack(159, crossHairOffset);

	sendBack(168, packetSize);
	sendBack(169, autoDeployTime);



	//spare1=inDisable;








	sendBack(184, holdStraight);






	sendBack(194, crossHair2X);
	sendBack(195, crossHair2Y);


	sendBack(199, elevFlag);

	//------------ 2015 variables -------------





	sendBack(306, blockCanIO);


	sendBack(324, newImageCount);
	sendBack(325, atonLimit);

	//----------------------------------------


	sendBack(309, robotTime);





	sendBack(350, atonNum);



	sendBack(394, cameraSelect);


	//SendBackValues
  sendBack(273,BoilerGoalY3);
  sendBack(305,move_OverHeadDroppoff);
  sendBack(308,TwoCubePickupAngle);
  sendBack(310,move_TwoCubeDistance);
  sendBack(311,move_CornerToScale);
  sendBack(312,strafe_behindHeadDropoff);
  sendBack(313,strafe_secondCubePickup);
  sendBack(320,strafe_firstCubePickup);
  sendBack(314,LiftScaleBehind);
  sendBack(315,ToolBarScaleBehind);
  sendBack(316,ultra3Distance);
  sendBack(317,ultra2Distance);
  sendBack(318,ultra1Distance);
  sendBack(321,move_CornerDropCube);
  sendBack(323,move_CrossDropCube);
  sendBack(322,HoldCube);
  sendBack(326,TwoCube);
  sendBack(327,LiftClimbSide);
  sendBack(348,ToolBarClimbSide);
  sendBack(349,Crossover);
  sendBack(335,LiftClimb);
  sendBack(337,ToolBarClimb);
  sendBack(336,favorScale);
  sendBack(334,move_ScalefromBehind);
  sendBack(331,move_FaceScale);
  sendBack(328,move_SwitchfromBehind);
  sendBack(329,move_BehindSwitch);
  sendBack(330,move_SideCenterofSwitch);
  sendBack(347,move_TouchSwitch);
  sendBack(346,move_CenterofSwitch);
  sendBack(341,move_ApproachSwitch);
  sendBack(332,matchNumber);
  sendBack(333,matchTimeLeft);
  sendBack(353,ToolBarPickup);
  sendBack(352,robotWatts);
  sendBack(354,EncoderGain);
  sendBack(355,LiftGain);
  sendBack(359,ToolBarGain);
  sendBack(343,TuningVar5);
  sendBack(342,TuningVar4);
  sendBack(340,TuningVar3);
  sendBack(339,TuningVar2);
  sendBack(338,TuningVar1);
  sendBack(356,ToolBarScaleHigh);
  sendBack(357,ToolBarScaleMid);
  sendBack(358,ToolBarScaleLow);
  sendBack(360,ToolBarSwitch);
  sendBack(361,LiftScaleHigh);
  sendBack(364,LiftScaleMid);
  sendBack(365,LiftScaleLow);
  sendBack(366,LiftSwitch);
  sendBack(373,LiftLow);
  sendBack(362,LiftDest);
  sendBack(363,ToolBarSpeed);
  sendBack(367,Lift2Volts);
  sendBack(368,Lift1Volts);
  sendBack(369,Lift2Amps);
  sendBack(370,Lift1Amps);
  sendBack(371,Lift2Speed);
  sendBack(372,Lift1Speed);
  sendBack(374,ToolBarAngle);
  sendBack(375,CubeIntakeAngle);
  sendBack(376,LiftHeight);
  sendBack(377,BatteryVoltage);
  sendBack(378,RBD2Amps);
  sendBack(379,RBD1Amps);
  sendBack(380,RFD2Amps);
  sendBack(381,RFD1Amps);
  sendBack(382,LBD2Amps);
  sendBack(383,LBD1Amps);
  sendBack(384,LFD2Amps);
  sendBack(385,LFD1Amps);
  sendBack(391,LFD1Speed);
  sendBack(386,RBD2Speed);
  sendBack(387,RBD1Speed);
  sendBack(388,LBD2Speed);
  sendBack(389,LBD1Speed);
  sendBack(392,RFD2Speed);
  sendBack(393,RFD1Speed);
  sendBack(395,LFD2Speed);
  sendBack(390,EncoderVelocity);
  sendBack(164,EncoderDistance);
  sendBack(182,aton_HopperRPM);
  sendBack(177,aton_ShooterRPM);

  sendBack(84,a1_MoveTime1);
  sendBack(232,a2_MoveTime1);
  sendBack(233,a3_MoveTime1);
  sendBack(234,a4_MoveTime1);
  sendBack(235,a5_MoveTime1);
  sendBack(236,a6_MoveTime1);



  sendBack(193,maxGoalAimSpeed);
  sendBack(198,GoalAimGain);
  sendBack(241,hopperShootRPM);
  sendBack(272,gearDropoffSpeed);
  sendBack(298,gearPickupSpeed);
  sendBack(242,maxPegAimSpeed);
  sendBack(240,maxPegDriveSpeed);
  sendBack(297,pegGoalDriveScale);
  sendBack(244,pegGoalAimScale);
  sendBack(271,camerafps);
  sendBack(279,PegGoalY1);
  sendBack(270,PegGoalX1);
  sendBack(275,BoilerGoalY2);
  sendBack(274,BoilerGoalY1);
  sendBack(277,BoilerGoalX3);
  sendBack(276,BoilerGoalX2);
  sendBack(296,BoilerGoalX1);


  sendBack(278,BoilerErrorScale);
  sendBack(60,cam1dr);
  sendBack(61,cam2dr);
  sendBack(74,cam2dg);
  sendBack(75,cam1dg);
  sendBack(76,ShooterShootRPM);
  sendBack(77,HopperShootSpeed);
  sendBack(78,ToolBarGearPickup);
  sendBack(79,ToolBarGearDropoff);
  sendBack(80,PegTargetGot);
  sendBack(81,PegGoalY);
  sendBack(82,PegGoalX);
  sendBack(83,BoilerTargetGot);
  sendBack(103,TurretScale);
  sendBack(104,TurretPOS);
  sendBack(117,HopperRPM);
  sendBack(123,HopperPos);
  sendBack(124,selectCamera);
  sendBack(129,BoilerGoalY);
  sendBack(132,BoilerGoalX);
  sendBack(145,GyroZ);
  sendBack(146,GyroY);
  sendBack(152,GyroX);
  sendBack(153,HopperSpeed);
  sendBack(154,HopperVolts);
  sendBack(155,HopperAmps);
  sendBack(160,BallIntakeAmps);
  sendBack(201,BallIntakeVolts);
  sendBack(200,BallIntakeSpeed);
  sendBack(204,IntakeRightAmps);
  sendBack(202,IntakeLeftVolts);
  sendBack(203,IntakeLeftAmps);
  sendBack(205,IntakeRightVolts);
  sendBack(206,IntakeRightSpeed);
  sendBack(207,IntakeLeftSpeed);

  sendBack(116,ToolBarAmps);
	sendBack(396, AtonFinalRotate);
	sendBack(252, CrossY4);
	sendBack(251, CrossX4);
	sendBack(249, CreapSpeed);

	sendBack(231, TrackingGainY);
	sendBack(230, TrackingY);
	sendBack(229, TrackingX);
	sendBack(228, TrackingGain);
	sendBack(227, gyro_Zero);

	sendBack(225, CrossY3);
	sendBack(224, CrossY2);
	sendBack(223, CrossY1);
	sendBack(222, CrossX3);
	sendBack(221, CrossX2);
	sendBack(220, CrossX1);
	sendBack(85, GyroGain);
	sendBack(84, a1_MoveTime1);

	sendBack(73, GyroLoc);
	sendBack(71, UseGyro);
	sendBack(72, LookForGoal);

	//---------------

	sendBack(219, ToolBarLoc);
	sendBack(216, ToolBarDest);
	sendBack(217, ToolBarVolts);
	sendBack(116, ToolBarAmps);

	sendBack(344, ShooterDest);
	sendBack(351, ShooterVolts);
	sendBack(345, ShooterAmps);

	sendBack(397, Winch1Speed);
	sendBack(265, Winch1Volts);
	sendBack(253, Winch2Amps);

	sendBack(209, Winch2Speed);
	sendBack(208, Winch2Volts);
	sendBack(250, Winch2Amps);

	sendBack(206, IntakeRightSpeed);
	sendBack(205, IntakeRightVolts);
	sendBack(204, IntakeRightAmps);

	sendBack(207, IntakeLeftSpeed);
	sendBack(202, IntakeLeftVolts);
	sendBack(203, IntakeLeftAmps);


	sendBack(201, BallIntakeVolts);
	sendBack(160, BallIntakeAmps);

	sendBack(153, HopperSpeed);
	sendBack(154, HopperVolts);
	sendBack(155, HopperAmps);

	sendBack(123, HopperPos);
	sendBack(117, HopperRPM);



}

static int indent = 0;

//-----------------------------------
void Ident(int idt)
{
	char tmp[256] = "                                                          ";

	tmp[idt] = 0;

	if (idt > 0)
	{

		printf(tmp);

		tmp[0] = '0' + (idt - 2) / 2;
		tmp[1] = '_';
		tmp[2] = 0;

		printf(tmp);
	}
}

void  CP(char* val)
{
	Ident(indent);
	printf(val);
	printf("\n");
}


void CB(char* val)
{
	printf("\n");

	if (indent < 30) indent += 2;

	Ident(indent);

	printf(val);
	printf("\n");

}
void CE(char* val)
{

	CP(val);
	indent -= 2; if (indent < 0) indent = 0;

}


// -------------------------
// commands recived from pc
// -------------------------

extern int keyPressDown;
extern int keyPressUp;


//void resetDist(){resetEncoder(aRightDist);resetEncoder(aLeftDist);}

void ZeroEncoder(int index);


void resetDist()
{
	ZeroEncoder(1);
	ZeroEncoder(2);

	//loc[aLeftDist ]=0;
	//loc[aRightDist]=0;

}
void setDist(float dist) {} //setEncoder(aRightDist,dist);setEncoder(aLeftDist,dist);


void moveUp10() {} //moveUpDown(loc[aUpDown]+10,0.5,0);
void moveDn10() {} //moveUpDown(loc[aUpDown]-10,0.5,0);

void MoveArmUp10() {}//moveArm   (loc[aArmElevation]-10,0.5,0);
void MoveArmDn10() {}//moveArm   (loc[aArmElevation]+10,0.5,0);
void MoveAzmCW10() {}//moveAzimth(loc[aAzimuth     ]+10,0.5,0);
void MoveAzmCC10() {}//moveAzimth(loc[aAzimuth     ]-10,0.5,0);

void RotateTestFinished()
{

	blockGyroReset = 0; atonActive = 0; inAt = 0;

}

void RotateRobotPos45()
{

	inAt = 1;

	atonActive = 1500;

	blockGyroReset = -1500;


	//resetGyro();

	//if (a3Level==0 || a3Angle==0)

	//  rotateRobot( 45,60,10, RotateTestFinished);

	//else

   //   rotateRobot(a3Level,a3Angle,5,RotateTestFinished);


}
void RotateRobotNeg45()
{

	inAt = 1;

	atonActive = 1500;

	blockGyroReset = -1500;



}



void ZeroAll();
void ZeroHood();
void ZeroArm();
void ZeroAngle();



void resetHood() {} //resetEncoder(aUpDown);

void ZeroAll() { ZeroArm(); ZeroAzimt(); resetDist(); }
void ZeroArm() { tprintf("ZeroArm\n");/*resetEncoder(aArm    );*/ }
void ZeroAzimt() {/*tprintf("ZeroAzimuth\n");resetEncoder(aFeeder);*/ }


void StartTracking();
void StopTracking();

void GyroTest();

void imageBeg() {}; // printf("imageBeg\n");}
void imageEnd() {}; // printf("imageEnd\n");}

int    forceCount[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
double forceValue[21] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

void processCommands(char *recBuf);



void processCommands(char *recBufA, int length)
{

	//tprintf("%s\n",recBufA);

	char *recBuf = &recBufA[0];

	static double lasttime = 0;

	double curtime = Timer::GetFPGATimestamp();

	float dTime = curtime - lasttime;

	lasttime = curtime;

	float cutRate = 1.0 / dTime;

	if (recBuf[0] == 'm' && cutRate < 5)

		printf("slow kinect %6.2f \n", cutRate);

	recBuf[length] = 0;

	//int displayIt=1;

   // if (recBuf[0]=='m' && (length>=22 && length<32)) displayIt=0;

	// check for multiple commands (ignore old commands??)

	for (int L = 0; L < length - 4; L++)
	{
		if (recBuf[L] == 0 && recBuf[L + 1] != 0)
		{

			//  printf("> %d  ",length);
			//  printf(recBuf);
			//  printf("\n");

			processCommands(recBuf);

			recBuf = &recBuf[L + 1];

			break;

		}

	}

	// if (displayIt)
	// {
	  //  printf("  %d  ",length);
	 //    printf(recBuf);
	  //   printf("\n");
	// }

	processCommands(recBuf);

}

void TestAtonOneMove();
void TestAtonTwoMove();
void TestAtonThreeMove();
void TestAtonFourMove();
void TestAtonFiveMove();
void TestAtonSixMove();

void TestAtonOneShoot();
void TestAtonTwoShoot();
void TestAtonThreeShoot();
void TestAtonFourShoot();
void TestAtonFiveShoot();
void TestAtonSixShoot();


void BumperDownCmd();
void BumperUpCmd();
void CatcherOpenCmd();
void CatcherCloseCmd();
void ToroUpCmd();
void ToroDownCmd();
void  ToroCollectCmd();
void  ToroSpitCmd();
void  ToroOffCmd();

void StartNewCameraCmd();

//void LoadOnceLow(){};

void processCommands(char *recBuf)
{

	  // printf("processCommands\n");
   //recBuf[length]=0;

	 //  printf(recBuf);
	  //   printf("\n");

	if (recBuf[0] == 'M')
	{


	}





/*

	if (strcmp(recBuf, "BumperDownCmd") == 0) { tprintf("[rec]BumperDownCmd\n"); BumperDownCmd(); return; }
	if (strcmp(recBuf, "BumperUpCmd") == 0) { tprintf("[rec]BumperUpCmd\n"); BumperUpCmd(); return; }
	if (strcmp(recBuf, "CatcherOpenCmd") == 0) { tprintf("[rec]CatcherOpenCmd\n"); CatcherOpenCmd(); return; }
	if (strcmp(recBuf, "CatcherCloseCmd") == 0) { tprintf("[rec]CatcherCloseCmd\n"); CatcherCloseCmd(); return; }
	if (strcmp(recBuf, "ToroUpCmd") == 0) { tprintf("[rec]ToroUpCmd\n"); ToroUpCmd(); return; }
	if (strcmp(recBuf, "ToroDownCmd") == 0) { tprintf("[rec]ToroDownCmd\n"); ToroDownCmd(); return; }
	if (strcmp(recBuf, "ToroCollectCmd") == 0) { tprintf("[rec]ToroCollectCmd\n"); ToroCollectCmd(); return; }
	if (strcmp(recBuf, "ToroSpitCmd") == 0) { tprintf("[rec]ToroSpitCmd\n"); ToroSpitCmd(); return; }
	if (strcmp(recBuf, "ToroOffCmd") == 0) { tprintf("[rec]ToroOffCmd\n"); ToroOffCmd(); return; }

*/
	void zeroToteLift();
	void zeroBinLift();


/*
	if (strcmp(recBuf, "ZeroToteLiftCmd") == 0) { tprintf("[rec]ZeroToteLiftCmd\n");    zeroToteLift(); return; }
	if (strcmp(recBuf, "ZeroBinLiftCmd") == 0) { tprintf("[rec]ZeroBinLiftCmd\n");    zeroBinLift(); return; }
*/




	if (recBuf[0] == 'A')
	{
		if (strcmp(recBuf, "Alive") == 0) { alive++; return; } // used to keep UDC channel open

	//   if (strcmp(recBuf,"AutoHangCmd"        )==0){AutoHang        (); return;}
   //    if (strcmp(recBuf,"AutoTrackAndHangCmd")==0){AutoTrackAndHang(); return;}
	//   if (strcmp(recBuf,"AutoDeployCmd"      )==0){delayedMiniBot  (); return;}
	}

	if (recBuf[0] == 'C')
	{

		//   if (strcmp(recBuf,"CenterTargetCmd")==0){CenterTarget(); return;}
	}

	if (recBuf[0] == 'G')
	{

		//	  if (strcmp(recBuf,"GyroTestCmd")==0){GyroTest(); return;}

	}


	//if (recBuf[0]=='K')
	//  {

	   // if (recBuf[3]=='D')
	   // {
		  //  keyPressDown=recBuf[7]; loc[8]=keyPressDown;
	   // }

	   // if (recBuf[3]=='U')
	   // {
		  // keyPressUp=recBuf[5];loc[6]=keyPressUp;

	   // }

	//  }



	if (recBuf[0] == 'R')
	{
		//   if (strcmp(recBuf,"ResetGyroCmd"   )==0){ resetGyro  (); return;}
		if (strcmp(recBuf, "ResetDistCmd") == 0) { resetDist(); return; }
		if (strcmp(recBuf, "ResetHoodCmd") == 0) { resetHood(); return; }

		//	if (strcmp(recBuf,"RotateRobotPos45Cmd" )==0){ RotateRobotPos45(); return;}
		 //   if (strcmp(recBuf,"RotateRobotNeg45Cmd" )==0){ RotateRobotNeg45(); return;}

	}

	if (recBuf[0] == 'S')
	{
		if (strcmp(recBuf, "StartAton1Cmd") == 0) { startAton(1); return; }
		if (strcmp(recBuf, "StartAton2Cmd") == 0) { startAton(2); return; }
		if (strcmp(recBuf, "StartAton3Cmd") == 0) { startAton(3); return; }
		if (strcmp(recBuf, "StartAton4Cmd") == 0) { startAton(4); return; }
		if (strcmp(recBuf, "StartAton5Cmd") == 0) { startAton(5); return; }
		if (strcmp(recBuf, "StartAton6Cmd") == 0) { startAton(6); return; }


		if (strcmp(recBuf, "StartTrackingCmd") == 0) { StartTracking(); return; }
		if (strcmp(recBuf, "StopTrackingCmd") == 0) { StopTracking(); return; }

		if (strcmp(recBuf, "StartNewCameraCmd") == 0) { StartNewCameraCmd(); return; }

	}

	if (recBuf[0] == 'T')
	{
		if (strcmp(recBuf, "TestAtonOneMoveCmd") == 0) { TestAtonOneMove(); return; }
		if (strcmp(recBuf, "TestAtonTwoMoveCmd") == 0) { TestAtonTwoMove(); return; }
		if (strcmp(recBuf, "TestAtonThreeMoveCmd") == 0) { TestAtonThreeMove(); return; }
		if (strcmp(recBuf, "TestAtonFourMoveCmd") == 0) { TestAtonFourMove(); return; }
		if (strcmp(recBuf, "TestAtonFiveMoveCmd") == 0) { TestAtonFiveMove(); return; }
		if (strcmp(recBuf, "TestAtonSixMoveCmd") == 0) { TestAtonSixMove(); return; }

		if (strcmp(recBuf, "TestAtonOneShootCmd") == 0) { TestAtonOneShoot(); return; }
		if (strcmp(recBuf, "TestAtonTwoShootCmd") == 0) { TestAtonTwoShoot(); return; }
		if (strcmp(recBuf, "TestAtonThreeShootCmd") == 0) { TestAtonThreeShoot(); return; }
		if (strcmp(recBuf, "TestAtonFourShootCmd") == 0) { TestAtonFourShoot(); return; }
		if (strcmp(recBuf, "TestAtonFiveShootCmd") == 0) { TestAtonFiveShoot(); return; }
		if (strcmp(recBuf, "TestAtonSixShootCmd") == 0) { TestAtonSixShoot(); return; }



	}


	if (recBuf[0] == 'Z')
	{
		if (strcmp(recBuf, "ZeroAllCmd") == 0) { ZeroAll(); return; }
		//if (strcmp(recBuf, "ZeroHoodCmd") == 0) { ZeroHood(); return; }
		if (strcmp(recBuf, "ZeroArmCmd") == 0) { ZeroArm(); return; }
		if (strcmp(recBuf, "ZeroAzimtCmd") == 0) { ZeroAzimt(); return; }
	}

	if (recBuf[0] == 'm')
	{
		char * ptr = &recBuf[1];

		for (int m = 0; m < 16; m++)
		{
			int neg = 0;

			for (int n = 0; n < 100; n++) if (ptr[0] == ' ') ptr++; else break;

			if (ptr[0] == ',') { ptr++; continue; }

			if (ptr[0] == '-')
			{
				neg = 1;

				ptr++;

				for (int n = 0; n < 100; n++) if (ptr[0] == ' ') ptr++; else break;
			}

			if (ptr[0] == 0) break;

			int value = 0;

			if (ptr[0] >= '0' && ptr[0] <= '9')
			{
				value = ptr[0] - '0';

				ptr++;
			}

			if (ptr[0] >= '0' && ptr[0] <= '9')
			{
				value = value * 10 + (ptr[0] - '0');

				ptr++;
			}

			if (neg) forceValue[m] = -value / 100.0; else forceValue[m] = value / 100.0;

			forceCount[m] = 100;

			for (int n = 0; n < 100; n++) if (ptr[0] == ' ') ptr++; else break;

			if (ptr[0] == ',')  ptr++;

		}



		return;
	}


	if (recBuf[0] == 'v')
	{

		printf(recBuf);
		printf("\n");

		char local[101];

		strcpy(local, recBuf);

		char * ptr = &local[2];

		for (int L = 0; L < 100; L++) if (ptr[0] == ' ') ptr++;

		char varName[101];

		int len = 0;

		for (int L = 0; L < 100; L++)
		{
			if (ptr[0] != ' ')
			{
				varName[L] = ptr[0]; ptr++;

				if (!varName[L]) return;
			}
			else
			{
				varName[L] = 0;
				len = L;
				break;
			}
		}

		if (!len) return;

		double val = atof(ptr);

		float v = (float)val;

		assignValue(varName, v);

		SaveGlobalData();

	}

}




void setupUserData(char* Buf, int maxSize)
{

	static int datCount = -1;

	datCount++; if (datCount > 50) datCount = 0;

	if (datCount == 0)
	{

		unsigned int *ptr;

		ptr = (unsigned int*)&sendBackValues[0];

		ptr[0] = 0x01130260;//0x60021301;       //
		ptr[1] = 0xFFFFFFFF;

		unsigned int *ptr2;

		ptr2 = (unsigned int*)&Buf[0];

		for (int L = 0; L < 200; L++)
		{
			ptr2[L] = ptr[L]; //sendBackValues[L];
		}

		return;

	}



	switch (slowVideoMode)
	{

	case 1: setupUserDataFullColor6Bit(Buf, maxSize); return;
	case 2: setupUserDataBW6bit(Buf, maxSize); return;
	case 3: setupUserDataFullColor3Bit(Buf, maxSize); return;
	case 4: setupUserDataBW6bitTC(Buf, maxSize); return;
	case 5: setupUserDataBW6bitLR(Buf, maxSize); return;


	default:;
	}

}


void NextVideoMode()
{
	slowVideoMode++;
	if (slowVideoMode > 5) slowVideoMode = 1;

	SaveGlobalData();
}



unsigned char slowImageBuf[160 * 120 * 4 + 10];
int slowLine = 0;

void setupUserDataFullColor6Bit(char* Buf, int maxSize)
{

	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int slowIndex = 2;
	int fastIndex = slowLine * 320 * 4;

	for (int L = 0; L < 320; L++)
	{
		int b1 = slowImageBuf[fastIndex];

		fastIndex += 1; if ((fastIndex & 0x3) == 3) fastIndex++;

		int b2 = slowImageBuf[fastIndex];

		fastIndex += 1; if ((fastIndex & 0x3) == 3) fastIndex++;

		int b3 = slowImageBuf[fastIndex];

		fastIndex += 1; if ((fastIndex & 0x3) == 3) fastIndex++;

		int b4 = slowImageBuf[fastIndex];

		fastIndex += 1;	if ((fastIndex & 0x3) == 3) fastIndex++;

		Buf[slowIndex + 0] = (b1 & 0xFC) | ((b2 >> 6) & 3);;
		Buf[slowIndex + 1] = ((b2 << 2) & 0xF0) | ((b3 >> 4) & 0xF);
		Buf[slowIndex + 2] = ((b3 << 4) & 0xC0) | (b4 >> 2);

		slowIndex += 3;

	}

	Buf[0] = 1;
	Buf[1] = slowLine;

	slowLine++;


	if (slowLine > 59) slowLine = 0;


}

void setupUserDataFullColor3Bit(char* Buf, int maxSize)
{

	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int slowIndex = 2;
	int fastIndex = slowLine * 2400 * 4;

	for (int L = 0; L < 300; L++)
	{
		int c1 = slowImageBuf[fastIndex + 0] & 0x80;
		int c2 = slowImageBuf[fastIndex + 1] & 0x80;
		int c3 = slowImageBuf[fastIndex + 2] & 0x80;

		int c4 = slowImageBuf[fastIndex + 4] & 0x80;
		int c5 = slowImageBuf[fastIndex + 5] & 0x80;
		int c6 = slowImageBuf[fastIndex + 6] & 0x80;

		int b1 = c1 | (c2 >> 1) | (c3 >> 2) | (c4 >> 3) | (c5 >> 4) | (c6 >> 5);

		c1 = slowImageBuf[fastIndex + 8] & 0x80;
		c2 = slowImageBuf[fastIndex + 9] & 0x80;
		c3 = slowImageBuf[fastIndex + 10] & 0x80;

		c4 = slowImageBuf[fastIndex + 12] & 0x80;
		c5 = slowImageBuf[fastIndex + 13] & 0x80;
		c6 = slowImageBuf[fastIndex + 14] & 0x80;

		int b2 = c1 | (c2 >> 1) | (c3 >> 2) | (c4 >> 3) | (c5 >> 4) | (c6 >> 5);

		c1 = slowImageBuf[fastIndex + 16] & 0x80;
		c2 = slowImageBuf[fastIndex + 17] & 0x80;
		c3 = slowImageBuf[fastIndex + 18] & 0x80;

		c4 = slowImageBuf[fastIndex + 20] & 0x80;
		c5 = slowImageBuf[fastIndex + 21] & 0x80;
		c6 = slowImageBuf[fastIndex + 22] & 0x80;

		int b3 = c1 | (c2 >> 1) | (c3 >> 2) | (c4 >> 3) | (c5 >> 4) | (c6 >> 5);

		c1 = slowImageBuf[fastIndex + 24] & 0x80;
		c2 = slowImageBuf[fastIndex + 25] & 0x80;
		c3 = slowImageBuf[fastIndex + 26] & 0x80;

		c4 = slowImageBuf[fastIndex + 28] & 0x80;
		c5 = slowImageBuf[fastIndex + 29] & 0x80;
		c6 = slowImageBuf[fastIndex + 30] & 0x80;

		fastIndex += 32;

		int b4 = c1 | (c2 >> 1) | (c3 >> 2) | (c4 >> 3) | (c5 >> 4) | (c6 >> 5);

		Buf[slowIndex + 0] = (b1 & 0xFC) | ((b2 >> 6) & 3);;
		Buf[slowIndex + 1] = ((b2 << 2) & 0xF0) | ((b3 >> 4) & 0xF);
		Buf[slowIndex + 2] = ((b3 << 4) & 0xC0) | (b4 >> 2);

		slowIndex += 3;

	}

	Buf[0] = 3;
	Buf[1] = slowLine;

	slowLine += 1;

	if (slowLine > 7) slowLine = 0;

}


int camByteCount = 0;

void setupUserDataBW6bit(char* Buf, int maxSize)
{

	// printf("%d\n",slowLine);

	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int slowIndex = 2;
	int fastIndex = slowLine * 160 * 4;

	for (int L = 0; L < 320; L++)
	{
		int b1 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b1 < 8) b1 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b1 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b1 = 0;
		}

		fastIndex += 4;

		int b2 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b2 < 8) b2 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b2 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b2 = 0;
		}


		fastIndex += 4;

		int b3 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b3 < 8) b3 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b3 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b3 = 0;
		}

		fastIndex += 4;

		int b4 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b4 < 8) b4 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b4 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b4 = 0;
		}

		fastIndex += 4;

		Buf[slowIndex + 0] = (b1 & 0xFC) | ((b2 >> 6) & 3);;
		Buf[slowIndex + 1] = ((b2 << 2) & 0xF0) | ((b3 >> 4) & 0xF);
		Buf[slowIndex + 2] = ((b3 << 4) & 0xC0) | (b4 >> 2);

		slowIndex += 3;

	}

	Buf[0] = 2;
	Buf[1] = slowLine;

	slowLine += 8;

	if (slowLine > 119) slowLine = 0;

}


void setupUserDataBW6bitTC(char* Buf, int maxSize)
{

	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int topTarget = 0; //mTargetY/8-2; if (topTarget<0) topTarget=0; topTarget*=8;

	if (topTarget > 79)

		topTarget = 79;

	if (slowLine < topTarget) slowLine = topTarget;


	int slowIndex = 2;
	int fastIndex = slowLine * 160 * 4;

	for (int L = 0; L < 320; L++)
	{
		int b1 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b1 < 8) b1 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b1 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b1 = 0;
		}

		fastIndex += 4;

		int b2 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b2 < 8) b2 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b2 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b2 = 0;
		}


		fastIndex += 4;

		int b3 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b3 < 8) b3 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b3 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b3 = 0;
		}

		fastIndex += 4;

		int b4 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		if (b4 < 8) b4 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b4 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b4 = 0;
		}

		fastIndex += 4;




		Buf[slowIndex + 0] = (b1 & 0xFC) | ((b2 >> 6) & 3);;
		Buf[slowIndex + 1] = ((b2 << 2) & 0xF0) | ((b3 >> 4) & 0xF);
		Buf[slowIndex + 2] = ((b3 << 4) & 0xC0) | (b4 >> 2);

		slowIndex += 3;

	}

	Buf[0] = 4;
	Buf[1] = slowLine;
	Buf[2] = topTarget;

	slowLine += 8;


	if (slowLine > topTarget + 40) { slowLine = 0; Buf[0] = Buf[0] | 0x80; }


} //setupUserDataBW6bitTC


void setupUserDataBW6bitLR(char* Buf, int maxSize)
{

	// printf("%d\n",slowLine);

	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int baseSlow = slowLine;

	int slowIndex = 2;
	int fastIndex = slowLine * 160 * 4;

	int col = 0;


	for (int L = 0; L < 320; L++)
	{
		int b1 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3; //b1=line;

		if (b1 < 8) b1 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b1 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b1 = 0;
		}

		fastIndex += 4;

		col++; if (col >= 160) { slowLine += 3; col = 0; fastIndex = slowLine * 160 * 4; }

		int b2 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;//b2=line;

		if (b2 < 8) b2 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b2 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b2 = 0;
		}


		fastIndex += 4;
		col++; if (col >= 160) { slowLine += 3; col = 0; fastIndex = slowLine * 160 * 4; }

		int b3 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;//b3=line;

		if (b3 < 8) b3 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b3 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b3 = 0;
		}

		fastIndex += 4;
		col++; if (col >= 160) { slowLine += 3; col = 0; fastIndex = slowLine * 160 * 4; }

		int b4 = ((int)slowImageBuf[fastIndex] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;//b4=line;

		if (b4 < 8) b4 = 8;

		if (!slowImageBuf[fastIndex + 1])
		{
			if (!slowImageBuf[fastIndex + 2] && slowImageBuf[fastIndex] == 255) b4 = 4;
			else
				if (!slowImageBuf[fastIndex] && slowImageBuf[fastIndex + 2] == 255) b4 = 0;
		}

		fastIndex += 4;
		col++; if (col >= 160) { slowLine += 3; col = 0; fastIndex = slowLine * 160 * 4; }



		Buf[slowIndex + 0] = (b1 & 0xFC) | ((b2 >> 6) & 3);;
		Buf[slowIndex + 1] = ((b2 << 2) & 0xF0) | ((b3 >> 4) & 0xF);
		Buf[slowIndex + 2] = ((b3 << 4) & 0xC0) | (b4 >> 2);

		slowIndex += 3;

	}

	Buf[0] = 5;
	Buf[1] = baseSlow;

	if (slowLine > 119 - 20) { slowLine = 0; Buf[0] = Buf[0] | 0x80; }


} //setupUserDataBW6bitLR



void setupUserDataFullRes(char* Buf, int maxSize)
{

	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int slowIndex = 2;
	int fastIndex = slowLine * 160 * 4;

	for (int L = 0; L < 320; L++)
	{
		Buf[slowIndex + 0] = slowImageBuf[fastIndex + 0];
		Buf[slowIndex + 1] = slowImageBuf[fastIndex + 1];
		Buf[slowIndex + 2] = slowImageBuf[fastIndex + 2];

		slowIndex += 3;
		fastIndex += 4;

	}

	Buf[0] = 0;
	Buf[1] = slowLine;

	slowLine++;
	slowLine++;

	if (slowLine > 119) slowLine = 0;

}

void setupUserDataBW(char* Buf, int maxSize)
{
	if (slowLine == 0) { memcpy(slowImageBuf, imgSend, camByteCount); imgGone = 1; }

	int slowIndex = 2;
	int fastIndex = slowLine * 160 * 4;

	for (int L = 0; L < 320 * 3; L++)
	{
		Buf[slowIndex + 0] = ((int)slowImageBuf[fastIndex + 0] + slowImageBuf[fastIndex + 1] + slowImageBuf[fastIndex + 2]) / 3;

		slowIndex += 1;
		fastIndex += 4;
	}

	Buf[0] = 0;
	Buf[1] = slowLine;

	slowLine += 6;

	if (slowLine > 119) slowLine = 0;
}



extern float miniBot;



void updateDashboardLights()
{
	if (matchStatus == 1 || matchStatus == 2)
	{
		if (inDisable) matchTimeBase = Timer::GetFPGATimestamp() - matchTime;
		else
			matchTime = Timer::GetFPGATimestamp() - matchTimeBase;
	}

	if (matchStatus == 1 && inTele && !inDisable && matchTime > 10) { matchStatus = 2; matchTimeBase = Timer::GetFPGATimestamp() - 10; }

	if (matchStatus == 2 && matchTime > 150.0 && inDisable) matchStatus = 3;

	if (matchStatus == 2 && matchTime > autoDeployTime && !inDisable && deployReady && !miniBot)
	{

	}
}


//#include "DriverStationLCD.h"

//extern DriverStationLCD *dsLCD;

void eMes(char *statusString)
{
	printf("%s\n", statusString);

	//setErrorData(statusString, strlen(statusString), 100);
}

void setShifterAngle(float angle)
{
	if (angle < 0) angle = 0;
	if (angle > 180) angle = 180;

	//float outvalue=angle/180; //0->0.5

	//pwm[aShift]=outvalue;
}

void setStopDropAngle(float angle)
{
	if (angle < 0) angle = 0;
	if (angle > 180) angle = 180;

	//float outvalue=angle/180; //0->0.5

	//pwm[aStopDrop]=outvalue;
}




double baseClock = 0;
double lastClock = 0;

void tp()
{

	double newClock = Timer::GetFPGATimestamp();

	float dif = newClock - lastClock;

	lastClock = newClock;

	if (fabs(dif) > 20.0) baseClock = newClock;

	int curTime = (int)((newClock - baseClock) * 100);

	char time[10] = { "   .  :" };

	time[5] = '0' + (curTime % 10); curTime /= 10;
	time[4] = '0' + (curTime % 10); curTime /= 10;
	time[2] = '0' + (curTime % 10); curTime /= 10;
	time[1] = '0' + (curTime % 10); curTime /= 10;
	time[0] = '0' + (curTime % 10); curTime /= 10;

	if (time[0] == '0') { time[0] = ' '; if (time[1] == 0) time[1] = ' '; }

	printf(time);

}

void tpp()
{

	double newClock = Timer::GetFPGATimestamp();

	float dif = newClock - lastClock;

	lastClock = newClock;

	if (fabs(dif) > 20.0) baseClock = newClock;

	int curTime = (int)((newClock - baseClock) * 100);

	char time[10] = { "      :" };

	time[5] = ' '; curTime /= 10;
	time[4] = ' '; curTime /= 10;
	time[2] = ' '; curTime /= 10;
	time[1] = ' '; curTime /= 10;
	time[0] = ' '; curTime /= 10;

	printf(time);

}

#include "wpilib.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#define BUFLEN 1000
#define BUFLENfrompi 40
#define PORT 1130

struct sockaddr_in si_me, si_other;
struct sockaddr_in si_mefrompi, si_otherfrompi;
int hSocket;
int hSocketfrompi;

int socketActive = 0;
int socketActivefrompi = 0;

void
UDCReceiverTestInit()
{

	//whm

	//return;

	if ((hSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		//printf("Socket Error\n");
		return;
	}
	memset((char *)&si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("10.4.94.5"); //


	if (bind(hSocket, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
		printf("Binding Error\n");
		return;
	}

	socketActive = 1;

	printf("UDCReceiverTestInit (%d)\n", (int)PORT);



}

double lastUDC = 0;

void UDCReceiverTest()
{
	if (!socketActive) UDCReceiverTestInit();

	if (!socketActive) return;


	//int slen=sizeof(si_other);
	char buf[BUFLEN];

	memset((char *)&buf[0], 0, BUFLEN);

	//int result=recvfrom(hSocket, buf, BUFLEN, MSG_DONTWAIT, (struct sockaddr *)&si_other, &slen);

	struct sockaddr *from = 0;
	int len = sizeof(from);

	int result = recvfrom(hSocket, buf, BUFLEN, MSG_DONTWAIT, from, (socklen_t*)&len);

	//

	if (result != -1)
	{
		//	printf("Robot::UDCReceiverTest %d\n",result);
		  // printf("Result %d\n",result);
		printf(buf);
		printf("\n");
		processCommands(buf, result);

		lastUDC = Timer::GetFPGATimestamp();

	}
	else
	{

		if (Timer::GetFPGATimestamp() - lastUDC > 2) //socket timeout (reset socket)
		{
			close(hSocket);

			socketActive = 0;

			lastUDC = Timer::GetFPGATimestamp();
		}
	}

	//close(hSocket); // unreachable with this simple example
}



//------------ sending code

//#include <vxWorks.h>

//#include <errnoLib.h>
 // #include <hostLib.h>
 //#include <inetLib.h>


 //#include <sockLib.h>


#define BUFLEN_SEND 512
#define PORT_SEND 1140
#define PORT_SENDpi 1131

struct sockaddr_in   pirecipient;
struct sockaddr_in   recipient;

static	int sendSocket;
static int sendSocketPi;

int sendSocketActive = 0;
int sendSocketActivepi = 0;

//static    struct addrinfo *results = NULL,
//                          *addrptr = NULL,
 //                         hints;

//static   int  retval;

extern int teamNum;

double startTimeUDC = 0;
double startTimeUDCpi = 0;

double lastUDCfrompi = 0;

int sendMesCount = 0;
int sendMesCountpi = 0;

void
UDCSendTestInit()
{

	//printf("UDCSendTestInit ( %d )\n",teamNum);

	//if (!alive) return;

	if ((sendSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		//printf("Socket Error\n");
		return;
	}
	memset((char *)&recipient, 0, sizeof(recipient));

	recipient.sin_family = AF_INET;
	recipient.sin_port = htons(PORT_SEND);

	//recipient.sin_addr.s_addr =inet_addr("10.4.94.5");


	//recipient.sin_addr.s_addr =inet_addr("mainro.5");

	//WHM



	//recipient.sin_addr.s_addr = inet_addr(TeamIP);

	recipient.sin_addr.s_addr = inet_addr(TeamDriveStationIP);


	//recipient.sin_addr.s_addr =inet_addr("10.0.70.5");

	//if (teamNum==494)
	//	recipient.sin_addr.s_addr = inet_addr("10.4.94.5");
	//if (teamNum== 70)
	//	recipient.sin_addr.s_addr = inet_addr("10.0.70.5");

	//if (teamNum== 14)
	//	recipient.sin_addr.s_addr = inet_addr("10.0.14.5");


	sendSocketActive = 1;

	startTimeUDC = Timer::GetFPGATimestamp();

	//if (teamNum!=494 && teamNum!=70 && teamNum!=14)
	//{

		//DriverStation *ds = DriverStation::GetInstance();
			//ds->WaitForData();
			//teamNum = ds->GetTeamNumber();

	//}

	//printf("sendSocketActive %d sendMesCount %d\n",sendSocketActive,sendMesCount);

	if (sendMesCount == 0)
	{

		//if (teamNum==494)
			//printf("UDCSendTestInit( 494 )\n");

					//if (teamNum==70) printf("UDCSendTestInit( 70 )\n");
					//if (teamNum==14) printf("UDCSendTestInit( 14 )\n");

					//else printf("USCSendTestInit( Unknown! %d )\n",teamNum);
	}

	sendMesCount++; if (sendMesCount > 11) sendMesCount = 0;

}

void
UDCSendTestInittoPI()
{

	//printf("UDCSendTestInit ( %d )\n",teamNum);

	//if (!alive) return;

	if ((sendSocketPi = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		//printf("Socket Error\n");
		return;
	}
	memset((char *)&pirecipient, 0, sizeof(pirecipient));

	pirecipient.sin_family = AF_INET;
	pirecipient.sin_port = htons(PORT_SENDpi);

	//recipient.sin_addr.s_addr =inet_addr("10.4.94.5");


	//recipient.sin_addr.s_addr =inet_addr("mainro.5");

	//WHM



	//recipient.sin_addr.s_addr = inet_addr(TeamIP);

	pirecipient.sin_addr.s_addr = inet_addr(TeamPiIP);


	//recipient.sin_addr.s_addr =inet_addr("10.0.70.5");

	//if (teamNum==494)
	//	recipient.sin_addr.s_addr = inet_addr("10.4.94.5");
	//if (teamNum== 70)
	//	recipient.sin_addr.s_addr = inet_addr("10.0.70.5");

	//if (teamNum== 14)
	//	recipient.sin_addr.s_addr = inet_addr("10.0.14.5");


	sendSocketActivepi = 1;

	startTimeUDCpi = Timer::GetFPGATimestamp();

	//if (teamNum!=494 && teamNum!=70 && teamNum!=14)
	//{

		//DriverStation *ds = DriverStation::GetInstance();
			//ds->WaitForData();
			//teamNum = ds->GetTeamNumber();

	//}

	//printf("sendSocketActivepi %d sendMesCount %d\n",sendSocketActivepi,sendMesCount);

	if (sendMesCountpi == 0)
	{

		//if (teamNum==494)
			//printf("UDCSendTestInit( 494 )\n");

					//if (teamNum==70) printf("UDCSendTestInit( 70 )\n");
					//if (teamNum==14) printf("UDCSendTestInit( 14 )\n");

					//else printf("USCSendTestInit( Unknown! %d )\n",teamNum);
	}

	sendMesCountpi++; if (sendMesCountpi > 11) sendMesCountpi = 0;

}

int sendUDC(char *buf, int size)
{

	if (!sendSocketActive) UDCSendTestInit();

	double curTime = Timer::GetFPGATimestamp();

	if (curTime - startTimeUDC > 10.0) { close(sendSocket); UDCSendTestInit(); }

	if (!sendSocketActive) return -1;

	if (size > 8192) size = 8192;

	int slen = sizeof(recipient);

	//printf("size %d slen %d\n",size,slen);

	int result = sendto(sendSocket, buf, size, 0, (struct sockaddr *)&recipient, slen);

	//printf("result %d\n",result);

	return result;
}

int sendUDCtoPI(char *buf, int size)
{

	if (!sendSocketActivepi) UDCSendTestInittoPI();

	double curTime = Timer::GetFPGATimestamp();

	if (curTime - startTimeUDCpi > 10.0) { close(sendSocketPi); UDCSendTestInittoPI(); }

	if (!sendSocketActivepi) return -1;

	if (size > 8192) size = 8192;

	int slen = sizeof(pirecipient);

	//printf("size %d slen %d\n",size,slen);
	//int result = sendto(sendSocket, buf, size, 0, (struct sockaddr *)&pirecipient, slen);
	int result = sendto(sendSocketPi, buf, size, 0, (struct sockaddr *)&pirecipient, slen);

	//printf("result %d\n",result);

	return result;
}






void
UDCReceiverTestInitfrompi()
{

	//whm

	//return;

	if ((hSocketfrompi = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		//printf("Socket Error\n");
		return;
	}
	memset((char *)&si_mefrompi, 0, sizeof(si_mefrompi));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORTfrompi);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("10.4.94.5"); //


	if (bind(hSocketfrompi, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
		printf("Binding Error\n");
		return;
	}

	socketActivefrompi = 1;

	printf("UDCReceiverTestInit (%d)\n", (int)PORTfrompi);



}



void UDCReceiverTestfrompi()
{
	if (!socketActivefrompi) UDCReceiverTestInitfrompi();

	if (!socketActivefrompi) return;


	//int slen=sizeof(si_other);
	char buf[BUFLENfrompi];

	memset((char *)&buf[0], 0, BUFLENfrompi);

	//int result=recvfrom(hSocket, buf, BUFLEN, MSG_DONTWAIT, (struct sockaddr *)&si_other, &slen);

	struct sockaddr *from = 0;
	int len = sizeof(from);

	int result = recvfrom(hSocketfrompi, buf, BUFLENfrompi, MSG_DONTWAIT, from, (socklen_t*)&len);

	//

	if (result != -1)
	{
		//	printf("Robot::UDCReceiverTest %d\n",result);
		  // printf("Result %d\n",result);
		float *goalInfo = (float*)&buf[0];
		BoilerGoalY = goalInfo[1];
		BoilerGoalX = goalInfo[0];
		BoilerTargetGot = goalInfo[2];
		PegGoalY = goalInfo[4];
		PegGoalX = goalInfo[3];
		PegTargetGot = goalInfo[5];
		camerafps = goalInfo[6];
		PegGoalAngle = goalInfo[7];


		lastUDCfrompi = Timer::GetFPGATimestamp();

	}
	else
	{

		if (Timer::GetFPGATimestamp() - lastUDCfrompi > 2) //socket timeout (reset socket)
		{
			close(hSocketfrompi);

			socketActivefrompi = 0;

			lastUDCfrompi = Timer::GetFPGATimestamp();
			}
	}

	//close(hSocket); // unreachable with this simple example
}










void GetSpeedHoodOffset(float Dist, float &Speed, float &Hood, float &Offset)
{


} //GetSpeedHoodOffset

void SendImageAndBackValues();

void SendBackValues()
{

	//SendImageAndBackValues();

	// return;

	//unsigned int *ptr;

	//  ptr=(unsigned int*)&sendBackValues[0];

	unsigned char *cptr = (unsigned char*)&sendBackValues[0];

	//ptr[0]=0x01130260;//0x60021301;       //
	//ptr[1]=0xFFFFFFFF;

	cptr[0] = 1;
	cptr[1] = 19;
	cptr[2] = 2;
	cptr[3] = 96;
	cptr[4] = 255;
	cptr[5] = 0;
	cptr[6] = 0;
	cptr[7] = 0;

	//for (int L=2;L<100; L++) sendBackValues[L]=123.0;

	//  printf("P1 %d %d %d %d \n",(int)cptr[8],(int)cptr[9],(int)cptr[10],(int)cptr[11]);

	sendUDC((char*)&sendBackValues[0], 400 * 4 + 5);

	void sendImage();

	sendImage();

}

extern char imgRaw0[500000];
extern char imgRaw1[500000];
extern int imgRawSize0;
extern int imgRawSize1;

char tmpBuf[500000];

void sendDebugImage();

void sendImage()
{
	/*

		int size=imgRawSize0;

		char * imgRaw=imgRaw0;

		//int baseLine=119-size/8000;

		int ptr=0;

		int payloadSize=8000;

		if (size<8000) payloadSize=size;

		unsigned char payloadSizeLB= payloadSize & 0xFF;
		unsigned char payloadSizeMB= (payloadSize & 0xFF00) >> 8;

		int i=0;

		tmpBuf[i++]=1;
		tmpBuf[i++]=19;
		tmpBuf[i++]=2;
		tmpBuf[i++]=103;
		tmpBuf[i++]=(unsigned char)119;
		tmpBuf[i++]=(char)payloadSizeLB;
		tmpBuf[i++]=(char)payloadSizeMB;

		for (int M=0; M<8000; M++)
		{
		 tmpBuf[i++]=imgRaw[ptr++];

		 size--; if (size<=0) break;
		}

		sendUDC(tmpBuf, i);

		i=0;

		if (DebugRobotVideo) { sendDebugImage(); return; }

	*/
}


void sendDebugImage()
{
	//int size = 320*240*4;

	char * imgRaw = imgSend;

	//int baseLine=119-size/8000;

	int ptr = 0;

	int payloadSize = 7680;

	unsigned char payloadSizeLB = payloadSize & 0xFF;
	unsigned char payloadSizeMB = (payloadSize & 0xFF00) >> 8;

	for (int Pass = 0; Pass < 30; Pass++)
	{

		int i = 0;

		tmpBuf[i++] = 1;
		tmpBuf[i++] = 19;
		tmpBuf[i++] = 2;
		tmpBuf[i++] = 103;
		tmpBuf[i++] = (unsigned char)(200 + Pass);
		tmpBuf[i++] = (char)payloadSizeLB;
		tmpBuf[i++] = (char)payloadSizeMB;

		for (int M = 0; M < 7680; M += 3)
		{
			//if (size<=0) break;

			tmpBuf[i++] = imgRaw[ptr++];
			tmpBuf[i++] = imgRaw[ptr++];
			tmpBuf[i++] = imgRaw[ptr++];

			ptr++;

			//size-=4;

		}

		sendUDC(tmpBuf, i);

		i = 0;

		//imgRaw+=8000;

	}

}

void SendImageAndBackValues()
{
	/*
		  int size=imgRawSize0;

			char * imgRaw=imgRaw0;

		   // int baseLine=119-size/8000;

			int ptr=0;


				int payloadSize=8000;

				if (size<8000) payloadSize=size;

				unsigned char payloadSizeLB= payloadSize & 0xFF;
				unsigned char payloadSizeMB= (payloadSize & 0xFF00) >> 8;

				int i=0;

				tmpBuf[i++]=1;
				tmpBuf[i++]=19;
				tmpBuf[i++]=2;
				tmpBuf[i++]=103;
				tmpBuf[i++]=(unsigned char)119;
				tmpBuf[i++]=(char)payloadSizeLB;
				tmpBuf[i++]=(char)payloadSizeMB;

				for (int M=0; M<payloadSize; M++)
				{
				 tmpBuf[i++]=imgRaw[ptr++];

				 size--; if (size<=0) break;
				}



			   // i=0;
	   // unsigned int *ptrR;

		   //    ptrR=(unsigned int*)&sendBackValues[0];

		   //   unsigned char *cptr=(unsigned char*)&sendBackValues[0];

		   //    ptrR[0]=0x01130260;//0x60021301;       //
			//   ptrR[1]=0xFFFFFFFF;

			   //cptr[i++]=1;
			  // cptr[i++]=19;
			  // cptr[i++]=2;
			  // cptr[i++]=96;
			  // cptr[i++]=255;
			  // cptr[i++]=0;
			  // cptr[i++]=0;
			  // cptr[i++]=0;

			  // for (int L=2;L<100; L++) sendBackValues[L]=123.0;

			 //  printf("P1 %d %d %d %d \n",(int)cptr[8],(int)cptr[9],(int)cptr[10],(int)cptr[11]);

			   //sendUDC((char*)&sendBackValues[0],400*4+5);

				unsigned char *cptr=(unsigned char*)&sendBackValues[0];

					  //ptr[0]=0x01130260;//0x60021301;       //
					  //ptr[1]=0xFFFFFFFF;

					  cptr[0]=1;
					  cptr[1]=19;
					  cptr[2]=2;
					  cptr[3]=96;
					  cptr[4]=255;
					  cptr[5]=0;
					  cptr[6]=0;
					  cptr[7]=0;


			   for (int L=0; L<400*4+5; L++)
			   {
				   tmpBuf[i++]=cptr[L];

			   }


				sendUDC(tmpBuf,i);

	*/
}




