
#ifndef __BASE
#define __BASE


#define TRUEE 1
#define FALSEE 0

#define useTalon 0

extern char TeamCameraIP[20];
extern char TeamDriveStationIP[20];

double GetClockk();

//#define TEAM14
//#define TEAM494
//#define TEAM70


//#ifdef TEAM70

//#define TeamIP "10.0.70.5"
//#define TeamCameraIP "10.0.70.11"

//#endif

extern float gyroLoc;

//double GetClock();

//#ifdef TEAM494

//#define TeamIP "10.4.94.5"
//#define TeamCameraIP "10.4.94.11"

//#endif

//#ifdef TEAM14

//#define TeamIP "10.0.14.5"
//#define TeamCameraIP "10.0.14.11"

//#endif

#define BaseId 20

#define FrontDriveRt_Num      0
#define RearDriveRt_Num       1
#define WinchMotor1_Num       2
#define ToolBar_Num           3
#define HeadLights_Num        4
#define ShooterElevation_Num  5
#define ToroRight_Num         6
#define BallFeed_Num          7
#define SpareTalon1_Num       8
#define ToroLeft_Num          9
#define ShooterBallShoe_Num   10
//#define ShareTalon2_Num       11
#define ShooterWheel_Num      12
#define WinchMotor2_Num       13
#define RearDriveLt_Num       14
#define FrontDriveLt_Num      15

#define FrontDriveRt_Id      BaseId+0
#define RearDriveRt_Id       BaseId+1
#define WinchMotor1_Id       BaseId+2
#define ToolBar_Id           BaseId+3
#define HeadLights_Id        BaseId+4
#define ShooterElevation_Id  BaseId+5
#define ToroRight_Id         BaseId+6
#define BallFeed_Id          BaseId+7
#define SpareTalon1_Id       BaseId+8
#define ToroLeft_Id          BaseId+9
#define ShooterBallShoe_Id   BaseId+10
#define ShareTalon2_Id       BaseId+11
#define ShooterWheel_Id      BaseId+12
#define WinchMotor2_Id       BaseId+13
#define RearDriveLt_Id       BaseId+14
#define FrontDriveLt_Id      BaseId+15


#define FrontDriveRt_Inv      0
#define RearDriveRt_Inv       0
#define WinchMotor1_Inv       0
#define ToolBar_Inv           0
#define HeadLights_Inv        0
#define ShooterElevation_Inv  0
#define ToroRight_Inv         0
#define BallFeed_Inv          0
#define SpareTalon1_Inv       0
#define ToroLeft_Inv          0
#define ShooterBallShoe_Inv   0
#define ShareTalon2_Inv       0
#define ShooterWheel_Inv      0
#define WinchMotor2_Inv       0
#define RearDriveLt_Inv       0
#define FrontDriveLt_Inv      0


//#define pwmLeftFront  FrontDriveLt
//#define pwmRightFront FrontDriveRt
//#define pwmLeftRear   RearDriveLt
//#define pwmRightRear  RearDriveRt
//#define pwmRightToro  ToroRight
//#define pwmLeftToro   ToroLeft


#ifdef _WIN32
#define uc unsigned char
#define ui unsigned short
#define li signed
#define lu unsigned
#define suc static unsigned char
#define int short
#endif


#ifndef _WIN32
#define uc unsigned char
#define ui unsigned int
#define li long int
#define lu unsigned long int
#define suc static unsigned char
#endif


#define tprintf printf
//tp ();printf
#define pprintf printf
//tpp();printf

#define ttprintf tp ();printf


#define shiftFast    0
#define shiftSlow    128
#define shiftNetural 255

#define aRightDist    1
#define aLeftDist     2

/*
#define pwmShooter2       1
#define pwmPickupRoller   2
#define pwmLeftArm        3
#define pwmLeftMotor1     4
#define pwmLeftMotor2     5
#define pwmElevator       6
#define pwmLeftShifter    7
#define pwmShoes          8



#define pwmShooter        11
#define pwmAzimuth        12
#define pwmRightArm       13
#define pwmRightMotor1    14
#define pwmRightMotor2    15
#define pwmShooterHood    16
#define pwmRightShifter   17
#define pwmShoes2         18
*/

//DefineVariables

extern float move_OverHeadDroppoff;
extern float TwoCubePickupAngle;
extern float move_TwoCubeDistance;
extern float move_CornerToScale;
extern float strafe_behindHeadDropoff;
extern float strafe_secondCubePickup;
extern float strafe_firstCubePickup;
extern float LiftScaleBehind;
extern float ToolBarScaleBehind;
extern float ultra3Distance;
extern float ultra2Distance;
extern float ultra1Distance;
extern float move_CornerDropCube;
extern float move_CrossDropCube;
extern float HoldCube;
extern float TwoCube;
extern float LiftClimbSide;
extern float ToolBarClimbSide;
extern float Crossover;
extern float LiftClimb;
extern float ToolBarClimb;
extern float favorScale;
extern float move_ScalefromBehind;
extern float move_FaceScale;
extern float move_SwitchfromBehind;
extern float move_BehindSwitch;
extern float move_SideCenterofSwitch;
extern float move_TouchSwitch;
extern float move_CenterofSwitch;
extern float move_ApproachSwitch;
extern float matchNumber;
extern float matchTimeLeft;
extern float ToolBarPickup;
extern float robotWatts;
extern float EncoderGain;
extern float LiftGain;
extern float ToolBarGain;
extern float TuningVar5;
extern float TuningVar4;
extern float TuningVar3;
extern float TuningVar2;
extern float TuningVar1;
extern float ToolBarScaleHigh;
extern float ToolBarScaleMid;
extern float ToolBarScaleLow;
extern float ToolBarSwitch;
extern float LiftScaleHigh;
extern float LiftScaleMid;
extern float LiftScaleLow;
extern float LiftSwitch;
extern float LiftLow;
extern float LiftDest;
extern float ToolBarSpeed;
extern float Lift2Volts;
extern float Lift1Volts;
extern float Lift2Amps;
extern float Lift1Amps;
extern float Lift2Speed;
extern float Lift1Speed;
extern float ToolBarAngle;
extern float CubeIntakeAngle;
extern float LiftHeight;
extern float BatteryVoltage;
extern float RBD2Amps;
extern float RBD1Amps;
extern float RFD2Amps;
extern float RFD1Amps;
extern float LBD2Amps;
extern float LBD1Amps;
extern float LFD2Amps;
extern float LFD1Amps;
extern float LFD1Speed;
extern float RBD2Speed;
extern float RBD1Speed;
extern float LBD2Speed;
extern float LBD1Speed;
extern float RFD2Speed;
extern float RFD1Speed;
extern float LFD2Speed;
extern float EncoderVelocity;
extern float EncoderDistance;
extern float aton_HopperRPM;
extern float aton_ShooterRPM;
extern float atonNum;
extern float maxGoalAimSpeed;
extern float GoalAimGain;
extern float hopperShootRPM;
extern float gearDropoffSpeed;
extern float gearPickupSpeed;
extern float maxPegAimSpeed;
extern float maxPegDriveSpeed;
extern float pegGoalDriveScale;
extern float pegGoalAimScale;
extern float camerafps;
extern float PegGoalY1;
extern float PegGoalX1;
extern float BoilerGoalY3;
extern float BoilerGoalY2;
extern float BoilerGoalY1;
extern float BoilerGoalX3;
extern float BoilerGoalX2;
extern float BoilerGoalX1;
extern float BoilerErrorScale;
extern float cam1dr;
extern float cam2dr;
extern float cam2dg;
extern float cam1dg;
extern float ShooterShootRPM;
extern float HopperShootSpeed;
extern float ToolBarGearPickup;
extern float ToolBarGearDropoff;
extern float PegTargetGot;
extern float PegGoalY;
extern float PegGoalX;
extern float BoilerTargetGot;
extern float TurretScale;
extern float TurretPOS;
extern float HopperRPM;
extern float HopperPos;
extern float selectCamera;
extern float BoilerGoalY;
extern float BoilerGoalX;
extern float GyroZ;
extern float GyroY;
extern float GyroX;
extern float HopperSpeed;
extern float HopperVolts;
extern float HopperAmps;
extern float BallIntakeAmps;
extern float BallIntakeVolts;
extern float BallIntakeSpeed;
extern float IntakeRightAmps;
extern float IntakeLeftVolts;
extern float IntakeLeftAmps;
extern float IntakeRightVolts;
extern float IntakeRightSpeed;
extern float IntakeLeftSpeed;
extern float Winch2Volts;
extern float Winch2Speed;
extern float Winch2Amps;
extern float Winch1Amps;
extern float Winch1Volts;
extern float Winch1Speed;
extern float ShooterDest;
extern float ShooterAmps;
extern float ShooterVolts;
extern float ToolBarAmps;
extern float AtonFinalRotate;
extern float CrossY4;
extern float CrossX4;
extern float CreapSpeed;
extern float a6_Rotate;
extern float a5_Rotate;
extern float a4_Rotate;
extern float a3_Rotate;
extern float a2_Rotate;
extern float a1_Rotate;
extern float a6_MoveTime1;
extern float a5_MoveTime1;
extern float a4_MoveTime1;
extern float a3_MoveTime1;
extern float a2_MoveTime1;
extern float TrackingGainY;
extern float TrackingY;
extern float TrackingX;
extern float TrackingGain;
extern float gyro_Zero;
extern float RotateLeftAngle;
extern float RotateRightAngle;
extern float CrossY3;
extern float CrossY2;
extern float CrossY1;
extern float CrossX3;
extern float CrossX2;
extern float CrossX1;
extern float GyroGain;
extern float a1_MoveTime1;
extern float ToolBarLoc;
extern float ToolBarError;
extern float ToolBarVolts;
extern float ToolBarDest;
extern float ToolBarAuto;
extern float ToolBarScale;
extern float ToolBarHigh;
extern float ToolBarMid;
extern float ToolBarLow;
extern float GyroLoc;
extern float UseGyro;
extern float LookForGoal;



//-----------------


extern float tableSize;
extern float table1[200];
extern float table2[200];
extern float table3[200];
extern float table4[200];


extern int pwmShooter2      ;
extern int pwmPickupRoller  ;
extern int pwmLeftArm       ;
extern int pwmLeftMotor1    ;
extern int pwmLeftMotor2    ;
extern int pwmElevator      ;
extern int pwmLeftShifter   ;
extern int pwmShoes         ;

extern int pwmBumperUpDown  ;

extern int pwmToroUpDown;
extern int pwmCatcher;

extern int pwmShooter        ;
extern int pwmAzimuth        ;
extern int pwmRightArm       ;
extern int pwmRightMotor1    ;
extern int pwmRightMotor2    ;
extern int pwmShooterHood    ;
extern int pwmRightShifter   ;
extern int pwmShoes2         ;

extern int pwmFeeder        ;
extern int pwmElevL;
extern int pwmElevR;

extern  int pwmShooterR1;
extern  int pwmShooterL1;
extern  int pwmShooterR2;
extern  int pwmShooterL2;

extern int pwmRoller1;
extern int pwmRoller2;


extern int pwmApptureTop;
extern int pwmApptureMid;
extern int pwmApptureBot;

extern int pwmButterfly;


//extern int pwmGraber;

//extern int pwmLift;



//#define aAzimuth  3
//#define aHood     4
//#define aArm      5

//#define aFeeder 3
//#define aElev   4
//#define aArm    5

#define aSonar 3

//#define aBinLift  0
//#define aToteLift 1
//#define aShoulder 2
//#define aElbo     3







//#define pwmToteIntakeRight    0
//#define pwmBinArmAzm          1
//#define pwmBinJaw             2
//#define pwmBinArmWrist        3
//#define pwmBinArmElbow        5
//#define pwmToteForkRetract    6
//#define pwmToteForkIntakeLeft 7
//#define pwmCop                8

//#define pwmLidarServo         9

#define pwmHangServo 0


//void BinArmAzm         (float speed) ;
//void BinJaw            (float speed) ;
//void BinArmElbow       (float speed) ;
//void BinArmWrist       (float speed) ;
//void ToteForkRetract   (float speed) ;
//void ToteForkIntake    (float speed) ;
//void Cop               (float speed) ;
//void ToteLift (float speed) ;
//void BinLift1 (float speed) ;

void  tMoveP( float dest, float speed, float accel,void (*finished)(),float maxTime);

void HangServoToZero();
void HangServoToNinty();


void Set_FrontDriveRt       (float speed);
void Set_RearDriveRt        (float speed);
void Set_WinchMotor1		(float speed);
void Set_ToolBar			(float speed);
void Set_HeadLights			(float speed);
void Set_ShooterElevation	(float speed);
void Set_ToroRight			(float speed);
void Set_BallFeed			(float speed);
void Set_SpareTalon1		(float speed);
void Set_ToroLeft			(float speed);
void Set_ShooterBallShoe	(float speed);
void Set_ShareTalon2		(float speed);
void Set_ShooterWheel		(float speed);
void Set_WinchMotor2		(float speed);
void Set_RearDriveLt		(float speed);
void Set_FrontDriveLt		(float speed);

double Get_Speed_FrontDriveRt       ();
double Get_Speed_RearDriveRt        ();
double Get_Speed_WinchMotor1		();
double Get_Speed_ToolBar			();
double Get_Speed_HeadLights		    ();
double Get_Speed_ShooterElevation   ();
double Get_Speed_ToroRight		    ();
double Get_Speed_BallFeed	        ();
double Get_Speed_SpareTalon1		();
double Get_Speed_ToroLeft		    ();
double Get_Speed_ShooterBallShoe    ();
double Get_Speed_ShareTalon2	    ();
double Get_Speed_ShooterWheel	    ();
double Get_Speed_WinchMotor2		();
double Get_Speed_RearDriveLt		();
double Get_Speed_FrontDriveLt	    ();

double Get_Loc_FrontDriveRt       ();
double Get_Loc_RearDriveRt        ();
double Get_Loc_WinchMotor1		  ();
double Get_Loc_ToolBar			  ();
double Get_Loc_HeadLights		  ();
double Get_Loc_ShooterElevation   ();
double Get_Loc_ToroRight		  ();
double Get_Loc_BallFeed	          ();
double Get_Loc_SpareTalon1		  ();
double Get_Loc_ToroLeft		      ();
double Get_Loc_ShooterBallShoe    ();
double Get_Loc_ShareTalon2	      ();
double Get_Loc_ShooterWheel	      ();
double Get_Loc_WinchMotor2		  ();
double Get_Loc_RearDriveLt		  ();
double Get_Loc_FrontDriveLt	      ();



extern float binDropLoc;






/*
#define aGyroPort     3
#define aAzimuth      4
#define aUpDown       5
#define aArmElevation 6

#define aRightWheel 1
#define aLeftWheel  2

#define pCimFrontRight 1
#define pCimMidRight   2
#define pCimBackRight  3
#define pCimFrontLeft  4
#define pCimMinLeft    5
#define pCimBackLeft   6


#define pHood         12
#define pUpperRoller    13
#define pLowerRoller    14
#define pArmElevation   15
#define pMiniBotRelease 16



#define aBatteryVolts 8

#define aFR 1
#define aFL 2
#define aBR 3
#define aBL 4
#define aFS 5
#define aBS 6
#define aKH 7

#define aWN 11
#define aW1 11
#define aW2 12
#define aSH 13
#define aSD 14

#define aEL    15
#define aAZ    16
#define aPK    17
*/


const int xboxA=1;
const int xboxB=2;
const int xboxX=3;
const int xboxY=4;
const int xboxLS=5;
const int xboxRS=6;
const int xboxBack=7;
const int xboxStart=8;
const int xboxLeftStickClick=9;
const int xboxRightStickClick=10;

const int leftStickX=1;
const int leftStickY=2;
const int triggers=3;
const int rightStickX=4;
const int rightStickY=4;

const int xboxUp   =11;
const int xboxDown =13;
const int xboxRight=12;
const int xboxLeft =14;

extern int blockDriving;
extern int sideDrive;

extern int tripShift   ;
extern int tripStopDrop;

extern float cmdLoc [20];
extern float cmdGain[20];
extern float cmdOff [20];
extern float cmdDead[20];

extern float rawLoc[21];

extern int autoKick;

extern float joyX[10];
extern float joyY[10];


extern float joyX2[10];
extern float joyY2[10];

extern float hatX[10];
extern float hatY[10];


extern float joyT[10];

extern int   p_sw[10][14];

extern float loc[21];

extern int revCount[20];

extern char sampleGood   [8];
extern int  AnalogSamplee [8];

extern int begRevTime[8];

extern int autonomous_mode;

extern int disabled_mode;

extern float speedLimit;

extern int newDrivingInfo;

//status info

extern int currentTeam; // 0: no teams 1: red team 2: blue Team 3: both teams

extern int trackRedTeam ;
extern int trackBlueTeam;

extern  float inAton;
extern  float inTele;
extern  float inDisable;

extern int redTeam ;
extern int blueTeam;

extern float battery;


extern float crossHairX;

extern int    forceCount[21];
extern double forceValue[21];


//----------- load/save variables --------------

extern float minDisplayLine;
extern float maxDisplayLine;
extern float blackAndWhite;

extern int shooterMode;



extern int blockGyroReset;

extern int stopAllDriveTrain;
extern float atonDelayTime;

extern float ArmGain;
extern float ArmRes;


extern	float  toroBlockAfterShotTime ;




extern float bumperIsDown;
extern float bumperIsUp;

extern float holdGain;
extern float holdGainR;

extern float turretGain;

extern float shotBlocker;
extern float teamNumber;

extern float minShootTime;
extern float moveBackRate;




//extern int   forceSteer;
//extern float forceSteerF;
//extern float forceSteerB;
extern int   forceSpeed;

extern float windUpMin  ;
extern float windUpMax  ;
extern float windUpSpeed;
extern float windUpMaxTime;

extern float rawKickMin  ;
extern float rawKickMax  ;
extern float rawKickSpeed;
extern float rawKickMaxTime;

extern float spinCener; //=80;
extern float spinGain;  //=5;
extern float gyroSpinScale; //=3;
extern float spinMin;   //=13;

extern float targetLockCount; //=5;
extern float autoKickCount;   //=5;
extern float autoKickRange;   //=5;
extern float iGainSpinKick;

extern float a1BallAngle;    //8
extern float a2BallAngle;    //3.18
extern float a3BallAngle;    //1.9;

extern float goalWidth; //36

extern float RH_ShotAngBeg           ;
extern float RH_ShotAngEnd        ;
extern float RH_ShotSpeed       ;
extern float trussShotDist  ;



extern int mTargetX;
extern int mTargetY;
extern int mTargetZ;
extern int mTargetW;

extern int inTestAton;

extern int inAt;

extern float mTargetA;

extern float useLeftDist;

extern int gyroFault;

extern  float tCX1;
extern  float tCX2;
extern  float tCX3;
extern  float tCX4;
extern  float tCY1;
extern  float tCY2;
extern  float tCY3;
extern  float tCY4;
extern  float tH1 ;
extern  float tH2 ;
extern  float tH3 ;
extern  float tH4 ;
extern  float tW1 ;
extern  float tW2 ;
extern  float tW3 ;
extern  float tW4 ;

extern float targetOffsetY;
extern float minWheelShootSpeed;

void lockStraightDrive(int timeOut);

void resetGyro();
float getGyroAngle(void);

void eMes(char *statusString);
void setStopDropAngle(float angle);
void setShifterAngle(float angle);

void CP(char* val);
void CB(char* val);
void CE(char* val);

//double GetClock(void);



void safe(void(*ptrAdr)());

void resetDist();


extern float spinCener;
extern float spinGain ;
extern float gyroSpinScale;
extern float spinMin  ;
extern float targetLockCount;
extern float autoKickCount;
extern float autoKickRange;

extern float gyroAdjust;

//extern float driveStraightGyro;

extern int spinFlag;

extern double spinAngle;

extern float iSpinGain;

extern int atonActive;

extern int blockGyroReset;

extern int steeringOff;

void sendBack(int index,float value);

void eMes(char *statusString);

extern float atonVar[61];

extern  float matchTime;
extern  double matchTimeBase;
extern  float  matchStatus;

//extern  float shoesFlag;
extern  float ballDetect2     ;
extern  float armFlag         ;
extern  float elevFlag        ;
extern  float armSpeed        ;
extern  float lookForHotGoal      ;
extern  float hotGoal     ;


extern float crossHairOffset ;
extern float lightsOffTime  ;
extern float lightsOnTime    ;
extern float WheelAtSpeed;
extern float ShiftModeFlag;
extern float ShiftLowRes;
extern float ShiftHighRes;


extern  float shoesMin1    	  ;
extern  float shoesMax1    	  ;
extern  float shoesMin2    	  ;
extern  float shoesMax2    	  ;
extern  float rightShiftMin   ;
extern  float rightShiftMax	  ;
extern  float leftShiftMin 	  ;
extern  float leftShiftMax 	  ;



extern  float curHotRatio ;
extern  float minHotRatio;




 extern int pwmPickup;
 extern int pwmElevR;
 extern int pwmElevL;
 extern int pwmRoller;


extern int mTargetX;
extern int mTargetW;

extern int atonSpin;
//extern int sideDrive;

//extern int backDrive;

extern int fastGyro;



extern int mTargetX;

void NextVideoMode();

extern int steeringTimeOut;

//extern float steeringGain;


extern float joyX[10];
extern float joyY[10];

extern int cameraLookFlag;

 extern int mTargetX;
 extern int mTargetY;
 extern int mTargetZ;

 extern float loc[21];



 extern int   totalLight;
 extern float targetTime;


 extern int   imgWorkNew;
 extern int   rawImageCount;
 extern char  imgWork[320*242*4];
 extern float offsetAdjust;
 extern float robotSpeed;
 extern int   atonSpin;
 extern int   crabDrive;

// extern int    steeringTimeOut;
 extern double updateRate;

 //extern int spitIt;
 //extern int rollIt;

 extern float atonDelayTime;

 //extern int sideDrive;

 extern int steeringOff;

 //extern float driveStraightGyro;

 //extern int sideDrive;

 extern float atonDelayTime;

 void setupUserDataFullColor6Bit(char* Buf,int maxSize);
 void setupUserDataBW6bit(char* Buf,int maxSize);
 void setupUserDataBW6bitTC(char* Buf,int maxSize);
 void setupUserDataBW6bitLR(char* Buf,int maxSize);
 void setupUserDataFullColor3Bit(char* Buf,int maxSize);


 int FindGreenTargets (char *imgBuf, int wide,int high );
 int FindTarget3b(char *imgBuf, int wide,int high );

 void processRawImage();

 void updateDashboardLights(void);
 void setupTimers(void);
 void motorInit();
 void encoderInit();

 void SaveGlobalData();
 void LoadGlobalData();

 void UpdateDashboard();

 void ReadEncoderRaw(int i,float newSample );

 void IFC_Local_Loop_Service(void);

 void updatePWM(void);

 //void updateSteering(float front,float back,int crabFlag,int forceBackWheel);

 int StopCameraTask();

 void cameraServoService(float offset);

 void GlobalSendBack(void);

 void setPwmFg(int L,float val);

 //float fabs(float val);

 //double GetClock(void);

 extern int windItUp;

 //extern int kickActive;

 //void kickBallLow();
 //void kickBall();

 //extern int noUnSpin;


 extern float forceKickRollerRate;

 extern int blockStick2;

 //------------------------------------


  //---------- Level Parameters -----------------

  extern float DistanceH;
  extern float DistanceY;
  extern float FilterHYFlag;
  extern float FilterHYLimit;
  extern float PickupRollerFlag;
  extern float DebugRobotVideo;
  extern float PickupRollerSpeed;
  extern float rollerCollectRateFast;


 extern  float  GoalFound;
 extern  float	GoalX;
 extern  float	GoalY;
 extern  float	GoalW;
 extern  float	GoalH;

 extern float autoDriveX;
 extern float autoDriveY;
 extern float autoDriveR;



  //extern float shoesFlag;

  extern float sSpeed[20];
  extern float sHood [20];
  extern float aCmd [20];


  extern float toroInManual;
  extern float catcherIsOpen;
  extern float catcherIsClosed;


  extern float armManualFlag;
  extern float rapidShootElevSpeed;
  extern float gyroShift;
  extern float gyroShiftState;
  extern float pickupAngle;
  extern float elevBrake;


  //---------- Roller Parameters ---------------

  extern float lowerSuckRate;
  extern float wantedShooterRPM;

  extern float shooterRPM;
  extern float shooterSpeed;

  extern float lowerRollRate;
  extern float upperRollRate;

  //------------ Aton Parameters ----------------



  //--------- line tracking ---------

  extern float armDownLimit;

  //----------------------------------

  extern float trackTimeOut;
  extern float trackLockMin;
  extern float trackCenterY;
  extern float trackCenterX;
  extern float trackState  ;
  extern float gfMinLevel  ;
  extern float bfMaxWide   ;
  extern float bfMaxHigh   ;
  extern float bfMaxRatio  ;
  extern float trackX      ;
  extern float trackY      ;
  extern float trackZ      ;

  extern float useLeftPot;
  extern float wantedArmAngle;
  extern float crossHair2X;
  extern float crossHair2Y;

  extern float sweepRate;
  extern float ShooterPulseMaxRPM  ;
  extern float ShooterPulseOffset ;

  extern float tIndex;
  extern float tValue1;
  extern float tValue2;
  extern float tValue3;
  extern float holdStraight;
  extern float tableSize;
  extern float liftFlag;
  extern float liftTime;


  extern float DistAdjH1    ;
  extern float DistAdjH2    ;
  extern float DistAdjY1    ;
  extern float DistAdjY2    ;
  extern float DistAdjDistH1;
  extern float DistAdjDistH2;
  extern float DistAdjDistY1;
  extern float DistAdjDistY2;

  extern float ShooterModeFlag;

  extern float lightsOffTimeInZone;
  extern float lightsOnTimeInZone;
  extern float AppReady;

  extern float AppStateTop;
  extern float AppStateMid;
  extern float AppStateBot;



  extern  float  ahArmAngle       ;
  extern  float  ahStartSpitDelay ;
  extern  float  ahStopSpitDelay  ;
  extern  float  ahStartDownDelay ;
  extern  float  ahStartArmUpDelay;

  extern float  maxManualSpitTime;

  extern float atonTopArmAngle;


  extern float eLoopCount;

 void moveHood(float dest,float vel,void(*doAtFinish)());
 void moveArm   (float dest,float vel,void(*doAtFinish)());
 void moveAzimth(float dest,float vel,void(*doAtFinish)());
 void moveElev  (float dest,float vel,void(*doAtFinish)());


 extern int blockArmUser;
 extern int blockAzimthUser;
 extern int blockHoodUser;

 void startAton( int atonNum);







 void setPwm  (int L,float val);



 extern float elevUpRate  ;
 extern float elevDownRate;
 extern float DriveModeFlag;


 // 2014 Shooter




  extern  float shooterEndDuration;




void ZeroAll  ();
void ZeroHood();
void ZeroArm  ();
void ZeroAzimt();


void ZeroAll  ();



bool IsDisabled();



void tp();
void tpp();

extern double lastClock;


 extern float a1_Cmd ;
 extern float a2_Cmd ;
 extern float a3_Cmd ;
 extern float a4_Cmd ;
 extern float a5_Cmd ;
 extern float a6_Cmd ;



 extern float cameraSelect;

 extern float rightDriveAdj;
 extern float leftDriveAdj;

 extern float lineTrackAdj;

 extern float rightDriveAdjN;
 extern float leftDriveAdjN;

 extern float packetSize;

 extern float autoDeployTime;

 extern int rollUsed;

 extern int gyroInitFinished;

 extern float hoodGain;
 extern float hoodMin;
 extern float hoodMax;


 extern float shoesOut2;
 extern float shoesIn2;


 extern float A_ShotDistBeg ;
 extern float A_ShotDistZone ;
 extern float B_ShotDistBeg ;
 extern float B_ShotDistZone ;
 extern float X_ShotDistBeg ;
 extern float X_ShotDistZone ;
 extern float Y_ShotDistBeg ;
 extern float Y_ShotDistZone ;
 extern float RS_ShotDistBeg;
 extern float RS_ShotDistZone;
 extern float LH_ShotDistBeg;
 extern float LH_ShotDistZone;
 extern float RH_ShotDistBeg;
 extern float RH_ShotDistZone;




extern float goalPixHigh;
extern float goalPixWide;

extern float toroIsSpitting;
extern float toroIsCollecting;
extern float toroIsDown;
extern float toroIsUp;

extern float quickShotAngle;
extern float longShotDelay;
extern float longShotAngle;
extern float shotDelay;


//----------------------- 2015 variables ---------------------

extern float tLoadedCount;
extern float tPickupLevel;
extern float tFloorLevel;
extern float tCarryLevel;
extern float tLoadLevel;
extern float tStepLevel;
extern float tStepLevel1;
extern float tStepLevel2;
extern float tStepLevel3;
extern float tGain;
extern float tZero;
extern float tResolution;
extern float tMaxUpSpeed;
extern float tMaxDnSpeed;
extern float tMode;
extern float tTimeout;
extern float bPickupLevel;
extern float bFloorLevel;
extern float bCarryLevel;
extern float bLoadLevel;
extern float bStepLevel;
extern float bStepLevelUp;
extern float bToteLevel1;
extern float bToteLevel2;
extern float bToteLevel3;
extern float bToteLevel4;
extern float bToteLevel5;
extern float bToteLevel6;
extern float bToteTopClr;
extern float bGain;
extern float bResolution;
extern float bMaxUpSpeed;
extern float bMaxDnSpeed;
extern float bMode;
extern float bTimeout;
extern float bZero;
extern float aPickupAngle;
extern float aFloorAngle;
extern float aCarryAngle;
extern float aLoadAngle;
extern float aStepAngle;
extern float aStepAngleUp;
extern float aToteAngle1;
extern float aToteAngle2;
extern float aToteAngle3;
extern float aToteAngle4;
extern float aToteAngle5;
extern float aToteAngle6;
extern float aToteTopClr;
extern float aGain;
extern float aResolution;
extern float aMaxCWSpeed;
extern float aMaxCCWSpeed;
extern float aMode;
extern float aTimeout;
extern float ePickupAngle;
extern float eFloorAngle;
extern float eCarryAngle;
extern float eLoadAngle;
extern float eStepAngle;
extern float eStepAngleUp;
extern float eToteAngle1;
extern float eToteAngle2;
extern float eToteAngle3;
extern float eToteAngle4;
extern float eToteAngle5;
extern float eToteAngle6;
extern float eToteTopClr;
extern float eGain;
extern float eResolution;
extern float eMaxCWSpeed;
extern float eMaxCCWSpeed;
extern float eMode;
extern float eTimeout;
extern float a1DriveTime;
extern float a1BinLevel;
extern float a1DriveSpeed;
extern float a2GrabAndLiftTime;
extern float a2DropOffTime;
extern float a2RightMoveTimeB1;
extern float a2RightMoveSpeedB1;
extern float a2LeftMoveTimeB1;
extern float a2LeftMoveSpeedB1;
extern float a2ForwardMoveTimeB2;
extern float a2ForwardMoveSpeedB2;
extern float a2RightMoveTimeB2;
extern float a2RightMoveSpeedB2;
extern float a2LeftMoveTimeB2;
extern float a2LeftMoveSpeedB2;
extern float a2ForwardMoveTimeB3;
extern float a2RightMoveTimeB3;
extern float a2RightMoveSpeedB3;
extern float a3ShoulderOutAngle;
extern float a2ElboOutAngle;
extern float a3InToGrapTime;
extern float a3InToGrapSpeed;
extern float a3OutToGrapTime;
extern float a3OutInToGrapSpeed;
extern float a3ForawrdMoveTime;
extern float a3ForawrdMoveSpeed;
extern float a3BinDropOffTime;

extern float a2DropAngleAmz;
extern float a2DropAngleElbo;
extern float a2PickupAngleAmz;
extern float a2PickupAngleElbo;
extern float a2DropCanTime;

extern float atonLimit;



extern float lidarSelect;
extern float tCmdPos;
extern float bCmdPos;
extern float aCmdPos;
extern float eCmdPos;
extern float sCmdPos;


extern float lidarValue;

extern float a4BinLevel;
extern float a4DriveSpeed;
extern float a4DriveTime;


extern float a5Amz;
extern float a5Elbo;
extern float atonSuckSpeed;
extern float userSuckSpeed;

extern float a5MoveSpeed;
extern float a5MoveTime;
extern float binDropLoc;

//-----------------------------------------------------------



 extern float preDeploy;

void drivingService();

void startAtonSelected();

	 int setServo(int L,float val);


void lineTrack( float dist,float speed,float maxTime,float trackTypeP,void(*serviceAddress)() );

void setEncoder(int i,float newLoc);

void setDist( float dist );

void rotateRobot( float dest,float speed,float maxTime, void(*serviceAddress)() );

void delayedMiniBot();

//--------------------- new 2012 ---------------

void setRightDriveTrain(int val,int shift);
void setLeftDriveTrain(int val ,int shift);

void cmdOpenButterfly();

void cmdCloseButterfly();

//-------------- new 2015 ---------

void tMove(float dest, void(*finished)()=0,float maxTime=5);
void bMove(float dest, void(*finished)()=0,float maxTime=5);
void aMove(float dest, void(*finished)()=0,float maxTime=5);
void eMove(float dest, void(*finished)()=0,float maxTime=5);
void sMove(float dest, void(*finished)()=0,float maxTime=5);


void MoveTo_bPickupLevel     (void(*finished)());
void MoveTo_bFloorLevel      (void(*finished)());
void MoveTo_bCarryLevel      (void(*finished)());
void MoveTo_bLoadLevel       (void(*finished)());
void MoveTo_bStepLevel        (void(*finished)());
void MoveTo_bStepLevelUp      (void(*finished)());
void MoveTo_bToteLevel1       (void(*finished)());
void MoveTo_bToteLevel2     (void(*finished)());
void MoveTo_bToteLevel3     (void(*finished)());
void MoveTo_bToteLevel4     (void(*finished)());
void MoveTo_bToteLevel5     (void(*finished)());
void MoveTo_bToteLevel6     (void(*finished)());
void MoveTo_bToteTopClr     (void(*finished)());
void MoveTo_aPickupAngle     (void(*finished)());
void MoveTo_aFloorAngle     (void(*finished)());
void MoveTo_aCarryAngle     (void(*finished)());
void MoveTo_aLoadAngle     (void(*finished)());
void MoveTo_aStepAngle     (void(*finished)());
void MoveTo_aStepAngleUp     (void(*finished)());
void MoveTo_aToteAngle1     (void(*finished)());
void MoveTo_aToteAngle2     (void(*finished)());
void MoveTo_aToteAngle3     (void(*finished)());
void MoveTo_aToteAngle4     (void(*finished)());
void MoveTo_aToteAngle5     (void(*finished)());
void MoveTo_aToteAngle6     (void(*finished)());
void MoveTo_aToteTopClr     (void(*finished)());
void MoveTo_ePickupAngle     (void(*finished)());
void MoveTo_eFloorAngle     (void(*finished)());
void MoveTo_eCarryAngle     (void(*finished)());
void MoveTo_eLoadAngle     (void(*finished)());
void MoveTo_eStepAngle     (void(*finished)());
void MoveTo_eStepAngleUp     (void(*finished)());
void MoveTo_eToteAngle1     (void(*finished)());
void MoveTo_eToteAngle2     (void(*finished)());
void MoveTo_eToteAngle3     (void(*finished)());
void MoveTo_eToteAngle4     (void(*finished)());
void MoveTo_eToteAngle5     (void(*finished)());
void MoveTo_eToteAngle6     (void(*finished)());
void MoveTo_eToteTopClr     (void(*finished)());
void MoveTo_tPickupLevel     (void(*finished)());
void MoveTo_tFloorLevel     (void(*finished)());
void MoveTo_tCarryLevel     (void(*finished)());
void MoveTo_tLoadLevel     (void(*finished)());
void MoveTo_tStepLevel     (void(*finished)());
void MoveTo_tStepLevel1     (void(*finished)());
void MoveTo_tStepLevel2     (void(*finished)());
void MoveTo_tStepLevel3     (void(*finished)());

void MoveTo_bPickupLevel  ();
void MoveTo_bFloorLevel   ();
void MoveTo_bCarryLevel   ();
void MoveTo_bLoadLevel    ();
void MoveTo_bStepLevel    ();
void MoveTo_bStepLevelUp  ();
void MoveTo_bToteLevel1   ();
void MoveTo_bToteLevel2   ();
void MoveTo_bToteLevel3   ();  
void MoveTo_bToteLevel4   ();  
void MoveTo_bToteLevel5   ();  
void MoveTo_bToteLevel6   ();  
void MoveTo_bToteTopClr   ();  
void MoveTo_aPickupAngle  ();  
void MoveTo_aFloorAngle   ();  
void MoveTo_aCarryAngle   ();  
void MoveTo_aLoadAngle    ();  
void MoveTo_aStepAngle    ();  
void MoveTo_aStepAngleUp  ();  
void MoveTo_aToteAngle1   ();  
void MoveTo_aToteAngle2   ();  
void MoveTo_aToteAngle3   ();  
void MoveTo_aToteAngle4   ();  
void MoveTo_aToteAngle5   ();  
void MoveTo_aToteAngle6   ();  
void MoveTo_aToteTopClr   ();  
void MoveTo_ePickupAngle  ();  
void MoveTo_eFloorAngle   ();  
void MoveTo_eCarryAngle   ();  
void MoveTo_eLoadAngle    ();  
void MoveTo_eStepAngle    ();  
void MoveTo_eStepAngleUp  ();  
void MoveTo_eToteAngle1   ();  
void MoveTo_eToteAngle2   ();  
void MoveTo_eToteAngle3   ();  
void MoveTo_eToteAngle4   ();  
void MoveTo_eToteAngle5   ();  
void MoveTo_eToteAngle6   ();  
void MoveTo_eToteTopClr   ();  
void MoveTo_tPickupLevel  ();  
void MoveTo_tPickupLevel2 ();  
void MoveTo_tPickupLevel3 ();  
void MoveTo_tPickupLevel4 ();  
void MoveTo_tFloorLevel   ();  
void MoveTo_tCarryLevel   ();  
void MoveTo_tLoadLevel    ();  
void MoveTo_tLoadLevel2   ();  
void MoveTo_tLoadLevel3   ();  
void MoveTo_tStepLevel    ();  
void MoveTo_tStepLevel1   ();  
void MoveTo_tStepLevel2   ();  
void MoveTo_tStepLevel3   ();  
							  

 
void LoadOnceLow();
void LoadOnceLow(void(*finished)());

void LoadOnceHigh();
void LoadOnceHigh(void(*finished)());

void CloseJaws();
void OpenJaws();
void Suck();
void Spit();

float getToteLiftAct();

void resetGyro();

#endif									    
 
