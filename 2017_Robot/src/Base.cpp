#include "base.h"
#include "cal.h"

/*
int pwmShooter2      =1;
int pwmPickupRoller  =2;
int pwmLeftArm       =3;
int pwmLeftMotor1    =4;
int pwmLeftMotor2    =5;
int pwmElevator      =6;
int pwmLeftShifter   =7;
int pwmShoes         =8;

int pwmShooter        =11;
int pwmAzimuth        =12;
int pwmRightArm       =13;
int pwmRightMotor1    =14;
int pwmRightMotor2    =15;
int pwmShooterHood    =16;
int pwmRightShifter   =17;
int pwmShoes2         =18;
 */

int pwmShooter2 = 0;
int pwmPickupRoller = 0;
int pwmLeftArm = 0;
int pwmElevator = 0;
int pwmShoes = 0;
int pwmAzimuth = 0;
int pwmRightArm = 0;
int pwmShooterHood = 0;
int pwmShoes2 = 0;
int pwmGraber = 0;

int pwmApptureBot = 0;
int pwmApptureMid = 0;
int pwmApptureTop = 0;

int pwmButterfly = 0;
int pwmFeeder = 0;
int pwmPickup = 0;
int pwmShooter = 0;
int pwmElevR = 0;
int pwmElevL = 0;
int pwmRoller = 0;
int pwmLift = 0;

//--------------------------
//--  2014 motor assignments
//--------------------------

int pwmLeftMotor1 = 1;
int pwmRightMotor1 = 2;
int pwmLeftShifter = 3;
int pwmRightShifter = 4;

int pwmToroUpDown = 5;
int pwmShooterR1 = 6;
int pwmShooterL1 = 7;
int pwmRoller1 = 8;


int pwmLeftMotor2 = 11;
int pwmRightMotor2 = 12;
int pwmBumperUpDown = 14;
int pwmCatcher = 15;
int pwmShooterR2 = 16;
int pwmShooterL2 = 17;
int pwmRoller2 = 18;

//tmp assignments

 //int pwmRoller1      =13;  //
// int pwmShooterR1    =14;  //
 //int pwmShooterL1    =15;  //pwmCatcher


//--------------------------
//--  2013 motor assignments
//--------------------------


 //int pwmLeftMotor1     =1;
 //int pwmRightMotor1    =2;
 //int pwmLeftShifter    =3;
 //int pwmRightShifter   =4;

 //int pwmGraber         =5;

 //int pwmApptureBot     =6;
 //int pwmApptureMid     =7;
 //int pwmApptureTop     =8;

 //int pwmButterfly      =9;

 //int pwmLeftMotor2     =11;
 //int pwmRightMotor2    =12;
 //int pwmFeeder         =13;
 //int pwmPickup         =14;
 //int pwmShooter        =15;
 //int pwmElevR          =16;
 //int pwmElevL          =17;
 //int pwmRoller         =0;
 //int pwmLift           =18;

 //--------------------------------------
 //--  Test motor assignments on 2012 bot
 //--------------------------------------

 /*
 int pwmLeftMotor1     = 4;
 int pwmRightMotor1    =14;
 int pwmLeftShifter    =0;
 int pwmRightShifter   =0;
 int pwmApptureTop     =0;
 int pwmApptureMid     =0;
 int pwmApptureBot     =0;

 int pwmLeftMotor2     = 5;
 int pwmRightMotor2    =15;
 int pwmFeeder         =0;
 int pwmPickup         =0;
 int pwmShooter        =0;
 int pwmElevR          =0;
 int pwmElevL          =0;
 int pwmRoller         =0;

 int pwmGraber         =8;

 int pwmLift           =11;

 int pwmButterfly      =7;
 */


uc p1_x = 0;
uc p2_x = 0;
uc p3_x = 0;
uc p4_x = 0;

uc p1_y = 0;
uc p2_y = 0;
uc p3_y = 0;
uc p4_y = 0;

uc p1_z = 0;
uc p2_z = 0;
uc p3_z = 0;
uc p4_z = 0;





float joyX[10] = { 0,0,0,0,0,0,0,0,0,0 };
float joyY[10] = { 0,0,0,0,0,0,0,0,0,0 };

int   p_sw[10][14] = { {0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0} };

float joyX2[10] = { 0,0,0,0,0,0,0,0,0,0 };
float joyY2[10] = { 0,0,0,0,0,0,0,0,0,0 };

float hatX[10] = { 0,0,0,0,0,0,0,0,0,0 };
float hatY[10] = { 0,0,0,0,0,0,0,0,0,0 };


float joyT[10] = { 0,0,0,0,0,0,0,0,0,0 };



int autonomous_mode = 0;

int disabled_mode = 0;

uc closeLoopFlag[8];

int axiaGain[8] = { AxiaGainRp,AxiaGainLp,AxiaGainUp,AxiaGainWp,AxiaGainDp,1,1,1 };
int axiaGainI[8] = { AxiaGainRi,AxiaGainLi,AxiaGainUi,AxiaGainWi,AxiaGainDi,1,1,1 };
int axiaGainD[8] = { AxiaGainRd,AxiaGainLd,AxiaGainUd,AxiaGainWd,AxiaGainDd,1,1,1 };
int axiaGainF[8] = { AxiaGainRf,AxiaGainLf,AxiaGainUf,AxiaGainWf,AxiaGainDf,0,0,0 };

li curError[8];
li lastLoc[8];
li curVelError[8];
li curVelErrorI[8];
li curErrorI[8];


//char buff[128]; //used to test remaining var space

li cmdPos[8];
li setPos[8];
li cmdVel[8];
li cmdAcc[8];
li curVel[8];
li endPos[8];

uc iRemote = 0;

float cmdLoc[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float cmdGain[20] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
float cmdOff[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float cmdDead[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
