//
// BS file
//

#include "joyStick.h"
#include "itimer.h"
#include <stdio.h>
#include "timer.h"


float inAton = 0;
float inTele = 0;
float inDisable = 0;

int redTeam = 0;
int blueTeam = 0;

float battery = 0;

int station = 0;




const signed char JOYSTICK_REMAP[128] =
{
0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2,
2, 2, 3, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8,
8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 16, 17, 17,
18, 19, 20, 20, 21, 22, 23, 24, 25, 25, 26, 27, 28, 29, 30, 31,
32, 33, 34, 35, 36, 37, 38, 39, 41, 42, 43, 44, 45, 46, 48, 49,
50, 51, 53, 54, 55, 56, 58, 59, 61, 62, 63, 65, 66, 68, 69, 71,
72, 74, 75, 77, 78, 80, 81, 83, 85, 86, 88, 89, 91, 93, 95, 96,
98, 100, 102, 103, 105, 107, 109, 111, 113, 114, 116, 118, 120, 122, 124, 126
};

int SoftStick(int input)
{

	if (input > 255) input = 255;
	if (input < 0) input = 0;

	if (input > 127) return ((int)(128 + JOYSTICK_REMAP[input - 128]));

	return ((int)(128 - JOYSTICK_REMAP[127 - input]));
}

int HardStick(int input)
{
	if (input > 255) input = 255;
	if (input < 0) input = 0;

	if (input > 200) input = 255;
	if (input < 50) input = 0;

	return input;
}

void(*trigBegAdr[56])(void);
void(*trigMidAdr[56])(void);
void(*trigEndAdr[56])(void);

unsigned int trigState[56] = { 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 };
unsigned int trigCount[56];

void onTrig(int stick, int trig, void(*begAdr)(void), void(*midAdr)(void), void(*endAdr)(void))
{
	int i = (stick - 1) * 14 + (trig - 1);

	if (stick == 0) i = trig;

	if (i < 0 || i>56) return;

	trigState[i] = 1;
	trigCount[i] = 0;

	if (begAdr == 0) begAdr = nullService;
	if (midAdr == 0) midAdr = nullService;
	if (endAdr == 0) endAdr = nullService;

	trigBegAdr[i] = begAdr;
	trigMidAdr[i] = midAdr;
	trigEndAdr[i] = endAdr;
}

void trigOff(int stick, int trig)
{
	int i = (stick - 1) * 14 + (trig - 1);

	if (stick == 0) i = trig;

	if (i >= 0 && i < 56) trigState[i] = 0;
}

//************************************************
//** triger sample with 0.05 second noise filter
//************************************************

int trigPressed(uc i)
{
	if (i > 56) return 0;

	int stick = i / 14;
	int trig = i - stick * 14;

	stick++;
	trig++;

	if (stick < 1 || stick>4) return 0;
	if (trig < 1 || trig >14) return 0;

	// printf("i %d stick %d trig %d\n",i,stick,trig);

	int pressed = p_sw[stick][trig];

	//if (pressed) printf("stick %d value %d\n",stick,trig);

	if (pressed)
		trigCount[i]++;
	else if (trigCount[i]) trigCount[i]--;

	if (trigCount[i] < (uc)5) pressed = 0;
	if (trigCount[i] > (uc)10) trigCount[i] = 10;

	return pressed;

} //trigPressed

void trigService()
{
	for (int i = 0; i < 56; i++)
	{
		int state = trigState[i];
		// if(state){printf("TriggerPressed\n");}
		if (state)
		{
			int pressed = trigPressed(i);
			switch (state)
			{

			case 1: if (pressed) { trigState[i] = 2; trigBegAdr[i](); } break;
			case 2: if (!pressed)  trigState[i] = 3; trigMidAdr[i](); break;
			case 3:                trigState[i] = 1; trigEndAdr[i](); break;

			}

		}

	}
} //trigService

//------------- keyboard control -------------


int keyPressDown = 0;
int keyPressUp = 0;

void(*keyBegAdr[256])(void);
void(*keyMidAdr[256])(void);
void(*keyEndAdr[256])(void);

unsigned int keyState[256] = { 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 };
unsigned int keyCount[256];

void onKey(int key, void(*begAdr)(void), void(*midAdr)(void), void(*endAdr)(void))
{
	int i = key;

	if (i < 1 || i>255) return;

	keyState[i] = 1;
	keyCount[i] = 0;

	if (begAdr == 0) begAdr = nullService;
	if (midAdr == 0) midAdr = nullService;
	if (endAdr == 0) endAdr = nullService;

	keyBegAdr[i] = begAdr;
	keyMidAdr[i] = midAdr;
	keyEndAdr[i] = endAdr;
}

void keyOff(int key)
{
	int i = key;

	if (i >= 0 && i < 256) keyState[i] = 0;
}

//************************************************
//** keyer sample with 0.05 second noise filter
//************************************************

int keyPressed(uc i)
{
	if (keyPressUp == keyPressDown) { keyPressUp = 0; keyPressDown = 0; return 0; }

	if (keyPressDown == i) return 1;

	return 0;

} //keyPressed

void initKeyState()
{
	for (int L = 0; L < 256; L++) keyState[L] = 0;
}

void keyService()
{

	for (int i = 1; i < 256; i++)
	{
		int state = keyState[i];

		if (state)
		{
			int pressed = keyPressed(i);

			switch (state)
			{

			case 1: if (pressed) { keyState[i] = 2; keyBegAdr[i](); } break;
			case 2: if (!pressed)  keyState[i] = 3; keyMidAdr[i](); break;
			case 3:                keyState[i] = 1; keyEndAdr[i](); break;

			}

		}

	}
} //keyService

//--------------- hat control


void(*hatBegAdr[56])(void);
void(*hatMidAdr[56])(void);
void(*hatEndAdr[56])(void);

unsigned int hatState[56] = { 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 };
unsigned int hatCount[56];

void onHatX(int stick, int hatDir, void(*begAdr)(void), void(*midAdr)(void), void(*endAdr)(void))
{
	return;

	if (stick < 1 || stick>4) return;
	if (hatDir < -1 || hatDir>1) return;

	hatDir++; if (hatDir == 1) hatDir = 0;

	int i = stick * 4 + hatDir; if (i < 0 || i>56) return;

	hatState[i] = 1;
	hatCount[i] = 0;

	if (begAdr == 0) begAdr = nullService;
	if (midAdr == 0) midAdr = nullService;
	if (endAdr == 0) endAdr = nullService;

	hatBegAdr[i] = begAdr;
	hatMidAdr[i] = midAdr;
	hatEndAdr[i] = endAdr;
}

void hatOff(int stick, int hatDir)
{
	return;

	if (stick < 1 || stick>4) return;
	if (hatDir < -1 || hatDir>1) return;

	hatDir++;

	int i = stick * 4 + hatDir; if (i < 0 || i>56) return;

	hatState[i] = 0;
}

//************************************************
//** hater sample with 0.05 second noise filter
//************************************************

int hatPressed(uc i)
{
	return 0;

	if (i > 16) return 0;

	int stick = i / 4;
	int hatDir = i - stick * 4;

	hatDir--;

	if (stick < 1 || stick>4) return 0;
	if (hatDir < -1 || hatDir>1) return 0;

	int dir = hatX[stick];

	int pressed = 0;

	if (hatDir == dir) pressed = 1;

	if (pressed)
		hatCount[i]++;
	else if (hatCount[i]) hatCount[i]--;

	if (hatCount[i] < (uc)5) pressed = 0;
	if (hatCount[i] > (uc)10) hatCount[i] = 10;

	return pressed;

} //hatPressed

void hatService()
{
	return;

	for (int i = 0; i < 17; i++)
	{
		int state = hatState[i];

		if (state)
		{
			int pressed = hatPressed(i);

			switch (state)
			{

			case 1: if (pressed) { hatState[i] = 2; hatBegAdr[i](); } break;
			case 2: if (!pressed)  hatState[i] = 3; hatMidAdr[i](); break;
			case 3:                hatState[i] = 1; hatEndAdr[i](); break;

			}

		}

	}
} //hatService




//aabbcc

#include "math.h"
#include <cstring>

//void *	memcpy (void *__s1, const void *__s2, size_t __n);
//void *	memset (void *__s, int __c, size_t __n);

#define REPEAT do {
#define UNTIL(Value)  } while (!(Value))

#define PROCEDURE void
#define sPROCEDURE static void
#define END   }
#define CASE  switch
#define BEGIN {
#define IF    if
#define THEN  {
#define FOR   for
#define CARDINAL int
#define PROCESS int
#define RETURN return
#define DO {
#define INTEGER int
#define iFUNCTION int
#define iPROCEDURE int
#define cFUNCTION int
#define cPROCEDURE int
#define csPROCEDURE static int
#define scPROCEDURE static int
#define rFUNCTION double
#define rPROCEDURE double
#define fPROCEDURE float
#define rsPROCEDURE static double
#define srPROCEDURE static double
#define LOOP for(;;) {
#define CHAR char
#define UCHAR unsigned char
#define UCHR unsigned char
#define SCHR signed  char
#define sCHAR static char
#define sUCHR static unsigned char
#define sSCHR static signed  char
#define USHR unsigned short
#define SSHR signed short
#define sUSHR static unsigned short
#define sSSHR static signed short

#define REAL double
#define GOTO goto
#define TRUNC int
#define FLOAT
#define Float
#define ORD
#define ADR
#define CONST const
#define NOT !
#define AND &&
#define OR  ||
#define XOR ^
#define WHILE while
#define ELSIF ; } else if

#define CHR
#define MOD %
#define DIV /
#define EXIT break
#define SINT  static int
#define SCARD static int
#define sCARD static int
#define SREAL static double
#define sREAL static double
#define SCRD static int
#define SREL static double

#define PTR static double *
#define PTC static int    *

#define CARD  int
#define UINT  unsigned int
#define sUINT static unsigned int
#define UCARD  unsigned int
#define sUCARD static unsigned int

// define BOOLEAN char
#define OF {
#define INC(Value) Value++
#define DEC(Value) Value--

#define LONGINT int
#define BYTE char
#define ELSE  ; } else {
#define PROC void

#define iPROC int
#define cPROC int
#define rPROC double


enum BOOLEAN { FALSE, TRUE };

#define CK(Value) if (!(Value)) return FALSE;
#define Ck(Value) if (!(Value)) return;

#define BOOL BOOLEAN
#define SBOOL static BOOLEAN
#define sBOOL static BOOLEAN
#define SBOL  static BOOLEAN
#define bFUNCTION BOOLEAN
#define bPROCEDURE BOOLEAN
#define bsPROCEDURE BOOLEAN static
#define sbPROCEDURE BOOLEAN static




extern float curHotRatio;
extern float minHotRatio;
extern float hotGoal;

extern float joyX2[10];
extern float joyY2[10];


//  extern float fastVideo   ;
extern float trackTimeOut;
extern float trackLockMin;
extern float trackCenterY;
extern float trackCenterX;
extern float trackState;
extern float gfMinLevel;
extern float bfMaxWide;
extern float bfMaxHigh;
extern float bfMaxRatio;
extern float trackX;
extern float trackY;
extern float trackZ;
extern float crossHairX;
extern float useLeftPot;
extern float wantedArmAngle;
extern float corssHair2X;
extern float crossHair2Y;

extern float sweepRate;
extern float ShooterPulseMaxRPM;
extern float ShooterPulseOffset;

extern float goalPixHigh;
extern float goalPixWide;

extern float wantedOffset;
extern float currentOffset;



extern float DistAdjH1;
extern float DistAdjH2;
extern float DistAdjY1;
extern float DistAdjY2;
extern float DistAdjDistH1;
extern float DistAdjDistH2;
extern float DistAdjDistY1;
extern float DistAdjDistY2;

extern float ShooterModeFlag;

extern float DistanceH;
extern float DistanceY;
extern float FilterHYFlag;
extern float FilterHYLimit;
extern float PickupRollerFlag;
extern float DebugRobotVideo;
extern float PickupRollerSpeed;
extern float rollerDeadZone;
extern float rollerCollectRate;
extern float shoesFlag;

extern float sSpeed[20];
extern float sHood[20];
extern float aCmd[20];

extern  float tCX1;
extern  float tCX2;
extern  float tCX3;
extern  float tCX4;
extern  float tCY1;
extern  float tCY2;
extern  float tCY3;
extern  float tCY4;
extern  float tH1;
extern  float tH2;
extern  float tH3;
extern  float tH4;
extern  float tW1;
extern  float tW2;
extern  float tW3;
extern  float tW4;

extern float targetOffsetX;
extern float targetOffsetY;


extern int trackRedTeam;
extern int trackBlueTeam;
extern int trackBalls;

extern  float GoalFound;
extern  float	GoalX;
extern  float	GoalY;
extern  float	GoalW;
extern  float	GoalH;

extern float autoDriveX;
extern float autoDriveY;
extern float autoDriveR;


//extern float goalWidth;

CARD quickFlag = 1;
CARD disableTrack = 0;

//scPROCEDURE abs(CARD val){IF (val<0) RETURN -val; RETURN val; }

sPROCEDURE swap(int &A, int &B) { int tmp = A; A = B; B = tmp; }

CARD testGoalDistance = 0;

CARD testGoalCount = 0;

extern int camRes;
extern int camWide;
extern int camHigh;
extern int camByteCount;

//--------- frc code -- do not change past this line without reflect ---------

//CHAR rawBuf[640*480*4+100];
//CHAR disBuf[640*480*4+100];

CHAR destBob[320 * 241 * 4];
CHAR holdBob[320 * 241 * 4];





PROCEDURE mFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD r, g, b;

FOR(CARD Y = 0; Y < high; Y++) DO

	FOR(CARD X = 0; X < wide; X++) DO

	//CARD zapMe=0;

	CARD pOff = Y*wide * 4 + X * 4;

r = (UCHAR)imgBuf[pOff];
g = (UCHAR)imgBuf[pOff + 1];
b = (UCHAR)imgBuf[pOff + 2];

CARD bw = (r + g + b) / 3;

//   CARD dr=r-bw;  IF (dr==0) dr=1;
CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

IF(db < 30) goto zapIt;

IF(dg > 0) goto zapIt;

IF(g > r) goto zapIt;
IF(g > b) goto zapIt;

IF(abs(10 * db / dg) > 13) goto zapIt;

//-------- new -------------

		 //	IF (db<70) goto zapIt;
IF(db < 50) goto zapIt;
//----------------------

continue;
zapIt:
imgBuf[pOff] = 0;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

END;

END; //mFilter

// Ball Filter

PROCEDURE bFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD r, g, b;

CARD wood = 0;
CARD yellow = 0;

FOR(CARD Y = 0; Y < high; Y++) DO

	FOR(CARD X = 0; X < wide; X++) DO

	//CARD zapMe=0;

	CARD pOff = Y*wide * 4 + X * 4;

r = (UCHAR)imgBuf[pOff];
g = (UCHAR)imgBuf[pOff + 1];
b = (UCHAR)imgBuf[pOff + 2];

CARD bw = (r + g + b) / 3;

CARD dr = r - bw;  IF(dr == 0) dr = 1;
CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

IF(abs(dr) < 20 AND abs(dg) < 20 AND abs(db) < 20) goto zapIt;

IF(dr < 0 AND dg>0 AND db > 0) goto zapIt;

IF(dr > 0 AND dg > 0 AND db < 0) goto zapIt;

yellow = 1;

if (dr < 20 OR dr>130) yellow = 0;
if (dg < -20 OR dg> 20) yellow = 0;
if (db<-40 OR db>-10) yellow = 0;

if (yellow) goto zapIt;


wood = 1;

if (dr < -35 OR dr>  0) wood = 0;
if (dg < -20 OR dg> 10) wood = 0;
if (db < 20 OR db> 45) wood = 0;

if (wood) goto zapIt;

continue;

/*

		   CARD limeGreen=1;

		   if (dr<-35 OR dr>-5 ) limeGreen=0;
		   if (dg< 15 OR dg> 60) limeGreen=0;
		   if (db<-35 OR db>-5 ) limeGreen=0;

		   if (limeGreen) continue;

		   CARD briteRed=1;

		   if (dr< 20 OR dr> 80)  briteRed=0;
		   if (dg<-80 OR dg>-20)  briteRed=0;
		   if (db<-20 OR db> 30)  briteRed=0;

		   if (briteRed) continue;

		   CARD briteRed2=1;

		   if (dr<70           )  briteRed2=0;
		   if (dg<-60 OR dg>-30)  briteRed2=0;
		   if (db<-80 OR db>-10)  briteRed2=0;

		   if (briteRed2) continue;

		   CARD blue=1;

		   if (dr<-55 OR dr>-25)  blue=0;
		   if (dg<-60 OR dg>-10)  blue=0;
		   if (db<45  OR db>100)  blue=0;

		   if (blue) continue;



		   CARD limeGreen=1;

		   if (dr<-45 OR dr>-15) limeGreen=0;
		   if (dg< 05 OR dg> 70) limeGreen=0;
		   if (db<-45 OR db> 5 ) limeGreen=0;

		   if (limeGreen) continue;

		   CARD briteRed=1;

		   if (dr< 10 OR dr> 90)  briteRed=0;
		   if (dg<-90 OR dg>-10)  briteRed=0;
		   if (db<-30 OR db> 40)  briteRed=0;

		   if (briteRed) continue;

		   CARD briteRed2=1;

		   if (dr<80           )  briteRed2=0;
		   if (dg<-70 OR dg>-20)  briteRed2=0;
		   if (db<-90 OR db>  0)  briteRed2=0;

		   if (briteRed2) continue;

		   CARD blue=1;

		   if (dr<-65 OR dr>-15)  blue=0;
		   if (dg<-70 OR dg>  0)  blue=0;
		   if (db<35  OR db>110)  blue=0;

		   if (blue) continue;

*/










goto zapIt;



continue;
zapIt:
imgBuf[pOff] = 0;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

END;

END; //bFilter















PROCEDURE cFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD r, g, b;

FOR(CARD Y = 0; Y < high; Y++) DO

	CARD pOff = Y*wide * 4 - 4;

FOR(CARD X = 0; X < wide; X++) DO

	pOff += 4;

r = (UCHAR)imgBuf[pOff];
g = (UCHAR)imgBuf[pOff + 1];
b = (UCHAR)imgBuf[pOff + 2];

CARD bw = (r + g + b) / 3;

CARD dr = r - bw;  IF(dr == 0) dr = 1;
CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

IF(dg < 30) goto zapIt;
IF(db > 0) goto zapIt;

IF(r > g) goto zapIt;
IF(b > g) goto zapIt;

IF(abs(dg / db) > 2) goto zapIt;

continue;
zapIt:
imgBuf[pOff] = 0;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

END;

END; //cFilter

CARD autoLevel = 20;

PROCEDURE blkFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD r, g, b;
/*
	CARD maxBW=0;

	FOR (CARD Y=0; Y<high; Y++) DO

		CARD pOff=Y*wide*4-4;

		FOR (CARD X=0; X<wide; X++) DO

			pOff+=4;

			r=(UCHAR)imgBuf[pOff  ];
			g=(UCHAR)imgBuf[pOff+1];
			b=(UCHAR)imgBuf[pOff+2];

			CARD bw=(r+g+b)/3;

			IF (bw>maxBW) maxBW=bw;

		END;

	END;
*/

FOR(CARD Y = 0; Y < high; Y++) DO

	CARD pOff = Y*wide * 4 - 4;

FOR(CARD X = 0; X < wide; X++) DO

	pOff += 4;

r = (UCHAR)imgBuf[pOff];
g = (UCHAR)imgBuf[pOff + 1];
b = (UCHAR)imgBuf[pOff + 2];

// CARD bw=(r+g+b)/3;

 //CARD limit=bw/3;

// CARD dr=r-bw;  IF (dr==0) dr=1;
// CARD dg=g-bw;  IF (dg==0) dg=1;
// CARD db=b-bw;  IF (db==0) db=1;

 //CARD minbw=bw-limit;
 //CARD maxbw=bw+limit;

 //IF (dr<-limit OR dr>limit) goto zapIt;
 //IF (dg<-limit OR dg>limit) goto zapIt;
 //IF (db<-limit OR db>limit) goto zapIt;

 //if (db>0) goto zapIt;

 //IF (!(r<50 AND b<50 AND g<50)) goto zapIt;

 //IF (r+g+b>100) goto zapIt;

 //if (dr<-10 OR dr>10) goto zapIt;
 //if (dg<-10 OR dg>10) goto zapIt;
 //if (db<-10 OR db>10) goto zapIt;

if (r > autoLevel OR b > autoLevel OR g > autoLevel) goto zapIt;

imgBuf[pOff] = 255;
imgBuf[pOff + 1] = 1;
imgBuf[pOff + 2] = 1;

continue;
zapIt:
imgBuf[pOff] = 0;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

END;

END; //blkFilter


PROCEDURE xxxgreenFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

//return ;

CARD pOff = 0; //Y*wide*4;

//FOR (CARD Y=0; Y<high; Y++) DO

	//FOR (CARD X=0; X<wide; X++) DO

int passes = wide*high;

FOR(CARD L = 0; L < passes; L++) DO

	CARD g1 = (UCHAR)imgBuf[pOff + 1];

if (g1 < gfMinLevel)
{
	imgBuf[pOff] = 0;
	imgBuf[pOff + 1] = 0;
	imgBuf[pOff + 2] = 0;

	pOff += 4;

	continue;

}

CARD r1 = (UCHAR)imgBuf[pOff];

CARD b1 = (UCHAR)imgBuf[pOff + 2];

IF(r1 > g1 || b1 > g1 || r1 + g1 + b1 > 600)
{
	imgBuf[pOff] = 0;
	imgBuf[pOff + 1] = 0;
	imgBuf[pOff + 2] = 0;
}



pOff += 4;

END;

//END;

END; //greenFilter

PROCEDURE greenFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD g, r, b, bw, dg, db, dr;

FOR(CARD Y = 0; Y < high; Y++) DO

	CARD pOff = Y*wide * 4;

FOR(CARD X = 0; X < wide; X++) DO


	g = (UCHAR)imgBuf[pOff + 1]; if (g < 100) goto reject;
b = (UCHAR)imgBuf[pOff];
r = (UCHAR)imgBuf[pOff + 2]; if (r > 100) goto reject;

bw = (r + g + b) / 3;

dg = g - bw;  if (dg < 11) goto reject;
dr = r - bw;
db = b - bw;


IF(dr > dg || db > dg)
{

reject:
	imgBuf[pOff] = 0;
	imgBuf[pOff + 1] = 0;
	imgBuf[pOff + 2] = 0;
}

//CARD r1=(UCHAR)imgBuf[pOff  ];
//CARD g1=(UCHAR)imgBuf[pOff+1];
//         CARD b1=(UCHAR)imgBuf[pOff+2];

		 //IF (r1*1.05>g1 || b1*1.05>g1 ) //|| r1+g1+b1>600)
   //      {
		 //  imgBuf[pOff  ]=0;
//           imgBuf[pOff+1]=0;
//           imgBuf[pOff+2]=0;
		 //}

		 //if (g1<gfMinLevel)
		 //{
		 //  imgBuf[pOff  ]=0;
//           imgBuf[pOff+1]=0;
//           imgBuf[pOff+2]=0;
		 //}

pOff += 4;

END;

END;

END; //greenFilter

PROCEDURE lightsFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD MaxCount = wide*high;
CARD count = 0;

CARD maxBlue = gfMinLevel / 1.5;
CARD minRed = gfMinLevel;
CARD minGreen = gfMinLevel;


FOR(CARD Y = 0; Y < high; Y++) DO

	CARD pOff = Y*wide * 4;

FOR(CARD X = 0; X < wide; X++) DO

	CARD b1 = (UCHAR)imgBuf[pOff];
CARD g1 = (UCHAR)imgBuf[pOff + 1];
CARD r1 = (UCHAR)imgBuf[pOff + 2];





IF(r1 < minRed || b1 < maxBlue || g1 < minGreen)
{
	//imgBuf[pOff  ]=0;
  //  imgBuf[pOff+1]=0;
   // imgBuf[pOff+2]=0;
}
else
{
	imgBuf[pOff] = 0;
	imgBuf[pOff + 1] = 255;
	imgBuf[pOff + 2] = 0;
	count++;

}


pOff += 4;

END;

END;

/*
		CARD tHigh=high/10;



	FOR (CARD Y=tHigh; Y<high-tHigh; Y++) DO

		CARD rowSize=wide*4;

		CARD pOff=Y*wide*4;

		FOR (CARD X=0; X<wide; X++) DO

			CARD r1=(UCHAR)imgBuf[pOff-rowSize];
			CARD r2=(UCHAR)imgBuf[pOff        ];
			CARD r3=(UCHAR)imgBuf[pOff+rowSize];

			IF ( (r1 && r2) || (r2 && r3 ) )
			{
			  imgBuf[pOff  ]=0;
			  imgBuf[pOff+1]=0;
			  imgBuf[pOff+2]=0;

			  count--;

			}

			pOff+=4;

		END;

	END;
*/
curHotRatio = count / (double)MaxCount;

if (curHotRatio > minHotRatio) hotGoal = 1; else hotGoal = 0;


END; // lightsFilter

PROCEDURE poFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD r, g, b;

FOR(CARD Y = 0; Y < high; Y++) DO

	CARD pOff = Y*wide * 4 - 4;

FOR(CARD X = 0; X < wide; X++) DO

	pOff += 4;

r = (UCHAR)imgBuf[pOff];
g = (UCHAR)imgBuf[pOff + 1];
b = (UCHAR)imgBuf[pOff + 2];

CARD bw = (r + g + b) / 3;

CARD dr = r - bw;  IF(dr == 0) dr = 1;
CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

//IF (dg<30) goto zapIt;
//IF (db>0 ) goto zapIt;

//IF (r>g) goto zapIt;
//IF (b>g) goto zapIt;

//IF ( abs(dg/db) >2) goto zapIt;

IF(bw < 100 || bw>200) goto zapIt;

IF(dr < -10 || dr>10) goto zapIt;
IF(dg < -10 || dg>10) goto zapIt;
IF(db < -10 || db>10) goto zapIt;




continue;
zapIt:
imgBuf[pOff] = 0;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

END;

END; //poFilter


//------------------------------------------------------------------------------
//sCARD scanWide=160;
//sCARD szMinY=0;
//sCARD szMaxY=120;
//sCHAR * imgDataPtr;
//sCHAR * scanSource;
//sCARD scanStride=160*4;
//sCARD scanMinY;
//sCARD scanMaxY;
//sCARD scanMinX;
//sCARD scanMaxX;
//sCARD zapCount;
#define zapMax 20000
sCARD zapX[zapMax + 1];
sCARD zapY[zapMax + 1];
//sCHAR *scanDest;
//sCARD SetupImage;
//sCARD imageDataMax;

sCARD bmFlag = 0;

sCARD bmIndex = 0;

sCARD xMaxBlobM[101];
sCARD yMaxBlobM[101];
sCARD xMinBlobM[101];
sCARD yMinBlobM[101];

sCARD bcIndex = 0;

sCARD xMaxBlobC[101];
sCARD yMaxBlobC[101];
sCARD xMinBlobC[101];
sCARD yMinBlobC[101];

sCARD xMaxMaxYBlobC[101];
sCARD xMinMaxYBlobC[101];

sCARD xMaxMaxXBlobC[101];
sCARD xMinMaxXBlobC[101];

sCARD blobPixCountC[101];

sCARD blobBotCount[101];
sCARD blobTopCount[101];


sCARD blobUsedC[101];
sCARD blobUsedM[101];

CARD goalCount = 0;
CARD goalCountt = 0;

CARD goalColor[100];
CARD goalX1[100];
CARD goalX2[100];
CARD goalY1[100];
CARD goalY2[100];
CARD goalCX[100];
CARD goalCY[100];
CARD goalWide[100];
CARD goalHigh[100];
CARD goalWideC[100];
CARD goalHighC[100];
CARD goalWideM[100];
CARD goalHighM[100];
CARD goalGap[100];
CARD goalDistH[100];
CARD goalDistY[100];

CARD goalX1yMax[100];
CARD goalX2yMax[100];
CARD goalX1xMax[100];
CARD goalX2xMax[100];



extern CARD totalMissCount;

//sCARD objectScanR(CARD x,CARD y);

unsigned char scanGrid[320][241];//[160][120];
unsigned char linkGrid[320][241];//[160][120];


//sCARD rCount=0;

//sCARD rTblX[1000];
//sCARD rTblY[1000];
//sCARD rCnt [1000];



CARD missBufEmpty = 1;

#define bOff  0
#define gOff  1
#define rOff  2

int camHigh = 240;
int camWide = 320;

PROCEDURE objectScan(CHAR* Dest, CHAR* Source, CARD yMin, CARD yMax, CARD fillColor)
BEGIN

CARD idx = -4;

unsigned char g, r, b;

int bw, dg, dr, db;

//CARD minGreen=gfMinLevel;if (minGreen<180) minGreen=180;


camHigh = 240;
camWide = 320;

memset(&scanGrid[0][0], 0, 77120);

if (DebugRobotVideo)
memset(&linkGrid[0][0], 0, 77120);

FOR(CARD Y = 0; Y < 240; Y++) DO

	FOR(CARD X = 0; X < 320; X++) DO

	idx += 4;

g = Source[idx + gOff]; if (g < 100)  continue;

r = Source[idx + rOff]; if (r > 100) continue;

b = Source[idx + bOff];

bw = (r + g + b) / 3;

dg = g - bw; IF(dg < 11)  continue;
dr = r - bw; IF(dr > dg)  continue;
db = b - bw;

IF(db < dg) scanGrid[X][Y] = 1;

END;

END;

FOR(CARD XX = 0; XX < 320; XX++) DO scanGrid[XX][0] = 0; scanGrid[XX][238] = 0; END;
FOR(CARD XX = 0; XX < 320; XX++) DO scanGrid[XX][1] = 0; scanGrid[XX][239] = 0; END;

FOR(CARD YY = 0; YY < 240; YY++) DO scanGrid[0][YY] = 0; scanGrid[318][YY] = 0; END;
FOR(CARD YY = 0; YY < 240; YY++) DO scanGrid[1][YY] = 0; scanGrid[319][YY] = 0; END;

CARD bobIndex = 0;
CARD overflow = 0;

FOR(CARD sY = 2; sY < 238; sY++) DO
	FOR(CARD sX = 2; sX < 318; sX++) DO

	IF(scanGrid[sX][sY]) THEN

	CARD X = sX;
CARD Y = sY;

CARD zIdx = 0;
CARD zScan = 0;

zapX[zIdx] = X;
zapY[zIdx] = Y;

again:

IF(scanGrid[X - 1][Y]) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y; scanGrid[X - 1][Y] = 0; }
IF(scanGrid[X + 1][Y]) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y; scanGrid[X + 1][Y] = 0; }
IF(scanGrid[X][Y - 1]) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y - 1; scanGrid[X][Y - 1] = 0; }
IF(scanGrid[X][Y + 1]) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y + 1; scanGrid[X][Y + 1] = 0; }

zScan++;

IF(zScan <= zIdx) THEN

X = zapX[zScan];
Y = zapY[zScan];

IF(zScan < 19900 && zIdx < 19900) goto again;

END;

IF(zIdx < 10 OR zIdx>6000 OR overflow) continue;

bobIndex++;

if (DebugRobotVideo)
FOR(CARD L = 0; L <= zIdx; L++) DO

linkGrid[zapX[L]][zapY[L]] = bobIndex;

END;

CARD xMax = 0;
CARD yMax = 0;
CARD xMin = 320;
CARD yMin = 240;



FOR(CARD L = 0; L < zIdx; L++) DO

	CARD x = zapX[L];
CARD y = zapY[L];

IF(x > xMax) xMax = x;
IF(y > yMax) yMax = y;
IF(x < xMin) xMin = x;
IF(y < yMin) yMin = y;

END;


//  xMinMaxY=99;
//   xMaxMaxY=49;


CARD midX = xMin + (xMax - xMin) / 2;
CARD midY = yMin + (yMax - yMin) / 2;

int bad = 0;

FOR(CARD L = 0; L < zIdx; L++) DO

	CARD x = midX - zapX[L]; if (x < -5 || x>5) continue;
CARD y = midY - zapY[L]; if (y < -5 || y>5) continue;

bad = 1;

break;

END;

if (bad) continue;

CARD wide = xMax - xMin + 1;
CARD high = yMax - yMin + 1;

IF(wide < 2 OR high < 3) continue;

IF(wide < 20 AND high < 30) continue;

REAL ratio = wide / (double)high;

IF(ratio<1.0 || ratio>3.0) continue;


// CARD xMaxMaxY=1;
//  CARD xMinMaxY=1;

//  CARD xMaxMaxX=0;
 // CARD xMinMaxX=320;
//
CARD yLevel[320];

for (int L = xMin; L <= xMax; L++) yLevel[L] = 0;

FOR(CARD L = 0; L < zIdx; L++) DO

	CARD x = zapX[L];
CARD y = zapY[L];

if (y > yLevel[x]) yLevel[x] = y;

END;

CARD hit = 0;
CARD begX = 0;
CARD begY = 0;
CARD endX = 0;
CARD endY = 0;

CARD bCnt = 0;

FOR(CARD L = xMin; L < midX; L++) DO

	CARD y = yLevel[L];

if (!hit && y) { hit = 1; begY = y; begX = L; }

if (!hit || !y) continue;

if (y > begY) { begY = y; begX = L; bCnt = 0; }
else { bCnt++; if (bCnt > 10) break; }

END;

hit = 0;
bCnt = 0;

FOR(CARD L = xMax; L > midX; L--) DO

CARD y = yLevel[L];

if (!hit && y) { hit = 1; endY = y; endX = L; }

if (!hit || !y) continue;

if (y > endY) { endY = y; endX = L; bCnt = 0; }
else { bCnt++; if (bCnt > 10) break; }

END;




xMaxBlobC[bcIndex] = xMax;
yMaxBlobC[bcIndex] = yMax;
xMinBlobC[bcIndex] = xMin;
yMinBlobC[bcIndex] = yMin;

xMaxMaxYBlobC[bcIndex] = endY;
xMinMaxYBlobC[bcIndex] = begY;

xMaxMaxXBlobC[bcIndex] = endX;
xMinMaxXBlobC[bcIndex] = begX;



blobPixCountC[bcIndex] = zIdx;

IF(bcIndex < 100) bcIndex++;

//edgemiss:

continue;

END;

END;
END;

quickFlag = 0;

IF(quickFlag) RETURN;

IF(!DebugRobotVideo) RETURN;

FOR(CARD Y = 0; Y < 240; Y++) DO

	FOR(CARD X = 0; X < 320; X++) DO

	CARD idx = Y * 320 * 4 + X * 4;

IF(linkGrid[X][Y]) THEN

Dest[idx + bOff] = 1;
Dest[idx + gOff] = 1; //if (Y>107) Dest[idx+gOff]=201;
Dest[idx + rOff] = 200;

ELSE

Dest[idx + bOff] = 0;
Dest[idx + gOff] = 0;
Dest[idx + rOff] = 0;

END;

END;

END;

END; //objectScan


PROCEDURE mFind(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

bmFlag = 1;

bmIndex = 0;

FOR(CARD l = 0; l < wide*high * 4; l++) holdBob[l] = imgBuf[l];

mFilter(imgBuf, wide, high);

objectScan(destBob, imgBuf, 0, high, 0);

FOR(CARD l = 0; l < wide*high * 4; l++) IF(destBob[l])imgBuf[l] = destBob[l]; else imgBuf[l] = holdBob[l];

END; //mFind


PROCEDURE cFind(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

bmFlag = 0;

bcIndex = 0;

FOR(CARD l = 0; l < wide*high * 4; l++) holdBob[l] = imgBuf[l];

cFilter(imgBuf, wide, high);

objectScan(destBob, imgBuf, 0, high, 0);

FOR(CARD l = 0; l < wide*high * 4; l++) IF(destBob[l])imgBuf[l] = destBob[l]; else imgBuf[l] = holdBob[l];

END; //cFind

PROCEDURE blkFind(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

bmFlag = 0;

bcIndex = 0;

FOR(CARD l = 0; l < wide*high * 4; l++) holdBob[l] = imgBuf[l];

blkFilter(imgBuf, wide, high);

objectScan(destBob, imgBuf, 0, high, 0);

FOR(CARD l = 0; l < wide*high * 4; l++)

	IF(destBob[l])imgBuf[l] = destBob[l]; else imgBuf[l] = holdBob[l];

END; //blkFind

PROCEDURE greenFind(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

bmFlag = 0;

bcIndex = 0;

// FOR (CARD l=0;l<wide*high*4;l++) holdBob[l]=imgBuf[l];

if (DebugRobotVideo)

memcpy(&holdBob[0], &imgBuf[0], wide*high * 4);

// greenFilter(imgBuf,wide,high);

objectScan(destBob, imgBuf, 0, high, 0);

if (DebugRobotVideo)

FOR(CARD l = 0; l < wide*high * 4; l++)

	IF(destBob[l])imgBuf[l] = destBob[l]; else imgBuf[l] = holdBob[l];

END; //greenFind

/*
PROCEDURE mcFindxx(CHAR *imgBuf, CARD wide,CARD high )
BEGIN

  bcIndex=0;
  bmIndex=0;

  IF (!quickFlag)
  FOR (CARD l=0;l<wide*high*4;l++) holdBob[l]=imgBuf[l];

  objectScan2(destBob, imgBuf,0,high,0 );

  IF (!quickFlag)
  FOR (CARD l=0;l<wide*high*4;l++) IF (destBob[l])imgBuf[l]=destBob[l]; else imgBuf[l]=holdBob[l];

END; //mFind
*/




















CHAR holdBob2[320 * 241 * 4];

PROCEDURE vLine(CHAR *buf, CARD wide, CARD high, CARD X, CARD r, CARD g, CARD b)
BEGIN

IF(X < 0 OR X >= wide) RETURN;

FOR(CARD Y = 0; Y < high; Y++) DO

	CARD pOff = Y*wide * 4 + X * 4;

buf[pOff++] = b;
buf[pOff++] = g;
buf[pOff] = r;

END;

END; //vLine

PROCEDURE vLine(CHAR *buf, CARD wide, CARD high, CARD X, CARD yMin, CARD yMax, CARD r, CARD g, CARD b)
BEGIN

IF(X < 0 OR X >= wide) RETURN;

IF(yMin < 0) yMin = 0;
IF(yMax >= high) yMax = high - 1;

FOR(CARD Y = yMin; Y <= yMax; Y++) DO

CARD pOff = Y*wide * 4 + X * 4;

buf[pOff++] = b;
buf[pOff++] = g;
buf[pOff] = r;

END;

END; //vLine
PROCEDURE hLine(CHAR *buf, CARD wide, CARD high, CARD Y, CARD r, CARD g, CARD b)
BEGIN

IF(Y < 0 OR Y >= high) RETURN;

FOR(CARD X = 0; X < wide; X++) DO

	CARD pOff = Y*wide * 4 + X * 4;

buf[pOff++] = b;
buf[pOff++] = g;
buf[pOff] = r;

END;

END; //hLine

PROCEDURE hLine(CHAR *buf, CARD wide, CARD high, CARD Y, CARD xMin, CARD xMax, CARD r, CARD g, CARD b)
BEGIN

IF(Y < 0 OR Y >= high) RETURN;

IF(xMin < 0) xMin = 0;
IF(xMax >= wide) xMax = wide - 1;

FOR(CARD X = xMin; X <= xMax; X++) DO

CARD pOff = Y*wide * 4 + X * 4;

buf[pOff++] = b;
buf[pOff++] = g;
buf[pOff] = r;

END;

END; //hLine


PROCEDURE Box(CHAR *buf, CARD wide, CARD high, CARD xMin, CARD yMin, CARD xMax, CARD yMax, CARD r, CARD g, CARD b)
BEGIN

return;

hLine(buf, wide, high, yMax, xMin, xMax, r, g, b);

hLine(buf, wide, high, yMin, xMin, xMax, r, g, b);

vLine(buf, wide, high, xMin, yMin, yMax, r, g, b);

vLine(buf, wide, high, xMax, yMin, yMax, r, g, b);

END; //Box


PROCEDURE Cross(CHAR *buf, CARD wide, CARD high, CARD xCen, CARD yCen, CARD xMin, CARD yMin, CARD xMax, CARD yMax, CARD r, CARD g, CARD b)
BEGIN

// return;

hLine(buf, wide, high, yCen, xMin, xMax, r, g, b);

//  hLine(buf,wide,high,yMin,xMin,xMax,r,g,b);

vLine(buf, wide, high, xCen, yMin, yMax, r, g, b);

//vLine(buf,wide,high,xMax,yMin,yMax,r,g,b);

END; //Cross

cPROCEDURE FindTarget(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD locX = 0;

FOR(CARD l = 0; l < wide*high * 4; l++) holdBob2[l] = imgBuf[l];

cFind(imgBuf, camWide, camHigh);

FOR(CARD l = 0; l < wide*high * 4; l++) imgBuf[l] = holdBob2[l];

mFind(imgBuf, camWide, camHigh);

FOR(CARD l = 0; l < wide*high * 4; l++) imgBuf[l] = holdBob2[l];

FOR(CARD L = 0; L < 100; L++) { blobUsedC[L] = 0; blobUsedM[L] = 0; }

FOR(CARD c = 0; c < bcIndex; c++) DO

	IF(blobUsedC[c]) continue;

CARD xMaxC = xMaxBlobC[c];
//    CARD yMaxC=yMaxBlobC[c];
CARD xMinC = xMinBlobC[c];
CARD yMinC = yMinBlobC[c];

FOR(CARD m = 0; m < bmIndex; m++) DO

	IF(blobUsedM[m]) continue;

CARD xMaxM = xMaxBlobM[m];
//CARD yMaxM=yMaxBlobM[c];
CARD xMinM = xMinBlobM[m];
CARD yMinM = yMinBlobM[m];

IF(xMaxC > xMinM && xMinC < xMaxM) THEN

	blobUsedM[m] = 1;
blobUsedC[c] = 0;

CARD X = xMinC + (xMaxC - xMinC) / 2;

IF(yMinC < yMinM) THEN //red

	vLine(imgBuf, wide, high, X, 0xff, 0, 0);

ELSE //blue

vLine(imgBuf, wide, high, X, 0, 0, 0xff);

END;

locX = X;

END;

END;

END;

RETURN locX;

END; //FindTarget


scPROCEDURE decodeColorTest(int r, int g, int b)
BEGIN

CARD bw = (r + g + b) / 3;

CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

IF(db < 60) goto zapIt;

IF(dg > 0) goto zapIt;

IF(g > r) goto zapIt;
IF(g > b) goto zapIt;

IF(abs(10 * db / dg) > 26) goto zapIt;

return 1;
zapIt:

IF(dg < 60) return 0;
IF(db > 0) return 0;

IF(r > g) return 0;
IF(b > g) return 0;

IF(abs(dg / db) > 4) return 0;

return 2;

END; //decodeColorTest


scPROCEDURE decodeColor(int r, int g, int b)
BEGIN

// return decodeColorTest(r,g,b);


CARD bw = (r + g + b) / 3;

CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

IF(db < 30) goto zapIt;

IF(dg > 0) goto zapIt;

IF(g > r) goto zapIt;
IF(g > b) goto zapIt;

IF(abs(10 * db / dg) > 13) goto zapIt;

// new -----------------

			//IF (db<70) goto zapIt;
IF(db < 50) goto zapIt;

//- new -------------------

return 1;
zapIt:

//return 0;

IF(dg < 30) return 0;
IF(db > 0) return 0;

IF(r > g) return 0;
IF(b > g) return 0;

IF(abs(dg / db) > 2) return 0;

return 2;

END; //decodeColor


scPROCEDURE decodeColorB(int r, int g, int b)
BEGIN

// return decodeColorTest(r,g,b);


CARD bw = (r + g + b) / 3;

CARD dr = r - bw;  IF(dr == 0) dr = 1;
CARD dg = g - bw;  IF(dg == 0) dg = 1;
CARD db = b - bw;  IF(db == 0) db = 1;

int k = 20;

IF(abs(dr) < k AND abs(dg) < k AND abs(db) < k) return 0;

IF(dr < 0 AND dg>0 AND db > 0) return 0;

IF(dr > 0 AND dg > 0 AND db < 0) return 0;

CARD yellow = 1;

if (dr < 20 OR dr>130) yellow = 0;
if (dg < -20 OR dg> 20) yellow = 0;
if (db<-40 OR db>-10) yellow = 0;

if (yellow) return 0;

//CARD wood=1;

if (dr < -35 OR dr>  0) return 1;
if (dg < -20 OR dg> 10) return 1;
if (db < 20 OR db> 45) return 1;

return 0;



END; //decodeColorB



PROCEDURE cmFilter(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

CARD r, g, b;

FOR(CARD Y = 0; Y < high; Y++) DO

	FOR(CARD X = 0; X < wide; X++) DO

	//CARD zapMe=0;

	CARD pOff = Y*wide * 4 + X * 4;

r = (UCHAR)imgBuf[pOff];
g = (UCHAR)imgBuf[pOff + 1];
b = (UCHAR)imgBuf[pOff + 2];

int color = decodeColor(r, g, b);

if (color == 0)  THEN

imgBuf[pOff] = 0;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

IF(color == 1) THEN

imgBuf[pOff] = (char)255;
imgBuf[pOff + 1] = 0;
imgBuf[pOff + 2] = 0;

END;

IF(color == 2) THEN

imgBuf[pOff] = 0;
imgBuf[pOff + 1] = (char)255;
imgBuf[pOff + 2] = 0;

END;

END;

END;

END; //cmFilter





PROCEDURE objectScan2(CHAR* Dest, CHAR* Source, CARD yMin, CARD yMax, CARD fillColor)
BEGIN



FOR(CARD Y = 0; Y < camHigh; Y++) DO

	FOR(CARD X = 0; X < camWide; X++) DO

	CARD idx = Y*camWide * 4 + X * 4;

int r = (UCHAR)Source[idx];
int g = (UCHAR)Source[idx + 1];
int b = (UCHAR)Source[idx + 2];

int color = decodeColor(r, g, b);

scanGrid[X][Y] = color;

linkGrid[X][Y] = 0;

END;

END;

CARD bobIndex = 0;



FOR(CARD sY = 1; sY < camHigh - 1; sY++) DO
	FOR(CARD sX = 1; sX < camWide - 1; sX++) DO

	int curColor = scanGrid[sX][sY];

IF(curColor) THEN

CARD X = sX;
CARD Y = sY;

CARD zIdx = 0;
CARD zScan = 0;

zapX[zIdx] = X;
zapY[zIdx] = Y;

again:

IF(scanGrid[X - 1][Y] == curColor) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y; scanGrid[X - 1][Y] = 0; }
IF(scanGrid[X + 1][Y] == curColor) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y; scanGrid[X + 1][Y] = 0; }
IF(scanGrid[X][Y - 1] == curColor) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y - 1; scanGrid[X][Y - 1] = 0; }
IF(scanGrid[X][Y + 1] == curColor) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y + 1; scanGrid[X][Y + 1] = 0; }
IF(scanGrid[X - 1][Y - 1] == curColor) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y - 1; scanGrid[X - 1][Y - 1] = 0; }
IF(scanGrid[X + 1][Y + 1] == curColor) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y + 1; scanGrid[X + 1][Y + 1] = 0; }
IF(scanGrid[X - 1][Y + 1] == curColor) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y + 1; scanGrid[X - 1][Y + 1] = 0; }
IF(scanGrid[X + 1][Y - 1] == curColor) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y - 1; scanGrid[X + 1][Y - 1] = 0; }

zScan++;

IF(zScan <= zIdx) THEN

X = zapX[zScan];
Y = zapY[zScan];

IF(zIdx < 9990) goto again;

END;

bobIndex++;

FOR(CARD L = 0; L <= zIdx; L++) DO

linkGrid[zapX[L]][zapY[L]] = bobIndex;

END;

CARD xMax = 0;
CARD yMax = 0;
CARD xMin = camWide;
CARD yMin = camHigh;

FOR(CARD L = 0; L < zIdx; L++) DO

	CARD x = zapX[L];
CARD y = zapY[L];

IF(x > xMax) xMax = x;
IF(y > yMax) yMax = y;
IF(x < xMin) xMin = x;
IF(y < yMin) yMin = y;

// UCHAR *dptr = (UCHAR*)Dest+scanStride*y+x*4;

 //  *(dptr+0)=fillBlu;
 //  *(dptr+1)=fillGrn;
 //  *(dptr+2)=fillRed;

END;

IF(curColor == 1) THEN

xMaxBlobM[bmIndex] = xMax;
yMaxBlobM[bmIndex] = yMax;
xMinBlobM[bmIndex] = xMin;
yMinBlobM[bmIndex] = yMin;

IF(bmIndex < 100) bmIndex++;

END;

IF(curColor == 2) THEN

xMaxBlobC[bcIndex] = xMax;
yMaxBlobC[bcIndex] = yMax;
xMinBlobC[bcIndex] = xMin;
yMinBlobC[bcIndex] = yMin;

IF(bcIndex < 100) bcIndex++;

END;

END;

END;
END;



IF(quickFlag) RETURN;

FOR(CARD Y = 0; Y < camHigh; Y++) DO

	FOR(CARD X = 0; X < camWide; X++) DO

	CARD idx = Y*camWide * 4 + X * 4;

IF(linkGrid[X][Y]) THEN

Dest[idx] = linkGrid[X][Y] * 10;
Dest[idx + 1] = linkGrid[X][Y] * 30;
Dest[idx + 2] = linkGrid[X][Y] * 50;

ELSE

Dest[idx] = 0;
Dest[idx + 1] = 0;
Dest[idx + 2] = 0;


END;

END;

END;

END; //objectScan2

PROCEDURE mcFind(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

bcIndex = 0;
bmIndex = 0;

IF(!quickFlag)
FOR(CARD l = 0; l < wide*high * 4; l++) holdBob[l] = imgBuf[l];

objectScan2(destBob, imgBuf, 0, high, 0);

IF(!quickFlag)
FOR(CARD l = 0; l < wide*high * 4; l++) IF(destBob[l])imgBuf[l] = destBob[l]; else imgBuf[l] = holdBob[l];

END; //mFind

PROCEDURE objectScan2b(CHAR* Dest, CHAR* Source, CARD yMin, CARD yMax, CARD fillColor)
BEGIN



FOR(CARD Y = 0; Y < camHigh; Y++) DO

	FOR(CARD X = 0; X < camWide; X++) DO

	CARD idx = Y*camWide * 4 + X * 4;

int r = (UCHAR)Source[idx];
int g = (UCHAR)Source[idx + 1];
int b = (UCHAR)Source[idx + 2];

int color = decodeColorB(r, g, b);

scanGrid[X][Y] = color;

linkGrid[X][Y] = 0;

END;

END;

CARD bobIndex = 0;



FOR(CARD sY = 1; sY < camHigh - 1; sY++) DO
	FOR(CARD sX = 1; sX < camWide - 1; sX++) DO

	int curColor = scanGrid[sX][sY];

IF(curColor) THEN

CARD X = sX;
CARD Y = sY;

CARD zIdx = 0;
CARD zScan = 0;

zapX[zIdx] = X;
zapY[zIdx] = Y;

again:

IF(scanGrid[X - 1][Y] == curColor) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y; scanGrid[X - 1][Y] = 0; }
IF(scanGrid[X + 1][Y] == curColor) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y; scanGrid[X + 1][Y] = 0; }
IF(scanGrid[X][Y - 1] == curColor) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y - 1; scanGrid[X][Y - 1] = 0; }
IF(scanGrid[X][Y + 1] == curColor) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y + 1; scanGrid[X][Y + 1] = 0; }
IF(scanGrid[X - 1][Y - 1] == curColor) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y - 1; scanGrid[X - 1][Y - 1] = 0; }
IF(scanGrid[X + 1][Y + 1] == curColor) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y + 1; scanGrid[X + 1][Y + 1] = 0; }
IF(scanGrid[X - 1][Y + 1] == curColor) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y + 1; scanGrid[X - 1][Y + 1] = 0; }
IF(scanGrid[X + 1][Y - 1] == curColor) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y - 1; scanGrid[X + 1][Y - 1] = 0; }

zScan++;

IF(zScan <= zIdx) THEN

X = zapX[zScan];
Y = zapY[zScan];

IF(zIdx < 9990) goto again;

END;

bobIndex++;

FOR(CARD L = 0; L <= zIdx; L++) DO

linkGrid[zapX[L]][zapY[L]] = bobIndex;

END;

CARD xMax = 0;
CARD yMax = 0;
CARD xMin = camWide;
CARD yMin = camHigh;

FOR(CARD L = 0; L < zIdx; L++) DO

	CARD x = zapX[L];
CARD y = zapY[L];

IF(x > xMax) xMax = x;
IF(y > yMax) yMax = y;
IF(x < xMin) xMin = x;
IF(y < yMin) yMin = y;

// UCHAR *dptr = (UCHAR*)Dest+scanStride*y+x*4;

 //  *(dptr+0)=fillBlu;
 //  *(dptr+1)=fillGrn;
 //  *(dptr+2)=fillRed;

END;

IF(curColor == 1) THEN

xMaxBlobM[bmIndex] = xMax;
yMaxBlobM[bmIndex] = yMax;
xMinBlobM[bmIndex] = xMin;
yMinBlobM[bmIndex] = yMin;

IF(bmIndex < 100) bmIndex++;

END;

IF(curColor == 2) THEN

xMaxBlobC[bcIndex] = xMax;
yMaxBlobC[bcIndex] = yMax;
xMinBlobC[bcIndex] = xMin;
yMinBlobC[bcIndex] = yMin;

IF(bcIndex < 100) bcIndex++;

END;

END;

END;
END;



IF(quickFlag) RETURN;

FOR(CARD Y = 0; Y < camHigh; Y++) DO

	FOR(CARD X = 0; X < camWide; X++) DO

	CARD idx = Y*camWide * 4 + X * 4;

IF(linkGrid[X][Y]) THEN

Dest[idx] = linkGrid[X][Y] * 10;
Dest[idx + 1] = linkGrid[X][Y] * 30;
Dest[idx + 2] = linkGrid[X][Y] * 50;

ELSE

Dest[idx] = 0;
Dest[idx + 1] = 0;
Dest[idx + 2] = 0;


END;

END;

END;

END; //objectScan2b



PROCEDURE bcFind(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

bcIndex = 0;
bmIndex = 0;

IF(!quickFlag)
FOR(CARD l = 0; l < wide*high * 4; l++) holdBob[l] = imgBuf[l];

objectScan2b(destBob, imgBuf, 0, high, 0);

IF(!quickFlag)
FOR(CARD l = 0; l < wide*high * 4; l++) IF(destBob[l])imgBuf[l] = destBob[l]; else imgBuf[l] = holdBob[l];

END; //bFind



CARD mTargetX = 0;
CARD mTargetY = 0;
CARD mTargetZ = 0;
CARD mTargetW = 0;
float mTargetA = 0;

int  totalLight = 0;

float targetTime = 0;




cPROCEDURE SumBritePixels(CHAR *imgBuf, CARD wide, CARD high)
BEGIN

int total = 0;

FOR(CARD l = 0; l < wide*high * 4; l++) if ((unsigned char)imgBuf[l] > 250) total++;

totalLight = total;

return total;

END; //


PROCEDURE swapGoal(int A, int B)
{
	swap(goalColor[A], goalColor[B]);
	swap(goalX1[A], goalX1[B]);
	swap(goalX2[A], goalX2[B]);
	swap(goalY1[A], goalY1[B]);
	swap(goalY2[A], goalY2[B]);
	swap(goalCX[A], goalCX[B]);
	swap(goalCY[A], goalCY[B]);
	swap(goalWide[A], goalWide[B]);
	swap(goalHigh[A], goalHigh[B]);
	swap(goalWideC[A], goalWideC[B]);
	swap(goalHighC[A], goalHighC[B]);
	swap(goalWideM[A], goalWideM[B]);
	swap(goalHighM[A], goalHighM[B]);
	swap(goalGap[A], goalGap[B]);
	swap(goalDistH[A], goalDistH[B]);
	swap(goalDistY[A], goalDistY[B]);
}

PROCEDURE deleteGoal(CARD idx)
BEGIN

IF(idx >= goalCount) RETURN;

IF(goalCount <= 0) RETURN;

goalCount--;

FOR(CARD A = idx; A < goalCount; A++) DO

	CARD B = A + 1;

goalColor[A] = goalColor[B];
goalX1[A] = goalX1[B];
goalX2[A] = goalX2[B];
goalY1[A] = goalY1[B];
goalY2[A] = goalY2[B];
goalCX[A] = goalCX[B];
goalCY[A] = goalCY[B];
goalWide[A] = goalWide[B];
goalHigh[A] = goalHigh[B];
goalWideC[A] = goalWideC[B];
goalHighC[A] = goalHighC[B];
goalWideM[A] = goalWideM[B];
goalHighM[A] = goalHighM[B];
goalGap[A] = goalGap[B];

goalDistH[A] = goalDistH[B];
goalDistY[A] = goalDistY[B];

END;

END;

//CARD totalMissCount=0;

int goalFrame = 0;

double GetDistUsingYCenter(double LocCY);
double GetDistanceUsingHigh(double High);

void sendBack(int index, float value);

int FindGreenTargets(char *imgBuf, int wide, int high)
BEGIN

double timeBeg = Timer::GetFPGATimestamp();

IF(disableTrack) RETURN 0;

CARD locX = 0;

// IF (!quickFlag)
//     memcpy(&holdBob2[0],&imgBuf[0],wide*high*4);

greenFind(imgBuf, wide, high);

//IF (!quickFlag)
//    memcpy(&imgBuf[0],&holdBob2[0],wide*high*4);

 //FOR (CARD L=0; L<100; L++)  blobUsedM[L]=0;

memset(&blobUsedM[0], 400, 0);

if (bcIndex > 99) bcIndex = 99;


goalCount = 0;

FOR(CARD c = 0; c < bcIndex; c++) DO

	CARD xMaxC = xMaxBlobC[c];
CARD yMaxC = yMaxBlobC[c];
CARD xMinC = xMinBlobC[c];
CARD yMinC = yMinBlobC[c];

CARD widthC = xMaxC - xMinC;

CARD highC = yMaxC - yMinC;

blobUsedC[c] = 1;

goalX1yMax[goalCount] = xMinMaxYBlobC[c];
goalX2yMax[goalCount] = xMaxMaxYBlobC[c];


goalX1xMax[goalCount] = xMinMaxXBlobC[c];
goalX2xMax[goalCount] = xMaxMaxXBlobC[c];


goalX1[goalCount] = xMinC;
goalX2[goalCount] = xMaxC;
goalY1[goalCount] = yMinC;
goalY2[goalCount] = yMaxC;
goalCX[goalCount] = xMinC + (xMaxC - xMinC) / 2;
goalCY[goalCount] = yMinC + (yMaxC - yMinC) / 2;
goalWide[goalCount] = xMaxC - xMinC;
goalHigh[goalCount] = yMaxC - yMinC;
goalWideC[goalCount] = xMaxC - xMinC;
goalHighC[goalCount] = yMaxC - yMinC;
goalWideM[goalCount] = xMaxC - xMinC;
goalHighM[goalCount] = yMaxC - yMinC;

goalDistY[goalCount] = GetDistUsingYCenter(goalCY[goalCount]);
goalDistH[goalCount] = GetDistanceUsingHigh(goalHigh[goalCount]);

IF(goalCount < 99) goalCount++;

END;

// Sort goal by CenterY Location

FOR(CARD M = 0; M < goalCount; M++) DO

	IF(goalY1[M] > goalY2[M])
	swap(goalY1[M], goalY2[M]);

END;

FOR(CARD M = 0; M < goalCount; M++) DO

	CARD iSwappedIt = 0;

FOR(CARD L = 0; L < goalCount - 1 - M; L++) DO

	IF(goalCY[L] > goalCY[L + 1]) THEN

	swapGoal(L, L + 1);

iSwappedIt = 1;

END;

END;

if (iSwappedIt == 0) break;

END;

// Find Largest Goal

CARD goalVolMax = goalWide[0] * goalHigh[0];

FOR(CARD L = 0; L < goalCount; L++) DO

	CARD goalVol = goalWide[L] * goalHigh[L];

IF(goalVol > goalVolMax) goalVolMax = goalVol;

END;

// filter all goal less than 0.5 of maxGoal

FOR(CARD L = 0; L < goalCount; L++) DO

	CARD goalVol = goalWide[L] * goalHigh[L];

IF(goalVol < goalVolMax*0.99) { deleteGoal(L); L--; }

END;

if (goalCount > 1) goalCount = 1;


/*
	  // filter all goal that are really out of place (distance miss match)
	  // remember only the top goal is checked

	  IF (FilterHYFlag && FilterHYLimit)

	  FOR (CARD L=0; L<goalCount; L++ ) DO

				REAL gDH=goalDistH[0];
				REAL gDY=goalDistY[0];

				REAL dif=(gDH-gDY); IF (dif<0) dif=-dif;

				REAL max=gDH; IF (gDY>gDH) max=gDY;

				REAL percent=dif/max*100;

				IF (percent>FilterHYLimit) THEN deleteGoal(L); continue; END;

				break;

	  END;

*/

tH1 = 0;
tH2 = 0;
tH3 = 0;
tH4 = 0;

//targetOffsetX = 0;
//targetOffsetY = 0;

IF(goalCount) THEN

GoalFound = 1;

GoalH = goalHigh[0];
GoalW = goalWide[0];
GoalX = goalCX[0];
GoalY = goalCY[0];

//xMinMaxYgoal = goalX1yMax[0];
//xMaxMaxYgoal = goalX2yMax[0];

//bottomWidth = goalX2xMax[0] - goalX1xMax[0];

//REAL dif = xMaxMaxYgoal - xMinMaxYgoal;

//goalAngle = atan(dif / bottomWidth)*57.2957795130 - goalAngleZero;

Cross(imgBuf, wide, high, goalCX[0], goalCY[0], goalX1[0], goalY1[0], goalX2[0], goalY2[0], 0xFF, 0, 0);

ELSE

GoalFound = 0;

GoalH = 0;
GoalW = 0;
GoalX = 160;
GoalY = 120;

END;












/*


FOR (CARD L=0; L<goalCount; L++) DO

  IF (L==0)
{

  tH1 =goalHigh[L];
  tW1 =goalWide[L];
  tCX1=goalCX  [L];
  tCY1=goalCY  [L];

  Cross(imgBuf,wide,high,goalCX[L],goalCY[L],goalX1[L],goalY1[L],goalX2[L],goalY2[L],0xFF,0,0);

  currentOffset=wantedOffset+(joyX2[2]+joyX2[1])*goalWide[L]/4;

  if (currentOffset<-0.01 || currentOffset>0.01)
	  {
	  vLine(imgBuf,wide,high,goalCX[L]-currentOffset,goalCY[L]-goalHigh [L]/4,goalCY[L]+goalHigh [L]/4,0xFF,0xFF,0);

	  goalCX[L]-=currentOffset;

	  targetOffsetX=currentOffset;

	  }

   float hoodOffset=(joyY2[2]+joyY2[1])*goalHigh[L];

   if (hoodOffset<-0.01 || hoodOffset>0.01)

	  hLine(imgBuf,wide,high,goalCY[L]+hoodOffset,goalCX[L]-goalWide [L]/4,goalCX[L]+goalWide [L]/4,0xFF,0xFF,0);

	  targetOffsetY=hoodOffset;

  }

else
{
	Cross(imgBuf,wide,high,goalCX[L],goalCY[L],goalX1[L],goalY1[L],goalX2[L],goalY2[L],0,0xFF,0);

	if (L==1)
	{
	  tH2 =goalHigh[L];
	  tW2 =goalWide[L];
	  tCX2=goalCX  [L];
	  tCY2=goalCY  [L];
	}

	if (L==2)
	{
	  tH3 =goalHigh[L];
	  tW3 =goalWide[L];
	  tCX3=goalCX  [L];
	  tCY3=goalCY  [L];
	}
	if (L==3)
	{
	  tH4 =goalHigh[L];
	  tW4 =goalWide[L];
	  tCX4=goalCX  [L];
	  tCY4=goalCY  [L];
	}

}


  //DisplayValue(L+21,10,"Wide ####",goalWide[L],15,4,4);
  //DisplayValue(L+21,15,"High ####",goalHigh[L],15,4,4);
  //DisplayValue(L+21,20,"YMin ####",goalY1[L]  ,15,4,4);
 // DisplayValue(L+21,25,"YMax ####",goalY2[L]  ,15,4,4);

END;
*/
//-----------------------

//finished:

int minDist = 0;

// FOR (CARD L=0; L<goalCount; L++) DO

  // IF (goalDist[L]<goalDist[minDist]) minDist=L;

// END;

goalCountt = goalCount;

IF(goalCount) THEN

DistanceH = goalDistH[0];
DistanceY = goalDistY[0];

goalPixHigh = goalHigh[0];
goalPixWide = goalWide[0];


trackX = locX = mTargetX = goalCX[minDist];

trackY = mTargetY = goalCY[minDist];
trackZ = mTargetZ = goalDistY[minDist];

//const double Rad  = 57.2957795130;

mTargetA = 0; //atan((trackX-crossHairX)/camWide)*Rad;

mTargetW = goalWide[minDist];

//float center=(float)abs(mTargetX-camWide/2);

testGoalDistance = goalDistY[minDist];

testGoalCount++;

ELSE

testGoalDistance = 0;

END;

mTargetA = testGoalDistance; sendBack(160, mTargetA);

double timeEnd = Timer::GetFPGATimestamp();

targetTime = timeEnd - timeBeg;

goalFrame++;

RETURN goalCount;

END; //FindGreenTargets

/*

451	9	5	79	84	81.5
416	11	8	77	85	81
379	12	9	74	83	78.5
348	13	10	71	81	76
312	15	11	67	78	72.5
291	16	12	64	76	70
263	17	13	60	73	66.5
248	19	14	57	71	64
229	20	15	53	68	60.5
213	21	16	49	65	57
202	23	17	46	63	54.5
189	24	18	42	60	51
177	26	19	38	57	47.5
167	27	20	34	55	44.5
160	28	21	31	52	41.5
151	30	22	27	49	38
141	31	23	22	45	33.5
135	33	24	19	43	31
131	33	25	16	41	28.5
122	36	26	11	3	24
114	37	27	5	32	18.5







	81.5	451
	81		416
	78.5	379
	76		348
	72.5	312
	70		291
	66.5	263
	64		248
	60.5	229
	57		213
	54.5	202
	51		189
	47.5	177
	44.5	167
	41.5	160
	38		151
	33.5	141
	31		135
	28.5	131
	24		122
	18.5	114












*/


/*
   Y-Center , Distance

	81.5	451
	81		416
	78.5	379
	76		348
	72.5	312
	70		291
	66.5	263
	64		248
	60.5	229
	57		213
	54.5	202
	51		189
	47.5	177
	44.5	167
	41.5	160
	38		151
	33.5	141
	31		135
	28.5	131
	24		122
	18.5	114

*/



double adjustYCenter(double yc);

double GetDistUsingYCenter(double x_in)
{
	// Hartmann4_model(double x_in)

	x_in = adjustYCenter(x_in);

	// coefficients
	double A = 1.4502431137532008E+00;
	double B = 1.0883253075457273E+04;
	double C = 1.0686941929865468E+02;
	double D = 8.2748298560097237E+02;
	double E = -5.7342874289440047E+01;

	double dist = A + B / (C - x_in) + D / (E - x_in);

	if (dist < 120) dist = 120;
	if (dist > 576) dist = 576;

	return dist;

}
/*
   High , Distance

	5	451
	8	416
	9	379
	10	348
	11	312
	12	291
	13	263
	14	248
	15	229
	16	213
	17	202
	18	189
	19	177
	20	167
	21	160
	22	151
	23	141
	24	135
	25	131
	26	122
	27	114

*/

double adjustYCenter(double yc)
{
	if (DistAdjY1 == DistAdjY2) return yc + DistAdjY1;

	if (DistAdjDistY1 == DistAdjDistY2) RETURN yc;

	REAL dAdjust = DistAdjY2 - DistAdjY1;
	REAL dDist = DistAdjDistY2 - DistAdjDistY1;
	REAL dslope = dAdjust / dDist;

	REAL dist1 = yc - DistAdjDistY1;
	REAL adj = dist1*dslope;

	REAL newyc = yc + adj + DistAdjY1;

	RETURN newyc;
}

double adjustHCenter(double high)
{
	if (DistAdjH1 == DistAdjH2) return high + DistAdjH1;

	if (DistAdjDistH1 == DistAdjDistH2) RETURN high;

	REAL dAdjust = DistAdjH2 - DistAdjH1;
	REAL dDist = DistAdjDistH2 - DistAdjDistH1;
	REAL dslope = dAdjust / dDist;

	REAL dist1 = high - DistAdjDistH1;
	REAL adj = dist1*dslope;

	REAL newyc = high + adj + DistAdjH1;

	RETURN newyc;
}



double GetDistanceUsingHigh(double x_in)
{
	x_in = adjustHCenter(x_in);

	// Hartmann1_model(double x_in)

	// coefficients
	double A = -4.9807182761425935E+01;
	double B = -4.8736804838046692E+03;
	double C = -2.4030711198555132E+00;
	double dist = A + B / (C - x_in);

	if (dist < 120) dist = 120;
	if (dist > 576) dist = 576;

	return dist;


}













