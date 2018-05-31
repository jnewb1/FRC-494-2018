#ifndef __Aton
#define __Aton

#include "base.h"



extern float distScale;
extern float steerScale;
extern float suckSpeed;
extern float spitSpeed;

extern float shiftServoBeg;
extern float shiftServoEnd;

extern float stopDropServoBeg;
extern float stopDropServoEnd;
extern float reverseSkid;
extern float disableGyro;

void atonService(void);

extern int atonSelect;
extern float atonNum;
extern int stationNum;
extern int atonMode;
extern int blockAton;

void cornerScale_6_1(unsigned char i);
void startShootCube();
void aton1_2(unsigned char i);
void aton5_5_2(unsigned char i);
void crossScale_4_2(unsigned char i);
void cornerScale_4_3(unsigned char i);
void crossScale_4_3(unsigned char i);
void aton1_5(unsigned char i);
void aton1_6(unsigned char i);
void aton1_7(unsigned char i);
void aton1_8(unsigned char i);
void startDropCubeSlow();
void aton1_9(unsigned char i);
void aton1_10(unsigned char i);
void aton1_11(unsigned char i);
void aton1_7(unsigned char i);
void aton2_4(unsigned char i);
void aton2_4_2(unsigned char);
void aton2_5(unsigned char i);
void aton2_6(unsigned char i);
void aton2_2(unsigned char i);
void aton3_2(unsigned char i);
void aton4_2(unsigned char i);
void aton4_5(unsigned char i);
void aton5_2(unsigned char i);
void aton6_2(unsigned char i);
void aton1_3(unsigned char i);
void aton2_3(unsigned char i);
void aton3_3(unsigned char i);
void aton4_3(unsigned char i);
void aton5_3(unsigned char i);
void aton3_6(unsigned char i);
void aton6_3(unsigned char i);
void aton1_4(unsigned char i);
void aton2_4(unsigned char i);
void aton3_4(unsigned char i);
void aton3_4_2(unsigned char i);
void aton4_4_2(unsigned char i);
void aton5_4_2(unsigned char i);
void aton3_5(unsigned char i);
void aton4_4(unsigned char i);
void aton5_4(unsigned char i);
void aton6_4(unsigned char i);
void aton5_5(unsigned char i);
void aton3_7(unsigned char i);
void aton3_8(unsigned char i);
void aton3_9(unsigned char i);
void aton3_10(unsigned char i);
void aton3_11(unsigned char i);
void aton4_6(unsigned char i);
void aton4_6_1(unsigned char i);
void aton4_7(unsigned char i);
void aton8_2(unsigned char i);
void aton8_3(unsigned char i);
void aton8_4(unsigned char i);
void aton8_5(unsigned char i);
void aton8_6(unsigned char i);
void aton8_7(unsigned char i);
void aton8_8(unsigned char i);
void aton8_9(unsigned char i);
void aton9_2(unsigned char i);
void aton9_3(unsigned char i);
void aton9_4(unsigned char i);
void aton9_4_2(unsigned char i);
void aton9_4_3(unsigned char i);
void startDropCube(unsigned char i);
void aton8_4_2(unsigned char i);
void aton9_5(unsigned char i);
void aton9_6(unsigned char i);
void aton9_7(unsigned char i);
void aton9_8(unsigned char i);
void aton9_9(unsigned char i);
void aton5_7(unsigned char i);
void aton5_6(unsigned char i);
void aton5_6_1(unsigned char i);
void aton9_6_2(unsigned char i);

void cornerScale_2(unsigned char i);
void cornerScale_3(unsigned char i);
void cornerScale_4(unsigned char i);
void cornerScale_4_2(unsigned char i);
void cornerScale_5(unsigned char i);
void cornerScale_6(unsigned char i);
void cornerScale_6_2(unsigned char i);
void cornerScale_6_3(unsigned char i);
void cornerScale_7(unsigned char i);
void cornerScale_8(unsigned char i);

void sideScale_2(unsigned char i);
void sideScale_3(unsigned char i);
void sideScale_4(unsigned char i);
void sideScale_5(unsigned char i);
void sideScale_6(unsigned char i);
void sideScale_7(unsigned char i);
void sideScale_8(unsigned char i);

void backScale_2(unsigned char i);
void backScale_3(unsigned char i);
void backScale_4(unsigned char i);
void backScale_4_2(unsigned char i);
void backScale_5(unsigned char i);
void backScale_6(unsigned char i);
void backScale_7(unsigned char i);
void backScale_8(unsigned char i);
void backScale_9(unsigned char i);

void pickupCubeStrafe_3(unsigned char i);
void pickupCubeStrafe_2(unsigned char i);
void pickupCubeStrafe_4(unsigned char i);
void pickupCubeStrafe_5(unsigned char i);
void pickupCubeStrafe_2_2(unsigned char i);



void crossScale_2(unsigned char i);
void crossScale_3(unsigned char i);
void crossScale_4(unsigned char i);
void crossScale_5(unsigned char i);



#define kbMoveRapid  (int)cVal[9]*1000
#define kbTurnRapid  (int)cVal[10]*1000
#define c1c2DistL1L1 (int)cVal[11]
#define c1c2DistL1L2 (int)cVal[12]
#define c1c2DistL1L3 (int)cVal[13]
#define c1c2DistL2L1 (int)cVal[14]
#define c1c2DistL2L2 (int)cVal[15]
#define c1c2DistL2L3 (int)cVal[16]
#define c1c2DistL3L1 (int)cVal[17]
#define c1c2DistL3L2 (int)cVal[18]
#define c1c2DistL3L3 (int)cVal[19]

#define storeU        (int)cVal[20]*10
#define storeW        (int)cVal[21]*10

#define pickupU       (int)cVal[22]*10
#define pickupW       (int)cVal[23]*10

#define carryU        (int)cVal[24]*10
#define carryW        (int)cVal[25]*10

#define ramU          (int)cVal[26]*10
#define ramW          (int)cVal[27]*10

#define hurdleU       (int)cVal[28]*10
#define hurdleW       (int)cVal[29]*10

//void atonSpinRobot(float angle,void(*finishAdr)());
//void atonRotateAndShoot(float angle,void(*finishAdr)());

#endif
