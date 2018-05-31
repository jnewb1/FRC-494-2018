#ifndef __Driving
#define __Driving

#include "base.h"

extern int drivingOff;

void drivingService(void);

void setupDriving(void);

void setupDriving(void);

void stopCube(unsigned char i);
void stopCube2();
void stopCube5();

void stopDrive();
void startDropCube();

void startShootCube();
void startLoadCube();
void startZero();

void dropCube();
void dropCubeScale();
void startDropCubeSlow2();
void slowDrive();
void dropCubeScaleFar();
void dropCubeScaleHigh();
void pickupCube();
void pickupCubeFar();
void pickupCubeFast();
float Get_ToolbarError();
void startPickup();
void startShootCube(unsigned char i);

void stopLift();
void startScaleMid();
void startToolbarScaleMid();
void startToolbarZero();
void startLiftScaleHigh();
void startLiftScaleMid();
void startLiftScaleLow();
void startLiftPickup();
void startLiftSwitch();
void startToolbarPickup();
void MoveToolBarUp();
void MoveToolBarDown();
void StopToolBar();
void stopCube4();
void dropCubeFast();
void startScaleBehind();


#endif
