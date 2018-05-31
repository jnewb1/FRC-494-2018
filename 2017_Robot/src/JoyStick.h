#ifndef __joyStick
#define __joyStick

#include "base.h"

 extern  float inAton;
 extern  float inTele;
 extern  float inDisable;

 extern int redTeam ;
 extern int blueTeam;

 extern int station;
 extern float battery;


void onTrig( int stick,int trig, void (*begAdr)(void),void (*midAdr)(void),void (*endAdr)(void));

void trigOff( int stick,int trig );

void trigService(void);


void onKey( int key, void (*begAdr)(void),void (*midAdr)(void),void (*endAdr)(void));

void keyOff( int key );

void keyService(void);


void onHatX( int stick,int hatDir, void (*begAdr)(void),void (*midAdr)(void),void (*endAdr)(void));

void hatOff( int stick,int hatDir );

void hatService();

int SoftStick(int input);

#endif
