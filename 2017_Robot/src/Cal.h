
#ifndef __Cal
#define __Cal

#include "base.h"

#define countsPerFootRight 1812
#define countsPerFootLeft  1812
#define countsPerInchRight  151
#define countsPerInchLeft   151

#define AxisR 0
#define AxisL 1
#define AxisU 2
#define AxisW 3
#define AxisD 4
#define AxisUU 6

#define AxiaGainRp  7
#define AxiaGainLp  7
#define AxiaGainUp  150
#define AxiaGainWp  7
#define AxiaGainDp  7

#define AxiaGainRi  0
#define AxiaGainLi  0
#define AxiaGainUi  0
#define AxiaGainWi  0
#define AxiaGainDi  0

#define AxiaGainRd  0
#define AxiaGainLd  0
#define AxiaGainUd  0
#define AxiaGainWd  0
#define AxiaGainDd  0

#define AxiaGainRf  0
#define AxiaGainLf  0
#define AxiaGainUf  0
#define AxiaGainWf  0
#define AxiaGainDf  0

li feetR(int numFeet);
li feetL(int numFeet);
li inchR(int numInch);
li inchL(int numInch);

int fps( int feet );

#endif
