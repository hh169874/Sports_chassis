#ifndef __PS2_H__
#define __PS2_H__

#include "main.h"

#define PSS_Rx 0 
#define PSS_Ry 1
#define PSS_Lx 2
#define PSS_Ly 3

#define PSB_Left        0
#define PSB_Down        1
#define PSB_Right       2
#define PSB_Up          3
#define PSB_Start       4
#define PSB_RightRocker	5
#define PSB_LeftRocker  6
#define PSB_Select      7
#define PSB_Square      8
#define PSB_Cross       9
#define PSB_Circle      10
#define PSB_Triangle    11
#define PSB_X           8
#define PSB_A			9
#define PSB_B			10
#define PSB_Y			11
#define PSB_R1          12
#define PSB_L1          13
#define PSB_R2          14
#define PSB_L2          15


void PS2OriginalValueGet(void);
void Delay_us(uint32_t udelay);
void PS2AllValueUpdate(void);
void RockerValueGet(void);
void PS2OriginalValueClear(void);
void ButtonValueGet(void);
int PS2RedLight(void);
extern uint8_t RockerValue[4];
#endif
