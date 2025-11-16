#ifndef __HC_SSr04__
#define __HC_SSr04__
#include "main.h"

#ifdef HC_SR04_ENABLE
#define voice_speed 0.3471f/2.0f
extern float distance[];
void Get_Dis(void);
#endif

#endif
