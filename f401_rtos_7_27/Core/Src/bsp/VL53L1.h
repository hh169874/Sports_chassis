#ifndef VL53L1_H
#define VL53L1_H
#include "main.h"

#ifdef VL53L1_ENABLE
#define add_default 0x52
#define add1 0x40
#define add2 0x42
#define add3 0x44
#define add4 0x46
#define add5 0x48

extern uint16_t Distance[5];
uint8_t single_init(uint16_t dev);
uint8_t multi_init(void);
void get_distance(void);
#endif
#endif //VL53L1_H
