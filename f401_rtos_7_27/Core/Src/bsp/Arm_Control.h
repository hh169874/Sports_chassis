#ifndef F401_05_3_ARM_CONTROL_H
#define F401_05_3_ARM_CONTROL_H

#include "main.h"
typedef struct arm_control{
    uint16_t height;
    uint16_t length;
    uint8_t state;
}arm_control;
//当前货盘位置记录
struct plate{
    int16_t cur_angle;
    uint8_t cur_number;
};
void set_arm_height(uint16_t height);
void set_arm_angle(uint8_t number);
void set_arm_length(int16_t length);
#endif //F401_05_3_ARM_CONTROL_H
