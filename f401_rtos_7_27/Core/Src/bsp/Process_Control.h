#ifndef PROCESS_CONTROL_H
#define PROCESS_CONTROL_H
#include "map.h"
#include "Arm_Control.h"

void Process_Generation(const uint8_t *sequence);
void Get_process(uint8_t number,uint8_t i);
void Put_process(uint8_t number,uint8_t i);
uint8_t put_plate_num(uint8_t layer);
uint8_t get_plate_num(uint8_t layer);
//typedef struct UART{
//    position pos_calibration;//校准后位置
//    uint8_t type;//0为不校准，1为货架校准，2为纸剁校准
//}UART;
typedef struct control_block{
    position pos_target;
    arm_control arm_target;
    uint8_t steering_engine;
    uint8_t uart;
    uint16_t delay;
}control_block;
extern uint8_t control_flow_size;
extern control_block control_flow[];
#endif //PROCESS_CONTROL_H
