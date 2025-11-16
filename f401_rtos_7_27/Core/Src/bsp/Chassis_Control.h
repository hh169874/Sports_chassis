#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "main.h"
#include "ps2.h"
#include "bsp_can.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

#include "can.h"

#define Car_H 296.0f//H-> 轴距，机器人前后麦轮的距离，单位：mm。
#define Car_W 212.0f//W-> 轮距，机器人左右麦轮的距离，单位：mm。
#define Car_R 157.5f//L-> 机器人各个全向轮到底盘中心的距离，单位：mm。
#define Omni_Wheel_radius 33.0f//全向轮半径，单位：mm。
#define Omni_Wheel_round 2*PI*Omni_Wheel_radius//全向轮周长，单位mm。
#define Mecanum_Wheel_radius 39.5f//麦轮半径，单位：mm。
#define Mecanum_Wheel_round 2*PI*Mecanum_Wheel_radius//麦轮周长，单位mm。

void motor(void);
void Mecanum_Reverse_Calculation(float Vx, float Vy, float Vz);
void Mecanum_Reverse_Calculation_unchanged(float Vx, float Vy, float Vz);
void Mecanum_set_target(void);
void Mecanum_Forward_Calculation(moto_measure_t *moto_measure);
void Mecanum_Forward_Calculation_patch(moto_measure_t *moto_measure);
void Omni_Reverse_Calculation(float Vx, float Vy, float Vz);
void Omni_Reverse_Calculation_unchanged(float Vx, float Vy, float Vz);
void Omni_set_target(void);
void Omni_Forward_Calculation(moto_measure_t *moto_measure);
extern uint8_t RockerValue[4];
extern uint8_t ButtonValue[16];
extern float Target_v1,Target_v2,Target_v3,Target_v4;
#endif


/**
* 1--4
 *
 *2--3
*/