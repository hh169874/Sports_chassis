#ifndef __PID__
#define __PID__
#include "main.h"
typedef struct _PID_TypeDef
{
	float target;							//目标值
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	
	float pout;
	float iout;
	float dout;
	
	float output;						//本次输出
	float last_output;			//上次输出
	
	float MaxOutput;				//输出限幅
	float IntegralLimit;		//积分限幅
	float DeadBand;			  //死区（绝对值）
//	float ControlPeriod;		//控制周期
    uint8_t arrive;           //到达死区
	float  Max_Err;					//最大误差

}PID_TypeDef;

	
void pid_param_init(
    PID_TypeDef * pid,
    float maxout,
    float intergral_limit,
    float deadband,
    float  max_err,
    float  target,

    float 	kp,
    float 	ki,
    float 	kd);
				   
float pid_calculate_speed(PID_TypeDef* pid, float measure);
float pid_calculate_position(PID_TypeDef* pid, float measure);
float pid_calculate_position_z(PID_TypeDef* pid, float measure);
extern PID_TypeDef motor_pid[4],arm_pid[3];
extern PID_TypeDef distance_x_pid,distance_y_pid,distance_z_pid;
extern PID_TypeDef calibration_x_pid,calibration_y_pid,calibration_z_pid;
#endif
