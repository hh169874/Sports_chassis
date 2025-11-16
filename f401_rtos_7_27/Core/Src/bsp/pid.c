#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#define ABS(x)		((x>0)? x: -x)
//电机速度pid
PID_TypeDef motor_pid[4];
//机械臂角度pid 0：转盘 1：升降 3：前后
PID_TypeDef arm_pid[3];
//电机速度pid
PID_TypeDef distance_x_pid,distance_y_pid,distance_z_pid;
//校准pid
PID_TypeDef calibration_x_pid,calibration_y_pid,calibration_z_pid;
//控制流事件组
extern EventGroupHandle_t arrive_event_group;
void pid_param_init(
    PID_TypeDef * pid,
    float maxout,
    float intergral_limit,
    float deadband,
    float  max_err,
    float  target,

    float 	kp,
    float 	ki,
    float 	kd)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->Max_Err = max_err;
    pid->target = target;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->arrive=0;
    pid->output = 0;
}
/**
 * @brief 速度环pid计算
 * @param pid 计算的pid结构体
 * @param measure 测量值
 * @return
 */
float pid_calculate_speed(PID_TypeDef* pid, float measure)//, int16_t target)
{
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;

    //开始pid计算
    pid->pout = pid->kp * pid->err;
    pid->iout += (pid->ki * pid->err);


    pid->dout =  pid->kd * (pid->err - pid->last_err);

    //积分是否超出限制
    if(pid->iout > pid->IntegralLimit)
        pid->iout = pid->IntegralLimit;
    if(pid->iout < - pid->IntegralLimit)
        pid->iout = - pid->IntegralLimit;

    //pid输出和
    pid->output = pid->pout + pid->iout + pid->dout;


    //pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
    if(pid->output>pid->MaxOutput)
    {
        pid->output = pid->MaxOutput;
    }
    if(pid->output < -(pid->MaxOutput))
    {
        pid->output = -(pid->MaxOutput);
    }
    //是否进入死区
    if(fabsf(pid->output)<pid->DeadBand)
    {
        pid->output=0;
    }
	return pid->output;
}

/**
 * @brief 位置环pid
 * @param pid
 * @param measure
 * @return
 */
float pid_calculate_position(PID_TypeDef* pid, float measure)//, int16_t target)
{
    pid->measure = measure;
    //	pid->target = target;

    pid->last_err  = pid->err;
    pid->last_output = pid->output;

    pid->err = pid->target - pid->measure;
    //是否进入死区
    if((ABS(pid->err) > pid->DeadBand))
    {
        pid->pout = pid->kp * pid->err;
        pid->iout += (pid->ki * pid->err);


        pid->dout =  pid->kd * (pid->err - pid->last_err);

        //积分是否超出限制
        if(pid->iout > pid->IntegralLimit)
            pid->iout = pid->IntegralLimit;
        if(pid->iout < - pid->IntegralLimit)
            pid->iout = - pid->IntegralLimit;

        //pid输出和
        pid->output = pid->pout + pid->iout + pid->dout;


        //pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
        if(pid->output>pid->MaxOutput)
        {
            pid->output = pid->MaxOutput;
        }
        if(pid->output < -(pid->MaxOutput))
        {
            pid->output = -(pid->MaxOutput);
        }
        //xEventGroupSetBits(arrive_event_group,0x01);
    }
    else
    {
        // 进入死区时，强制输出为0并重置积分
        pid->output = 0;
        pid->iout = 0;  // 清除积分累积
    }

    return pid->output;
}

/**
 * @brief 位置环pid_旋转专用
 * @param pid
 * @param measure
 * @return
 */
float pid_calculate_position_z(PID_TypeDef* pid, float measure)//, int16_t target)
{
    pid->measure = measure;
    //	pid->target = target;

    pid->last_err  = pid->err;
    pid->last_output = pid->output;

    pid->err = pid->target - pid->measure;
    if(pid->err>PI)
    {
        pid->err-=2*PI;
    }
    if(pid->err<-PI)
    {
        pid->err+=2*PI;
    }
    //是否进入死区
    if((ABS(pid->err) > pid->DeadBand))
    {
        pid->pout = pid->kp * pid->err;
        pid->iout += (pid->ki * pid->err);


        pid->dout =  pid->kd * (pid->err - pid->last_err);

        //积分是否超出限制
        if(pid->iout > pid->IntegralLimit)
            pid->iout = pid->IntegralLimit;
        if(pid->iout < - pid->IntegralLimit)
            pid->iout = - pid->IntegralLimit;

        //pid输出和
        pid->output = pid->pout + pid->iout + pid->dout;


        //pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
        if(pid->output>pid->MaxOutput)
        {
            pid->output = pid->MaxOutput;
        }
        if(pid->output < -(pid->MaxOutput))
        {
            pid->output = -(pid->MaxOutput);
        }
        //xEventGroupSetBits(arrive_event_group,0x01);
    }
    else
    {
        // 进入死区时，强制输出为0并重置积分
        pid->output = 0;
        pid->iout = 0;  // 清除积分累积
    }

    return pid->output;
}
