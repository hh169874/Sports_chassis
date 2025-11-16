#include "Chassis_Control.h"
#include "pid.h"

#include "map.h"
#include "MPU9250.h"
#include "Gyro.h"


float MaxSpeedM=500.0f;// 0.5m/s
const float SQRT2_OVER_2 = 0.7071067811865475f;
float Target_v1,Target_v2,Target_v3,Target_v4;
float last_motor_dis_0=0,last_motor_dis_1=0,last_motor_dis_2=0,last_motor_dis_3=0;
//float Vx=0,Vy=0,Vz=0;
/**
 * @brief 麦轮运动逆解
 * @param Vx_world 世界x轴速度
 * @param Vy_world 世界y轴速度
 * @param Vz_world 世界z轴速度
 */
void Mecanum_Reverse_Calculation(float Vx_world, float Vy_world, float Vz_world)
{
    //坐标轴转换使用标准角度0，1.5pi，pi，0.5pi
    float err = 15.0f*PI/180.0f;
    float fix_yaw=0;
    if (cur_pos.yaw<err&&cur_pos.yaw>-err)
        fix_yaw=0;
    else if(cur_pos.yaw<(0.5*PI+err)&&cur_pos.yaw>(0.5*PI-err))
        fix_yaw=0.5f*PI;
    else if(cur_pos.yaw<(PI+err)&&cur_pos.yaw>(PI-err))
        fix_yaw=PI;
    else if(cur_pos.yaw<(1.5*PI+err)&&cur_pos.yaw>(1.5*PI-err))
        fix_yaw=1.5f*PI;
    float cos= arm_cos_f32(fix_yaw);
    float sin= arm_sin_f32(fix_yaw);
    //坐标轴转换
    float Vx = Vx_world * cos + Vy_world * sin;
    float Vy = -Vx_world * sin + Vy_world * cos;

    // 进行逆向解算
    Target_v1= Vx - Vy - Vz_world * (Car_H / 2 + Car_W / 2);
    Target_v2= Vx + Vy - Vz_world * (Car_H / 2 + Car_W / 2);
    Target_v3= Vx - Vy + Vz_world * (Car_H / 2 + Car_W / 2);
    Target_v4= Vx + Vy + Vz_world * (Car_H / 2 + Car_W / 2);
    Mecanum_set_target();
}
/**
 * @brief 麦轮运动逆解,不进行坐标变换
 * @param Vx 小车x轴速度
 * @param Vy 小车y轴速度
 * @param Vz 小车z轴速度
 */
void Mecanum_Reverse_Calculation_unchanged(float Vx, float Vy, float Vz)
{
    Target_v1= Vx - Vy - Vz * (Car_H / 2 + Car_W / 2);
    Target_v2= Vx + Vy - Vz * (Car_H / 2 + Car_W / 2);
    Target_v3= Vx - Vy + Vz * (Car_H / 2 + Car_W / 2);
    Target_v4= Vx + Vy + Vz * (Car_H / 2 + Car_W / 2);
    Mecanum_set_target();
}
/**
 * @brief 全向轮运动逆解
 * @param Vx_world 世界x轴速度
 * @param Vy_world 世界y轴速度
 * @param Vz_world 世界z轴速度
 */
void Omni_Reverse_Calculation(float Vx_world, float Vy_world, float Vz_world)
{
    //坐标轴转换使用标准角度0，1.5pi，pi，0.5pi
    float err = 15.0f*PI/180.0f;
    float fix_yaw=0;
    if (cur_pos.yaw<err&&cur_pos.yaw>-err)
        fix_yaw=0;
    else if(cur_pos.yaw<(0.5*PI+err)&&cur_pos.yaw>(0.5*PI-err))
        fix_yaw=0.5f*PI;
    else if(cur_pos.yaw<(PI+err)&&cur_pos.yaw>(PI-err))
        fix_yaw=PI;
    else if(cur_pos.yaw<(1.5*PI+err)&&cur_pos.yaw>(1.5*PI-err))
        fix_yaw=1.5f*PI;
    float cos= arm_cos_f32(fix_yaw);
    float sin= arm_sin_f32(fix_yaw);
    //坐标轴转换
    float Vx = Vx_world * cos + Vy_world * sin;
    float Vy = -Vx_world * sin + Vy_world * cos;

    // 进行逆向解算
    Target_v1 = SQRT2_OVER_2 * (Vx - Vy) - Vz_world * Car_R;
    Target_v2 = SQRT2_OVER_2 * (Vx + Vy) - Vz_world * Car_R;
    Target_v3 = SQRT2_OVER_2 * (-Vx + Vy) - Vz_world * Car_R;
    Target_v4 = SQRT2_OVER_2 * (-Vx - Vy) - Vz_world * Car_R;
    Omni_set_target();
}
/**
 * @brief 全向轮运动逆解,不进行坐标变换
 * @param Vx 小车x轴速度
 * @param Vy 小车y轴速度
 * @param Vz 小车z轴速度
 */
void Omni_Reverse_Calculation_unchanged(float Vx, float Vy, float Vz)
{
    // 进行逆向解算
    Target_v1 = SQRT2_OVER_2 * (Vx - Vy) - Vz * Car_R;
    Target_v2 = SQRT2_OVER_2 * (Vx + Vy) - Vz * Car_R;
    Target_v3 = SQRT2_OVER_2 * (-Vx + Vy) - Vz * Car_R;
    Target_v4 = SQRT2_OVER_2 * (-Vx - Vy) - Vz * Car_R;
    Omni_set_target();
}
/**
 * @brief 麦轮转换，设置电机目标转速，线速度->角速度->rps->rpm->减速电机
 */
void Mecanum_set_target(void)
{
	motor_pid[0].target= Target_v1 / Mecanum_Wheel_radius / (2 * PI) * 60 * 36;
	motor_pid[1].target= Target_v2 / Mecanum_Wheel_radius / (2 * PI) * 60 * 36;
	motor_pid[2].target= -Target_v3 / Mecanum_Wheel_radius / (2 * PI) * 60 * 36;
	motor_pid[3].target= -Target_v4 / Mecanum_Wheel_radius / (2 * PI) * 60 * 36;
}
/**
 * @brief 全向轮转换，设置电机目标转速，线速度->角速度->rps->rpm->减速电机
 */
void Omni_set_target(void)
{
    motor_pid[0].target= Target_v1 / Omni_Wheel_radius / (2 * PI) * 60 * 36;
    motor_pid[1].target= Target_v2 / Omni_Wheel_radius / (2 * PI) * 60 * 36;
    motor_pid[2].target= Target_v3 / Omni_Wheel_radius / (2 * PI) * 60 * 36;
    motor_pid[3].target= Target_v4 / Omni_Wheel_radius / (2 * PI) * 60 * 36;
}
/**
 * @brief 全向轮正解，根据电机距离计算小车距离
 * @param moto_measure
 */
void Omni_Forward_Calculation(moto_measure_t *moto_measure)
{
    // 1. 计算当前电机位移（单位：mm）
    static float last_motor_dis_0=0,last_motor_dis_1=0,last_motor_dis_2=0,last_motor_dis_3=0;
    float current_motor_dis_0=(moto_measure->total_angle/8192.0f)/36.0f*Omni_Wheel_round;
    float current_motor_dis_1=((moto_measure+1)->total_angle/8192.0f)/36.0f*Omni_Wheel_round;
    float current_motor_dis_2=((moto_measure+2)->total_angle/8192.0f)/36.0f*Omni_Wheel_round;
    float current_motor_dis_3=((moto_measure+3)->total_angle/8192.0f)/36.0f*Omni_Wheel_round;
    // 2. 计算电机位移增量（本次 - 上次）
    float delta_motor_dis_0 = current_motor_dis_0 - last_motor_dis_0;
    float delta_motor_dis_1 = current_motor_dis_1 - last_motor_dis_1;
    float delta_motor_dis_2 = current_motor_dis_2 - last_motor_dis_2;
    float delta_motor_dis_3 = current_motor_dis_3 - last_motor_dis_3;
    // 3. 更新上一次的电机位移（用于下次计算）
    last_motor_dis_0 = current_motor_dis_0;
    last_motor_dis_1 = current_motor_dis_1;
    last_motor_dis_2 = current_motor_dis_2;
    last_motor_dis_3 = current_motor_dis_3;
    // 4. 计算小车坐标系的位移增量 (delta_dis_x, delta_dis_y) 和角度增量 (delta_yaw)
    float delta_dis_x = (delta_motor_dis_0 + delta_motor_dis_1 - delta_motor_dis_2 - delta_motor_dis_3) * 0.5f * SQRT2_OVER_2;
    float delta_dis_y = (-delta_motor_dis_0 + delta_motor_dis_1 + delta_motor_dis_2 - delta_motor_dis_3) * 0.5f * SQRT2_OVER_2;
    float delta_yaw = -(delta_motor_dis_0 + delta_motor_dis_1 + delta_motor_dis_2 + delta_motor_dis_3) / (4.0f * Car_R);

    //获取当前角度
    float cur_x,cur_y,yaw;
    get_cur_pos(&cur_x,&cur_y,&yaw);
    yaw+=delta_yaw;

//    //坐标轴转换使用标准角度0，1.5pi，pi，0.5pi
    float err = 15.0f*PI/180.0f;
    float fix_yaw=0;
    if (yaw<err&&yaw>-err)
        fix_yaw=0;
    else if(yaw<(0.5*PI+err)&&yaw>(0.5*PI-err))
        fix_yaw=0.5f*PI;
    else if(yaw<(PI+err)&&yaw>(PI-err))
        fix_yaw=PI;
    else if(yaw<(1.5*PI+err)&&yaw>(1.5*PI-err))
        fix_yaw=1.5f*PI;

    //小车坐标系转地图坐标系
    float cos= arm_cos_f32(fix_yaw);
    float sin= arm_sin_f32(fix_yaw);
    cur_x+=delta_dis_x* cos- delta_dis_y*sin;
    cur_y+=delta_dis_x* sin+ delta_dis_y*cos;

    set_cur_pos(cur_x,cur_y,yaw);

//    float pitch,roll,yaw2;
//    uint8_t err = mpu_mpl_get_data(&pitch,&roll,&yaw2);
//    if(err == 0)
//    {
//        printf("data: %f,%f,%f \r\n", roll,pitch,yaw2);
//    }
//    if(dis_x>=1000)
//    {
//        Omni_Reverse_Calculation(0,0,0);
//        Omni_set_target();
//    }
//    printf("%f\t%f\t%f\n",dis_x,dis_y,yaw);
}
/**
 * @brief 麦轮正解，根据电机距离计算小车距离
 * @param moto_measure
 */
void Mecanum_Forward_Calculation(moto_measure_t *moto_measure)
{
    // 1. 计算当前电机位移（单位：mm）
    float current_motor_dis_0=(moto_measure->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    float current_motor_dis_1=((moto_measure+1)->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    float current_motor_dis_2=((moto_measure+2)->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    float current_motor_dis_3=((moto_measure+3)->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    // 2. 计算电机位移增量（本次 - 上次）
//    float delta_motor_dis_0 = current_motor_dis_0 - last_motor_dis_0;
    float delta_motor_dis_1 = current_motor_dis_1 - last_motor_dis_1;
    float delta_motor_dis_2 = -(current_motor_dis_2 - last_motor_dis_2);
    float delta_motor_dis_3 = -(current_motor_dis_3 - last_motor_dis_3);
    // 3. 更新上一次的电机位移（用于下次计算）
    last_motor_dis_0 = current_motor_dis_0;
    last_motor_dis_1 = current_motor_dis_1;
    last_motor_dis_2 = current_motor_dis_2;
    last_motor_dis_3 = current_motor_dis_3;
    // 4. 计算小车坐标系的位移增量 (delta_dis_x, delta_dis_y) 和角度增量 (delta_yaw)
    float delta_dis_x = (delta_motor_dis_1+delta_motor_dis_2) * 0.5f;
    float delta_dis_y = (delta_motor_dis_3-delta_motor_dis_2) * 0.5f;
    float delta_yaw =(delta_motor_dis_3-delta_motor_dis_1)/(Car_H+Car_W) ;

    //获取当前角度
    float cur_x,cur_y,yaw;
    get_cur_pos(&cur_x,&cur_y,&yaw);
    yaw+=delta_yaw;

//    //坐标轴转换使用标准角度0，1.5pi，pi，0.5pi
    float err = 15.0f*PI/180.0f;
    float fix_yaw=0;
    if (yaw<err&&yaw>-err)
        fix_yaw=0;
    else if(yaw<(0.5*PI+err)&&yaw>(0.5*PI-err))
        fix_yaw=0.5f*PI;
    else if(yaw<(PI+err)&&yaw>(PI-err))
        fix_yaw=PI;
    else if(yaw<(1.5*PI+err)&&yaw>(1.5*PI-err))
        fix_yaw=1.5f*PI;

    //小车坐标系转地图坐标系
    float cos= arm_cos_f32(fix_yaw);
    float sin= arm_sin_f32(fix_yaw);
    cur_x+=delta_dis_x* cos- delta_dis_y*sin;
    cur_y+=delta_dis_x* sin+ delta_dis_y*cos;
//使用陀螺仪或步长计算
#ifdef GYRO_ENABLE
    extern int32_t value;
    yaw=(float)value*0.001f*PI/180.0f;
#endif

    if(yaw>=2*PI)
        yaw-=2*PI;
    if(yaw<=-2*PI)
        yaw+=2*PI;
    set_cur_pos(cur_x,cur_y,yaw);

//    printf("%f\n",yaw);
//    float pitch,roll,yaw2;
//    uint8_t err = mpu_mpl_get_data(&pitch,&roll,&yaw2);
//    if(err == 0)
//    {
//        printf("data: %f,%f,%f \r\n", roll,pitch,yaw2);
//    }
//    if(dis_x>=1000)
//    {
//        Omni_Reverse_Calculation(0,0,0);
//        Omni_set_target();
//    }
//    printf("%f\t%f\t%f\n",dis_x,dis_y,yaw);
}
/**
 * @brief 麦轮正解补丁，消除暂停任务时存储的上次电机位移last_motor_dis_0，使其为当前位移
 */
void Mecanum_Forward_Calculation_patch(moto_measure_t *moto_measure)
{
    last_motor_dis_0=(moto_measure->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    last_motor_dis_1=((moto_measure+1)->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    last_motor_dis_2=((moto_measure+2)->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
    last_motor_dis_3=((moto_measure+3)->total_angle/8192.0f)/36.0f*Mecanum_Wheel_round;
}
/**
 * @brief PS2控制函数
 */
void motor(void)
{
    taskENTER_CRITICAL(); // 关闭中断
	PS2AllValueUpdate();
    taskEXIT_CRITICAL();  // 恢复中断
	float Vx=MaxSpeedM*(128.0f-(float)RockerValue[3])/128.0f;
	float Vy=MaxSpeedM*(127.0f-(float)RockerValue[2])/127.0f;
    float Vz=MaxSpeedM*(127.0f-(float)RockerValue[0])/127.0f;
    printf("%d\t%d\t%d\n",RockerValue[3],RockerValue[2],RockerValue[0]);
 //   Mecanum_Reverse_Calculation(Vx, Vy, Vz);
    Omni_Reverse_Calculation(Vx, Vy, Vz);
	if(PS2RedLight()==0)//无指令状态
	{
        Target_v1= 0;
        Target_v2= 0;
        Target_v3= 0;
        Target_v4= 0;
	}
    Omni_set_target();
//    printf("%f %f %f\n", Target_v1, Target_v2, Target_v3);
}
