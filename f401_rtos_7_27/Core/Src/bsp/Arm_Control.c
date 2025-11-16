#include "Arm_Control.h"
#include "pid.h"
struct plate cur_plate={
        .cur_angle=0,
        .cur_number=0
};
/**
/**
 * @brief 机械臂高度设置,单位mm
 * @param height
 */
void set_arm_height(uint16_t height)
{
    if(height>490)
        height=490;
    arm_pid[1].target=-(float)height/12.6f/PI*8192*36;
}
/**
 * @brief 机械臂伸缩长度设置,单位mm
 * @param length
 */
void set_arm_length(int16_t length)
{
    if(length>145)
        length=145;
    if(length<-120)
        length=-120;
    arm_pid[2].target=-(float)length/12.6f/PI*8192*36;
}
/**
 * @brief 设置当前车头的置物盘
 * @param number
 * 0：无置物盘
 * 1：车体右侧置物盘
 * 2：车体后方置物盘
 * 3：车体左侧置物盘
 */
void set_arm_angle(uint8_t number)
{
    // 货物在货盘上的固定偏移（度），顺时针方向
    static int16_t offsets[]={ 0,-90,-180,-270 };
    // 计算目标绝对角度（使得该货物位于0°）
    int16_t target_angle=(int16_t)(-offsets[number]);
    target_angle= (int16_t)(target_angle % 360);
    if (target_angle < 0) target_angle += 360; 
    // 计算从当前角度到目标角度的最小转动
    int16_t delta = (int16_t)(target_angle - cur_plate.cur_angle);

    // 调整到 [-180, 180] 范围，确保最小转动
    if (delta > 180) delta = (int16_t)(delta - 360);
    else if (delta < -180) delta = (int16_t)(delta + 360);

    // 最终的目标角度 = 当前角度 + 最小转动
    int16_t final_target = (int16_t)(cur_plate.cur_angle + delta);
    final_target=(int16_t)(final_target % 360);

    cur_plate.cur_angle=final_target;
    cur_plate.cur_number=number;
    arm_pid[0].target=(float)final_target/360*8*8192*36;
}
