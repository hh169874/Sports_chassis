#include "map.h"
position cur_pos={0,0,0},tar_pos={0,0,0};
/**
 * @brief 设置小车当前实际坐标
 * @param x
 * @param y
 * @param yaw
 */
void set_cur_pos(float x,float y,float yaw)
{
    cur_pos.x=x;
    cur_pos.y=y;
    cur_pos.yaw=yaw;
}
/**
 * @brief 获取小车当前实际坐标
 * @param x
 * @param y
 * @param yaw
 */
void get_cur_pos(float *x,float *y,float *yaw)
{
    *x=cur_pos.x;
    *y=cur_pos.y;
    *yaw=cur_pos.yaw;
}
/**
 * @brief 设置小车目标实际坐标
 * @param x
 * @param y
 * @param yaw
 */
void set_tar_pos(float x,float y,float yaw)
{
    tar_pos.x=x;
    tar_pos.y=y;
    tar_pos.yaw=yaw;
}

