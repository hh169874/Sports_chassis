#ifndef MAP_H
#define MAP_H
#include "main.h"
typedef struct point{
    float x;
    float y;
    float yaw;
}position;
void set_cur_pos(float x,float y,float yaw);
void set_tar_pos(float x,float y,float yaw);
void get_cur_pos(float *x,float *y,float *yaw);
extern position cur_pos,tar_pos;
#endif
