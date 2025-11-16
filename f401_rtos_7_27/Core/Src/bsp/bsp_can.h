#ifndef __BSP_CAN__
#define __BSP_CAN__
#include "main.h"

/*CAN发送或是接收的ID*/
typedef enum
{

	CAN_2006Moto_ALL_ID = 0x200,
	CAN_2006Moto1_ID = 0x201,
	CAN_2006Moto2_ID = 0x202,
	CAN_2006Moto3_ID = 0x203,
	CAN_2006Moto4_ID = 0x204,
	
}CAN_Message_ID;
/*电机反馈参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
    float  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint8_t			buf_idx;
	uint16_t			angle_buf[4];
	uint16_t			fited_angle;
	uint32_t			msg_cnt;
}moto_measure_t;

void get_moto_chassis_measure(moto_measure_t *ptr);
void get_moto_arm_measure(moto_measure_t *ptr);
void CAN_Filter_Config(void);
extern moto_measure_t  moto_chassis[4];
extern moto_measure_t moto_arm[3];
void set_moto_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void set_moto_arm_current(int16_t iq1, int16_t iq2, int16_t iq3);

#endif
