#include "bsp_can.h"
#include "can.h"
moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t moto_arm[3] = {0};
CAN_RxHeaderTypeDef RxHeader1,RxHeader2;
CAN_TxHeaderTypeDef TxHeader1,TxHeader2;
uint8_t RxData1[8],RxData2[8],TxData1[8],TxData2[8];
uint32_t TxMailbox=0;
/**
 * @brief CAN1过滤器
 */
void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef can_filter;

    /*------------------------ CAN1 过滤器配置 ------------------------*/
    can_filter.FilterBank = 0;                       // CAN1 使用 Filter Bank 0~13
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码模式
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;  // 32 位模式
    can_filter.FilterIdHigh = 0x0000;            // ID 高 16 位（标准帧左移 5）
    can_filter.FilterIdLow = 0x0000;                 // ID 低 16 位（未使用）
    can_filter.FilterMaskIdHigh = 0x0000;        // 高 16 位掩码（全匹配）
    can_filter.FilterMaskIdLow = 0x0000;             // 低 16 位掩码（未使用）
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // 消息存入 FIFO0
    can_filter.FilterActivation = ENABLE;            // 启用过滤器
    can_filter.SlaveStartFilterBank = 14;            // CAN2 从 Filter Bank 14 开始

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);

    /*------------------------ CAN2 过滤器配置 ------------------------*/
    can_filter.FilterBank = 14;                      // CAN2 使用 Filter Bank 14~27
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码模式
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;  // 32 位模式
    can_filter.FilterIdHigh = 0x0000;            // ID 高 16 位（标准帧左移 5）
    can_filter.FilterIdLow = 0x0000;                 // ID 低 16 位（未使用）
    can_filter.FilterMaskIdHigh = 0x0000;        // 高 16 位掩码（全匹配）
    can_filter.FilterMaskIdLow = 0x0000;             // 低 16 位掩码（未使用）
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // 消息存入 FIFO0
    can_filter.FilterActivation = ENABLE;            // 启用过滤器

    HAL_CAN_ConfigFilter(&hcan2, &can_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
}
/**
 * @brief CAN接收中断回调
 * @param hcan
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//ignore can1 or can2.
    if(hcan==&hcan1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1);

        uint8_t i = 0;
        i = RxHeader1.StdId - CAN_2006Moto1_ID;
        get_moto_chassis_measure(&moto_chassis[i]);
    }

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);

    if(hcan==&hcan2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader2, RxData2);

        uint8_t i = 0;
        i = RxHeader2.StdId - CAN_2006Moto1_ID;
        get_moto_arm_measure(&moto_arm[i]);
    }
}
/**
 * @brief 获取底盘电机转速角度信息
 * @param ptr
 */
void get_moto_chassis_measure(moto_measure_t *ptr)
{
    static char first_get_offset=1;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxData1[0] << 8 | RxData1[1]) ;
    if(first_get_offset)
    {
        ptr->offset_angle=ptr->angle;
        ptr->last_angle = ptr->angle;
        first_get_offset=0;
    }
	ptr->speed_rpm  = (int16_t)(RxData1[2] << 8 | RxData1[3]);
//	ptr->real_current = (RxData1[4]<<8 | RxData1[5])*5.f/16384.f;//转矩

//	ptr->hall = RxData1[6];
	
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
//	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
//total_angle计算
    int res1, res2, delta;
    if(ptr->angle < ptr->last_angle){			//可能的情况
        res1 = ptr->angle + 8192 - ptr->last_angle;	//正转，delta=+
        res2 = ptr->angle - ptr->last_angle;				//反转	delta=-
    }else{	//angle > last
        res1 = ptr->angle - 8192 - ptr->last_angle ;//反转	delta -
        res2 = ptr->angle - ptr->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(ABS(res1)<ABS(res2))
        delta = res1;
    else
        delta = res2;
    ptr->total_angle += delta;
}
/**
 * @brief 获取机械臂电机转速角度信息
 * @param ptr
 */
void get_moto_arm_measure(moto_measure_t *ptr)
{
    static char first_get_offset=1;
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(RxData2[0] << 8 | RxData2[1]) ;
    if(first_get_offset)
    {
        ptr->offset_angle=ptr->angle;
        ptr->last_angle = ptr->angle;
        first_get_offset=0;
    }
    ptr->speed_rpm  = (int16_t)(RxData2[2] << 8 | RxData2[3]);
//	ptr->real_current = (RxData2[4]<<8 | RxData2[5])*5.f/16384.f;//转矩

//	ptr->hall = RxData2[6];


    if(ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt --;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt ++;
//	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
//total_angle计算
    int res1, res2, delta;
    if(ptr->angle < ptr->last_angle){			//可能的情况
        res1 = ptr->angle + 8192 - ptr->last_angle;	//正转，delta=+
        res2 = ptr->angle - ptr->last_angle;				//反转	delta=-
    }else{	//angle > last
        res1 = ptr->angle - 8192 - ptr->last_angle ;//反转	delta -
        res2 = ptr->angle - ptr->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(ABS(res1)<ABS(res2))
        delta = res1;
    else
        delta = res2;
    ptr->total_angle += delta;
}
/**
 * @brief 向底盘电机发送电流值
 * @param iq1
 * @param iq2
 * @param iq3
 * @param iq4
 */
void set_moto_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
	{

        TxHeader1.StdId = 0x200;
        TxHeader1.IDE = CAN_ID_STD;
        TxHeader1.RTR = CAN_RTR_DATA;
        TxHeader1.DLC = 0x08;
        TxHeader1.TransmitGlobalTime = DISABLE;
        TxData1[0] = (iq1 >> 8);
        TxData1[1] = iq1;
        TxData1[2] = (iq2 >> 8);
        TxData1[3] = iq2;
        TxData1[4] = iq3 >> 8;
        TxData1[5] = iq3;
        TxData1[6] = iq4 >> 8;
        TxData1[7] = iq4;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox);
}
/**
 * @brief 向机械臂电机发送电流值
 * @param iq1
 * @param iq2
 * @param iq3
 * @param iq4
 */
void set_moto_arm_current(int16_t iq1, int16_t iq2, int16_t iq3)
{
    TxHeader2.StdId = 0x200;
    TxHeader2.IDE = CAN_ID_STD;
    TxHeader2.RTR = CAN_RTR_DATA;
    TxHeader2.DLC = 0x08;
    TxHeader2.TransmitGlobalTime = DISABLE;
    TxData2[0] = (iq1 >> 8);
    TxData2[1] = iq1;
    TxData2[2] = (iq2 >> 8);
    TxData2[3] = iq2;
    TxData2[4] = (iq3 >> 8);
    TxData2[5] = iq3;
    TxData2[6] = 0;
    TxData2[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox);
}
