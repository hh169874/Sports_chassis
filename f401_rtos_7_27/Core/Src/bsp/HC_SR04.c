#include "HC_SR04.h"

#ifdef HC_SR04_ENABLE
#include "tim.h"
float distance[]= {0,0,0,0};
uint16_t count[]={0,0,0,0};
void Get_Dis(void)
{
		HAL_GPIO_WritePin(SR04_1_Trig_GPIO_Port,SR04_1_Trig_Pin,GPIO_PIN_SET);
		Delayus(12);
        HAL_GPIO_WritePin(SR04_1_Trig_GPIO_Port,SR04_1_Trig_Pin,GPIO_PIN_RESET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(SR04_1_Trig_GPIO_Port,SR04_1_Trig_Pin,GPIO_PIN_SET);
        Delayus(12);
        HAL_GPIO_WritePin(SR04_1_Trig_GPIO_Port,SR04_1_Trig_Pin,GPIO_PIN_RESET);
        HAL_Delay(5);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        static uint8_t rise=1;
        if (rise)
        {
            count[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            rise=0;
        }
        else
        {
            uint16_t cur_count=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            if (cur_count > count[0])
            {
                cur_count = cur_count - count[0];
            }
            else
            {
                cur_count = (0xFFFF - count[0]) + cur_count;
            }
            distance[0] = cur_count*voice_speed;
            rise=1;
        }
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==SR04_5_Echo_Pin)
	{
		if(HAL_GPIO_ReadPin(SR04_1_Echo_GPIO_Port,SR04_1_Echo_Pin)==1)
		{
            count[0]=__HAL_TIM_GetCounter(&htim13);
		}
		else
		{
			uint16_t cur_counter = __HAL_TIM_GetCounter(&htim13);
            if(cur_counter<count[0])//已触发更新时间
            {
                cur_counter=0xffff-count[0]+cur_counter;
            }
            else
                cur_counter=cur_counter-count[0];
            //25° 50%湿度
			distance[0] = cur_counter*voice_speed;
		}
	}
    else if(GPIO_Pin==SR04_2_Echo_Pin)
    {
        if(HAL_GPIO_ReadPin(SR04_2_Echo_GPIO_Port,SR04_2_Echo_Pin)==1)
        {
            count[1]=__HAL_TIM_GetCounter(&htim13);
        }
        else
        {
            uint16_t cur_counter = __HAL_TIM_GetCounter(&htim13);
            if(cur_counter<count[1])//已触发更新时间
            {
                cur_counter=0xffff-count[1]+cur_counter;
            }
            else
                cur_counter=cur_counter-count[1];
            //25° 50%湿度
            distance[1] = cur_counter*voice_speed;
        }
    }
    else if(GPIO_Pin==SR04_3_Echo_Pin)
    {
        if(HAL_GPIO_ReadPin(SR04_3_Echo_GPIO_Port,SR04_3_Echo_Pin)==1)
        {
            count[2]=__HAL_TIM_GetCounter(&htim13);
        }
        else
        {
            uint16_t cur_counter = __HAL_TIM_GetCounter(&htim13);
            if(cur_counter<count[2])//已触发更新时间
            {
                cur_counter=0xffff-count[2]+cur_counter;
            }
            else
                cur_counter=cur_counter-count[2];
            //25° 50%湿度
            distance[2] = cur_counter*voice_speed;
        }
    }
    else if(GPIO_Pin==SR04_4_Echo_Pin)
    {
        if(HAL_GPIO_ReadPin(SR04_4_Echo_GPIO_Port,SR04_4_Echo_Pin)==1)
        {
            count[3]=__HAL_TIM_GetCounter(&htim13);
        }
        else
        {
            uint16_t cur_counter = __HAL_TIM_GetCounter(&htim13);
            if(cur_counter<count[3])//已触发更新时间
            {
                cur_counter=0xffff-count[3]+cur_counter;
            }
            else
                cur_counter=cur_counter-count[3];
            //25° 50%湿度
            distance[3] = cur_counter*voice_speed;
        }
    }
}
#endif
