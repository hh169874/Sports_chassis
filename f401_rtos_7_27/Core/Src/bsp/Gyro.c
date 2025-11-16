#include "Gyro.h"
float Gyro_yaw=0;
/**
 * @brief 陀螺仪reset函数
 */
void Gyro_Reset(void)
{
    HAL_GPIO_WritePin(Gyro_Reset_GPIO_Port,Gyro_Reset_Pin,GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(Gyro_Reset_GPIO_Port,Gyro_Reset_Pin,GPIO_PIN_SET);
}
//void Get_yaw(uint8_t *buffer)
//{
//    float yaw;
//    int32_t value;
//    memcpy(&value,buffer+36+6,sizeof(int32_t));
//    yaw=value*0.001f;
////    printf("%f\n",yaw);
////    HAL_UART_Transmit(&huart1,buffer,54,100);
////    printf("aaaaaaaa");
//}
