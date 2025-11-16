/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"

#include "bsp_can.h"
#include "pid.h"
#include "Chassis_Control.h"
#include "mpu6050.h"
#include "Chassis_Control.h"
#include "OLED.h"
#include "HC_SR04.h"
#include "map.h"
#include "Arm_Control.h"
#include "VL53L1.h"
#include "MPU9250.h"
#include "VL53L1X_API.h"
#include "Process_Control.h"
#include "event_groups.h"
#include "usart.h"
#include "Gyro.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t RxBuffer[16];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
BaseType_t ret;//返回值
TaskHandle_t OFC;//运动正解任务
TaskHandle_t DSP;//双环pid任务 distance_speed_pid_task
TaskHandle_t CPT;//校准pid任务 calibration_pid_task
TaskHandle_t OLEDT;//oled显示任务 oled_task
TaskHandle_t PCT;//流程控制任务 process_control_task
TaskHandle_t APT;//机械臂控制任务 arm_pid_task
//QueueHandle_t control_flow_queue;//控制流队列
//EventGroupHandle_t arrive_event_group;//控制流事件组
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Omni_Forward_Calculation_task(void *param);
void distance_speed_pid_task(void *param);
void calibration_pid_task(void *parm);
void oled_task(void *parm);
void process_control_task(void *parm);
void arm_pid_task(void *parm);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//    control_flow_queue= xQueueCreate(5, sizeof(control_block));//创建控制队列
//    arrive_event_group= xEventGroupCreate();//创建控制完成事件组
  /* add threads, ... */
  ret= xTaskCreate(oled_task,"oled_task",256,NULL,3,&OLEDT);
  ret= xTaskCreate(Omni_Forward_Calculation_task,"Omni_Forward_Calculation_task",256,NULL,3,&OFC);
  ret= xTaskCreate(distance_speed_pid_task,"distance_speed_pid_task",256,NULL,3,&DSP);
  ret= xTaskCreate(process_control_task,"process_control_task",256,NULL,4,&PCT);
  ret= xTaskCreate(arm_pid_task,"arm_pid_task",128,NULL,3,&APT);
  ret= xTaskCreate(calibration_pid_task,"calibration_pid_task",256,NULL,3,&CPT);
    //阻塞校准pid任务
    vTaskSuspend(CPT);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief 全向轮正解任务
 * @param param
 */
void Omni_Forward_Calculation_task(void *param)
{
    while(1)
    {
        Mecanum_Forward_Calculation(moto_chassis);
        vTaskDelay(20);
    }
}
/**
 * @brief 双环pid计算任务，距离->速度->电流
 * @param param
 */
void distance_speed_pid_task(void *param)
{
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1)
    {
        //位置环
        pid_calculate_position(&distance_x_pid, cur_pos.x);
        pid_calculate_position(&distance_y_pid, cur_pos.y);
        pid_calculate_position_z(&distance_z_pid, cur_pos.yaw);
        //计算四轮速度并设置速度环pid target
        //distance_x_pid.output=0;
//        distance_y_pid.output=0;
//        distance_z_pid.output=0;
        Mecanum_Reverse_Calculation(distance_x_pid.output,distance_y_pid.output,distance_z_pid.output);
//        Mecanum_Reverse_Calculation(100,0,0);
//        printf("%d,%d,%d,%d,%f,%f,%f,%f,%f\n",moto_chassis[0].speed_rpm,moto_chassis[1].speed_rpm,moto_chassis[2].speed_rpm,moto_chassis[3].speed_rpm,motor_pid[0].target,motor_pid[1].target,motor_pid[2].target,motor_pid[3].target,cur_pos.yaw);
//        motor();
        //速度环
        pid_calculate_speed(&motor_pid[0], moto_chassis[0].speed_rpm);
        pid_calculate_speed(&motor_pid[1], moto_chassis[1].speed_rpm);
        pid_calculate_speed(&motor_pid[2], moto_chassis[2].speed_rpm);
        pid_calculate_speed(&motor_pid[3], moto_chassis[3].speed_rpm);
        set_moto_chassis_current(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);

        //printf("%f,%f,%f,%f\n",distance_z_pid.measure,distance_z_pid.target,distance_z_pid.output,distance_x_pid.output);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
/**
 * @brief 机械臂id计算任务，位置->电流
 * @param param
 */
void arm_pid_task(void *parm)
{
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1)
    {
        pid_calculate_position(&arm_pid[0], moto_arm[0].total_angle);
        pid_calculate_position(&arm_pid[1], moto_arm[1].total_angle);
        pid_calculate_position(&arm_pid[2], moto_arm[2].total_angle);
//        arm_pid[1].output=0;
        set_moto_arm_current(arm_pid[0].output, arm_pid[1].output, arm_pid[2].output);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
/**
 * @brief 校准pid
 * @param parm
 */
void calibration_pid_task(void *parm)
{
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1)
    {
//        printf("%f,%f,%f\n",calibration_x_pid.measure,calibration_y_pid.measure,calibration_z_pid.measure);
//        HAL_UART_Transmit_IT(UART_DEBUG,RxBuffer,10);

        float x=calibration_x_pid.measure,y=calibration_y_pid.measure,z=calibration_z_pid.measure;
        pid_calculate_position(&calibration_x_pid, x);
        pid_calculate_position(&calibration_y_pid, y);
        pid_calculate_position(&calibration_z_pid, z);
//        printf("%f,%f,%f,%f,%f,%f\n",calibration_x_pid.output,calibration_y_pid.output,calibration_z_pid.output,calibration_x_pid.measure,calibration_y_pid.measure,calibration_z_pid.measure);
        //计算四轮速度并设置速度环pid target,不进行坐标变换
        Mecanum_Reverse_Calculation_unchanged(calibration_x_pid.output,calibration_y_pid.output,calibration_z_pid.output);

        pid_calculate_speed(&motor_pid[0], moto_chassis[0].speed_rpm);
        pid_calculate_speed(&motor_pid[1], moto_chassis[1].speed_rpm);
        pid_calculate_speed(&motor_pid[2], moto_chassis[2].speed_rpm);
        pid_calculate_speed(&motor_pid[3], moto_chassis[3].speed_rpm);
        set_moto_chassis_current(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);

        HAL_UART_Receive_IT(UART_PI,RxBuffer,8);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
/**
 * @brief oled屏幕显示任务
 * @param parm
 */
void oled_task(void *parm)
{
//    extern MPU9250 angle;
    TickType_t xLastWakeTime=xTaskGetTickCount();
    while(1)
    {
        float x,y,yaw;
        get_cur_pos(&x,&y,&yaw);

        OLED_ShowFloatNum(0, 0, x,4, 2,OLED_8X16);
        OLED_ShowFloatNum(0, 16,y,4, 2,OLED_8X16);
        OLED_ShowFloatNum(0, 32, yaw,4, 2,OLED_8X16);

//        OLED_ShowFloatNum(64, 0, distance_x_pid.output,3, 2,OLED_8X16);
//        OLED_ShowFloatNum(64, 16, distance_y_pid.output,3, 2,OLED_8X16);
//        OLED_ShowFloatNum(64, 32, distance_z_pid.output,3, 2,OLED_8X16);
        OLED_ShowFloatNum(64, 0, moto_chassis[0].speed_rpm,3, 2,OLED_8X16);
        OLED_ShowFloatNum(64, 16, moto_chassis[1].speed_rpm,3, 2,OLED_8X16);
        OLED_ShowFloatNum(64, 32, moto_chassis[2].speed_rpm,3, 2,OLED_8X16);
        OLED_ShowFloatNum(64, 48, moto_chassis[3].speed_rpm,3, 2,OLED_8X16);
//        OLED_ShowFloatNum(0, 48, moto_chassis[0].angle,3, 2,OLED_8X16);
//        OLED_ShowFloatNum(0, 48, (double)angle.yaw,3, 2,OLED_8X16);
        OLED_Update();
        //printf("%d,%d,%d,%d,%f,%f,%f,%f\n",moto_chassis[0].speed_rpm,moto_chassis[1].speed_rpm,moto_chassis[2].speed_rpm,moto_chassis[3].speed_rpm,motor_pid[0].target,motor_pid[1].target,motor_pid[2].target,motor_pid[3].target);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
/**
 * @brief 流程控制任务
 * @param param
 */
void process_control_task(void *parm)
{
    TickType_t xLastWakeTime=xTaskGetTickCount();
    uint8_t tx_data;
    for(uint8_t i=0;i<control_flow_size;i++)
    {
        if(control_flow[i].uart)
        {
            tx_data=(control_flow[i].uart<<4)|control_flow[i].uart;
            HAL_UART_Transmit_IT(UART_PI, &tx_data, 1);
//            printf("%d\n",tx_data);
            //开始校准
            HAL_UART_Receive_IT(UART_PI,RxBuffer,8);
            //阻塞双环pid
            vTaskSuspend(DSP);
            //阻塞全向轮正解任务
            vTaskSuspend(OFC);
            //恢复校准pid
            vTaskResume(CPT);
            set_cur_pos(control_flow[i].pos_target.x,control_flow[i].pos_target.y,control_flow[i].pos_target.yaw);
            HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET);
        }
        else
        {
            //阻塞校准pid
            vTaskSuspend(CPT);
            HAL_UART_AbortReceive_IT(&huart2);  // 中止接收中断
            //恢复双环pid
            vTaskResume(DSP);
            //恢复全向轮正解任务,恢复前更新上次运动位移
            Mecanum_Forward_Calculation_patch(moto_chassis);
            vTaskResume(OFC);
            HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET);
        }
        //设置运动位置
        distance_x_pid.target=control_flow[i].pos_target.x;
        distance_y_pid.target=control_flow[i].pos_target.y;
        distance_z_pid.target=control_flow[i].pos_target.yaw;
        //设置转盘角度
        set_arm_angle(control_flow[i].arm_target.state);
        //设置升降高度
        set_arm_height(control_flow[i].arm_target.height);
        //设置伸缩长度
        set_arm_length(control_flow[i].arm_target.length);
        //设置舵机夹取
        if(control_flow[i].steering_engine)//夹取
            __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,20);
        else
            __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,5);
        vTaskDelayUntil(&xLastWakeTime, control_flow[i].delay);
    }
    vTaskDelete(NULL);
}

/* USER CODE END Application */

