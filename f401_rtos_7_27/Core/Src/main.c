/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t time=0,cost=0;
uint8_t RxBuffer[16];
uint8_t DMABuffer[DMABuffer_Size];
int32_t value=0;
uint8_t sequence[8];
//标志顺序接收状态
uint8_t state=0;
//MPU9250 angle;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_TIM8_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    RetargetInit(&huart1);
    CAN_Filter_Config();
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_UART_Receive_IT(UART_PI,RxBuffer,10);//接收顺序
    HAL_TIM_Base_Start(&htim11);
    HAL_TIM_Base_Start(&htim13);
//    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
    //底盘电机
    pid_param_init(&motor_pid[0],	3000,500,10,8000,0,2.0f,0.20f,0.3f);
    pid_param_init(&motor_pid[1],	3000,500,10,8000,0,2.0f,0.20f,0.3f);
    pid_param_init(&motor_pid[2],	3000,500,10,8000,0,2.0f,0.20f,0.3f);
    pid_param_init(&motor_pid[3],	3000,500,10,8000,0,2.0f,0.20f,0.3f);
    //机械臂转盘
    pid_param_init(&arm_pid[0],	1000,400,360,1000,5,0.02f,0.01f,0.2f);
    //机械臂升降
    pid_param_init(&arm_pid[1],	2000,500,100,1000,0,0.02f,0.01f,0.1f);
    //机械臂伸缩
    pid_param_init(&arm_pid[2],	2000,500,100,1000,0,0.02f,0.01f,0.1f);
    //运动距离控制
    pid_param_init(&distance_x_pid, 500, 200, 2, 2000, 0, 1.2f, 0.01f, 0.3f);
    pid_param_init(&distance_y_pid, 500, 200, 2,  2000, 0, 1.2f, 0.01f, 0.3f);
    pid_param_init(&distance_z_pid, 1.0f, 200, 0.0175f, 2000, 0, 3.0f, 0.001f, 5);
    //校准pid
    pid_param_init(&calibration_x_pid, 800, 70, 1, 180, 0, 0.9f, 0.01f, 0.2f);
    pid_param_init(&calibration_y_pid, 800, 70, 1, 120, 0, 0.9f, 0.01f, 0.2f);
    pid_param_init(&calibration_z_pid, 0.1f, 0.2f, 0.0175f, 90, 0, 0.1f, 0, 0);
//    MPU9250_init();
//    mpu_dmp_init();
    OLED_Init();
//    uint8_t id=MPU9250_ID();
//    printf("%d",id);
    //    set_arm_height(300);
//  //  set_arm_angle(360);
//    HAL_Delay(10000);
// //   set_arm_angle(0);
//    set_arm_height(0);
//    float pitch=0, roll=0, yaw=0,ax ,ay ,az;
//    if(DMP_Init()!=0)
//    {
//        printf("error");
//    }
//    HAL_Delay(1000);
//    Omni_Reverse_Calculation(500,0,0);
//    Omni_set_target();
//    HAL_Delay(2000);
//    Omni_Reverse_Calculation(0,0,0);
//    Omni_set_target();
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,5);

#ifdef GYRO_ENABLE
    HAL_UARTEx_ReceiveToIdle_DMA(UART_GYRO,DMABuffer,DMABuffer_Size);
    Gyro_Reset();
#endif
//    HAL_UART_Receive_IT(&huart3, rx_yaw, sizeof(float));
//    multi_init();
//

//
//    OLED_ShowNum(0, 0, Distance[0], 4,OLED_8X16);
//
//    OLED_ShowFloatNum(40, 0, (double)angle.yaw,3, 2,OLED_8X16);
//
//    OLED_ShowNum(0, 16, Distance[1], 4,OLED_8X16);
//
//    OLED_ShowNum(0, 32, Distance[2], 4,OLED_8X16);
//
//    OLED_ShowNum(0, 48, Distance[3], 4,OLED_8X16);
 //   OLED_Update();
//调试代码
//    distance_x_pid.target=-1000;
//    distance_y_pid.target=0;
//    distance_z_pid.target=0;
//    uint8_t tx=0x55;
//   HAL_TIM_Base_Start_IT(&htim14);

//    set_cur_pos(0,-400,0);
//    uint8_t sequence[8]= {1,2,3,4,5,6,3,3};
//    Process_Generation(sequence);
    //未获取到数据，等待树莓派数据
    while (state!=2)
    {
        OLED_ShowString(0,0,"waiting for data",OLED_8X16);
        OLED_Update();
    }
    Process_Generation(sequence);
    //打印获取数据
    OLED_Clear();
    for(int i=0;i<6;i++)
        OLED_ShowNum(16*i,0,sequence[i],1,OLED_8X16);
    OLED_ShowNum(0,16,sequence[6],1,OLED_8X16);
    OLED_ShowNum(16,16,sequence[7],1,OLED_8X16);
    OLED_Update();
    //等待按下开始按钮
    while(HAL_GPIO_ReadPin(Start_Button_GPIO_Port,Start_Button_Pin))
    {

    }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //定位
      Omni_Forward_Calculation(moto_chassis);
//      MPU9250_Get_Angle(&angle);
////      Get_Dis();
//      get_distance();
//      OLED_ShowNum(0, 0, Distance[0], 4,OLED_8X16);
//      OLED_ShowFloatNum(40, 0, (double)angle.yaw,3, 2,OLED_8X16);
//      OLED_ShowNum(0, 16, Distance[1], 4,OLED_8X16);
//      OLED_ShowNum(0, 32, Distance[2], 4,OLED_8X16);
//      OLED_ShowNum(0, 48, Distance[3], 4,OLED_8X16);
      OLED_Update();
//      HAL_Delay(5000);
//      __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,10);
//     HAL_Delay(5000);
//      __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,5);

//    motor();
//    Omni_Forward_Calculation(moto_chassis);
//    HAL_Delay(3);
//      MPU6050_DMP_Get_Data(&pitch, &roll, &yaw);
//    //  MPU6050_DMP_Get_Accel(&ax ,&ay ,&az);
//      HAL_UART_Transmit(&huart1,&tx,1,100);
//        printf("%f\n",angle.yaw);
//      printf("%f,%f,%f,%f,%f\n",distance_x_pid.target,cur_pos.x,distance_x_pid.output,cur_pos.y,cur_pos.yaw);
//      HAL_Delay(10);
//        printf("%d\n",time);
//        HAL_Delay(10);
//      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
//      HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==UART_PI)
    {
      //获取夹取顺序
      if(RxBuffer[0]==0xaa&&RxBuffer[9]==0xbb)
      {
        if (RxBuffer[1]==0xee)//数据正确
        {
            state=2;
            return;
        }
        for (int i=0;i<8;i++)
        {
          sequence[i]=RxBuffer[i+1];
        }
        state=1;
        // printf("%f\t%f\t\n",calibration_x_pid.measure,calibration_y_pid.measure);
        HAL_UART_Transmit_IT(UART_PI,RxBuffer,10);
        HAL_UART_Transmit_IT(UART_DEBUG,RxBuffer,10);
        HAL_UART_Receive_IT(UART_PI,RxBuffer,10);
      }
      //获取校准值
      if(RxBuffer[0]==0xcc&&RxBuffer[7]==0xdd)
      {

          calibration_x_pid.measure=(float)((int16_t)(RxBuffer[1]<<8)|RxBuffer[2]);
          calibration_y_pid.measure=(float)((int16_t)(RxBuffer[3]<<8)|RxBuffer[4]);
          calibration_z_pid.measure=(float)((int16_t)(RxBuffer[5]<<8)|RxBuffer[6]);
//           printf("%f\t%f\t\n",calibration_x_pid.measure,calibration_y_pid.measure);
//          printf("%f,%f,%f\n",calibration_x_pid.measure,calibration_y_pid.measure,calibration_z_pid.measure);

      }

    }
}
// 空闲中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance==USART3)
    {
        memcpy(&value,DMABuffer+36+6,sizeof(int32_t));
        if(value>180000||value<-180000)
            value=0;
        if(value<0)
        {
            value=360000+value;
        }
//        printf("%f----\n",yaw_value);
        //再次开启空闲中断接收，不然只会接收一次数据
        HAL_UARTEx_ReceiveToIdle_DMA(UART_GYRO,DMABuffer,DMABuffer_Size);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    if(htim==&htim14)
    {
//        MPU9250_Get_Angle(&angle);

//        time=__HAL_TIM_GET_COUNTER(&htim13);
//        pid_calculate_speed(&distance_x_pid, cur_pos.x);
//        pid_calculate_speed(&distance_y_pid, cur_pos.y);
//        pid_calculate_speed(&distance_z_pid, cur_pos.yaw);
//        Omni_Reverse_Calculation(distance_x_pid.output,distance_y_pid.output,distance_z_pid.output);
//        pid_calculate_speed(&arm_pid[0], moto_arm[0].total_angle);
//        pid_calculate_speed(&arm_pid[1], moto_arm[1].total_angle);
//        pid_calculate_speed(&arm_pid[2], moto_arm[2].total_angle);
//        set_moto_arm_current(arm_pid[0].output,arm_pid[1].output,0);
//        pid_calculate_speed(&motor_pid[0], moto_chassis[0].speed_rpm);
//        pid_calculate_speed(&motor_pid[1], moto_chassis[1].speed_rpm);
//        pid_calculate_speed(&motor_pid[2], moto_chassis[2].speed_rpm);
//        pid_calculate_speed(&motor_pid[3], moto_chassis[3].speed_rpm);
//        set_moto_chassis_current(motor_pid[0].output, motor_pid[1].output, motor_pid[2].output, motor_pid[3].output);
//        uint16_t count=__HAL_TIM_GET_COUNTER(&htim13);
//        if(count<time)
//        {
//            cost=0xffff-time+count;
//        }
//        else
//            cost=count-time;
//        count++;
    }
//开始程序备份
//    //未获取到数据
//    while (state!=2)
//    {
//        OLED_ShowString(0,0,"waiting for data",OLED_8X16);
//        OLED_Update();
//    }
//    Process_Generation(sequence);
//    //打印获取数据
//    for(int i=0;i<6;i++)
//        OLED_ShowNum(16*i,0,sequence[i],1,OLED_8X16);
//    OLED_ShowNum(0,16,sequence[6],1,OLED_8X16);
//    OLED_ShowNum(16,16,sequence[7],1,OLED_8X16);
//    OLED_Update();
//    while(HAL_GPIO_ReadPin(start_button_GPIO_Port,start_button_Pin))
//    {
//
//    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
