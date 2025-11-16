#ifndef __MPU_6050__
#define __MPU_6050__
#include "main.h"

#ifdef MPU6050_ENABLE
//MPU6050的AD0是IIC地址引脚，接地则IIC地址为0x68,接VCC则IIC地址为0x69

#define MPU6050_RA_SMPLRT_DIV       0x19        //陀螺仪采样率 地址
#define MPU6050_RA_CONFIG           0x1A        //设置数字低通滤波器 地址
#define MPU6050_RA_GYRO_CONFIG      0x1B        //陀螺仪配置寄存器
#define MPU6050_RA_ACCEL_CONFIG     0x1C        //加速度传感器配置寄存器
#define MPU_INT_EN_REG              0X38        //中断使能寄存器
#define MPU_USER_CTRL_REG           0X6A        //用户控制寄存器
#define MPU_FIFO_EN_REG             0X23        //FIFO使能寄存器
#define MPU_PWR_MGMT2_REG           0X6C        //电源管理寄存器2
#define MPU_GYRO_CFG_REG            0X1B        //陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG           0X1C        //加速度计配置寄存器
#define MPU_CFG_REG                 0X1A        //配置寄存器
#define MPU_SAMPLE_RATE_REG         0X19        //采样频率分频器
#define MPU_INTBP_CFG_REG           0X37        //中断/旁路设置寄存器

#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C

#define MPU6050_WHO_AM_I            0x75
#define MPU6050_SMPLRT_DIV          0            //8000Hz
#define MPU6050_DLPF_CFG            0
#define MPU6050_GYRO_OUT            0x43         //MPU6050陀螺仪数据寄存器地址
#define MPU6050_ACC_OUT             0x3B         //MPU6050加速度数据寄存器地址

#define MPU6050_RA_TEMP_OUT_H       0x41        //温度高位
#define MPU6050_RA_TEMP_OUT_L       0x42        //温度低位

#define MPU_ACCEL_XOUTH_REG         0X3B        //加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG         0X3C        //加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG         0X3D        //加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG         0X3E        //加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG         0X3F        //加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG         0X40        //加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG           0X41        //温度值高八位寄存器
#define MPU_TEMP_OUTL_REG           0X42        //温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG          0X43        //陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG          0X44        //陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG          0X45        //陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG          0X46        //陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG          0X47        //陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG          0X48        //陀螺仪值,Z轴低8位寄存器

uint8_t MPU6050_WriteReg(uint8_t regaddr, uint8_t num, uint8_t *regdata);
uint8_t MPU6050_ReadData(uint8_t regaddr, uint8_t num, uint8_t *regdata);
uint8_t I2C_WriteReg(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data);
uint8_t I2C_ReadReg(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data);

char MPU6050_Init(void);
void MPU6050ReadGyro(short *gyroData);
void MPU6050ReadAcc(short *accData);
float MPU6050_GetTemp(void);
uint8_t MPU6050ReadID(void);

uint8_t DMP_Init(void);
uint8_t MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw);
uint8_t MPU6050_DMP_Get_Accel(float *ax ,float *ay ,float *az);
#endif
#endif
//DMP_Init();
//	float pitch,roll,yaw,ax,ay,az;
//  /* USER CODE END 2 */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
////MPU6050_DMP_Get_Data(&pitch,&roll,&yaw,&ax,&ay,&az);
////		printf("%f\t%f\t%f\t%f\t%f\t%f\r\n", pitch,roll,yaw,ax,ay,az);
////				motor();
////				printf("%f,%f,%f,%d,%d,%d\r\n",motor_pid[0].target,motor_pid[0].measure,motor_pid[0].output,RockerValue[3],RockerValue[2],RockerValue[0]);
////				//printf("%d %d %d %d \r\n",moto_chassis[0].speed_rpm,moto_chassis[1].speed_rpm,moto_chassis[2].speed_rpm,moto_chassis[3].speed_rpm);
//				HAL_Delay(50);
//  }