#include "MPU9250.h"
//#include "Myiic.h"


#ifdef MPU9250_ENABLE

#define i2c_mpu9250 hi2c1
#define EMPL

#include "mpu9250.h"
#include "main.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdio.h"


//#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//MPU9250驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
void MPU9250_Init(void)
{
    printf("-- Mpu9250 Project Start -- \r\n");
    uint8_t recv = 0x00;
    uint8_t i2c_err;
    i2c_err = HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x75, I2C_MEMADD_SIZE_8BIT, &recv, 1, 0xfff);
    if(recv == 0x71)
    {
        printf("mpu9250 ID Read: OK(0x71 at 0x75)\r\n");
    }
    else
    {
        printf("Err:%d\r\n",i2c_err);
    }
    uint8_t err = mpu_dmp_init();
    if(err)
    {
        printf("mpu_init_err:%d\r\n",err);
    }
    else
    { printf("mpu9250 Ok\r\n"); }
}


//初始化MPU9250
//返回值:0,成功
//    其他,错误代码
//u8 MPU9250_Init(void)
//{
//    u8 res=0;
//    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//??MPU9250
//    HAL_Delay(100);  //??100ms
//    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//??MPU9250
//    MPU_Set_Gyro_Fsr(3);					        	//??????,?000dps
//	  MPU_Set_Accel_Fsr(0);					       	 	//??????,?g
//    MPU_Set_Rate(50);						       	 	//?????50Hz
//    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //??????
//	  MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C?????
//	  MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//??FIFO
//	  MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT???????,??bypass??,?????????
//    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //??MPU6500?ID
//    if(res==MPU6500_ID) //??ID??
//    {
//        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//??CLKSEL,PLL X????
//        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//??????????
//		    MPU_Set_Rate(50);						       	//??????50Hz
//    }else return 1;
//
//    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//??AK8963 ID
//    if(res==AK8963_ID)
//    {
//        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//??AK8963???????
//    }else return 1;
//
//    return 0;
//}

//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((u16)buf[0]<<8)|buf[1];
    temp=21+((double)raw)/333.87;
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];
		*gy=((u16)buf[2]<<8)|buf[3];
		*gz=((u16)buf[4]<<8)|buf[5];
	}
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];
		*ay=((u16)buf[2]<<8)|buf[3];
		*az=((u16)buf[4]<<8)|buf[5];
	}
    return res;;
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res;
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];
		*my=((u16)buf[3]<<8)|buf[2];
		*mz=((u16)buf[5]<<8)|buf[4];
	}
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;;
}

//IIC???
//addr:????
//reg:?????
//len:????
//buf:???
//???:0,??
//    ??,????
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
     HAL_I2C_Mem_Write(&i2c_mpu9250, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
     return 0;
}

//IIC???
//addr:????
//reg:?????????
//len:??????
//buf:?????????
//???:0,??
//    ??,????
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    HAL_I2C_Mem_Read(&i2c_mpu9250, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    return 0;
}

//IIC?????
//devaddr:??IIC??
//reg:?????
//data:??
//???:0,??
//    ??,????
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    HAL_I2C_Mem_Write(&i2c_mpu9250, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xfff);
    return 0;
}

//IIC?????
//reg:?????
//???:?????
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    HAL_I2C_Mem_Read(&i2c_mpu9250, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 0xfff);
    return res;
}

#endif
