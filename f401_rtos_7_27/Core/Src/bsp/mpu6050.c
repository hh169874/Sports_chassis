#include "mpu6050.h"

#ifdef MPU6050_ENABLE

#include "i2c.h"
#include "stdio.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#define q30  1073741824.0f //用于归一化四元数

#define DEFAULT_MPU_HZ  (200) //DMP采样频率200Hz

short gyro[3], accel[3], sensors; //用于读取DMP_FIFO

static signed char gyro_orientation[9] = { 1, 0, 0,  //方向矩阵
                                           0, 1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row) //用于初始化DMP
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;

}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx) //用于初始化DMP
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;

}

static void run_self_test(void) //用于初始化DMP
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        printf("setting bias succesfully ......\r\n");
    }

}

// DMP（数字运动处理器）初始化函数
uint8_t DMP_Init(void)
{
    uint8_t temp[1]={0};
    MPU6050_ReadData(0X75,1,temp);     // 读取MPU6050设备ID
    if(temp[0]!=0x68) NVIC_SystemReset(); // 校验设备ID是否为0x68（正常值），错误则系统复位

    // 主初始化流程
    if(!mpu_init(NULL)) // 成功返回0，初始化MPU6050底层,与步骤7相呼应
    {
        // 配置传感器使能状态
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {} // 启用陀螺仪和加速度计的三轴数据

        // FIFO配置
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {} // 配置FIFO存储陀螺仪和加速度计数据

        // 采样率设置
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ)) {} // 设置采样率为默认值（典型值如100Hz）

        // DMP固件加载
        if(!dmp_load_motion_driver_firmware()) {} // 加载DMP运动驱动固件

        // 方向矩阵配置
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {} // 设置传感器方向校准矩阵

        // 启用DMP特性
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |  // 启用6轴低功耗四元数 | 敲击检测
                               DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |   // Android方向识别 | 原始加速度数据
                               DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {}       // 校准后的陀螺仪数据 | 陀螺仪校准

        // FIFO速率设置
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {} // 设置DMP输出速率与采样率同步

        run_self_test(); // 运行自检程序（校准传感器）

        // 启用DMP
        if(!mpu_set_dmp_state(1)) {} // 激活DMP功能（1=启用，0=关闭）
    }
//    // 1. 初始化MPU6050
//    mpu_init(NULL);
//    // 2. 设置MPU6050传感器
//    // 配置FIFO存储陀螺仪和加速度计数据
//    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//    // 设置陀螺仪量程: ±2000dps
//    mpu_set_gyro_fsr(2000);
//    // 设置加速度计量程: ±2g
//    mpu_set_accel_fsr(2);
//    // 设置数字低通滤波器: 42Hz
//    mpu_set_lpf(42);
//    // 设置采样率: 200Hz (1000/(1 + 4))
//    mpu_set_sample_rate(200);
//
//    // 3. 初始化DMP
//    if (dmp_load_motion_driver_firmware() != 0)
//    {
//        return 1;
//    }
//
//    // 4. 设置DMP方向矩阵 (根据实际安装方向调整)
//    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
//
//    // 5. 启用DMP特性
//    // 启用3轴四元数计算
//    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |
//                           DMP_FEATURE_GYRO_CAL |
//                           DMP_FEATURE_SEND_RAW_ACCEL |
//                           DMP_FEATURE_SEND_CAL_GYRO) != 0)
//    {
//        return 2;
//    }
//
//    // 6. 设置DMP输出速率 (4-200Hz)
//    if (dmp_set_fifo_rate(100) != 0)  // 设置为100Hz
//    {
//        return 3;
//    }
//
//    // 7. 启用DMP
//    if (mpu_set_dmp_state(1) != 0)
//    {
//        return 4;
//    }
//
//    // 8. 重置FIFO
//    mpu_reset_fifo();

    return 0;
}
uint8_t MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw)
{
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];
    
    // 从DMP FIFO读取四元数数据
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        return 1;
		
    // 四元数数据解析（q30格式转浮点）
    if (sensors & INV_WXYZ_QUAT) 
    {
				float q0,q1,q2,q3;
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;

        // 欧拉角计算（单位：度）
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // 俯仰角
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, 
                      -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  // 横滚角
        *yaw   = atan2(2 * (q1 * q2 + q0 * q3),
                       q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;  // 偏航角
    }
    else {
        return 2;
    }
    
    return 0;
}
uint8_t MPU6050_DMP_Get_Accel(float *ax ,float *ay ,float *az)
{
    short accel_data[3];
    if (mpu_get_accel_reg(accel_data, NULL) != 0)
    {
        return 1;
    }

    // 根据设置的量程(这里是±2g)将原始数据转换为g
    accel[0] = (float)accel_data[0] / 16384.0f;  // 16384 = 2g量程的灵敏度
    accel[1] = (float)accel_data[1] / 16384.0f;
    accel[2] = (float)accel_data[2] / 16384.0f;
    return 0;
}



uint8_t I2C_WriteReg(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data)
{ 
	   HAL_StatusTypeDef status;
		status = HAL_I2C_Mem_Write(&hi2c2,0x68<<1,reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, length, HAL_MAX_DELAY);
		return status;
}

uint8_t I2C_ReadReg(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data)
{ 
	   HAL_StatusTypeDef status;
		status = HAL_I2C_Mem_Read(&hi2c2,0x68<<1,reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, length, HAL_MAX_DELAY);
		return status;
}
uint8_t MPU6050_WriteReg(uint8_t regaddr, uint8_t num, uint8_t *regdata)
{
    HAL_StatusTypeDef status;
		status = HAL_I2C_Mem_Write(&hi2c2,0x68<<1,regaddr, I2C_MEMADD_SIZE_8BIT, regdata, num, HAL_MAX_DELAY);
		return status;
}
uint8_t MPU6050_ReadData( uint8_t regaddr,uint8_t num,uint8_t* Read)
{
HAL_StatusTypeDef status;
    // 使用HAL_I2C_Mem_Read函数实现连续读取
    status = HAL_I2C_Mem_Read(&hi2c2,0x68<<1,regaddr, I2C_MEMADD_SIZE_8BIT, Read, num, HAL_MAX_DELAY);
	return status;
}
/******************************************************************
 * 函 数 名 称：MPU_Set_Gyro_Fsr
 * 函 数 说 明：设置MPU6050陀螺仪传感器满量程范围
 * 函 数 形 参：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * 函 数 返 回：0,设置成功  其他,设置失败
 * 作       者：LC
 * 备       注：无
******************************************************************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
        return MPU6050_WriteReg(MPU_GYRO_CFG_REG,1,(uint8_t*)(fsr<<3)); //设置陀螺仪满量程范围
}

/******************************************************************
 * 函 数 名 称：MPU_Set_Accel_Fsr
 * 函 数 说 明：设置MPU6050加速度传感器满量程范围
 * 函 数 形 参：fsr:0,±2g;1,±4g;2,±8g;3,±16g
 * 函 数 返 回：0,设置成功  其他,设置失败
 * 作       者：LC
 * 备       注：无
******************************************************************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
        return MPU6050_WriteReg(MPU_ACCEL_CFG_REG,1,(uint8_t*)(fsr<<3)); //设置加速度传感器满量程范围
}

/******************************************************************
 * 函 数 名 称：MPU_Set_LPF
 * 函 数 说 明：设置MPU6050的数字低通滤波器
 * 函 数 形 参：lpf:数字低通滤波频率(Hz)
 * 函 数 返 回：0,设置成功  其他,设置失败
 * 作       者：LC
 * 备       注：无
******************************************************************/
uint8_t MPU_Set_LPF(uint16_t lpf)
{
        uint8_t data=0;

        if(lpf>=188)data=1;
        else if(lpf>=98)data=2;
        else if(lpf>=42)data=3;
        else if(lpf>=20)data=4;
        else if(lpf>=10)data=5;
        else data=6;
    return data=MPU6050_WriteReg(MPU_CFG_REG,1,&data);//设置数字低通滤波器
}
/******************************************************************
 * 函 数 名 称：MPU_Set_Rate
 * 函 数 说 明：设置MPU6050的采样率(假定Fs=1KHz)
 * 函 数 形 参：rate:4~1000(Hz)  初始化中rate取50
 * 函 数 返 回：0,设置成功  其他,设置失败
 * 作       者：LC
 * 备       注：无
******************************************************************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
        uint8_t data;
        if(rate>1000)rate=1000;
        if(rate<4)rate=4;
        data=1000/rate-1;
        data=MPU6050_WriteReg(MPU_SAMPLE_RATE_REG,1,&data);        //设置数字低通滤波器
         return MPU_Set_LPF(rate/2);            //自动设置LPF为采样率的一半
}

/******************************************************************
 * 函 数 名 称：MPU6050ReadGyro
 * 函 数 说 明：读取陀螺仪数据
 * 函 数 形 参：陀螺仪数据存储地址
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void MPU6050ReadGyro(short *gyroData)
{
        uint8_t buf[6];
        uint8_t reg = 0;
        //MPU6050_GYRO_OUT = MPU6050陀螺仪数据寄存器地址
        //陀螺仪数据输出寄存器总共由6个寄存器组成，
        //输出X/Y/Z三个轴的陀螺仪传感器数据，高字节在前，低字节在后。
        //每一个轴16位，按顺序为xyz
        reg = MPU6050_ReadData(MPU6050_GYRO_OUT,6,buf);
        if( reg == 0 )
        {
                gyroData[0] = (buf[0] << 8) | buf[1];
                gyroData[1] = (buf[2] << 8) | buf[3];
                gyroData[2] = (buf[4] << 8) | buf[5];
        }
}

/******************************************************************
 * 函 数 名 称：MPU6050ReadAcc
 * 函 数 说 明：读取加速度数据
 * 函 数 形 参：加速度数据存储地址
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void MPU6050ReadAcc(short *accData)
{
        uint8_t buf[6];
        uint8_t reg = 0;
        //MPU6050_ACC_OUT = MPU6050加速度数据寄存器地址
        //加速度传感器数据输出寄存器总共由6个寄存器组成，
        //输出X/Y/Z三个轴的加速度传感器值，高字节在前，低字节在后。
        reg = MPU6050_ReadData( MPU6050_ACC_OUT, 6, buf);
        if( reg == 0)
        {
                accData[0] = (buf[0] << 8) | buf[1];
                accData[1] = (buf[2] << 8) | buf[3];
                accData[2] = (buf[4] << 8) | buf[5];
        }
}

/******************************************************************
 * 函 数 名 称：MPU6050_GetTemp
 * 函 数 说 明：读取MPU6050上的温度
 * 函 数 形 参：无
 * 函 数 返 回：温度值单位为℃
 * 作       者：LC
 * 备       注：温度换算公式为：Temperature = 36.53 + regval/340
******************************************************************/
float MPU6050_GetTemp(void)
{
        short temp3;
        uint8_t buf[2];
        float Temperature = 0;
        MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,2,buf);
    temp3= (buf[0] << 8) | buf[1];
        Temperature=((double) temp3/340.0)+36.53;
    return Temperature;
}

/******************************************************************
 * 函 数 名 称：MPU6050ReadID
 * 函 数 说 明：读取MPU6050的器件地址
 * 函 数 形 参：无
 * 函 数 返 回：0=检测不到MPU6050   1=能检测到MPU6050
 * 作       者：LC
 * 备       注：无
******************************************************************/
uint8_t MPU6050ReadID(void)
{
        unsigned char Re[2] = {0};
        //器件ID寄存器 = 0x75
        printf("mpu=%d\r\n",MPU6050_ReadData(0X75,1,Re)); //读器件地址

        if (Re[0] != 0x68)
        {
                printf("检测不到 MPU6050 模块");
                return 1;
         }
        else
        {
                printf("MPU6050 ID = %x\r\n",Re[0]);
                return 0;
        }
        return 0;
}

/******************************************************************
 * 函 数 名 称：MPU6050_Init
 * 函 数 说 明：MPU6050初始化
 * 函 数 形 参：无
 * 函 数 返 回：0成功  1没有检测到MPU6050
 * 作       者：LC
 * 备       注：无
******************************************************************/
char MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 1,(uint8_t*)(0x80));
    HAL_Delay(100);
    //电源管理寄存器
    //选择X轴陀螺作为参考PLL的时钟源，设置CLKSEL=001
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1,1, (uint8_t*)(0x00));

    MPU_Set_Gyro_Fsr(3);    //陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);   //加速度传感器,±2g
    MPU_Set_Rate(50);

    MPU6050_WriteReg(MPU_INT_EN_REG , 1,(uint8_t*)0x00);        //关闭所有中断
    MPU6050_WriteReg(MPU_USER_CTRL_REG,1,(uint8_t*)0x00);        //I2C主模式关闭
    MPU6050_WriteReg(MPU_FIFO_EN_REG,1,(uint8_t*)0x00);                //关闭FIFO
    MPU6050_WriteReg(MPU_INTBP_CFG_REG,1,(uint8_t*)0X80);        //INT引脚低电平有效

    if( MPU6050ReadID() == 0 )//检查是否有6050
    {
            MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 1,(uint8_t*)0x01);//设置CLKSEL,PLL X轴为参考
            MPU6050_WriteReg(MPU_PWR_MGMT2_REG, 1,(uint8_t*)0x00);//加速度与陀螺仪都工作
            MPU_Set_Rate(50);
            return 1;
    }
    return 0;
}

#endif
