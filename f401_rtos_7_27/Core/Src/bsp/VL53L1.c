#include "VL53L1.h"
#ifdef VL53L1_ENABLE
#include "VL53L1X_API.h"
#include "VL53L1X_calibration.h"
int16_t offset;
uint16_t xtalk;
uint16_t Distance[5];
uint8_t single_init(uint16_t dev)
{
    int8_t status=0;
    /* This function must to be called to initialize the sensor with the default setting  */

    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    status = VL53L1X_SetDistanceMode(dev, 1); /* 1=short, 2=long */
    status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//    status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */
//  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
    status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
//	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
//    status = VL53L1X_SetOffset(dev,offset);
//	status = VL53L1X_CalibrateXtalk(dev, 400, &xtalk); /* may take few second to perform the xtalk cal */
    status = VL53L1X_StartRanging(dev);
    return status;
}
uint8_t multi_init(void)
{
    int8_t status=0;
    HAL_GPIO_WritePin(XShut1_GPIO_Port,XShut1_Pin,GPIO_PIN_SET);
    HAL_Delay(2);
    status = VL53L1X_SensorInit(add_default);
    status = VL53L1X_SetI2CAddress(add_default,add1);
    single_init(add1);
    HAL_Delay(2);

    HAL_GPIO_WritePin(XShut2_GPIO_Port,XShut2_Pin,GPIO_PIN_SET);
    HAL_Delay(2);
    status = VL53L1X_SensorInit(add_default);
    status = VL53L1X_SetI2CAddress(add_default,add2);
    single_init(add2);
    HAL_Delay(2);

    HAL_GPIO_WritePin(XShut3_GPIO_Port,XShut3_Pin,GPIO_PIN_SET);
    HAL_Delay(2);
    status = VL53L1X_SensorInit(add_default);
    status = VL53L1X_SetI2CAddress(add_default,add3);
    single_init(add3);

    HAL_GPIO_WritePin(XShut4_GPIO_Port,XShut4_Pin,GPIO_PIN_SET);
    HAL_Delay(2);
    status = VL53L1X_SensorInit(add_default);
    status = VL53L1X_SetI2CAddress(add_default,add4);
    single_init(add4);

//    HAL_GPIO_WritePin(XShut5_GPIO_Port,XShut5_Pin,GPIO_PIN_SET);
//    status = VL53L1X_SensorInit(add_default);
//    status = VL53L1X_SetI2CAddress(add_default,add5);
//    single_init(add5);
    return status;
}
void get_distance(void)
{
    VL53L1X_GetDistance(add1, Distance);
    VL53L1X_ClearInterrupt(add1);

    VL53L1X_GetDistance(add2, Distance+1);
    VL53L1X_ClearInterrupt(add2);

    VL53L1X_GetDistance(add3, Distance+2);
    VL53L1X_ClearInterrupt(add3);

    VL53L1X_GetDistance(add4, Distance+3);
    VL53L1X_ClearInterrupt(add4);

//    VL53L1X_GetDistance(add5, Distance+4);
//    VL53L1X_ClearInterrupt(add5);
}
#endif
