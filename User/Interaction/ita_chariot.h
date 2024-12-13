#ifndef _ITA_CHARIOT_H_
#define _ITA_CHARIOT_H_
#ifdef __cplusplus
#include "crt_booster.h"
#include "crt_gimbal.h"
#include "dvc_dr16.h"
#include "drv_uart.h"
#include "drv_math.h"
#include "alg_fsm.h"
#include "crt_chassis.h"
#include "drv_can.h"
class Class_Chariot;
/**
 * @brief 云台通讯状态
 *
 */
enum Enum_Gimbal_Status_
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_ENABLE = 1,
};

class Class_Chariot
{
public:
    Class_Tricycle_Chassis chassis;
    void Init();
    void TIM_Control_Callback();
    void TIM_Chariot_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void CAN_Chassis_Rx_Gimbal_Callback();
    void TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
protected:
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object = &CAN2_Manage_Object;
    uint32_t Gimbal_Alive_Flag = 0;
    uint32_t Pre_Gimbal_Alive_Flag = 0;
    Enum_Gimbal_Status_ Gimbal_Status =  Gimbal_Status_DISABLE;
    //内部函数
    void Control_Chassis();
};
#define DR16_Yaw_Angle_Resolution  0.005f * PI * 57.29577951308232
#endif
#endif
