/**
 * @file dvc_AKmotor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief AK电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 初稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef CHASSIS_COMMUNICATION_H
#define CHASSIS_COMMUNICATION_H

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"
#include "drv_math.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief AK电机状态
 *
 */
enum Enum_Chassis_Communication_Status
{
    Chassis_Communication_Status_DISABLE = 0,
    Chassis_Communication_Status_ENABLE,
};

/**
 * @brief AK电机的ID枚举类型
 *
 */
enum Enum_Chassis_Communication_ID : uint8_t
{
    Chassis_Communication_ID_0x11 = 0x11,
    Chassis_Communication_ID_0x12,
    Chassis_Communication_ID_0x13,
    Chassis_Communication_ID_0x14,
    Chassis_Communication_ID_0x15,
    Chassis_Communication_ID_0x16,
//    Chassis_Task_ID_0x17,
//	Chassis_Task_ID_0x18,
};

//struct Struct_Tx_Data_ID_0x11
//{
//	int16_t Chassis_Vx;
//	int16_t Chassis_Vy;
//	int16_t Chassis_Wz;
//	uint16_t Robotarm_Pz;
//} __attribute__((packed));

/**
 * @brief 底盘通信类
 *
 */
class Class_Chassis_Communication
{
public:

	void Init(CAN_HandleTypeDef *hcan);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void Task_Alive_PeriodElapsedCallback();
    void Task_Process_PeriodElapsedCallback();
	
	inline Enum_Chassis_Communication_Status Get_Chassis_Communication_Status();
	//电机对外接口信息
	void Communication_Data(float Data,float Min_Data,float Max_Data,Enum_Chassis_Communication_ID Chassis_Communication_ID,uint8_t Pos);
    void Communication_Data(uint8_t Mode,Enum_Chassis_Communication_ID Chassis_Communication_ID,uint8_t Pos);
protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_Chassis_Communication_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
	//ID_0x11发送内容
//	Struct_Tx_Data_ID_0x11 Tx_Data_ID_0x11;
    //位置反馈偏移

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
	//ID偏移量
	const uint8_t ID_Bias = 0x11;

    //读变量

    //状态
    Enum_Chassis_Communication_Status Chassis_Communication_Status = Chassis_Communication_Status_DISABLE;
    

    //写变量

    //读写变量
    //内部函数

    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Chassis_Task_Status 电机状态
 */
Enum_Chassis_Communication_Status Class_Chassis_Communication::Get_Chassis_Communication_Status()
{
    return (Chassis_Communication_Status);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
