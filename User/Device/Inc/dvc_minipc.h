/**
 * @file dvc_minipc.h
 * @author lez by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DVC_MINIPC_H
#define DVC_MINIPC_H

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "main.h"
#include "drv_usb.h"
#include "dvc_imu.h"
#include "dvc_referee.h"
#include "math.h"
/* Exported macros -----------------------------------------------------------*/

class Class_Gimbal_Pitch_Motor_GM6020;

class Class_Gimbal_Yaw_Motor_GM6020;

/* Exported types ------------------------------------------------------------*/

static const uint16_t MINPC_CRC16_INIT = 0xFFFF;

static const uint16_t W_CRC_TABLE[256] = 
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/**
 * @brief 风车类型
 *
 */
enum Enum_Windmill_Type :uint8_t
{
    Windmill_Type_Small = 0,
    Windmill_Type_Big,
};


/**
 * @brief 迷你主机控制类型
 *
 */
enum Enum_MiniPC_Type : uint8_t
{
    MiniPC_Type_Nomal = 4,    //装甲板
    MiniPC_Type_Windmill,       //风车
};


/**
 * @brief 迷你主机状态
 *
 */
enum Enum_MiniPC_Status :uint8_t
{
    MiniPC_Status_DISABLE = 0,
    MiniPC_Status_ENABLE,
};

/**
 * @brief 裁判系统UI刷新状态
 *
 */
enum Enum_Referee_UI_Refresh_Status : uint8_t
{
    Referee_UI_Refresh_Status_DISABLE = 0,
    Referee_UI_Refresh_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum Enum_MiniPC_Data_Status : uint8_t
{
    MiniPC_Data_Status_DISABLE = 0,
    MiniPC_Data_Status_ENABLE,
};

/**
 * @brief 比赛阶段
 *
 */
enum Enum_MiniPC_Game_Stage : uint8_t
{
    MiniPC_Game_Stage_NOT_STARTED = 0,
    MiniPC_Game_Stage_READY,
    MiniPC_Game_Stage_SELF_TESTING,
    MiniPC_Game_Stage_5S_COUNTDOWN,
    MiniPC_Game_Stage_BATTLE,
    MiniPC_Game_Stage_SETTLEMENT,
};

/**
 * @brief 底盘运动控制方式
 *
 */
enum Enum_MiniPC_Chassis_Control_Mode : uint8_t
{
    MiniPC_Chassis_Control_Mode_NORMAL = 0,  // 不随动
    MiniPC_Chassis_Control_Mode_FOLLOW,      // 随动
    MiniPC_Chassis_Control_Mode_SPIN,        // 小陀螺
    MiniPC_Chassis_Control_Mode_NORMAL_SPIN, // 不随动+受击打
    MiniPC_Chassis_Control_Mode_FOLLOW_SPIN, // 随动+受击打
};

/**
 * @brief 云台运动控制方式
 *
 */
enum Enum_MiniPC_Gimbal_Control_Mode : uint8_t
{
    MiniPC_Gimbal_Control_Mode_CRUISE = 0, // 巡航
    MiniPC_Gimbal_Control_Mode_AUTO,       // 非巡航
};

/**
 * @brief 红蓝方
 *
 */
enum Enum_MiniPC_Self_Color : uint8_t
{
    MiniPC_Self_Color_RED = 0,
    MiniPC_Self_Color_BLUE,
};

/**
 * @brief 迷你主机源数据
 *
 */
struct Struct_MiniPC_USB_Data
{
    uint8_t Frame_Header;
    uint8_t Data[50];
} __attribute__((packed));

enum Enum_Auto_aim_Status : uint8_t
{
    Auto_aim_Status_DISABLE = 0,
    Auto_aim_Status_ENABLE,
};

/**
 * @brief 下位机接收的规划数据
 *
 */
struct Struct_MiniPC_Rx_Data
{
    uint8_t header;                                        // 帧头
    int16_t Chassis_Angular_Velocity_Yaw;                    // 底盘转动的角速度, rad/s
    int16_t MiniPC_To_Chassis_Target_Velocity_X;             // 目标线速度 x
    int16_t MiniPC_To_Chassis_Target_Velocity_Y;             // 目标线速度 y
    int16_t Gimbal_Angular_Velocity_Yaw_Main;                     // 目标角速度 w
    int16_t Gimbal_Angular_Velocity_Yaw_A;
    int16_t Gimbal_Angular_Velocity_Pitch_A;                     // 目标角速度 p   
    float Gimbal_Target_X_A;                                 // 装甲板在云台坐标系的 x 坐标
    float Gimbal_Target_Y_A;                                 // 装甲板在云台坐标系的 y 坐标
    float Gimbal_Target_Z_A;                                 // 装甲板在云台坐标系的 z 坐标
    Enum_MiniPC_Chassis_Control_Mode Chassis_Control_Mode; // 底盘控制模式 随动/小陀螺
    Enum_Auto_aim_Status Auto_aim_Status_A;   // 云台控制模式 巡航/非巡航
    uint16_t crc16;
} __attribute__((packed));


/**
 * @brief 下位机发送的反馈数据
 *
 */
struct Struct_MiniPC_Tx_Data
{
    uint8_t header;                // 帧头
    float Gimbal_Now_Yaw_Angle_A;    // 当前云台yaw角度
    float Gimbal_Now_Yaw_Angle_Main;  // 当前云台pitch角度
    float Gimbal_Now_Pitch_Angle_A;  // 当前云台pitch角度
    float Gimbal_Now_Roll_Angle_A;   // 当前云台roll角度
    float Chassis_Now_yaw_Angle;   // 当前底盘yaw角度
    uint8_t Game_process;          // 比赛阶段
    uint16_t Self_blood;           // 自身hp
    uint16_t Self_Outpost_HP;      // 己方前哨战hp
    uint16_t Oppo_Outpost_HP;      // 对方前哨战hp
    uint16_t Self_Base_HP;         // 己方基地hp
    uint16_t Projectile_allowance; // 允许发弹量
    uint16_t Remaining_Time;       // 比赛剩余时间
    uint8_t Color_Invincible_State;      // 敌对方无敌状态/自身颜色
    uint16_t crc16;
} __attribute__((packed));



typedef __packed struct
{
    uint8_t game_process;
    uint16_t time;
    uint16_t self_blood;
    uint16_t self_outpost_HP;
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} can_rx1_t;

typedef __packed struct
{
    uint16_t self_base_HP;
    uint16_t oppo_outpost_HP;
    uint16_t projectile_allowance_17mm;
    uint8_t invincible_state;
    uint8_t outpost_rfid : 1;
    uint8_t color : 1;
} can_rx2_t;

typedef __packed struct // 0x0207 实时射击信息
{
    uint8_t bullet_type;         // 子弹类型
    uint8_t shooter_number;      // 发射机构ID
    uint8_t launching_frequency; // 射频
    float initial_speed;         // 弹速
} can_rx3_t;

struct Struct_MiniPC_Tx_Data_B{
    float Gimbal_Now_Yaw_Angle_B;
    float Gimbal_Now_Pitch_Angle_B;
    float Gimbal_Now_Roll_Angle_B;
    uint16_t crc16;
}__attribute__((packed));

struct Struct_MiniPC_Rx_Data_B{
    uint8_t header;
    int16_t Gimbal_Angular_Velocity_Yaw_B;
    int16_t Gimbal_Angular_Velocity_Pitch_B;
    float Gimbal_Target_X_B;
    float Gimbal_Target_Y_B;
    float Gimbal_Target_Z_B;
    Enum_Auto_aim_Status Auto_aim_Status_B;
    uint16_t crc16;
}__attribute__((packed));

extern Struct_MiniPC_Tx_Data_B MiniPC_Tx_Data_B;
extern Struct_MiniPC_Rx_Data_B MiniPC_Rx_Data_B;


class Class_MiniPC
{
public:
    void Init(Struct_USB_Manage_Object* __MiniPC_USB_Manage_Object, uint8_t __frame_header = 0x5A, uint8_t __frame_rear = 0x01);

    inline Enum_MiniPC_Status Get_MiniPC_Status();
    inline float Get_Chassis_Target_Velocity_X();
    inline float Get_Chassis_Target_Velocity_Y();
    inline float Get_Gimbal_Angular_Velocity_Yaw_Main();
    inline float Get_Gimbal_Angular_Velocity_Yaw_A();
    inline float Get_Gimbal_Angular_Velocity_Pitch_A();
    inline float Get_Gimbal_Angular_Velocity_Yaw_B();
    inline float Get_Gimbal_Angular_Velocity_Pitch_B();
    inline float Get_Gimbal_Target_Y_A();
    inline float Get_Gimbal_Target_X_A();
    inline float Get_Gimbal_Target_Z_A();
    inline float Get_Gimbal_Target_Y_B();
    inline float Get_Gimbal_Target_X_B();
    inline float Get_Gimbal_Target_Z_B();
    inline float Get_Gimbal_Error_A();
    inline float Get_Gimbal_Error_B();
    inline float Get_Booster_Frequency();
    inline float Get_Rx_Pitch_Angle_A();
    inline float Get_Rx_Yaw_Angle_A();
    inline float Get_Rx_Pitch_Angle_B();
    inline float Get_Rx_Yaw_Angle_B();
    inline float Get_Now_Yaw_Angle_A();
    inline float Get_Now_Pitch_Angle_A();
    inline float Get_Now_Roll_Angle_A();
    inline float Get_Now_Yaw_Angle_B();
    inline float Get_Now_Pitch_Angle_B();
    inline float Get_Now_Roll_Angle_B();
    inline float Get_Now_Relative_Angle();

    inline uint8_t Get_Target_Invincible_State();
    inline Enum_MiniPC_Chassis_Control_Mode Get_Chassis_Control_Mode();
    inline Enum_MiniPC_Gimbal_Control_Mode Get_Gimbal_Control_Mode();
    inline Enum_Auto_aim_Status Get_Auto_aim_Status_A();
    inline Enum_Auto_aim_Status Get_Auto_aim_Status_B();

    inline void Set_Game_Stage(Enum_MiniPC_Game_Stage __Game_Stage);
    //inline void Set_Chassis_Now_Velocity_X(float __Chassis_Now_Velocity_X);
    inline void Set_Chassis_Now_Velocity_Y(float __Chassis_Now_Velocity_Y);
    inline void Set_Chassis_Now_Omega(float __Chassis_Now_Omega);
    inline void Set_Gimbal_Now_Yaw_Angle_A(float __Gimbal_Now_Yaw_Angle);
    inline void Set_Gimbal_Now_Yaw_Omega_A(float __Gimbal_Now_Yaw_Omega);
    inline void Set_Gimbal_Now_Pitch_Angle_A(float __Gimbal_Now_Pitch_Angle);
    inline void Set_Gimbal_Now_Roll_Angle_A(float __Gimbal_Now_Roll_Angle);
    inline void Set_Gimbal_Now_Yaw_Angle_B(float __Gimbal_Now_Yaw_Angle);
    inline void Set_Gimbal_Now_Pitch_Angle_B(float __Gimbal_Now_Pitch_Angle);
    inline void Set_Gimbal_Now_Roll_Angle_B(float __Gimbal_Now_Roll_Angle);
    inline void Set_Gimbal_Now_Omega_B(float __Gimbal_Now_Omega);
    inline void Set_Gimbal_Now_Relative_Angle(float __Gimbal_Now_Relative_Angle);
    inline void Set_Armor_Attacked_ID(uint8_t __Armor_Attacked_ID);
    inline void Set_Armor_Attacked_Ammo_Status(Enum_MiniPC_Data_Status __Armor_Attacked_Ammo_Status);
    inline void Set_Self_Color(Enum_MiniPC_Self_Color __Self_Color);
    inline void Set_Outpost_Status(Enum_MiniPC_Data_Status __Outpost_Status);
    inline void Set_Auto_aim_Status_A(Enum_Auto_aim_Status __Auto_aim_Status);
    inline void Set_Auto_aim_Status_B(Enum_Auto_aim_Status __Auto_aim_Status);

    void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
    bool Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
    uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);

    inline void Transform_Angle_Tx();
    inline void Transform_Angle_Rx();

    float calc_yaw(float x, float y, float z);
    float calc_distance(float x, float y, float z) ;
    float calc_pitch(float x, float y, float z) ;
    float Calc_Error(float x, float y, float z, float now_yaw, float now_pitch);
    void Auto_aim(float x,float y,float z,float *yaw,float *pitch,float *distance);

    float meanFilter(float input);

    void USB_RxCpltCallback(uint8_t *Rx_Data);
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void TIM_Write_PeriodElapsedCallback();

    Class_IMU *IMU;
    Class_Referee *Referee;
    Class_Gimbal_Pitch_Motor_GM6020 *Gimbal_Pitch_Motor_GM6020_A;
    Class_Gimbal_Pitch_Motor_GM6020 *Gimbal_Pitch_Motor_GM6020_B;
    Class_Gimbal_Yaw_Motor_GM6020 *Gimbal_Yaw_Motor_GM6020_Main;
    Class_Gimbal_Yaw_Motor_GM6020 *Gimbal_Yaw_Motor_GM6020_A;
    Class_Gimbal_Yaw_Motor_GM6020 *Gimbal_Yaw_Motor_GM6020_B;

protected:
    //初始化相关常量
    

    //绑定的USB
    Struct_USB_Manage_Object *USB_Manage_Object;
    //数据包头标
    uint8_t Frame_Header;
    //数据包尾标
    uint8_t Frame_Rear; 

    //常量
    
    //内部变量

    //当前时刻的迷你主机接收flag
    uint32_t Flag = 0;
    //前一时刻的迷你主机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    //迷你主机对外接口信息
    Struct_MiniPC_Rx_Data Data_NUC_To_MCU;

	float Now_Angle_Roll_A;
    float Now_Angle_Pitch_A;
    float Now_Angle_Yaw_A;

    float Now_Angle_Roll_B;
    float Now_Angle_Pitch_B;
    float Now_Angle_Yaw_B;

    float Now_Angle_Relative;

	float Rx_Angle_Roll_A;
	float Rx_Angle_Pitch_A;
	float Rx_Angle_Yaw_A;

	float Rx_Angle_Roll_B;
	float Rx_Angle_Pitch_B;
	float Rx_Angle_Yaw_B;


    const float g = 9.8; // 重力加速度
    const float bullet_v = 28.0; // 子弹速度  

    // 距离
    float Distance_A;
    float Distance_B;
    float Error_A;
    float Error_B;


    //写变量

    //迷你主机对外接口信息
    Struct_MiniPC_Tx_Data Data_MCU_To_NUC;

    //读写变量

    //内部函数

    void Data_Process();
    void Output();
};
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


float Class_MiniPC::Get_Now_Relative_Angle()
{
    return (Now_Angle_Relative);
}

float Class_MiniPC::Get_Rx_Pitch_Angle_A()
{
    return (Rx_Angle_Pitch_A);
}

float Class_MiniPC::Get_Rx_Yaw_Angle_A()
{
    return (Rx_Angle_Yaw_A);
}

float Class_MiniPC::Get_Rx_Pitch_Angle_B()
{
    return (Rx_Angle_Pitch_B);
}

float Class_MiniPC::Get_Rx_Yaw_Angle_B()    
{
    return (Rx_Angle_Yaw_B);
}

/**
 * @brief 获取迷你主机状态
 *
 * @return Enum_MiniPC_Status 迷你主机状态
 */
Enum_MiniPC_Status Class_MiniPC::Get_MiniPC_Status()
{
    return (MiniPC_Status);
}

/**
 * @brief 获取底盘目标速度x
 *
 * @return float 底盘目标速度x
 */
float Class_MiniPC::Get_Chassis_Target_Velocity_X()
{
    return (Data_NUC_To_MCU.MiniPC_To_Chassis_Target_Velocity_X);
}

/**
 * @brief 获取底盘目标速度y
 *
 * @return float 底盘目标速度y
 */
float Class_MiniPC::Get_Chassis_Target_Velocity_Y()
{
    return (Data_NUC_To_MCU.MiniPC_To_Chassis_Target_Velocity_Y);
}

/**
 * @brief 获取底盘目标速度omega
 *
 * @return float 获取底盘目标速度omega
 */
float Class_MiniPC::Get_Gimbal_Angular_Velocity_Yaw_Main()
{
    return (Data_NUC_To_MCU.Gimbal_Angular_Velocity_Yaw_Main);
}

/**
 * @brief 获取X
 *
 * @return float X
 */
float Class_MiniPC::Get_Gimbal_Target_X_A()
{
    return (Data_NUC_To_MCU.Gimbal_Target_X_A);
}

/**
 * @brief 获取Y
 *
 * @return float Y
 */
float Class_MiniPC::Get_Gimbal_Target_Y_A()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Y_A);
}

/**
 * @brief 获取Z
 *
 * @return float Z
 */
float Class_MiniPC::Get_Gimbal_Target_Z_A()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Z_A);
}

float Class_MiniPC::Get_Gimbal_Error_A()
{
    return (Error_A);
}
float Class_MiniPC::Get_Gimbal_Error_B()
{
    return (Error_B);
}
/**
 * @brief 获取底盘移动控制模式
 *
 * @return Enum_MiniPC_Chassis_Control_Mode 移动控制模式
 */
Enum_MiniPC_Chassis_Control_Mode Class_MiniPC::Get_Chassis_Control_Mode()
{
    return (Data_NUC_To_MCU.Chassis_Control_Mode);
}

// /**
//  * @brief 获取云台控制模式
//  *
//  * @return Enum_MiniPC_Gimbal_Control_Mode 移动控制模式
//  */
// Enum_MiniPC_Gimbal_Control_Mode Class_MiniPC::Get_Gimbal_Control_Mode()
// {
//     return (Data_NUC_To_MCU.Gimbal_Control_Mode);
// }

Enum_Auto_aim_Status Class_MiniPC::Get_Auto_aim_Status_A()
{
    return (Data_NUC_To_MCU.Auto_aim_Status_A);
}
// /**
//  * @brief 判断目标是否处于无敌状态
//  *
//  * @return uint8_t 否：0，是：1
//  */
// extern can_rx2_t can_rx2;
// uint8_t Class_MiniPC::Get_Target_Invincible_State()
// {
//     return (can_rx2.invincible_state >> 0 & 0x01);
// }

/**
 * @brief 设定云台当前角度yaw
 *
 * @param __Gimbal_Now_Yaw_Angle 云台当前角度yaw
 */
void Class_MiniPC::Set_Gimbal_Now_Yaw_Angle_A(float __Gimbal_Now_Yaw_Angle)
{
    Now_Angle_Yaw_A = __Gimbal_Now_Yaw_Angle;
}

/**
 * @brief 设定云台当前角度pitch
 *
 * @param __Gimbal_Now_Pitch_Angle 云台当前角度pitch
 */
void Class_MiniPC::Set_Gimbal_Now_Pitch_Angle_A(float __Gimbal_Now_Pitch_Angle)
{
    Now_Angle_Pitch_A = __Gimbal_Now_Pitch_Angle;
}

/**
 * @brief 设定云台当前角度roll
 *
 * @param __Gimbal_Now_Roll_Angle 云台当前角度roll
 */
void Class_MiniPC::Set_Gimbal_Now_Roll_Angle_A(float __Gimbal_Now_Roll_Angle)
{
    Now_Angle_Roll_A = __Gimbal_Now_Roll_Angle;
}


/**
 * @brief 设定云台当前角度roll
 *
 * @param __Gimbal_Now_Roll_Angle 云台当前角度roll
 */
void Class_MiniPC::Set_Gimbal_Now_Relative_Angle(float __Gimbal_Now_Relative_Angle)
{
    Now_Angle_Relative = __Gimbal_Now_Relative_Angle;
}


// /**
//  * @brief 发送数据处理
//  *
//  */
// void Class_MiniPC::Transform_Angle_Tx()
// {
//     Tx_Angle_Pitch = -IMU->Get_Angle_Pitch();
//     Tx_Angle_Roll = IMU->Get_Angle_Roll();
//     Tx_Angle_Yaw = IMU->Get_Angle_Yaw();
// }

// /**
//  * @brief 接收数据处理
//  *
//  */
// void Class_MiniPC::Transform_Angle_Rx()
// {
//     Rx_Angle_Pitch = Pack_Rx.target_pitch;
//     Rx_Angle_Yaw = Pack_Rx.target_yaw;
// }

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
