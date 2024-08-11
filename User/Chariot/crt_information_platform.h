#ifndef CRT_INFORMATION_PLATFORM_H_
#define CRT_INFORMATION_PLATFORM_H_

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"




/* Exported macros -----------------------------------------------------------*/
#define CHASSIS_DATA_LENGTH 6
#define GIMBAL_DATA_LENGTH 8

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘通讯状态
 *
 */
enum Enum_Chassis_Status
{
    Chassis_Status_DISABLE = 0,
    Chassis_Status_ENABLE,
};

/**
 * @brief 云台通讯状态
 *
 */
enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_ENABLE,
};

typedef struct Struct_Chassis_Data
{
    uint8_t Robot_Id;
    uint8_t Game_Stage;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Shooter_Barrel_Cooling_Value;

};

union Union_Chassis_Data
{
    Struct_Chassis_Data chassis_data_s;
    uint8_t chassis_data[CHASSIS_DATA_LENGTH];
};

#include <cstdint>

typedef struct Struct_Gimbal_Data
{
    uint16_t chassis_velocity_x;
    uint16_t chassis_velocity_y;
    uint8_t chassis_omega;
    uint16_t gimbal_pitch_angle;
        struct 
        {
            uint8_t Referee_UI_Refresh_Status:1;
            uint8_t MiniPC_Status:1;
            uint8_t MiniPC_Aim_Status:1;
            uint8_t Fric_Status:1;
            uint8_t Bulletcap_Status:1;
            uint8_t Sprint_Status:1;
            uint8_t Chassis_Control_Type:2;
        };

} Struct_Gimbal_Data;


union Union_Gimbal_Data
{
    Struct_Gimbal_Data gimbal_data_s;
    uint8_t gimbal_data[GIMBAL_DATA_LENGTH];
};


class Class_Chassis_Platform
{
public:

    Union_Chassis_Data chassis_tx_part;

    Union_Gimbal_Data gimbal_rx_part;

protected:
 
};

class Class_Gimbal_Platform
{
public:

    Union_Chassis_Data chassis_rx_part;

    Union_Gimbal_Data gimbal_tx_part;
 
protected:
 
};

class Class_Information_Platform
{
public:

    float Velocity_X_Max;
    //速度Y限制
    float Velocity_Y_Max;
    //角速度限制
    float Omega_Max=8.0f;
    //底盘小陀螺模式角速度
    float Spin_Omega = 4.0f;
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;

    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Spin_Omega();

    #ifdef CHASSIS
    Class_Chassis_Platform chassis_platform;

    float Gimbal_Tx_Pitch_Angle = 0;
    uint32_t Gimbal_Alive_Flag = 0;
    uint32_t Pre_Gimbal_Alive_Flag = 0;
    Enum_Gimbal_Status Gimbal_Status =  Gimbal_Status_DISABLE;

    inline void Set_Robot_ID(uint8_t id);
    inline void Set_Game_Stage(uint8_t stage);
    inline void Set_Shooter_Barrel_Heat_Limit(uint16_t limit);
    inline void Set_Shooter_Barrel_Cooling_Value(uint16_t value);
    
    inline uint8_t Get_Robot_ID(void);
    inline uint8_t Get_Game_Stage(void);
    inline uint16_t Get_Shooter_Barrel_Heat_Limit(void);
    inline uint16_t Get_Shooter_Barrel_Cooling_Value(void);

    inline void Set_Chassis_Velocity_X(uint16_t velocity_x);
    inline void Set_Chassis_Velocity_Y(uint16_t velocity_y);
    inline void Set_Chassis_Omega(uint8_t omega);
    inline void Set_Gimbal_Pitch_Angle(uint16_t angle);
    inline void Set_Referee_UI_Refresh_Status(uint8_t status);
    inline void Set_MiniPC_Status(uint8_t status);
    inline void Set_MiniPC_Aim_Status(uint8_t status);
    inline void Set_Fric_Status(uint8_t status);
    inline void Set_Bulletcap_Status(uint8_t status);
    inline void Set_Sprint_Status(uint8_t status);
    inline void Set_Chassis_Control_Type(uint8_t type);

    inline uint16_t Get_Chassis_Velocity_X(void);
    inline uint16_t Get_Chassis_Velocity_Y(void);
    inline uint8_t Get_Chassis_Omega(void);
    inline uint16_t Get_Gimbal_Pitch_Angle(void);
    inline uint8_t Get_Referee_UI_Refresh_Status(void);
    inline uint8_t Get_MiniPC_Status(void);
    inline uint8_t Get_MiniPC_Aim_Status(void);
    inline uint8_t Get_Fric_Status(void);
    inline uint8_t Get_Bulletcap_Status(void);
    inline uint8_t Get_Sprint_Status(void);
    inline uint8_t Get_Chassis_Control_Type(void);
    #endif

    #ifdef GIMBAL
    Class_Gimbal_Platform gimbal_platform;

    uint32_t Chassis_Alive_Flag = 0;
    uint32_t Pre_Chassis_Alive_Flag = 0;
    Enum_Chassis_Status Chassis_Status = Chassis_Status_DISABLE;

    inline void Set_Robot_ID(uint8_t id);
    inline void Set_Game_Stage(uint8_t stage);
    inline void Set_Shooter_Barrel_Heat_Limit(uint16_t limit);
    inline void Set_Shooter_Barrel_Cooling_Value(uint16_t value);
    
    inline uint8_t Get_Robot_ID(void);
    inline uint8_t Get_Game_Stage(void);
    inline uint16_t Get_Shooter_Barrel_Heat_Limit(void);
    inline uint16_t Get_Shooter_Barrel_Cooling_Value(void);

    inline void Set_Chassis_Velocity_X(uint16_t velocity_x);
    inline void Set_Chassis_Velocity_Y(uint16_t velocity_y);
    inline void Set_Chassis_Omega(uint8_t omega);
    inline void Set_Gimbal_Pitch_Angle(uint16_t angle);
    inline void Set_Referee_UI_Refresh_Status(uint8_t status);
    inline void Set_MiniPC_Status(uint8_t status);
    inline void Set_MiniPC_Aim_Status(uint8_t status);
    inline void Set_Fric_Status(uint8_t status);
    inline void Set_Bulletcap_Status(uint8_t status);
    inline void Set_Sprint_Status(uint8_t status);
    inline void Set_Chassis_Control_Type(uint8_t type);

    inline uint16_t Get_Chassis_Velocity_X(void);
    inline uint16_t Get_Chassis_Velocity_Y(void);
    inline uint8_t Get_Chassis_Omega(void);
    inline uint16_t Get_Gimbal_Pitch_Angle(void);
    inline uint8_t Get_Referee_UI_Refresh_Status(void);
    inline uint8_t Get_MiniPC_Status(void);
    inline uint8_t Get_MiniPC_Aim_Status(void);
    inline uint8_t Get_Fric_Status(void);
    inline uint8_t Get_Bulletcap_Status(void);
    inline uint8_t Get_Sprint_Status(void);
    inline uint8_t Get_Chassis_Control_Type(void);

    #endif
    void Init(Struct_CAN_Manage_Object* __CAN_Manage_Object );
    void Platform_Tx_Callback();
    void Platform_Rx_Callback();
    void Platform_Alive_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object ;

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Information_Platform::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Information_Platform::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}
/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Information_Platform::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}


/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Information_Platform::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Information_Platform::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}
/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Information_Platform::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Information_Platform::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Information_Platform::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Information_Platform::Get_Spin_Omega()
{
    return (Spin_Omega);
}

#ifdef CHASSIS


uint8_t Class_Information_Platform::Get_Robot_ID(void)
{
    return (chassis_platform.chassis_tx_part.chassis_data_s.Robot_Id); 
}
uint8_t Class_Information_Platform::Get_Game_Stage(void)
{
    return (chassis_platform.chassis_tx_part.chassis_data_s.Game_Stage);
}
uint16_t Class_Information_Platform::Get_Shooter_Barrel_Heat_Limit(void)
{
    return (chassis_platform.chassis_tx_part.chassis_data_s.Shooter_Barrel_Heat_Limit);
}
uint16_t Class_Information_Platform::Get_Shooter_Barrel_Cooling_Value(void)
{
    return (chassis_platform.chassis_tx_part.chassis_data_s.Shooter_Barrel_Cooling_Value);
}

void Class_Information_Platform::Set_Robot_ID(uint8_t id)
{
    chassis_platform.chassis_tx_part.chassis_data_s.Robot_Id = id;
}
void Class_Information_Platform::Set_Game_Stage(uint8_t stage)
{
    chassis_platform.chassis_tx_part.chassis_data_s.Game_Stage = stage;
}
void Class_Information_Platform::Set_Shooter_Barrel_Heat_Limit(uint16_t limit)
{
    chassis_platform.chassis_tx_part.chassis_data_s.Shooter_Barrel_Heat_Limit = limit;
}
void Class_Information_Platform::Set_Shooter_Barrel_Cooling_Value(uint16_t value)
{
    chassis_platform.chassis_tx_part.chassis_data_s.Shooter_Barrel_Cooling_Value = value;
}

uint16_t Class_Information_Platform::Get_Chassis_Velocity_X(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.chassis_velocity_x);
}

uint16_t Class_Information_Platform::Get_Chassis_Velocity_Y(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.chassis_velocity_y);
}

uint8_t Class_Information_Platform::Get_Chassis_Omega(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.chassis_omega);
}
uint16_t Class_Information_Platform::Get_Gimbal_Pitch_Angle(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.gimbal_pitch_angle);
}

uint8_t Class_Information_Platform::Get_Referee_UI_Refresh_Status(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.Referee_UI_Refresh_Status);
}

uint8_t Class_Information_Platform::Get_MiniPC_Status(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.MiniPC_Status);
}

uint8_t Class_Information_Platform::Get_MiniPC_Aim_Status(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.MiniPC_Aim_Status);
}
uint8_t Class_Information_Platform::Get_Fric_Status(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.Fric_Status);
}

uint8_t Class_Information_Platform::Get_Bulletcap_Status(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.Bulletcap_Status);
}
uint8_t Class_Information_Platform::Get_Sprint_Status(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.Sprint_Status);
}

uint8_t Class_Information_Platform::Get_Chassis_Control_Type(void)
{
    return (chassis_platform.gimbal_rx_part.gimbal_data_s.Chassis_Control_Type);
}

void Class_Information_Platform::Set_Chassis_Velocity_X(uint16_t velocity_x)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.chassis_velocity_x = velocity_x;
}
void Class_Information_Platform::Set_Referee_UI_Refresh_Status(uint8_t status)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.Referee_UI_Refresh_Status = status;
}
void Class_Information_Platform::Set_Chassis_Velocity_Y(uint16_t velocity_y)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.chassis_velocity_y = velocity_y;
}
void Class_Information_Platform::Set_Gimbal_Pitch_Angle(uint16_t angle)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.gimbal_pitch_angle=angle;
}
void Class_Information_Platform::Set_Chassis_Omega(uint8_t omega)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.chassis_omega = omega;
}

void Class_Information_Platform::Set_MiniPC_Status(uint8_t status)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.MiniPC_Status = status;
}
void Class_Information_Platform::Set_Fric_Status(uint8_t status)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.Fric_Status = status;
}
void Class_Information_Platform::Set_Bulletcap_Status(uint8_t status)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.Bulletcap_Status = status;
}
void Class_Information_Platform::Set_Sprint_Status(uint8_t status)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.Sprint_Status = status;
}
void Class_Information_Platform::Set_Chassis_Control_Type(uint8_t type)
{
    chassis_platform.gimbal_rx_part.gimbal_data_s.Chassis_Control_Type = type;
}
#endif



#ifdef GIMBAL

uint8_t Class_Information_Platform::Get_Robot_ID(void)
{
    return (gimbal_platform.chassis_rx_part.chassis_data_s.Robot_Id); 
}
uint8_t Class_Information_Platform::Get_Game_Stage(void)
{
    return (gimbal_platform.chassis_rx_part.chassis_data_s.Game_Stage);
}
uint16_t Class_Information_Platform::Get_Shooter_Barrel_Heat_Limit(void)
{
    return (gimbal_platform.chassis_rx_part.chassis_data_s.Shooter_Barrel_Heat_Limit);
}
uint16_t Class_Information_Platform::Get_Shooter_Barrel_Cooling_Value(void)
{
    return (gimbal_platform.chassis_rx_part.chassis_data_s.Shooter_Barrel_Cooling_Value);
}

void Class_Information_Platform::Set_Robot_ID(uint8_t id)
{
    gimbal_platform.chassis_rx_part.chassis_data_s.Robot_Id = id;
}
void Class_Information_Platform::Set_Game_Stage(uint8_t stage)
{
    gimbal_platform.chassis_rx_part.chassis_data_s.Game_Stage = stage;
}
void Class_Information_Platform::Set_Shooter_Barrel_Heat_Limit(uint16_t limit)
{
    gimbal_platform.chassis_rx_part.chassis_data_s.Shooter_Barrel_Heat_Limit = limit;
}
void Class_Information_Platform::Set_Shooter_Barrel_Cooling_Value(uint16_t value)
{
    gimbal_platform.chassis_rx_part.chassis_data_s.Shooter_Barrel_Cooling_Value = value;
}

uint16_t Class_Information_Platform::Get_Chassis_Velocity_X(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.chassis_velocity_x);
}

uint16_t Class_Information_Platform::Get_Chassis_Velocity_Y(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.chassis_velocity_y);
}

uint8_t Class_Information_Platform::Get_Chassis_Omega(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.chassis_omega);
}
uint16_t Class_Information_Platform::Get_Gimbal_Pitch_Angle(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.gimbal_pitch_angle);
}

uint8_t Class_Information_Platform::Get_Referee_UI_Refresh_Status(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.Referee_UI_Refresh_Status);
}

uint8_t Class_Information_Platform::Get_MiniPC_Status(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.MiniPC_Status);
}

uint8_t Class_Information_Platform::Get_MiniPC_Aim_Status(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.MiniPC_Aim_Status);
}
uint8_t Class_Information_Platform::Get_Fric_Status(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.Fric_Status);
}

uint8_t Class_Information_Platform::Get_Bulletcap_Status(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.Bulletcap_Status);
}
uint8_t Class_Information_Platform::Get_Sprint_Status(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.Sprint_Status);
}

uint8_t Class_Information_Platform::Get_Chassis_Control_Type(void)
{
    return (gimbal_platform.gimbal_tx_part.gimbal_data_s.Chassis_Control_Type);
}
void Class_Information_Platform::Set_Referee_UI_Refresh_Status(uint8_t status)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.Referee_UI_Refresh_Status = status;
}
void Class_Information_Platform::Set_Chassis_Velocity_X(uint16_t velocity_x)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.chassis_velocity_x = velocity_x;
}

void Class_Information_Platform::Set_Chassis_Velocity_Y(uint16_t velocity_y)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.chassis_velocity_y = velocity_y;
}
void Class_Information_Platform::Set_Chassis_Omega(uint8_t omega)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.chassis_omega = omega;
}
void Class_Information_Platform::Set_Gimbal_Pitch_Angle(uint16_t angle)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.gimbal_pitch_angle=angle;
}
void Class_Information_Platform::Set_MiniPC_Status(uint8_t status)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.MiniPC_Status = status;
}
void Class_Information_Platform::Set_Fric_Status(uint8_t status)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.Fric_Status = status;
}
void Class_Information_Platform::Set_Bulletcap_Status(uint8_t status)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.Bulletcap_Status = status;
}
void Class_Information_Platform::Set_Sprint_Status(uint8_t status)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.Sprint_Status = status;
}
void Class_Information_Platform::Set_Chassis_Control_Type(uint8_t type)
{
    gimbal_platform.gimbal_tx_part.gimbal_data_s.Chassis_Control_Type = type;
}
#endif



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

#endif