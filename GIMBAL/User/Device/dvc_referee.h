/**
 * @file dvc_referee.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_REFEREE_H
#define DVC_REFEREE_H

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "limits.h"
#include "string.h"

/* Exported macros -----------------------------------------------------------*/

//设定当前车车
// #define Robot_HERO_1
// #define Robot_ENGINEER_2
 #define Robot_INFANTRY_3
// #define Robot_INFANTRY_4
// #define Robot_INFANTRY_5
// #define Robot_AERIAL_6
//#define Robot_SENTRY_7
// #define Robot_DART_8
// #define Robot_RADAR_9
// #define Robot_BASE_10
// #define Robot_OUTPOST_11

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统状态
 *
 */
enum Enum_Referee_Status
{
    Referee_Status_DISABLE = 0,
    Referee_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum Enum_Referee_Data_Status : uint8_t
{
    Referee_Data_Status_DISABLE = 0,
    Referee_Data_Status_ENABLE,
};

/**
 * @brief 裁判系统命令码类型
 *
 */
enum Enum_Referee_Command_ID : uint16_t
{
    Referee_Command_ID_GAME_STATUS = 0x0001,
    Referee_Command_ID_GAME_RESULT,
    Referee_Command_ID_GAME_ROBOT_HP,
    Referee_Command_ID_GAME_DART_STATUS_DISABLED,
    Referee_Command_ID_GAME_AI_DISABLED,
    Referee_Command_ID_EVENT_DATA = 0x0101,
    Referee_Command_ID_EVENT_SUPPLY,
    Referee_Command_ID_EVENT_SUPPLY_REQUEST_DISABLED,
    Referee_Command_ID_EVENT_REFEREE_WARNING,
    Referee_Command_ID_EVENT_DART_REMAINING_TIME,
    Referee_Command_ID_ROBOT_STATUS = 0x0201,
    Referee_Command_ID_ROBOT_POWER_HEAT,
    Referee_Command_ID_ROBOT_POSITION,
    Referee_Command_ID_ROBOT_BUFF,
    Referee_Command_ID_ROBOT_AERIAL_ENERGY,
    Referee_Command_ID_ROBOT_DAMAGE,
    Referee_Command_ID_ROBOT_BOOSTER,
    Referee_Command_ID_ROBOT_REMAINING_AMMO,
    Referee_Command_ID_ROBOT_RFID,
    Referee_Command_ID_ROBOT_DART_COMMAND,
    Referee_Command_ID_INTERACTION = 0x0301,
    Referee_Command_ID_INTERACTION_CUSTOM_CONTROLLER,
    Referee_Command_ID_INTERACTION_RADAR_SEND,
    Referee_Command_ID_INTERACTION_REMOTE_CONTROL,
    Referee_Command_ID_INTERACTION_Client_RECEIVE,
};

/**
 * @brief 通用单方机器人ID
 *
 */
enum Enum_Referee_Data_Robot_ID : uint8_t
{
    Referee_Data_Robot_ID_NULL = 0,
    Referee_Data_Robot_ID_HERO_1,
    Referee_Data_Robot_ID_ENGINEER_2,
    Referee_Data_Robot_ID_INFANTRY_3,
    Referee_Data_Robot_ID_INFANTRY_4,
    Referee_Data_Robot_ID_INFANTRY_5,
    Referee_Data_Robot_ID_AERIAL_6,
    Referee_Data_Robot_ID_SENTRY_7,
    Referee_Data_Robot_ID_DART_8,
    Referee_Data_Robot_ID_RADAR_9,
    Referee_Data_Robot_ID_BASE_10,
    Referee_Data_Robot_ID_OUTPOST_11,
};

/**
 * @brief 通用双方机器人ID
 *
 */
enum Enum_Referee_Data_Robots_ID : uint8_t
{
    Referee_Data_Robots_ID_NO = 0,
    Referee_Data_Robots_ID_RED_HERO_1,
    Referee_Data_Robots_ID_RED_ENGINEER_2,
    Referee_Data_Robots_ID_RED_INFANTRY_3,
    Referee_Data_Robots_ID_RED_INFANTRY_4,
    Referee_Data_Robots_ID_RED_INFANTRY_5,
    Referee_Data_Robots_ID_RED_AERIAL_6,
    Referee_Data_Robots_ID_RED_SENTRY_7,
    Referee_Data_Robots_ID_RED_DART_8,
    Referee_Data_Robots_ID_RED_RADAR_9,
    Referee_Data_Robots_ID_RED_BASE_10,
    Referee_Data_Robots_ID_RED_OUTPOST_11,
    Referee_Data_Robots_ID_BLUE_HERO_1 = 101,
    Referee_Data_Robots_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_ID_BLUE_AERIAL_6,
    Referee_Data_Robots_ID_BLUE_SENTRY_7,
    Referee_Data_Robots_ID_BLUE_DART_8,
    Referee_Data_Robots_ID_BLUE_RADAR_9,
    Referee_Data_Robots_ID_BLUE_BASE_10,
    Referee_Data_Robots_ID_BLUE_OUTPOST_11,
};
/**
 * @brief 通用双方机器人ID
 *
 */
enum Enum_Referee_Data_Robots_Client_ID : uint16_t
{
    Referee_Data_Robots_Client_ID_NO = 0,
    Referee_Data_Robots_Client_ID_RED_HERO_1 = 0x0101,
    Referee_Data_Robots_Client_ID_RED_ENGINEER_2,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_3,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_4,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_5,
    Referee_Data_Robots_Client_ID_RED_AERIAL_6,
    Referee_Data_Robots_Client_ID_BLUE_HERO_1 = 0x0165,
    Referee_Data_Robots_Client_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_Client_ID_BLUE_AERIAL_6,
};

/**
 * @brief 比赛类型
 *
 */
enum Enum_Referee_Game_Status_Type
{
    Referee_Game_Status_Type_RMUC = 1,
    Referee_Game_Status_Type_SINGLE,
    Referee_Game_Status_Type_ICRA,
    Referee_Game_Status_Type_RMUL_3V3,
    Referee_Game_Status_Type_RMUL_1V1,
};

/**
 * @brief 比赛阶段
 *
 */
enum Enum_Referee_Game_Status_Stage
{
    Referee_Game_Status_Stage_NOT_STARTED = 0,
    Referee_Game_Status_Stage_READY,
    Referee_Game_Status_Stage_SELF_TESTING,
    Referee_Game_Status_Stage_5S_COUNTDOWN,
    Referee_Game_Status_Stage_BATTLE,
    Referee_Game_Status_Stage_SETTLEMENT,
};

/**
 * @brief 比赛结果
 *
 */
enum Enum_Referee_Game_Result : uint8_t
{
    Referee_Game_Result_DRAW = 0,
    Referee_Game_Result_RED_WIN,
    Referee_Game_Result_BLUE_WIN,
};

/**
 * @brief 补给站状态
 *
 */
enum Enum_Referee_Data_Event_Supply_Status : uint8_t
{
    Referee_Data_Event_Supply_Status_CLOSED = 0,
    Referee_Data_Event_Supply_Status_READY,
    Referee_Data_Event_Supply_Status_DROPPING,
};

/**
 * @brief 补给站提供子弹数量
 *
 */
enum Enum_Referee_Data_Event_Supply_Ammo_Number : uint8_t
{
    Referee_Data_Event_Supply_Ammo_Number_50 = 50,
    Referee_Data_Event_Supply_Ammo_Number_100 = 100,
    Referee_Data_Event_Supply_Ammo_Number_150 = 150,
    Referee_Data_Event_Supply_Ammo_Number_200 = 200,
};

/**
 * @brief 裁判警告等级
 *
 */
enum Enum_Referee_Data_Event_Referee_Warning_Level : uint8_t
{
    Referee_Data_Referee_Warning_Level_YELLOW = 1,
    Referee_Data_Referee_Warning_Level_RED,
    Referee_Data_Referee_Warning_Level_FAIL,
};

/**
 * @brief 伤害类型
 *
 */
enum Enum_Referee_Data_Event_Robot_Damage_Type
{
    Referee_Data_Robot_Damage_Type_ARMOR_ATTACKED = 0,
    Referee_Data_Robot_Damage_Type_MODULE_OFFLINE,
    Referee_Data_Robot_Damage_Type_BOOSTER_SPEED,
    Referee_Data_Robot_Damage_Type_BOOSTER_HEAT,
    Referee_Data_Robot_Damage_Type_CHASSIS_POWER,
    Referee_Data_Robot_Damage_Type_ARMOR_COLLISION,
};

/**
 * @brief 子弹类型
 *
 */
enum Enum_Referee_Data_Robot_Ammo_Type : uint8_t
{
    Referee_Data_Robot_Ammo_Type_BOOSTER_17MM = 1,
    Referee_Data_Robot_Ammo_Type_BOOSTER_42mm,
};

/**
 * @brief 发射机构类型
 *
 */
enum Enum_Referee_Data_Robot_Booster_Type : uint8_t
{
    Referee_Data_Robot_Booster_Type_BOOSTER_17MM_1 = 1,
    Referee_Data_Robot_Booster_Type_BOOSTER_17MM_2,
    Referee_Data_Robot_Booster_Type_BOOSTER_42mm,
};

/**
 * @brief 飞镖发射口状态
 *
 */
enum Enum_Referee_Data_Robot_Dart_Command_Status : uint8_t
{
    Referee_Data_Robot_Dart_Command_Status_OPEN = 0,
    Referee_Data_Robot_Dart_Command_Status_CLOSED,
    Referee_Data_Robot_Dart_Command_Status_EXECUTING,
};

/**
 * @brief 飞镖击打目标
 *
 */
enum Enum_Referee_Data_Robot_Dart_Command_Target : uint8_t
{
    Referee_Data_Robot_Dart_Command_Target_OUTPOST = 0,
    Referee_Data_Robot_Dart_Command_Target_BASE,
};

/**
 * @brief 图形操作交互信息
 *
 */
enum Enum_Referee_Data_Interaction_Layer_Delete_Operation : uint8_t
{
    Referee_Data_Interaction_Layer_Delete_Operation_NULL = 0,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ONE,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR,
};

/**
 * @brief 图形操作
 *
 */
enum Enum_Graphic_Operation
{
    Graphic_Operation_NULL = 0,
    Graphic_Operation_ADD,
    Graphic_Operation_CHANGE,
    Graphic_Operation_DELETE,
};

/**
 * @brief 图形类型
 *
 */
enum Enum_Graphic_Type
{
    Graphic_Type_LINE = 0,
    Graphic_Type_RECTANGLE,
    Graphic_Type_CIRCLE,
    Graphic_Type_OVAL,
    Graphic_Type_ARC,
    Graphic_Type_FLOAT,
    Graphic_Type_INTEGER,
    Graphic_Type_STRING,
};

/**
 * @brief 图形颜色
 *
 */
enum Enum_Graphic_Color
{
    Graphic_Color_MAIN = 0,
    Graphic_Color_YELLOW,
    Graphic_Color_GREEN,
    Graphic_Color_ORANGE,
    Graphic_Color_PURPLE,
    Graphic_Color_PINK,
    Graphic_Color_CYAN,
    Graphic_Color_BLACK,
    Graphic_Color_WHITE,
};

/**
 * @brief 直线图形结构体
 *
 */
struct Struct_Graphic_Line
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Reserved_3 : 10;
    uint32_t End_X : 11;
    uint32_t End_Y : 11;
} __attribute__((packed));

/**
 * @brief 矩形图形结构体
 *
 */
struct Struct_Graphic_Rectangle
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Reserved_3 : 10;
    uint32_t End_X : 11;
    uint32_t End_Y : 11;
} __attribute__((packed));

/**
 * @brief 圆形图形结构体
 *
 */
struct Struct_Graphic_Circle
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Center_X : 11;
    uint32_t Center_Y : 11;
    uint32_t Radius : 10;
    uint32_t Reserved_3 : 11;
    uint32_t Reserved_4 : 11;
} __attribute__((packed));

/**
 * @brief 椭圆图形结构体
 *
 */
struct Struct_Graphic_Oval
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Center_X : 11;
    uint32_t Center_Y : 11;
    uint32_t Reserved_3 : 10;
    uint32_t Half_Length_X : 11;
    uint32_t Half_Length_Y : 11;
} __attribute__((packed));

/**
 * @brief 圆弧图形结构体
 * Angle单位是角度制而非弧度制
 *
 */
struct Struct_Graphic_Arc
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Start_Angle : 9;
    uint32_t End_Angle : 9;
    uint32_t Line_Width : 10;
    uint32_t Center_X : 11;
    uint32_t Center_Y : 11;
    uint32_t Reserved : 10;
    uint32_t Half_Length_X : 11;
    uint32_t Half_Length_Y : 11;
} __attribute__((packed));

/**
 * @brief 浮点数图形结构体
 * Decimal_Number 小数点后位数, 最多3位
 * Float *1000后的32位有符号整数
 *
 */
struct Struct_Graphic_Float
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Font_Size : 9;
    uint32_t Decimal_Number : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    int32_t Float;
} __attribute__((packed));

/**
 * @brief 整型数图形结构体
 *
 */
struct Struct_Graphic_Integer
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Font_Size : 9;
    uint32_t Reserved : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    int32_t Integer;
} __attribute__((packed));

/**
 * @brief 字符串图形结构体
 *
 */
struct Struct_Graphic_String
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Font_Size : 9;
    uint32_t Length : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Reserved;
} __attribute__((packed));

/**
 * @brief 各种图形的联合体
 *
 */
union Union_Graphic
{
    Struct_Graphic_Line Line;
    Struct_Graphic_Rectangle Rectangle;
    Struct_Graphic_Circle Circle;
    Struct_Graphic_Oval Oval;
    Struct_Graphic_Arc Arc;
    Struct_Graphic_Float Float;
    Struct_Graphic_Integer Integer;
    Struct_Graphic_String String;
} __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_Referee_UART_Data
{
    uint8_t Frame_Header;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    Enum_Referee_Command_ID Referee_Command_ID;
    uint8_t Data[121];
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0001比赛状态, 3Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Game_Status
{
    uint8_t Frame_Header;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    Enum_Referee_Command_ID Referee_Command_ID;
    uint8_t Type_Enum : 4;
    uint8_t Stage_Enum : 4;
    uint16_t Remaining_Time;
    uint64_t Timestamp;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0002比赛结果, 比赛结束后发送
 *
 */
struct Struct_Referee_Rx_Data_Game_Result
{
    Enum_Referee_Game_Result Result;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0003机器人血量, 1Hz
 *
 */
struct Struct_Referee_Rx_Data_Game_Robot_HP
{
    uint16_t Red_Hero_1;
    uint16_t Red_Engineer_2;
    uint16_t Red_Infantry_3;
    uint16_t Red_Infantry_4;
    uint16_t Red_Infantry_5;
    uint16_t Red_Sentry_7;
    uint16_t Red_Outpost_11;
    uint16_t Red_Base_10;
    uint16_t Blue_Hero_1;
    uint16_t Blue_Engineer_2;
    uint16_t Blue_Infantry_3;
    uint16_t Blue_Infantry_4;
    uint16_t Blue_Infantry_5;
    uint16_t Blue_Sentry_7;
    uint16_t Blue_Outpost_11;
    uint16_t Blue_Base_10;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0101场地事件, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Data
{
    uint32_t Supply_1_Status_Enum : 1;
    uint32_t Supply_2_Status_Enum : 1;
    uint32_t Supply_3_Status_Enum : 1;
    uint32_t Energy_Status_Enum : 1;
    uint32_t Energy_Small_Status_Enum : 1;
    uint32_t Energy_Big_Status_Enum : 1;
    uint32_t Highland_2_Status_Enum : 1;
    uint32_t Highland_3_Status_Enum : 1;
    uint32_t Highland_4_Status_Enum : 1;
    uint32_t Base_Shield_Status_Enum : 1;
    uint32_t Outpost_Status_Enum : 1;
    uint32_t Reserved : 21;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0102补给站状态, 补给请求后对应发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Supply
{
    uint8_t Supply_ID;
    Enum_Referee_Data_Robots_ID Robot;
    Enum_Referee_Data_Event_Supply_Status Status;
    Enum_Referee_Data_Event_Supply_Ammo_Number Ammo_Number;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0104裁判警告信息, 判罚发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Referee_Warning
{
    Enum_Referee_Data_Event_Referee_Warning_Level Level;
    Enum_Referee_Data_Robot_ID Robot_ID;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0105飞镖15s倒计时, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Dart_Remaining_Time
{
    uint8_t Dart_Remaining_Time;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0201机器人状态, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Status
{
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Level;
    uint16_t HP;
    uint16_t HP_Max;
    uint16_t Booster_17mm_1_Heat_CD;
    uint16_t Booster_17mm_1_Heat_Max;
    uint16_t Booster_17mm_1_Speed_Max;
    uint16_t Booster_17mm_2_Heat_CD;
    uint16_t Booster_17mm_2_Heat_Max;
    uint16_t Booster_17mm_2_Speed_Max;
    uint16_t Booster_42mm_Heat_CD;
    uint16_t Booster_42mm_Heat_Max;
    uint16_t Booster_42mm_Speed_Max;
    uint16_t Chassis_Power_Max;
    uint8_t PM01_Gimbal_Status_Enum : 1;
    uint8_t PM01_Chassis_Status_Enum : 1;
    uint8_t PM01_Booster_Status_Enum : 1;
    uint8_t Reserved : 5;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0202当前机器人实时功率热量, 50Hz发送
 * 电压mV, 电流mA
 *
 */
struct Struct_Referee_Rx_Data_Robot_Power_Heat
{
    uint16_t Chassis_Voltage;
    uint16_t Chassis_Current;
    float Chassis_Power;
    uint16_t Chassis_Energy_Buffer;
    uint16_t Booster_17mm_1_Heat;
    uint16_t Booster_17mm_2_Heat;
    uint16_t Booster_42mm_Heat;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0203当前机器人实时位置, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Position
{
    float Location_X;
    float Location_Y;
    float Location_Z;
    float Location_Yaw;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0204当前机器人增益, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Buff
{
    uint8_t HP_Buff_Status_Enum : 1;
    uint8_t Booster_Heat_CD_Buff_Status_Enum : 1;
    uint8_t Defend_Buff_Status_Enum : 1;
    uint8_t Damage_Buff_Status_Enum : 1;
    uint8_t Reserved : 4;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0205无人机可攻击时间, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Aerial_Energy
{
    uint8_t Remaining_Time;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0206伤害情况, 伤害发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Damage
{
    uint8_t Armor_ID : 4;
    uint8_t Type_Enum : 4;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0207子弹信息, 射击发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Booster
{
    Enum_Referee_Data_Robot_Ammo_Type Ammo_Type;
    Enum_Referee_Data_Robot_Booster_Type Booster_Type;
    uint8_t Frequency;
    uint16_t Speed;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0208子弹剩余信息, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Remaining_Ammo
{
    uint16_t Booster_17mm;
    uint16_t Booster_42mm;
    uint16_t Money;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0209RFID状态信息, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_RFID
{
    uint32_t Base_Status_Enum : 1;
    uint32_t Highland_Status_Enum : 1;
    uint32_t Energy_Status_Enum : 1;
    uint32_t Flyover_Status_Enum : 1;
    uint32_t Outpost_Status_Enum : 1;
    uint32_t HP_Status_Enum : 1;
    uint32_t Engineer_Status_Enum : 1;
    uint32_t Reserved : 25;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020a飞镖状态, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Dart_Command
{
    Enum_Referee_Data_Robot_Dart_Command_Status Status;
    Enum_Referee_Data_Robot_Dart_Command_Target Target;
    uint16_t Change_Target_Timestamp;
    uint32_t Last_Confirm_Timestamp;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送或接收的数据, 0x0301机器人间交互信息, 用户自主发送
 * TODO 视情况启用
 * Header 0x0200~0x02ff
 * Data 最大112
 *
 */
// struct Struct_Referee_Data_Interaction_Students
// {
//     uint16_t Header;
//     Enum_Referee_Data_Robots_ID Sender;
//     uint8_t Reserved_1;
//     Enum_Referee_Data_Robots_ID Receiver;
//     uint8_t Reserved_2;
//     uint8_t Data[112];
//     uint16_t CRC_16;
// }__attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301图形删除交互信息, 用户自主发送
 * Header 0x0100
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Layer_Delete
{
    uint16_t Header = 0x0100;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Enum_Referee_Data_Interaction_Layer_Delete_Operation Operation;
    uint8_t Delete_Serial;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画一个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_1
{
    uint16_t Header = 0x0101;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画两个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_2
{
    uint16_t Header = 0x0102;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
    Union_Graphic Graphic_2;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画五个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_5
{
    uint16_t Header = 0x0103;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
    Union_Graphic Graphic_2;
    Union_Graphic Graphic_3;
    Union_Graphic Graphic_4;
    Union_Graphic Graphic_5;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画七个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_7
{
    uint16_t Header = 0x0104;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
    Union_Graphic Graphic_2;
    Union_Graphic Graphic_3;
    Union_Graphic Graphic_4;
    Union_Graphic Graphic_5;
    Union_Graphic Graphic_6;
    Union_Graphic Graphic_7;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画字符图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_String
{
    uint16_t Header = 0x0110;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_String;
    uint8_t String[30];
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0302自定义控制器交互信息, 用户自主发送
 * TODO 视情况赋予Data含义
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Custom_Controller
{
    uint8_t Data[30];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0303雷达发送小地图交互信息, 用户自主最高30Hz发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Radar_Send
{
    float Coordinate_X;
    float Coordinate_Y;
    float Coordinate_Z;
    uint8_t Keyboard;
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0304图传遥控交互信息, 30Hz发送
 * TODO 等待扩展
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Remote_Control
{
    uint16_t Mouse_X;
    uint16_t Mouse_Y;
    uint16_t Mouse_Z;
    Enum_Referee_Data_Status Mouse_Left_Key_Status;
    Enum_Referee_Data_Status Mouse_Right_Key_Status;
    uint16_t Keyboard_Key;
    uint16_t Reserved;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0305客户端接收小地图交互信息, 用户自主最高30Hz发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Client_Receive
{
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Reserved_1;
    float Coordinate_X;
    float Coordinate_Y;
    uint8_t Reserved_2;
} __attribute__((packed));

/**
 * @brief Specialized, 裁判系统
 *
 */
class Class_Referee
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xa5);

    inline Enum_Referee_Status Get_Referee_Status();
    inline Enum_Referee_Game_Status_Type Get_Game_Type();
    inline Enum_Referee_Game_Status_Stage Get_Game_Stage();
    inline uint16_t Get_Remaining_Time();
    inline uint64_t Get_Timestamp();
    inline Enum_Referee_Game_Result Get_Result();
    inline uint16_t Get_HP(Enum_Referee_Data_Robots_ID Robots_ID);
    inline Enum_Referee_Data_Status Get_Event_Supply_Status(uint8_t Supply_ID);
    inline Enum_Referee_Data_Status Get_Event_Energy_Status();
    inline Enum_Referee_Data_Status Get_Event_Energy_Small_Activate_Status();
    inline Enum_Referee_Data_Status Get_Event_Energy_Big_Activate_Status();
    inline Enum_Referee_Data_Status Get_Event_Highland_Status(uint8_t Highland_ID);
    inline Enum_Referee_Data_Status Get_Event_Base_Shield_Status();
    inline Enum_Referee_Data_Status Get_Event_Outpost_Status();
    inline uint8_t Get_Supply_ID();
    inline Enum_Referee_Data_Robots_ID Get_Supply_Request_Robot();
    inline Enum_Referee_Data_Event_Supply_Status Get_Supply_Request_Status();
    inline Enum_Referee_Data_Event_Supply_Ammo_Number Get_Supply_Ammo_Number();
    inline Enum_Referee_Data_Event_Referee_Warning_Level Get_Referee_Warning();
    inline Enum_Referee_Data_Robot_ID Get_Referee_Warning_Robot();
    inline uint8_t Get_Dart_Remaining_Time();
    inline Enum_Referee_Data_Robots_ID Get_ID();
    inline uint8_t Get_Level();
    inline uint16_t Get_HP();
    inline uint16_t Get_HP_Max();
    inline uint16_t Get_Booster_17mm_1_Heat_CD();
    inline uint16_t Get_Booster_17mm_1_Heat_Max();
    inline uint16_t Get_Booster_17mm_1_Speed_Max();
    inline uint16_t Get_Booster_17mm_2_Heat_CD();
    inline uint16_t Get_Booster_17mm_2_Heat_Max();
    inline uint16_t Get_Booster_17mm_2_Speed_Max();
    inline uint16_t Get_Booster_42mm_Heat_CD();
    inline uint16_t Get_Booster_42mm_Heat_Max();
    inline uint16_t Get_Booster_42mm_Speed_Max();
    inline uint16_t Get_Chassis_Power_Max();
    inline Enum_Referee_Data_Status Get_PM01_Gimbal_Status();
    inline Enum_Referee_Data_Status Get_PM01_Chassis_Status();
    inline Enum_Referee_Data_Status Get_PM01_Booster_Status();
    inline float Get_Chassis_Voltage();
    inline float Get_Chassis_Current();
    inline float Get_Chassis_Power();
    inline uint16_t Get_Chassis_Energy_Buffer();
    inline uint16_t Get_Booster_17mm_1_Heat();
    inline uint16_t Get_Booster_17mm_2_Heat();
    inline uint16_t Get_Booster_42mm_Heat();
    inline float Get_Location_X();
    inline float Get_Location_Y();
    inline float Get_Location_Z();
    inline float Get_Location_Yaw();
    inline Enum_Referee_Data_Status Get_HP_Buff_Status();
    inline Enum_Referee_Data_Status Get_Booster_Heat_CD_Buff_Status();
    inline Enum_Referee_Data_Status Get_Defend_Buff_Status();
    inline Enum_Referee_Data_Status Get_Damage_Buff_Status();
    inline uint8_t Get_Aerial_Remaining_Time();
    inline uint8_t Get_Armor_Attacked_ID();
    inline Enum_Referee_Data_Event_Robot_Damage_Type Get_Attacked_Type();
    inline Enum_Referee_Data_Robot_Ammo_Type Get_Shoot_Ammo_Type();
    inline Enum_Referee_Data_Robot_Booster_Type Get_Shoot_Booster_Type();
    inline uint8_t Get_Shoot_Frequency();
    inline float Get_Shoot_Speed();
    inline uint16_t Get_17mm_Remaining();
    inline uint16_t Get_42mm_Remaining();
    inline uint16_t Get_Money_Remaining();
    inline Enum_Referee_Data_Status Get_Base_RFID_Status();
    inline Enum_Referee_Data_Status Get_Highland_RFID_Status();
    inline Enum_Referee_Data_Status Get_Energy_RFID_Status();
    inline Enum_Referee_Data_Status Get_Flyover_RFID_Status();
    inline Enum_Referee_Data_Status Get_Outpost_RFID_Status();
    inline Enum_Referee_Data_Status Get_HP_RFID_Status();
    inline Enum_Referee_Data_Status Get_Engineer_RFID_Status();
    inline Enum_Referee_Data_Robot_Dart_Command_Status Get_Dart_Command_Status();
    inline Enum_Referee_Data_Robot_Dart_Command_Target Get_Dart_Command_Target();
    inline uint16_t Get_Dart_Change_Target_Timestamp();
    inline uint16_t Get_Dart_Last_Confirm_Timestamp();
    inline Enum_Referee_Data_Robots_ID Get_Radar_Send_Robot_ID();
    inline float Get_Radar_Send_Coordinate_X();
    inline float Get_Radar_Send_Coordinate_Y();

    void UART_RxCpltCallback(uint8_t *Rx_Data);
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void CAN_RxCpltCallback(uint8_t *Rx_Data, uint32_t id);

protected:
    // 初始化相关常量

    // 绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
    //数据包头标
    uint8_t Frame_Header;

    //常量

    //内部变量

    //当前时刻的裁判系统接收flag
    uint32_t Flag = 0;
    //前一时刻的裁判系统接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //裁判系统状态
    Enum_Referee_Status Referee_Status = Referee_Status_DISABLE;
    //比赛状态
    Struct_Referee_Rx_Data_Game_Status Game_Status;
    //比赛结果
    Struct_Referee_Rx_Data_Game_Result Game_Result;
    //机器人血量
    Struct_Referee_Rx_Data_Game_Robot_HP Game_Robot_HP;
    //场地事件
    Struct_Referee_Rx_Data_Event_Data Event_Data;
    //补给站状态
    Struct_Referee_Rx_Data_Event_Supply Event_Supply;
    //裁判警告信息
    Struct_Referee_Rx_Data_Event_Referee_Warning Event_Referee_Warning;
    //飞镖15s倒计时
    Struct_Referee_Rx_Data_Event_Dart_Remaining_Time Event_Dart_Remaining_Time;
    //机器人状态
    Struct_Referee_Rx_Data_Robot_Status Robot_Status;
    //当前机器人实时功率热量
    Struct_Referee_Rx_Data_Robot_Power_Heat Robot_Power_Heat;
    //当前机器人实时位置
    Struct_Referee_Rx_Data_Robot_Position Robot_Position;
    //当前机器人增益
    Struct_Referee_Rx_Data_Robot_Buff Robot_Buff;
    //无人机可攻击时间
    Struct_Referee_Rx_Data_Robot_Aerial_Energy Robot_Aerial_Energy;
    //伤害情况
    Struct_Referee_Rx_Data_Robot_Damage Robot_Damage;
    //子弹信息
    Struct_Referee_Rx_Data_Robot_Booster Robot_Booster;
    //子弹剩余信息
    Struct_Referee_Rx_Data_Robot_Remaining_Ammo Robot_Remaining_Ammo;
    // RFID状态信息
    Struct_Referee_Rx_Data_Robot_RFID Robot_RFID;
    //飞镖状态
    Struct_Referee_Rx_Data_Robot_Dart_Command Robot_Dart_Command;
    //客户端接收小地图交互信息
    Struct_Referee_Tx_Data_Interaction_Client_Receive Interaction_Client_Receive;
    //图传链路
    Struct_Referee_Tx_Data_Interaction_Remote_Control Interaction_Remote_Control;

    // 写变量

    // 图形删除交互信息
    Struct_Referee_Tx_Data_Interaction_Layer_Delete Interaction_Layer_Delete;
    //画一个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_1 Interaction_Graphic_1;
    //画两个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_2 Interaction_Graphic_2;
    //画五个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_5 Interaction_Graphic_5;
    //画七个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_7 Interaction_Graphic_7;
    //画字符图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_String Interaction_Graphic_String;
    //雷达发送小地图交互信息
    Struct_Referee_Tx_Data_Interaction_Radar_Send Interaction_Radar_Send;

    //读写变量

    //内部函数

    uint8_t CRC_8(uint8_t *Message, uint32_t Length);
    uint16_t CRC_16(uint8_t *Message, uint32_t Length);
    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取裁判系统状态
 *
 * @return Enum_Referee_Status 裁判系统状态
 */
Enum_Referee_Status Class_Referee::Get_Referee_Status()
{
    return (Referee_Status);
}

/**
 * @brief 获取比赛类型
 *
 * @return Enum_Referee_Game_Status_Type 比赛类型
 */
Enum_Referee_Game_Status_Type Class_Referee::Get_Game_Type()
{
    return (static_cast<Enum_Referee_Game_Status_Type>(Game_Status.Type_Enum));
}

/**
 * @brief 获取比赛阶段
 *
 * @return Enum_Referee_Game_Status_Stage 比赛阶段
 */
Enum_Referee_Game_Status_Stage Class_Referee::Get_Game_Stage()
{
    return (static_cast<Enum_Referee_Game_Status_Stage>(Game_Status.Stage_Enum));
}

/**
 * @brief 获取当前阶段剩余时间
 *
 * @return uint16_t 当前阶段剩余时间
 */
uint16_t Class_Referee::Get_Remaining_Time()
{
    return (Game_Status.Remaining_Time);
}

/**
 * @brief 获取系统时间戳
 *
 * @return uint64_t 系统时间戳
 */
uint64_t Class_Referee::Get_Timestamp()
{
    return (Game_Status.Timestamp);
}

/**
 * @brief 获取比赛结果
 *
 * @return Enum_Referee_Game_Result 比赛结果
 */
Enum_Referee_Game_Result Class_Referee::Get_Result()
{
    return (Game_Result.Result);
}

/**
 * @brief 获取机器人血量
 *
 * @param Robots_ID 通用双方机器人ID
 * @return uint16_t 机器人血量
 */
uint16_t Class_Referee::Get_HP(Enum_Referee_Data_Robots_ID Robots_ID)
{
    switch (Robots_ID)
    {
    case (Referee_Data_Robots_ID_RED_HERO_1):
    {
        return (Game_Robot_HP.Red_Hero_1);
    }
    break;
    case (Referee_Data_Robots_ID_RED_ENGINEER_2):
    {
        return (Game_Robot_HP.Red_Engineer_2);
    }
    break;
    case (Referee_Data_Robots_ID_RED_INFANTRY_3):
    {
        return (Game_Robot_HP.Red_Infantry_3);
    }
    break;
    case (Referee_Data_Robots_ID_RED_INFANTRY_4):
    {
        return (Game_Robot_HP.Red_Infantry_4);
    }
    break;
    case (Referee_Data_Robots_ID_RED_INFANTRY_5):
    {
        return (Game_Robot_HP.Red_Infantry_5);
    }
    break;
    case (Referee_Data_Robots_ID_RED_SENTRY_7):
    {
        return (Game_Robot_HP.Red_Sentry_7);
    }
    break;
    case (Referee_Data_Robots_ID_RED_OUTPOST_11):
    {
        return (Game_Robot_HP.Red_Outpost_11);
    }
    break;
    case (Referee_Data_Robots_ID_RED_BASE_10):
    {
        return (Game_Robot_HP.Red_Base_10);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_HERO_1):
    {
        return (Game_Robot_HP.Blue_Hero_1);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_ENGINEER_2):
    {
        return (Game_Robot_HP.Blue_Engineer_2);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_3):
    {
        return (Game_Robot_HP.Blue_Infantry_3);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_4):
    {
        return (Game_Robot_HP.Blue_Infantry_4);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_5):
    {
        return (Game_Robot_HP.Blue_Infantry_5);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_SENTRY_7):
    {
        return (Game_Robot_HP.Blue_Sentry_7);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_OUTPOST_11):
    {
        return (Game_Robot_HP.Blue_Outpost_11);
    }
    break;
    case (Referee_Data_Robots_ID_BLUE_BASE_10):
    {
        return (Game_Robot_HP.Blue_Base_10);
    }
    break;
    }
}

/**
 * @brief 获取补给站占领状态
 *
 * @param Supply_ID 补给站ID, 1~3
 * @return Enum_Referee_Data_Status 补给站占领状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Supply_Status(uint8_t Supply_ID)
{
    switch (Supply_ID)
    {
    case (1):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_1_Status_Enum));
    }
    break;
    case (2):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_2_Status_Enum));
    }
    break;
    case (3):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_3_Status_Enum));
    }
    break;
    }
}

/**
 * @brief 获取能量机关占领状态
 *
 * @return Enum_Referee_Data_Status 能量机关占领状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Energy_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Energy_Status_Enum));
}

/**
 * @brief 获取小能量机关激活状态
 *
 * @return Enum_Referee_Data_Status 小能量机关激活状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Energy_Small_Activate_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Energy_Small_Status_Enum));
}

/**
 * @brief 获取大能量机关激活状态
 *
 * @return Enum_Referee_Data_Status 大能量机关激活状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Energy_Big_Activate_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Energy_Big_Status_Enum));
}

/**
 * @brief 获取高地占领状态
 *
 * @param Highland_ID 高地ID, 2~4
 * @return Enum_Referee_Data_Status 高地占领状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Highland_Status(uint8_t Highland_ID)
{
    switch (Highland_ID)
    {
    case (2):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_2_Status_Enum));
    }
    break;
    case (3):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_3_Status_Enum));
    }
    break;
    case (4):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_4_Status_Enum));
    }
    break;
    }
}

/**
 * @brief 获取基地护盾状态
 *
 * @return Enum_Referee_Data_Status 基地护盾状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Base_Shield_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Base_Shield_Status_Enum));
}

/**
 * @brief 获取前哨站存活状态
 *
 * @return Enum_Referee_Data_Status 前哨站存活状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Outpost_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Outpost_Status_Enum));
}

/**
 * @brief 获取补给点ID
 *
 * @return uint8_t 补给点ID
 */
uint8_t Class_Referee::Get_Supply_ID()
{
    return (Event_Supply.Supply_ID);
}

/**
 * @brief 获取请求补给的机器人ID
 *
 * @return Enum_Referee_Data_Robots_ID 请求补给的机器人ID
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_Supply_Request_Robot()
{
    return (Event_Supply.Robot);
}

/**
 * @brief 获取补给站的补给状态
 *
 * @return Enum_Referee_Data_Event_Supply_Status 补给站的补给状态
 */
Enum_Referee_Data_Event_Supply_Status Class_Referee::Get_Supply_Request_Status()
{
    return (Event_Supply.Status);
}

/**
 * @brief 获取补给子弹数量
 *
 * @return Enum_Referee_Data_Event_Supply_Ammo_Number 补给子弹数量
 */
Enum_Referee_Data_Event_Supply_Ammo_Number Class_Referee::Get_Supply_Ammo_Number()
{
    return (Event_Supply.Ammo_Number);
}

/**
 * @brief 获取裁判判罚信息
 *
 * @return Enum_Referee_Data_Event_Referee_Warning_Level 裁判判罚信息
 */
Enum_Referee_Data_Event_Referee_Warning_Level Class_Referee::Get_Referee_Warning()
{
    return (Event_Referee_Warning.Level);
}

/**
 * @brief 获取裁判判罚机器人
 *
 * @return Enum_Referee_Data_Robot_ID 裁判判罚机器人
 */
Enum_Referee_Data_Robot_ID Class_Referee::Get_Referee_Warning_Robot()
{
    return (Event_Referee_Warning.Robot_ID);
}

/**
 * @brief 获取飞镖剩余时间
 *
 * @return uint8_t 飞镖剩余时间
 */
uint8_t Class_Referee::Get_Dart_Remaining_Time()
{
    return (Event_Dart_Remaining_Time.Dart_Remaining_Time);
}

/**
 * @brief 获取自身ID
 *
 * @return Enum_Referee_Data_Robots_ID 自身ID
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_ID()
{
    return (Robot_Status.Robot_ID);
}

/**
 * @brief 获取自身等级
 *
 * @return uint8_t 自身等级
 */
uint8_t Class_Referee::Get_Level()
{
    return (Robot_Status.Level);
}

/**
 * @brief 获取自身血量
 *
 * @return uint16_t 自身血量
 */
uint16_t Class_Referee::Get_HP()
{
    return (Robot_Status.HP);
}

/**
 * @brief 获取自身血量上限
 *
 * @return uint16_t 自身血量上限
 */
uint16_t Class_Referee::Get_HP_Max()
{
    return (Robot_Status.HP_Max);
}

/**
 * @brief 获取17mm1枪口冷却速度
 *
 * @return uint16_t 17mm1枪口冷却速度
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Heat_CD()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_1_Heat_CD == 0)
    {
        return (40);
    }
#endif
    return (Robot_Status.Booster_17mm_1_Heat_CD);
}

/**
 * @brief 获取17mm1枪口热量上限
 *
 * @return uint16_t 17mm1枪口热量上限
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Heat_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_1_Heat_Max == 0)
    {
        return (240);
    }
#endif
    return (Robot_Status.Booster_17mm_1_Heat_Max);
}

/**
 * @brief 获取17mm1枪口射速上限
 *
 * @return uint16_t 17mm1枪口射速上限
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Speed_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_1_Speed_Max == 0)
    {
        return (30);
    }
#endif
    return (Robot_Status.Booster_17mm_1_Speed_Max);
}

/**
 * @brief 获取17mm2枪口冷却速度
 *
 * @return uint16_t 17mm2枪口冷却速度
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Heat_CD()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_2_Heat_CD == 0)
    {
        return (40);
    }
#endif
    return (Robot_Status.Booster_17mm_2_Heat_CD);
}

/**
 * @brief 获取17mm2枪口热量上限
 *
 * @return uint16_t 17mm2枪口热量上限
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Heat_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_2_Heat_Max == 0)
    {
        return (240);
    }
#endif
    return (Robot_Status.Booster_17mm_2_Heat_Max);
}

/**
 * @brief 获取17mm2枪口射速上限
 *
 * @return uint16_t 17mm2枪口射速上限
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Speed_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_2_Speed_Max == 0)
    {
        return (30);
    }
#endif
    return (Robot_Status.Booster_17mm_2_Speed_Max);
}

/**
 * @brief 获取42mm枪口冷却速度
 *
 * @return uint16_t 42mm枪口冷却速度
 */
uint16_t Class_Referee::Get_Booster_42mm_Heat_CD()
{
    return (Robot_Status.Booster_42mm_Heat_CD);
}

/**
 * @brief 获取42mm枪口热量上限
 *
 * @return uint16_t 42mm枪口热量上限
 */
uint16_t Class_Referee::Get_Booster_42mm_Heat_Max()
{
    return (Robot_Status.Booster_42mm_Heat_Max);
}

/**
 * @brief 获取42mm枪口射速上限
 *
 * @return uint16_t 42mm枪口射速上限
 */
uint16_t Class_Referee::Get_Booster_42mm_Speed_Max()
{
    return (Robot_Status.Booster_42mm_Speed_Max);
}

/**
 * @brief 获取底盘功率上限
 *
 * @return uint16_t 底盘功率上限
 */
uint16_t Class_Referee::Get_Chassis_Power_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Chassis_Power_Max == 0)
    {
        //裁判系统如果寄了
        return (80);
        // //高校联盟赛
        // return (100);
        // //超级对抗赛
        // return (150);
    }
#endif
    return (Robot_Status.Chassis_Power_Max);
}

/**
 * @brief 获取Gimbal供电状态
 *
 * @return Enum_Referee_Data_Status Gimbal供电状态
 */
Enum_Referee_Data_Status Class_Referee::Get_PM01_Gimbal_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Status.PM01_Gimbal_Status_Enum));
}

/**
 * @brief 获取Chassis供电状态
 *
 * @return Enum_Referee_Data_Status Chassis供电状态
 */
Enum_Referee_Data_Status Class_Referee::Get_PM01_Chassis_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Status.PM01_Chassis_Status_Enum));
}

/**
 * @brief 获取Booster供电状态
 *
 * @return Enum_Referee_Data_Status Booster供电状态
 */
Enum_Referee_Data_Status Class_Referee::Get_PM01_Booster_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Status.PM01_Booster_Status_Enum));
}

/**
 * @brief 获取底盘电压
 *
 * @return float 底盘电压
 */
float Class_Referee::Get_Chassis_Voltage()
{
    return (Robot_Power_Heat.Chassis_Voltage / 1000.0f);
}

/**
 * @brief 获取底盘电流
 *
 * @return float 底盘电流
 */
float Class_Referee::Get_Chassis_Current()
{
    return (Robot_Power_Heat.Chassis_Current / 1000.0f);
}

/**
 * @brief 获取底盘功率
 *
 * @return float 底盘功率
 */
float Class_Referee::Get_Chassis_Power()
{
    return (Robot_Power_Heat.Chassis_Power);
}

/**
 * @brief 获取底盘能量缓冲
 *
 * @return uint16_t 底盘能量缓冲
 */
uint16_t Class_Referee::Get_Chassis_Energy_Buffer()
{
    return (Robot_Power_Heat.Chassis_Energy_Buffer);
}

/**
 * @brief 获取17mm1热量
 *
 * @return uint16_t 17mm1热量
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Heat()
{
    return (Robot_Power_Heat.Booster_17mm_1_Heat);
}

/**
 * @brief 获取17mm2热量
 *
 * @return uint16_t 17mm2热量
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Heat()
{
    return (Robot_Power_Heat.Booster_17mm_2_Heat);
}

/**
 * @brief 获取42mm热量
 *
 * @return uint16_t 42mm热量
 */
uint16_t Class_Referee::Get_Booster_42mm_Heat()
{
    return (Robot_Power_Heat.Booster_42mm_Heat);
}

/**
 * @brief 获取自身位置x
 *
 * @return float 自身位置x
 */
float Class_Referee::Get_Location_X()
{
    return (Robot_Position.Location_X);
}

/**
 * @brief 获取自身位置y
 *
 * @return float 自身位置y
 */
float Class_Referee::Get_Location_Y()
{
    return (Robot_Position.Location_Y);
}

/**
 * @brief 获取自身位置z
 *
 * @return float 自身位置z
 */
float Class_Referee::Get_Location_Z()
{
    return (Robot_Position.Location_Z);
}

/**
 * @brief 获取自身方向yaw
 *
 * @return float 自身方向yaw
 */
float Class_Referee::Get_Location_Yaw()
{
    return (Robot_Position.Location_Yaw);
}

/**
 * @brief 获取补血buff状态
 *
 * @return Enum_Referee_Data_Status 补血buff状态
 */
Enum_Referee_Data_Status Class_Referee::Get_HP_Buff_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Buff.HP_Buff_Status_Enum));
}

/**
 * @brief 获取冷却缩减buff状态
 *
 * @return Enum_Referee_Data_Status 冷却缩减buff状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Booster_Heat_CD_Buff_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Buff.Booster_Heat_CD_Buff_Status_Enum));
}

/**
 * @brief 获取防御加成buff状态
 *
 * @return Enum_Referee_Data_Status 防御加成buff状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Defend_Buff_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Buff.Defend_Buff_Status_Enum));
}

/**
 * @brief 获取攻击加成buff状态
 *
 * @return Enum_Referee_Data_Status 攻击加成buff状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Damage_Buff_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Buff.Damage_Buff_Status_Enum));
}

/**
 * @brief 获取无人机时间
 *
 * @return uint8_t 无人机时间
 */
uint8_t Class_Referee::Get_Aerial_Remaining_Time()
{
    return (Robot_Aerial_Energy.Remaining_Time);
}

/**
 * @brief 获取受击装甲板ID
 *
 * @return uint8_t 受击装甲板ID
 */
uint8_t Class_Referee::Get_Armor_Attacked_ID()
{
    return (Robot_Damage.Armor_ID);
}

/**
 * @brief 获取受击类型
 *
 * @return Enum_Referee_Data_Event_Robot_Damage_Type 受击类型
 */
Enum_Referee_Data_Event_Robot_Damage_Type Class_Referee::Get_Attacked_Type()
{
    return (static_cast<Enum_Referee_Data_Event_Robot_Damage_Type>(Robot_Damage.Type_Enum));
}

/**
 * @brief 获取射击子弹类型
 *
 * @return Enum_Referee_Data_Robot_Ammo_Type 射击子弹类型
 */
Enum_Referee_Data_Robot_Ammo_Type Class_Referee::Get_Shoot_Ammo_Type()
{
    return (Robot_Booster.Ammo_Type);
}

/**
 * @brief 获取发射机构类型
 *
 * @return Enum_Referee_Data_Robot_Booster_Type 发射机构类型
 */
Enum_Referee_Data_Robot_Booster_Type Class_Referee::Get_Shoot_Booster_Type()
{
    return (Robot_Booster.Booster_Type);
}

/**
 * @brief 获取射频, Hz
 *
 * @return uint8_t 射频, Hz
 */
uint8_t Class_Referee::Get_Shoot_Frequency()
{
    return (Robot_Booster.Frequency);
}

/**
 * @brief 获取射速
 *
 * @return float 射速
 */
float Class_Referee::Get_Shoot_Speed()
{
    return (Robot_Booster.Speed);
}

/**
 * @brief 获取17mm弹丸剩余数
 *
 * @return uint16_t 17mm弹丸剩余数
 */
uint16_t Class_Referee::Get_17mm_Remaining()
{
    return (Robot_Remaining_Ammo.Booster_17mm);
}

/**
 * @brief 获取42mm弹丸剩余数
 *
 * @return uint16_t 42mm弹丸剩余数
 */
uint16_t Class_Referee::Get_42mm_Remaining()
{
    return (Robot_Remaining_Ammo.Booster_42mm);
}

/**
 * @brief 获取金币剩余数
 *
 * @return uint16_t 金币剩余数
 */
uint16_t Class_Referee::Get_Money_Remaining()
{
    return (Robot_Remaining_Ammo.Money);
}

/**
 * @brief 获取基地增益RFID状态
 *
 * @return Enum_Referee_Data_Status 基地增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Base_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Base_Status_Enum));
}

/**
 * @brief 获取高地增益RFID状态
 *
 * @return Enum_Referee_Data_Status 高地增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Highland_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Highland_Status_Enum));
}

/**
 * @brief 获取能量机关增益RFID状态
 *
 * @return Enum_Referee_Data_Status 能量机关增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Energy_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Energy_Status_Enum));
}

/**
 * @brief 获取飞坡增益RFID状态
 *
 * @return Enum_Referee_Data_Status 飞坡增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Flyover_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Flyover_Status_Enum));
}

/**
 * @brief 获取前哨站增益RFID状态
 *
 * @return Enum_Referee_Data_Status 前哨站增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Outpost_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Outpost_Status_Enum));
}

/**
 * @brief 获取补血点增益RFID状态
 *
 * @return Enum_Referee_Data_Status 补血点增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_HP_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.HP_Status_Enum));
}

/**
 * @brief 获取工程复活卡增益RFID状态
 *
 * @return Enum_Referee_Data_Status 工程复活卡增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Engineer_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Engineer_Status_Enum));
}

/**
 * @brief 获取飞镖发射口状态
 *
 * @return Enum_Referee_Data_Robot_Dart_Command_Status 飞镖发射口状态
 */
Enum_Referee_Data_Robot_Dart_Command_Status Class_Referee::Get_Dart_Command_Status()
{
    return (Robot_Dart_Command.Status);
}

/**
 * @brief 获取飞镖击打目标
 *
 * @return Enum_Referee_Data_Robot_Dart_Command_Target 飞镖击打目标
 */
Enum_Referee_Data_Robot_Dart_Command_Target Class_Referee::Get_Dart_Command_Target()
{
    return (Robot_Dart_Command.Target);
}

/**
 * @brief 获取击打目标改变的时间戳
 *
 * @return uint16_t 击打目标改变的时间戳
 */
uint16_t Class_Referee::Get_Dart_Change_Target_Timestamp()
{
    return (Robot_Dart_Command.Change_Target_Timestamp);
}

/**
 * @brief 获取云台手下达指令的时间戳
 *
 * @return uint16_t 云台手下达指令的时间戳
 */
uint16_t Class_Referee::Get_Dart_Last_Confirm_Timestamp()
{
    return (Robot_Dart_Command.Last_Confirm_Timestamp);
}

/**
 * @brief 获取雷达发送目标机器人ID
 *
 * @return Enum_Referee_Data_Robots_ID 雷达发送目标的机器人ID
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_Radar_Send_Robot_ID()
{
    return (Interaction_Client_Receive.Robot_ID);
}

/**
 * @brief 获取雷达发送目标机器人位置x
 *
 * @return float 雷达发送目标机器人位置x
 */
float Class_Referee::Get_Radar_Send_Coordinate_X()
{
    return (Interaction_Client_Receive.Coordinate_X);
}

/**
 * @brief 获取雷达发送目标机器人位置y
 *
 * @return float 雷达发送目标机器人位置y
 */
float Class_Referee::Get_Radar_Send_Coordinate_Y()
{
    return (Interaction_Client_Receive.Coordinate_Y);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
