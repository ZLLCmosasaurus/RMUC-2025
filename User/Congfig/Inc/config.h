/**
 * @file config.h
 * @author lez
 * @brief 工程配置文件
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

//底盘或云台状态
// #define CHASSIS
#define GIMBAL

//调试或比赛状态
#define DEBUG

//功率控制相关
#define POWER_CONTROL 1 //启用功率控制
//#define BIG_P_ALLOCATE
//#define BUFFER_LOOP

/* 兵种/底盘类型/舵小板选择 ------------------------------------------------------------*/
//#define AGV      //舵轮底盘
#define OMNI_WHEEL //全向轮底盘
//#define INFANTRY //步兵
//#define HERO  
#define SENTRY //哨兵

/*轮组数据*/
#ifdef INFANTRY
// #define ENCODER_TO_OUTPUT_RATIO 1.0f / 4.0f // 编码器转四圈，输出轴转一圈
// #define OUTPUT_TO_ENCODER_RATIO 4.0f        
// #define DIR_ROTOR_TO_OUTPUT_RATIO 1.0f / 8.0f // 转向电机转子转八圈，输出轴转一圈
// #define DIR_OUTPUT_TO_ROTOR_RATIO 8.0f      
// #define MOT_ROTOR_TO_OUTPUT_RATIO 1.0f / 14.0f // 动力电机转子转14圈，输出轴转一圈
// #define MOT_OUTPUT_TO_ROTOR_RATIO 14.0f        

#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef HERO
//需要英雄组同学进行填充
#define ENCODER_TO_OUTPUT_RATIO
#define OUTPUT_TO_ENCODER_RATIO 
#define ROTOR_TO_OUTPUT_RATIO 
#define OUTPUT_TO_ROTOR_RATIO 

#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef SENTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#define Chassis_Radius 0.46000000f // 底盘半径，单位为m
#endif



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
