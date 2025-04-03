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

//#define CHASSIS
#define GIMBAL

#define BULLET_SPEED_PID


#ifdef CHASSIS

#define POWER_LIMIT
#ifdef POWER_LIMIT
#define POWER_LIMI//T_BUFFER_LOOP
#define BIG_P_ALLOCATE
//        #define POWER_LIMIT_NEW_CONTROL
//        //#define POWER_LIMIT_OLD_CONTROL
#endif

#define SPEED_SLOPE

#endif

#define DISABLE_SUPEACAP

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
