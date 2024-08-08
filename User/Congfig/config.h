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
// #define GIMBAL
#define INFORMATION_PLATFORM
#define CONTROLLER
#define DR16_REMOTE
/* Exported macros -----------------------------------------------------------*/
// #define MINI_PC
#define CHASSIS
// #define CHASSIS
#define C_IMU
// #define REFEREE
/*Speed optimization*/
#define SPEED_SLOPE
/*SperCap*/
#define SUPERCAP
#define SPI
#define IIC
#define UART
#ifdef SUPERCAP
#define POWER_LIMIT
#endif

/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
