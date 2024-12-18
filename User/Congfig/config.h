/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief �˻����������߼�
 * @version 0.1
 * @date 2023-08-29 0.1 23��������
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

//#define CHASSIS
#define GIMBAL


#ifdef CHASSIS 	

    #define POWER_LIMIT
    #ifdef POWER_LIMIT
        //#define POWER_LIMIT_BUFFER_LOOP 
        ///#define POWER_LIMIT_NEW_CONTROL
        #define POWER_LIMIT_OLD_CONTROL
    #endif

    //#define SPEED_SLOPE

#endif


/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
