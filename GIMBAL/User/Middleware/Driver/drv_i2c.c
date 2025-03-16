/**
 * @file drv_i2c.c
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 仿照SCUT-Robotlab改写的I2C通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_i2c.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_IIC_Manage_Object IIC1_Manage_Object = {0};
Struct_IIC_Manage_Object IIC2_Manage_Object = {0};
Struct_IIC_Manage_Object IIC3_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 初始化SPI
 *
 * @param hspi UART编号
 * @param Callback_Function 处理回调函数
 */
void IIC_Init(I2C_HandleTypeDef *hi2c, IIC_Call_Back Callback_Function)
{
    if(hi2c->Instance == I2C1)
    {
        IIC1_Manage_Object.IIC_Handler = hi2c;
        IIC1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if(hi2c->Instance == I2C2)
    {
        IIC2_Manage_Object.IIC_Handler = hi2c;
        IIC2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if(hi2c->Instance == I2C3)
    {
        IIC3_Manage_Object.IIC_Handler = hi2c;
        IIC3_Manage_Object.Callback_Function = Callback_Function;
    }
}

/**
 * @brief 交互数据帧
 * 
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t IIC_Send_Receive_Data(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout,Enum_IIC_Read_Write Read_Write)
{
    if (hi2c->Instance == I2C1)
    {		
        IIC1_Manage_Object.DevAddress = DevAddress;
        IIC1_Manage_Object.MemAddress = MemAddress;
        IIC1_Manage_Object.MemAddSize = MemAddSize;
        IIC1_Manage_Object.pData = pData;
        if(Read_Write == IIC_READ)
        {
            return (HAL_I2C_Mem_Read(hi2c,DevAddress,MemAddress,MemAddSize,pData, Size, Timeout));
        }
        else
        {
            return (HAL_I2C_Mem_Write(hi2c,DevAddress,MemAddress,MemAddSize,pData, Size, Timeout));
        }
    }
    if (hi2c->Instance == I2C2)
    {		
        IIC2_Manage_Object.DevAddress = DevAddress;
        IIC2_Manage_Object.MemAddress = MemAddress;
        IIC2_Manage_Object.MemAddSize = MemAddSize;
        IIC2_Manage_Object.pData = pData;
        if(Read_Write == IIC_READ)
        {
            return (HAL_I2C_Mem_Read(hi2c,DevAddress,MemAddress,MemAddSize,pData, Size, Timeout));
        }
        else
        {
            return (HAL_I2C_Mem_Write(hi2c,DevAddress,MemAddress,MemAddSize,pData, Size, Timeout));
        }
    }
    if (hi2c->Instance == I2C3)
    {		
        IIC3_Manage_Object.DevAddress = DevAddress;
        IIC3_Manage_Object.MemAddress = MemAddress;
        IIC3_Manage_Object.MemAddSize = MemAddSize;
        IIC3_Manage_Object.pData = pData;
        if(Read_Write == IIC_READ)
        {
            return (HAL_I2C_Mem_Read(hi2c,DevAddress,MemAddress,MemAddSize,pData, Size, Timeout));
        }
        else
        {
            return (HAL_I2C_Mem_Write(hi2c,DevAddress,MemAddress,MemAddSize,pData, Size, Timeout));
        }
    }
}

/* Function prototypes -------------------------------------------------------*/



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
