/**
 * @file DR16.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief DR16遥控器
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"
#include "tsk_config_and_callback.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器DR16初始化
 *
 * @param huart 指定的UART
 */
void Class_DR16::Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
//    else if (huart->Instance == UART7)
//    {
//        UART_Manage_Object = &UART7_Manage_Object;
//    }
//    else if (huart->Instance == UART8)
//    {
//        UART_Manage_Object = &UART8_Manage_Object;
//    }
}

/**
 * @brief 判断拨动开关状态
 *
 */
void Class_DR16::Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    //带触发的判断
    switch (Pre_Status)
    {
    case (SWITCH_UP):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_DOWN):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_MIDDLE;
        }
        break;
        }
    }
    break;
    }
}

/**
 * @brief 判断按键状态
 *
 */
void Class_DR16::Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    //带触发的判断
    switch (Pre_Status)
    {
    case (KEY_FREE):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = DR16_Key_Status_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;
        }
        break;
        }
    }
    break;
    case (KEY_PRESSED):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = DR16_Key_Status_PRESSED;
        }
        break;
        }
    }
    break;
    }
}

/**
 * @brief 判断遥控器更新状态
 *
 */
void Class_DR16::Judge_Updata(Struct_DR16_UART_Data Pre_UART_Rx_Data,Struct_DR16_UART_Data Now_UART_Rx_Data)
{
    if((Pre_UART_Rx_Data.Channel_0 == Now_UART_Rx_Data.Channel_0)&&
       (Pre_UART_Rx_Data.Channel_1 == Now_UART_Rx_Data.Channel_1)&&
       (Pre_UART_Rx_Data.Channel_2 == Now_UART_Rx_Data.Channel_2)&&
       (Pre_UART_Rx_Data.Channel_3 == Now_UART_Rx_Data.Channel_3))
    {
        DR16_Updata_Status = DR16_Status_DisUpdata;
    }
    else
    {
        DR16_Updata_Status = DR16_Status_Updata;
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DR16::Data_Process()
{
    //获取当前原始值数据
    memcpy(&Now_UART_Rx_Data, UART_Manage_Object->Rx_Buffer, 18 * sizeof(uint8_t));
    //数据处理过程
    Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *)UART_Manage_Object->Rx_Buffer;

    /*源数据转为对外数据*/

    //摇杆信息
    Data.Right_X = (tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num;
    Data.Right_Y = (tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num;
    Data.Left_X = (tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num;
    Data.Left_Y = (tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num;

    //判断拨码触发
    Judge_Switch(&Data.Left_Switch, tmp_buffer->Switch_1, Pre_UART_Rx_Data.Switch_1);
    Judge_Switch(&Data.Right_Switch, tmp_buffer->Switch_2, Pre_UART_Rx_Data.Switch_2);

    //鼠标信息
    Data.Mouse_X = tmp_buffer->Mouse_X / 32768.0f;
    Data.Mouse_Y = tmp_buffer->Mouse_Y / 32768.0f;
    Data.Mouse_Z = tmp_buffer->Mouse_Z / 32768.0f;

    //判断鼠标触发
    Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
    Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);

    //判断键盘触发
    for (int i = 0; i < 16; i++)
    {
        Judge_Key(&Data.Keyboard_Key[i], ((tmp_buffer->Keyboard_Key) >> i) & 0x1, ((Pre_UART_Rx_Data.Keyboard_Key) >> i) & 0x1);
    }

    //左前轮信息
    Data.Wheel = (tmp_buffer->Channel_Wheel - Rocker_Offset) / Rocker_Num;

    Judge_Updata(Pre_UART_Rx_Data,Now_UART_Rx_Data);
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DR16::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断遥控器是否在线
    Flag += 1;

    Data_Process();

    //保留上一次数据
    memcpy(&Pre_UART_Rx_Data, UART_Manage_Object->Rx_Buffer, 18 * sizeof(uint8_t));
}

/**
 * @brief TIM定时器中断定期检测遥控器是否存活
 *
 */
void Class_DR16::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过遥控器数据
    if (Flag == Pre_Flag)
    {
        // 遥控器断开连接
        DR16_Status = DR16_Status_DISABLE;
        // 遥控器中途断联导致错误，重启 DMA
        if (huart3.ErrorCode)
        {
            HAL_UART_DMAStop(&huart3); // 停止以重启
            // HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
        }
    }
    else
    {
        // 遥控器保持连接
        DR16_Status = DR16_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
