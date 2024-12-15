/**
 * @file dvc_referee.cpp
 * @author lez by yssickjgd
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_referee.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 裁判系统初始化
 *
 * @param __huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Class_Referee::Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header)
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

    Frame_Header = __Frame_Header;
}


/**
 * @brief 数据处理过程, 为节约性能不作校验但提供了接口
 * 如遇到大规模丢包或错乱现象, 可重新启用校验过程
 *
 */    
uint16_t buffer_index = 0;
uint16_t cmd_id,data_length;
void Class_Referee::Data_Process()
{
    buffer_index = 0; 
    uint16_t buffer_index_max = UART_Manage_Object->Rx_Buffer_Length - 1;
    while(buffer_index<buffer_index_max)
    {
        //通过校验和帧头
        if ((UART_Manage_Object->Rx_Buffer[buffer_index]==0xA5) && 
            (Verify_CRC8_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],5)==1))
        {
            //数据处理过程
            cmd_id=(UART_Manage_Object->Rx_Buffer[buffer_index+6])&0xff;
            cmd_id=(cmd_id<<8)|UART_Manage_Object->Rx_Buffer[buffer_index+5];  
            data_length=UART_Manage_Object->Rx_Buffer[buffer_index+2]&0xff;
            data_length=(data_length<<8)|UART_Manage_Object->Rx_Buffer[buffer_index+1];
            Enum_Referee_Command_ID CMD_ID = (Enum_Referee_Command_ID)cmd_id;
            int16_t remain_length = buffer_index_max-buffer_index;
            switch (CMD_ID)
            {
                case (Referee_Command_ID_GAME_STATUS):
                {   
                    if((remain_length>=(sizeof(Struct_Referee_Rx_Data_Game_Status)+7))&&
                    Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Game_Status)+7)==1)
                    {
                        memcpy(&Game_Status, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Game_Status));
                        buffer_index+=sizeof(Struct_Referee_Rx_Data_Game_Status)+7;
                    }
                }
                break;
                case (Referee_Command_ID_GAME_RESULT):
                {
                    if((buffer_index_max-buffer_index>=sizeof(Struct_Referee_Rx_Data_Game_Result)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Game_Result)+7)==1)
                        {
                            memcpy(&Game_Result, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Game_Result));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Game_Result)+7;
                        }
                }
                break;
                case (Referee_Command_ID_GAME_ROBOT_HP):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Game_Robot_HP)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Game_Robot_HP)+7)==1)
                        {
                            memcpy(&Game_Robot_HP, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Game_Robot_HP));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Game_Robot_HP)+7;
                        }
                }
                break;
                case (Referee_Command_ID_EVENT_DATA):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Event_Data)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Event_Data)+7)==1)
                        {
                            memcpy(&Event_Data, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Event_Data));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Event_Data)+7;
                        }
                }
                break;
                case (Referee_Command_ID_EVENT_SUPPLY):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Event_Supply)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Event_Supply)+7)==1)
                        {
                            memcpy(&Event_Supply, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Event_Supply));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Event_Supply)+7;
                        }
                }
                break;
                case (Referee_Command_ID_EVENT_REFEREE_WARNING):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning)+7)==1)
                        {
                            memcpy(&Event_Referee_Warning, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning)+7;
                        }
                }
                break;
                case (Referee_Command_ID_EVENT_DART_REMAINING_TIME):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Event_Dart_Remaining_Time)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Event_Dart_Remaining_Time)+7)==1)
                        {
                            memcpy(&Event_Dart_Remaining_Time, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Event_Dart_Remaining_Time));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Event_Dart_Remaining_Time)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_STATUS):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Status)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Status)+7)==1)
                        {
                            memcpy(&Robot_Status, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Status));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Status)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_POWER_HEAT):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat)+7)==1)
                        {
                            memcpy(&Robot_Power_Heat, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_POSITION):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Position)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Position)+7)==1)
                        {
                            memcpy(&Robot_Position, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Position));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Position)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_BUFF):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Buff)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Buff)+7)==1)
                        {
                            memcpy(&Robot_Buff, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Buff));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Buff)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_AERIAL_ENERGY):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Aerial_Energy)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Aerial_Energy)+7)==1)
                        {
                            memcpy(&Robot_Aerial_Energy, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Aerial_Energy));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Aerial_Energy)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_DAMAGE):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Damage)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Damage)+7)==1)
                        {
                            memcpy(&Robot_Damage, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Damage));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Damage)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_BOOSTER):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Booster)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Booster)+7)==1)
                        {
                            memcpy(&Robot_Booster, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Booster));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Booster)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_REMAINING_AMMO):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo)+7)==1)
                        {
                            memcpy(&Robot_Remaining_Ammo, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_RFID):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_RFID)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_RFID)+7)==1)
                        {
                            memcpy(&Robot_RFID, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_RFID));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_RFID)+7;
                        }
                }
                break;
                case (Referee_Command_ID_ROBOT_DART_COMMAND):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command)+7)==1)
                        {
                            memcpy(&Robot_Dart_Command, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command));
                            buffer_index+=sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command)+7;
                        }
                }
                break;
                case (Referee_Command_ID_INTERACTION_Client_RECEIVE):
                {
                    if(buffer_index_max-buffer_index>=(sizeof(Struct_Referee_Tx_Data_Interaction_Client_Receive)+7)&&
                        Verify_CRC16_Check_Sum(&UART_Manage_Object->Rx_Buffer[buffer_index],sizeof(Struct_Referee_Tx_Data_Interaction_Client_Receive)+7)==1)
                        {
                            memcpy(&Interaction_Client_Receive, &UART_Manage_Object->Rx_Buffer[buffer_index+7], sizeof(Struct_Referee_Tx_Data_Interaction_Client_Receive));
                            buffer_index+=sizeof(Struct_Referee_Tx_Data_Interaction_Client_Receive)+7;
                        }
                }
                break;
                default:
                break;
            }
        }
        buffer_index++;
    }
   
    // buffer_index += UART_Manage_Object->Rx_Length;
    // buffer_index %= UART_Manage_Object->Rx_Buffer_Length;
}


/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Referee::UART_RxCpltCallback(uint8_t *Rx_Data,uint16_t Length)
{
    //滑动窗口, 判断裁判系统是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测裁判系统是否存活
 *
 */
void Class_Referee::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过裁判系统数据
    if (Flag == Pre_Flag)
    {
        //裁判系统断开连接
        Referee_Status = Referee_Status_DISABLE;
    }
    else
    {
        //裁判系统保持连接
        Referee_Status = Referee_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

unsigned char Get_CRC8_Check_Sum(unsigned  char  *pchMessage,unsigned  int dwLength,unsigned char ucCRC8)
{
	unsigned char ucIndex;
	while (dwLength--)
	{
	ucIndex = ucCRC8^(*pchMessage++);
	ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return 0;
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ CRC16_Table[((uint16_t)(wCRC)^(uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return 0;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC16_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
	pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return;
	}
	wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC16_INIT );
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
