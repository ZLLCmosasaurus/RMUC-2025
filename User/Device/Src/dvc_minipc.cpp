/**
 * @file dvc_minipc.cpp
 * @author lez by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object* __USB_Manage_Object, Struct_UART_Manage_Object* __UART_Manage_Object, Struct_CAN_Manage_Object* __CAN_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
	  USB_Manage_Object = __USB_Manage_Object;
    UART_Manage_Object = __UART_Manage_Object;
    CAN_Manage_Object = __CAN_Manage_Object;
    Frame_Header = __frame_header;
    Frame_Rear = __frame_rear;
}
/**
 * @brief 数据处理过程
 *
 */
int head = 0;
uint8_t temp_data[128];
int PACKET_LEN  = sizeof(Struct_MiniPC_Rx_Data);
void Class_MiniPC::Data_Process(Enum_MiniPC_Data_Source Data_Source)
{
  if (Data_Source == USB)
  {
    if(!Verify_CRC16_Check_Sum(USB_Manage_Object->Rx_Buffer,USB_Manage_Object->Rx_Buffer_Length)) return;
    memcpy(&Data_NUC_To_MCU, USB_Manage_Object->Rx_Buffer, PACKET_LEN);
    Auto_aim(float(Data_NUC_To_MCU.Gimbal_Target_X_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Z_A / 100.f), &Rx_Angle_Yaw_A, &Rx_Angle_Pitch_A, &Distance_A, Booster_Type_A);
    Auto_aim(float(Data_NUC_To_MCU.Gimbal_Target_X_B / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_B / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Z_B / 100.f), &Rx_Angle_Yaw_B, &Rx_Angle_Pitch_B, &Distance_B, Booster_Type_B);
    Rx_Angle_Yaw_Main = calc_yaw(float(Data_NUC_To_MCU.Gimbal_Target_X_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_A / 100.f) + 13.5f, float(Data_NUC_To_MCU.Gimbal_Target_Z_A / 100.f));
  }
  else if (Data_Source == UART)
  {
    // 解包部分
    int Header_Flag = -1;
    for (int step = 0; step < 128; step++) {
        int i = (head + step) % 128;
        if (UART_Manage_Object->Rx_Buffer[i] == 0xA5) {
            Header_Flag = i;
            break;
        }
    }
    if (Header_Flag < 0) return;// 没找到包头
    for(int k = 0; k < sizeof(Struct_MiniPC_Rx_Data); k++)
    {
      temp_data[k] = UART_Manage_Object->Rx_Buffer[(Header_Flag + k) % 128];
    }    
    if(!Verify_CRC16_Check_Sum(temp_data, sizeof(Struct_MiniPC_Rx_Data))) return;
    memcpy(&Data_NUC_To_MCU, temp_data, sizeof(Struct_MiniPC_Rx_Data));
    head = (Header_Flag + PACKET_LEN) % 128;

    Auto_aim(float(Data_NUC_To_MCU.Gimbal_Target_X_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Z_A / 100.f), &Rx_Angle_Yaw_A, &Rx_Angle_Pitch_A, &Distance_A, Booster_Type_A);
    Auto_aim(float(Data_NUC_To_MCU.Gimbal_Target_X_B / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_B / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Z_B / 100.f), &Rx_Angle_Yaw_B, &Rx_Angle_Pitch_B, &Distance_B, Booster_Type_B);
    Rx_Angle_Yaw_Main = calc_yaw(float(Data_NUC_To_MCU.Gimbal_Target_X_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_A / 100.f) + 13.5f, float(Data_NUC_To_MCU.Gimbal_Target_Z_A / 100.f));
  }
  else if (Data_Source == CAN)
  {
    // CAN数据包处理
    switch(CAN_Manage_Object->Rx_Buffer.Header.Identifier)
    {
        case (0x104):
        {
          memcpy(&Rx_A,CAN_Manage_Object->Rx_Buffer.Data, sizeof(Rx_A));
          Data_NUC_To_MCU.Chassis_Angular_Velocity_Yaw = Rx_A.Chassis_Angular_Velocity_Yaw;
          Data_NUC_To_MCU.MiniPC_To_Chassis_Target_Velocity_X = Rx_A.MiniPC_To_Chassis_Target_Velocity_X;
          Data_NUC_To_MCU.MiniPC_To_Chassis_Target_Velocity_Y = Rx_A.MiniPC_To_Chassis_Target_Velocity_Y;
          Data_NUC_To_MCU.Gimbal_Angular_Velocity_Yaw_Main = Rx_A.Gimbal_Angular_Velocity_Yaw_Main;
          break;
        }
        case (0x105):
        {
          memcpy(&Rx_B,CAN_Manage_Object->Rx_Buffer.Data, sizeof(Rx_B));
          Data_NUC_To_MCU.Gimbal_Angular_Velocity_Yaw_A = Rx_B.Gimbal_Angular_Velocity_Yaw_A;
          Data_NUC_To_MCU.Gimbal_Angular_Velocity_Pitch_A = Rx_B.Gimbal_Angular_Velocity_Pitch_A;
          Data_NUC_To_MCU.Gimbal_Angular_Velocity_Yaw_B = Rx_B.Gimbal_Angular_Velocity_Yaw_B;
          Data_NUC_To_MCU.Gimbal_Angular_Velocity_Pitch_B = Rx_B.Gimbal_Angular_Velocity_Pitch_B;
          break;
        }
        case (0x106):
        {
          memcpy(&Rx_C,CAN_Manage_Object->Rx_Buffer.Data, sizeof(Rx_C));
          Data_NUC_To_MCU.Gimbal_Target_X_A = Rx_C.Gimbal_Target_X_A;
          Data_NUC_To_MCU.Gimbal_Target_Y_A = Rx_C.Gimbal_Target_Y_A;
          Data_NUC_To_MCU.Gimbal_Target_Z_A = Rx_C.Gimbal_Target_Z_A;
          Data_NUC_To_MCU.Control_Type_A = Rx_C.Control_Type_A;
          Data_NUC_To_MCU.Control_Type_B = Rx_C.Control_Type_B;
          Auto_aim(float(Data_NUC_To_MCU.Gimbal_Target_X_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_A / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Z_A / 100.f), &Rx_Angle_Yaw_A, &Rx_Angle_Pitch_A, &Distance_A, Booster_Type_A);
          break;
        }
        case (0x107):
        {
          memcpy(&Rx_D,CAN_Manage_Object->Rx_Buffer.Data, sizeof(Rx_D));
          Data_NUC_To_MCU.Gimbal_Target_X_B = Rx_D.Gimbal_Target_X_B;
          Data_NUC_To_MCU.Gimbal_Target_Y_B = Rx_D.Gimbal_Target_Y_B;
          Data_NUC_To_MCU.Gimbal_Target_Z_B = Rx_D.Gimbal_Target_Z_B;
          Data_NUC_To_MCU.Chassis_Control_Mode = Rx_D.Chassis_Control_Mode;
          Data_NUC_To_MCU.Device_Mode = Rx_D.Device_Mode;
          Auto_aim(float(Data_NUC_To_MCU.Gimbal_Target_X_B / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Y_B / 100.f), float(Data_NUC_To_MCU.Gimbal_Target_Z_B / 100.f), &Rx_Angle_Yaw_B, &Rx_Angle_Pitch_B, &Distance_B, Booster_Type_B);
          break;
        }
    }
  }
}

/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
extern Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
extern Referee_Rx_C_t CAN3_Chassis_Rx_Data_C;
extern Referee_Rx_D_t CAN3_Chassis_Rx_Data_D;
volatile int index = 0;
void Class_MiniPC::Output()
{
  //
	Data_MCU_To_NUC.header                         = Frame_Header;
  Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle_A       = int16_t((Now_Angle_Pitch_A) * 100);
  Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle_A         = int16_t( Now_Angle_Yaw_A * 100);
  Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle_B       = int16_t((Now_Angle_Pitch_B) * 100);
  Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle_B         = int16_t( Now_Angle_Yaw_B * 100);
  Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle_Main      = int16_t( IMU->Get_Angle_Yaw() * 100);
  Data_MCU_To_NUC.Chassis_Now_yaw_Angle          = int16_t((IMU->Get_Angle_Yaw() + Now_Angle_Relative) * 100);
  Data_MCU_To_NUC.Game_process                   = CAN3_Chassis_Rx_Data_A.game_process;
  Data_MCU_To_NUC.Self_blood                     = CAN3_Chassis_Rx_Data_A.self_blood;
  Data_MCU_To_NUC.Self_Outpost_HP                = CAN3_Chassis_Rx_Data_A.self_outpost_HP;
  Data_MCU_To_NUC.Remaining_Time                 = CAN3_Chassis_Rx_Data_A.remaining_time;
  Data_MCU_To_NUC.Oppo_Outpost_HP                = CAN3_Chassis_Rx_Data_B.oppo_outpost_HP;
  Data_MCU_To_NUC.Self_Base_HP                   = CAN3_Chassis_Rx_Data_B.self_base_HP;   
  Data_MCU_To_NUC.Color_Invincible_State         = CAN3_Chassis_Rx_Data_A.color_invincible_state << 7 | CAN3_Chassis_Rx_Data_A.color_invincible_state << 5;
  Data_MCU_To_NUC.Projectile_allowance           = CAN3_Chassis_Rx_Data_B.projectile_allowance_17mm;
  // 有发送优先级版本，暂时停用(测试过，可用)
  // if(CAN3_Chassis_Rx_Data_D.Hero_Position_X != 0 || CAN3_Chassis_Rx_Data_D.Hero_Position_Y != 0)
  // {
  //     Data_MCU_To_NUC.Robot_Position_X = 0x00 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Hero_Position_X / 255.f) * 16384.f);
  //     Data_MCU_To_NUC.Robot_Position_Y = 0x00 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Hero_Position_Y / 255.f) * 16384.f);
  // }
  // else if(CAN3_Chassis_Rx_Data_D.Infantry_3_Position_X != 0 || CAN3_Chassis_Rx_Data_D.Infantry_3_Position_Y != 0)
  // {
  //     Data_MCU_To_NUC.Robot_Position_X = 0x02 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_3_Position_X / 255.f) * 16384.f);
  //     Data_MCU_To_NUC.Robot_Position_Y = 0x02 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_3_Position_Y / 255.f) * 16384.f);
  // }
  // else if (CAN3_Chassis_Rx_Data_D.Infantry_4_Position_X != 0 || CAN3_Chassis_Rx_Data_D.Infantry_4_Position_Y != 0)
  // {
  //     Data_MCU_To_NUC.Robot_Position_X = 0x03 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_4_Position_X / 255.f) * 16384.f);
  //     Data_MCU_To_NUC.Robot_Position_Y = 0x03 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_4_Position_Y / 255.f) * 16384.f);
  // }
  // else if(CAN3_Chassis_Rx_Data_D.Sentry_Position_X != 0 || CAN3_Chassis_Rx_Data_D.Sentry_Position_Y != 0)
  // {
  //     Data_MCU_To_NUC.Robot_Position_X = 0x01 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Sentry_Position_X / 255.f) * 16384.f);
  //     Data_MCU_To_NUC.Robot_Position_Y = 0x01 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Sentry_Position_Y / 255.f) * 16384.f);
  // }
  // else
  // {
  //     Data_MCU_To_NUC.Robot_Position_X = 0;
  //     Data_MCU_To_NUC.Robot_Position_Y = 0;
  // }

  //顺序发送版本
  switch(index)
  {
    case 0:
    {
      Data_MCU_To_NUC.Robot_Position_X = 0x00 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Hero_Position_X / 255.f) * 16384.f);
      Data_MCU_To_NUC.Robot_Position_Y = 0x00 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Hero_Position_Y / 255.f) * 16384.f);
      break;
    }
    case 1:
    {
      Data_MCU_To_NUC.Robot_Position_X = 0x02 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_3_Position_X / 255.f) * 16384.f);
      Data_MCU_To_NUC.Robot_Position_Y = 0x02 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_3_Position_Y / 255.f) * 16384.f);
      break;
    }
    case 2:
    {
      Data_MCU_To_NUC.Robot_Position_X = 0x03 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_4_Position_X / 255.f) * 16384.f);
      Data_MCU_To_NUC.Robot_Position_Y = 0x03 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Infantry_4_Position_Y / 255.f) * 16384.f);
      break;
    }
    case 3:
    {
      Data_MCU_To_NUC.Robot_Position_X = 0x01 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Sentry_Position_X / 255.f) * 16384.f);
      Data_MCU_To_NUC.Robot_Position_Y = 0x01 << 14 | (uint16_t)(float(CAN3_Chassis_Rx_Data_D.Sentry_Position_Y / 255.f) * 16384.f);
      break;
    }
  }
  Data_MCU_To_NUC.crc16                          = 0xffff;

  //USB通信
	memcpy(USB_Manage_Object->Tx_Buffer, &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
  USB_Manage_Object->Tx_Buffer_Length = sizeof(Struct_MiniPC_Tx_Data);
  //crc16 校验
  Append_CRC16_Check_Sum(USB_Manage_Object->Tx_Buffer, sizeof(Struct_MiniPC_Tx_Data));

  //UART通信
  memcpy(UART_Manage_Object->Tx_Buffer, &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
  UART_Manage_Object->Tx_Buffer_Length = sizeof(Struct_MiniPC_Tx_Data);
  //crc校验
  Append_CRC16_Check_Sum(UART_Manage_Object->Tx_Buffer, sizeof(Struct_MiniPC_Tx_Data));

  //CAN通信
  Tx_A.Gimbal_Now_Pitch_Angle_A = Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle_A;
  Tx_A.Gimbal_Now_Yaw_Angle_A   = Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle_A;
  Tx_A.Gimbal_Now_Pitch_Angle_A = Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle_A;
  Tx_A.Gimbal_Now_Yaw_Angle_B   = Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle_B;
  Tx_B.Gimbal_Now_Pitch_Angle_B = Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle_B;
  Tx_B.Chassis_Now_yaw_Angle    = Data_MCU_To_NUC.Chassis_Now_yaw_Angle;
  Tx_B.Self_blood               = Data_MCU_To_NUC.Self_blood;
  Tx_B.Self_Outpost_HP          = Data_MCU_To_NUC.Self_Outpost_HP;
  Tx_C.Oppo_Outpost_HP          = Data_MCU_To_NUC.Oppo_Outpost_HP;
  Tx_C.Projectile_allowance     = Data_MCU_To_NUC.Projectile_allowance;
  Tx_C.Remaining_Time           = Data_MCU_To_NUC.Remaining_Time;  
  Tx_C.Self_Base_HP             = Data_MCU_To_NUC.Self_Base_HP;
  Tx_D.Color_Invincible_State   = Data_MCU_To_NUC.Color_Invincible_State;
  Tx_D.Robot_Position_X         = Data_MCU_To_NUC.Robot_Position_X;
  Tx_D.Robot_Position_Y         = Data_MCU_To_NUC.Robot_Position_Y;
  Tx_D.Game_process             = Data_MCU_To_NUC.Game_process;  
  memcpy(CAN3_MiniPC_Tx_Data_A, &Tx_A, sizeof(MiniPC_Tx_A_t));
  memcpy(CAN3_MiniPC_Tx_Data_B, &Tx_B, sizeof(MiniPC_Tx_B_t));
  memcpy(CAN3_MiniPC_Tx_Data_C, &Tx_C, sizeof(MiniPC_Tx_C_t));
  memcpy(CAN3_MiniPC_Tx_Data_D, &Tx_D, sizeof(MiniPC_Tx_D_t));

  //重新排序
  index++;
  if(index == 4)index = 0;
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
  Output();
}

/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(USB);
}

/**
 * @brief uart通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::UART_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(UART);
}

/**
 * @brief can通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::CAN_RxCpltCallback()
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(CAN);
}



/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        //迷你主机断开连接
        MiniPC_Status =  MiniPC_Status_DISABLE;
    }
    else
    {
        //迷你主机保持连接
        MiniPC_Status =  MiniPC_Status_ENABLE ;
    }

    Pre_Flag = Flag;
}

/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

/**
 * 计算给定向量的偏航角（yaw）。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float Class_MiniPC::calc_yaw(float x, float y, float z) 
{
    // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
    float yaw = atan2f(y, x);

    // 将弧度制的偏航角转换为角度制
    yaw = (yaw * 180 / PI); // 向左为正，向右为负

    return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float Class_MiniPC::calc_distance(float x, float y, float z) 
{
    // 计算各分量的平方和，并取其平方根得到欧几里德距离
    float distance = sqrtf(x * x + y * y + z * z);

    return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */
extern Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
float Class_MiniPC::calc_pitch(float x, float y, float z, Enum_Booster_Type Booster_Type) 
{
  // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
  float pitch = atan2f(z, sqrtf(x * x + y * y));
  // if(Booster_Type == Booster_Type_A)
  // {
  //   bullet_v = CAN3_Chassis_Rx_Data_E.Bullet_Speed_A;
  // }
  // else if(Booster_Type == Booster_Type_B)
  // {
  //   bullet_v = CAN3_Chassis_Rx_Data_E.Bullet_Speed_B;
  // }
  //使用重力加速度模型迭代更新俯仰角
  for (size_t i = 0; i < 20; i++) {
    float v_x = bullet_v * cosf(pitch);
    float v_y = bullet_v * sinf(pitch);
    // 计算子弹飞行时间
    float t = sqrtf(x * x + y * y) / v_x;
    float h = v_y * t - 0.5 * g * t * t;
    float dz = z - h;

    if (abs(dz) < 0.01) 
    {
      break;
    }
    // 根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
    pitch += asinf(dz / calc_distance(x, y, z));
  }

  // 将弧度制的俯仰角转换为角度制
  pitch = -(pitch * 180 / PI); // 向上为负，向下为正

  return pitch;
}

/**
 * 计算当前瞄准点与目标点的偏差 (映射到同一球面的弦长)
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @param now_yaw   实际yaw轴角度（弧度制）
 * @param now_pitch 实际pitch轴角度（弧度制）
 * @return 偏差值
 */
float Class_MiniPC::Calc_Error(float x, float y, float z, float now_yaw, float now_pitch)
{
    float dis = sqrtf(x*x + y*y + z*z);
    float x0 = dis*cos(now_pitch)*cos(now_yaw);
    float y0 = dis*cos(now_pitch)*sin(now_yaw);
    float z0 = dis*sin(now_pitch);
    float err = sqrtf(pow(x-x0,2) + pow(y-y0,2) + pow(z-z0,2));
    return err;
}

/**
 * 计算计算yaw，pitch
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */
void Class_MiniPC::Auto_aim(float x, float y, float z, float *yaw, float *pitch, float *distance, Enum_Booster_Type Booster_Type)
{
    *yaw = calc_yaw(x, y, z);//第一次标- 现改为正
    *pitch = calc_pitch(x, y, z, Booster_Type);//第一次标- 现改为正
    *distance = calc_distance(x, y, z);
     //这里的z为上位机直接发的z，不是弹道解算后的z1，判断时存在一定误差（瞄准误差允许范围为5cm时，不影响10m内打弹）
}

float Class_MiniPC::meanFilter(float input) 
{
    static float buffer[5] = {0};
    static uint64_t index = 0;
    float sum = 0;

    // Replace the oldest value with the new input value
    buffer[index] = input;

    // Increment the index, wrapping around to the start of the array if necessary
    index = (index + 1) % 5;

    // Calculate the sum of the buffer's values
    for(int i = 0; i < 5; i++) {
        sum += buffer[i];
    }

    // Return the mean of the buffer's values
    return sum / 5.0;
}

/************************ copyright(c) ustc-robowalker **************************/
