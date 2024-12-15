/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
// 底盘速度指令更新
void Chassis_Speed_Update(void)
{
    // switch (chassis.send.mode)
    // {
    // case CHASSIS_MODE_TOPANGLE:
    // case CHASSIS_MODE_ABSOLUTE:
    // case CHASSIS_MODE_PRECISE:
    //     chassis.send.velocity.vx = RC.rc_sent.x_speed;
    //     chassis.send.velocity.vy = RC.rc_sent.y_speed;
    //     chassis.send.velocity.vw = RC.rc_sent.r_speed;
    //     break;
    // case CHASSIS_MODE_NOFORCE:
    //     chassis.send.velocity.vx = 0;
    //     chassis.send.velocity.vy = 0;
    //     chassis.send.velocity.vw = 0;
    //     break;
    // }
}

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
#ifdef CHASSIS

    // 裁判系统
    Referee.Init(&huart6);

    // 底盘
    Chassis.Referee = &Referee;
    Chassis.Init(10.0f, 10.0f, 2.0f);

    // 底盘随动PID环初始化
    PID_Chassis_Fllow.Init(20.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.05f);

#ifdef POWER_LIMIT
    // 串口超电
    Supercap.Init_UART(&huart1);
#endif

#elif defined(GIMBAL)

    Chassis.Set_Velocity_X_Max(10.0f);
    Chassis.Set_Velocity_Y_Max(10.0f);

    // 遥控器
    DR16.Init(&huart3);
    DR16_Dead_Zone = __DR16_Dead_Zone;

    // 云台
    Gimbal.Init();

    // 发射机构
    Booster.Referee = &Referee;
    Booster.Init();

    // 上位机
    MiniPC.Init(&MiniPC_USB_Manage_Object);
    MiniPC.IMU = &Gimbal.Boardc_BMI;
    MiniPC.Referee = &Referee;
    MiniPC.Gimbal_Pitch_Motor_GM6020 = &Gimbal.Motor_Pitch;
    MiniPC.Gimbal_Yaw_Motor_GM6020 = &Gimbal.Motor_Yaw;

    // select the MiniPC object
    Gimbal.MiniPC = &MiniPC;

#endif
}

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Control_RxCpltCallback()
{
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 底盘和云台夹角（弧度制）
    float derta_angle;
    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y;
    // 底盘控制类型
    Enum_Chassis_Control_Type tmp_chassis_control_type;
    // 底盘角速度
    float target_omega;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(int16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(int16_t));
    memcpy(&tmp_chassis_control_type, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(uint8_t));
    // Math_Endian_Reverse_16((void *)CAN_Manage_Object->Rx_Buffer.Data[0], (void *)&tmp_velocity_x);
    // Math_Endian_Reverse_16((void *)CAN_Manage_Object->Rx_Buffer.Data[2], (void *)&tmp_velocity_y);

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());

    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    Chassis_Angle = Chassis.Motor_Yaw.Get_Now_Angle();
    derta_angle = Chassis_Angle - Reference_Angle;

    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));

    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(tmp_chassis_control_type);

    // 底盘控制方案
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        target_omega = Chassis.Get_Spin_Omega();
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
        Chassis.PID_Chassis_Fllow.Set_Target(Reference_Angle);
        Chassis.PID_Chassis_Fllow.Set_Now(Chassis.Motor_Yaw.Get_Now_Angle());
        Chassis.PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        target_omega = Chassis.PID_Chassis_Fllow.Get_Out();
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        target_omega = 0;
    }
    else
    {
        target_omega = 0;
    }

    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Omega(-target_omega);
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_RxCpltCallback()
{
}

float gimbal_velocity_x = 0, gimbal_velocity_y = 0;

float testtt = 0;
/**
 * @brief can云台向底盘发送数据
 *
 */
void Class_Chariot::CAN_Gimbal_TxCpltCallback()
{
    // 云台坐标系速度目标值 float

    float relative_angle = 0;

    int chassis_velocity_x = 0, chassis_velocity_y = 0, chassis_velocity_w = 0;

    float gimbal_angle = 0, chassis_angle = 0;

    float tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0;
    // 设定速度

    gimbal_velocity_x = Gimbal.Get_Gimbal_Speed_X();
    gimbal_velocity_y = Gimbal.Get_Gimbal_Speed_Y();

    gimbal_angle = Gimbal.Get_Gimbal_Head_Angle();
    chassis_angle = Gimbal.Motor_Yaw.Get_Now_Angle();
    relative_angle = gimbal_angle - chassis_angle;
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
        relative_angle += PI / 8;
    testtt = relative_angle / PI * 180;
    // 测试，假设没有相对角度
    // relative_angle = 0;

    tmp_chassis_velocity_x = gimbal_velocity_x * cos(relative_angle) + gimbal_velocity_y * sin(relative_angle);
    tmp_chassis_velocity_y = -gimbal_velocity_x * sin(relative_angle) + gimbal_velocity_y * cos(relative_angle);

    if (DR16.Get_DR16_Status() != DR16_Status_DISABLE && DR16.Get_Left_Switch() == DR16_Switch_Status_UP)
    {
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ANTI_SPIN);
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_MIDDLE)
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
    }

    //    memcpy(CAN2_0x150_Tx_Data + 4, &chassis_velocity_w, sizeof(int16_t));

    memcpy(CAN2_0x150_Tx_Data, &tmp_chassis_velocity_x, sizeof(float));
    memcpy(CAN2_0x150_Tx_Data + 4, &tmp_chassis_velocity_y, sizeof(float));

    CAN2_0x152_Tx_Data[0] = Chassis.Get_Chassis_Control_Type();
    //		CAN2_0x152_Tx_Data[0] =0	;
    CAN2_0x152_Tx_Data[1] = MiniPC.Get_Vision_Mode();
    CAN2_0x152_Tx_Data[2] = Chassis.Get_Supercap_State();
    CAN2_0x152_Tx_Data[3] = Booster.Get_Booster_Control_Type(); // 摩擦轮状态
    CAN2_0x152_Tx_Data[4] = Gimbal.Get_Gimbal_Control_Type();   // 云台状态
    CAN2_0x152_Tx_Data[5] = MiniPC.Get_MiniPC_Status();
    CAN2_0x152_Tx_Data[6] = Booster.Get_Booster_Jamming_Type();
    CAN2_0x152_Tx_Data[7] = Chassis.Get_Chassis_UI_Init_flag();

    // CAN2_0x153_Tx_Data[0] = Gimbal.Get_Gimbal_Control_Type();//云台状态
    // 		//CAN2_0x153_Tx_Data[0] =0	;
    // CAN2_0x153_Tx_Data[1] = Booster.Get_Booster_Control_Type();//摩擦轮状态
    // CAN2_0x153_Tx_Data[2] = Chassis.Get_Chassis_Control_Type();//底盘状态
    // CAN2_0x153_Tx_Data[2] = 0;
    // CAN2_0x153_Tx_Data[3] = 0;
}

#endif
/**
 * @brief 底盘控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y;
    // 云台坐标系速度目标值 float
    float gimbal_velocity_x = 0, gimbal_velocity_y = 0;

    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE) // 失能
    {
        // 遥控器离线或左下方失能
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }
    else
    {
        // 遥控器键盘鼠标操作逻辑,右下键鼠操作
        if ((DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN))
        {
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);

            if (DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_TRIG_FREE_PRESSED) // Q键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }

            if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED) // x轴
            {
                gimbal_velocity_x -= Chassis.Get_Velocity_X_Max();
            }
            if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
            {
                gimbal_velocity_x += Chassis.Get_Velocity_X_Max();
            }
            if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
            {
                gimbal_velocity_y += Chassis.Get_Velocity_Y_Max();
            }
            if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
            {
                gimbal_velocity_y -= Chassis.Get_Velocity_Y_Max();
            }
            if (DR16.Get_Keyboard_Key_E() == DR16_Key_Status_PRESSED)
            {
                Chassis.Set_Chassis_UI_Init_flag(UI_INIT_ON);
            }
            else
            {
                Chassis.Set_Chassis_UI_Init_flag(UI_INIT_OFF);
            }

            if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED)
            {
                Chassis.Set_Supercap_State(SUPERCAP_ON);
            }
            else
            {
                Chassis.Set_Supercap_State(SUPERCAP_OFF);
            }
        }
        else
        {
            // 遥控器操作逻辑

            // 排除遥控器死区
            dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
            dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

            // 设定矩形到圆形映射进行控制
            gimbal_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
            gimbal_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

            // 键盘遥控器操作逻辑
            if ((DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)) // 左中随动
            {
                // 底盘随动
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }
            if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上小陀螺模式
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            }
//                        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
//                        {
//                           gimbal_velocity_y=1;
//            
//                       }
//            
//                       if(DR16.Get_Right_Switch() ==DR16_Switch_Status_MIDDLE)
//                       {
//                           gimbal_velocity_y=0;
//            							gimbal_velocity_x=0;
//                       }
        }
    }

    Gimbal.Set_Gimbal_Speed_X(gimbal_velocity_x);
    Gimbal.Set_Gimbal_Speed_Y(gimbal_velocity_y);
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Gimbal()
{
    // 角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch;
    // 遥控器摇杆值
    float dr16_y, dr16_r_y;
    // 获取当前角度值

    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();

    // 排除遥控器死区
    dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
    dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

    // 按F切换期望来源

    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE) // 失能
    {
        // 遥控器离线或下方失能
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
    }
    else
    {

        if ((DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN))
        {
            if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED)
            {
                if (DR16.Get_Keyboard_Key_V() == DR16_Key_Status_PRESSED)
                {
                    if (MiniPC.Get_Vision_Mode() == ARMOR_MODE)
                        MiniPC.Set_Vision_Mode(WINDMILL_MODE);
                    else
                        MiniPC.Set_Vision_Mode(ARMOR_MODE);
                }
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            }
            else if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_FREE)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            // 键盘遥控器操作逻辑
            // 切换瞄准模式
            if (DR16.Get_Keyboard_Key_F() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL)
                {
                    Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

                    // 防止切换状态时云台角度突变

                    tmp_gimbal_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
                    tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
                }
                else
                {
                    Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

                    // 防止切换状态时云台角度突变
                    tmp_gimbal_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
                    tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
                }
            }
            else
            {
                if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC && MiniPC.Get_MiniPC_Status() == MiniPC_Data_Status_ENABLE)
                {
                    // if (MiniPC.Get_MiniPC_Status() == MiniPC_Data_Status_DISABLE)
                    // {

                    //     tmp_gimbal_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
                    //     tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
                    //     tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
                    //     tmp_gimbal_pitch -= DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
                    // }
                    // else
                    {
                        tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                        tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                        // 键盘遥控器操作逻辑
                        tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution * 10;
                        tmp_gimbal_pitch -= DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution * 10;
                    }
                }
                if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL || MiniPC.Get_MiniPC_Status() == MiniPC_Data_Status_DISABLE)
                {
                    // 键盘遥控器操作逻辑
                    tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
                    tmp_gimbal_pitch -= DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
                }
            }
        }
        else
        {
            // 左下上位机
            if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
            {
                if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL)
                {

                    // 防止切换状态时云台角度突变
                    tmp_gimbal_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
                    tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
                }
                else
                {
                    tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                    tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                }
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

                // 遥控器操作逻辑
                tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * 20;
                tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Resolution * 20;
            }
            else
            {
                if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC)
                {

                    // 防止切换状态时云台角度突变
                    tmp_gimbal_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
                    tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
                }
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

                // 遥控器操作逻辑
                tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * 10;
                tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Resolution * 10;
            }
        }
    }

    // 设定角度

    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
}

/*上位机和正常公用云台的tmp，*/
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */

#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    // Booster.Set_Driver_Omega(Booster.Get_Default_Driver_Omega());

    // 键鼠操作
    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
    {
        // 遥控器离线或下方失能
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);

        return;
    }
    else
    {
        if ((DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN))
        {
            if (DR16.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Get_Booster_Control_Type() == Booster_Control_Type_DISABLE)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                else
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            }
            // 切换连发模式
            if (DR16.Get_Keyboard_Key_V() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Get_Booster_User_Control_Type() == Booster_User_Control_Type_SINGLE)
                    Booster.Set_Booster_User_Control_Type(Booster_User_Control_Type_MULTI);
                else
                    Booster.Set_Booster_User_Control_Type(Booster_User_Control_Type_SINGLE);
            }

            if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Get_Booster_User_Control_Type() == Booster_User_Control_Type_SINGLE)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                if (Booster.Get_Booster_User_Control_Type() == Booster_User_Control_Type_MULTI)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            }

            if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_PRESSED)
            {
                if (Booster.Get_Booster_User_Control_Type() == Booster_User_Control_Type_SINGLE)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                if (Booster.Get_Booster_User_Control_Type() == Booster_User_Control_Type_MULTI)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            }
            // else if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_PRESSED_FREE)
            // {
            //     Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            // }
        }
        else
        {
            // if (DR16.Get_Wheel() > 0.9)
            //     Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            // if (DR16.Get_Wheel() < -0.9)
            //     Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);

            // if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_UP)
            //     Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);

            // if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_UP_MIDDLE)
            //     Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        }
    }
}

#endif

/**
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS

    // 底盘的控制策略
    //        Control_Chassis();
    // 各个模块的分别解算
    Chassis.TIM_Calculate_PeriodElapsedCallback();

    Supercap.Set_Now_Power(Chassis.Referee->Get_Chassis_Power());
    Supercap.Set_Limit_Power(Chassis.Referee->Get_Chassis_Power_Max());
    Supercap.TIM_UART_PeriodElapsedCallback();

#elif defined(GIMBAL)

    // 各个模块的分别解算

    Gimbal.TIM_Calculate_PeriodElapsedCallback();

    Booster.TIM_Calculate_PeriodElapsedCallback();
    // 传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();

    this->CAN_Gimbal_TxCpltCallback();

#endif
}

/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
}
#endif
/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
#ifdef CHASSIS

        Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

#ifdef POWER_LIMIT
        Supercap.TIM_Alive_PeriodElapsedCallback();
#endif

        for (auto &wheel : Chassis.Motor_Wheel)
        {
            wheel.TIM_Alive_PeriodElapsedCallback();
        }

#elif defined(GIMBAL)

        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Pitch_LK6010.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();

        MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

#endif

        mod50 = 0;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
