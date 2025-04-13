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
#include "dvc_GraphicsSendTask.h"
#include "config.h"
/* Private macros ------------------------------------------------------------*/
// 机器人控制对象
Class_Chariot chariot;
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

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
    Chassis.Init(Chassis_Velocity_Max,Chassis_Velocity_Max);

    // 底盘随动PID环初始化
    PID_Chassis_Fllow.Init(3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.001f);

    // yaw电机canid初始化  只获取其编码器值用于底盘随动，并不参与控制
    Motor_Yaw.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2);

    //底盘缓冲能量处理
    PID_Chassis_Buffer_Power.Init(2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 45.0f, 0.0f, 0.0f, 0.0f, 0.001f);
#elif defined(GIMBAL)

    // 遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);

    // 遥控器
    DR16.Init(&huart3, &huart6);
    DR16_Dead_Zone = __DR16_Dead_Zone;

    //底盘初始化限制速度
    Chassis.Init(Chassis_Velocity_Max,Chassis_Velocity_Max);

    // 云台
    Gimbal.Init();
    Gimbal.MiniPC = &MiniPC;

    // 发射机构
    Booster.Referee = &Referee;
    Booster.Init();

    //图传
    Image.Init();
    
    // 上位机
    MiniPC.Init(&MiniPC_USB_Manage_Object);
    MiniPC.IMU = &Gimbal.Boardc_BMI;
    MiniPC.Referee = &Referee;
    MiniPC.Init_UART(&UART6_Manage_Object);
#endif
}

#ifdef CHASSIS
float Class_Chariot::Get_Chassis_Coordinate_System_Angle_Rad()
{
    float GM6020_Angle_Rad = ((float)Motor_Yaw.Get_Now_Total_Encoder()) / 8191 * 2 * PI / 2;
    float Yaw_Angle_Rad = fabsf(fmodf(GM6020_Angle_Rad, 2.0f * PI));
    if(Motor_Yaw.Get_Now_Total_Round() < 0)
        Yaw_Angle_Rad = 2.0f * PI - Yaw_Angle_Rad;
    if(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= 2 * PI;//得到角度范围为[-PI,PI]
    
    Yaw_Angle_Rad -=  Reference_Angle;

    while(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= PI * 2.0f;
    while(Yaw_Angle_Rad < -PI)
        Yaw_Angle_Rad += PI * 2.0f;
    
    return (Yaw_Angle_Rad);
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS
// 控制类型字节
uint8_t control_type;
float fps;
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    //fps = FPS_Counter_Update();
    Gimbal_Alive_Flag++;
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 目标角速度
    //float chassis_omega;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    // 底盘和云台夹角（弧度制）
    float derta_angle;
    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y,tmp_gimbal_pitch;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    
    memcpy(&tmp_gimbal_pitch, &CAN_Manage_Object->Rx_Buffer.Data[5], sizeof(uint16_t));
    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[7], sizeof(uint8_t));

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    if(fabs(gimbal_velocity_x) < 0.0002f)
        gimbal_velocity_x = 0.0f;
    if(fabs(gimbal_velocity_y) < 0.0002f)
        gimbal_velocity_y = 0.0f;

    float Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch, 0, 0x7FFF, -50.f, 50.f);
    Set_Gimbal_Pitch_Angle(Gimbal_Tx_Pitch_Angle);
    chassis_control_type = (Enum_Chassis_Control_Type)(control_type & 0x03);
    Chassis_Logics_Direction = (Enum_Chassis_Logics_Direction)(control_type >> 2 & 0x01);
    Bulletcap_Status = (Enum_Bulletcap_Status)(control_type >> 3 & 0x01);
    Fric_Status = (Enum_Fric_Status)(control_type >> 4 & 0x01);
    Supercap_Control_Status = (Enum_Supercap_Control_Status)(control_type >> 5 & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)(control_type >> 6 & 0x01);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)(control_type >> 7 & 0x01);
    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);
    //小陀螺补偿角度
    if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Positive||
        Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Negative)
    {
        //Offset_Angle = fabs(Motor_Yaw.Get_Now_Omega_Radian()/2.0f) * 0.01f;
        Offset_Angle = 15.0f * DEG_TO_RAD;
    }
    else
    {
        Offset_Angle = 0.0f;
    }
    //
    
    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    Chassis_Angle = Chassis_Coordinate_System_Angle_Rad;
    derta_angle = (Reference_Angle-Reference_Angle) - Chassis_Angle + Offset_Angle;
    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));
    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    // Chassis.Set_Target_Omega(chassis_omega);
}

void Class_Chariot::Control_Chassis_Omega_TIM_PeriodElapsedCallback()
{
	
    // 目标角速度
    float chassis_omega;

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
        chassis_omega = Chassis.Get_Spin_Omega();
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Negative)
        chassis_omega = -Chassis.Get_Spin_Omega();

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {   
        PID_Chassis_Fllow.Set_Target(Reference_Angle-Reference_Angle);
        PID_Chassis_Fllow.Set_Now(Chassis_Angle);
        PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        chassis_omega = -PID_Chassis_Fllow.Get_Out();
    }

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        chassis_omega = 0;
    }

    Chassis.Set_Target_Omega(chassis_omega);
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
#ifdef GIMBAL
Enum_Referee_Data_Robots_ID robo_id;
Enum_Referee_Game_Status_Stage game_stage;
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback(uint8_t *Rx_Data)
{
    Chassis_Alive_Flag++;
    float shoot_speed;
    memcpy(&robo_id,CAN_Manage_Object->Rx_Buffer.Data,sizeof(uint8_t));
    memcpy(&game_stage,CAN_Manage_Object->Rx_Buffer.Data+1,sizeof(uint8_t));
    memcpy(&shoot_speed,CAN_Manage_Object->Rx_Buffer.Data+2,sizeof(float));
    Referee.Set_Robot_ID(robo_id);
    Referee.Set_Game_Stage(game_stage);
    Referee.Set_Shoot_Speed(shoot_speed);
}
#endif

/**
 * @brief can回调函数给底盘发送数据
 *
 */
#ifdef GIMBAL
// 控制类型字节
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    uint8_t control_type;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, gimbal_pitch;
    // 映射之后的目标速度 int16_t
    uint16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0,tmp_gimbal_yaw = 0;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    
    // 控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
    //Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    control_type = (uint8_t)(Referee_UI_Refresh_Status << 7 |  MiniPC_Status<< 6 | Supercap_Control_Status << 5 | Fric_Status << 4 | Bulletcap_Status << 3 | Chassis_Logics_Direction << 2 | chassis_control_type);

    // 设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max(), 0, 0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(uint16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max(), 0, 0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(uint16_t));

    tmp_gimbal_yaw = Math_Float_To_Int(gimbal_pitch, -50.f, 50.f ,0,0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 5, &tmp_gimbal_yaw, sizeof(uint16_t));

    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 7, &control_type, sizeof(uint8_t));
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
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_omega = 0;

    /************************************遥控器控制逻辑*********************************************/
    if (Get_DR16_Control_Type() == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Negative);
            }
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
        {
            // 上位机模式底盘暂时失能
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

        }

        // if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 狙击模式
        // {
        //     // 底盘锁死 云台可动
        //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        //     Gimbal.Set_Launch_Mode(Launch_Enable);
        // }
        // else
        // {
        //     Gimbal.Set_Launch_Mode(Launch_Disable);
        // }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if (Get_DR16_Control_Type() == DR16_Control_Type_KEYBOARD)
    {

        if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED) // 按住shift加速
        {
            DR16_Mouse_Chassis_Shift = 1.0f;
            Sprint_Status = Sprint_Status_ENABLE;
        }
        else
        {
            DR16_Mouse_Chassis_Shift = 2.0f;
            Sprint_Status = Sprint_Status_DISABLE;
        }

        if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED) // x轴
        {
            chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
        {
            chassis_velocity_x = Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
        {
            chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
        {
            chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
        }
        //Q键自瞄模式切换代码未写
        //code
        if (DR16.Get_Keyboard_Key_E() == DR16_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
        {
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            }
            else
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }

        if (DR16.Get_Keyboard_Key_R() == DR16_Key_Status_PRESSED) // 按下R键刷新UI
        {
            Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
        }
        else
        {
            Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
        }

        if(DR16.Get_Keyboard_Key_Z() == DR16_Key_Status_TRIG_FREE_PRESSED)//按下切换开关超电
        {
            if(Supercap_Control_Status == Supercap_Control_Status_DISABLE)
            {
                Supercap_Control_Status = Supercap_Control_Status_ENABLE;
            }
            else
                Supercap_Control_Status = Supercap_Control_Status_DISABLE;
                
        }
        // if(DR16.Get_Keyboard_Key_Z() == DR16_Key_Status_PRESSED)
        // {
        //     Chassis.Supercap.Get_Supercap_Control_Status(Supercap_Control_Status_ENABLE);
        // }
        // if(DR16.Get_Keyboard_Key_X() == DR16_Key_Status_PRESSED)
        // {
        //     Chassis.Supercap.Get_Supercap_Control_Status(Supercap_Control_Status_DISABLE);
        // }
    }
    else
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Omega(chassis_omega);
}
#endif

/**
 * @brief 鼠标数据转换
 *
 */
#ifdef GIMBAL
void Class_Chariot::Transform_Mouse_Axis()
{
    True_Mouse_X = -DR16.Get_Mouse_X();
    True_Mouse_Y = -DR16.Get_Mouse_Y();
    True_Mouse_Z = DR16.Get_Mouse_Z();
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
    //static uint8_t Switch_Mode_Flag = 0;
    //static float tmp_yaw_offest = 0.0f;
    static float Remote_K = 3.0f;//1.25
    /************************************遥控器控制逻辑*********************************************/
    if (Get_DR16_Control_Type() == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;
        // pitch赋值逻辑
        tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
        tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Angle_Resolution * 0.5f;
        // yaw赋值逻辑
        tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
        tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * Remote_K;

        // switch (Gimbal.Get_Launch_Mode()) // 吊射模式
        // {
        // case Launch_Disable:
        // {
        //     Switch_Mode_Flag = 0;
        //     Remote_K = 0.75f;
        // }
        // break;
        // case Launch_Enable:
        // {
        //     if (!Switch_Mode_Flag)
        //     {
        //         Switch_Mode_Flag = 1;
        //         // tmp_yaw_offest = Gimbal.Motor_Yaw.Get_True_Angle_Yaw_From_Encoder()-Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
        //         Remote_K = 0.25f;
        //     }
        //     // Gimbal.Set_Target_Yaw_Encoder_Angle(tmp_gimbal_yaw+tmp_yaw_offest);//编码器角度值
        // }
        // break;
        // }
        // 自瞄模式逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下自瞄
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);

        }
        else // 非自瞄模式
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }
        //设定角度
        Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
        Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw); // IMU角度值
    }
    /************************************键鼠控制逻辑*********************************************/
    else if (Get_DR16_Control_Type() == DR16_Control_Type_KEYBOARD)
    {
        //修正坐标系正方向
        Transform_Mouse_Axis();
        // yaw赋值逻辑
        tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
        tmp_gimbal_yaw += True_Mouse_X * DR16_Mouse_Yaw_Angle_Resolution * 2.0f;
        // pitch赋值逻辑
        tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
        tmp_gimbal_pitch -= True_Mouse_Y * DR16_Mouse_Pitch_Angle_Resolution;
        // 长按右键  开启自瞄
        if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal); // 开启自瞄默认为四点
        }
        else
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }

        //C键按下 一键调头
        if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            tmp_gimbal_yaw += 180;

            // if(Chassis_Logics_Direction == Chassis_Logic_Direction_Positive)
            //     Chassis_Logics_Direction = Chassis_Logic_Direction_Negative;
            // else
            //     Chassis_Logics_Direction = Chassis_Logic_Direction_Positive;
        }
        
        // E键按下切换Pitch锁定模式和free模式
        // if (DR16.Get_Keyboard_Key_G() == DR16_Key_Status_TRIG_FREE_PRESSED)
        // {
        //     if (Pitch_Control_Status == Pitch_Status_Control_Free)
        //         Pitch_Control_Status = Pitch_Status_Control_Lock;
        //     else
        //         Pitch_Control_Status = Pitch_Status_Control_Free;
        // }
        // 设定角度
        Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
        Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw); // IMU角度值
    }

    // 如果小陀螺/随动 yaw给不同参数
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
        // Gimbal.Motor_Yaw.PID_Angle.Set_K_P(40.0f);
        // Gimbal.Motor_Yaw.PID_Angle.Set_K_D(0.25f);
        // Gimbal.Motor_Yaw.PID_Omega.Set_K_P(75.0f);
        // Gimbal.Motor_Yaw.PID_Omega.Set_K_D(0.15f);
    }
}

#endif


#ifdef GIMBAL
/**
 * @brief 图传控制逻辑
 *
 */
void Class_Chariot::Control_Image()
{
    //设置pitch yaw角度
    float tmp_image_pitch = 0.0f, tmp_image_roll = 0.0f;
    if (Get_DR16_Control_Type() == DR16_Control_Type_KEYBOARD)
    {
        if(DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            tmp_image_pitch = 0.0f;
            tmp_image_roll = 0.0f;
        }
        else
        {
            tmp_image_pitch = 0.0f;
            tmp_image_roll = 0.0f;
        }
    }

    Image.Set_Target_Image_Pitch_Angle(tmp_image_pitch + Image.get_Image_Pitch_Calibrate_Offset());
    Image.Set_Target_Image_Roll_Angle(tmp_image_roll + Image.get_Image_Roll_Calibrate_Offset());
    
}
#endif


/**
 * @brief 发射机构控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    static uint8_t fire_cmd = 0;
    /************************************遥控器控制逻辑*********************************************/
    if (Get_DR16_Control_Type() == DR16_Control_Type_REMOTE)
    {
        // 右上 开启摩擦轮和发射机构
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            // 设置单发模式
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);

                if (DR16.Get_Yaw() < 0.2 && DR16.Get_Yaw() > -0.2)
                {
                    Shoot_Flag = 0;
                }
                else if (DR16.Get_Yaw() > 0.8 && Shoot_Flag == 0)
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    Shoot_Flag = 1;
                }
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Get_DR16_Control_Type()==DR16_Control_Type_KEYBOARD)
    {   
        //鼠标左键单点控制开火 单发
        if((DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED) &&
        abs(Booster.Fric[0].Get_Now_Omega_Rpm()) > Booster.Get_Friction_Omega_Threshold() &&
        abs(Booster.Fric[2].Get_Now_Omega_Rpm()) > Booster.Get_Friction_Omega_Threshold())
        {
            //单发
            Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
        }
        //CTRL键控制摩擦轮
        if(DR16.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            if(Booster.Get_Friction_Control_Type()==Friction_Control_Type_ENABLE)
            {
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            }
            else
            {
                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            }				
        }
    }
    //UI显示检测摩擦轮是否开启
    if(abs(Booster.Fric[0].Get_Now_Omega_Rpm()) > Booster.Get_Friction_Omega_Threshold() &&
    abs(Booster.Fric[2].Get_Now_Omega_Rpm()) > Booster.Get_Friction_Omega_Threshold())
    {
        Fric_Status = Fric_Status_OPEN;
    }
    else
    {
        Fric_Status = Fric_Status_CLOSE;
    }

    // if(Fric_Status == Fric_Status_OPEN)
    // {
    //     Gimbal.Motor_Yaw.PID_Angle.Set_K_P(80.0f);
    //     Gimbal.Motor_Yaw.PID_Angle.Set_K_D(0.04f);
    //     Gimbal.Motor_Yaw.PID_Omega.Set_K_P(150.0f);
    //     Gimbal.Motor_Yaw.PID_Omega.Set_K_D(0.0075f);
    // }
    // else
    // {
    //     Gimbal.Motor_Yaw.PID_Angle.Set_K_P(40.0f);
    //     Gimbal.Motor_Yaw.PID_Angle.Set_K_D(0.25f);
    //     Gimbal.Motor_Yaw.PID_Omega.Set_K_P(75.0f);
    //     Gimbal.Motor_Yaw.PID_Omega.Set_K_D(0.15f);
    // }
}
#endif
#ifdef CHASSIS
uint8_t robot_id,game_state;
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    // uint16_t Shooter_Barrel_Cooling_Value;
    // uint16_t Shooter_Barrel_Heat_Limit;
    // Shooter_Barrel_Heat_Limit = Referee.Get_Booster_42mm_Heat_Max();
    // Shooter_Barrel_Cooling_Value = Referee.Get_Booster_42mm_Heat_CD();
    //发送数据给云台
//    uint8_t robot_id,game_state;
    float shoot_speed = Referee.Get_Shoot_Speed();
    robot_id = Referee.Get_ID();
    game_state = Referee.Get_Game_Stage();
    memcpy(CAN2_Chassis_Tx_Gimbal_Data,&robot_id,sizeof(uint8_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 1,&game_state,sizeof(uint8_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &shoot_speed, sizeof(float));
    // memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &Shooter_Barrel_Heat_Limit, sizeof(uint16_t));
    // memcpy(CAN2_Chassis_Tx_Gimbal_Data + 4, &Shooter_Barrel_Cooling_Value, sizeof(uint16_t));
    
}
#endif

#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Streeing_Wheel_Callback()
{
    //与舵小板通信有效数据为：角度、角速度
    uint16_t tmp_angle = 0;
    int16_t tmp_omega = 0;
    uint8_t chassis_status = (uint8_t)(Chassis.Get_Chassis_Control_Type());
    // 舵轮A 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[0].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[0].streeing_wheel_omega);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[0], &tmp_angle, 2);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[2], &tmp_omega, 2);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[4], &chassis_status, 1);
    // 舵轮B 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[1].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[1].streeing_wheel_omega);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[0], &tmp_angle, 2);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[2], &tmp_omega, 2);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[4], &chassis_status, 1);
    // 舵轮C 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[2].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[2].streeing_wheel_omega);
    memcpy(&CAN1_0x1c_Tx_Streeing_Wheel_C_data[0], &tmp_angle, 2);
    memcpy(&CAN1_0x1c_Tx_Streeing_Wheel_C_data[2], &tmp_omega, 2);
    memcpy(&CAN1_0x1c_Tx_Streeing_Wheel_C_data[4], &chassis_status, 1);
    // 舵轮D 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[3].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[3].streeing_wheel_omega);
    memcpy(&CAN1_0x1d_Tx_Streeing_Wheel_D_data[0], &tmp_angle, 2);
    memcpy(&CAN1_0x1d_Tx_Streeing_Wheel_D_data[2], &tmp_omega, 2);
    memcpy(&CAN1_0x1d_Tx_Streeing_Wheel_D_data[4], &chassis_status, 1);

}
#endif
#ifdef CHASSIS
float supercap_target_power;
float Buffer_Power;
float Chassis_Actual_Limit_Power;
void Class_Chariot::CAN_Chassis_Tx_Max_Power_Callback()
{  
    //读取底盘限制功率
    float Power_Max = Chassis.Referee->Get_Chassis_Power_Max();
     //读取底盘实际功率
    float Chassis_Actual_Power = fabs(Chassis.Supercap.Get_Chassis_Actual_Power());
    //针对于关闭超电的缓冲环
    PID_Chassis_Buffer_Power.Set_Target(30.f);
    PID_Chassis_Buffer_Power.Set_Now(Chassis.Referee->Get_Chassis_Energy_Buffer());
    PID_Chassis_Buffer_Power.TIM_Adjust_PeriodElapsedCallback();
    Buffer_Power = -PID_Chassis_Buffer_Power.Get_Out();
    
    Chassis_Actual_Limit_Power = Power_Max + Buffer_Power;

    if(Chassis.Supercap.Get_Supercap_Status() == Supercap_Status_ENABLE && Supercap_Control_Status == Supercap_Control_Status_ENABLE)
        Chassis_Actual_Limit_Power += ((Chassis.Supercap.Get_Supercap_Buffer_Power() > 0)? Chassis.Supercap.Get_Supercap_Buffer_Power() : 0);
    else
        Chassis_Actual_Limit_Power = Chassis_Actual_Limit_Power;
    // //处理超电低电压保护
    // if( Chassis.Supercap.Get_Supercap_Buffer_Power() == 0.0f)
    //     Power_Max = Power_Max+fabs(Buffer_Power) + 5.0f;
    // else
    //     Power_Max = Power_Max;

    //控制发送给超电的功率上限数据
    supercap_target_power = (Buffer_Power>0)?0:Buffer_Power;
    Chassis.Supercap.Set_Limit_Power(Power_Max + supercap_target_power + 5.0f);
    //控制超电一直处于使能状态
    Chassis.Supercap.Set_Supercap_Control_Status((Enum_Supercap_Control_Status)SuperCap);

    memcpy(CAN1_0x01E_Tx_Data, &Chassis_Actual_Limit_Power, sizeof(float));
	memcpy(CAN1_0x01E_Tx_Data+4,&Chassis_Actual_Power,sizeof(float));
}

#endif

#ifdef CHASSIS
void Class_Chariot::Chariot_Referee_UI_Tx_Callback(Enum_Referee_UI_Refresh_Status __Referee_UI_Refresh_Status)
{
    
    static uint8_t String_Index = 0;
    String_Index++;
    if (String_Index > 6)
    {
        String_Index = 0;
    }

    switch (__Referee_UI_Refresh_Status)
    {
    case (Referee_UI_Refresh_Status_DISABLE):
    {
        // 摩擦轮状态
        if (Fric_Status == Fric_Status_OPEN)
        {
            //Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Zero, 0, 0x00, 0, 20, 2, 500/2, 400+410, "Fric_OPEN", (sizeof("Fric_OPEN") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_One,0,0x0A,Graphic_Color_PINK,10,430,820,480,770,Referee_UI_CHANGE);
        }
        else
        {
            //Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Zero, 0, 0x00, 0, 20, 2, 500/2, 400+410, "Fric_CLOSE", (sizeof("Fric_CLOSE") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_One,0,0x0A,Graphic_Color_WHITE,10,430,820,480,770,Referee_UI_CHANGE);
        }
        // 底盘状态
        if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
        {
            Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Zero, 0, 0x01, Graphic_Color_PINK, 20, 5, 500/2+800, 400+410, "Follow", (sizeof("Follow") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Zero, 0, 0x10, Graphic_Color_WHITE, 20, 5, 500/2+800, 660, "Spin", (sizeof("Spin") - 1), Referee_UI_CHANGE);
        }
        else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive || Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Negative)
        {
            Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Zero, 0, 0x01, Graphic_Color_WHITE, 20, 5, 500/2+800, 400+410, "Follow", (sizeof("Follow") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Zero, 0, 0x10, Graphic_Color_PINK, 20, 5, 500/2+800, 660, "Spin", (sizeof("Spin") - 1), Referee_UI_CHANGE);
        }
        else
        {
            Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Zero, 0, 0x01, Graphic_Color_WHITE, 20, 5, 500/2+800, 400+410, "Follow", (sizeof("Follow") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Zero, 0, 0x10, Graphic_Color_WHITE, 20, 5, 500/2+800, 660, "Spin", (sizeof("Spin") - 1), Referee_UI_CHANGE);
        }
        // 云台状态
        if (Gimbal_Status == Gimbal_Status_ENABLE)
        {
            //Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Zero, 0, 0x02, 0, 20, 2, 500/2, 300+410, "Gimbal_Alive", (sizeof("Gimbal_Alive") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_Two,0,0x0B,Graphic_Color_PINK,10,430,820-150,480,770-150,Referee_UI_CHANGE);
        }
        else
        {
            //Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Zero, 0, 0x02, 0, 20, 2, 500/2, 300+410, "Gimbal_Dead", (sizeof("Gimbal_Dead") - 1), Referee_UI_CHANGE);
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_Two,0,0x0B,Graphic_Color_WHITE,10,430,820-150,480,770-150,Referee_UI_CHANGE);
        }

        Referee.Referee_UI_Draw_Line(Referee.Get_ID(),Referee_UI_Five , 1, 0x08, 6, 10,960-400+120 , 45,960-400+120+(uint32_t)(560.0f*Chassis.Supercap.Get_Supercap_Charge_Percentage()/100.0f), 45, Referee_UI_CHANGE);
        
        if(MiniPC_Status == MiniPC_Status_ENABLE)
        {
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_Zero,1,0x09,4,3,960-300,540-150,960+300,540+300,Referee_UI_CHANGE);
        }
        else
        {
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_Zero,1,0x09,8,3,960-300,540-150,960+300,540+300,Referee_UI_CHANGE);
        }

        if(Supercap_Control_Status == Supercap_Control_Status_ENABLE)
        {
            Referee.Referee_UI_Draw_String(4, Referee.Get_ID(), Referee_UI_Zero, 0, 0x11 , Graphic_Color_PINK, 20, 5, 500/2+800, 510, "SuperCap", (sizeof("SuperCap") - 1), Referee_UI_CHANGE);
        }
        else
        {
            Referee.Referee_UI_Draw_String(4, Referee.Get_ID(), Referee_UI_Zero, 0, 0x11 , Graphic_Color_WHITE, 20, 5, 500/2+800, 510, "SuperCap", (sizeof("SuperCap") - 1), Referee_UI_CHANGE);
        }

        Referee.Referee_UI_Draw_Float_Graphic_5(Referee.Get_ID(),Referee_UI_Three,0,0x0F,Graphic_Color_GREEN,20,5,500/2+800+150, 400+410,Pitch_IMU_Angle,Referee_UI_CHANGE);

    }
    break;
    case (Referee_UI_Refresh_Status_ENABLE):
    {
        //摩擦轮状态
        //Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Zero, 0, 0x00, 0, 20, 2, 500/2, 400+410, "Fric_CLOSE", (sizeof("Fric_CLOSE") - 1), Referee_UI_ADD);
        Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_One,0,0x0A,Graphic_Color_WHITE,10,430,820,480,770,Referee_UI_ADD);
        //底盘状态
        Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Zero, 0, 0x01, Graphic_Color_WHITE, 20, 5, 500/2+800, 400+410, "Follow", (sizeof("Follow") - 1), Referee_UI_ADD);
        Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Zero, 0, 0x10, Graphic_Color_WHITE, 20, 5, 500/2+800, 660, "Spin", (sizeof("Spin") - 1), Referee_UI_ADD);
        // 云台状态
        //Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Zero, 0, 0x02, 0, 20, 2, 500/2, 300+410, "Gimbal_Dead", (sizeof("Gimbal_Dead") - 1), Referee_UI_ADD);
        Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_Two,0,0x0B,Graphic_Color_WHITE,10,430,820-150,480,770-150,Referee_UI_ADD);
        //超电
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(),Referee_UI_Five , 1, 0x08, 6, 10,960-400+120 , 45,960-400+120+(uint32_t)(560.0f*0), 45, Referee_UI_ADD);
        //自瞄
        Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(),Referee_UI_Zero,1,0x09,8,3,960-300,540-150,960+300,540+300,Referee_UI_ADD);
        //超电
        Referee.Referee_UI_Draw_String(4, Referee.Get_ID(), Referee_UI_Zero, 0, 0x11 , Graphic_Color_WHITE, 20, 5, 500/2+800, 510, "SuperCap", (sizeof("SuperCap") - 1), Referee_UI_ADD);
        //pitch
        Referee.Referee_UI_Draw_Float_Graphic_5(Referee.Get_ID(),Referee_UI_Three,0,0x0F,Graphic_Color_GREEN,20,5,500/2+800+150, 400+410,0.0f,Referee_UI_ADD);
    }
    break;
    }
    Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Zero, 0, 0x00, Graphic_Color_GREEN, 20, 5, 500/2, 400+410, "Fric :", (sizeof("Fric :") - 1), Referee_UI_ADD);
    Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Zero, 0, 0x02,Graphic_Color_GREEN , 20, 5, 500/2, 660, "Gimbal:", (sizeof("Gimbal:") - 1), Referee_UI_ADD);
    // 画线
    Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Zero, 1, 0x03, 3, 3, 960-400+120, 200, 900, 200, Referee_UI_ADD);
    Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_One, 1, 0x04, 3, 3, 1020, 200, 960+400-120, 200, Referee_UI_ADD);
    Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Two, 1, 0x05, 3, 3, 960-400, 100, 960-400+120, 200, Referee_UI_ADD);
    Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Three, 1, 0x06, 3, 3, 960+400-120, 200, 960+400, 100, Referee_UI_ADD);
    
    // 超电容量
    Referee.Referee_UI_Draw_Rectangle(Referee.Get_ID(), Referee_UI_Four, 1, 0x07, 8, 3,960-400+120, 50,960+400-120, 40, Referee_UI_ADD);

    // 善后处理
    Referee.UART_Tx_Referee_UI(String_Index);
}
#endif

/**
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS
        //获取底盘坐标系角度
        Chassis_Coordinate_System_Angle_Rad = Get_Chassis_Coordinate_System_Angle_Rad();
        // 底盘给云台发消息
        CAN_Chassis_Tx_Gimbal_Callback();
        //底盘给分别给四个舵轮发消息
        CAN_Chassis_Tx_Streeing_Wheel_Callback();
        //底盘给舵小板发送最大功率
        CAN_Chassis_Tx_Max_Power_Callback();
        //云台，随动掉线保护
        // if(Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && Gimbal_Status == Gimbal_Status_ENABLE)
        // {
        //     Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        // }
        // else
        // {
        //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        // }
        Control_Chassis_Omega_TIM_PeriodElapsedCallback();
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        //画UI
        static uint8_t mod34 = 0;
        mod34++;
        if(mod34==20)
        {
            Chariot_Referee_UI_Tx_Callback(Referee_UI_Refresh_Status);
            mod34 = 0;
        }
        //超电通信
        Chassis.Supercap.TIM_Supercap_PeriodElapsedCallback();
#elif defined(GIMBAL)
    
    // 各个模块的分别解算
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    Booster.TIM_Calculate_PeriodElapsedCallback();
    Image.TIM_Calculate_PeriodElapsedCallback();
    // 传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();
    // 给下板发送数据
    CAN_Gimbal_Tx_Chassis_Callback();
    // 弹舱舵机控制
    //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Compare);

#endif   
}

/**
 * @brief 判断DR16控制数据来源
 *
 */
#ifdef GIMBAL
void Class_Chariot::Judge_DR16_Control_Type()
{
    if (DR16.Get_Left_X() != 0 ||
        DR16.Get_Left_Y() != 0 ||
        DR16.Get_Right_X() != 0 ||
        DR16.Get_Right_Y() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_REMOTE;
    }
    else if (DR16.Get_Mouse_X() != 0 ||
             DR16.Get_Mouse_Y() != 0 ||
             DR16.Get_Mouse_Z() != 0 ||
             DR16.Get_Keyboard_Key_A() != 0 ||
             DR16.Get_Keyboard_Key_D() != 0 ||
             DR16.Get_Keyboard_Key_W() != 0 ||
             DR16.Get_Keyboard_Key_S() != 0 ||
             DR16.Get_Keyboard_Key_Shift() != 0 ||
             DR16.Get_Keyboard_Key_Ctrl() != 0 ||
             DR16.Get_Keyboard_Key_Q() != 0 ||
             DR16.Get_Keyboard_Key_E() != 0 ||
             DR16.Get_Keyboard_Key_R() != 0 ||
             DR16.Get_Keyboard_Key_F() != 0 ||
             DR16.Get_Keyboard_Key_G() != 0 ||
             DR16.Get_Keyboard_Key_Z() != 0 ||
             DR16.Get_Keyboard_Key_C() != 0 ||
             DR16.Get_Keyboard_Key_V() != 0 ||
             DR16.Get_Keyboard_Key_B() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_KEYBOARD;
    }
}
#endif


/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    // 判断DR16控制数据来源
    Judge_DR16_Control_Type();

    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
    Control_Image();
}
#endif
/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
#ifdef CHASSIS

            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            
            if(mod50_mod3%3 == 0)
            {
                TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
                Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;
            }
#elif defined(GIMBAL)

        if (mod50_mod3 % 3 == 0)
        {
            // 判断底盘通讯在线状态
            TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }

        Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();
        Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Image.Motor_Image_Pitch.TIM_Alive_PeriodElapsedCallback();
        Image.Motor_Image_Roll.TIM_Alive_PeriodElapsedCallback();
        for (auto i = 0; i < 4; i++)
        {
            Booster.Fric[i].TIM_Alive_PeriodElapsedCallback();
        }

        MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

#endif

        mod50 = 0;
    }    
}

/**
 * @brief 离线保护函数
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
// 云台离线保护
#ifdef GIMBAL

        if(DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            //记录离线前一状态
            Pre_Gimbal_Control_Type = Gimbal.Get_Gimbal_Control_Type();
            Pre_Chassis_Control_Type = Chassis.Get_Chassis_Control_Type();
            //控制模块禁用
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            // 遥控器中途断联导致错误，重启 DMA
            if (huart3.ErrorCode)
            {
                HAL_UART_DMAStop(&huart3); // 停止以重启
                //HAL_Delay(10); // 等待错误结束
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
            }
        }
        else
        {
            // Gimbal.Set_Gimbal_Control_Type(Pre_Gimbal_Control_Type);
            // Chassis.Set_Chassis_Control_Type(Pre_Chassis_Control_Type);
        }

#endif

// 底盘离线保护
#ifdef CHASSIS

#endif

}

/**
 * @brief 底盘通讯在线判断回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback()
{
    if (Chassis_Alive_Flag == Pre_Chassis_Alive_Flag)
    {
        Chassis_Status = Chassis_Status_DISABLE;
    }
    else
    {
        Chassis_Status = Chassis_Status_ENABLE;
    }
    Pre_Chassis_Alive_Flag = Chassis_Alive_Flag;   
}
#endif

#ifdef CHASSIS
void Class_Chariot::TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback()
{
    if (Gimbal_Alive_Flag == Pre_Gimbal_Alive_Flag)
    {
        Gimbal_Status = Gimbal_Status_DISABLE;
    }
    else
    {
        Gimbal_Status = Gimbal_Status_ENABLE;
    }
    Pre_Gimbal_Alive_Flag = Gimbal_Alive_Flag;  
}
#endif
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
            {
                // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
                if (huart3.ErrorCode)
                {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(4);
                }

                // 转移为 在线状态
                if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
                {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(2);
                }

                // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
                if (Status[Now_Status_Serial].Time > 1000)
                {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(1);
                }
            }
            break;
            // 遥控器关闭状态
            case (1):
            {
                // 离线保护
                Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

                if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
                {
                    Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                    Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(2);
                }

                // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
                if (huart3.ErrorCode)
                {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(4);
                }
            }
            break;
            // 遥控器在线状态
            case (2):
            {
                // 转移为 刚离线状态
                if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(3);
                }
            }
            break;
            // 刚离线状态
            case (3):
            {
                // 记录离线检测前控制模式
                Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
                Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

                // 无条件转移到 离线检测状态
                Status[Now_Status_Serial].Time = 0;
                Set_Status(0);
            }
            break;
            // 遥控器串口错误状态
            case (4):
            {
                HAL_UART_DMAStop(&huart3); // 停止以重启
                // HAL_Delay(10); // 等待错误结束
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

                // 处理完直接跳转到 离线检测状态
                Status[Now_Status_Serial].Time = 0;
                Set_Status(0);
            }
            break;
            }
        }
#endif

    /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
