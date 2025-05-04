/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
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

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __Dead_Zone)
{
    #ifdef CHASSIS

        //裁判系统
        Referee.Init(&huart6);

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();

        #ifdef FLYING_SLOPE

        Chassis.Flying_Slope.IMU = &Chassis.Boardc_BMI;

        #endif

        //底盘随动PID环初始化
        PID_Chassis_Fllow.Init(10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 20.0f,0.0f,0.0f,0.0f,0.001f,0.01f);

        //yaw电机canid初始化  只获取其编码器值用于底盘随动，并不参与控制
        Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x205);

    #elif defined(GIMBAL)
        
        Chassis.Set_Velocity_X_Max(18.0f);
        Chassis.Set_Velocity_Y_Max(18.0f);

        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);

        #ifdef USE_VT13
        FSM_Alive_Control_VT13.Chariot = this;
        FSM_Alive_Control_VT13.Init(5,0);
        #endif

        Active_Controller = Controller_NONE;

        //遥控器
        DR16.Init(&huart3,&huart1);
        Dead_Zone = __Dead_Zone;   

        //云台
        Gimbal.Init();
        Gimbal.MiniPC = &MiniPC;
        
        //发射机构
        Booster.Referee = &Referee;
        Booster.Init();
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object, &hcan1);
        MiniPC.IMU = &Gimbal.Boardc_BMI;
        MiniPC.Referee = &Referee;

    #endif
}

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS    
    //控制类型字节
    uint8_t control_type;
    //底盘和云台夹角（弧度制）
    float derta_angle;
    float gimbal_velocity_x, gimbal_velocity_y;
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{   
    Gimbal_Alive_Flag++;
    //云台坐标系的目标速度
    
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
//    //底盘和云台夹角（弧度制）
//    float derta_angle;
    //float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y, tmp_gimbal_pitch;
    uint8_t tmp_omega;


    memcpy(&tmp_velocity_x,&CAN_Manage_Object->Rx_Buffer.Data[0],sizeof(uint16_t));
    memcpy(&tmp_velocity_y,&CAN_Manage_Object->Rx_Buffer.Data[2],sizeof(uint16_t));
    memcpy(&tmp_omega,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(uint8_t));
    memcpy(&tmp_gimbal_pitch,&CAN_Manage_Object->Rx_Buffer.Data[5],sizeof(uint16_t));
    memcpy(&control_type,&CAN_Manage_Object->Rx_Buffer.Data[7],sizeof(uint8_t));
    
    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x,0,0x7FFF,-1 * Chassis.Get_Velocity_X_Max(),Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y,0,0x7FFF,-1 * Chassis.Get_Velocity_Y_Max(),Chassis.Get_Velocity_Y_Max());
    Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch,0,0x7FFF,-10.0f,30.0f);

    chassis_control_type = (Enum_Chassis_Control_Type)(control_type & 0x03);
    Sprint_Status = (Enum_Sprint_Status)(control_type>>2 & 0x01);
    Bulletcap_Status = (Enum_Bulletcap_Status)(control_type>>3 & 0x01);
    Fric_Status = (Enum_Fric_Status)(control_type>>4 & 0x01);
    MiniPC_Aim_Status = (Enum_MinPC_Aim_Status)(control_type>>5 & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)(control_type>>6 & 0x01);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)(control_type>>7 & 0x01);

   //获取云台坐标系和底盘坐标系的夹角（弧度制）
   //角速度前馈，保证小陀螺时走直线
   float Feedback_Angle =  0.0f;
   if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive){
        Feedback_Angle = -0.025f * Math_Int_To_Float(tmp_omega,0,0xFF,-1 * 20.0f,20.0f);
   }
   else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
   {
        Feedback_Angle = 0.025f * Math_Int_To_Float(tmp_omega,0,0xFF,-1 * 20.0f,20.0f);
   }
   else{
        Feedback_Angle = 0.0f;
   }
   

   Chassis_Angle = Motor_Yaw.Get_Now_Radian();
   derta_angle = Reference_Angle - Chassis_Angle + Offset_Angle + Feedback_Angle;
   derta_angle = derta_angle<0?(derta_angle+2*PI):derta_angle;  

   //云台坐标系的目标速度转为底盘坐标系的目标速度
   chassis_velocity_x = -1.0f * ((float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle)));
   chassis_velocity_y =  1.0f * ((float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle)));
    //设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);
    
    //底盘控制方案
    if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive || 
        Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
    {
        chassis_omega = Math_Int_To_Float(tmp_omega,0,0xFF,-1 * 20.0f,20.0f);
        Chassis.Set_Spin_Omega(chassis_omega);
    }
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
        //在别的地方跑了
        // //随动yaw角度优化
        // float temp_yaw,temp_reference;
        // temp_yaw = Chassis_Angle;
        // temp_reference = Reference_Angle;
        // // if(Chassis_Angle > PI)
        // //     temp_yaw = Chassis_Angle - 2 * PI;
        // // else if(Chassis_Angle < -PI)
        // //     temp_yaw = Chassis_Angle + 2 * PI;
        // // if(Reference_Angle > PI)
        // //     temp_reference = Reference_Angle - 2 * PI;  
        // //随动环
        // PID_Chassis_Fllow.Set_Target(temp_reference);
        // PID_Chassis_Fllow.Set_Now(temp_yaw);
        // PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        // chassis_omega = PID_Chassis_Fllow.Get_Out();
        //chassis_omega = 0;
    }
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        chassis_omega = 0;
    }
    
    //设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Omega(chassis_omega);
}

void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback_2()
{
    uint8_t control_type_2;
    uint16_t Fric_Omega_u16;
    control_type_2 = CAN_Manage_Object->Rx_Buffer.Data[2];
    Gimbal_Control_Type = (Enum_Gimbal_Control_Type)(control_type_2 & 0x03);
    Booster_User_Control_Type = (Enum_Booster_User_Control_Type)((control_type_2 >> 2) & 0x03);

    memcpy(&Fric_Omega_u16, &CAN_Manage_Object->Rx_Buffer.Data[0],sizeof(uint16_t));
    Fric_Omega = Math_Int_To_Float(Fric_Omega_u16,0, 0x7FFF, -800.0f, 800.0f);
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;

    Enum_Referee_Data_Robots_ID robo_id;
    Enum_Referee_Game_Status_Stage game_stage;
    uint16_t Shooter_Barrel_Cooling_Value;
    uint16_t Shooter_Barrel_Heat_Limit;
    int16_t Shooter_Barrel_Heat_Now;

    robo_id = (Enum_Referee_Data_Robots_ID)CAN_Manage_Object->Rx_Buffer.Data[0];
    game_stage = (Enum_Referee_Game_Status_Stage)CAN_Manage_Object->Rx_Buffer.Data[1];
    memcpy(&Shooter_Barrel_Heat_Limit, CAN_Manage_Object->Rx_Buffer.Data + 2, sizeof(uint16_t));
    memcpy(&Shooter_Barrel_Cooling_Value, CAN_Manage_Object->Rx_Buffer.Data + 4, sizeof(uint16_t));
    memcpy(&Shooter_Barrel_Heat_Now, CAN_Manage_Object->Rx_Buffer.Data + 6, sizeof(int16_t));

    Referee.Set_Robot_ID(robo_id);
    Referee.Set_Booster_17mm_1_Heat(Shooter_Barrel_Heat_Now);
    Referee.Set_Booster_17mm_1_Heat_CD(Shooter_Barrel_Cooling_Value);
    Referee.Set_Booster_17mm_1_Heat_Max(Shooter_Barrel_Heat_Limit);
    Referee.Set_Game_Stage(game_stage);
}
#endif


/**
 * @brief can回调函数给地盘发送数据
 *
 */
#ifdef GIMBAL
    //控制类型字节
    uint8_t control_type, control_type_2;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    //云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, gimbal_pitch, Fric_Omega; 
    //映射之后的目标速度 int16_t
    uint16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_omega = 0, Fric_Omega_u16;
    uint8_t tmp_gimbal_pitch = 0;
    float chassis_omega = 0;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    Enum_Gimbal_Control_Type gimbal_control_type;           //2bit
    Enum_Booster_User_Control_Type booster_user_control_type;         //3bit

    //控制类型字节
    gimbal_control_type = Gimbal.Get_Gimbal_Control_Type();
    booster_user_control_type = Booster.Get_Booster_User_Control_Type();
    control_type_2 = (uint8_t)(booster_user_control_type << 2 | gimbal_control_type);

    Fric_Omega = (abs(Booster.Motor_Friction_Left.Get_Now_Omega_Radian()) + abs(Booster.Motor_Friction_Right.Get_Now_Omega_Radian()))/2.0f;
    Fric_Omega_u16 = Math_Float_To_Int(Fric_Omega, -800.0f, 800.0f,0,0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data_2, &Fric_Omega_u16, sizeof(uint16_t));

    memcpy(CAN2_Gimbal_Tx_Chassis_Data_2 + 2, &control_type_2, sizeof(uint8_t));

    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_omega = Chassis.Get_Target_Omega();
    gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();           //不知道为什么，加个负号
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    control_type =  (uint8_t)(Referee_UI_Refresh_Status << 7|MiniPC_Status << 6|MiniPC_Aim_Status << 5|Fric_Status << 4|Bulletcap_Status << 3|Sprint_Status << 2|chassis_control_type);

    //设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x,-1 * Chassis.Get_Velocity_X_Max() ,Chassis.Get_Velocity_X_Max() ,0,0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(uint16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y,-1 * Chassis.Get_Velocity_Y_Max() ,Chassis.Get_Velocity_Y_Max() ,0,0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(uint16_t));
    
    tmp_chassis_omega = Math_Float_To_Int(chassis_omega,-1 * 20.0f ,20.0f ,0,0xFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_omega, sizeof(uint8_t));

    tmp_gimbal_pitch = Math_Float_To_Int(gimbal_pitch, -40.0f, 30.0f ,0,0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 5, &tmp_gimbal_pitch, sizeof(uint16_t));

    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 7,&control_type ,sizeof(uint8_t));

}
void Class_Chariot::Judge_Active_Controller()
{
    // 检查DR16是否有输入
    Judge_DR16_Control_Type();

    // 检查VT13是否有输入
    Judge_VT13_Control_Type();

    // 判断当前活动的控制器
    if (VT13_Control_Type != VT13_Control_Type_NONE)
    {
        Active_Controller = Controller_VT13;
    }
    else if (DR16_Control_Type != DR16_Control_Type_NONE)
    {
        Active_Controller = Controller_DR16;
    }
    else
    {
        Active_Controller = Controller_NONE;
    }
}
#endif

/**
 * @brief 底盘控制逻辑
 *
 */  		
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    //遥控器摇杆值
    Judge_Active_Controller();
    //云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    static float chassis_omega = 0;     

	/************************************遥控器控制逻辑*********************************************/
    
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        float dr16_l_x, dr16_l_y;    
        //排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > Dead_Zone) ? DR16.Get_Left_Y() : 0;

        //设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max() ;
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max() ;

        //键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch()==DR16_Switch_Status_MIDDLE)  //左中 随动模式
        {
            //底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);   
        }
        if (DR16.Get_Left_Switch()==DR16_Switch_Status_UP)  //左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = Chassis.Get_Spin_Omega();
            if(DR16.Get_Right_Switch()== DR16_Switch_Status_DOWN)  //右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_NePositive);
                chassis_omega = -Chassis.Get_Spin_Omega();
            }
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) 
    {   
        Sprint_Status = Sprint_Status_DISABLE;
        if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED) // 按住shift加速
        {
            Mouse_Chassis_Shift = 1.0f;
            Sprint_Status = Sprint_Status_ENABLE;
        }
        else
        {
            Mouse_Chassis_Shift = 2.0f;
            Sprint_Status = Sprint_Status_DISABLE;
        }

        if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED) // x轴
        {
            chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
        {
            chassis_velocity_x = Chassis.Get_Velocity_X_Max() / Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
        {
            chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
        {
            chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / Mouse_Chassis_Shift;
        }

        if (DR16.Get_Keyboard_Key_E() == DR16_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
        {
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                chassis_omega = Chassis.Get_Spin_Omega();
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
    }
    
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        float vt13_l_x, vt13_l_y;    
        //排除遥控器死区
        Sprint_Status = Sprint_Status_ENABLE;           //遥控器下默认开启超电冲刺
        vt13_l_x = (Math_Abs(VT13.Get_Left_X()) > Dead_Zone) ? VT13.Get_Left_X() : 0;
        vt13_l_y = (Math_Abs(VT13.Get_Left_Y()) > Dead_Zone) ? VT13.Get_Left_Y() : 0;

        //设定矩形到圆形映射进行控制
        chassis_velocity_x = vt13_l_x * sqrt(1.0f - vt13_l_y * vt13_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = vt13_l_y * sqrt(1.0f - vt13_l_x * vt13_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        //按下右键开启小陀螺        
        if(VT13.Get_Switch() == VT13_Switch_Status_Left){
            if(Chassis.Get_Chassis_Control_Type() != Chassis_Control_Type_SPIN_Positive && 
                Chassis.Get_Chassis_Control_Type() != Chassis_Control_Type_SPIN_NePositive)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                    chassis_omega = Chassis.Get_Spin_Omega();
                }    
        }
        else if(VT13.Get_Switch() == VT13_Switch_Status_Middle){
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }

        //按下扳机切换小陀螺转向
        if(VT13.Get_Trigger() == VT13_Trigger_TRIG_FREE_PRESSED){
            if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive){
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_NePositive);
                chassis_omega = -1 * Chassis.Get_Spin_Omega();
            }
            else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive){
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                chassis_omega = Chassis.Get_Spin_Omega();
            }
        }
        
    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD) 
    {   
        if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_DISABLE){
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }

        Sprint_Status = Sprint_Status_DISABLE;
        if (VT13.Get_Keyboard_Key_Shift() == VT13_Key_Status_PRESSED) // 按住shift加速
        {
            Mouse_Chassis_Shift = 1.0f;
            Sprint_Status = Sprint_Status_ENABLE;
        }
        else
        {
            Mouse_Chassis_Shift = 2.0f;
            Sprint_Status = Sprint_Status_DISABLE;
        }

        if (VT13.Get_Keyboard_Key_A() == VT13_Key_Status_PRESSED) // x轴
        {
            chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / Mouse_Chassis_Shift;
        }
        if (VT13.Get_Keyboard_Key_D() == VT13_Key_Status_PRESSED)
        {
            chassis_velocity_x = Chassis.Get_Velocity_X_Max() / Mouse_Chassis_Shift;
        }
        if (VT13.Get_Keyboard_Key_W() == VT13_Key_Status_PRESSED) // y轴
        {
            chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / Mouse_Chassis_Shift;
        }
        if (VT13.Get_Keyboard_Key_S() == VT13_Key_Status_PRESSED)
        {
            chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / Mouse_Chassis_Shift;
        }

        if (VT13.Get_Keyboard_Key_E() == VT13_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
        {
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                chassis_omega = Chassis.Get_Spin_Omega();
            }
            else
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }

        if (VT13.Get_Keyboard_Key_R() == VT13_Key_Status_PRESSED) // 按下R键刷新UI
        {
            Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
        }
        else
        {
            Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
        }
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
void Class_Chariot::Transform_Mouse_Axis(){
        True_Mouse_X = -DR16.Get_Mouse_X();
        True_Mouse_Y =  DR16.Get_Mouse_Y();
        True_Mouse_Z =  DR16.Get_Mouse_Z();
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Gimbal()
{
    //角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch;
    //遥控器摇杆值
    
    //获取当前角度值
    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    //tmp_gimbal_yaw = Gimbal.Boardc_BMI.Get_Angle_Yaw();           零飘造成误差
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    float dr16_y, dr16_r_y;
    if(Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > Dead_Zone) ? DR16.Get_Right_Y() : 0;

        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下自瞄
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

            //两次开启自瞄分别切换四点五点
            if(Gimbal.MiniPC->Get_MiniPC_Type()==MiniPC_Type_Nomal)
                Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill); //五点
            else 
                Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal); 
        }
        else  // 非自瞄模式
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            //遥控器操作逻辑
            tmp_gimbal_yaw -= dr16_y * Yaw_Angle_Resolution;
            tmp_gimbal_pitch += dr16_r_y * Pitch_Angle_Resolution;

        }
        if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW &&
           DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN)  // 随动才能开舵机 右拨中-下 打开舵机
        {
			Compare = 1700;  
            Bulletcap_Status = Bulletcap_Status_OPEN;
        }
        else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW &&
                DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE) //随动才能开舵机 右拨下-中 关闭舵机
        {
			Compare = 400;
            Bulletcap_Status = Bulletcap_Status_CLOSE;
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD)
    {
        // 长按右键  开启自瞄
        if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED && MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal); // 开启自瞄默认为四点

            tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
            tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();

		    // if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN){
			// 	tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle()+minipc_yaw_offset;
            //     tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
			// }
        }
        else
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // Transform_Mouse_Axis();
         
        }
        tmp_gimbal_yaw -= DR16.Get_Mouse_X() * Mouse_Yaw_Angle_Resolution;
        tmp_gimbal_pitch -= DR16.Get_Mouse_Y() * Mouse_Pitch_Angle_Resolution;
        // R键按下 一键开关弹舱
        if (DR16.Get_Keyboard_Key_F() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Compare == 1700)
            {
                Bulletcap_Status = Bulletcap_Status_CLOSE;
                Compare = 400;
            }
            else
            {
                Bulletcap_Status = Bulletcap_Status_OPEN;
                Compare = 1700;
            }
        }
        // F键按下 一键调头
        if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            tmp_gimbal_yaw += 180;
        }
        // V键按下 自瞄模式中切换四点和五点模式
        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC &&
            DR16.Get_Keyboard_Key_V() == DR16_Key_Status_PRESSED)
        {
            Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
            // Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        // G键按下切换Pitch锁定模式和free模式
        if (DR16.Get_Keyboard_Key_G() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Pitch_Control_Status == Pitch_Status_Control_Free)
                Pitch_Control_Status = Pitch_Status_Control_Lock;
            else
                Pitch_Control_Status = Pitch_Status_Control_Free;
        }
    }


    
    else if(Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE){
        static uint8_t Start_Flag = 0;              //记录云台第一次上电，以初始化
        float vt13_y, vt13_r_y;
        // 排除遥控器死区
        vt13_y = (Math_Abs(VT13.Get_Right_X()) > Dead_Zone) ? VT13.Get_Right_X() : 0;
        vt13_r_y = (Math_Abs(VT13.Get_Right_Y()) > Dead_Zone) ? VT13.Get_Right_Y() : 0;

        //按下左键切换上位机或者normal
        if(!Start_Flag && VT13.Get_VT13_Status() == VT13_Status_ENABLE){
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Start_Flag = 1;
        }
        if(VT13.Get_Button_Left() == VT13_Button_TRIG_FREE_PRESSED){
            if(Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL){
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

                //反复按两次切换到风车
                if(Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Nomal){
                    Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
                }
                else if(Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill){
                    Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
                }
            }
            else if(Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC){
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
        }
        //更新目标角度
        if(Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL){
            tmp_gimbal_yaw -= vt13_y * Yaw_Angle_Resolution;
            tmp_gimbal_pitch += vt13_r_y * Pitch_Angle_Resolution;
        }

    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD)
    {
        // 长按右键  开启自瞄
        if (VT13.Get_Mouse_Right_Key() == VT13_Key_Status_PRESSED && MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal); // 开启自瞄默认为四点

            tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
            tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();

		    // if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN){
			// 	tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle()+minipc_yaw_offset;
            //     tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
			// }
        }
        else
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // Transform_Mouse_Axis();
         
        }
        tmp_gimbal_yaw -= VT13.Get_Mouse_X() * Mouse_Yaw_Angle_Resolution;
        tmp_gimbal_pitch += VT13.Get_Mouse_Y() * Mouse_Pitch_Angle_Resolution;
        // F键按下 一键开关弹舱
        if (VT13.Get_Keyboard_Key_F() == VT13_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Compare == 1700)
            {
                Bulletcap_Status = Bulletcap_Status_CLOSE;
                Compare = 400;
            }
            else
            {
                Bulletcap_Status = Bulletcap_Status_OPEN;
                Compare = 1700;
            }
        }
        // C键按下 一键调头
        if (VT13.Get_Keyboard_Key_C() == VT13_Key_Status_TRIG_FREE_PRESSED)
        {
            tmp_gimbal_yaw += 180;
        }
        // V键按下 自瞄模式中切换四点和五点模式
        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC &&
            VT13.Get_Keyboard_Key_V() == VT13_Key_Status_PRESSED)
        {
            Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
            // Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        // G键按下切换Pitch锁定模式和free模式
        if (VT13.Get_Keyboard_Key_G() == VT13_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Pitch_Control_Status == Pitch_Status_Control_Free)
                Pitch_Control_Status = Pitch_Status_Control_Lock;
            else
                Pitch_Control_Status = Pitch_Status_Control_Free;
        }
    }


    //如果pitch为锁定状态
    if(Pitch_Control_Status == Pitch_Status_Control_Lock)
        tmp_gimbal_pitch = -4.43f;

    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
}
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    Judge_Active_Controller();
    /************************************遥控器控制逻辑*********************************************/
    if(Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        //左上 开启摩擦轮和发射机构
        if(DR16.Get_Right_Switch()==DR16_Switch_Status_UP)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            Fric_Status = Fric_Status_OPEN;

            if(DR16.Get_Yaw()>-0.2f && DR16.Get_Yaw()<0.2f)
            {
                Shoot_Flag = 0;
            }
            if(DR16.Get_Yaw()<-0.8f && Shoot_Flag==0) //单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if(DR16.Get_Yaw()>0.8f && Shoot_Flag==0)  //五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            } 
        }
        else 
        {
           Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
           Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
           Fric_Status = Fric_Status_CLOSE;
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD)
    {   
        if (DR16.Get_Keyboard_Key_B() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
            {
                Booster.Booster_User_Control_Type = Booster_User_Control_Type_MULTI;
            }
            else
            {
                Booster.Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;
            }
        }

        // 自瞄模式+五点模式 左键变成单发
        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC &&
            Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill)
        {
            // 按下左键并且摩擦轮开启 单发
            if ((DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED) &&
                (abs(Booster.Motor_Friction_Left.Get_Now_Omega_Radian()) > Booster.Get_Friction_Omega_Threshold()))
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            }
        }

        // 正常模式
        else if ((DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED) &&
                 (abs(Booster.Motor_Friction_Left.Get_Now_Omega_Radian()) > Booster.Get_Friction_Omega_Threshold()) /*&&
                 (Booster.Motor_Driver.Get_Now_Radian() - Booster.Motor_Driver.Get_Target_Radian() < 0.1f)*/
        )
        {
            if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
            if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
        }
        // C键控制摩擦轮
        if (DR16.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {

                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                Fric_Status = Fric_Status_CLOSE;
                // Booster.Booster_User_Control_Type = Booster_User_Control_Type_DISABLE;
            }
            else
            {

                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                Fric_Status = Fric_Status_OPEN;
            }
        }
    }

    else if(Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        //开启摩擦轮和发射机构
        if(VT13.Get_Button_Right() == VT13_Button_TRIG_FREE_PRESSED){
            if(Fric_Status == Fric_Status_CLOSE){
                Fric_Status = Fric_Status_OPEN;
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            }
            else if(Fric_Status == Fric_Status_OPEN){
                Fric_Status = Fric_Status_CLOSE;
                Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            }
        }
        //如果要强制打完一次到一下次，防止一直按着连发，改成Booster_Control_Type_CEASEFIRE判断
        if(Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE && 
            Booster.Get_Booster_Control_Type() != Booster_Control_Type_DISABLE){
            if (VT13.Get_Yaw() > -0.2f && VT13.Get_Yaw() < 0.2f)
            {
                Shoot_Flag = 0;
            }
            if (VT13.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if (VT13.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            }
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if(Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD)
    {   
        if (VT13.Get_Keyboard_Key_B() == VT13_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
            {
                Booster.Booster_User_Control_Type = Booster_User_Control_Type_MULTI;
            }
            else
            {
                Booster.Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;
            }
        }

        // 自瞄模式+五点模式 左键变成单发
        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC &&
            Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill)
        {
            // 按下左键并且摩擦轮开启 单发
            if ((VT13.Get_Mouse_Left_Key() == VT13_Key_Status_TRIG_FREE_PRESSED) &&
                (abs(Booster.Motor_Friction_Left.Get_Now_Omega_Radian()) > Booster.Get_Friction_Omega_Threshold()))
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            }
        }

        // 正常模式
        else if ((VT13.Get_Mouse_Left_Key() == VT13_Key_Status_TRIG_FREE_PRESSED) &&
                 (abs(Booster.Motor_Friction_Left.Get_Now_Omega_Radian()) > Booster.Get_Friction_Omega_Threshold()) /*&&
                 (Booster.Motor_Driver.Get_Now_Radian() - Booster.Motor_Driver.Get_Target_Radian() < 0.1f)*/
        )
        {
            if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
            if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
        }
        // C键控制摩擦轮
        if (VT13.Get_Keyboard_Key_Ctrl() == VT13_Key_Status_TRIG_FREE_PRESSED)
        {
            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {

                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                Fric_Status = Fric_Status_CLOSE;
                // Booster.Booster_User_Control_Type = Booster_User_Control_Type_DISABLE;
            }
            else
            {

                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                Fric_Status = Fric_Status_OPEN;
            }
        }
    }

}
#endif

#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Barrel_Cooling_Value;
    uint16_t Shooter_Barrel_Heat_Limit;
    int16_t Shooter_Barrel_Heat_Now = Referee.Get_Booster_17mm_1_Heat();
    Shooter_Barrel_Heat_Limit = Referee.Get_Booster_17mm_1_Heat_Max();
    Shooter_Barrel_Cooling_Value = Referee.Get_Booster_17mm_1_Heat_CD();

    //发送数据给云台
    CAN2_Chassis_Tx_Gimbal_Data[0] = Referee.Get_ID();
    CAN2_Chassis_Tx_Gimbal_Data[1] = Referee.Get_Game_Stage();
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &Shooter_Barrel_Heat_Limit, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 4, &Shooter_Barrel_Cooling_Value, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 6, &Shooter_Barrel_Heat_Now, sizeof(int16_t));
    
}
#endif
/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    #ifdef CHASSIS
    
    // 小陀螺 随动计算角速度
    if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Positive || 
        Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_NePositive)
    {
        Chassis.Set_Target_Omega(Chassis.Get_Spin_Omega());
    }
    else if (Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_FLLOW)
    {
        //随动yaw角度优化
        float temp_yaw,temp_reference;
        temp_yaw = Chassis_Angle;
        temp_reference = Reference_Angle;

        //处理6020绝对编码器0-360度突变问题    有时有用有时没用
        // if(Chassis_Angle > PI)
        //     temp_yaw = Chassis_Angle - 2 * PI; 
        // else if(Chassis_Angle < -PI)
        //     temp_yaw = Chassis_Angle + 2 * PI;
        // if(Reference_Angle > PI)
        //     temp_reference = Reference_Angle - 2 * PI;  

        //对角度的偏移，使角度线性化，即处理0-2PI的角度，使其偏移到-PI -- PI,使突变发生在-PI跳到PI
        if(Chassis_Angle < PI){
            temp_yaw = Chassis_Angle;
        }
        else if(Chassis_Angle > PI && Chassis_Angle < 2*PI){
            temp_yaw = Chassis_Angle - 2*PI;
        }

        if(temp_reference < PI){
            temp_reference = temp_reference;
        }
        else if(temp_reference > PI && temp_reference < 2*PI){
            temp_reference = temp_reference - 2*PI;
        }

        // 计算角度差，选择最短路径（优弧）
        float angle_diff = temp_reference - temp_yaw;

        // 优弧劣弧判断与优化
        if (angle_diff > PI)
        {
            angle_diff -= 2 * PI;
        }
        else if (angle_diff < -PI)
        {
            angle_diff += 2 * PI;
        }

        //随动环
        PID_Chassis_Fllow.Set_Target(temp_yaw + angle_diff);
        PID_Chassis_Fllow.Set_Now(temp_yaw);
        PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        //Chassis.Set_Target_Omega(0.0f);   
        Chassis.Set_Target_Omega(PID_Chassis_Fllow.Get_Out());            
    }
    // 底盘解算任务
    Chassis.Supercap.Set_Supercap_Mode(Supercap_Mode_ENABLE);
    Chassis.Set_Sprint_Status(Sprint_Status);

    Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);

    #elif defined(GIMBAL)

        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster.Set_Ture_Booster_Heat(Referee.Get_Booster_17mm_1_Heat());
        Booster.Set_Max_Booster_Heat(Referee.Get_Booster_17mm_1_Heat_Max());
        Booster.TIM_Calculate_PeriodElapsedCallback();        
        //传输数据给上位机
        MiniPC.TIM_Write_PeriodElapsedCallback();

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
             DR16.Get_Keyboard_Key_S() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_KEYBOARD;
    }
    else{
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            DR16_Control_Type = DR16_Control_Type_NONE;
    }
    
}

/**
 * @brief 判断VT13控制数据来源
 *
 */
void Class_Chariot::Judge_VT13_Control_Type()
{
    if (VT13.Get_Left_X() != 0 ||
        VT13.Get_Left_Y() != 0 ||
        VT13.Get_Right_X() != 0 ||
        VT13.Get_Right_Y() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_REMOTE;
    }
    else if (VT13.Get_Mouse_X() != 0 ||
             VT13.Get_Mouse_Y() != 0 ||
             VT13.Get_Mouse_Z() != 0 ||
             VT13.Get_Keyboard_Key_A() != 0 ||
             VT13.Get_Keyboard_Key_D() != 0 ||
             VT13.Get_Keyboard_Key_W() != 0 ||
             VT13.Get_Keyboard_Key_S() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_KEYBOARD;
    }
    else
    {
        if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            VT13_Control_Type = VT13_Control_Type_NONE;
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
    Judge_DR16_Control_Type();
    Judge_VT13_Control_Type();

    //底盘，云台，发射机构控制逻辑
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
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
        #ifdef CHASSIS

            
            Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            for (auto& wheel : Chassis.Motor_Wheel) {
                wheel.TIM_Alive_PeriodElapsedCallback();
            }
            if(mod50_mod3%3 == 0)
            {   
                Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
                TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;
            }
            //云台，随动掉线保护
            if(Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE || Gimbal_Status == Gimbal_Status_DISABLE)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);          
            }
        #elif defined(GIMBAL)

            if(mod50_mod3%3==0)
            {
                //判断底盘通讯在线状态
                TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();    
                DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
                VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;         
            }
                
            Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            //Gimbal.Motor_Pitch_LK6010.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

            Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
						
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
    //云台离线保护
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

    //底盘离线保护
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

            //转移为 在线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
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
            //转移为 刚离线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
			Chariot->Set_Pre_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chariot->Set_Pre_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
//            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
//            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart3); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}

void Class_FSM_Alive_Control_VT13::Reload_TIM_Status_PeriodElapsedCallback(){
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart6.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }

            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart6.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            	Chariot->Set_Pre_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        Chariot->Set_Pre_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
//            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
//            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart6); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}

#endif

#ifdef CHASSIS
void Class_Chariot::Chariot_Referee_UI_Tx_Callback(Enum_Referee_UI_Refresh_Status __Referee_UI_Refresh_Status)
{
    static uint8_t circle_flag = 0;
    static uint8_t cnt = 0,start_flag = 0;
    static uint8_t String_Index = 0;
    String_Index++;
    cnt++;
    if (String_Index >= 6 && String_Index < 10)
    {
        String_Index = 0;
    }
    if(String_Index >= 14){
        String_Index = 0;
    }
    if(cnt == 24 + circle_flag){            //从0开启，24+1刚好是偶数次，会导致每次只能进入cicle_flag == 0的时候
        String_Index = 8;
    }

    start_flag ++;
    if(start_flag < 50){
        __Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
    }
    else{
        __Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
    }
    
    switch (__Referee_UI_Refresh_Status)
    {
    case (Referee_UI_Refresh_Status_DISABLE):
    {
        // 摩擦轮状态
        if (Fric_Status == Fric_Status_OPEN)
        {
            Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Five, 0, 0x20 , Graphic_Color_GREEN, 20, 3, 1765, 880, "ON ", (sizeof("ON ") - 1), Referee_UI_CHANGE);
        }
        else
        {
            Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Five, 0, 0x20 , Graphic_Color_PINK, 20, 3, 1765, 880, "OFF", (sizeof("OFF") - 1), Referee_UI_CHANGE);
        }

        // 底盘状态
        if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
        {
            Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Six, 0, 0x30 , Graphic_Color_GREEN, 20, 3, 1765, 800, "FLLOW  ", (sizeof("FLLOW  ") - 1), Referee_UI_CHANGE);
        }
        else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive || 
                    Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
        {
            Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Six, 0, 0x30 , Graphic_Color_ORANGE, 20, 3, 1765, 800, "SPIN   ", (sizeof("SPIN   ") - 1), Referee_UI_CHANGE);
        }
        else
        {
            Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Six, 0, 0x30 , Graphic_Color_PINK, 20, 3, 1765, 800, "DISABLE", (sizeof("DISABLE") - 1), Referee_UI_CHANGE);
        }
        
        //超电电量
        Referee.Referee_UI_Draw_Line_Graphic_5(Referee.Get_ID(),Referee_UI_Two , 0, 0x50, 2, 25,665, 48,665 + (uint32_t)(590*Chassis.Supercap.Get_Now_Voltage()), 48, Referee_UI_CHANGE);

        //射击状态更新
        if(Booster_User_Control_Type == Booster_User_Control_Type_SINGLE){
            Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Six, 0, 0x60 , Graphic_Color_ORANGE, 20, 3, 1765, 760, "SINGLE ", (sizeof("SINGLE ") - 1), Referee_UI_CHANGE);
        }
        else if(Booster_User_Control_Type == Booster_User_Control_Type_MULTI){
            Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Six, 0, 0x60 , Graphic_Color_GREEN, 20, 3, 1765, 760, "MULTI  ", (sizeof("MULTI  ") - 1), Referee_UI_CHANGE);
        }
        else{
            Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Six, 0, 0x60 , Graphic_Color_PINK, 20, 3, 1765, 760, "DISABLE ", (sizeof("DISABLE ") - 1), Referee_UI_CHANGE);
        }

        if(MiniPC_Aim_Status == MinPC_Aim_Status_ENABLE)
        {
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(), Referee_UI_One, 0, 0x10, Graphic_Color_GREEN, 3,670, 270, 670 + 580, 270 + 580, Referee_UI_CHANGE);
        }
        else
        {
            Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(), Referee_UI_One, 0, 0x10, Graphic_Color_WHITE, 3,670, 270, 670 + 580, 270 + 580, Referee_UI_CHANGE);
        }

        
        if(Gimbal_Control_Type == Gimbal_Control_Type_NORMAL){
            Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Six, 0, 0x40 , Graphic_Color_GREEN, 20, 3, 1765, 840, "NORMAL ", (sizeof("NORMAL") - 1), Referee_UI_CHANGE);
        }
        else if(Gimbal_Control_Type == Gimbal_Control_Type_MINIPC){
            Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Six, 0, 0x40 , Graphic_Color_ORANGE, 20, 3, 1765, 840, "MINPC  ", (sizeof("MINPC  ") - 1), Referee_UI_CHANGE);
        }
        else{
            Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Six, 0, 0x40 , Graphic_Color_PINK, 20, 3, 1765, 840, "DISABLE", (sizeof("DISABLE") - 1), Referee_UI_CHANGE);
        }
    }
    break;
    case (Referee_UI_Refresh_Status_ENABLE):
    {
        //自瞄变色框
        Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(), Referee_UI_One, 0, 0x10, Graphic_Color_WHITE, 3,670, 270, 670 + 580, 270 + 580, Referee_UI_ADD);
        //摩擦轮状态
        Referee.Referee_UI_Draw_String(0, Referee.Get_ID(), Referee_UI_Five, 0, 0x20 , Graphic_Color_PINK, 20, 3, 1765, 880, "OFF", (sizeof("OFF") - 1), Referee_UI_ADD);
        //底盘状态
        Referee.Referee_UI_Draw_String(2, Referee.Get_ID(), Referee_UI_Six, 0, 0x30 , Graphic_Color_PINK, 20, 3, 1765, 800, "DISABLE", (sizeof("DISABLE") - 1), Referee_UI_ADD);
        //云台状态
        Referee.Referee_UI_Draw_String(1, Referee.Get_ID(), Referee_UI_Six, 0, 0x40 , Graphic_Color_PINK, 20, 3, 1765, 840, "DISABLE", (sizeof("DISABLE") - 1), Referee_UI_ADD);
        //射击状态
        Referee.Referee_UI_Draw_String(3, Referee.Get_ID(), Referee_UI_Six, 0, 0x60 , Graphic_Color_PINK, 20, 3, 1765, 760, "DISABLE ", (sizeof("DISABLE ") - 1), Referee_UI_ADD);
        //超电容量      （35+45)/2 = 40
        Referee.Referee_UI_Draw_Line_Graphic_5(Referee.Get_ID(),Referee_UI_Two , 0, 0x50, Graphic_Color_GREEN, 25,665, 48,665 + (uint32_t)(590*Chassis.Supercap.Get_Now_Voltage()), 48, Referee_UI_ADD);
    }
    break;
    }

    // 画瞄准辅助线
    if(!circle_flag){
        circle_flag = 1;
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Zero, 1, 0x01, Graphic_Color_ORANGE, 2, 620, 0, 770, 450, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_One, 1, 0x02, Graphic_Color_ORANGE, 2, 1300, 0, 1150, 450, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Two, 1, 0x03, Graphic_Color_ORANGE, 2, 960, 420, 960, 500, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Three, 1, 0x04, Graphic_Color_GREEN, 2, 910, 465, 1010, 465, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Four, 1, 0x05, Graphic_Color_GREEN, 2, 930, 445, 990, 445, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Five, 1, 0x06, Graphic_Color_GREEN, 2, 940, 420, 980, 420, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Six, 1, 0x07, Graphic_Color_GREEN, 2, 950, 395, 970, 395, Referee_UI_ADD);
    }
    else{
        circle_flag = 0;
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Zero, 2, 0x11, Graphic_Color_ORANGE, 2, 952, 540, 935, 540, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_One, 2, 0x22, Graphic_Color_ORANGE, 2, 968, 540, 985, 540, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Two, 2, 0x33, Graphic_Color_ORANGE, 2, 960, 548, 960, 565, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Line(Referee.Get_ID(), Referee_UI_Three, 2, 0x44, Graphic_Color_ORANGE, 2, 960, 532, 960, 515, Referee_UI_ADD);
        Referee.Referee_UI_Draw_Rectangle(Referee.Get_ID(), Referee_UI_Four, 2, 0x55, Graphic_Color_PURPLE, 2, 960, 540, 961, 541, Referee_UI_ADD);
    }
    
    // 超电容量 边框
    Referee.Referee_UI_Draw_Rectangle_Graphic_5(Referee.Get_ID(), Referee_UI_Zero, 0, 0x08, Graphic_Color_WHITE, 1,665, 35, 665 + 590, 35 + 25, Referee_UI_ADD);

    //摩擦轮等状态字符
    if (cnt == 35)
    {      //减小固定字符的发送频率
        String_Index = 10;
        Referee.Referee_UI_Draw_String(10, Referee.Get_ID(), Referee_UI_Zero, 1, 0x09, Graphic_Color_YELLOW, 20, 3, 1548, 880, "FRICTION :", (sizeof("FRICTION :") - 1), Referee_UI_ADD);
        Referee.Referee_UI_Draw_String(11, Referee.Get_ID(), Referee_UI_One, 1, 0x0A, Graphic_Color_YELLOW, 20, 3, 1548, 840, "GIMBAL   :", (sizeof("GIMBAL   :") - 1), Referee_UI_ADD);
        Referee.Referee_UI_Draw_String(12, Referee.Get_ID(), Referee_UI_Two, 1, 0x0B, Graphic_Color_YELLOW, 20, 3, 1548, 800, "CHASSIS  :", (sizeof("CHASSIS  :") - 1), Referee_UI_ADD);
        Referee.Referee_UI_Draw_String(13, Referee.Get_ID(), Referee_UI_Three, 1, 0x0C, Graphic_Color_YELLOW, 20, 3, 1548, 760, "SHOOT    :", (sizeof("SHOOT    :") - 1), Referee_UI_ADD);
        cnt = 0;
    }

    // 善后处理
    Referee.UART_Tx_Referee_UI(String_Index);
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
