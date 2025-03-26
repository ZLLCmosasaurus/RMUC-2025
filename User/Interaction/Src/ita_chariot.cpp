/**
 * @file ita_chariot.cpp
 * @author cjw by yssickjgd
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
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
    #ifdef CHASSIS
    
        //裁判系统
        Referee.Init(&huart10);

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();

    #elif defined(GIMBAL)
        
        Chassis.Set_Velocity_X_Max(4.0f);
        Chassis.Set_Velocity_Y_Max(4.0f);

        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);

        //遥控器
        DR16.Init(&huart5,&huart1);
        DR16_Dead_Zone = __DR16_Dead_Zone;   

        //云台
        Gimbal.Init();
        Gimbal.MiniPC = &MiniPC;

        //发射机构
        Booster_A.Set_Booster_Type(Booster_Type_A);
        Booster_B.Set_Booster_Type(Booster_Type_B);
        Booster_A.Referee = &Referee;
        Booster_A.Init(Booster_A.Get_Booster_Type());
        Booster_B.Referee = &Referee;
        Booster_B.Init(Booster_B.Get_Booster_Type());
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object);
        MiniPC.IMU = &Gimbal.Boardc_BMI;
        MiniPC.Referee = &Referee;

        //底盘随动环pid初始化(角度结算在上板完成)
        
        Chassis.Chassis_Follow_PID_Angle.Init(0.15f, 0.0f, 0.0f, 0.0f, 200.0f, 200.0f); //随动PID初始化

    #endif
}


#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Barrel_Cooling_Value_A,Shooter_Barrel_Cooling_Value_B;
    uint16_t Self_HP,Self_Outpost_HP,Oppo_Outpost_HP,Self_Base_HP,Ammo_number;
    uint8_t color;
    uint16_t Pre_HP[6] = {0};
    uint16_t HP[6] = {0};
    uint8_t Flag[6] = {0};
    float Count[6] = {0};
    float Pre_Count[6] = {0};
    //数据更新
    if(Referee.Get_ID() == Referee_Data_Robots_ID_RED_SENTRY_7)
    {
        color = 1;
        Oppo_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_OUTPOST_11);
        Self_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_OUTPOST_11);
        Self_Base_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_BASE_10);

        for(int i = 0;i < 6;i++)
        {
            Pre_HP[i] = HP[i];
        }
        
        HP[0] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_HERO_1);
        HP[1] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_ENGINEER_2);
        HP[2] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_3);
        HP[3] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_4);
        HP[4] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_5);
        HP[5] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_SENTRY_7);

    }
    else if(Referee.Get_ID() == Referee_Data_Robots_ID_BLUE_SENTRY_7)
    {
        color = 0;
        Oppo_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_OUTPOST_11);
        Self_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_OUTPOST_11);
        Self_Base_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_BASE_10);

        for(int i = 0;i < 6;i++)
        {
            Pre_HP[i] = HP[i];
        }

        HP[0] = Referee.Get_HP(Referee_Data_Robots_ID_RED_HERO_1);
        HP[1] = Referee.Get_HP(Referee_Data_Robots_ID_RED_ENGINEER_2);
        HP[2] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_3);
        HP[3] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_4);
        HP[4] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_5);
        HP[5] = Referee.Get_HP(Referee_Data_Robots_ID_RED_SENTRY_7);

    }

    Shooter_Barrel_Cooling_Value_B = Referee.Get_Booster_17mm_1_Heat();
    Shooter_Barrel_Cooling_Value_A = Referee.Get_Booster_17mm_2_Heat();
    Self_HP = Referee.Get_HP();
    Ammo_number = Referee.Get_17mm_Remaining();

    for(int i = 0;i < 6;i++)//无敌状态辨认
    {
        if(HP[i] > 0 && Pre_HP[i] == 0)
        {
            Flag[i] = 1;
            Pre_Count[i] = DWT_GetTimeline_s();
        }
        if(Flag[i] == 1)
        {
            Count[i] = DWT_GetTimeline_s();
        }
        if((Count[i] - Pre_Count[i]) > 10.f)
        {
            Flag[i] = 0;
            Pre_Count[i] = 0;
            Count[i] = 0;
        }
    }

    //发送数据给云台
    //A包
    CAN3_Chassis_Tx_Data_A[0] = Referee.Get_Game_Stage();
    CAN3_Chassis_Tx_Data_A[1] = Referee.Get_Remaining_Time() >> 8;
    CAN3_Chassis_Tx_Data_A[2] = Referee.Get_Remaining_Time();
    //memcpy(CAN3_Chassis_Tx_Data_A + 3, &Self_HP, sizeof(uint16_t));
    CAN3_Chassis_Tx_Data_A[3] = Referee.Get_HP() >> 8;
    CAN3_Chassis_Tx_Data_A[4] = Referee.Get_HP();
    //memcpy(CAN3_Chassis_Tx_Data_A + 5, &Self_Outpost_HP, sizeof(uint16_t));
    CAN3_Chassis_Tx_Data_A[5] = Self_Outpost_HP >> 8;
    CAN3_Chassis_Tx_Data_A[6] = Self_Outpost_HP;
    CAN3_Chassis_Tx_Data_A[7] = color << 7 | Flag[5] << 5 | Flag[4] << 4 | Flag[3] << 3 | Flag[2] << 2 | Flag[1] << 1 | Flag[0] << 0;

    //B包
    memcpy(CAN3_Chassis_Tx_Data_B + 0, &Self_Base_HP, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 2, &Oppo_Outpost_HP, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 4, &Ammo_number, sizeof(uint16_t));

    //C包
    memcpy(CAN3_Chassis_Tx_Data_C + 0, &Shooter_Barrel_Cooling_Value_A, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 2, &Shooter_Barrel_Cooling_Value_B, sizeof(uint16_t));
    
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS    
//控制类型字节
uint8_t control_type;
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data)
{   
    Gimbal_Alive_Flag++;
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //float映射到int16之后的速度
    int16_t tmp_velocity_x, tmp_velocity_y, tmp_omega;

    memcpy(&tmp_velocity_x,&CAN_Manage_Object->Rx_Buffer.Data[0],sizeof(int16_t));
    memcpy(&tmp_velocity_y,&CAN_Manage_Object->Rx_Buffer.Data[2],sizeof(int16_t));
    memcpy(&tmp_omega,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(int16_t));
    memcpy(&control_type,&CAN_Manage_Object->Rx_Buffer.Data[7],sizeof(uint8_t));
    
    #ifdef AGV
    chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x,-450,450,-4,4);
    chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y,-450,450,-4,4);
    chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -4.f, 4.f)/ Chassis_Radius;//映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度
    #else
    chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x,-450,450,-15.f,15.f);
    chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y,-450,450,-15.f,15.f);
    chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -80.f, 80.f);
    #endif
    chassis_control_type = (Enum_Chassis_Control_Type)control_type;
    //设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);
    if(chassis_omega < 1.f && chassis_omega > -1.f)chassis_omega = 0;
    //设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    #ifdef OMNI_WHEEL
        Chassis.Set_Target_Velocity_Y(-chassis_velocity_y);
    #endif
    Chassis.Set_Target_Omega(chassis_omega);//线速度
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
Referee_Rx_A_t PRE_CAN3_Chassis_Rx_Data_A;
Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
Referee_Rx_C_t CAN3_Chassis_Rx_Data_C;
volatile int atk_flag = 0;
int atk_cnt = 0;
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;
    uint16_t Shooter_Barrel_Cooling_Value;
    uint16_t Shooter_Barrel_Heat_Limit;
    switch(CAN_Manage_Object->Rx_Buffer.Header.Identifier){
        case (0x88):{
            memcpy(&PRE_CAN3_Chassis_Rx_Data_A, &CAN3_Chassis_Rx_Data_A, sizeof(Referee_Rx_A_t));
            CAN3_Chassis_Rx_Data_A.game_process = CAN_Manage_Object->Rx_Buffer.Data[0];
            CAN3_Chassis_Rx_Data_A.remaining_time = CAN_Manage_Object->Rx_Buffer.Data[1] << 8 | CAN_Manage_Object->Rx_Buffer.Data[2];
            CAN3_Chassis_Rx_Data_A.self_blood = CAN_Manage_Object->Rx_Buffer.Data[3] << 8 | CAN_Manage_Object->Rx_Buffer.Data[4];
            CAN3_Chassis_Rx_Data_A.self_outpost_HP = CAN_Manage_Object->Rx_Buffer.Data[5] << 8 | CAN_Manage_Object->Rx_Buffer.Data[6];
            CAN3_Chassis_Rx_Data_A.color_invincible_state = CAN_Manage_Object->Rx_Buffer.Data[7];
            if(PRE_CAN3_Chassis_Rx_Data_A.self_blood > CAN3_Chassis_Rx_Data_A.self_blood)
            {
                atk_flag = 1;
                atk_cnt = 0;
            }
            if(PRE_CAN3_Chassis_Rx_Data_A.self_blood == CAN3_Chassis_Rx_Data_A.self_blood)atk_cnt++;
            if(atk_cnt > 100)atk_flag = 0;
            break;
        }
        case (0x99):{
            memcpy(&CAN3_Chassis_Rx_Data_B, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_B_t));
            break;
        }
        case (0x78):{
            memcpy(&CAN3_Chassis_Rx_Data_C, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_C_t));
            Booster_A.Set_Heat(CAN3_Chassis_Rx_Data_C.Booster_Heat_CD_A);
            Booster_B.Set_Heat(CAN3_Chassis_Rx_Data_C.Booster_Heat_CD_B);
            break;
        }
    }
}
#endif


/**
 * @brief can回调函数给地盘发送数据
 *
 */
#ifdef GIMBAL
//控制类型字节
uint8_t control_type;
float test_ang,test_ome;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0; 
    //映射之后的目标速度 int16_t
    int16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_omega = 0;
    float chassis_omega = 0;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_omega = Chassis.Get_Target_Omega();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    
    test_ome = chassis_omega;
    //设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x,-4.f , 4.f ,-450,450);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(int16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y,-4.f , 4.f ,-450,450);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(int16_t));
    
    tmp_chassis_omega = -Math_Float_To_Int(chassis_omega,-4.f ,4.f ,-200,200);//随动环 逆时针为正所以加负号
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_omega, sizeof(int16_t));

    control_type =  (uint8_t)chassis_control_type;
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 7,&control_type ,sizeof(uint8_t));

}
#endif

float jjj = 0;
/**
 * @brief 底盘控制逻辑
 *
 */  		
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    //遥控器摇杆值
    float dr16_l_x, dr16_l_y;    
    //云台坐标系速度目标值 float
    float gimbal_velocity_x = 0, gimbal_velocity_y = 0;      
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_omega = 0;  
    //云台坐标系角度目标值 float
    float gimbal_angle = 0,chassis_angle = 0,relative_angle = 0;

	
    //排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

    //设定矩形到圆形映射进行控制
    gimbal_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max() ;
    gimbal_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max() ;

    //遥控器操作逻辑
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    switch(DR16_Left_Switch_Status){
        case (DR16_Switch_Status_UP):   // 左上 小陀螺模式
        {

            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            break;
        }
        case(DR16_Switch_Status_MIDDLE): // 左中 随动模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            break;
        }
        case(DR16_Switch_Status_DOWN):  // 左下 上位机
        {
            
            if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                Chassis.Set_Target_Velocity_X(0);
                Chassis.Set_Target_Velocity_Y(0);
                break;
            }

            volatile int MiniPC_Chassis_control_type = MiniPC.Get_Chassis_Control_Mode();
            switch(MiniPC_Chassis_control_type)
            {
                case(MiniPC_Chassis_Control_Mode_NORMAL):{
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                }
                break;
                case(MiniPC_Chassis_Control_Mode_FOLLOW):{
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                }
                break;
                case(MiniPC_Chassis_Control_Mode_SPIN):{
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
                }
                break;
                case(MiniPC_Chassis_Control_Mode_NORMAL_SPIN):{
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                }
                break;
            }
        }
    }

    Chassis.Set_Target_Velocity_X(gimbal_velocity_y);
    Chassis.Set_Target_Velocity_Y(-gimbal_velocity_x);//前x左y正

    //相对角度计算
    gimbal_angle = Gimbal.Motor_Main_Yaw.Get_Zero_Position();
    chassis_angle = Gimbal.Motor_Main_Yaw.Get_Now_Angle();
    relative_angle = chassis_angle - gimbal_angle;
    if(relative_angle < 1.f && relative_angle > -1.f)relative_angle = 0;
    
    MiniPC.Set_Gimbal_Now_Relative_Angle(relative_angle);
    relative_angle = DEG_TO_RAD * relative_angle;

    if(MiniPC.Get_MiniPC_Status() != MiniPC_Status_DISABLE && DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN){//上位机导航信息接收
        if(MiniPC.Get_Chassis_Target_Velocity_X() != 0 || MiniPC.Get_Chassis_Target_Velocity_Y() != 0){
            Chassis.Set_Target_Velocity_X(float(MiniPC.Get_Chassis_Target_Velocity_X() / 100.f));
            Chassis.Set_Target_Velocity_Y(float(MiniPC.Get_Chassis_Target_Velocity_Y() / 100.f));
        }
    }
   
    //云台到底盘坐标系转换
    volatile int Chassis_control_type = Chassis.Get_Chassis_Control_Type(); 
     switch(Chassis_control_type){
        case(Chassis_Control_Type_DISABLE):{//失能
            chassis_velocity_x = 0;
            chassis_velocity_y = 0;
            chassis_omega = 0;
            break;
        }
        case(Chassis_Control_Type_FLLOW):{//随动 附有非随动和受击陀螺逻辑
            if(Gimbal.Motor_Main_Yaw.Get_LK_Motor_Status() == LK_Motor_Status_DISABLE){//大yaw离线失能
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);           
            }
            else{//正常随动
                Chassis.Chassis_Follow_PID_Angle.Set_Target(0);
                Chassis.Chassis_Follow_PID_Angle.Set_Now(relative_angle * 180 / PI);
                Chassis.Chassis_Follow_PID_Angle.TIM_Adjust_PeriodElapsedCallback();
                chassis_omega = Chassis.Chassis_Follow_PID_Angle.Get_Out() / 2;
                chassis_omega = 0;//暂时设为0 取消随动
                chassis_velocity_x = Chassis.Get_Target_Velocity_X() * cos(relative_angle) - Chassis.Get_Target_Velocity_Y() * sin(relative_angle);
                chassis_velocity_y = Chassis.Get_Target_Velocity_X() * sin(relative_angle) + Chassis.Get_Target_Velocity_Y() * cos(relative_angle);
            }
            if(MiniPC.Get_MiniPC_Status() != MiniPC_Status_DISABLE
                    && DR16.Get_Left_Switch()  == DR16_Switch_Status_DOWN)
            {//正常非随动加受击陀螺
                if( MiniPC.Get_Chassis_Control_Mode() == MiniPC_Chassis_Control_Mode_NORMAL || 
                    MiniPC.Get_Chassis_Control_Mode() == MiniPC_Chassis_Control_Mode_NORMAL_SPIN)
                    chassis_omega = 0;
                break;
            }
            else if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_DISABLE 
                  && DR16.Get_Left_Switch()     == DR16_Switch_Status_DOWN){
                    chassis_omega = 0;
            }
            break;
        }
        case(Chassis_Control_Type_SPIN):{
            chassis_omega = 0.75f;//符合映射规则
            chassis_velocity_x = Chassis.Get_Target_Velocity_X() * cos(relative_angle) - Chassis.Get_Target_Velocity_Y() * sin(relative_angle);
            chassis_velocity_y = Chassis.Get_Target_Velocity_X() * sin(relative_angle) + Chassis.Get_Target_Velocity_Y() * cos(relative_angle);
            if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN && DR16.Get_Left_Switch() == DR16_Switch_Status_UP)chassis_omega = -0.75f;
            if(MiniPC.Get_Chassis_Target_Velocity_Omega() != 0.f && DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
            {
                chassis_omega = float(MiniPC.Get_Chassis_Target_Velocity_Omega() / 100.f);
            }
            break;
        }
        
    }
    if(chassis_omega > 4)chassis_omega = 4;
    if(chassis_omega < -4)chassis_omega = -4;
    
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);//前x左y正
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
        // 角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch_a, tmp_gimbal_pitch_b;
    // 遥控器摇杆值
    float dr16_y, dr16_r_y;

    // 排除遥控器死区
    dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
    dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch_a = Gimbal.Motor_Pitch_A.Get_Target_Angle();
    tmp_gimbal_pitch_b = Gimbal.Motor_Pitch_B.Get_Target_Angle();

    // 遥控器操作逻辑
    tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution;
    tmp_gimbal_pitch_a -= dr16_r_y * DR16_Pitch_Angle_Resolution;
    tmp_gimbal_pitch_b -= dr16_r_y * DR16_Pitch_Angle_Resolution;
    // 限制角度范围 处理yaw轴180度问题
    if ((tmp_gimbal_yaw ) > 180.0f)
    {
        tmp_gimbal_yaw -= (360.0f);
    }
    else if ((tmp_gimbal_yaw) < -180.0f)
    {
        tmp_gimbal_yaw += (360.0f);
    }

    if(tmp_gimbal_pitch_a > 18.0f)tmp_gimbal_pitch_a = 18.0f;
    if(tmp_gimbal_pitch_a < -25.0f)tmp_gimbal_pitch_a = -25.0f;
    if(tmp_gimbal_pitch_b > 18.0f)tmp_gimbal_pitch_b = 18.0f;
    if(tmp_gimbal_pitch_b < -25.0f)tmp_gimbal_pitch_b = -25.0f;

    if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下 上位机
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
        
    }
    else // 其余位置都是遥控器控制
    {
        // 中间遥控模式
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

        // 设定角度
        Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
        Gimbal.Set_Target_Pitch_Angle_A(tmp_gimbal_pitch_a);
        Gimbal.Set_Target_Pitch_Angle_B(tmp_gimbal_pitch_b);
    }
}
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */
int Booster_Sign = 0;
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    static uint8_t booster_sign = 0,booster_sign_a = 0,booster_sign_b = 0;
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    switch(DR16_Left_Switch_Status){
        case(DR16_Switch_Status_MIDDLE): // 左中 失能
        {
            Booster_A.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Booster_A.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            Booster_B.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Booster_B.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            // if(DR16.Get_Right_Switch() == DR16_Switch_Status_UP) // 右上 发射机构开火
            // {
            //     Booster_A.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            //     Booster_B.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            //     if (DR16.Get_Yaw() >= -0.2 && DR16.Get_Yaw() <= 0.2)
            //     {
            //         booster_sign = 0;
            //     }
            //     else if (DR16.Get_Yaw() >= 0.8 && booster_sign == 0) // 单发
            //     {
            //         Booster_A.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
            //         Booster_B.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
            //         booster_sign = 1;
            //     }
            //     else if (DR16.Get_Yaw() <= -0.8 && booster_sign == 0) // 五连发
            //     {
            //         Booster_A.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            //         Booster_B.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
            //         booster_sign = 1;
            //     }
            // }
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
            {
                Booster_A.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster_A.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                // if (DR16.Get_Yaw() >= -0.2 && DR16.Get_Yaw() <= 0.2)
                // {
                //     booster_sign_a = 0;
                // }
                // else if (DR16.Get_Yaw() >= 0.8 && booster_sign_a == 0) // 单发
                // {
                //     Booster_A.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                //     booster_sign_a = 1;
                // }
                // else if (DR16.Get_Yaw() <= -0.8 && booster_sign_a == 0) // 五连发
                // {
                //     Booster_A.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                //     booster_sign_a = 1;
                // }
            }
            else if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
            {
                Booster_B.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster_B.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                // if (DR16.Get_Yaw() >= -0.2 && DR16.Get_Yaw() <= 0.2)
                // {
                //     booster_sign_b = 0;
                // }
                // else if (DR16.Get_Yaw() >= 0.8 && booster_sign_b == 0) // 单发
                // {
                //     Booster_B.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                //     booster_sign_b = 1;
                // }
                // else if (DR16.Get_Yaw() <= -0.8 && booster_sign_b == 0) // 五连发
                // {
                //     Booster_B.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                //     booster_sign_b = 1;
                // }
            }
            break;
        }
        case(DR16_Switch_Status_DOWN):  // 左下 上位机
        {
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
            {
                Booster_A.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                if (DR16.Get_Yaw() >= -0.2 && DR16.Get_Yaw() <= 0.2)
                {
                    booster_sign_a = 0;
                }
                else if (DR16.Get_Yaw() >= 0.8 && booster_sign_a == 0) // 单发
                {
                    Booster_A.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    booster_sign_a = 1;
                }
                else if (DR16.Get_Yaw() <= -0.8 && booster_sign_a == 0) // 五连发
                {
                    Booster_A.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                    booster_sign_a = 1;
                }
            }
            else if(DR16.Get_Right_Switch() == DR16_Switch_Status_MIDDLE)
            {
                Booster_B.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                if (DR16.Get_Yaw() >= -0.2 && DR16.Get_Yaw() <= 0.2)
                {
                    booster_sign_b = 0;
                }
                else if (DR16.Get_Yaw() >= 0.8 && booster_sign_b == 0) // 单发
                {
                    Booster_B.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    booster_sign_b = 1;
                }
                else if (DR16.Get_Yaw() <= -0.8 && booster_sign_b == 0) // 五连发
                {
                    Booster_B.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                    booster_sign_b = 1;
                }
            }
            else if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
            {
                if(MiniPC.Get_Auto_aim_Status_A() == Auto_aim_Status_ENABLE)Booster_A.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                else if (MiniPC.Get_Auto_aim_Status_A() == Auto_aim_Status_DISABLE)Booster_A.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                if(MiniPC.Get_Auto_aim_Status_B() == Auto_aim_Status_ENABLE)Booster_B.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                else if (MiniPC.Get_Auto_aim_Status_B() == Auto_aim_Status_DISABLE)Booster_B.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
            break;
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
        // 底盘给云台发消息
        CAN_Chassis_Tx_Gimbal_Callback();

        //云台，随动掉线保护
        if(Get_Gimbal_Status() == DR16_Status_ENABLE || Referee.Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE){
            Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        }
        else
        {
            for(int i = 0; i < 4; i++){
                Chassis.Motor_Wheel[i].Set_Out(0.0f);
                Chassis.Motor_Steer[i].Set_Out(0.0f);
            }
        }

        
				
    #elif defined(GIMBAL)

        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster_A.TIM_Calculate_PeriodElapsedCallback();
        Booster_B.TIM_Calculate_PeriodElapsedCallback();
        //传输数据给上位机
        MiniPC.TIM_Write_PeriodElapsedCallback();
        //给下板发送数据
        CAN_Gimbal_Tx_Chassis_Callback();
    #endif   
}

/**
 * @brief 判断DR16控制数据来源
 *
 */
#ifdef GIMBAL
void Class_Chariot::Judge_DR16_Control_Type()
{
    DR16_Control_Type = DR16_Control_Type_REMOTE;
}
#endif
/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
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
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
        //TIM_Unline_Protect_PeriodElapsedCallback();
        #ifdef CHASSIS

            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            for (auto& wheel : Chassis.Motor_Wheel) {
                wheel.TIM_Alive_PeriodElapsedCallback();
            }
            for (auto& steer : Chassis.Motor_Steer) {
                steer.TIM_Alive_PeriodElapsedCallback();
            }
            
            if(mod50_mod3%3 == 0)
            {
                TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;
            }
            if(Get_Gimbal_Status() == Gimbal_Status_DISABLE){
                Chassis.Set_Target_Velocity_X(0);
                Chassis.Set_Target_Velocity_Y(0);
                Chassis.Set_Target_Omega(0);
            }   
        #elif defined(GIMBAL)

            if(mod50_mod3%3==0)
            {
                //判断底盘通讯在线状态
                TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();    
                DR16.TIM1msMod50_Alive_PeriodElapsedCallback();	   
                mod50_mod3 = 0;         
            }
            if(CAN3_Chassis_Rx_Data_A.game_process != 4)
            {
                if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                {
                    Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                    Booster_A.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                    Booster_B.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                }
            }
                
            Gimbal.Motor_Pitch_A.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Pitch_B.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw_A.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw_B.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Main_Yaw.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

            Booster_A.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            Booster_A.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster_A.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
            Booster_B.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            Booster_B.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster_B.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
						
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
            // Pre_Gimbal_Control_Type = Gimbal.Get_Gimbal_Control_Type();
            // Pre_Chassis_Control_Type = Chassis.Get_Chassis_Control_Type();
            // Pre_Booster_Control_Type = Booster_A.Get_Booster_Control_Type();
            // Pre_Booster_Control_Type = Booster_B.Get_Booster_Control_Type();
            //控制模块禁用
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster_A.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Booster_B.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            // 遥控器中途断联导致错误，重启 DMA
            if (huart5.ErrorCode)
            {
                HAL_UART_DMAStop(&huart5); // 停止以重启
                //HAL_Delay(10); // 等待错误结束
                HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
            }
        }
        else if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            // Gimbal.Set_Gimbal_Control_Type(Pre_Gimbal_Control_Type);
            // Chassis.Set_Chassis_Control_Type(Pre_Chassis_Control_Type);
            // Booster_A.Set_Booster_Control_Type(Pre_Booster_Control_Type);
            // Booster_B.Set_Booster_Control_Type(Pre_Booster_Control_Type);
        }

    #endif

    //底盘离线保护
    #ifdef CHASSIS
    if(Get_Gimbal_Status() == Gimbal_Status_DISABLE)
    {
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
    }
        
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
            if (huart5.ErrorCode)
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
            Chariot->Booster_A.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Booster_B.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
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
            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart5); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
