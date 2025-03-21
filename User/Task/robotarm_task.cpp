/**
 * @file crt_gimbal.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 云台电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "robotarm_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_Robotarm Robotarm;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 云台初始化
 *
 */
 float kp_Angle=22,ki_Angle=0.15,kd_Angle=0;
 float kp_Omega=20,ki_Omega=0.15,kd_Omega=0;
#define _CAN_PACKET_SET_POS_SPD

//#define _CAN_PACKET_SET_RUN_CONTROL
void Class_Robotarm::Init()
{	
  //遥控器离线控制 状态机
    FSM_Alive_Control.Robotarm = this;
    FSM_Alive_Control.Init(5, 0);
	//机械臂解算任务初始化
	Robotarm_Resolution.Robotarm = this;
	Robotarm_Resolution.Init(50 , 0);
	//裁判系统初始化
	Robotarm.Referee.Init(&huart6,0xA5);
	//
	DR16.Init(&huart3,&huart1);
	//板载imu初始化
	Boardc_BMI.Init();
	//底盘通信初始化
	Chassis_Communication.Init(&hcan2);
	//底盘YawPid初始化
	Chassis.PID_Yaw.Init(0.15f,0.f,0.f,0.f,0.f,4.0f);
	//MiniPC初始化
	MiniPc.Init(&MiniPC_USB_Manage_Object);
	//设置机械臂为正常模式
	Set_Robotarm_Control_Type(Robotarm_Control_Type_NORMAL);
	#ifdef _CAN_PACKET_SET_POS_SPD
    //1轴电机
  	Motor_Joint1.Init(&hcan2, AK_Motor_ID_0x01, CAN_PACKET_SET_POS_SPD , 30.0f , 2.0f);
		Motor_Joint1.Set_Target_Omega_SET_POS_SPD(1500);
		Motor_Joint1.Set_Target_Torque_SET_POS_SPD(200);

	//2轴电机
	Motor_Joint2.Init(&hcan1, AK_Motor_ID_0x04, CAN_PACKET_SET_POS_SPD ,30.0f , 2.0f);
		Motor_Joint2.Set_Target_Omega_SET_POS_SPD(1800);
		Motor_Joint2.Set_Target_Torque_SET_POS_SPD(250);

	#endif

    //3轴电机
	Motor_Joint3.Init(&hcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.94359f, 10.0f);

	//4-5轴左2006电机初始化
	Motor_Joint4.Init(&hcan1, DJI_Motor_ID_0x203,DJI_Motor_Control_Method_ANGLE); 
	Motor_Joint4.PID_Angle.Init(kp_Angle, ki_Angle, kd_Angle, 0.0f, 500.0f, 500.0f);
	Motor_Joint4.PID_Omega.Init(kp_Omega, ki_Omega, kd_Omega, 0.0f, 4000.0f, 10000.0f);
	Motor_Joint4.Slope.Init(0.5f, 0.5f);
	//4-5轴右2006电机初始化
	Motor_Joint5.Init(&hcan1, DJI_Motor_ID_0x202,DJI_Motor_Control_Method_ANGLE);
	Motor_Joint5.PID_Angle.Init(kp_Angle, ki_Angle, kd_Angle, 0.0f, 500.0f, 500.0f);
	Motor_Joint5.PID_Omega.Init(kp_Omega, ki_Omega, kd_Omega,0.0f, 4000.0f, 10000.0f);
	Motor_Joint5.Slope.Init(0.5f, 0.5f);
	//机械臂抬升初始化
	Arm_Uplift.Init(&hcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_ANGLE);
	Arm_Uplift.PID_Angle.Init(80.0f, 0.05f, 0.0f, 0.0f, 180.0f, 250.0f);
	Arm_Uplift.PID_Omega.Init(100.0f, 0.01f, 0.0f, 0.0f, 2000.0f, 10000.0f);

	Relay1.Init(GPIOE,GPIO_PIN_11);
	Relay2.Init(GPIOE,GPIO_PIN_13);
}
/***********************************校准完后机械臂控制逻辑*********************************************/
float test_dm_o=.f;
void Class_Robotarm::Output()
{
	#ifdef _CAN_PACKET_SET_POS_SPD
	// 关节1 Ak电机
	Motor_Joint1.Set_AK_Motor_Control_Method(CAN_PACKET_SET_POS_SPD);
	Motor_Joint1.Set_Target_Angle(Jonit_AngleInit[0] + Robotarm.Get_Joint_Offset_Angle(1));

	// 关节2 Ak电机
	Robotarm.Motor_Joint2.Set_AK_Motor_Control_Method(CAN_PACKET_SET_POS_SPD);
	Motor_Joint2.Set_Target_Angle(Jonit_AngleInit[1] + (Robotarm.Get_Joint_Offset_Angle(2) - 223.0f));
	#endif

	// 关节3 达妙电机
	Motor_Joint3.Set_Target_Angle((Jonit_AngleInit[2]) * DEG_TO_RAD);
	Motor_Joint3.Set_Target_Omega(test_dm_o);
	Motor_Joint3.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

	//左电机
	Robotarm.Motor_Joint4.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
	float motor_4_tmp_target_angle = -Jonit_AngleInit[3] - Jonit_AngleInit[4];
	Robotarm.Motor_Joint4.Set_Target_Angle(motor_4_tmp_target_angle + Robotarm.Get_Joint_Offset_Angle(4));

	//右电机
	Robotarm.Motor_Joint5.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
	float motor_5_tmp_target_angle = -Jonit_AngleInit[3] + Jonit_AngleInit[4];
	Robotarm.Motor_Joint5.Set_Target_Angle(motor_5_tmp_target_angle + Robotarm.Get_Joint_Offset_Angle(5));

} 
void Class_Robotarm::TIM_Robotarm_Task_PeriodElapsedCallback()
{	


	Motor_Joint1.Task_Process_PeriodElapsedCallback();
  Motor_Joint2.Task_Process_PeriodElapsedCallback();
  Motor_Joint3.TIM_Process_PeriodElapsedCallback();
	Motor_Joint4.TIM_PID_PeriodElapsedCallback();
	//校准的时候不进行右边2006电机控制
	//if (init_finished)
	//{
		Motor_Joint5.TIM_PID_PeriodElapsedCallback();
		Robotarm.Arm_Uplift.Calculate_Actual_Up_Length(Robotarm.Get_Joint_Offset_Angle(6));
	//}

 	Robotarm.Arm_Uplift.TIM_PID_PeriodElapsedCallback();

	Robotarm.Set_Joint_World_Angle_Now(1,Motor_Joint1.Get_Now_Angle()-Robotarm.Get_Joint_Offset_Angle(1));
	Robotarm.Set_Joint_World_Angle_Now(2,Motor_Joint2.Get_Now_Angle()-Robotarm.Get_Joint_Offset_Angle(2)+223);
	Robotarm.Set_Joint_World_Angle_Now(3,Motor_Joint3.Get_Now_Angle());
	Robotarm.Set_Joint_World_Angle_Now(4,Motor_Joint4.Get_Now_Angle()-Robotarm.Get_Joint_Offset_Angle(4));
	Robotarm.Set_Joint_World_Angle_Now(5,Motor_Joint5.Get_Now_Angle()-Robotarm.Get_Joint_Offset_Angle(5));
	
}
void Class_Robotarm::Task_Alive_PeriodElapsedCallback()
{
	FSM_Alive_Control.Reload_TIM_Statu_PeriodElapsedCallback();
	DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
	Motor_Joint1.Task_Alive_PeriodElapsedCallback();
	Motor_Joint2.Task_Alive_PeriodElapsedCallback();
	Motor_Joint3.TIM_Alive_PeriodElapsedCallback();
	Motor_Joint4.TIM_Alive_PeriodElapsedCallback();
	Motor_Joint5.TIM_Alive_PeriodElapsedCallback();
}
void Class_Robotarm::TIM_Robotarm_Disable_PeriodElapsedCallback()
{
	Arm_Uplift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
	Arm_Uplift.Set_Target_Torque(0.0f);
	Motor_Joint1.Set_AK_Motor_Control_Method(CAN_PACKET_DIS_RUN_CONTROL);
	Motor_Joint2.Set_AK_Motor_Control_Method(CAN_PACKET_DIS_RUN_CONTROL);
	Motor_Joint3.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
	Motor_Joint4.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
	Motor_Joint4.Set_Target_Torque(0.0f);
	Motor_Joint5.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
	Motor_Joint5.Set_Target_Torque(0.0f);
}
/*********************************抬升机构************************************************************/
void Class_RoRobotic_Arm_Uplift::Calculate_Actual_Up_Length(float Off_Set_Angle)
{
	Actual_Up_Length = (Get_Now_Angle()-Off_Set_Angle)*K_Actual;
}
void Class_Robotarm::Control_RoRobotic_Arm_Uplift()
{
	if(DR16.Get_Left_Switch()==DR16_Switch_Status_UP&&DR16.Get_DR16_Status()==DR16_Status_ENABLE)
	{
		Arm_Uplift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
		Arm_Uplift.Target_Up_Length = DR16.Get_Right_Y()*Arm_Uplift.K_Target+Arm_Uplift.Actual_Up_Length;
    	Math_Constrain(Arm_Uplift.Target_Up_Length,0.0f,40.0f);
	}
	else
	{
		Arm_Uplift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
		Arm_Uplift.Set_Target_Torque(0.0f);
	}
}
void Class_RoRobotic_Arm_Uplift::TIM_PID_PeriodElapsedCallback()
{
  switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
			
        PID_Angle.Set_Target(Target_Up_Length);
        PID_Angle.Set_Now(Actual_Up_Length);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Angle = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Angle);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
		case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Angle);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}

/************************************底盘控制逻辑*********************************************/
void Class_Robotarm::Control_Chassis_Task()
{
	float dr16_l_x, dr16_l_y, dr16_r_x;
	float chassis_velocity_x = 0, chassis_velocity_y = 0;
  float chassis_omega = 0;

	// 设置底盘速度
	if (DR16.Get_DR16_Status() == DR16_Status_ENABLE)
	{
	 if (Get_DR16_Control_Type() == DR16_Control_Type_REMOTE)
	{
		// 设置底盘控制方式
		Chassis_control_type = CHASSIS_Control_Type_FLLOW;
		// 排除遥控器死区
		//  dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
		//  dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
		dr16_l_x = DR16.Get_Left_X();
		dr16_l_y = DR16.Get_Left_Y();
		dr16_r_x = DR16.Get_Right_X();
		// 设定矩形到圆形映射进行控制
		Chassis.Chassis_Vx = dr16_l_x * Chassis.Max_Chassis_Vx;
		Chassis.Chassis_Vy = dr16_l_y * Chassis.Max_Chassis_Vy;
		Chassis.Chassis_Omega = dr16_r_x * Chassis.Max_Chassis_Omega;
	}
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
            chassis_velocity_x = -Chassis.Max_Chassis_Vx / DR16_Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
        {
            chassis_velocity_x = Chassis.Max_Chassis_Vx / DR16_Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
        {
            chassis_velocity_y = Chassis.Max_Chassis_Vy / DR16_Mouse_Chassis_Shift;
        }
        if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
        {
            chassis_velocity_y = -Chassis.Max_Chassis_Vy / DR16_Mouse_Chassis_Shift;
        }
		if (DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_PRESSED) // 按下q键打开气泵
        {	if(Robotarm.Relay1.Get_Open_flag()==1)
			Robotarm.Relay1.Set_Open_flag(0);
			else
			Robotarm.Relay1.Set_Open_flag(1);
        }
		Chassis.Actual_Yaw=-Robotarm.Boardc_BMI.Get_Angle_YawTotal();
		Chassis.Target_Yaw+=Robotarm.DR16.Get_Mouse_X()*10.0f;
		Chassis.TIM_PID_PeriodElapsedCallback();
		chassis_omega=Chassis.PID_Yaw.Get_Out();
				
				//				 if (DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_PRESSED) // y轴
//        {
//            chassis_omega = -2.5;
//        }
//        if (DR16.Get_Keyboard_Key_E()  == DR16_Key_Status_PRESSED)
//        {
//            chassis_omega = 2.5;
//        }
//        if (DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_TRIG_FREE_PRESSED) // Q键切换小陀螺与随动
//        {
//            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
//            {
//                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
//                chassis_omega = Chassis.Get_Spin_Omega();
//            }
//            else
//                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
//        }

//        if (DR16.Get_Keyboard_Key_G() == DR16_Key_Status_PRESSED) // 按下G键刷新UI
//        {
//            Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
//        }
//        else
//        {
//            Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
//        }

			Chassis.Chassis_Vx =chassis_velocity_x;
			Chassis.Chassis_Vy =chassis_velocity_y;
		  Chassis.Chassis_Omega=chassis_omega;
    }
	}
		
		else
		{
		Chassis_control_type = CHASSIS_Control_Type_DISABLE;
		Chassis.Chassis_Vx = 0.0f;
		Chassis.Chassis_Vy = 0.0f;
		Chassis.Chassis_Omega = 0.0f;
		}
	
}
uint8_t Control_Type = 0;
uint8_t Now_statu=0;
void Class_Robotarm::CAN_Gimbal_Tx_Chassis()
{
	uint16_t tmp_chassis = 0;
	tmp_chassis = Math_Float_To_Int(Chassis.Chassis_Vx,-Chassis.Max_Chassis_Vx,Chassis.Max_Chassis_Vx,0,0xFFFF);
	memcpy(CAN2_Gimbal_Tx_Chassis_Data,&tmp_chassis,2);
	tmp_chassis = Math_Float_To_Int(Chassis.Chassis_Vy,-Chassis.Max_Chassis_Vy,Chassis.Max_Chassis_Vy,0,0xFFFF);
	memcpy(CAN2_Gimbal_Tx_Chassis_Data+2,&tmp_chassis,2);
	tmp_chassis = Math_Float_To_Int(Chassis.Chassis_Omega,-Chassis.Max_Chassis_Omega,Chassis.Max_Chassis_Omega,0,0xFFFF);
	memcpy(CAN2_Gimbal_Tx_Chassis_Data+4,&tmp_chassis,2);
	Control_Type = (uint8_t)Chassis_control_type;
	memcpy(CAN2_Gimbal_Tx_Chassis_Data+6,&Control_Type,1);
	Now_statu=Robotarm_Resolution.Get_Now_Status_Serial();
	memcpy(CAN2_Gimbal_Tx_Chassis_Data+7,&Now_statu,1);
}

/************************************机械臂校准逻辑*********************************************/
/**
 * @brief C620校准函数
 *
 * @param Motor C620指针
 * @param num 关节角标
 * @param Cali_Omega 校准速度
 * @param Cali_Max_Out 校准最大电流输出
 * @param Target_Angle 关节目标角度
 */

bool Class_Robotarm::Motor_Calibration(Class_DJI_Motor_C620 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle)
{
	//记录电机堵转时间
	static uint16_t count=0;
	//设置为速度环校准
	Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Motor.Set_Target_Omega_Angle(Cali_Omega);
	Motor.PID_Omega.Set_Out_Max(Cali_Max_Out);
	
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if((fabs(Motor.Get_Now_Torque()) >= 1500) && (fabs(Motor.Get_Now_Omega_Radian()) < 0.02f*PI))
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 200)
		{
			count = 0;
			//改为角度环，设置关节角度
			Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Joint_World_Angle[4 - 1] = Target_Angle;
			Motor.Set_Target_Angle(Motor.Get_Now_Angle());
			Joint_Offset_Angle[4 - 1] = Motor.Get_Now_Angle();
			Motor.PID_Omega.Set_Out_Max(5000);
			return true;
		}
	}
  else
  {
    count=0;
  }
	return false;
}
/**
 * @brief AK校准函数
 *
 * @param Motor AK80_6指针
 * @param num 关节角标 1为1轴 2为2轴
 * @param Cali_Omega 校准途中的转速 电气转速
 * @param Target_Angle 关节目标角度
 */
bool Class_Robotarm::Motor_Calibration(Class_AK_Motor_80_6 &Motor,uint8_t num,float Cali_Omega,float Target_Angle)
{
	static uint16_t count[2] = {0,0};
	Motor.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RPM);

	if(Motor.Get_Rx_Data() != 0)//如果收到反馈帧率 则进入校准
	{
		Motor.Set_Target_Omega(Cali_Omega);
		if(fabs(Motor.Get_Now_Torque()) >= 10.0f)
		{
			count[num - 1]++;
			if(count[num - 1] >= 300)
			{
				//记录当前限位角度
					Motor.Set_Target_Omega(0);
				Joint_Offset_Angle[num - 1] = Motor.Get_Now_Angle();
					Motor.Set_AK_Motor_Control_Method(CAN_PACKET_SET_POS_SPD);
				if(num==1)
				Motor.Set_Target_Angle(Joint_Offset_Angle[num - 1]+5);
				else if(num==2)
				Motor.Set_Target_Angle(Joint_Offset_Angle[num - 1]-2);
				Motor.Task_Process_PeriodElapsedCallback();
				//清零计数
				count[num - 1] = 0; 
				//返回校准成功
				return true;
			}
		}
	}
	return false;
}
/**
 * @brief C610校准函数
 *
 * @param Motor C610指针
 * @param num 关节角标
 * @param Cali_Omega 校准速度
 * @param Cali_Max_Out 校准最大电流输出
 * @param Target_Angle 关节目标角度
 */
bool Class_Robotarm::Motor_Calibration(Class_DJI_Motor_C610 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle)
{
	//记录电机堵转时间
	static uint16_t count;
	//设置为速度环校准
	Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Motor.Set_Target_Omega_Angle(Cali_Omega);
	Motor.PID_Omega.Set_Out_Max(Cali_Max_Out);
	
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Motor.Get_Now_Torque()) >= 1000) && (fabs(Motor.Get_Now_Omega_Radian()) < 0.01f*PI) )
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 200)
		{
			count = 0;
			//改为角度环，设置关节角度
			Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			//Joint_World_Angle[num-1] = Target_Angle;
			//分别记录左右2006的零位角度
			Joint_Offset_Angle[4-1] = Motor.Get_Now_Angle();
			Joint_Offset_Angle[5-1] = Motor_Joint5.Get_Now_Angle();
			Motor.Set_Target_Angle(Motor.Get_Now_Angle());
			Motor_Joint5.Set_Target_Angle(Motor_Joint5.Get_Now_Angle());
			Motor.PID_Omega.Set_Out_Max(5000);
			return true;
		}	
	}
	else
	{
		count=0;
	}
	return false;
}

/**
 * @brief C610校准函数
 *
 * @param Cali_Omega 校准速度
 * @param Cali_Max_Out 校准最大电流输出
 */
bool Class_Robotarm::Motor_Calibration(float Cali_Omega,float Cali_Max_Out)
{
	//记录电机堵转时间
	static uint16_t count;
	//设置为速度环校准
	Motor_Joint4.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Motor_Joint4.Set_Target_Omega_Angle(Cali_Omega);
	// Motor_Joint4.PID_Omega.Set_Out_Max(Cali_Max_Out);
	
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Motor_Joint4.Get_Now_Torque()) >= Cali_Max_Out) && (fabs(Motor_Joint4.Get_Now_Omega_Radian()) < 0.01f*PI) )
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 200)
		{
			count = 0;
			//改为角度环，设置关节角度
			Motor_Joint4.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			//Joint_World_Angle[num-1] = Target_Angle;
			//分别记录左右2006的零位角度
			Joint_Offset_Angle[4-1] = Motor_Joint4.Get_Now_Angle();
			Joint_Offset_Angle[5-1] = Motor_Joint5.Get_Now_Angle();
			Motor_Joint4.Set_Target_Angle(Motor_Joint4.Get_Now_Angle());
			Motor_Joint5.Set_Target_Angle(Motor_Joint5.Get_Now_Angle());
			return true;
		}	
	}
	else
	{
		count=0;
	}
	return false;
}


bool Class_Robotarm::Motor_Calibration_Uplift(Class_RoRobotic_Arm_Uplift &Uplift,float Cali_Omega,float Cali_Max_Out)
{
	static uint16_t count;
	//设置为速度环校准
	Uplift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Uplift.Set_Target_Omega_Angle(Cali_Omega);
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Uplift.Get_Now_Torque()) >= Cali_Max_Out) && (fabs(Uplift.Get_Now_Omega_Radian()) < 0.01f*PI) )
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 200)
		{
			count = 0;
			//改为角度环
			Uplift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Joint_Offset_Angle[5] = Uplift.Get_Now_Angle();
			//记录最低高度
			return true;
		}	
	}
	else
	{
		count=0;
	}
	return false;
}
/**
 * @brief 机械臂校准任务
 *
 */
float test_speed_2006=10;
float test_speed_3508=10;
uint8_t Arm_Cal_Flag_test =0;
bool Class_Robotarm::Robotarm_Calibration()
{
    //电机校准标志位，1位表示一个电机
	//关节1校准
	static uint8_t Arm_Cal_Flag =0;
	if((Arm_Cal_Flag & (1<<1)) == 0)
	{
		if(Motor_Calibration(Motor_Joint1,1,-300.0f,85.422f) == true)
		{
			Arm_Cal_Flag |= (1<<1);
			
		}
	}
	//关节2校准
	if((Arm_Cal_Flag & (1<<2)) == 0)
	{
		if(Motor_Calibration(Motor_Joint2,2,300.0f,-170.845f) == true)
		{
			Arm_Cal_Flag |= (1<<2);
		}
	}
	
	 //关节4 5校准
	 if((Arm_Cal_Flag & (1<<3)) == 0)
	 {
		//左右电机同步校准
	 	if(Motor_Calibration(200,1500) == true)
	 	{
	 		Arm_Cal_Flag |= (1<<3);
	 	}
	 }
	 if((Arm_Cal_Flag & (1<<4)) == 0)
	 {
	 if(Motor_Calibration_Uplift(Arm_Uplift,-100,2000) == true)
	 	{
	 		Arm_Cal_Flag |= (1<<4);
	 	}
	 }
	Robotarm.Motor_Joint3.Set_Target_Angle(0);
	Robotarm.Motor_Joint3.Set_Target_Omega(1);
	Robotarm.Motor_Joint3.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE); 
	//校准完毕后，标志位全部置1
	 Arm_Cal_Flag_test=Arm_Cal_Flag;
	if(Arm_Cal_Flag == 30)
	{
		Arm_Cal_Flag = 0;
		return true;
	}
	return false;
}
uint8_t test_arm;
bool Class_Robotarm::Robotarm_Angle_verification(float *Angle_now,float *Angle_target)
{
	static uint8_t Arm_Check_Flag =0;
	
	for(int i=1;i<=5;i++)
	{
		if(i!=3)
	{
		if((Arm_Check_Flag & (1<<i)) == 0)
		{
			if(Math_Abs(Angle_now[i-1]-Angle_target[i-1])<10)
			{
				Arm_Check_Flag |= (1<<i);
			}
		}
	}
}
	test_arm=Arm_Check_Flag;
//	if((Arm_Check_Flag == 54)||(Arm_Check_Flag == 62))
//	{
//		Arm_Check_Flag = 0;
//		return true;
//	}
	if(Arm_Check_Flag>=6)
	{
		Arm_Check_Flag = 0;
		return true;
	}
	return false;
}
void Class_Robotarm::Judge_DR16_Control_Type()
{
    if (DR16.Get_Left_X() != 0 ||
         DR16.Get_Left_Y() != 0 ||
        DR16.Get_Right_X() != 0)
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
}


//遥控器状态机函数
void Class_FSM_Alive_Control::Reload_TIM_Statu_PeriodElapsedCallback()
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
            if(Robotarm->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
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
            if(Robotarm->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
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
            if(Robotarm->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
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


//#ifdef _OLD
/**
 * @brief 机械臂解算回调函数
 *
 */
uint8_t Angle_Play_Flag =0;

void Class_Robotarm_Resolution::Reload_Task_Status_PeriodElapsedCallback()
{
	static uint16_t mod;
	if(Robotarm->Robotarm_Control_Type == Robotarm_Control_Type_NORMAL)
	{
		Status[Now_Status_Serial].Time++;
		//自己接着编写状态转移函数
		switch (Now_Status_Serial)
		{
			//校准任务
			case (Robotarm_Task_Status_Calibration):
			{
				//电机校准完成后进入下一状态
				if(Robotarm->init_finished)
				{
					Set_Status(Robotarm_Task_Status_Wait_Order);
				}
			}
			break;
			case (Robotarm_Task_Status_Error):
			{
				//等待错误指令
			}
			break;
			case (Robotarm_Task_Status_Wait_Order):
			{
				//给出指令
				if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP)//开始取矿石
				{
					Set_Status(Robotarm_Task_Status_Pick_First_Sliver);
				}
				if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)//开始取金矿石
				{
					Set_Status(Robotarm_Task_Status_Pick_Gold_1);
				}
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_1):
			{
				Robotarm->Arm_Uplift.Target_Up_Length=25;
				memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Gold_1, 6 * sizeof(float));
					if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<2)
				{
					if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
				{			
						Set_Status(Robotarm_Task_Status_Pick_Gold_2);
				}
			
				}
		
			}	
			break;
			case(Robotarm_Task_Status_Pick_Gold_2)://这个状态辅助机构x轴伸出
			{
					 if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP)
					{Set_Status(Robotarm_Task_Status_Pick_Gold_3);}
				 if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
				 {Set_Status(Robotarm_Task_Status_Pick_Gold_4);}
				
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_3)://过渡态，这个状态辅助机构抬起一点，把矿从槽里提起
			{
					 if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_MIDDLE)
					{Set_Status(Robotarm_Task_Status_Pick_Gold_2);}
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_4)://这个状态辅助机构x轴回收一半
			{
					 if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
					{
						mod++;
						if(mod==500)//按键消抖防止状态跳变
					Set_Status(Robotarm_Task_Status_Pick_Gold_5);
					}
					else mod=0;
			}
			break;
			case(Robotarm_Task_Status_Pick_Gold_5)://这个状态辅助机构x轴回收一半之后，大臂进入准备取出第三个金矿的姿态
			{
				
				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Gold[5];//优先高度
				if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<6)
				{
						memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Gold, 6 * sizeof(float));
						if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
						{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN){
						Set_Status(Robotarm_Task_Status_Exchange);}
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
						Robotarm->Arm_Uplift.Target_Up_Length+=3.5f;
						memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Gold_UP, 6 * sizeof(float));}
						}
				}
			}
			break;
			

			
			
			
			
			case (Robotarm_Task_Status_Pick_First_Sliver):
			{
				//设置对应角度，使机械臂到达第一个矿石的指定位置
				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Fisrt[5];
				memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Fisrt, 6 * sizeof(float));
				if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
				{			
//				//等待指令，
					//Robotarm->Relay1.Set_Open_flag(1);
					if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
						//Set_Status(Robotarm_Task_Status_Place_First_Sliver);//中期考核暂时不放矿仓
						Set_Status(Robotarm_Task_Status_First_Sliver_Test);
						}
					else if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
					{
						Robotarm->Arm_Uplift.Target_Up_Length-=5;
					}
				}
			}
			break;
			case (Robotarm_Task_Status_First_Sliver_Test):
			{
				Robotarm->Arm_Uplift.Target_Up_Length=15;
					if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<2)
				{
						Set_Status(Robotarm_Task_Status_First_Sliver_Test_2);
				}
			}
			break;
			case (Robotarm_Task_Status_First_Sliver_Test_2):
			{
				Robotarm->Arm_Uplift.Target_Up_Length=13;
				memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_On_The_Way, 6 * sizeof(float));
			
			}
			break;
			case (Robotarm_Task_Status_Place_First_Sliver):
			{
				//设置对应角度，使机械臂到达第一个矿石的矿仓位置
				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Place_Fisrt[5];//优先高度
				if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<3)
				{
					memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Place_Fisrt, 6 * sizeof(float));
					if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
					{	
						//Robotarm->Relay1.Set_Open_flag(0);
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP)
						Set_Status(Robotarm_Task_Status_Pick_Second_Sliver);
					}
				}
				
			 if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
					{Set_Status(Robotarm_Task_Status_Wait_Order);}
			}
			break;
			case (Robotarm_Task_Status_Pick_Second_Sliver):
			{
				
				memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Second, 6 * sizeof(float));//优先角度
				if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
				{		
							Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Second[5];
							//Robotarm->Relay1.Set_Open_flag(1);
					//				//等待指令，
					if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<3){

	
					if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
					Set_Status(Robotarm_Task_Status_Place_Second_Sliver);
					}
				
				}
				}

				if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
				{	Robotarm->Arm_Uplift.Target_Up_Length-=5;}
				}
			
			break;
			case (Robotarm_Task_Status_Place_Second_Sliver):
			{

					//设置对应角度，使机械臂到达第二个矿石的矿仓位置
				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Place_Second[5];//优先高度
				if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<3)
				{
						memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Place_Second, 6 * sizeof(float));
						if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
						{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
						Set_Status(Robotarm_Task_Status_Pick_Third_Sliver);}
						}
				}
			 if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
					{Set_Status(Robotarm_Task_Status_Wait_Order);}
			break;
			}
			case (Robotarm_Task_Status_Pick_Third_Sliver):
			{
				//设置对应角度，使机械臂到达第一个矿石的指定位置
			 memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Third, 6 * sizeof(float));
			if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
				{		
						Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Third[5];
					if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<3){
					if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
					Set_Status(Robotarm_Task_Status_Exchange);}
				
				}
				}
				if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
				{Set_Status(Robotarm_Task_Status_Wait_Order);}	
			}
			break;
			case (Robotarm_Task_Status_Pick_First_Gold):
			{
		
				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Gold[5];//优先高度
				if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<1.2)
				{
						memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Gold, 6 * sizeof(float));
						if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
						{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN){
						Set_Status(Robotarm_Task_Status_Place_First_Gold);}
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
						Robotarm->Arm_Uplift.Target_Up_Length+=2.0f;}
						}
				}
			}
			break;
			case (Robotarm_Task_Status_Place_First_Gold):
			{

				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Place_Fisrt[5];//优先高度
				if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<3)
				{
					memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Place_Fisrt, 6 * sizeof(float));
					if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
					{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
						Set_Status(Robotarm_Task_Status_Pick_Second_Gold);
					}
				}
			}
			break;
			case (Robotarm_Task_Status_Pick_Second_Gold):
			{
				
				//if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<1.2)
				//{
						memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Gold, 6 * sizeof(float));
						if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
						{
						Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Gold[5];//优先高度
						if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<1.2)
						{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN){
						Set_Status(Robotarm_Task_Status_Place_Second_Gold);}
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
						Robotarm->Arm_Uplift.Target_Up_Length+=2.0f;}
						}
				}
				
			}
			break;
			case (Robotarm_Task_Status_Place_Second_Gold):
			{
				Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Place_Second[5];//优先高度
				if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<3)
				{
					memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Place_Second, 6 * sizeof(float));
					if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
					{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN)
						Set_Status(Robotarm_Task_Status_Pick_Third_Gold);
					}
				}
			}
			break;
			case (Robotarm_Task_Status_Pick_Third_Gold):
			{
				
				//if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<1.2)
				//{
						memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_Pick_Gold, 6 * sizeof(float));
						if(Robotarm->Robotarm_Angle_verification(Robotarm->Joint_World_Angle_Now,Robotarm->Jonit_AngleInit)==true)
						{
						Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_Pick_Gold[5];//优先高度
						if(Math_Abs(Robotarm->Arm_Uplift.Actual_Up_Length-Robotarm->Arm_Uplift.Target_Up_Length)<1.2)
						{
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_DOWN){
						Set_Status(Robotarm_Task_Status_Exchange);}
						if(Robotarm->DR16.Get_Left_Switch()==DR16_Switch_Status_UP){
						Robotarm->Arm_Uplift.Target_Up_Length+=2.0f;}
						}
						}
				
			}
			break;
	
			case (Robotarm_Task_Status_Exchange)://兑换模式
			{
			Robotarm->Arm_Uplift.Target_Up_Length=Robotarm->Angle_On_The_Way[5];	
			memcpy(Robotarm->Jonit_AngleInit, Robotarm->Angle_On_The_Way, 6 * sizeof(float));
			}
			break;
	}
}
}
void Class_Chassis_Communication__::TIM_PID_PeriodElapsedCallback()
{
		PID_Yaw.Set_Target(Target_Yaw);
        PID_Yaw.Set_Now(Actual_Yaw);
        PID_Yaw.TIM_Adjust_PeriodElapsedCallback();
       
}
//#endif
#ifdef _OLD
/**
 * @brief 输出到电机
 *
 */

void Class_Robotarm::Output()
{
    if (Robotarm_Control_Type == Robotarm_Control_Type_DISABLE)
    {
        //云台失能
		for(uint8_t i=0;i<5;i++)
		{
			switch(i)
			{
				case 0:
				case 1:
				{
					auto temp_Motor = reinterpret_cast<Class_AK_Motor_80_6 *>(Motor_Joint[i]);
//					temp_Motor->Set_AK_Motor_Control_Method(CAN_PACKET_DIS_RUN_CONTROL);
				}
				break;
				
				case 2:
					{
						auto temp_Motor = reinterpret_cast<Class_DM_Motor_J4310 *>(Motor_Joint[i]);
						temp_Motor->Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
//						temp_Motor->PID_Angle.Set_Integral_Error(0.0f);
//						temp_Motor->PID_Omega.Set_Integral_Error(0.0f);
//						temp_Motor->Set_Target_Torque(0.0f);
					}
				break;
				case 3:
				case 4:
					{
						auto temp_Motor = reinterpret_cast<Class_DJI_Motor_C610 *>(Motor_Joint[i]);
						temp_Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
						temp_Motor->PID_Angle.Set_Integral_Error(0.0f);
						temp_Motor->PID_Omega.Set_Integral_Error(0.0f);
						temp_Motor->Set_Target_Torque(0.0f);
					}
				break;
			}
		}
    }
    else if (Robotarm_Control_Type == Robotarm_Control_Type_NORMAL)
    {
		if(Robotarm_Resolution.Get_Now_Status_Serial() != Robotarm_Task_Status_Calibration)
        {
			//云台工作
			Motor_Joint1.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RUN_CONTROL);	
			Motor_Joint2.Set_AK_Motor_Control_Method(CAN_PACKET_SET_RUN_CONTROL);	
			Motor_Joint3.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
			Motor_Joint4.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Motor_Joint5.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			
			
		}
		Set_Motor_Angle(Motor_Joint1,1);
		Set_Motor_Angle(Motor_Joint2,2);
		Set_Motor_Angle(Motor_Joint3,3);
		Set_Motor_Angle(Motor_Joint4,4);
		Set_Motor_Angle(Motor_Joint5,5);
		
    }
}
void Class_Robotarm::Task_Alive_PeriodElapsedCallback()
{
	memcpy(&Custom_Communication_Data,&Referee.Interaction_Custom_Controller,sizeof(Struct_Custom_Communication_Data));
	static bool Suspend_flag = false;
	static bool alive_flag = false;
	if(alive_flag == false)
	{
		Motor_Joint1.Task_Alive_PeriodElapsedCallback();
		Motor_Joint2.Task_Alive_PeriodElapsedCallback();
		alive_flag = true;
	}
	else
	{
		Motor_Joint3.TIM_Alive_PeriodElapsedCallback();
		alive_flag = false;
	}
	
	 Motor_Joint4.TIM_Alive_PeriodElapsedCallback();
	 Motor_Joint5.TIM_Alive_PeriodElapsedCallback();
	
	if((Motor_Joint1.Get_AK_Motor_Status() == AK_Motor_Status_ENABLE)
	&& (Motor_Joint2.Get_AK_Motor_Status() == AK_Motor_Status_ENABLE)
	&& (Motor_Joint3.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE)
//	&& (Motor_Joint4.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
//	&& (Motor_Joint5.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
	&& (Suspend_flag == true))
	{
		Robotarm_Control_Type = Robotarm_Control_Type_NORMAL;
		//osThreadResume(RobotarmTaskHandle);
		Suspend_flag = false;
	}
	else if(((Motor_Joint1.Get_AK_Motor_Status() == AK_Motor_Status_DISABLE)
	|| (Motor_Joint2.Get_AK_Motor_Status() == AK_Motor_Status_DISABLE)
	|| (Motor_Joint3.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE))
//	|| (Motor_Joint4.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
//	|| (Motor_Joint5.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE))
	&& (Suspend_flag == false))
	{
	    Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
		Robotarm_Resolution.Set_Status(Robotarm_Task_Status_Calibration);
		for(uint8_t i = 0;i < 5;i++)
		{
			Joint_Offset_Angle[i] = Joint_Limit_Angle[Joint_Limit_Flag[i]][i];
			Joint_World_Angle[i] = 0;
		}
		Motor_Joint1.Reset_Rx_Data();
		Motor_Joint2.Reset_Rx_Data();
		Motor_Joint1.Set_Target_Angle(0);
		Motor_Joint2.Set_Target_Angle(0);
		Motor_Joint3.Set_Target_Angle(0);
		Target_Position_Orientation = {209.314f, 0 ,30.0f};
		//osThreadSuspend(RobotarmTaskHandle);//挂起任务
		Suspend_flag = true;
	}
}

/**
 * @brief Task计算回调函数
 *
 */
void Class_Robotarm::Task_Calculate_PeriodElapsedCallback()
{
    Output();
	
	Motor_Joint1.Task_Process_PeriodElapsedCallback();
	Motor_Joint2.Task_Process_PeriodElapsedCallback();
	Motor_Joint3.TIM_Process_PeriodElapsedCallback();
	Motor_Joint4.TIM_PID_PeriodElapsedCallback();
	Motor_Joint5.TIM_PID_PeriodElapsedCallback();
}

/**
 * @brief 遥控器控制任务
 *
 */
void Class_Robotarm::Task_Control_Robotarm()
{
	//角度目标值
//    float tmp_robotarm_x, tmp_robotarm_y,robotarm_yaw;
	//遥控器摇杆值
	memcpy(&Custom_Communication_Data,&Referee.Interaction_Custom_Controller,sizeof(Struct_Custom_Communication_Data));
	static Position_Orientation_t Last_Position_Orientation = Target_Position_Orientation;
	static Chassis_Move_t Last_Chassis_Move = Chassis_Move;
	
	if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
	{
		float dr16_left_x, dr16_left_y,dr16_right_x,dr16_right_y,dr16_yaw;
		// 排除遥控器死区
		dr16_left_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
		dr16_left_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
		dr16_right_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
		dr16_right_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;
		dr16_yaw = (Math_Abs(DR16.Get_Yaw()) > DR16_Dead_Zone) ? DR16.Get_Yaw() : 0;
		switch(DR16.Get_Left_Switch())
		{
			case DR16_Switch_Status_UP:
				//底盘移动
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						
					break;
					case DR16_Switch_Status_MIDDLE:
						Target_Position_Orientation.Z_Position += dr16_right_y * Robotarm_Z_Resolution;
						Chassis_Move.Chassis_Vx = dr16_left_x * Chassis_X_Resolution;
						Chassis_Move.Chassis_Vy = dr16_left_y * Chassis_Y_Resolution;
						Chassis_Move.Chassis_Wz = dr16_right_x * Chassis_Z_Resolution;
					break;
					case DR16_Switch_Status_DOWN:
						
					break;
				}
				if((Last_Position_Orientation != Target_Position_Orientation)||(Last_Chassis_Move != Chassis_Move))
				{
					//osSemaphoreRelease(Communication_SemHandle);
					Last_Position_Orientation = Target_Position_Orientation;
					Last_Chassis_Move = Chassis_Move;
				}
				
			break;
			case DR16_Switch_Status_MIDDLE:
				//机械臂移动
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);
						
					break;
					case DR16_Switch_Status_MIDDLE:
						Target_Position_Orientation.X_Position += dr16_left_y * Robotarm_X_Resolution;
						Target_Position_Orientation.Y_Position -= dr16_left_x * Robotarm_Y_Resolution;
						
						Target_Position_Orientation.Pitch_Angle -= dr16_right_y * Robotarm_Pitch_Resolution;
						Target_Position_Orientation.Roll_Angle += dr16_right_x * Robotarm_Roll_Resolution;
						Target_Position_Orientation.Yaw_Angle += dr16_yaw * Robotarm_Yaw_Resolution;
					break;
					case DR16_Switch_Status_DOWN:
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
					break;
				}
				
			break;
			case DR16_Switch_Status_DOWN:
				//整车无力
				switch(DR16.Get_Right_Switch())
				{
					case DR16_Switch_Status_UP:
						
					break;
					case DR16_Switch_Status_MIDDLE:
//						Target_Position_Orientation = {209.314f, 0 ,30.0f};
					break;
					case DR16_Switch_Status_DOWN:
						Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
					break;
				}
				
			break;
		}
	}
	if(Referee.Get_Referee_Status() == Referee_Status_ENABLE)
	{
		float Controller_x, Controller_y,Controller_pitch,Controller_roll,Controller_yaw;
		
		Controller_x = Math_Int_To_Float(Custom_Communication_Data.Flow_x, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_y = Math_Int_To_Float(Custom_Communication_Data.Flow_y, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_pitch = Math_Int_To_Float(Custom_Communication_Data.pitch, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_roll = Math_Int_To_Float(Custom_Communication_Data.roll, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		Controller_yaw = Math_Int_To_Float(Custom_Communication_Data.yaw, 0, (1 << 16) - 1 , -1.0f , 1.0f );
		
		Controller_x = (Math_Abs(Controller_x) > Controller_Dead_Zone) ? Controller_x : 0;
		Controller_y = (Math_Abs(Controller_y) > Controller_Dead_Zone) ? Controller_y : 0;
		Controller_pitch = (Math_Abs(Controller_pitch) > Controller_Dead_Zone) ? Controller_pitch : 0;
		Controller_roll = (Math_Abs(Controller_roll) > Controller_Dead_Zone) ? Controller_roll : 0;
		Controller_yaw = (Math_Abs(Controller_yaw) > Controller_Dead_Zone) ? Controller_yaw : 0;
		
		Target_Position_Orientation.X_Position -= (float)Controller_x * Robotarm_X_Resolution;
		Target_Position_Orientation.Y_Position -= (float)Controller_y * Robotarm_Y_Resolution ;
		
		Target_Position_Orientation.Pitch_Angle -= Controller_pitch * Robotarm_Pitch_Resolution;
		Target_Position_Orientation.Roll_Angle -= Controller_roll * Robotarm_Roll_Resolution;
		Target_Position_Orientation.Yaw_Angle += Controller_yaw * Robotarm_Yaw_Resolution;
	}
//	else
//	{
//		Robotarm_Control_Type = Robotarm_Control_Type_DISABLE;
//	}

}

/**
 * @brief 底盘通信任务，当遥控器值更新时才发送
 *

 */
void Class_Robotarm::Task_Chassis_Communication_PeriodElapsedCallback()
{
	//同步信号量等待
	//osSemaphoreWait(Communication_SemHandle,osWaitForever);
	//底盘速度内容填充
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Vx, -1.0f, 1.0f , Chassis_Communication_ID_0x11,0);
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Vy, -1.0f, 1.0f , Chassis_Communication_ID_0x11,2);
	Chassis_Communication.Communication_Data(Chassis_Move.Chassis_Wz, -1.0f, 1.0f , Chassis_Communication_ID_0x11,4);
	//机械臂抬升高度
	Math_Constrain(Target_Position_Orientation.Z_Position,0.0f,1160.0f);
	Chassis_Communication.Communication_Data(Target_Position_Orientation.Z_Position, 0, 1160.0f , Chassis_Communication_ID_0x11,6);
	
	
	//发送函数
	Chassis_Communication.Task_Process_PeriodElapsedCallback();
}
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

 /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
