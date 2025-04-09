#include "ita_chariot.h"

void Class_Chariot::Init()
{
    chassis.Init(1.0f,3.0f,2.0f);
    Auxiliary_Arm_Uplift_X_Lift.Init(&hcan1, DJI_Motor_ID_0x207, DJI_Motor_Control_Method_ANGLE);
    Auxiliary_Arm_Uplift_X_Lift.PID_Angle.Init(100.0f, 0.05f, 0.0f, 0.0f, 180.0f, 250.0f);
    Auxiliary_Arm_Uplift_X_Lift.PID_Omega.Init(40.0f, 0.1f, 0.0f, 0.0f, 2000.0f, 7000.0f);
    Auxiliary_Arm_Uplift_X_Right.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
    Auxiliary_Arm_Uplift_X_Right.PID_Angle.Init(100.0f, 0.05f, 0.0f, 0.0f, 180.0f, 250.0f);
    Auxiliary_Arm_Uplift_X_Right.PID_Omega.Init(40.0f, 0.1f, 0.0f, 0.0f, 2000.0f, 7000.0f);
    Auxiliary_Arm_Uplift_Y_Lift.Init(&hcan1, DJI_Motor_ID_0x208, DJI_Motor_Control_Method_ANGLE);
    Auxiliary_Arm_Uplift_Y_Lift.PID_Angle.Init(1900.0f, 0.f, 0.0f, 0.0f, 180.0f, 200.0f);
    Auxiliary_Arm_Uplift_Y_Lift.PID_Omega.Init(30.0f, 0.f, 0.0f, 0.0f, 3000.0f, 7000.0f);
    Auxiliary_Arm_Uplift_Y_Right.Init(&hcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE);
    Auxiliary_Arm_Uplift_Y_Right.PID_Angle.Init(1900.0f, 0.f, 0.0f, 0.0f, 180.0f, 200.0f);
    Auxiliary_Arm_Uplift_Y_Right.PID_Omega.Init(30.0f, 0.f, 0.0f, 0.0f, 3000.0f, 7000.0f);
    
}
uint8_t Calibration_Flag__test;
bool Class_Chariot::Motor_Calibration()
{
	//记录电机堵转时间
	static uint16_t count_1;
    static uint16_t count_2;
    static uint16_t count_3;
    static uint16_t count_4;
    static uint8_t Calibration_Flag;

	//设置为速度环校准
	if(((Calibration_Flag & (1<<1)) == 0)){	Auxiliary_Arm_Uplift_X_Lift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Auxiliary_Arm_Uplift_X_Lift.Set_Target_Omega_Angle(-80);}
	if(((Calibration_Flag & (1<<2)) == 0)){Auxiliary_Arm_Uplift_X_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Auxiliary_Arm_Uplift_X_Right.Set_Target_Omega_Angle(80);}
	if(((Calibration_Flag & (1<<3)) == 0)){ Auxiliary_Arm_Uplift_Y_Lift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Auxiliary_Arm_Uplift_Y_Lift.Set_Target_Omega_Angle(-50);}
	if(((Calibration_Flag & (1<<4)) == 0)){Auxiliary_Arm_Uplift_Y_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	Auxiliary_Arm_Uplift_Y_Right.Set_Target_Omega_Angle(50);}

	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	
	if( (fabs(Auxiliary_Arm_Uplift_X_Lift.Get_Now_Torque()) >= 1700) && (fabs(Auxiliary_Arm_Uplift_X_Lift.Get_Now_Omega_Radian()) < 0.01f*PI)&&((Calibration_Flag & (1<<1)) == 0))
	{
		count_1++;
		//当到达一定时间，判定为堵转
		if(count_1 >= 200)
		{
			count_1 = 0;
			//改为角度环，设置关节角度
			Auxiliary_Arm_Uplift_X_Lift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

			Arm_Uplift_Offset_Angle[0] = Auxiliary_Arm_Uplift_X_Lift.Get_Now_Angle();
			Auxiliary_Arm_Uplift_X_Lift.Set_Target_Angle(Auxiliary_Arm_Uplift_X_Lift.Get_Now_Angle());
			
            Calibration_Flag |= (1<<1);
		}	
	}
//	else
//	{
//		count_1=0;
//	}
    	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Auxiliary_Arm_Uplift_X_Right.Get_Now_Torque()) >= 1700) && (fabs(Auxiliary_Arm_Uplift_X_Right.Get_Now_Omega_Radian()) < 0.01f*PI)&&((Calibration_Flag & (1<<2)) == 0) )
	{
		count_2++;
		//当到达一定时间，判定为堵转
		if(count_2 >= 200)
		{
			count_2 = 0;
			//改为角度环，设置关节角度
			Auxiliary_Arm_Uplift_X_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

			Arm_Uplift_Offset_Angle[1] = Auxiliary_Arm_Uplift_X_Right.Get_Now_Angle();
			Auxiliary_Arm_Uplift_X_Right.Set_Target_Angle(Auxiliary_Arm_Uplift_X_Right.Get_Now_Angle());
		
            Calibration_Flag |= (1<<2);
		}	
	}
//	else
//	{
//		count_2=0;
//	}
    	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Auxiliary_Arm_Uplift_Y_Lift.Get_Now_Torque()) >= 1300) && (fabs(Auxiliary_Arm_Uplift_Y_Lift.Get_Now_Omega_Radian()) < 0.01f*PI)&&((Calibration_Flag & (1<<3)) == 0) )
	{
		count_3++;
		//当到达一定时间，判定为堵转
		if(count_3 >= 200)
		{
			count_3 = 0;
			//改为角度环，设置关节角度
			Auxiliary_Arm_Uplift_Y_Lift.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
	
			Arm_Uplift_Offset_Angle[2] = Auxiliary_Arm_Uplift_Y_Lift.Get_Now_Angle();
			Auxiliary_Arm_Uplift_Y_Lift.Set_Target_Angle(Auxiliary_Arm_Uplift_Y_Lift.Get_Now_Angle());
						Auxiliary_Arm_Uplift_Y_Lift.Target_Up_Length=3;
            Calibration_Flag |= (1<<3);
		}	
	}
//		else
//	{
//		count_3=0;
//	}
    	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(Auxiliary_Arm_Uplift_Y_Right.Get_Now_Torque()) >= 1300) && (fabs(Auxiliary_Arm_Uplift_Y_Right.Get_Now_Omega_Radian()) < 0.01f*PI) &&((Calibration_Flag & (1<<4)) == 0))
	{
		count_4++;
		//当到达一定时间，判定为堵转
		if(count_4 >= 200)
		{
			count_4 = 0;
			//改为角度环，设置关节角度
			Auxiliary_Arm_Uplift_Y_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			Arm_Uplift_Offset_Angle[3] = Auxiliary_Arm_Uplift_Y_Right.Get_Now_Angle();
			Auxiliary_Arm_Uplift_Y_Right.Set_Target_Angle(Auxiliary_Arm_Uplift_Y_Right.Get_Now_Angle());
					Auxiliary_Arm_Uplift_Y_Right.Target_Up_Length=-3;
        Calibration_Flag |= (1<<4);
		}	
	}
//		else
//	{
//		count_4=0;
//	}
	Calibration_Flag__test=Calibration_Flag;
	if(Calibration_Flag == 30)
	{
		Calibration_Flag = 0;
		return true;
	}
	return false;
}


void Class_Chariot::Control_Chassis()
{
}

void Class_Chariot::TIM_Control_Callback()
{
}
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{

        for (auto i = 0; i < 4; i++)
        {
            chassis.Motor_Wheel[i].TIM_Alive_PeriodElapsedCallback();
        }
				Auxiliary_Arm_Uplift_X_Lift.TIM_Alive_PeriodElapsedCallback();
				Auxiliary_Arm_Uplift_Y_Lift.TIM_Alive_PeriodElapsedCallback();
				if((Auxiliary_Arm_Uplift_X_Lift.Get_DJI_Motor_Status()==DJI_Motor_Status_DISABLE)&&(Auxiliary_Arm_Uplift_Y_Lift.Get_DJI_Motor_Status()==DJI_Motor_Status_DISABLE))
				{
					
					init_finish_flag=false;
				}
				
				
        TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
        if(Gimbal_Status == Gimbal_Status_DISABLE)
        {
            chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
}
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    Gimbal_Alive_Flag++;
	
		static uint8_t mod50;
    //云台坐标系的目标速度
	float gimbal_velocity_x, gimbal_velocity_y;
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y, tmp_omega;
    memcpy(&tmp_velocity_x,&CAN_Manage_Object->Rx_Buffer.Data[0],sizeof(uint16_t));
    memcpy(&tmp_velocity_y,&CAN_Manage_Object->Rx_Buffer.Data[2],sizeof(uint16_t));
    memcpy(&tmp_omega,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(uint16_t));
		memcpy(&chassis_control_type,&CAN_Manage_Object->Rx_Buffer.Data[6],1);
		memcpy(&New_status,&CAN_Manage_Object->Rx_Buffer.Data[7],1);
    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x,0,0xFFFF,-1 * chassis.Get_Velocity_X_Max(),chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y,0,0xFFFF,-1 * chassis.Get_Velocity_Y_Max(),chassis.Get_Velocity_Y_Max());
    chassis_omega = Math_Int_To_Float(tmp_omega,0,0xFFFF,-1 * chassis.Get_Omega_Max(),chassis.Get_Omega_Max());
		
    chassis_velocity_x = gimbal_velocity_x;
    chassis_velocity_y = gimbal_velocity_y;
		chassis.Set_Chassis_Control_Type(chassis_control_type);
		
		if(New_status!=Now_status)
		{
		mod50++;
		if(mod50>=50)
		{
		Now_status=New_status;
		mod50=0;}
		
		}else
		{
		mod50=0;
		}
		
		
		
    if(chassis_control_type==Chassis_Control_Type_FLLOW){
    //设定底盘目标速度
    chassis.Set_Target_Velocity_X(-chassis_velocity_x);
    chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    chassis.Set_Target_Omega(chassis_omega);
		}else 
		{
		  //设定底盘目标速度
    chassis.Set_Target_Velocity_X(0);
    chassis.Set_Target_Velocity_Y(0);
    chassis.Set_Target_Omega(0);
		
		}
		
		
		
}
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
void Class_Chariot::TIM_Chariot_PeriodElapsedCallback()
{
    chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_DISABLE);
		Auxiliary_Arm_Uplift_X_Lift.Calculate_Actual_Up_Length(Arm_Uplift_Offset_Angle[0]);
		Auxiliary_Arm_Uplift_Y_Lift.Calculate_Actual_Up_Length(Arm_Uplift_Offset_Angle[2]);
		Auxiliary_Arm_Uplift_X_Right.Calculate_Actual_Up_Length(Arm_Uplift_Offset_Angle[1]);
		Auxiliary_Arm_Uplift_Y_Right.Calculate_Actual_Up_Length(Arm_Uplift_Offset_Angle[3]);
	
    Auxiliary_Arm_Uplift_X_Lift.TIM_PID_PeriodElapsedCallback();
    Auxiliary_Arm_Uplift_Y_Lift.TIM_PID_PeriodElapsedCallback();
    Auxiliary_Arm_Uplift_X_Right.TIM_PID_PeriodElapsedCallback();
    Auxiliary_Arm_Uplift_Y_Right.TIM_PID_PeriodElapsedCallback();

}

void Class_Auxiliary_Arm_Uplift_Y::Calculate_Actual_Up_Length(float Off_Set_Angle)
{
	Actual_Up_Length = (Get_Now_Angle()-Off_Set_Angle)*K_Actual;
}
void Class_Auxiliary_Arm_Uplift_X::Calculate_Actual_Up_Length(float Off_Set_Angle)
{
	Actual_Up_Length = (Get_Now_Angle()-Off_Set_Angle)*K_Actual;
}
void Class_Auxiliary_Arm_Uplift_Y::TIM_PID_PeriodElapsedCallback()
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
void Class_Auxiliary_Arm_Uplift_X::TIM_PID_PeriodElapsedCallback()
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