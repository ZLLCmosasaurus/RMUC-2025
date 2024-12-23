#include "ita_chariot.h"

void Class_Chariot::Init()
{
    chassis.Init(0.5f,3.0f,2.0f);
}


void Class_Chariot::Control_Chassis()
{
}

void Class_Chariot::TIM_Control_Callback()
{
}
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    Gimbal_Alive_Flag++;
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
    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x,0,0xFFFF,-1 * chassis.Get_Velocity_X_Max(),chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y,0,0xFFFF,-1 * chassis.Get_Velocity_Y_Max(),chassis.Get_Velocity_Y_Max());
    chassis_omega = Math_Int_To_Float(tmp_omega,0,0xFFFF,-1 * chassis.Get_Omega_Max(),chassis.Get_Omega_Max());
		
    chassis_velocity_x = gimbal_velocity_x;
    chassis_velocity_y = gimbal_velocity_y;
		chassis.Set_Chassis_Control_Type(chassis_control_type);
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
}

