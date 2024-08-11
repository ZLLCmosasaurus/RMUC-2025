#include "crt_information_platform.h""

void Class_Information_Platform::Init(Struct_CAN_Manage_Object* __CAN_Manage_Object)
{
    CAN_Manage_Object=__CAN_Manage_Object;
    Set_Velocity_X_Max(4.0f);
    Set_Velocity_Y_Max(4.0f);
    #ifdef GIMBAL
    Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    #endif
}



void Class_Information_Platform::Platform_Tx_Callback()
{
    
    #ifdef GIMBAL
        memcpy(CAN2_Chassis_Tx_Gimbal_Data,gimbal_platform.gimbal_tx_part.gimbal_data,GIMBAL_DATA_LENGTH);
    #endif

    #ifdef CHASSIS
        memcpy(CAN2_Chassis_Tx_Gimbal_Data,chassis_platform.chassis_tx_part.chassis_data,CHASSIS_DATA_LENGTH);
    #endif
}

void Class_Information_Platform::Platform_Rx_Callback()
{
    #ifdef GIMBAL
    Chassis_Alive_Flag++;
    memcpy(gimbal_platform.chassis_rx_part.chassis_data,CAN_Manage_Object->Rx_Buffer.Data,CHASSIS_DATA_LENGTH);
    #endif
    #ifdef CHASSIS
        Gimbal_Alive_Flag++;
        float chassis_velocity_x, chassis_velocity_y;
        //目标角速度
        float chassis_omega;
        //底盘和云台夹角（弧度制）
        float derta_angle;
        //float映射到int16之后的速度
        uint16_t tmp_velocity_x, tmp_velocity_y, tmp_omega, tmp_gimbal_pitch;

        memcpy(chassis_platform.gimbal_rx_part.gimbal_data,CAN_Manage_Object->Rx_Buffer.Data,GIMBAL_DATA_LENGTH);
    
        chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x,0,0x7FFF,-1 * Get_Velocity_X_Max(),Get_Velocity_X_Max());
        chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y,0,0x7FFF,-1 * Get_Velocity_Y_Max(),Get_Velocity_Y_Max());
        Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch,0,0x7FFF,-10.0f,30.0f);

    

    //设定底盘目标速度
    Set_Target_Velocity_X(chassis_velocity_x);
    Set_Target_Velocity_Y(chassis_velocity_y);
    Set_Target_Omega(chassis_omega);

    #endif
}

void Class_Information_Platform::Platform_Alive_PeriodElapsedCallback()
{
    #ifdef GIMBAL
    if (Chassis_Alive_Flag == Pre_Chassis_Alive_Flag)
    {
        Chassis_Status = Chassis_Status_DISABLE;
    }
    else
    {
        Chassis_Status = Chassis_Status_ENABLE;
    }
    Pre_Chassis_Alive_Flag = Chassis_Alive_Flag;   
    #endif

    #ifdef CHASSIS
     if (Gimbal_Alive_Flag == Pre_Gimbal_Alive_Flag)
    {
        Gimbal_Status = Gimbal_Status_DISABLE;
    }
    else
    {
        Gimbal_Status = Gimbal_Status_ENABLE;
    }
    Pre_Gimbal_Alive_Flag = Gimbal_Alive_Flag;  

    #endif
}