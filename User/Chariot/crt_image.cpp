#include "crt_image.h"

float target_roll = -160.0f;
void Class_FSM_Image_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0)://校准预备状态
    {   
        if(Image->Image_Control_Type == Image_Control_Type_ENABLE)
            Image->Set_Image_Control_Type(Image_Control_Type_CALIBRATE);
        if(Image->Motor_Image_Roll.Get_Now_Torque()>Image->Image_Roll_Calibrate_Stiffness)
        {
            Set_Status(1);
        }
    }
    break;
    case (1)://校准状态
    {
        Image->Set_Image_Control_Type(Image_Control_Type_CALIBRATE);
        if(Image->Motor_Image_Roll.Get_Now_Torque()>Image->Image_Roll_Calibrate_Stiffness &&
        Status[Now_Status_Serial].Time>1500)//待定
        {
            Image->Set_Image_Roll_Calibrate_Offset(Image->Motor_Image_Roll.Get_Now_Angle());
            Set_Status(2);
        }
    }
    break;
    case (2)://正常状态
    {
        Image->Set_Image_Control_Type(Image_Control_Type_NORMAL);
        Image->Set_Target_Image_Roll_Angle(target_roll+Image->get_Image_Roll_Calibrate_Offset());
        if(Image->Motor_Image_Roll.Get_DJI_Motor_Status()==DJI_Motor_Status_DISABLE)
        {
            Set_Status(3);
        }
    }
    break;
    case (3)://电机预失能状态
    {
        Image->Set_Image_Control_Type(Image_Control_Type_NORMAL);
        if(Image->Motor_Image_Roll.Get_DJI_Motor_Status()==DJI_Motor_Status_DISABLE&&
        Status[Now_Status_Serial].Time>1000)
        {
            Set_Status(0);
        }
        else
        {
            Set_Status(2);
        }
    }
    break;
    }
}

void Class_Image::Init()
{
    FSM_Image_Control.Init(4,0);
    FSM_Image_Control.Image = this;
     //图传roll轴电机
    Motor_Image_Roll.PID_Angle.Init(50.0f, 1.5f, 2.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Roll.PID_Omega.Init(150.0f, 0.5f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Roll.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
    //图传pitch轴电机
    Motor_Image_Pitch.PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Pitch.PID_Omega.Init(5.0f, 0.5f, 0.0f, 0.0f, 0.0f, 850.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Pitch.Init(&hcan1, LK_Motor_ID_0x141,0.0f,0,33.0f,LK_Motor_Control_Method_ANGLE);
}

void Class_Image::Output()
{
    switch (Image_Control_Type)
    {
    case Image_Control_Type_DISABLE:
    {
         //图传电机失能
        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Image_Pitch.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        Motor_Image_Roll.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Image_Roll.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Image_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Image_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Image_Pitch.Set_Target_Torque(0.0f);
        Motor_Image_Roll.Set_Target_Torque(0.0f);
    }
    break;
    case Image_Control_Type_CALIBRATE:
    {
        Motor_Image_Pitch.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
        Motor_Image_Pitch.Set_Target_Angle(Target_Image_Pitch_Angle);

        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Image_Roll.Set_Target_Omega_Radian(Image_Roll_Calibrate_Speed);//待定 
    }
    break;
    case Image_Control_Type_NORMAL:
    {
        Motor_Image_Pitch.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
        Motor_Image_Pitch.Set_Target_Angle(Target_Image_Pitch_Angle);
        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Image_Roll.Set_Target_Radian(Target_Image_Roll_Angle/180.0f*PI);
    }
    break;
    }
}

void Class_Image::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    FSM_Image_Control.Reload_TIM_Status_PeriodElapsedCallback();

    Motor_Image_Pitch.TIM_Process_PeriodElapsedCallback();
	Motor_Image_Roll.TIM_PID_PeriodElapsedCallback();
}