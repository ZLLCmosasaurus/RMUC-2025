#include "crt_image.h"

float target_roll = 0.0f;
float target_pitch = 0.0f;
void Class_FSM_Image_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0): // 校准开始状态
    {
        static uint8_t Cali_Flag[2] = {0, 0};

        if (Image->Motor_Image_Pitch.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
        {
            Cali_Flag[0] = Image->Motor_Calibration(&Image->Motor_Image_Pitch,
                                                    &Image->Image_Pitch_Calibrate_Offset,
                                                    Image->Image_Pitch_Calibrate_Speed,
                                                    Image->Image_Pitch_Calibrate_Stiffness);
        }
        if (Image->Motor_Image_Roll.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)
        {

            Cali_Flag[1] = Image->Motor_Calibration(&Image->Motor_Image_Roll,
                                                    &Image->Image_Roll_Calibrate_Offset,
                                                    Image->Image_Roll_Calibrate_Speed,
                                                    Image->Image_Roll_Calibrate_Stiffness);
        }

        if (Cali_Flag[0] && Cali_Flag[1])
            Set_Status(1);
    }
    break;
    case (1)://正常状态
    {
        Image->Set_Image_Control_Type(Image_Control_Type_NORMAL);
        Image->Output();
        if (Image->Motor_Image_Roll.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE ||
            Image->Motor_Image_Pitch.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
        {
            Set_Status(0);//电机失能后从新校准
        }
        
    }
    break;
    }
}

bool  Class_Image::Motor_Calibration(Class_DJI_Motor_C610 *motor,float *Cali_Offset,float Cali_Omega,float Cali_Max_Out)
{
    //记录电机堵转时间
	static uint16_t count = 0;
	//设置为速度环校准
	motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	motor->Set_Target_Omega_Angle(Cali_Omega);
	//当电流值大于阈值，同时速度小于一定阈值，判定为堵转条件
	if( (fabs(motor->Get_Now_Torque()) >= Cali_Max_Out) && (fabs(motor->Get_Now_Omega_Radian()) < 0.01f*PI) )
	{
		count++;
		//当到达一定时间，判定为堵转
		if(count >= 200)
		{
			count = 0;
			//改为角度环，设置关节角度
			motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			//分别记录2006的零位角度
			*Cali_Offset = motor->Get_Now_Angle();
            //设置电机的目标角度
            motor->Set_Target_Radian(*Cali_Offset);
			return true;
		}	
	}
	else
	{
		count=0;
	}
	return false;
}

void Class_Image::Init()
{
    FSM_Image_Control.Init(2,0);
    FSM_Image_Control.Image = this;
     //图传roll轴电机
    Motor_Image_Roll.PID_Angle.Init(50.0f, 1.5f, 2.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Roll.PID_Omega.Init(150.0f, 0.5f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Roll.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
    //图传pitch轴电机
    Motor_Image_Pitch.PID_Angle.Init(50.0f, 1.5f, 2.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Pitch.PID_Omega.Init(150.0f, 0.5f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Pitch.Init(&hcan1, DJI_Motor_ID_0x206,DJI_Motor_Control_Method_ANGLE);
}

void Class_Image::Output()
{
    switch (Image_Control_Type)
    {
    case Image_Control_Type_DISABLE:
    {
         //图传电机失能
        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Image_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
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
        Motor_Calibration(&Motor_Image_Roll,&Image_Roll_Calibrate_Offset,Image_Roll_Calibrate_Speed,Image_Roll_Calibrate_Stiffness);
        Motor_Calibration(&Motor_Image_Pitch,&Image_Pitch_Calibrate_Offset,Image_Pitch_Calibrate_Speed,Image_Pitch_Calibrate_Stiffness);
    }
    break;
    case Image_Control_Type_NORMAL:
    {
        Motor_Image_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Image_Pitch.Set_Target_Radian(Target_Image_Pitch_Angle/180.0f*PI);
        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Image_Roll.Set_Target_Radian(Target_Image_Roll_Angle/180.0f*PI);
    }
    break;
    }
}

void Class_Image::TIM_Calculate_PeriodElapsedCallback()
{
    FSM_Image_Control.Reload_TIM_Status_PeriodElapsedCallback();

    Motor_Image_Pitch.TIM_PID_PeriodElapsedCallback();
	Motor_Image_Roll.TIM_PID_PeriodElapsedCallback();
}