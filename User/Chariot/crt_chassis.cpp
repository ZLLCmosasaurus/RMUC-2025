/**
 * @file crt_chassis.cpp
 * @author lez by wanghongxi
 * @brief 底盘
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */

void Class_Tricycle_Chassis:: mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}


void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;
    #ifdef SPEED_SLOPE
    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.004f,0.008f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.004f,0.008f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);
    #endif
    #ifdef SUPERCAP
    //超级电容初始化
    Supercap.Init(&hcan2,45);
    #endif
    #ifdef C_IMU
    Boardc_BMI.Init();
    Yaw.IMU=&Boardc_BMI;
    Pitch.IMU=&Boardc_BMI;
    Roll.IMU=&Boardc_BMI;
    #endif
    //电机PID批量初始化

    Joint_Motor[0].motor.Init(&hcan1, DM_Motor_ID_0xA1,DM_Motor_Control_Method_MIT_POSITION);
    Joint_Motor[1].motor.Init(&hcan1, DM_Motor_ID_0xA2,DM_Motor_Control_Method_MIT_POSITION);
    Joint_Motor[2].motor.Init(&hcan1, DM_Motor_ID_0xA3,DM_Motor_Control_Method_MIT_POSITION);
    Joint_Motor[3].motor.Init(&hcan1, DM_Motor_ID_0xA4,DM_Motor_Control_Method_MIT_POSITION);

    Wheel_Motor[0].motor.Init(&hcan1,AK_Motor_ID_0x02);
    Wheel_Motor[1].motor.Init(&hcan1,AK_Motor_ID_0x01);

    Left_Leg.VMC_Init(LEFT);
    Right_Leg.VMC_Init(RIGHT);

    left_leg_length_pid.Init(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f);
    right_leg_length_pid.Init(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f);
    roll_pid.Init(1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f );
    Tp_pid.Init( 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    turn_pid.Init( 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    // for (int i = 0; i < 4; i++)
    // {
    //     Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    // }

    // //轮向电机ID初始化
    // Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    // Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    // Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    // Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);
}

void Class_Tricycle_Chassis::Pose_Calculate(void){

	
	if(Pitch.Get_True_Rad_Pitch()<(3.1415926f/6.0f)&&Pitch.Get_True_Rad_Pitch()>(-3.1415926f/6.0f))
	{//根据pitch角度判断倒地自起是否完成
	recover_flag=0;
	}

    Left_Leg.phi1=PI/2.0f-Joint_Motor[0].motor.Get_Now_Angle();
    Left_Leg.phi4=PI/2.0f-Joint_Motor[1].motor.Get_Now_Angle();

    Right_Leg.phi1=PI/2.0f+Joint_Motor[2].motor.Get_Now_Angle();
    Right_Leg.phi4=PI/2.0f+Joint_Motor[3].motor.Get_Now_Angle();
    
    mypitch[0]=Pitch.Get_True_Rad_Pitch();
    mypitchgyro[0]=Pitch.Get_True_Gyro_Pitch();
    mypitch[1]=0.f-Pitch.Get_True_Rad_Pitch();
    mypitchgyro[1]=0.0f-Pitch.Get_True_Gyro_Pitch();

    total_yaw=Yaw.Get_True_Angle_Total_Yaw();
    roll=Roll.Get_True_Rad_Roll();
    leg_theta_err=0.0f-(Left_Leg.theta+Right_Leg.theta);
    if(Pitch.Get_True_Rad_Pitch()<(PI/6.0f)&&Pitch.Get_True_Rad_Pitch()>(-PI/6.0f))
    recover_flag=0;
}
/**
 * @brief 速度解算
 *
 */
float temp_test_1,temp_test_2,temp_test_3,temp_test_4;
void Class_Tricycle_Chassis::Speed_Resolution(){

    CHASSIS_DWT_Dt = DWT_GetDeltaT(&CHASSIS_DWT_Count);

    Left_Leg.VMC_calc_1(CHASSIS_DWT_Dt);
    Right_Leg.VMC_calc_1(CHASSIS_DWT_Dt);
    for(int i=0;i<12;i++){

        Left_Leg.LQR_K[i]=Left_Leg.LQR_K_calc(&Poly_Coefficient[i][0],Left_Leg.L0);
        
        Right_Leg.LQR_K[i]=Right_Leg.LQR_K_calc(&Poly_Coefficient[i][0],Right_Leg.L0);

    }
    turn_pid.Set_Now(Yaw.Get_True_Gyro_Yaw());
    turn_pid.Set_Target(yaw_set-total_yaw);
    turn_pid.TIM_Adjust_PeriodElapsedCallback();
    turn_T=turn_pid.Get_Out();

    roll_pid.Set_Now(Roll.Get_True_Gyro_Roll());
    roll_pid.Set_Target(roll_set-roll);
    roll_pid.TIM_Adjust_PeriodElapsedCallback();
    roll_f0=roll_pid.Get_Out();

    Tp_pid.Set_Now(leg_theta_err);
    Tp_pid.Set_Target(0.0f);
    Tp_pid.TIM_Adjust_PeriodElapsedCallback();
    leg_tp=Tp_pid.Get_Out();

    Wheel_Motor[0].wheel_T=(Left_Leg.LQR_K[0]*(Left_Leg.theta-0.0f)
																	+Left_Leg.LQR_K[1]*(Left_Leg.d_theta-0.0f)
																	+Left_Leg.LQR_K[2]*(x_set-x_filter)
																	+Left_Leg.LQR_K[3]*(0.4f*v_set-v_filter)
																	+Left_Leg.LQR_K[4]*(mypitch[0]+0.04f)
																	+Left_Leg.LQR_K[5]*(mypitchgyro[0]-0.0f));
	
    Wheel_Motor[1].wheel_T=(Right_Leg.LQR_K[0]*(Right_Leg.theta-0.0f)
																	+Right_Leg.LQR_K[1]*(Right_Leg.d_theta-0.0f)
																	+Right_Leg.LQR_K[2]*(x_filter-x_set)
																	+Right_Leg.LQR_K[3]*(v_filter-0.0f)
																	+Right_Leg.LQR_K[4]*(mypitch[1]-0.04f-phi_set)
																	+Right_Leg.LQR_K[5]*(mypitchgyro[1]-0.0f));

    Left_Leg.Tp=(Left_Leg.LQR_K[6]*(Left_Leg.theta-0.0f+theta_set)
						+Left_Leg.LQR_K[7]*(Left_Leg.d_theta-0.0f)
						+Left_Leg.LQR_K[8]*(x_set-x_filter)
						+Left_Leg.LQR_K[9]*(0.4f*v_set-v_filter)
						+Left_Leg.LQR_K[10]*(mypitch[0]-(-0.04f))
						+Left_Leg.LQR_K[11]*(mypitchgyro[0]-0.0f));
	
		Left_Leg.Tp=Left_Leg.Tp+leg_tp;//髋关节输出力矩
	
    Right_Leg.Tp=(Right_Leg.LQR_K[6]*(Right_Leg.theta-0.0f)
					+Right_Leg.LQR_K[7]*(Right_Leg.d_theta-0.0f)
					+Right_Leg.LQR_K[8]*(x_filter-x_set)
				  +Right_Leg.LQR_K[9]*(v_filter-0.0f)
					+Right_Leg.LQR_K[10]*(mypitch[1]-0.04f-phi_set)
					+Right_Leg.LQR_K[11]*(mypitchgyro[1]-0.0f));

	Right_Leg.Tp=Right_Leg.Tp+leg_tp;//髋关节输出力矩

    Wheel_Motor[0].wheel_T=Wheel_Motor[0].wheel_T-turn_T;//轮毂电机输出力矩
    Wheel_Motor[1].wheel_T=Wheel_Motor[0].wheel_T-turn_T;//轮毂电机输出力矩
    mySaturate(&Wheel_Motor[0].wheel_T,-10.0f,10.0f);
    mySaturate(&Wheel_Motor[1].wheel_T,-10.0f,10.0f);

    if(jump_flag[1]==1||jump_flag[1]==2||jump_flag[1]==3)
	{
    if(jump_flag[1]==1)
		{//压缩阶段
        
         right_leg_length_pid.Set_Now(Right_Leg.L0);
         right_leg_length_pid.Set_Target(0.08f);
         right_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();
		 Right_Leg.F0=mg/arm_cos_f32(Right_Leg.theta)+right_leg_length_pid.Get_Out();//前馈+pd

		 if(Right_Leg.L0<0.10f)
		 {
		  jump_time[1]++;
		 }
		 if(jump_time[1]>=10&&jump_time[0]>=10)
		 {  
			 jump_time[1]=0;
			 jump_time[0]=0;
			 jump_flag[1]=2;//压缩完毕进入上升加速阶段
			 jump_flag[0]=2;//压缩完毕进入上升加速阶段
		 }			 
		}
		else if(jump_flag[1]==2)
		{//上升加速阶段	

             right_leg_length_pid.Set_Now(Right_Leg.L0);
             right_leg_length_pid.Set_Target(0.4f);
             right_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
			 Right_Leg.F0=mg/arm_cos_f32(Right_Leg.theta)+right_leg_length_pid.Get_Out();//前馈+pd
			
			 if(Right_Leg.L0>0.18f)
			 {
				jump_time[1]++;
			 }
			 if(jump_time[1]>=2&&jump_time[0]>=2)
			 {  
				 jump_time[1]=0;
				 jump_time[0]=0;
				 jump_flag[1]=3;//上升完毕进入缩腿阶段
				 jump_flag[0]=3;
			 }	 
		}
		else if(jump_flag[1]==3)
		{//缩腿阶段
			 right_leg_length_pid.Set_Now(Right_Leg.L0);
             right_leg_length_pid.Set_Target(0.1f);
             right_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
			 Right_Leg.F0=right_leg_length_pid.Get_Out();//前馈+pd
			theta_set=0.0f;
		  if(Right_Leg.L0<0.15f)
		  {
			 jump_time[1]++;
		  }
		  if(jump_time[1]>=3&&jump_time[0]>=3)
		  { 
			 jump_time[1]=0;
			 jump_time[0]=0;
			 leg_set=0.10f;
			 last_leg_set=0.10f;
			 jump_flag[1]=0;//缩腿完毕
		   jump_flag[0]=0;			
		  }
		}
	}	
	else
	{
         right_leg_length_pid.Set_Now(Right_Leg.L0);
             right_leg_length_pid.Set_Target(leg_set);
             right_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
			 Right_Leg.F0=mg/arm_cos_f32(Right_Leg.theta)+right_leg_length_pid.Get_Out();//前馈+pd
	}

    if(jump_flag[0]==1||jump_flag[0]==2||jump_flag[0]==3)
	{
    if(jump_flag[0]==1)
		{//压缩阶段
        left_leg_length_pid.Set_Now(Right_Leg.L0);
        left_leg_length_pid.Set_Target(0.08f);
        left_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
		Left_Leg.F0=mg/arm_cos_f32(Left_Leg.theta)+left_leg_length_pid.Get_Out();//前馈+pd

		 if(Left_Leg.L0<0.10f)
		 {
		  jump_time[0]++;
		 }
		 if(jump_time[1]>=10&&jump_time[0]>=10)
		 {  
			 jump_time[1]=0;
			 jump_time[0]=0;
			 jump_flag[1]=2;//压缩完毕进入上升加速阶段
			 jump_flag[0]=2;//压缩完毕进入上升加速阶段
		 }			 
		}
		else if(jump_flag[0]==2)
		{//上升加速阶段			
			left_leg_length_pid.Set_Now(Right_Leg.L0);
            left_leg_length_pid.Set_Target(0.4f);
            left_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
		    Left_Leg.F0=mg/arm_cos_f32(Left_Leg.theta)+left_leg_length_pid.Get_Out();//前馈+pd
			
			 if(Left_Leg.L0>0.18f)
			 {
				jump_time[0]++;
			 }
			 if(jump_time[1]>=2&&jump_time[0]>=2)
			 {  
				 jump_time[1]=0;
				 jump_time[0]=0;
				 jump_flag[1]=3;//上升完毕进入缩腿阶段
				 jump_flag[0]=3;
			 }	 
		}
		else if(jump_flag[0]==3)
		{//缩腿阶段
			left_leg_length_pid.Set_Now(Right_Leg.L0);
            left_leg_length_pid.Set_Target(0.1f);
            left_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
		    Left_Leg.F0=left_leg_length_pid.Get_Out();//前馈+pd
			theta_set=0.0f;
		  if(Left_Leg.L0<0.15f)
		  {
			 jump_time[0]++;
		  }
		  if(jump_time[1]>=3&&jump_time[0]>=3)
		  { 
			 jump_time[1]=0;
			 jump_time[0]=0;
			 leg_set=0.10f;
			 last_leg_set=0.10f;
			 jump_flag[1]=0;//缩腿完毕
		   jump_flag[0]=0;			
		  }
		}
	}	
	else
	{
		left_leg_length_pid.Set_Now(Right_Leg.L0);
        left_leg_length_pid.Set_Target(leg_set);
        left_leg_length_pid.TIM_Adjust_PeriodElapsedCallback();		
		Left_Leg.F0=mg/arm_cos_f32(Left_Leg.theta)+left_leg_length_pid.Get_Out();//前馈+pd
	}

    suspend_flag[0]=Left_Leg.ground_detection(Left_Leg.leg_side);
    suspend_flag[1]=Right_Leg.ground_detection(Right_Leg.leg_side);

    if(recover_flag==0)
    {
         if((suspend_flag[0]==1&&suspend_flag[1]==1&&Left_Leg.leg_flag==0&&jump_flag[0]!=1&&jump_flag[1]!=1)||jump_flag[0]==3)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			//排除跳跃的压缩阶段和跳跃的缩腿阶段
				Wheel_Motor[0].wheel_T=0.0f;
				Left_Leg.Tp=Left_Leg.LQR_K[6]*(Left_Leg.theta-0.0f)+ Left_Leg.LQR_K[7]*(Left_Leg.d_theta-0.0f);

				x_filter=0.0f;
				x_set=x_filter;

				Left_Leg.Tp=Left_Leg.Tp+leg_tp;	
		}
		else
		{//没有离地
			Left_Leg.leg_flag=0;//置为0
	
			if(jump_flag[0]==0)
			{//不跳跃的时候需要roll轴补偿
				
				Left_Leg.F0=Left_Leg.F0-roll_f0;//roll轴补偿取反然后加上去	
				
			}
		}
		if((suspend_flag[1]==1&&suspend_flag[0]==1&&Right_Leg.leg_flag==0&&jump_flag[1]!=1&&jump_flag[0]!=1)||jump_flag[1]==3)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			//排除跳跃的压缩阶段、跳跃的缩腿阶段
				Wheel_Motor[1].wheel_T=0.0f;
				Right_Leg.Tp=Right_Leg.LQR_K[6]*(Right_Leg.theta-0.0f)+ Right_Leg.LQR_K[7]*(Right_Leg.d_theta-0.0f);

				x_filter=0.0f;
				x_set=x_filter;
				Right_Leg.Tp=Right_Leg.Tp+leg_tp;			 
		}
		else
		{//没有离地
			Right_Leg.leg_flag=0;//置为0
							
			if(jump_flag[1]==0)
			{//不跳跃的时候需要roll轴补偿						
			 Right_Leg.F0=Right_Leg.F0+roll_f0;//roll轴补偿取反然后加上去    			
			}
		}
    }
    else if(recover_flag==1)
	 {
		 Left_Leg.Tp=0.0f;
		 Left_Leg.F0=0.0f;
         Right_Leg.Tp=0.0f;
		 Right_Leg.F0=0.0f;
	 }

	 mySaturate(&Right_Leg.F0,-100.0f,100.0f);//限幅 
	 mySaturate(&Left_Leg.F0,-100.0f,100.0f);//限幅 

	 Left_Leg.VMC_calc_2();
	 Right_Leg.VMC_calc_2();

	 if(jump_flag[0]==1||jump_flag[0]==2||jump_flag[0]==3||jump_flag[1]==1||jump_flag[1]==2||jump_flag[1]==3)
	{//跳跃的时候需要更大扭矩
		mySaturate(&Left_Leg.torque_set[1],-6.0f,6.0f);	
		mySaturate(&Left_Leg.torque_set[0],-6.0f,6.0f);
		mySaturate(&Right_Leg.torque_set[1],-6.0f,6.0f);	
		mySaturate(&Right_Leg.torque_set[0],-6.0f,6.0f);		
	}	
	else
	{//不跳跃的时候最大为额定扭矩
    	mySaturate(&Left_Leg.torque_set[1],-3.0f,3.0f);	
		mySaturate(&Left_Leg.torque_set[0],-3.0f,3.0f);	
		mySaturate(&Right_Leg.torque_set[1],-3.0f,3.0f);	
		mySaturate(&Right_Leg.torque_set[0],-3.0f,3.0f);	
	}	

    for (int i = 0; i < 4; i++)
    {
                Joint_Motor[i].motor.TIM_Process_PeriodElapsedCallback();
                DWT_Delay(0.001);
    }
    for (int i = 0; i < 2; i++)
    {           
                Wheel_Motor[i].motor.Set_Target_Current(Wheel_Motor[i].wheel_T/Wheel_Motor[i].motor.Get_KT()/6.f);
                Wheel_Motor[i].motor.Task_Process_PeriodElapsedCallback();
                DWT_Delay(0.001);
    }
    //获取当前速度值，用于速度解算初始值获取
    
    // switch (Chassis_Control_Type)
    // {
    //     case (Chassis_Control_Type_DISABLE):
    //     {
    //         //底盘失能 四轮子自锁
    //         for (int i = 0; i < 4; i++)
    //         {
    //             Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    //             Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
    //             Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
    //             Motor_Wheel[i].Set_Out(0.0f);
    //         }            
    //     }
    //     break;
	// 	case (Chassis_Control_Type_SPIN) :
    //     case (Chassis_Control_Type_FLLOW):
    //     {
    //         //底盘四电机模式配置
    //         for (int i = 0; i < 4; i++)
    //         {
    //             Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    //         }
    //         //底盘限速
    //         if (Velocity_X_Max != 0)
    //         {
    //             Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
    //         }
    //         if (Velocity_Y_Max != 0)
    //         {
    //             Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
    //         }
    //         if (Omega_Max != 0)
    //         {
    //             Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
    //         }

    //         #ifdef SPEED_SLOPE
    //         //速度换算，正运动学分解
    //         float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
    //         float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
    //         float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
    //         float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
    //         #else
    //         //速度换算，正运动学分解
    //         float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
    //         float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
    //         float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
    //         float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
    //         #endif     
    //         //线速度 cm/s  转角速度  RAD 
    //         float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
    //         float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
    //         float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
    //         float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
    //         //角速度*减速比  设定目标 直接给到电机输出轴
    //         Motor_Wheel[0].Set_Target_Omega_Radian(  motor2_temp_rad);
    //         Motor_Wheel[1].Set_Target_Omega_Radian(- motor1_temp_rad);
    //         Motor_Wheel[2].Set_Target_Omega_Radian(- motor3_temp_rad);
    //         Motor_Wheel[3].Set_Target_Omega_Radian(  motor4_temp_rad);
    //         //各个电机具体PID
    //         for (int i = 0; i < 4; i++){
    //             Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    //         }

    //     }
    //     break;
    // }   

}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Power_Limit_K = 1.0f;
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    #ifdef C_IMU
    Yaw.Transform_Angle();
    Pitch.Transform_Angle();
    Roll.Transform_Angle();
    #endif

    Left_Leg.Set_Pitch(Pitch.Get_True_Angle_Pitch());
    Right_Leg.Set_Pitch(Pitch.Get_True_Angle_Pitch());
    Left_Leg.Set_Pitch_Gyro(Pitch.Get_True_Gyro_Pitch());
    Right_Leg.Set_Pitch_Gyro(Pitch.Get_True_Gyro_Pitch());
    Left_Leg.Set_MotionAccel(Boardc_BMI.Get_Motion_Accel_Z_N());
    Left_Leg.Set_MotionAccel(Boardc_BMI.Get_Motion_Accel_Z_N());
    Pose_Calculate();
    #ifdef SPEED_SLOPE
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    #endif
    
    //速度解算
    Speed_Resolution();
    #ifdef SUPERCAP
    /****************************超级电容***********************************/
    #ifdef REFEREE
    Supercap.Set_Now_Power(Referee->Get_Chassis_Power());

    if(Referee->Get_Referee_Status()==Referee_Status_DISABLE)
        Supercap.Set_Limit_Power(45.0f);
    else
    {
        float offset;
        offset = (Referee->Get_Chassis_Energy_Buffer()-20.0f)/4;
        Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    }
    #else
        Supercap.Set_Limit_Power(45.0f);
    #endif
    Supercap.TIM_Supercap_PeriodElapsedCallback();
    #endif
    /*************************功率限制策略*******************************/
    #ifdef POWER_LIMIT
    if(__Sprint_Status==Sprint_Status_ENABLE)
    {
        //功率限制  
        #ifdef REFEREE
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max()*1.5f);
        #else
        Power_Limit.Set_Power_Limit(45.0f);
        #endif
    }
    else
    {   
        #ifdef REFEREE
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
        #else
        Power_Limit.Set_Power_Limit(30.0f);
        #endif
    }
   
    // Power_Limit.Set_Motor(Motor_Wheel);   //添加四个电机的控制电流和当前转速
    #ifdef REFEREE
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
    #else
    Power_Limit.Set_Chassis_Buffer(30);
    #endif

    if(Supercap.Get_Supercap_Status()==Supercap_Status_DISABLE)
        Power_Limit.Set_Supercap_Enegry(0.0f);
    else
        Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());
    
    // Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法
    #endif
   
}

void Class_Chassis_Pitch::Transform_Angle()
{
    True_Rad_Pitch  =   IMU->Get_Rad_Roll();
    True_Gyro_Pitch =   IMU->Get_Gyro_Roll();
    True_Angle_Pitch =   IMU->Get_Angle_Roll();
    True_Angle_Total_Pitch =   IMU->Get_True_Angle_Total_Roll();
}

void Class_Chassis_Yaw::Transform_Angle()
{
    True_Rad_Yaw  =   IMU->Get_Rad_Yaw();
    True_Gyro_Yaw =   IMU->Get_Gyro_Yaw();
    True_Angle_Yaw =   IMU->Get_Angle_Yaw();
    True_Angle_Total_Yaw =   IMU->Get_True_Angle_Total_Yaw();

}

void Class_Chassis_Roll::Transform_Angle()
{
    True_Rad_Roll  =   IMU->Get_Rad_Pitch();
    True_Gyro_Roll =   IMU->Get_Gyro_Pitch();
    True_Angle_Roll =   IMU->Get_Angle_Pitch();
    True_Angle_Total_Roll =   IMU->Get_True_Angle_Total_Pitch();

}



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
