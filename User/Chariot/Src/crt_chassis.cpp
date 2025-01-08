/**
 * @file crt_chassis.cpp
 * @author cjw
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
void Class_Tricycle_Chassis::Vector_Plus()//实现向量a+b
{
    for(int i = 0; i < 4; i++)
    {
    float x = Motor_Wheel[i].v * cos(Motor_Steer[i].Yaw) + Motor_Wheel[i].init_v * cos(Motor_Steer[i].init_Yaw);
	float y = Motor_Wheel[i].v * sin(Motor_Steer[i].Yaw) + Motor_Wheel[i].init_v * sin(Motor_Steer[i].init_Yaw);
	Motor_Wheel[i].v = sqrt(x * x + y * y);
	Motor_Steer[i].Yaw = atan2f(y, x);
    }		
}

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.004f,0.008f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.004f,0.008f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    #ifdef POWER_LIMIT
    //超级电容初始化
    Supercap.Init(&hfdcan2,45);
    #endif

    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    //舵向电机PID初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Steer[i].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[i].Get_Output_Max(), Motor_Steer[i].Get_Output_Max(),5,11,0.25,0.001);
        Motor_Steer[i].PID_Omega.Init(15.0f, 60.0f, 0.0f, 0.0f, 4000, Motor_Steer[i].Get_Output_Max(),3,7);
        Motor_Steer[i].PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Steer[i].Get_Output_Max(), Motor_Steer[i].Get_Output_Max());       
    }

    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan2, DJI_Motor_ID_0x206);
    Motor_Steer[1].Init(&hfdcan2, DJI_Motor_ID_0x208);
    Motor_Steer[2].Init(&hfdcan2, DJI_Motor_ID_0x207);
    Motor_Steer[3].Init(&hfdcan2, DJI_Motor_ID_0x205);
    //舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(5.1963);
    Motor_Steer[1].Set_Zero_Position(5.2040);
    Motor_Steer[2].Set_Zero_Position(1.0607);
    Motor_Steer[3].Set_Zero_Position(0.8268);

    Motor_Steer[0].init_Yaw = -PI/4;
    Motor_Steer[1].init_Yaw = PI/4;
    Motor_Steer[2].init_Yaw = PI/4;
    Motor_Steer[3].init_Yaw = -PI/4;

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}


/**
 * @brief 速度解算
 *
 */
float temp_test_1,temp_test_2,temp_test_3,temp_test_4;
float car_V;//车体总体朝向与速度
float car_R;
float car_yaw;
void Class_Tricycle_Chassis::Speed_Resolution(){  
    for(int i = 0; i < 4; i++){
        Motor_Steer[i].Pre_Yaw = Motor_Steer[i].Yaw;
    }
    
    //获取当前速度值，用于速度解算初始值获取
    car_V = sqrtf(Get_Target_Velocity_X() * Get_Target_Velocity_X() + Get_Target_Velocity_Y() * Get_Target_Velocity_Y());
    car_yaw = atan2f(Get_Target_Velocity_Y(),Get_Target_Velocity_X());

    //角度处理
    if(car_yaw > PI/2) {
        car_yaw = car_yaw - PI;
        car_V = -car_V;
    }
    else if(car_yaw < -PI/2){
        car_yaw = car_yaw + PI;
        car_V = -car_V;
    }
   

    Motor_Steer[0].Yaw = car_yaw; 
    Motor_Wheel[0].v = car_V;
    Motor_Steer[1].Yaw = car_yaw; 
    Motor_Wheel[1].v = car_V;
    Motor_Steer[2].Yaw = car_yaw;
    Motor_Wheel[2].v = car_V;
    Motor_Steer[3].Yaw = car_yaw;
    Motor_Wheel[3].v = car_V;

    //运动模式
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 轮组无力
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
                Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                Motor_Wheel[i].Set_Out(0.0f);

                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Motor_Steer[i].PID_Torque.Set_Integral_Error(0.0f);
                Motor_Steer[i].Set_Target_Torque(0.0f);
                Motor_Steer[i].Set_Out(0.0f);
            }            
        }
        break;
		case (Chassis_Control_Type_SPIN) :
		{
		    Motor_Wheel[0].init_v = Get_Target_Omega();
            Motor_Wheel[1].init_v = - Get_Target_Omega();
            Motor_Wheel[2].init_v = Get_Target_Omega();
            Motor_Wheel[3].init_v = - Get_Target_Omega();

            Vector_Plus(); 

            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Target_Angle(Motor_Steer[i].Yaw * 180 / PI);//电机坐标系逆时针为正 编码器左手坐标系
                Motor_Wheel[i].Set_Target_Omega_Radian(Motor_Wheel[i].v / Wheel_Diameter / 2); //线速度转角速度
            }

			//底盘四电机模式配置
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);
            }

            Control_Update();
		}
        break;
        case (Chassis_Control_Type_FLLOW):
        {
            //底盘四电机模式配置
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);
            }
           
            Motor_Wheel[0].init_v = Get_Target_Omega();
            Motor_Wheel[1].init_v = - Get_Target_Omega();
            Motor_Wheel[2].init_v = Get_Target_Omega();
            Motor_Wheel[3].init_v = - Get_Target_Omega();

            Vector_Plus();           

            for (int i = 0; i < 4; i++) {
                Motor_Steer[i].Set_Target_Angle(Motor_Steer[i].Yaw * 180 / PI);//电机坐标系逆时针为正 编码器左手坐标系
                Motor_Wheel[i].Set_Target_Omega_Radian(Motor_Wheel[i].v / Wheel_Diameter / 2); //线速度转角速度
            }

            Control_Update();
            
        }
        break;
    }   
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Power_Limit_K = 1.0f;
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
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
    

    #if POWER_CONTROL == 1
    
    /****************************超级电容***********************************/
    // Supercap.Set_Now_Power(Referee->Get_Chassis_Power());
    // if(Referee->Get_Referee_Status()==Referee_Status_DISABLE)
    //     Supercap.Set_Limit_Power(45.0f);
    // else
    // {
    //     float offset;
    //     offset = (Referee->Get_Chassis_Energy_Buffer()-20.0f)/4;
    //     Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    // }
        
    // Supercap.TIM_Supercap_PeriodElapsedCallback();

    /*************************功率限制策略*******************************/
    Power_Limit_Update();

    #endif
}

void Class_Tricycle_Chassis::Axis_Transform(void){
    for(int i=0;i<4;i++){
        if(Motor_Steer[i].Get_Now_Radian() > Motor_Steer[i].Get_Zero_Position()){
            Motor_Steer[i].Yaw = -(Motor_Steer[i].Get_Now_Radian() - Motor_Steer[i].Get_Zero_Position());//电机数据转现实坐标系
            if(Motor_Steer[i].Yaw < -PI) Motor_Steer[i].Yaw += 2*PI;
        }
        else if (Motor_Steer[i].Get_Now_Radian() < Motor_Steer[i].Get_Zero_Position())
        {
            Motor_Steer[i].Yaw = Motor_Steer[i].Get_Zero_Position() - Motor_Steer[i].Get_Now_Radian();
            if(Motor_Steer[i].Yaw > PI) Motor_Steer[i].Yaw -= 2*PI;
        }
        
    }
}

void Class_Tricycle_Chassis::Control_Update()
{
    float temp_err = 0.0f,temp_min = 0.0f;
    for(int i=0;i<4;i++)
    {
        // 计算误差，考虑当前电机状态
        temp_err = Motor_Steer[i].Get_Target_Angle() - Motor_Steer[i].t_yaw*180/PI - Motor_Steer[i].invert_flag * 180.0f;

        // 标准化到[0, 360)范围
        while (temp_err > 360.0f)
            temp_err -= 360.0f;
        while (temp_err < 0.0f)
            temp_err += 360.0f;

        // 比较路径长度
        if (fabs(temp_err) < (360.0f - fabs(temp_err)))
            temp_min = fabs(temp_err);
        else
            temp_min = 360.0f - fabs(temp_err);

        // 判断是否需要切换方向
        if (temp_min > 90.0f)
        {
            Motor_Steer[i].invert_flag = !Motor_Steer[i].invert_flag;
            // 重新计算误差
            temp_err = Motor_Steer[i].Get_Target_Angle() - Motor_Steer[i].t_yaw*180/PI - Motor_Steer[i].invert_flag * 180.0f;
        }

        if(temp_err > 180.0f)      temp_err -= 360.0f;
        else if(temp_err < -180.0f)temp_err += 360.0f;

        Motor_Steer[i].Set_Target_Angle(Motor_Steer[i].t_yaw*180/PI + temp_err);
        if(Motor_Steer[i].invert_flag == 1)Motor_Wheel[i].Set_Target_Omega_Radian(-Motor_Wheel[i].Get_Target_Omega_Radian());
        else Motor_Wheel[i].Set_Target_Omega_Radian(Motor_Wheel[i].Get_Target_Omega_Radian());

        Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    }

}

void Class_Tricycle_Chassis::Power_Limit_Update()
{
    int16_t temp_out = 0,temp_torque = 0,temp_omega = 0,index = 0;
    Power_Management.Max_Power = 100.f;
    for(int i=0;i<8;i++)
    {
        if(i % 2 == 0){
            index = i / 2 + i % 2;
            temp_out = Motor_Steer[index].Get_Out();
            temp_torque = Motor_Steer[index].Get_Out() * GET_CMD_CURRENT_TO_TORQUE(i);
            temp_omega = Motor_Steer[index].Get_Now_Omega_Radian() * 60 / (2 * PI);
            Power_Management.Motor_Data[i].pid_output = temp_out;
            Power_Management.Motor_Data[i].torque = temp_torque;
            Power_Management.Motor_Data[i].feedback_omega = temp_omega;
        }
        else if(i % 2 == 1){
            index = i / 2 + i % 2;
            temp_out = Motor_Wheel[index].Get_Out();
            temp_torque = Motor_Wheel[index].Get_Out() * GET_CMD_CURRENT_TO_TORQUE(i);
            temp_omega = Motor_Wheel[index].Get_Now_Omega_Radian() * 60 / (2 * PI);
            Power_Management.Motor_Data[i].pid_output = temp_out;
            Power_Management.Motor_Data[i].torque = temp_torque;
            Power_Management.Motor_Data[i].feedback_omega = temp_omega;
        }
    }

    Power_Limit.Power_Task(Power_Management);

    //输出
    for(int i=0;i<8;i++)
    {
        if(i % 2 == 0){
            index = i / 2 + i % 2;
            Motor_Steer[index].Set_Out(Power_Management.Motor_Data[i].output);
        }
        else if(i % 2 == 1){
            index = i / 2 + i % 2;
            Motor_Wheel[index].Set_Out(Power_Management.Motor_Data[i].output);
        }
    }
    
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
