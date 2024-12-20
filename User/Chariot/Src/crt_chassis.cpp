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

double My_atan(double y, double x)
{
	double atan;
	atan = atan2(y, x);
	if (x == 0)
	{
		if (y > 0)
			atan = PI / 2;
		else if (y < 0)
			atan = -PI / 2;
	}
	if (y == 0)
	{
		if (x > 0)
			atan = 0;
		else if (x < 0)
			atan = PI;
	}
	return atan;
}

float Square(float Input) //
{
	float Ans;
	Ans = Input * Input;
	return Ans;
}


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
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    //舵向电机PID初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Steer[i].PID_Angle.Init(0.1f, 0.0f, 0.0f, 0.0f, Motor_Steer[i].Get_Output_Max(), Motor_Steer[i].Get_Output_Max(),5,11,0.25,0.001);
        Motor_Steer[i].PID_Omega.Init(10.0f, 5.0f, 0.0f, 0.0f, Motor_Steer[i].Get_Output_Max(), Motor_Steer[i].Get_Output_Max(),3,7);
        Motor_Steer[i].PID_Torque.Init(0.8f, 100.0f, 0.0f, 0.0f, Motor_Steer[i].Get_Output_Max(), Motor_Steer[i].Get_Output_Max());       
    }

    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan2, DJI_Motor_ID_0x206);
    Motor_Steer[1].Init(&hfdcan2, DJI_Motor_ID_0x208);
    Motor_Steer[2].Init(&hfdcan2, DJI_Motor_ID_0x205);
    Motor_Steer[3].Init(&hfdcan2, DJI_Motor_ID_0x207);
    //舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(0.5261);
    Motor_Steer[1].Set_Zero_Position(2.8739);
    Motor_Steer[2].Set_Zero_Position(1.6260);
    Motor_Steer[3].Set_Zero_Position(5.7485);

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
    //Axis_Transform();   
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
   

    Motor_Steer[0].Yaw = car_yaw; //+ DEG_TO_RAD*8; 
    Motor_Wheel[0].v = car_V;
    Motor_Steer[1].Yaw = car_yaw; //+ DEG_TO_RAD*13;
    Motor_Wheel[1].v = car_V;
    Motor_Steer[2].Yaw = car_yaw; //+ DEG_TO_RAD*13;
    Motor_Wheel[2].v = car_V;
    Motor_Steer[3].Yaw = car_yaw; //+ DEG_TO_RAD*13;
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

			//底盘四电机模式配置
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);
            }
            //底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
            }

            for (int i = 0; i < 4; i++){
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
            }
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
           
            
            // Motor_Steer[0].init_Yaw = 0 + -PI/4;
            // Motor_Steer[1].init_Yaw = 0 + PI/4;
            // Motor_Steer[2].init_Yaw = 0 + PI/4;
            // Motor_Steer[3].init_Yaw = 0 + -PI/4;

            // for (int i = 0; i < 4; i++) {
			// 	Motor_Steer[i].Yaw += 0;
			// }

            Vector_Plus();           

            // for (int i = 0; i < 4; i++) {
            //     // if (abs(Motor_Steer[i].Pre_Yaw - Motor_Steer[i].Yaw) > PI/2 )//如果变换角度大于90° 
            //     // {
            //     //     if (Motor_Steer[i].Yaw < Motor_Steer[i].Pre_Yaw) {
            //     //         Motor_Steer[i].Yaw += PI;
            //     //     }
            //     //     else { 
            //     //         Motor_Steer[i].Yaw -= PI;
            //     //     }
            //     //     Motor_Wheel[i].v = -Motor_Wheel[i].v;
            //     // }
            //     // if(Motor_Steer[i].Yaw > PI/2) {
            //     //     Motor_Steer[i].Yaw = Motor_Steer[i].Yaw - PI;
            //     //     Motor_Wheel[i].v = -Motor_Wheel[i].v;
            //     // }
            //     // else if(Motor_Steer[i].Yaw < -PI/2){
            //     //     Motor_Steer[i].Yaw = Motor_Steer[i].Yaw + PI;
            //     //     Motor_Wheel[i].v = -Motor_Wheel[i].v;
            //     }		
	        // }


            for (int i = 0; i < 4; i++) {
                if(Motor_Steer[i].Yaw-Motor_Steer[i].t_yaw>= PI) 
                {
                    Motor_Steer[i].Yaw-= 2*PI;
                }

                if(Motor_Steer[i].Yaw-Motor_Steer[i].t_yaw<= -PI) 
                {
                    Motor_Steer[i].Yaw+= 2*PI;
                }
                if (abs(Motor_Steer[i].Pre_Yaw - Motor_Steer[i].Yaw) > PI/2 )//如果变换角度大于90° 
                {
                    if (Motor_Steer[i].Yaw < Motor_Steer[i].Pre_Yaw) {
                        Motor_Steer[i].Yaw += PI;
                    }
                    else { 
                        Motor_Steer[i].Yaw -= PI;
                    }
                    Motor_Wheel[i].v = -Motor_Wheel[i].v;
                }
                Motor_Steer[i].Set_Target_Angle(Motor_Steer[i].Yaw*360/2/PI);//电机坐标系逆时针为正 编码器左手坐标系
                Motor_Wheel[i].Set_Target_Omega_Radian(Motor_Wheel[i].v*10.0f); //rad/s
            }
            
            //各个电机具体PID
            for (int i = 0; i < 4; i++){
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
            }
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
    

    #ifdef POWER_LIMIT
    
    /****************************超级电容***********************************/
    Supercap.Set_Now_Power(Referee->Get_Chassis_Power());
    if(Referee->Get_Referee_Status()==Referee_Status_DISABLE)
        Supercap.Set_Limit_Power(45.0f);
    else
    {
        float offset;
        offset = (Referee->Get_Chassis_Energy_Buffer()-20.0f)/4;
        Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + offset);
    }
        
    Supercap.TIM_Supercap_PeriodElapsedCallback();

    /*************************功率限制策略*******************************/
    if(__Sprint_Status==Sprint_Status_ENABLE)
    {
        //功率限制  
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max()*1.5f);
    }
    else
    {
        Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    }
    //Power_Limit.Set_Power_Limit(45.0f);
    Power_Limit.Set_Motor(Motor_Wheel);   //添加四个电机的控制电流和当前转速
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());

    if(Supercap.Get_Supercap_Status()==Supercap_Status_DISABLE)
        Power_Limit.Set_Supercap_Enegry(0.0f);
    else
        Power_Limit.Set_Supercap_Enegry(Supercap.Get_Stored_Energy());
    
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);  //功率限制算法

    #endif
}

void Class_Tricycle_Chassis::Axis_Transform(void){
//	tttt_test = Motor_Steer[0].Get_Now_Radian();
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
/*
初始状态：
Now_Angle = 0°
Target_Angle = 135°
invert_flag = 0（不反转）

1. 角度优化：
   temp_err = 135° - 0° - (0 * 180°) = 135°

   比较路径：
   - 直接路径 = |135°| = 135°
   - 绕行路径 = 360° - |135°| = 225°
   temp_min = min(135°, 225°) = 135°

   因为 temp_min(135°) > 90°：
   - invert_flag 切换为 1（反转）
   - 重新计算误差：
     temp_err = 135° - 0° - (1 * 180°) = -45°

2. 优劣弧优化：
   temp_err = -45° 已经在[-180°, 180°]范围内，无需调整

3. 最终结果：
   - 电机反转
   - 逆时针转动45°
*/
// void Control_Update(Class_Steering_Wheel *steering_wheel)
// {
//     float temp_err = 0.0f;

//     // 1. 角度优化
//     if (steering_wheel->parameter.deg_optimization == ENABLE_MINOR_DEG_OPTIMIZEATION)
//     {
//         float temp_min;

//         // 计算误差，考虑当前电机状态
//         temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;

//         // 标准化到[0, 360)范围
//         while (temp_err > 360.0f)
//             temp_err -= 360.0f;
//         while (temp_err < 0.0f)
//             temp_err += 360.0f;

//         // // 比较路径长度
//         // if (fabs(temp_err) < (360.0f - fabs(temp_err)))
//         //     temp_min = fabs(temp_err);
//         // else
//         //     temp_min = 360.0f - fabs(temp_err);

//         // // 判断是否需要切换方向
//         // if (temp_min > 90.0f)
//         // {
//         //     steering_wheel->invert_flag = !steering_wheel->invert_flag;
//         //     // 重新计算误差
//         //     temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle - steering_wheel->invert_flag * 180.0f;
//         // }
//     }
//     else
//     {
//         temp_err = steering_wheel->Target_Angle - steering_wheel->Now_Angle;
//     }

//     // // 2. 优劣弧优化，实际上角度优化那里已经完成了
//     // if (steering_wheel->parameter.arc_optimization == ENABLE_MINOR_ARC_OPTIMIZEATION)
//     // {
//     //     if (temp_err > 180.0f)
//     //     {
//     //         temp_err -= 360.0f;
//     //     }
//     //     else if (temp_err < -180.0f)
//     //     {
//     //         temp_err += 360.0f;
//     //     }
//     // }
//     steering_wheel->Target_Angle = steering_wheel->Now_Angle + temp_err;
//     // PID参数更新
//     steering_wheel->Motion_Motor.Set_Target_Omega_Radian(steering_wheel->Target_Velocity / Wheel_Diameter * 2 * steering_wheel->invert_flag);
//     steering_wheel->Motion_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Velocity / Wheel_Diameter * 2); // 这里有点怪异，因为Get_Now_Omega_Radian和Set_Target_Omega_Radian返回的数据不一样，准确来说Get_Now_Omega_Radian应该改为Get_Now_Motor_Data_Omega_Radian,但是懒得改了

//     steering_wheel->Directive_Motor.Set_Target_Radian(steering_wheel->Target_Angle * DEG_TO_RAD);
//     steering_wheel->Directive_Motor.Set_Now_Radian(steering_wheel->Now_Angle * DEG_TO_RAD);       // 转向轮的当前数据不直接来自于电机
//     steering_wheel->Directive_Motor.Set_Now_Omega_Radian(steering_wheel->Now_Omega * DEG_TO_RAD); // 转向轮的当前数据不直接来自于电机

//     // PID计算
//     steering_wheel->Motion_Motor.TIM_PID_PeriodElapsedCallback();
//     steering_wheel->Directive_Motor.TIM_PID_PeriodElapsedCallback();
// }
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
