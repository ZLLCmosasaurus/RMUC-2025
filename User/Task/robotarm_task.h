/**
 * @file robotarm_task.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 机械臂电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef ROBOTARM_TASK_H
#define ROBOTARM_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "dvc_djimotor.h"
#include "dvc_AKmotor.h"
#include "dvc_referee.h"
#include "alg_fsm.h"
#include "dvc_dr16.h"
#include "chassis_Communication.h"
#include "dvc_dmmotor.h"
#include "dvc_imu.h"
#include "drv_math.h"
#include "drv_can.h"
#include "dvc_minipc.h"
#include "dvc_relays.h"
#include "crt_chassis.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class Class_Robotarm;
/**
 * @brief 云台控制类型
 *
 */
enum Enum_Robotarm_Control_Type
{
    Robotarm_Control_Type_DISABLE = 0,
    Robotarm_Control_Type_NORMAL,
};
/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type
{
    Gimbal_Control_Type_DR16 = 0,
    Gimbal_Control_Type_KEY,
};
/**
 * @brief 键鼠控制机械臂时控制类型
 *
 */
enum Enum_Gimbal_Control_Key_Type
{
    Gimbal_Control_Type_Key_Normal = 0,
    Gimbal_Control_Type_Key_Customize,
	Gimbal_Control_Type_Key_FSM,
};
/**
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,
};

/**
 * @brief 解算任务运行阶段
 *
 */
enum Enum_Robotarm_Task_Status
{
  	  	Robotarm_Task_Status_Calibration = 1,
		Robotarm_Task_Status_Wait_Order=2,
		Robotarm_Task_Status_Error=3,
		Robotarm_Task_Status_Exchange=4,
	
    	Robotarm_Task_Status_Pick_Sliver_1=11,
		Robotarm_Task_Status_Pick_Sliver_2=12,//中期考核使用状态，后期去除
		Robotarm_Task_Status_Pick_Sliver_3=13,
		Robotarm_Task_Status_Pick_Gold_1=21,
		Robotarm_Task_Status_Pick_Gold_2=22,
		Robotarm_Task_Status_Pick_Gold_3=23,
		Robotarm_Task_Status_Pick_Gold_4=24,
		Robotarm_Task_Status_Pick_Gold_5=25,
		Robotarm_Task_Status_Pick_Gold_6=24,
		Robotarm_Task_Status_Pick_Gold_7=25,
	
	
		
	
	


};

/**
 * @brief 解算任务运行阶段
 *
 */
enum Enum_Joint_Limit_Flag
{
    Joint_Limit_Flag_Min = 0,
    Joint_Limit_Flag_Max,
};
/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type__ 
{
    CHASSIS_Control_Type_DISABLE = 0,
    CHASSIS_Control_Type_FLLOW = 1,
    CHASSIS_Control_Type_SPIN = 2,
};
/**
* @brief 是否冲刺控制类型
 *
 */
//enum Enum_Sprint_Status : uint8_t
//{
//    Sprint_Status_DISABLE = 0, 
//    Sprint_Status_ENABLE,
//};

struct Struct_Custom_Communication_Data {
	uint16_t Flow_x;
	uint16_t Flow_y;
    uint16_t roll; 
	uint16_t pitch;
	uint16_t yaw;
}__attribute__((packed));

//底盘移动
struct Chassis_Move_t
{	
	//移动速度
	float Chassis_Vx;
	float Chassis_Vy;
	float Chassis_Wz;
	
	//运算符重载 !=
	bool operator != (const Chassis_Move_t &a)
	{
		if((this->Chassis_Vx != a.Chassis_Vx)||(this->Chassis_Vy != a.Chassis_Vy)||(this->Chassis_Wz != a.Chassis_Wz))
		 {
			return true;
		 }
		 else 
		 {
			return false;
		 }
		
	}
};

//机械臂目标位姿
struct Position_Orientation_t
{
	//位置
	float X_Position;
	float Y_Position;
	float Z_Position;
	//姿态
	float Pitch_Angle;
	float Yaw_Angle;
	float Roll_Angle;
	
	//运算符重载 +
	Position_Orientation_t operator + (const Position_Orientation_t &a)
	{
		Position_Orientation_t c;
		c.X_Position = this->X_Position + a.X_Position;
		c.Y_Position = this->Y_Position + a.Y_Position;
		c.Z_Position = this->Z_Position + a.Z_Position;
		c.Pitch_Angle = this->Pitch_Angle + a.Pitch_Angle;
		c.Yaw_Angle = this->Yaw_Angle + a.Yaw_Angle;
		c.Roll_Angle = this->Roll_Angle + a.Roll_Angle;
		return c;
	}
	
	//运算符重载 !=
	bool operator != (const Position_Orientation_t &a)
	{
		if((this->X_Position != a.X_Position)||(this->Y_Position != a.Y_Position)||(this->Z_Position != a.Z_Position)
		  ||(this->Pitch_Angle != a.Pitch_Angle)||(this->Yaw_Angle != a.Yaw_Angle)||(this->Roll_Angle != a.Roll_Angle))
		 {
			return true;
		 }
		 else 
		 {
			return false;
		 }
		
	}
};
/**
 * @brief Specialized, 解算策略有限自动机
 *
 */
class Class_Robotarm_Resolution : public Class_FSM
{
public:
    Class_Robotarm *Robotarm;

    void Reload_Task_Status_PeriodElapsedCallback();
};
//抬升机构类
class Class_RoRobotic_Arm_Uplift : public Class_DJI_Motor_C620
{
public:
	//变量
	float Target_Up_Length;
	float Actual_Up_Length;
	float K_Target = 5.0f;
  float K_Actual = 0.018f;
	//函数
	void Calculate_Actual_Up_Length(float Off_Set_Angle);
  void TIM_PID_PeriodElapsedCallback();

};
class Class_Chassis_Communication__
{
	
public:
	// PID角度环控制
  Class_PID PID_Yaw;
	float Chassis_Vx;
	float Chassis_Vy;
	float Chassis_Omega;
	float Target_Yaw=0.f;
	float Actual_Yaw;
	float Max_Chassis_Vx = 3.0f;
	float Max_Chassis_Vy = 3.0f;
	float Max_Chassis_Omega = 3.0f;
  void TIM_PID_PeriodElapsedCallback();
 
};

/**
 * @brief 机器人是否离线 控制模式有限自动机
 *
 */
class Class_FSM_Alive_Control : public Class_FSM
{
public:
    Class_Robotarm *Robotarm;

    void Reload_TIM_Statu_PeriodElapsedCallback();
};
/**
 * @brief Specialized, 大臂取矿石，兑矿有限自动机
 *
 */
class Class_FSM_Manipulator_Move : public Class_FSM
{
public:
    Class_Robotarm *Robotarm;
//    float Heat;

    void Reload_TIM_Status_PeriodElapsedCallback();
};


/**
* @brief Specialized, 机械臂类
 *
 */
class Class_Robotarm
{
public:
	float servo_angle_picth=100;
	float servo_angle_yaw=90;
	//1-5轴关节初始化角度+抬升机构长度
	float Jonit_AngleInit[6]={0,0,0,0,0,0};
		//Joint坐标系角度
	float Joint_World_Angle[5] = {0.0f};
	float Joint_World_Angle_Now[6] = {0.0f};
    //遥控器离线保护控制状态机
       Class_FSM_Alive_Control FSM_Alive_Control;
      friend class Class_FSM_Alive_Control;
		//	

	//机械臂解算有限状态机
	friend class Class_Robotarm_Resolution;
	Class_Robotarm_Resolution Robotarm_Resolution;
    //imu对象
   Class_IMU Boardc_BMI;
	//遥控器
	Class_DR16 DR16;
	//裁判系统
    Class_Referee Referee;
	//底盘通信
	Class_Chassis_Communication Chassis_Communication;
	//迷你主机通信
	Class_MiniPC MiniPc;
	//校准完成标志位
	bool init_finished = false;
    // 1轴电机
	Class_AK_Motor_80_6 Motor_Joint1;
	// 2轴电机
	Class_AK_Motor_80_6 Motor_Joint2;
	// 3轴电机
 	Class_DM_Motor_J4310 Motor_Joint3;
	// 4轴电机
	Class_DJI_Motor_C610 Motor_Joint4;
	// 6轴电机
	Class_DJI_Motor_C610 Motor_Joint5;
	// 继电器类
	Class_Relays Relays;
	//机械臂抬升类
	Class_RoRobotic_Arm_Uplift Arm_Uplift;
	//底盘通信
	Class_Chassis_Communication__ Chassis;
	//底盘控制方式
	Enum_Chassis_Control_Type__  Chassis_control_type;
	//机械臂控制方式
	Enum_Gimbal_Control_Type Gimbal_Control_Type=Gimbal_Control_Type_DR16;

	Enum_Gimbal_Control_Key_Type Gimbal_Control_Key_Type =Gimbal_Control_Type_Key_Normal;
	//冲刺
	Enum_Sprint_Status Sprint_Status = Sprint_Status_DISABLE;
	  //裁判系统UI刷新状态
   Enum_Referee_UI_Refresh_Status Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
	 void Robotarm_Referee_UI_Tx_Callback(Enum_Referee_UI_Refresh_Status __Referee_UI_Refresh_Status);
	//存放对象的地址，需要用强制转换
	uint32_t Motor_Joint[6]= {reinterpret_cast<uint32_t>(&Motor_Joint1),reinterpret_cast<uint32_t>(&Motor_Joint2),reinterpret_cast<uint32_t>(&Motor_Joint3),
							 reinterpret_cast<uint32_t>(&Motor_Joint4),reinterpret_cast<uint32_t>(&Motor_Joint5)};
	//获得关节角度
	inline float Get_Joint_World_Angle(uint8_t Num);
	
	inline float Get_Joint_Offset_Angle(uint8_t Num);
	
	inline float Get_Joint_Limit_Angle(uint8_t Num);
							 
	inline Enum_DR16_Control_Type Get_DR16_Control_Type();

	//获得目标位姿
	inline Position_Orientation_t Get_Target_Position_Orientation();
	//获得实际位姿
	inline Position_Orientation_t Get_Actual_Position_Orientation();
	//设置机械臂是否使能
	inline void Set_Robotarm_Control_Type(Enum_Robotarm_Control_Type __Robotarm_Control_Type);
	//设置关节角度
	inline void Set_Joint_World_Angle(uint8_t Num,float __Joint_World_Angle);
	inline void Set_Joint_World_Angle_Now(uint8_t Num,float __Joint_World_Angle);
	//设置目标位姿
	inline void Set_Target_Position_Orientation(Position_Orientation_t __Target_Position_Orientation);
							 
	inline void Set_Joint_1_5_Angle_Init_Data();

	//初始化类
   void Init();
	//机械臂抬升
	void Control_RoRobotic_Arm_Uplift();
	//控制底盘
	void Control_Chassis_Task();
	//控制机械臂
	void Control_Gimbal_Task();
	//控制继电器
	void Control_Relays_Task();
	//云台CAN发送数据给底盘
	void CAN_Gimbal_Tx_Chassis();
	//控制任务
	void TIM_Control_Callback();
	//机械臂上电校准
	bool Robotarm_Calibration();
	//检查机械臂是否到达目标位置
	bool Robotarm_Angle_verification(float *Angle_now,float *Angle_target);
	void TIM_Robotarm_Task_PeriodElapsedCallback();
	void Task_Alive_PeriodElapsedCallback();
	void TIM_Robotarm_Disable_PeriodElapsedCallback();
	//机械臂输出
    void Output();
	void Judge_DR16_Control_Type();
	void Control_Chassis();
protected:
    //初始化相关常量

    //常量
		//键鼠模式按住shift 最大速度缩放系数	
        float DR16_Mouse_Chassis_Shift = 2.0f;
		//车在特定位置，各种情况机械臂的角度
		//取出银矿相关角度
		//const float Angle_Pick_Fisrt[6]={90,145,145,0,0,5};
		const float Angle_Pick_Fisrt[6]={70,110,180,0,0,5};
		const float Angle_Place_Second[6]={45,3,10,0,0,13};
		//const float Angle_Pick_Second[6]={45,145,178,0,0,5};
		const float Angle_Pick_Second[6]={70,110,0,0,0,5};
		const float Angle_Place_Fisrt[6]={150,178,135,0,0,13};
    	const float Angle_Pick_Third[6]={3,140,170,0,0,0};
		//取出金矿相关角度		
		const float Angle_Pick_Gold[6]={120,45,115,0,90,5.0f};
		const float Angle_Pick_Gold_UP[6]={120,45,115,0,115,5.0f};
		
		const float Angle_Pick_Gold_1[6]={120.f,0.f,180.f,0.f,90.f,0.f};


		//行进过程中角度			
		const float Angle_On_The_Way[6] = {60.f,210.0f,0.0f,0.0f,90.0f,0};
    //关节角度限制
    const float Joint_Limit_Angle[2][5] = {{-90.00f,-179.85f,-121.8f,-90.0f,-40.6f},
										   {91.22f,135.0f,109.88f,90.0f,207.541f}};

	const Enum_Joint_Limit_Flag Joint_Limit_Flag[5] = {Joint_Limit_Flag_Max,Joint_Limit_Flag_Min,Joint_Limit_Flag_Min,Joint_Limit_Flag_Max,Joint_Limit_Flag_Min};


	//遥控器拨动的死区, 0~1
    const float DR16_Dead_Zone = 0.01f;
	const float Controller_Dead_Zone = 0.1f;

    //常量
	

    //内部变量
	
    //读变量	

    //写变量

    //机械臂状态
    Enum_Robotarm_Control_Type Robotarm_Control_Type = Robotarm_Control_Type_NORMAL;

    //读写变量
	//底盘移动结构体
	Chassis_Move_t Chassis_Move;
	
	Struct_Custom_Communication_Data Custom_Communication_Data;

	// 到达机械限位后的电机角度值，机械臂＋抬升
	float Joint_Offset_Angle[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	
	// 目标位置(mm)
	Position_Orientation_t Target_Position_Orientation = {209.314f, 0.0f,30.0f};
	// 两连杆目标位置(mm)
	Position_Orientation_t Target_Position_Orientation_Two;
	// 实际位置(mm)
	Position_Orientation_t Actual_Position_Orientation;
	// 连杆3Cos分解后的长度(mm)
	float Arm3_Cos_Length = 0.0f;
	
	//内部函数
	
	//通过模版函数来设置电机角度
	template <typename T1>
	inline void Set_Motor_Angle(T1 &Motor,uint8_t num);
	
	//电机上电校准
	bool Motor_Calibration(Class_DJI_Motor_C620 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle);
	bool Motor_Calibration(Class_AK_Motor_80_6 &Motor,uint8_t num,float Cali_Omega,float Target_Angle);
	bool Motor_Calibration(Class_DJI_Motor_C610 &Motor,uint8_t num,float Cali_Omega,float Cali_Max_Out,float Target_Angle);
	bool Motor_Calibration(float Cali_Omega,float Cali_Max_Out);
  bool Motor_Calibration_Uplift(Class_RoRobotic_Arm_Uplift &Uplift,float Cali_Omega,float Cali_Max_Out);
	
	Enum_DR16_Control_Type DR16_Control_Type = DR16_Control_Type_REMOTE;
	//键鼠操作相关函数
  
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


 /**
     * @brief 获取DR16控制数据来源
     * 
     * @return Enum_DR16_Control_Type DR16控制数据来源
     */

Enum_DR16_Control_Type Class_Robotarm::Get_DR16_Control_Type()
    {
        return (DR16_Control_Type);
    }


/**
 * @brief 获取第Num个Joint的世界坐标系角度
 *
 * @return float 第Num个Joint的角度
 */
float Class_Robotarm::Get_Joint_World_Angle(uint8_t Num)
{
    return (Joint_World_Angle[Num-1]);
}

/**
 * @brief 获取第Num个Joint的世界坐标系角度
 *
 * @return float 第Num个Joint的角度
 */
float Class_Robotarm::Get_Joint_Offset_Angle(uint8_t Num)
{
    return (Joint_Offset_Angle[Num-1]);
}

/**
 * @brief 获取第Num个Joint的世界坐标系角度
 *
 * @return float 第Num个Joint的角度
 */
float Class_Robotarm::Get_Joint_Limit_Angle(uint8_t Num)
{
    return (Joint_Limit_Angle[Joint_Limit_Flag[Num-1]][Num-1]);
}

template <typename T1>
void Class_Robotarm::Set_Motor_Angle(T1 &Motor,uint8_t num)
{
	Motor.Set_Target_Angle(Joint_World_Angle[num-1] + Joint_Offset_Angle[num-1] - Joint_Limit_Angle[Joint_Limit_Flag[num-1]][num-1]);
}

/**
 * @brief 获取目标位置单位(mm)
 *
 * @return Position_Orientation_t 位置单位(mm)
 */
Position_Orientation_t Class_Robotarm::Get_Target_Position_Orientation()
{
    return (Target_Position_Orientation);
}

/**
 * @brief 获取实际位置单位(mm)
 *
 * @return Position_Orientation_t 位置单位(mm)
 */
Position_Orientation_t Class_Robotarm::Get_Actual_Position_Orientation()
{
    return (Actual_Position_Orientation);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Robotarm::Set_Robotarm_Control_Type(Enum_Robotarm_Control_Type __Robotarm_Control_Type)
{
    Robotarm_Control_Type = __Robotarm_Control_Type;
}

/**
 * @brief 设置第Num个Joint的世界坐标系角度
 *
 * @param Num 第Num个Joint __Joint_World_Angle 设置角度
 */
void Class_Robotarm::Set_Joint_World_Angle(uint8_t Num,float __Joint_World_Angle)
{
	Joint_World_Angle[Num-1] = __Joint_World_Angle;
}
/**
 * @brief 设置第Num个Joint的世界坐标系角度
 *
 * @param Num 第Num个Joint __Joint_World_Angle 设置角度
 */
void Class_Robotarm::Set_Joint_World_Angle_Now(uint8_t Num,float __Joint_World_Angle)
{
	Joint_World_Angle_Now[Num-1] = __Joint_World_Angle;
}

/**
 * @brief 设定位置X单位(mm)
 *
 */
void Class_Robotarm::Set_Target_Position_Orientation(Position_Orientation_t __Target_Position_Orientation)
{
    Target_Position_Orientation = __Target_Position_Orientation;
}



#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
