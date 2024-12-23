/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{ 
    //yaw电机canid初始化
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x201);
    //拉力电机canid初始化
    Motor_Down.Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Up.Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Left.Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_Right.Init(&hcan1, DJI_Motor_ID_0x204);

    // PID 初始化
    //拉力环 速度环
    PID_Tension.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Motor_Down.PID_Omega.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // 上膛电机 速度环即可
    Motor_Left.PID_Omega.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Motor_Right.PID_Omega.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // yaw电机 角度环 速度环
    Motor_Yaw.PID_Omega.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Motor_Yaw.PID_Angle.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // 推镖电机 速度环即可
    Motor_Up.PID_Omega.Init(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    //TensionMeter初始化
    Tension_Meter.Init(GPIOF, GPIO_PIN_1, GPIOF, GPIO_PIN_0);
    //舵机初始化
    Servo_Load_1.Init(&htim5,TIM_CHANNEL_1);
    Servo_Load_2.Init(&htim5,TIM_CHANNEL_2);
    Servo_Load_3.Init(&htim5,TIM_CHANNEL_3);
    Servo_Load_4.Init(&htim5,TIM_CHANNEL_4);
    //扳机舵机初始化
    Servo_Trigger.Init(&htim4,TIM_CHANNEL_4);

    //遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);
    // Dart 控制 状态机
    FSM_Dart_Control.Chariot = this;
    FSM_Dart_Control.Init(7, 0);

    //遥控器 初始化
    DR16.Init(&huart1,&huart6);
    DR16_Dead_Zone = __DR16_Dead_Zone;   

    //裁判系统 初始化
    Referee.Init(&huart6,0xA5);

    //调试控制 初始化
    DebugControl.Init(&huart7);
            
    //上位机
    // MiniPC.Init(&MiniPC_USB_Manage_Object);
    // MiniPC.Referee = &Referee;
}


bool Class_Chariot::Calibrate()
{
    static bool Calibration_Motor_Yaw_Flag = false;
    static bool Calibration_Motor_Up_Flag = false;

    if(!Calibration_Motor_Yaw_Flag)
    {
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Yaw.Set_Target_Omega_Radian(Calibration_Motor_Yaw_Target_Omega_Angle);
        if(Motor_Yaw.Get_Now_Torque()>Calibration_Motor_Yaw_Troque_Threshold)
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Calibration_Motor_Yaw_Angle_Offset = Motor_Yaw.Get_Now_Angle(); // 记录上电校准后电机初始offset角度 角度制
            Motor_Yaw.Set_Target_Radian(Calibration_Motor_Yaw_Angle_Offset*DEG_TO_RAD); // 电机初始位置
            Calibration_Motor_Yaw_Flag = true; // 退出校准
        }
    }

    if(!Calibration_Motor_Up_Flag)
    {
        Motor_Up.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Up.Set_Target_Omega_Radian(Calibration_Motor_Up_Target_Omega_Radian);
        if(Motor_Up.Get_Now_Torque()>Calibration_Motor_Up_Torque_Threshold)
        {
            Motor_Up.Set_Target_Radian(0);  //校准结束就停下来
            Calibration_Motor_Up_Radian_Offset = Motor_Up.Get_Now_Radian(); // 记录上电校准后电机初始offset角度 弧度制
            Calibration_Motor_Up_Flag = true; // 退出校准
        }
    }

    Calibration_Finish = Calibration_Motor_Yaw_Flag && Calibration_Motor_Up_Flag;

    return  Calibration_Finish;
}


void Class_Chariot::Updata_Distance_Angle()
{
    Now_Distance_Motor_Up = (Motor_Up.Get_Now_Radian()-Calibration_Motor_Up_Radian_Offset) * Radian_To_Diatance;
    Now_Angle_Yaw = (Motor_Yaw.Get_Now_Angle()-Calibration_Motor_Yaw_Angle_Offset) / Reduction_Ratio;
}

void Class_Chariot::Servo_Reload()
{
    Servo_Load_1.Set_Target_Angle(0);
    Servo_Load_2.Set_Target_Angle(0);
    Servo_Load_3.Set_Target_Angle(0);
    Servo_Load_4.Set_Target_Angle(0);
}

void Class_Chariot::Servo_Init()
{
    Servo_Load_1.Set_Target_Angle(0);
    Servo_Load_2.Set_Target_Angle(0);
    Servo_Load_3.Set_Target_Angle(0);
    Servo_Load_4.Set_Target_Angle(0);
}

/**
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    // yaw电机 角度环
    Motor_Yaw.Set_Now_Angle(Now_Angle_Yaw);
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    // 推镖电机 速度环
    Motor_Up.TIM_PID_PeriodElapsedCallback();
    // 拉皮筋电机 拉力环->速度环
    PID_Tension.Set_Now(Tension_Meter.Get_Tension_Meter());
    PID_Tension.TIM_Adjust_PeriodElapsedCallback(); 
    Motor_Down.Set_Target_Omega_Radian(PID_Tension.Get_Out());  
    Motor_Down.TIM_PID_PeriodElapsedCallback();
    // 上膛电机 速度环
    Motor_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Right.TIM_PID_PeriodElapsedCallback();
}
/**
 * @brief 控制回调函数
 *
 */
void Class_Chariot::TIM_Control_Callback()
{

}
/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM5msMod10_Alive_PeriodElapsedCallback()
{
    static uint8_t mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        Motor_Yaw.TIM_Alive_PeriodElapsedCallback();  
        Motor_Up.TIM_Alive_PeriodElapsedCallback();       
        Motor_Down.TIM_Alive_PeriodElapsedCallback();
        Motor_Left.TIM_Alive_PeriodElapsedCallback();
        Motor_Right.TIM_Alive_PeriodElapsedCallback();
        Tension_Meter.TIM_Alive_PeriodElapsedCallback();
        //MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
        //Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        mod10 = 0;
    }    
}


void Class_FSM_Dart_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
        case Dart_Init_Status:
        {
            // 初始化完成 进入准备状态
            if(Chariot->Calibrate())
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(Dart_Ready_Status);
            }
        }
        break;
        case Dart_Ready_Status:
        {

            // debubg模式
            if(Chariot->DebugControl.Debug_Start_Flag)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(Dart_Debug_Status);
            }
        }
        break;
        case Dart_First_Status:
        {
            Chariot->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            // 3508上膛
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Chariot->Motor_Left.Set_Target_Radian(Chariot->Target_Speed_Motor_Left);
            // 已经上膛 失能3508
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Left.Set_Out(0);
            // 2006电机控制拉力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Chariot->PID_Tension.Set_Target(Chariot->Target_Tension);
            // 达到目标拉力 2006失能无力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Down.Set_Out(0);
            // 扳机舵机发射
            Chariot->Servo_Trigger.Set_Target_Angle(Chariot->Shoot_Angle_Trigger);
        }
        break;
        case Dart_Second_Status:
        {
            // 装弹
            Chariot->Servo_Reload();
            // 3508上膛
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Chariot->Motor_Left.Set_Target_Radian(Chariot->Target_Speed_Motor_Left);
            // 失能3508 恢复舵机位置
            Chariot->Servo_Init();
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Left.Set_Out(0);
            // 2006电机控制拉力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Chariot->PID_Tension.Set_Target(Chariot->Target_Tension);
            // 达到目标拉力 2006失能无力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Down.Set_Out(0);
            // 扳机舵机发射
            Chariot->Servo_Trigger.Set_Target_Angle(Chariot->Shoot_Angle_Trigger);
        }
        break;
        case Dart_Third_Status:
        {
            // 2006推弹
            Chariot->Motor_Up.Set_Target_Radian(Chariot->Target_Speed_Motor_Up);
            // 舵机装弹
            Chariot->Servo_Reload();
            // 3508上膛
            Chariot->Motor_Left.Set_Target_Radian(Chariot->Target_Speed_Motor_Left);
            // 失能3508 恢复舵机位置
            Chariot->Servo_Init();
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Left.Set_Out(0);
            // 2006电机控制拉力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Chariot->PID_Tension.Set_Target(Chariot->Target_Tension);
            // 达到目标拉力 2006失能无力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Down.Set_Out(0);
            // 扳机舵机发射
            Chariot->Servo_Trigger.Set_Target_Angle(Chariot->Shoot_Angle_Trigger);
        }
        break;
        case Dart_Fourth_Status:
        {
            // 2006推弹
            Chariot->Motor_Up.Set_Target_Radian(Chariot->Target_Speed_Motor_Up);
            // 舵机装弹
            Chariot->Servo_Reload();
            // 3508上膛
            Chariot->Motor_Left.Set_Target_Radian(Chariot->Target_Speed_Motor_Left);
            // 失能3508 恢复舵机位置
            Chariot->Servo_Init();
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Left.Set_Out(0);
            // 2006电机控制拉力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Chariot->PID_Tension.Set_Target(Chariot->Target_Tension);
            // 达到目标拉力 2006失能无力
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Down.Set_Out(0);
            // 扳机舵机发射
            Chariot->Servo_Trigger.Set_Target_Angle(Chariot->Shoot_Angle_Trigger);
        }
        break;
        case Dart_Debug_Status:
        {
            // 调试模式
            Chariot->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Chariot->Motor_Yaw.PID_Angle.Set_Target(Chariot->DebugControl.Get_Target_Yaw());
            Chariot->Motor_Yaw.PID_Angle.TIM_Adjust_PeriodElapsedCallback();
            Chariot->Motor_Yaw.PID_Omega.Set_Target(Chariot->Motor_Yaw.PID_Angle.Get_Out());
            Chariot->Motor_Yaw.PID_Omega.TIM_Adjust_PeriodElapsedCallback();
            Chariot->Motor_Yaw.Set_Out(Chariot->Motor_Yaw.PID_Omega.Get_Out());

            // 根据按下的按键进行不同的操作
            switch (Chariot->DebugControl.Get_DebugControl_Status())
            {
                case DebugControl_Control_Status_RELOAD:
                {
                    Chariot->Servo_Reload();
                    // 如果触发了微动开关或者延时时间到了
                    Chariot->Servo_Init();                    
                }break;
                case DebugControl_Control_Status_SHOOT:
                {
                    Chariot->Servo_Trigger.Set_Target_Angle(Chariot->Shoot_Angle_Trigger);
                }break;
                case DebugControl_Control_Status_TENSION:
                {
                    // 3508上膛
                    Chariot->Motor_Left.Set_Target_Radian(Chariot->Target_Speed_Motor_Left);
                    Chariot->Motor_Right.Set_Target_Radian(Chariot->Target_Speed_Motor_Right);
                    // 失能3508 恢复舵机位置
                    Chariot->Servo_Init();
                    Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
                    Chariot->Motor_Left.Set_Out(0);
                    // 2006电机控制拉力
                    Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Chariot->PID_Tension.Set_Target(Chariot->Target_Tension);
                    // 达到目标拉力 2006失能无力
                    Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
                    Chariot->Motor_Down.Set_Out(0);
                }break;
                default:
                break;
            }
        }
        break;
        case Dart_Disable_Status:
        {
            // 失能模式有待商榷
            Chariot->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Up.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Chariot->Motor_Yaw.Set_Out(0);
            Chariot->Motor_Down.Set_Out(0);
            Chariot->Motor_Right.Set_Out(0);
            Chariot->Motor_Up.Set_Out(0);
            Chariot->Motor_Left.Set_Out(0);
        }
        break;
        default:
        break;
    }
}

/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
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
            //离线保护

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Set_Status(2);
            }

            //遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
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
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
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
            HAL_UART_DMAStop(&huart1); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
