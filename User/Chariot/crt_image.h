#ifndef CRT_IMAGE_H
#define CRT_IMAGE_H

#include "dvc_djimotor.h"
#include "dvc_lkmotor.h"
#include "alg_fsm.h"

class Class_Image;

enum Enum_Image_Control_Type :uint8_t
{
    Image_Control_Type_DISABLE = 0,
    Image_Control_Type_CALIBRATE,//校准模式
    Image_Control_Type_NORMAL,//正常模式
};
class Class_FSM_Image_Control : public Class_FSM
{
public:
    Class_Image *Image;
    void Reload_TIM_Status_PeriodElapsedCallback();
};


class Class_Image
{
public:
    //图传roll轴电机
    Class_DJI_Motor_C610 Motor_Image_Roll;
    //图传pitch轴电机
    Class_DJI_Motor_C610 Motor_Image_Pitch;
    //图传有限状态机
    Class_FSM_Image_Control FSM_Image_Control;
    friend class Class_FSM_Image_Control;
    //函数声明
    void Init();
    inline float Get_Target_Image_Roll_Angle();
    inline float Get_Target_Image_Pitch_Angle();
    inline float get_Image_Roll_Calibrate_Speed();
    inline float get_Image_Pitch_Calibrate_Offset();
    inline float get_Image_Roll_Calibrate_Offset();
    inline void Set_Target_Image_Roll_Angle(float __Target_Image_Roll_Angle);
    inline void Set_Target_Image_Pitch_Angle(float __Target_Image_Pitch_Angle);
    inline void Set_Image_Control_Type(Enum_Image_Control_Type __Image_Control_Type);
    inline void Set_Image_Roll_Calibrate_Offset(float __Image_Roll_Calibrate_Offset);
    inline void Set_Image_Pitch_Calibrate_Offset(float __Image_Pitch_Calibrate_Offset);
    inline void Set_Set_Image_Roll_Calibrate_Speed(float __Image_Roll_Calibrate_Speed);
    bool Motor_Calibration(Class_DJI_Motor_C610 *motor,float *Cali_Offset,float Cali_Omega,float Cali_Max_Out);
    void TIM_Calculate_PeriodElapsedCallback();
protected:

    //图传控制模式
    Enum_Image_Control_Type Image_Control_Type = Image_Control_Type_DISABLE;
     //图传roll轴角度
    float Target_Image_Roll_Angle = 0.0f;
    //图传pitch轴角度
    float Target_Image_Pitch_Angle = 0.0f;//待定
    //图传roll轴校准速度rad/s
    float Image_Roll_Calibrate_Speed = 3.5f;
    //图传pitch轴校准速度rad/s 
    float Image_Pitch_Calibrate_Speed = -5.0f;
    //图传roll轴校准完后相对位置
    float Image_Roll_Calibrate_Offset = 0.0f;
    //图传pitch轴校准完后相对位置
    float Image_Pitch_Calibrate_Offset = 0.0f;
    //图传roll轴校准临界力矩
    float Image_Roll_Calibrate_Stiffness = 510.0f;
    //图传pitch轴校准临界力矩
    float Image_Pitch_Calibrate_Stiffness = 4500.0f;
    //内部函数
    void Output();
};

/**
 * @brief 获取图传roll轴角度
 *
 */
float Class_Image::Get_Target_Image_Roll_Angle()
{
    return (Target_Image_Roll_Angle);
}
/**
 *   @brief 获取图传pitch轴角度
 *
 */
float Class_Image::Get_Target_Image_Pitch_Angle()
{
    return (Target_Image_Pitch_Angle);
}
/**
 * @brief 获取图传roll轴校准速度
 */
float Class_Image::get_Image_Roll_Calibrate_Speed()
{
    return (Image_Roll_Calibrate_Speed);
}
/**
 * @brief 获取图传roll轴校准完后相对位置
 * 
 */
float Class_Image::get_Image_Pitch_Calibrate_Offset()
{
    return (Image_Pitch_Calibrate_Offset);
}
/**
 * @brief 获取图传roll轴校准完后相对位置
 * 
 */
float Class_Image::get_Image_Roll_Calibrate_Offset()
{
    return (Image_Roll_Calibrate_Offset);
}
/**
 * @brief 设定图传roll轴角度
 */
void Class_Image::Set_Target_Image_Roll_Angle(float __Target_Image_Roll_Angle)
{
    Target_Image_Roll_Angle = __Target_Image_Roll_Angle;
}
/**
 * @brief 设定图传pitch轴角度
 * 
 */
void Class_Image::Set_Target_Image_Pitch_Angle(float __Target_Image_Pitch_Angle)
{
    Target_Image_Pitch_Angle = __Target_Image_Pitch_Angle;
}
/**
 * @brief 设定图传控制模式
 * 
 */
void Class_Image::Set_Image_Control_Type(Enum_Image_Control_Type __Image_Control_Type)
{
    Image_Control_Type = __Image_Control_Type;
}
/**
 * @brief 设定图传roll轴校准完后相对位置
 * 
 */
void Class_Image::Set_Image_Roll_Calibrate_Offset(float __Image_Roll_Calibrate_Offset)
{
    Image_Roll_Calibrate_Offset = __Image_Roll_Calibrate_Offset;
}

/**
 * @brief 设定图传roll轴校准完后相对位置
 * 
 */
void Class_Image::Set_Image_Pitch_Calibrate_Offset(float __Image_Pitch_Calibrate_Offset)
{
    Image_Pitch_Calibrate_Offset = __Image_Pitch_Calibrate_Offset;
}

/**
 * @brief 设定图传roll轴校准速度
 */
void Class_Image::Set_Set_Image_Roll_Calibrate_Speed(float __Image_Roll_Calibrate_Speed)
{
    Image_Roll_Calibrate_Speed = __Image_Roll_Calibrate_Speed;
}
#endif
