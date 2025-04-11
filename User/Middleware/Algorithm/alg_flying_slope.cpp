
#include "alg_flying_slope.h"


/**
 * @brief 轮序0123对应车子从左上第一个轮子顺时针排序
 * @param __Motor_Wheel 
 */
void Class_Flying_Slope::Init(Class_DJI_Motor_C620 *__Motor_Wheel0, Class_DJI_Motor_C620 *__Motor_Wheel1,
                            Class_DJI_Motor_C620 *__Motor_Wheel2, Class_DJI_Motor_C620 *__Motor_Wheel3)
{
  Motor_Wheel[0] = __Motor_Wheel0;
  Motor_Wheel[1] = __Motor_Wheel1;
  Motor_Wheel[2] = __Motor_Wheel2;
  Motor_Wheel[3] = __Motor_Wheel3;
}

/**
 * @brief 按照底盘c板更改
 */
void Class_Flying_Slope::Transform_Angle()
{
  Slope_Angle = IMU->Get_Angle_Pitch();
  Roll_Angle = IMU->Get_Angle_Roll();
}

/**
 * @brief 计算需要补偿的前馈值
 */
void Class_Flying_Slope::TIM_Calcualte_Feekback()
{
  float N1 = 0.0f, N2 = 0.0f;                            //1后轮，2前轮轮组每个轮子分到的支持力

  //Math_Constrain(*Slope_Angle, -23.0f, 23.0f);

  if(fabs(Roll_Angle) > 5){                         //车体姿态不正常
    Feekback_Value[0] = Feekback_Value[1] = 0;     //前轮
    Feekback_Value[2] = Feekback_Value[3] = 0;     //后轮

    return;
  }

  float Delta_L = CHASSIS_R * tan(Slope_Angle);

  //避免轮组投影点与重力斜坡投影点太近，输出大容易打滑
  if(Delta_L * 2/CHASSIS_L > 0.2377f){           //30度，麦轮
    Delta_L = CHASSIS_R * tan(30);
  }
  else if(Delta_L * 2/CHASSIS_L < -0.2377f){
    Delta_L = -CHASSIS_R * tan(30);
  }

  float L1 = CHASSIS_L/2 - Delta_L;       //后轮轮组到重心投影点
  float L2 = CHASSIS_L/2 + Delta_L;
  
  N1 = L2 * Gravity * sin(Slope_Angle)/CHASSIS_L;
  N2 = L1 * Gravity * sin(Slope_Angle)/CHASSIS_L;

  Feekback_Value[0] = Feekback_Value[1] = N2 * M3508_CMD_CURRENT_TO_TORQUE;     //前轮
  Feekback_Value[2] = Feekback_Value[3] = N1 * M3508_CMD_CURRENT_TO_TORQUE;     //后轮

}

/**
 * @brief 输出补偿后的前馈值
 */
void Class_Flying_Slope::Output(){
  for(int i = 0; i<4; i++){

    float Out = Feekback_Value[i] + Motor_Wheel[i]->Get_Out();

    Math_Constrain(&Out, -(float)Motor_Wheel[i]->Get_Output_Max(), (float)Motor_Wheel[i]->Get_Output_Max());
    if(Out > Motor_Wheel[i]->Get_Output_Max()){
      Out = Motor_Wheel[i]->Get_Output_Max();
    }
    else if(Out < -Motor_Wheel[i]->Get_Output_Max()){
      Out = -Motor_Wheel[i]->Get_Output_Max();
    }

    Motor_Wheel[i]->Set_Out(Out);
    Motor_Wheel[i]->Output();
  }
}