
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
  Slope_Angle = -1 * IMU->Get_Angle_Roll();
  Roll_Angle  =      IMU->Get_Angle_Pitch();

  Slope_Radian = Slope_Angle * PI/180.0f;
}

/**
 * @brief 计算需要补偿的前馈值
 */

void Class_Flying_Slope::TIM_Calcualte_Feekback()
{
  float F1 = 0.0f, F2 = 0.0f;                            //1后轮，2前轮轮组，每个轮子需要补偿的摩擦力
  float Delta_L = 0.0f;
  float L1 =0.0f ,L2 = 0.0f;

  //Math_Constrain(*Slope_Angle, -23.0f, 23.0f);
  if(fabs(Roll_Angle) > 5 || fabs(Slope_Angle) < 3){                         //车体姿态不正常
    Feekback_Value[0] = Feekback_Value[1] = 0;     //前轮
    Feekback_Value[2] = Feekback_Value[3] = 0;     //后轮

    return;
  }

  Delta_L = CHASSIS_R * tan(Slope_Radian);

  //避免轮组投影点与重力斜坡投影点太近，输出大容易打滑
  if(Delta_L * 2/CHASSIS_L > 0.25f){           //30度，麦轮
    Delta_L = CHASSIS_R * tan(30 * PI /180);
    Slope_Radian = 30 * PI / 180.0f;
  }
  else if(Delta_L * 2/CHASSIS_L < -0.25f){
    Delta_L = -CHASSIS_R * tan(30 * PI /180);
    Slope_Radian = -30 * PI / 180.0f;
  }

  L1 = CHASSIS_L/2 - Delta_L;       //后轮轮组到重心投影点
  L2 = CHASSIS_L/2 + Delta_L;

  float N1 = Gravity * cos(Slope_Radian) * L2 / CHASSIS_L;
  float N2 = Gravity * cos(Slope_Radian) * L1 / CHASSIS_L;
  
  //物理上对质量分布的再分配，意义是进一步减小前轮输出，增大后轮输出，当N2 * 的数值大于1， 前轮输出的力矩制动，反向
  float Compensation_Offset = N2 * 0.4;            

  F1 = (N1 + Compensation_Offset) * tan(Slope_Radian) / 2.0f;           //后轮的单轮补偿
  F2 = (N2 - Compensation_Offset) * tan(Slope_Radian) / 2.0f;           //前轮的单轮补偿

  Feekback_Value[0] =      F1 * CHASSIS_R / M3508_CMD_CURRENT_TO_TORQUE;     //前轮
  Feekback_Value[1] = -1 * F1 * CHASSIS_R / M3508_CMD_CURRENT_TO_TORQUE;
  Feekback_Value[2] = -1 * F2 * CHASSIS_R / M3508_CMD_CURRENT_TO_TORQUE;     //后轮
  Feekback_Value[3] =      F2 * CHASSIS_R / M3508_CMD_CURRENT_TO_TORQUE;

}

/**
 * @brief 输出补偿后的前馈值
 */
void Class_Flying_Slope::Output(){
  for(int i = 0; i<4; i++){

    float Out = Feekback_Value[i] + Motor_Wheel[i]->Get_Out();

    Math_Constrain(&Out, -(float)Motor_Wheel[i]->Get_Output_Max(), (float)Motor_Wheel[i]->Get_Output_Max());

    Motor_Wheel[i]->Set_Out(Out);
    Motor_Wheel[i]->Output();
  }
}