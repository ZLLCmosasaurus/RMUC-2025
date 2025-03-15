#include "alg_vmc.h"

void Class_VMC::VMC_Init(void)
{
    l1 = 0.15f;
    l2 = 0.27f;
    l3 = 0.27f;
    l4 = 0.15f;
    l5 = 0.15f;
    D_Theta_Filter.Init(-1000, 1000, Filter_Fourier_Type_LOWPASS, 5, 0, 200, 3);
}
//计算theta和d_theta给lqr用，同时也计算腿长L0 
void Class_VMC::VMC_calc_1(float dt)
{

    // 将原始的俯仰角和陀螺仪数据赋值给成员变量
    Pitch = Raw_Pitch;
    PithGyro = Raw_Gyro;

    Y_D = l4 * arm_sin_f32(phi4);      // D的y坐标
    Y_B = l1 * arm_sin_f32(phi1);      // B的y坐标
    X_D = l5 + l4 * arm_cos_f32(phi4); // D的x坐标
    X_B = l1 * arm_cos_f32(phi1);      // B的x坐标

    L_BD = sqrt((X_D - X_B) * (X_D - X_B) + (Y_D - Y_B) * (Y_D - Y_B));

    A0 = 2 * l2 * (X_D - X_B);
    B0 = 2 * l2 * (Y_D - Y_B);
    C0 = l2 * l2 + L_BD * L_BD - l3 * l3;
    phi2 = 2 * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
    phi3 = atan2f(Y_B - Y_D + l2 * arm_sin_f32(phi2), X_B - X_D + l2 * arm_cos_f32(phi2));
    // C点直角坐标
    X_C = l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(phi2);
    Y_C = l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(phi2);
    // C点极坐标
    L0 = sqrt((X_C - l5 / 2.0f) * (X_C - l5 / 2.0f) + Y_C * Y_C);

    phi0 = atan2f(Y_C, (X_C - l5 / 2.0f)); // phi0用于计算lqr需要的theta
    if (phi0 <= PI / 2.0f)
    {
        alpha = PI / 2.0f - phi0;
    }
    else if (phi0 > PI / 2.0f)
    {
        alpha = PI / 2.0f * 5.0f - phi0;
    }

    if (first_flag == 0)
    {
        last_phi0 = phi0;
        first_flag = 1;
    }
    raw_d_phi0 = (phi0 - last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    D_Theta_Filter.Set_Now(raw_d_phi0);
    D_Theta_Filter.TIM_Adjust_PeriodElapsedCallback();
    
    d_phi0 = D_Theta_Filter.Get_Out();

    d_alpha = 0.0f - d_phi0;

    theta = PI / 2.0f - Pitch - phi0; // 得到状态变量1
    d_theta = (-PithGyro - d_phi0);    // 得到状态变量2

    last_phi0 = phi0;

    d_L0 = (L0 - last_L0) / dt;      // 腿长L0的一阶导数
    dd_L0 = (d_L0 - last_d_L0) / dt; // 腿长L0的二阶导数

    last_d_L0 = d_L0;
    last_L0 = L0;

    dd_theta = (d_theta - last_d_theta) / dt;
    last_d_theta = d_theta;
}

void Class_VMC::Set_LQR_K(float Length_L[])
{

    for (size_t i = 0; i < 12; i++)
    {
        LQR_K[i] = Length_L[i];
    }
}

void Class_VMC::VMC_calc_2(void)
{
    j11 = (l1 * arm_sin_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2)) / arm_sin_f32(phi3 - phi2);
    j12 = (l1 * arm_cos_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2)) / (L0 * arm_sin_f32(phi3 - phi2));
    j21 = (l4 * arm_sin_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4)) / arm_sin_f32(phi3 - phi2);
    j22 = (l4 * arm_cos_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4)) / (L0 * arm_sin_f32(phi3 - phi2));

    torque_set[0] = j11 * F0 + j12 * Tp;
    torque_set[1] = j21 * F0 + j22 * Tp;
}

uint8_t Class_VMC::ground_detection()
{
    // 计算当前的支持力 FN
    FN = F0 * arm_cos_f32(theta) + Tp * arm_sin_f32(theta) / L0 + 13.23f;
    // FN = F0 * arm_cos_f32(theta) + Tp * arm_sin_f32(theta) / L0
    //     + 0.6f * (MotionAccel_n - dd_L0 * arm_cos_f32(theta) + 2.0f * d_L0 * d_theta * arm_sin_f32(theta)
    //               + L0 * dd_theta * arm_sin_f32(theta) + L0 * d_theta * d_theta * arm_cos_f32(theta));

    // 更新平均值数组（FIFO队列的形式）
    aver[0] = aver[1];
    aver[1] = aver[2];
    aver[2] = aver[3];
    aver[3] = FN;

    // 对支持力进行均值滤波
    aver_fn = 0.25f * (aver[0] + aver[1] + aver[2] + aver[3]);

    // 根据滤波后的支持力判断是否离地
    if (aver_fn < 13.0f)
    { // 离地
        return 1;
    }
    else
    { // 没有离地
        return 0;
    }
}

float Class_VMC::LQR_K_calc(float *coe, float len)
{
    //由于计算出的系数呈降幂排列，故与开源公式不同
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}
