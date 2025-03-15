#ifndef ALG_VMC_H_
#define ALG_VMC_H_

#include "main.h"
#include "arm_math.h"

#include "alg_filter.h"
#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679

class Class_VMC
{
public:
    /*左右两腿公共参数，固定不变，单位：m*/
    float l1;
    float l2;
    float l3;
    float l4;
    float l5;
    /*直角坐标系坐标*/
    float X_B, Y_B;
    float X_D, Y_D;
    float X_C, Y_C;
    /*极坐标系坐标*/
    float L0, phi0;

    float alpha;
    float d_alpha;

    float L_BD;
float raw_d_phi0;
    float d_phi0;
    float last_phi0;

    float A0, B0, C0;
    float phi2, phi3;
    float phi1, phi4;

    float j11, j12, j21, j22;
    float torque_set[2];

    float Raw_Pitch;
    float Raw_Gyro;

    float F0;
    float Tp;
    float F02;

    float theta;
    float d_theta;
    float last_d_theta;
    float dd_theta;

    float d_L0;
    float dd_L0;
    float last_L0;
    float last_d_L0;

    float FN;

    uint8_t first_flag;
    uint8_t leg_flag;

    float MotionAccel_n;
    float Pitch;
    float PithGyro;

    Class_Filter_Fourier D_Theta_Filter;
    // 下面是测试变量。

    float tp_theta_out;
    float tp_dtheta_out;
    float tp_x_out;
    float tp_v_out;
    float tp_myPith_out;
    float tp_myPithGyro_out;

    float t_theta_out;
    float t_dtheta_out;
    float t_x_out;
    float t_v_out;
    float t_myPith_out;
    float t_myPithGyro_out;

    float aver[4];
    float aver_fn;
    float LQR_K[12];

    void VMC_Init();
    void VMC_calc_1(float dt);
    void VMC_calc_2(void);
    void Set_LQR_K(float Length_L[]);
    uint8_t ground_detection();
    float LQR_K_calc(float *coe, float len);

    inline void Set_Pitch(float pitch);
    inline void Set_Pitch_Gyro(float gyro);
    inline void Set_MotionAccel(float accel);

    inline float Get_Pitch(void);
    inline float Get_Pitch_Gyro(void);
    inline float Get_MotionAccel(void);
};

void Class_VMC ::Set_Pitch(float pitch)
{
    Raw_Pitch = pitch;
}

void Class_VMC ::Set_Pitch_Gyro(float gyro)
{
    Raw_Gyro = gyro;
}

void Class_VMC ::Set_MotionAccel(float accel)
{
    MotionAccel_n = accel;
}

float Class_VMC ::Get_Pitch(void)
{
    return Pitch;
}

float Class_VMC ::Get_Pitch_Gyro(void)
{
    return PithGyro;
}

float Class_VMC ::Get_MotionAccel(void)
{
    return MotionAccel_n;
}
#endif