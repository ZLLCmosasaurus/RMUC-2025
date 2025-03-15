#ifndef CRT_OBSERVE_H_
#define CRT_OBSERVE_H_

#include "main.h"

#include "stdint.h"
#include "alg_vmc.h"
#include "dvc_dmmotor.h"
#include "dvc_AKmotor.h"
#include "dvc_imu.h"

typedef struct
{

	Class_DM_Motor_J4310 motor;
	float Joint_T;

} Joint_Motor_T;

typedef struct
{

	Class_AK_Motor_80_6 motor;
	float wheel_T;

} Wheel_Motor_T;

float RAMP_float(float final, float now, float ramp);
class Class_observe
{
public:
	KalmanFilter_t vaEstimateKF;
	float wr, wl;
	float vrb, vlb;
	float aver_v;
	Class_VMC *Left_Leg;
	Class_VMC *Right_Leg;
	Class_IMU *Boardc_BMI;
	Joint_Motor_T *Joint_Motor[4];
	Wheel_Motor_T *Wheel_Motor[2];
	float vel_acc[2];
	float v_filter;
	float x_filter;
	void TIM_Calculate_PeriodElapsedCallback(float OBSERVE_TIME);
	void xvEstimateKF_Init(void);
	void xvEstimateKF_Update(float acc, float vel);
	inline float Get_X_Filter(void);
	inline float Get_V_Filter(void);
};

float Class_observe::Get_X_Filter(void)
{
	return x_filter;
}

float Class_observe::Get_V_Filter(void)
{
	return v_filter;
}

#endif