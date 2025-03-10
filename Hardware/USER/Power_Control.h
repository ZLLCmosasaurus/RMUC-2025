#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "PID.h"
#include "Measure.h"
#include "math.h"

int Power_on_Self_Test();
void  PID_Control(void);
void Input_undervoltage_protection(void);
void Mode_Judgment(void);
void HRTIM_DISABLE();
void HRTIM_ENABLE();
void Update_PWM(float ratio);
void Recovery_detect();
double map_input_to_output(double input_value) ;
#define MAX_DUTY 1.2    //DUTY=V_CAP/V_VIN
#define MIN_DUTY 0.1
#define MAX_P_DCDC 120  //充电
#define MIN_P_DCDC -300 //放电

#define HRTIMMaster_Period 8000
#define FALASE 0
#define TRUE 1
#define V_SET 24.3f
#define BUCK_RIGHT_DUTY   (0.97 * HRTIMMaster_Period+50)    	  //0.9*5760+50=5234                     Buck模式下，右桥固定占空比90%
#define BUCK_LEFT_MIN_DUTY		(0.10 * HRTIMMaster_Period+50)  //0.1*5760+50=626                        Buck模式下，左桥最小占空比10%
#define BUCK_LEFT_MAX_DUTY   (0.97 * HRTIMMaster_Period+50) //0.9*5760+50=5234  			                 Buck模式下，左桥最大占空比85%

#define	BUCK_BOOST_LEFT_DUTY  (0.97 * HRTIMMaster_Period+50)   			 //0.9*5760+50=5234                 BUCK-BOOST模式下，左桥固定占空比90%，给太大放不了电
#define BUCK_BOOST_RIGHT_MIN_DUTY  (0.70 * HRTIMMaster_Period+50)    //0.7*5760+50=4082                 BUCK-BOOST模式下，右桥最小占空比50%，给太大充不了电
#define BUCK_BOOST_RIGHT_MAX_DUTY  (0.97 * HRTIMMaster_Period+50)    //0.9*5760+50=5234                 BUCK-BOOST模式下，右桥最大占空比90%
#define MIN_REG_VALUE   25                     //HRTIM reg mini value

#define MIN_UVP_VAL    20.0f//20V欠压保护
#define MIN_UVP_VAL_RE 21.0f//21V欠压保护恢复
#define MAX_UVP_VAL 28.0f//28V过压保护
#define VIN_I_MAX 6.0f//10A过流保护
#define CAP_I_MAX 10.0f

extern float ratio_test;
 extern int PWM_CONTROL;
enum LOOP_MODE
{
    LOOP_MODE_CV = 1,                                       //恒压模式
    LOOP_MODE_CC,                                            //恒流模式
    DISCHANGE
};

enum CAP_MODE
{
    Normal=1,                                    //BUCK模态
		Standby,                                     //故障或待机
    LISTEN

};


enum ERROR_CODE
{
  NO=0,
  VOLT,
 CURRENT,
  LOSE      
 
};
typedef struct {
  pid_type_def powerin_loop, currout_loop, voltout_loop;  //pid结构体
  float P_set;                                        //设定电管输出功率
	float I_Set;																						//电管输出电流
  float dcdc_power;                                       //DC-DC设定功率
  float I_Charge_limited;                                        //DC-DC设定电流
  float dcdc_max_curr;                                    //dcdc最大电流
  float cap_v_max;                                        //电容最大电压
  float cap_max_curr;                                     //电容最大电流
  float Vloop_ratio;                                      //电压环输出
  float Cloop_ratio;                                      //电流环输出
  float Volt_ratio;                                       //PID计算最终输出电压比值 (Vout / Vin)
  float Real_ratio_forward;                                         
  float Real_ratio;                                        //实际ADC测量值
	enum	CAP_MODE Cap_Mode;																//超电状态
	enum	LOOP_MODE Loop_Mode;															//PID环状态
  enum ERROR_CODE error_code;                             //错误码
  float test_ratio;
  float DutyA;
  float DutyB;
	int buck_left_feedforward_duty;																			//BUCK左桥前馈占空比
	int buck_boost_right_feedforward_duty;																			//BUCK-BOOST右桥前馈占空比	
	int left_feedforward_duty;                              
  int right_feedforward_duty;
	uint8_t BBModeChange;   																//工作模式切换标志位
	uint8_t flag;
	int left_duty;
  int right_duty;
	float I_DCDC_IN_MAX;																				//H桥左侧最大输入电流
	float I_CAP_IN_MAX;																					//H桥右侧最大输入电流
	float I_DCDC_OUT_MAX;																				//H桥左侧输出最大电流
	float I_CAP_OUT_MAX;																				//H桥右侧输出最大电流
  uint8_t power_limit;                                        //电控功率限制开关  
  float recovery_decrease_ratio;
} control_struct_t;

extern control_struct_t control;
#define M_MIN(a, b) ((a) > (b) ? (a) : (b)) 
#define M_MAX(a, b) ((a) < (b) ? (a) : (b))
#ifdef __cplusplus
}
#endif

#endif
