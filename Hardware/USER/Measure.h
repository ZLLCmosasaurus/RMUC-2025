#ifndef __MEASURE_H
#define __MEASURE_H
#include "Power_Control.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define V_Magnification	16						//电阻分压倍数
#define I_Magnification 20                       //电流检测芯片放大倍数
#define VIN_Sense_resistor 0.004			   //电流检测阻值（单位为欧）
#define MOTOR_Sense_resistor 0.004
#define CAP_Sense_resistor 0.004
#define Measure_count 12
void ADC_init(void);
void ADC_Measure(void);
void ADC_VREF(void);
void ratio_init(void);
extern uint16_t LED_Cnt;

typedef struct {
    int measured_value;
    float measured_array[Measure_count];
    int bias;
} STRUCT_REPLACE;

typedef struct
{	
	STRUCT_REPLACE replace;
	float median_value;
	double real_valu1;      //真实值1 外部仪表获得
	double get_volt1;       //测量值1  单片机滤波后获得
	double real_valu2;      //真实值2
	double get_volt2;       //测量值2
	
	double real_valu3;      //真实值1 外部仪表获得
	double get_volt3;       //测量值1  单片机滤波后获得
	double real_valu4;      //真实值2
	double get_volt4;       //测量值2
	
	float offset;          //漂移系数
	float ratio;           //比例系数
	
	int16_t measured_value; //原始ADC值
	int16_t bias;							//原始偏移量
	float filter_out;						//环形数组处理后的值
	float Solved_value;						//解算出来的电压和电流
	int16_t measured_array[20];
	float Solved_filter_out;
	
} ELEC_INFO_STRUCT;

extern ELEC_INFO_STRUCT ADC_I_IN;
extern ELEC_INFO_STRUCT ADC_VIN;
extern ELEC_INFO_STRUCT ADC_I_MOTOR;
extern ELEC_INFO_STRUCT ADC_I_CAP;
extern ELEC_INFO_STRUCT ADC_V_CAP;
extern uint16_t AD_Buf_1[3];
extern uint16_t AD_Buf_2[3];


float ringbuf_cal(int16_t *adc_ch , int16_t wide);
void ratio_init(void);
void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, 
																											double real2, double get2);
float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p,uint16_t cn_kalman);//ADC值变成电压电流
void ADC_Measure(void);
void ADC_init(void);
void sumBuffer();
extern uint8_t board_number;



typedef struct 
	{
		
		float P_In;	
		float P_Motor;
		float P_Cap;
		float V_DCDC;
		float I_DCDC;
		float P_DCDC;
		float efficiency;
		float surplus_energy;
		uint16_t V_REFINT_CAL;
		float p_beyondl;			//超功率值
	} measure_struct_t;
extern measure_struct_t measure; 

typedef struct 
	{
	
		uint16_t Internally_read_values;	
		uint32_t Externally_read_values_total;
		float Average_value;
		uint16_t Externally_read_values;
		
	} ADC_V_REF_struct_t;
extern ADC_V_REF_struct_t ADC_V_REF; 


typedef struct
{
	float  Input;
	float  Output;
	float  Fc;
	float  Fs;
	float  Ka;
	float  Kb;
}LOWPASS_FILTER_STRUCT;//一阶低通滤波

#define PI2   6.28318530F // 2π
void low_filter_init(LOWPASS_FILTER_STRUCT *p, float sample_f, float cutoff_f);
void low_filter_calc(LOWPASS_FILTER_STRUCT *p);	//一阶低通滤波计算
void replace_extremes_with_average(int16_t *array, int size); 
#ifdef __cplusplus
}
#endif

#endif
