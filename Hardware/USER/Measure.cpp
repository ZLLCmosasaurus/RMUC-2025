#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"                  // Device header
#include "Measure.h"
#include "adc.h"
#include "config.h"
#include "dvc_dwt.h"
#include "Communication.h"
uint16_t AD_Buf_1[3];
uint16_t AD_Buf_2[3];
ELEC_INFO_STRUCT ADC_I_IN;
ELEC_INFO_STRUCT ADC_VIN;
ELEC_INFO_STRUCT ADC_I_MOTOR;
ELEC_INFO_STRUCT ADC_I_CAP;
ELEC_INFO_STRUCT ADC_V_CAP;
uint16_t LED_Cnt;
ADC_V_REF_struct_t ADC_V_REF;
measure_struct_t measure; 
float V_Motor=0;
uint8_t ringbuf_cnt=0;

float ringbuf_cal(int16_t *adc_ch , int16_t wide)
{
	int32_t Total_Adc=0;
	for(int j=0;j<wide;j++)
	{
		Total_Adc+=adc_ch[j];
	}
	uint16_t adc_tmp;
	adc_tmp=Total_Adc/wide;
	return adc_tmp;
}

void ratio_init()
{

	ADC_I_IN.ratio = ADC_V_REF.Average_value/(I_Magnification*VIN_Sense_resistor);			//修改宏定义即可
	ADC_VIN.ratio = ADC_V_REF.Average_value*V_Magnification;
	ADC_I_MOTOR.ratio=ADC_V_REF.Average_value/(I_Magnification*MOTOR_Sense_resistor);
	ADC_I_CAP.ratio = -ADC_V_REF.Average_value/(I_Magnification*CAP_Sense_resistor);		//正值为充电，负值为放电
	ADC_V_CAP.ratio=ADC_V_REF.Average_value*V_Magnification;
	
	ADC_I_IN.bias = 2048;//1.5V的叠加值
	ADC_I_MOTOR.bias=2048;
	ADC_I_CAP.bias = 2048;
switch(Board_number)  			//线性校准
	{							//设置ID号在congfig.h中
	
		case 001:
		{
		ADC_Linear_calibration_init(&ADC_VIN,24.0,24.0,22.0,22.0);
		ADC_Linear_calibration_init(&ADC_I_IN,1.08,1.08,5.15,5.15);
		ADC_Linear_calibration_init(&ADC_I_MOTOR,5,5,1,1);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
		ADC_Linear_calibration_init(&ADC_V_CAP,21.9f,21.9f,16.33f,16.33f);
		break;
		}
		case 002:
		{
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}	
		case 003:
		{
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}
		case 004:
		{	
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}
		case 005:
		{	
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}		
	}
}
void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2) //线性校准运行一次
{
	double a, b;
	p->get_volt1 = get1;
	p->real_valu1 = real1;
	p->get_volt2 = get2;
	p->real_valu2 = real2;
	if ((p->get_volt2 - p->get_volt1 == 0) || (p->real_valu2 - p->real_valu1 == 0))
	{
		return;
	}

	a = (p->real_valu2 - p->real_valu1) / (p->get_volt2 - p->get_volt1);
	b = (p->real_valu1 - p->get_volt1 * a);

	p->ratio = a * p->ratio;
	p->offset = b;
}



float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p,uint16_t cn_kalman)//ADC值变成电压电流
{
	float a;
    p->measured_value=cn_kalman;
    a = (p->measured_value) * p->ratio + p->offset ;
	return a;
}

void ADC_VREF()
{	
	for(int i=0;i<200;i++)
	{
		ADC_V_REF.Externally_read_values_total+=AD_Buf_2[2];
	}
	ADC_V_REF.Internally_read_values = *(__IO uint32_t *)(0X1FFFF7BA);			//单片机内部电压读取地址
	ADC_V_REF.Average_value = 1.1120879f/ADC_V_REF.Internally_read_values;//内部读取读取电压对应4096量程数值
}

void ADC_Measure(void)
{	
	LED_Cnt++;
	sumBuffer();						//环形数组求和取平均
	ADC_I_IN.Solved_value=ADCvalue_to_ELEC(&ADC_I_IN,ADC_I_IN.filter_out);			//转化单片机读取值为实际值
	ADC_VIN.Solved_value=ADCvalue_to_ELEC(&ADC_VIN,ADC_VIN.filter_out);	
	ADC_I_MOTOR.Solved_value=ADCvalue_to_ELEC(&ADC_I_MOTOR,ADC_I_MOTOR.filter_out);
	ADC_V_CAP.Solved_value=ADCvalue_to_ELEC(&ADC_V_CAP,ADC_V_CAP.filter_out);
	ADC_I_CAP.Solved_value=ADCvalue_to_ELEC(&ADC_I_CAP,ADC_I_CAP.filter_out);
	
	//计算电压比
	control.Real_ratio=ADC_V_CAP.Solved_value/ADC_VIN.Solved_value;
	//计算电管输出功率
	measure.P_In=ADC_I_IN.Solved_value*ADC_VIN.Solved_value;
		
	//计算输入底盘功率
	V_Motor=ADC_VIN.Solved_value-2*ADC_I_MOTOR.Solved_value*MOTOR_Sense_resistor/I_Magnification;		//考虑检测电阻
	measure.P_Motor=ADC_I_MOTOR.Solved_value*V_Motor;
		
	//计算DC-DC输入/输出功率
	measure.V_DCDC = ADC_VIN.Solved_value;
	measure.I_DCDC = ADC_I_IN.Solved_value - ADC_I_MOTOR.Solved_value ;
	measure.P_DCDC = measure.V_DCDC*measure.I_DCDC ;
		
	//计算电容输入/输出功率
	measure.P_Cap=ADC_V_CAP.Solved_value*ADC_I_CAP.Solved_value;
	
	//计算DC-DC效率
	if(measure.I_DCDC>0)									//正向充电	
	{
		measure.efficiency=measure.P_Cap/measure.P_DCDC*100;
	}	
	else													//反向放电
	{
		measure.efficiency=measure.P_DCDC/measure.P_Cap*100;
	}
	//计算剩余能量百分比
	measure.surplus_energy=(ADC_V_CAP.Solved_value*ADC_V_CAP.Solved_value)/(23.85f*23.85f)*100;

		if(measure.surplus_energy>100)
		{
			measure.surplus_energy=100;
		}

	//计算目标DC-DC输入电流
	//P.set为裁判系统限制功率值
	control.I_Set = control.P_set / ADC_VIN.Solved_value ;
	control.I_Charge_limited = control.I_Set - ADC_I_MOTOR.Solved_value ;
	//限制目标DC-DC输入电流
	control.I_CAP_IN_MAX=10.0f;
	control.I_CAP_OUT_MAX=-8.0f;
	control.I_DCDC_IN_MAX = ADC_V_CAP.Solved_value*control.I_CAP_IN_MAX/measure.V_DCDC;
	control.I_DCDC_OUT_MAX = ADC_V_CAP.Solved_value*control.I_CAP_OUT_MAX/measure.V_DCDC;
   	if(control.I_Charge_limited>control.I_DCDC_IN_MAX)
	{
		control.I_Charge_limited=control.I_DCDC_IN_MAX;
	}

	if(control.I_Charge_limited<control.I_DCDC_OUT_MAX)                     
	{
		control.I_Charge_limited=control.I_DCDC_OUT_MAX;
	}

	if(measure.P_In-3.0f>control.P_set)				
	{												//抑制抖动重复发送
		measure.p_beyondl=measure.P_In-control.P_set;
		Send_Error();								//超功率发送错误信息
	}
	else
	{
		measure.p_beyondl=0;
	}
}	


void ADC_init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);//ADC软件校准
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);//ADC软件校准
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AD_Buf_1,3);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&AD_Buf_2,3);
	ADC_VREF();											  //ADC基准值读取
	ratio_init();										  //设置ADC值转换系数 
}

void sumBuffer()
{
	for (int i = 0; i < Measure_count; i++)
	{
	ADC_VIN.measured_value=AD_Buf_1[0];
	ADC_I_IN.measured_value=AD_Buf_1[1]-ADC_I_IN.bias;
	ADC_I_MOTOR.measured_value=AD_Buf_1[2]-ADC_I_MOTOR.bias;
	ADC_V_CAP.measured_value=AD_Buf_2[0];
	ADC_I_CAP.measured_value=AD_Buf_2[1]-ADC_I_CAP.bias;
	//ADC_V_REFINT.measured_value=AD_Buf_2[2];
	
	ADC_VIN.measured_array[ringbuf_cnt]=ADC_VIN.measured_value;
	ADC_I_IN.measured_array[ringbuf_cnt]=ADC_I_IN.measured_value;
	ADC_I_MOTOR.measured_array[ringbuf_cnt]=ADC_I_MOTOR.measured_value;
	ADC_V_CAP.measured_array[ringbuf_cnt]=ADC_V_CAP.measured_value;
	ADC_I_CAP.measured_array[ringbuf_cnt]=ADC_I_CAP.measured_value;
	ringbuf_cnt++;
	}
	ringbuf_cnt=0;
	replace_extremes_with_average(ADC_I_IN.measured_array,Measure_count);
	replace_extremes_with_average(ADC_I_MOTOR.measured_array,Measure_count);
	
	ADC_VIN.filter_out=ringbuf_cal(ADC_VIN.measured_array,Measure_count);
	ADC_I_IN.filter_out=ringbuf_cal(ADC_I_IN.measured_array,Measure_count);
	ADC_I_MOTOR.filter_out=ringbuf_cal(ADC_I_MOTOR.measured_array,Measure_count);
	ADC_V_CAP.filter_out=ringbuf_cal(ADC_V_CAP.measured_array,Measure_count);
	ADC_I_CAP.filter_out=ringbuf_cal(ADC_I_CAP.measured_array,Measure_count);
	
}

// 函数：替换数组中的最大值和最小值为平均值
void replace_extremes_with_average(int16_t *array, int size) 
{
    float sum = 0;
    float avg;
    int maxIndex = 0, minIndex = 0;

    // 计算总和
    for (int i = 0; i < size; i++) {
        sum += array[i];
    }

    // 计算平均值
    avg = sum / size;

    // 找出最大值和最小值的索引
    for (int i = 1; i < size; i++) {
        if (array[i] > array[maxIndex]) {
            maxIndex = i;
        }
        if (array[i] < array[minIndex]) {
            minIndex = i;
        }
    }

    // 替换最大值和最小值为平均值
    array[maxIndex] = avg;
    array[minIndex] = avg;
}
/*------一阶低通滤波-----------------------------------------------------*/
void low_filter_init(LOWPASS_FILTER_STRUCT *p, float sample_f, float cutoff_f)
{
  float Tc; //时间常数τ
  p->Fc = cutoff_f;
  p->Fs = sample_f;
  if (p->Fc <= 0.0f || p->Fs <= 0.0f)
  {
    p->Ka = 1.0f;
    p->Kb = 0.0f;
    return;
  }
  Tc = 1.0f / (PI2 * p->Fc);

  p->Ka = 1.0f / (1.0f + Tc * p->Fs);
  p->Kb = 1.0f - p->Ka;
  p->Input = 0.0f;
  p->Output = 0.0f;
}


void low_filter_calc(LOWPASS_FILTER_STRUCT *p)	//一阶低通滤波计算
{
  if (p->Output == p->Input)
    return;

  p->Output = p->Ka * p->Input + p->Kb * p->Output;
}
