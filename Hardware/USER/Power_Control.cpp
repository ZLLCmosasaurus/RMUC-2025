#include "stm32f3xx_hal.h"
#include "Power_Control.h"
#include "PID.h"
#include "hrtim.h"
#include "TIMER.h"
#include "Buzzer.h"
#include "Communication.h"
control_struct_t control;                               //控制结构体

int flag_forward;
int recovery_cnt_V;
int recovery_cnt_I;
int recovery_flag_I;
int recovery_flag_V;
int recovery_flag;
float ratio_test;
int pwm_1;
int BUCK_BOOST_BBModeChange=0;
extern uint16_t CHA1,CHA2,CHB1,CHB2;
uint16_t Undervoltage_cnt=0;
uint16_t UVPCnt = 0;
uint16_t UVPCnt_I = 0;
uint16_t UVPCnt_Lose = 0;		//电管断电保护
uint16_t RSCnt = 0;
uint8_t PreBBFlag = 0;

void  PID_Control()
{	
	
	    if (control.BBModeChange)
    {
        PID_clear(&control.currout_loop);
        PID_clear(&control.voltout_loop);
        PID_clear(&control.powerin_loop);
		control.BBModeChange=0;
    }
    switch (control.Cap_Mode)
    {
    case Standby:   //故障
    {
		BUZZER(1000);
	/*	
	if(control.error_code==VOLT&&ADC_VIN.Solved_value>MAX_UVP_VAL)
		{
			control.Volt_ratio=ADC_V_CAP.Solved_value/24.0f;
		}
			*/
        break;

    }
    case Normal:  
    	{	
		BUZZER(0);
		control.Cloop_ratio=PID_calc(&control.currout_loop, measure.I_DCDC, control.I_Charge_limited );			//DCDC电流环
		control.Vloop_ratio=PID_calc(&control.voltout_loop, ADC_V_CAP.Solved_value, V_SET);						//电容电压环
		if(control.I_Charge_limited>0)//需要充电
	{	
		if(control.Real_ratio>0.9f)
		{
			if(control.Cloop_ratio<control.Vloop_ratio)					//选择电压环还是电流环，选择小的那一部分
				{
					control.Volt_ratio=control.Cloop_ratio;
					control.Loop_Mode=LOOP_MODE_CC;
				}
			else 
				{
					control.Volt_ratio=control.Vloop_ratio;
					control.Loop_Mode=LOOP_MODE_CV;
				}
		}											
		else
		{
			control.Volt_ratio=control.Cloop_ratio;
			control.Loop_Mode=LOOP_MODE_CC;
		}
			
	}
		else if (measure.P_In>measure.P_Cap)
	{	
		control.Volt_ratio=control.Cloop_ratio;
		control.Loop_Mode=DISCHANGE;		  		//电流环放电

		if(ADC_V_CAP.Solved_value<4.0f)
		{
		Send_Error();
		}
	}
	
		break;
    	}
    }
	control.Volt_ratio=M_MAX(control.Volt_ratio,1.2);			//限幅
	control.Volt_ratio=M_MIN(control.Volt_ratio,0);
	Update_PWM(control.Volt_ratio);
//	Update_PWM(control.test_ratio);
if(	PreBBFlag != control.Cap_Mode)
	{
		if(PreBBFlag==Standby)
		{	
			HRTIM_ENABLE();
		}
		else
		{
			HRTIM_DISABLE();
		}
	}
}

int  Power_on_Self_Test()
{
	control.P_set=45; 
	for(int i=0;i<8;i++)
	{
	ADC_VIN.measured_array[i]=AD_Buf_1[0];
	ADC_I_IN.measured_array[i]=AD_Buf_1[1]-ADC_I_IN.bias;
	ADC_I_MOTOR.measured_array[i]=AD_Buf_1[2]-ADC_I_MOTOR.bias;
		
	ADC_V_CAP.measured_array[i]=AD_Buf_2[0];
	ADC_I_CAP.measured_array[i]=AD_Buf_2[1]-ADC_I_CAP.bias;
	}
	ADC_VIN.filter_out=ringbuf_cal(ADC_VIN.measured_array,8);
	ADC_I_IN.filter_out=ringbuf_cal(ADC_I_IN.measured_array,8);
	ADC_I_MOTOR.filter_out=ringbuf_cal(ADC_I_MOTOR.measured_array,8);
	ADC_V_CAP.filter_out=ringbuf_cal(ADC_V_CAP.measured_array,8);
	ADC_I_CAP.filter_out=ringbuf_cal(ADC_I_CAP.measured_array,8);
	
	ADC_VIN.Solved_filter_out=ADCvalue_to_ELEC(&ADC_VIN,ADC_VIN.filter_out);
	ADC_I_IN.Solved_filter_out=ADCvalue_to_ELEC(&ADC_I_IN,ADC_I_IN.filter_out);
	ADC_I_MOTOR.Solved_filter_out=ADCvalue_to_ELEC(&ADC_I_MOTOR,ADC_I_MOTOR.filter_out);
	ADC_V_CAP.Solved_filter_out=ADCvalue_to_ELEC(&ADC_V_CAP,ADC_V_CAP.filter_out);
	ADC_I_CAP.Solved_filter_out=ADCvalue_to_ELEC(&ADC_I_CAP,ADC_I_CAP.filter_out);
	
	control.Real_ratio_forward=ADC_V_CAP.Solved_filter_out/ADC_VIN.Solved_filter_out;
	
		if(ADC_VIN.Solved_filter_out < 20.0f||ADC_VIN.Solved_filter_out>28.0f)          //过压欠压
		{
			control.Cap_Mode=Standby;
			HRTIM_DISABLE();
		}
		else if(ADC_I_IN.Solved_filter_out>0.5f||ADC_I_MOTOR.Solved_filter_out>0.5f||ADC_I_CAP.Solved_filter_out>0.5f)		//电流检测异常
		{
			control.Cap_Mode=Standby;
			HRTIM_DISABLE();
		}
		else if(control.Real_ratio_forward>0&&control.Real_ratio_forward<1.2f)
		{	
			control.Cap_Mode=Normal;
			control.currout_loop.Kf=control.Real_ratio_forward-0.1f;
			control.voltout_loop.Kf=control.Real_ratio_forward-0.1f;
			Update_PWM(control.Real_ratio_forward);
			HRTIM_ENABLE();
		}
	if (control.Cap_Mode==Standby)
		{
			return 1;
		}
	else
		{
			return 0;
		}
}


void Error_check()
{
	    //过压保护判据保持计数器定义 
	if(ADC_VIN.Solved_value<MIN_UVP_VAL||ADC_VIN.Solved_value>MAX_UVP_VAL)
	{
		UVPCnt++;
		if(UVPCnt>=50)
		{
			UVPCnt=0;
			control.Cap_Mode=Standby;
			control.error_code=VOLT;
		}
	}
	else
		{
		UVPCnt=0;
		}	
	if(ADC_I_IN.Solved_value>VIN_I_MAX||ADC_I_CAP.Solved_value>CAP_I_MAX)
	{
		UVPCnt_I++;
		if (UVPCnt_I>=10)
		{
		UVPCnt_I=0;
		control.Cap_Mode=Standby;
		//HRTIM_DISABLE();
		control.error_code=CURRENT;
		}
	}
	else
	{
		UVPCnt_I=0;
	}


	if(measure.P_In<measure.P_Cap)			//电管断电保护
	{
		UVPCnt_Lose++;
		if (UVPCnt_Lose>=50)
		{
		UVPCnt_Lose=0;
		control.Cap_Mode=Standby;
		control.error_code=LOSE;
		}
	}
	else
	{
		UVPCnt_Lose=0;
	}
}


void Mode_Judgment(void)
{

    //暂存上一次的模式状态量
    PreBBFlag = control.Cap_Mode;
    switch (control.Cap_Mode)
    {
    //NA
    case Standby:
    {
	 if(ADC_I_IN.Solved_filter_out>0.5f||ADC_I_MOTOR.Solved_filter_out>0.5f||ADC_I_CAP.Solved_filter_out>0.5f)		//
	{	 
		recovery_cnt_I=0;
		recovery_flag_I=FALASE;
	}
	 else
	{
		recovery_cnt_I++;
		if(recovery_cnt_I>=600)
		{	
			recovery_flag_I=TRUE;		//计数600次后标志位置数
			recovery_cnt_I=0; 
		}
	}
	
	if(ADC_VIN.Solved_value<MIN_UVP_VAL||ADC_VIN.Solved_value>MAX_UVP_VAL||ADC_V_CAP.Solved_value>MAX_UVP_VAL)
	 {	 
		recovery_cnt_V=0;
		recovery_flag_V=FALASE;
	 }
	 else
	{
		recovery_cnt_V++;
		if(recovery_cnt_V>=600)
		{	
			recovery_flag_V=TRUE;
			recovery_cnt_V=0; 
		}
	}
	recovery_flag=(recovery_flag_I&&recovery_flag_V);
	if(recovery_flag==TRUE)
	{
		control.Cap_Mode = Normal; 
		recovery_flag=FALASE;
	}
	break;
    }

    case Normal:
    {
		Error_check();
        break;
    }		
    }

	if(	PreBBFlag != control.Cap_Mode)
	{
		if(PreBBFlag==Standby)
		{	
		//	HRTIM_ENABLE();
		control.currout_loop.Kf=control.Real_ratio-0.1f;
	//	control.voltout_loop.Kf=control.Real_ratio-0.1f;
		control.error_code=NO;
		}
		else
		{
		//	HRTIM_DISABLE();
		}
		control.BBModeChange=1;
		
	}
}


void HRTIM_DISABLE()
{
	HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //通道关闭
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

void HRTIM_ENABLE()
{
	uint8_t state;
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //通道打开
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	state=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	while ( state ^ 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

void Update_PWM(float VBToVA)		//输入值为电容电压比输入电压
									//将占空比转换为寄存器实际值
{
	VBToVA=M_MAX(VBToVA,1.2);			//限幅
	VBToVA=M_MIN(VBToVA,0);
		// A代表DCDC左测
    	// B代表DCDC右侧
			if (VBToVA <0.95f)
			{
				control.DutyA = 0.95f * VBToVA;
				control.DutyB = 0.95f;
			} 
			else if (0.95f<VBToVA <1.1f)
			{
				control.DutyA = (VBToVA + 1.0f) * 0.4f;
				control.DutyB = (1.0f / VBToVA + 1.0f) * 0.4f;
			}
			else
			{
				control.DutyA = 0.95f ;
				control.DutyB = 0.95f/VBToVA;
			}

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A , HRTIM_COMPAREUNIT_1, (HRTIMMaster_Period/2)*(1-control.DutyA));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A , HRTIM_COMPAREUNIT_3, (HRTIMMaster_Period/2)*(1+control.DutyA));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B , HRTIM_COMPAREUNIT_1, (HRTIMMaster_Period/2)*(1-control.DutyB));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B , HRTIM_COMPAREUNIT_3, (HRTIMMaster_Period/2)*(1+control.DutyB));
	control.left_duty=HRTIMMaster_Period * control.DutyA;
	control.right_duty=HRTIMMaster_Period * control.DutyB;
}

