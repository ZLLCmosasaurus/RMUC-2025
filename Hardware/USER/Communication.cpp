#include "stm32f3xx_hal.h"
#include "Communication.h"
int data_to_send[18];
#include "usart.h"
#include "Measure.h"
#include "Power_Control.h"
#include "can.h"
#include "string.h"
#include "drv_can.h"
#include "config.h"
extern int16_t CHA1,CHA2,CHB1,CHB2;
extern float pwm;
extern float pwm_1;
extern control_struct_t control; 
extern  uint16_t UVPCnt;
extern uint32_t Data_1,Data_2;
int rx_cnt;
/*------------------------------------------------------------------------------------------------------
【函    数】老梁的蓝牙传输函数(两个函数组合）
【功    能】                
【参    数】USART2
【返 回 值】
【注意事项】                         
-------------------------------------------------------------------------------------------------------*/

void Bluetooth_transmission1()
{
	data_to_send[0]=ADC_VIN.Solved_value*100;
	data_to_send[1]=ADC_I_IN.Solved_value*100;
	data_to_send[2]=measure.P_In*100;
	data_to_send[3]=ADC_I_IN.filter_out;  //发送数据到匿名科创站
	data_to_send[4]=ADC_V_CAP.Solved_value*100;
	data_to_send[5]=ADC_I_CAP.Solved_value*100;
	data_to_send[6]=measure.P_DCDC*100;
	data_to_send[7]=ADC_I_MOTOR.Solved_value*100;
	data_to_send[8]=measure.P_Motor*100;
	data_to_send[9]=control.P_set*100;
	Data_Send_F1(data_to_send,10);
}

void Bluetooth_transmission2()
{
	data_to_send[0]=control.voltout_loop.out;
	data_to_send[1]=control.voltout_loop.out;
	data_to_send[2]=control.left_duty;
	data_to_send[3]=control.right_duty;
	data_to_send[4]=measure.I_DCDC*100;
	data_to_send[5]=control.I_Charge_limited*100;
	data_to_send[6]=777;
	data_to_send[7]=777;
	data_to_send[8]=control.Cap_Mode; 
	data_to_send[9]=measure.surplus_energy;
	Data_Send_F2(data_to_send,10);
}

void Data_Send_F1(int *pst, unsigned char len)
{
	unsigned char sum = 0,add=0;
	unsigned char i;
	unsigned char data_to_send[45];
	data_to_send[0] = 0xAA;
    data_to_send[1] = 0xFF;
    data_to_send[2] = 0xF1;
    data_to_send[3] = 2*len;
    for(i=0;i<len;i++)
    {
        data_to_send[2*i+4]=(unsigned char)pst[i];
        data_to_send[2*i+5]=(unsigned char)(pst[i]>>8);
    }
    for(i=0;i<2*len+4;i++)
    {
        sum += data_to_send[i];
		add+=sum;
    }
    data_to_send[2*len+4] = sum;
	data_to_send[2*len+5] = add;
    HAL_UART_Receive_DMA(&huart2, data_to_send, 2*len+6);
}

void Data_Send_F2(int *pst, unsigned char len)
{
	unsigned char sum = 0,add=0;
    unsigned char i;
    unsigned char data_to_send[45];
    data_to_send[0] = 0xAA;
    data_to_send[1] = 0xFF;
    data_to_send[2] = 0xF2;
    data_to_send[3] = 2*len;
    for(i=0;i<len;i++)
    {
        data_to_send[2*i+4]=(unsigned char)pst[i];
        data_to_send[2*i+5]=(unsigned char)(pst[i]>>8);
    }
    for(i=0;i<2*len+4;i++)
    {
        sum += data_to_send[i];
		add+=sum;
    }
    data_to_send[2*len+4] = sum;
	data_to_send[2*len+5] = add;
    HAL_UART_Receive_DMA(&huart2, data_to_send, 2*len+6);
}



/*------------------------------------------------------------------------------------------------------
【函    数】CAN通信
【功    能】                
【参    数】	
【返 回 值】
【注意事项】                         
-------------------------------------------------------------------------------------------------------*/

void CAN_Data_Process(uint8_t *RX_Data)
{   
    memcpy(&control.P_set,&RX_Data[0],4);
    memcpy(&control.power_limit,&RX_Data[4],1);
    rx_cnt++;
}

void Send_Error()
{
    ERROR_DATA[0]=Low_Volt;                   //低电压警告
    memcpy(&ERROR_DATA[1],& measure.p_beyondl,4);          //底盘功率
    CAN_Send_Data(&hcan,ERROR_ID,ERROR_DATA,8);
}