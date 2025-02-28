#include "stm32f4xx_hal.h"
enum Enum_Relays_Control_State
{
    Relays_Control_State_DISABLE = 0,
    Relays_Control_State_ENABLE,
};
class Class_Relays
{
	public:
 void Init(GPIO_TypeDef* GPIOX, uint16_t __Driver_GPIO_Pin);
 void Set_Target_State(Enum_Relays_Control_State __Target_State);
 uint8_t Get_Open_flag();
 void Set_Open_flag(uint8_t Flag);
 protected:
     //��ʼ����س���

     //������ʱ�����
     GPIO_TypeDef *Driver_GPIOX;
     //��ʱ��ͨ��
     uint8_t Driver_GPIO_Pin;

     uint8_t Open_flag=0;
     //����

     //�ڲ�����

     //������

     //д����

     //��д����
		Enum_Relays_Control_State Target_State;
     //�ڲ�����
    
     void Output();
 };


