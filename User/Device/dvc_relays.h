#include "stm32f4xx_hal.h"
enum Enum_Relays_Control_State
{
    Relays_Control_State_DISABLE = 0,
    Relays_Control_State_ENABLE,
};
enum Enum_Relays_Id
{
	Relays_1=0,//����1���������
	Relays_2,//����2
	Relays_3,//��ŷ�1����۵�ŷ�
	Relays_4,//��ŷ�2����
	Relays_5,//��ŷ�3����

};
//* @brief 云台控制类型
enum Enum_Relays_Control_Type
{
   Relays_Control_Type_Auto = 0,
   Gimbal_Control_Type_Hand,
};
class Class_Relays
{
	public:
	Enum_Relays_Control_State Relays_Control_State [5]={Relays_Control_State_DISABLE,Relays_Control_State_DISABLE,Relays_Control_State_DISABLE,Relays_Control_State_DISABLE,Relays_Control_State_DISABLE};
 	Enum_Relays_Control_State Relays_Control_State_Old [5];
	 Enum_Relays_Control_Type Relays_Control_Type=Relays_Control_Type_Auto;
	void Set_Relay_State(Enum_Relays_Id Relays_Id,Enum_Relays_Control_State Relays_Control_State);
	void Relay_Control();
  protected:
 };


