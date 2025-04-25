#include "dvc_relays.h"
// /* Private macros ------------------------------------------------------------*/

// /* Private types -------------------------------------------------------------*/

// /* Private variables ---------------------------------------------------------*/

// /* Private function declarations ---------------------------------------------*/

// /* Function prototypes -------------------------------------------------------*/


void Class_Relays::Set_Relay_State(Enum_Relays_Id Relays_Id,Enum_Relays_Control_State __Relays_Control_State)
{
	uint8_t id=Relays_Id;
	
	Relays_Control_State [id]=__Relays_Control_State;
	

}


 void Class_Relays::Relay_Control()
 { 
	 if(Relays_Control_State[0]!=Relays_Control_State_Old[0])
	 {
	 	if(Relays_Control_State[0]==Relays_Control_State_ENABLE)
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
		else if(Relays_Control_State[0]==Relays_Control_State_DISABLE)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET); 
	 
		
		Relays_Control_State_Old[0]=Relays_Control_State[0];
	 }
	 
		if(Relays_Control_State[1]!=Relays_Control_State_Old[1])
	 {
	 	if(Relays_Control_State[1]==Relays_Control_State_ENABLE)
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
		else if(Relays_Control_State[1]==Relays_Control_State_DISABLE)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET); 
	 
		
		Relays_Control_State_Old[1]=Relays_Control_State[1];
	 }
	 
	 	if(Relays_Control_State[2]!=Relays_Control_State_Old[2])
	 {
	 	if(Relays_Control_State[2]==Relays_Control_State_ENABLE)
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
		else if(Relays_Control_State[2]==Relays_Control_State_DISABLE)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET); 
	 
		
		Relays_Control_State_Old[2]=Relays_Control_State[0];
	 }
	 
	 
	 	if(Relays_Control_State[3]!=Relays_Control_State_Old[3])
	 {
	 	if(Relays_Control_State[3]==Relays_Control_State_ENABLE)
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
		else if(Relays_Control_State[3]==Relays_Control_State_DISABLE)
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET); 
	 
		
		Relays_Control_State_Old[3]=Relays_Control_State[3];
	 }
	 
	 	if(Relays_Control_State[4]!=Relays_Control_State_Old[4])
	 {
	 	if(Relays_Control_State[4]==Relays_Control_State_ENABLE)
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
		else if(Relays_Control_State[4]==Relays_Control_State_DISABLE)
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET); 
	 
		
		Relays_Control_State_Old[4]=Relays_Control_State[4];
	 }
	 
	 
	 
	 
	 
	 
 
 }
// /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
