#include "dvc_agvboard.h"
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_Agv_Board_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (Agv_BoardA):
        {
            tmp_tx_data_ptr = &(CAN1_0x01A_Tx_Data[0]);
        }
        break;
        case (Agv_BoardB):
        {
            tmp_tx_data_ptr = &(CAN1_0x01B_Tx_Data[0]);
        }
        break;
        case(Agv_BoardC):
        {
            tmp_tx_data_ptr = &(CAN1_0x01C_Tx_Data[0]);
        }
        break;
        case (Agv_BoardD):
        {
            tmp_tx_data_ptr = &(CAN1_0x01D_Tx_Data[0]);
        }
        break;
        // case (Agv_Board_Max_Power):
        // {
        //     tmp_tx_data_ptr = &(CAN1_0x01E_Tx_Data[0]);
        // }
        // break;
        }
    }
    else if (hcan == &hcan2)
    {
          switch (__CAN_ID)
        {
        case (Agv_BoardA):
        {
            tmp_tx_data_ptr = &(CAN2_0x01A_Tx_Data[0]);
        }
        break;
        case (Agv_BoardB):
        {
            tmp_tx_data_ptr = &(CAN2_0x01B_Tx_Data[0]);
        }
        break;
        case(Agv_BoardC):
        {
            tmp_tx_data_ptr = &(CAN2_0x01C_Tx_Data[0]);
        }
        break;
        case (Agv_BoardD):
        {
            tmp_tx_data_ptr = &(CAN2_0x01D_Tx_Data[0]);
        }
        break;
        // case (Agv_Board_Max_Power):
        // {
        //     tmp_tx_data_ptr = &(CAN2_0x01E_Tx_Data[0]);
        // }
      //  break;
      
        }
    }
    return (tmp_tx_data_ptr);

}
void Class_Agv_Board::Init(CAN_HandleTypeDef *hcan, Enum_Agv_Board_ID __CAN_ID,Enum_Agv_Board_Control_Method __Control_Method, float __Gearbox_Rate, float __Torque_Max,float __Max_Target_Velocity_X,float __Max_Target_Velocity_Y)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    __Control_Method = __Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Torque_Max = __Torque_Max;
    Max_Target_Velocity_X = __Max_Target_Velocity_X;
    Max_Target_Velocity_Y = __Max_Target_Velocity_Y;
    this->CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

void Class_Agv_Board::Output(){
   
    float target_vx=Get_Target_Velocity_X();
    float target_vy=Get_Target_Velocity_Y();
     memcpy(CAN_Tx_Data, &target_vx, sizeof(float));
     memcpy(CAN_Tx_Data + sizeof(float), &target_vy, sizeof(float));
}
//测试用
float sum=0;
void Class_Agv_Board::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    memcpy(&Rx_Data,& sum , sizeof(float));
}