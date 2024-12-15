#ifndef DVC_AGVBOARD_H
#define DVC_AGVBOARD_H

//include
#include "drv_can.h"
#include "dvc_referee.h"
enum Enum_Agv_Board_ID{
Agv_BoardA, //发送舵轮轮组矢量速度
Agv_BoardB,
Agv_BoardC,
Agv_BoardD,
Agv_Board_Max_Power,
};
enum Enum_Agv_Board_Control_Method
{
    Control_Method_OPENLOOP = 0,
    Control_Method_TORQUE,
    Control_Method_OMEGA,
    Control_Method_ANGLE,
    Control_Method_IMU_OMEGA,
    Control_Method_IMU_ANGLE,
};
/// @brief 
class Class_Agv_Board{
public:
void Init(CAN_HandleTypeDef *hcan, Enum_Agv_Board_ID __CAN_ID, Enum_Agv_Board_Control_Method __Control_Method=Control_Method_OMEGA, float __Gearbox_Rate=13.933f, float __Torque_Max=16384.0f,float __Max_Target_Velocity_X=5,float __Max_Target_Velocity=5);

inline void Set_Target_Velocity_X(float _Target_Velocity_X);//设置目标速度
inline void Set_Target_Velocity_Y(float _Target_Velocity_Y);
inline float Get_Target_Velocity_X();//获取目标速度
inline float Get_Target_Velocity_Y();//获取目标速度
 
inline void Set_k1(float _k1);//设置功率缩放系数k1;
inline void Set_k2(float _k2);//设置功率缩放系数k2;
inline float Get_k1();//获取功率缩放系数k1;
inline float Get_k2();//获取功率缩放系数k2;
void CAN_RxCpltCallback(uint8_t *Rx_Data);//测试用，接受舵小板发送给的总功率
void Output();//数据处理，发送数据
float Actual_Power=0.0f;

protected:

    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;

float k1=0.0f;//功率缩放系数k1
float k2=0.0f;//功率缩放系数k2
 //绑定的CAN
Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
Enum_Agv_Board_ID CAN_ID;



    //发送缓存区
uint8_t *CAN_Tx_Data;

 float Gearbox_Rate=0;
    //最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
    float Torque_Max=0.0;
    float Max_Target_Velocity_X=4.0f;
    float Max_Target_Velocity_Y=4.0f;


};
void Class_Agv_Board::Set_Target_Velocity_X(float _Target_Velocity_X){
    Target_Velocity_X = _Target_Velocity_X;
}
void Class_Agv_Board::Set_Target_Velocity_Y(float _Target_Velocity_Y){
    Target_Velocity_Y = _Target_Velocity_Y;
}
float Class_Agv_Board::Get_Target_Velocity_X(){
    return Target_Velocity_X;
}
float Class_Agv_Board::Get_Target_Velocity_Y(){
    return Target_Velocity_Y;
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
void Class_Agv_Board::Set_k1(float _k1){
    k1=_k1;
}
void Class_Agv_Board::Set_k2(float _k2){
    k2=_k2;
}
float Class_Agv_Board::Get_k1(){
    return k1;
}
float Class_Agv_Board::Get_k2(){
    return k2;
}
#endif