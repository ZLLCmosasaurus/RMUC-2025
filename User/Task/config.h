#ifndef _CONFIG_H_
#define _CONFIG_H_

//云台底盘代码切换
#define GIMBAL
//#define CHASSIS

#define USE_DR16
//#define USE_VT13

//#define SPEED_SLOPE
#define NO_SPEED_SLOPE
//底盘X轴Y轴最大速度
#define Chassis_Velocity_Max 5.0f //m/s
//底盘小陀螺转速(最大限制为8)
#define  Chassis_Spin_Omega 6.0f//rad/s

//使能超电
#define SuperCap 0
//使能发射机构超速低速调整
#define Booster_Speed_Adjust 
#endif
//黄-gnd-黑
//黑-txd-黄
//红-rxd-绿