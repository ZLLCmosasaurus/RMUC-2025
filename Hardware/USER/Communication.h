#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Measure.h"
#ifdef __cplusplus
extern "C"
{
#endif
void Bluetooth_transmission1(void);
void Bluetooth_transmission2(void);
void Data_Send_F1(int *pst, unsigned char len);
void Data_Send_F2(int *pst, unsigned char len);
void communication_can_send();
void CAN_Data_Process(uint8_t *RX_Data);
void Send_Error();
extern int rx_cnt;
#define Low_Volt 1
#ifdef __cplusplus
}
#endif
#endif
