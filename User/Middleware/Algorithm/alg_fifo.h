#ifndef ALG_FIFO_H
#define ALG_FIFO_H

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_BYTE_SIZE 512 // fifo可以申请的最大堆长度（单位Byte）


/**
 * @brief 队列操作状态
 * 
 */
typedef enum Enum_Fifo_Status
{
    FIFO_ERROR = 0,
    FIFO_OK,
} Enum_Fifo_Status;

/**
 * @brief 循环队列结构体
 * 
 * @param pdata 队列首地址
 * @param element_byte_size 元素大小（单位Byte）
 * @param max_element_number 队列能装下的该元素最大个数
 * @param front_index 队头index
 * @param rear_index 队尾index
 * @param count 队列中元素个数
 */
typedef struct
{
    uint8_t *pdata;             // 队列首地址
    uint8_t element_byte_size;  // 元素大小（单位Byte）
    uint8_t max_element_number; // 队列能装下的该元素最大个数
    uint8_t front_index;        // 队头index
    uint8_t rear_index;         // 队尾index
    uint8_t count;              // 队列中元素个数
} CircularFifo;


Enum_Fifo_Status Fifo_Init(CircularFifo *fifo, uint8_t __element_byte_size, uint8_t __max_element_number);
uint8_t Fifo_Is_Full(CircularFifo *fifo);
uint8_t Fifo_Is_Empty(CircularFifo *fifo);
Enum_Fifo_Status Fifo_Enqueue(CircularFifo *fifo, const uint8_t *pitem, uint8_t element_byte_size);
Enum_Fifo_Status Fifo_Dequeue(CircularFifo *fifo, uint8_t *pitem, uint8_t element_byte_size);


#endif // ALG_FIFO_H

