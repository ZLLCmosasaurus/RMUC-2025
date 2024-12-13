#include "alg_fifo.h"

/**
 * @brief 初始化循环队列
 * 
 * @param fifo 队列指针
 * @param __element_byte_size 元素大小（单位Byte）
 * @param __max_element_number 队列能装下的该元素最大个数
 * @return Enum_Fifo_Status 队列状态
 *     - FIFO_OK: 初始化成功
 *     - FIFO_ERROR: 初始化失败
 */
Enum_Fifo_Status Fifo_Init(CircularFifo *fifo, uint8_t __element_byte_size, uint8_t __max_element_number)
{
    if (fifo->element_byte_size * fifo->max_element_number > MAX_BYTE_SIZE)
        return FIFO_ERROR;
    else
    {
        fifo->element_byte_size = __element_byte_size;
        fifo->max_element_number = __max_element_number;
    }
    if (fifo->pdata = (uint8_t *)malloc(fifo->element_byte_size * fifo->max_element_number))
    {
        fifo->front_index = 0;
        fifo->rear_index = 0;
        fifo->count = 0;
        return FIFO_OK;
    }
    else
        return FIFO_ERROR;
}

/**
 * 检查循环FIFO是否已满。
 *
 * @param fifo 要检查的循环FIFO。
 * @return 如果FIFO已满，则返回1；否则返回0。
 */
uint8_t Fifo_Is_Full(CircularFifo *fifo)
{
    return fifo->count == fifo->max_element_number;
}

/**
 * 检查循环FIFO是否为空。
 *
 * @param fifo 要检查的循环FIFO。
 * @return 如果FIFO为空，则返回1；否则返回0。
 */
uint8_t Fifo_Is_Empty(CircularFifo *fifo)
{
    return fifo->count == 0;
}

/**
 * 将一个元素入队列。
 *
 * @param fifo 要操作的循环FIFO。
 * @param pitem 要入队列的元素。
 * @param element_byte_size 元素大小（单位Byte）
 */
Enum_Fifo_Status Fifo_Enqueue(CircularFifo *fifo, const uint8_t *pitem, uint8_t element_byte_size)
{
    if (element_byte_size != fifo->element_byte_size)
        return FIFO_ERROR;
    if (Fifo_Is_Full(fifo))
    {
        memcpy(fifo->pdata + fifo->rear_index * fifo->element_byte_size, pitem, fifo->element_byte_size); // 入队列
        fifo->rear_index = (fifo->rear_index + 1) % fifo->max_element_number;
        fifo->front_index = (fifo->front_index + 1) % fifo->max_element_number;
    }
    else
    {
        memcpy(fifo->pdata + fifo->rear_index * fifo->element_byte_size, pitem, fifo->element_byte_size); // 入队列
        fifo->rear_index = (fifo->rear_index + 1) % fifo->max_element_number;
        fifo->count++;
    }
    return FIFO_OK;
}

/**
 * 将一个元素出队列。
 *
 * @param fifo 要操作的循环FIFO。
 * @param pitem 用于存储出队列元素的缓冲区。
 * @param element_byte_size 元素大小（单位Byte）
 * @return 如果成功出队列，则返回1；否则返回0。
 */
Enum_Fifo_Status Fifo_Dequeue(CircularFifo *fifo, uint8_t *pitem, uint8_t element_byte_size)
{
    if (Fifo_Is_Empty(fifo) || element_byte_size != fifo->element_byte_size)
    {
        return FIFO_ERROR;
    }
    else
    {
        memcpy(pitem, fifo->pdata + fifo->front_index * fifo->element_byte_size, fifo->element_byte_size); // 出队列
        fifo->front_index = (fifo->front_index + 1) % fifo->max_element_number;
        fifo->count--;
        return FIFO_OK;
    }
}
