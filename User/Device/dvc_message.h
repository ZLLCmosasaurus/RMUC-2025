#ifndef _MESSAGE_H
#define _MESSAGE_H

#include "alg_fifo.h"
#include <cstdlib>

#define MAX_TOPIC_NAME_LEN 32 // 最大的话题名长度,每个话题都有字符串来命名
#define MAX_TOPIC_COUNT 12    // 最多支持的话题数量
#define QUEUE_SIZE 10         // 每个订阅者的消息队列大小


struct Struct_Subscriber
{
    /* 用数组模拟FIFO队列 */
    CircularFifo Message_Queue;
    /* 指向下一个订阅了相同的话题的订阅者的指针 */
    Struct_Subscriber *Next_Subscriber; // 使得发布者可以通过链表访问所有订阅了相同话题的订阅者
};


/**
 * @brief 发布者类型.每个发布者拥有发布者实例,并且可以通过链表访问所有订阅了自己发布的话题的订阅者
 *
 */
struct Struct_Publisher
{
    char Topic_Name[MAX_TOPIC_NAME_LEN];
    int Data_Len;                        // 该话题的数据长度
    /* 指向第一个订阅了该话题的订阅者,通过链表访问所有订阅者 */
    Struct_Subscriber *First_Subscriber;
    /* 指向下一个Publisher的指针 */
    Struct_Publisher *Next_Topic_Node;
    int Pub_Registered_Flag; // 用于标记该发布者是否已经注册
};


/* 用于初始化空的话题链表头 */
static Struct_Publisher Null_Publisher_Head = {
    "Message_Manager",
    0,
    NULL,
    NULL,
    0
};

class Class_Message
{
    public:
        Struct_Subscriber* SubRegister(const char __topic_name[],uint8_t __data_len);
        Struct_Publisher* PubRegister(const char __topic_name[],uint8_t __data_len);
        template <typename T>
        void SubGet_Message(Struct_Subscriber* __subscriber, T& __data);
        template <typename T>
        void PubPush_Message(Struct_Publisher* __publisher, T& __data);
    private:
        Struct_Publisher* First_Publisher = &Null_Publisher_Head;
        int Topic_Count;
};

/**
 * @brief 从消息队列中检索特定订阅者的消息。
 * 
 * 此函数通过遍历链表结构中的所有主题和订阅者。
 * 如果找到订阅者，则从订阅者的消息队列中出队一个消息。
 * 然后将出队的消息复制到提供的数据变量中。
 * 
 * @tparam T 要从消息队列中检索的数据类型。
 * @param __subscriber 指向订阅者结构的指针。
 * @param __data 存储检索到的数据的变量的引用。
 */
template <typename T>
void Class_Message::SubGet_Message(Struct_Subscriber* __subscriber, T& __data)
{
    Struct_Publisher* Now_Publisher = First_Publisher;
    Struct_Subscriber* Now_Subscriber = First_Publisher->First_Subscriber;
    // 通过链表访问所有话题
    while(Now_Publisher->Next_Topic_Node != NULL)
    {
        Now_Publisher = Now_Publisher->Next_Topic_Node;
        Now_Subscriber = Now_Publisher->First_Subscriber;
        // 通过链表访问所有订阅者 更新消息队列
        while(Now_Subscriber != NULL)
        {
            if(Now_Subscriber == __subscriber)
            {
                uint8_t data[Now_Publisher->Data_Len];
                if(Fifo_Dequeue(&__subscriber->Message_Queue, data, Now_Publisher->Data_Len) == FIFO_OK)
                {
                    memcpy(&__data, data, Now_Publisher->Data_Len);
                    return;
                }
            }
            Now_Subscriber = Now_Subscriber->Next_Subscriber;
        }
    }
    return;
}

/**
 * @brief 将消息推送到指定话题的订阅者的消息队列中。
 * 
 * 此函数通过遍历链表结构中的所有话题和订阅者。
 * 如果找到指定的话题，则将消息复制到订阅者的消息队列中。
 * 
 * @tparam T 要推送的数据类型。
 * @param __publisher 指向发布者结构的指针。
 * @param __data 要推送的数据的引用。
 */
template <typename T>
void Class_Message::PubPush_Message(Struct_Publisher* __publisher, T& __data)
{
    Struct_Publisher* Now_Publisher = First_Publisher;
    Struct_Subscriber* Now_Subscriber = First_Publisher->First_Subscriber;
    // 通过链表访问所有话题
    while(Now_Publisher->Next_Topic_Node != NULL)
    {
        Now_Publisher = Now_Publisher->Next_Topic_Node;
        if(Now_Publisher == __publisher)
        {
            Now_Subscriber = Now_Publisher->First_Subscriber;
            // 通过链表访问所有订阅者 更新消息队列
            while(Now_Subscriber != NULL)
            {
                uint8_t data[Now_Publisher->Data_Len];
                memcpy(data, &__data, Now_Publisher->Data_Len);
                Fifo_Enqueue(&Now_Subscriber->Message_Queue, data, Now_Publisher->Data_Len);
                Now_Subscriber = Now_Subscriber->Next_Subscriber;
            }
            return;
        }
    }
    return;
}


typedef Struct_Subscriber* Subscriber;
typedef Struct_Publisher* Publisher;

extern Class_Message Message_Manager;

#endif 