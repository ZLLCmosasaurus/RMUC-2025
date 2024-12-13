#include "dvc_message.h"

/**
 * @brief 注册订阅者到指定的话题
 * 
 * @param __topic_name 话题名称
 * @param __data_len 消息数据长度
 * @return Struct_Subscriber* 返回注册的订阅者指针
 */
Struct_Subscriber* Class_Message::SubRegister(const char  __topic_name[],uint8_t __data_len)
{
    if(strlen(__topic_name) > MAX_TOPIC_NAME_LEN)
    {
        while(1){};
    }
    Struct_Subscriber* subscriber = (Struct_Subscriber*)malloc(sizeof(Struct_Subscriber));
    if (subscriber == NULL)
    {
        while(1){};
    }
    memset(subscriber, 0, sizeof(Struct_Subscriber));
    // 初始化订阅者的消息队列
    Fifo_Init(&subscriber->Message_Queue, __data_len, QUEUE_SIZE);
    subscriber->Next_Subscriber = NULL;
    // 从头开始遍历所有话题
    Struct_Publisher* Now_Publisher = First_Publisher;
    // 通过链表访问所有话题，看看该话题是否已经被注册
    while(Now_Publisher->Next_Topic_Node != NULL)
    {
        Now_Publisher = Now_Publisher->Next_Topic_Node;
        if(strcmp(Now_Publisher->Topic_Name, __topic_name) == 0)
        {
            if (Now_Publisher->First_Subscriber == NULL) 
            {
                Now_Publisher->First_Subscriber = subscriber;
            } 
            else 
            {
                //头插法插入订阅者 减少遍历时间
                subscriber->Next_Subscriber = Now_Publisher->First_Subscriber;
                Now_Publisher->First_Subscriber = subscriber;
            }
            return subscriber;
        }
    }
    // 如果没有找到该话题，说明该话题还没有被注册 注册话题
    Struct_Publisher* publisher = (Struct_Publisher*)malloc(sizeof(Struct_Publisher));
    if (publisher == NULL)
    {
        while(1){};
    }
    memset(publisher, 0, sizeof(Struct_Publisher));
    // 初始化发布者
    strcpy(publisher->Topic_Name, __topic_name);
    publisher->Data_Len = __data_len;
    publisher->First_Subscriber = subscriber;
    publisher->Next_Topic_Node = NULL;
    publisher->Pub_Registered_Flag = 1;
    Now_Publisher->Next_Topic_Node = publisher;
    Topic_Count++;

    return subscriber;
}


/**
 * @brief 注册发布者到指定的话题
 * 
 * @param __topic_name 话题名称
 * @param __data_len 消息数据长度
 * @return Struct_Publisher* 返回注册的发布者指针
 */
Struct_Publisher* Class_Message::PubRegister(const char __topic_name[],uint8_t __data_len)
{
    // 从头开始遍历所有话题
    Struct_Publisher* Now_Publisher = First_Publisher;
    // 通过链表访问所有话题，看看该话题是否已经被注册
    while(Now_Publisher->Next_Topic_Node != NULL)
    {
        Now_Publisher = Now_Publisher->Next_Topic_Node;
        if(strcmp(Now_Publisher->Topic_Name, __topic_name) == 0)
        {
            return Now_Publisher;
        }
    }
    // 如果没有找到该话题，说明该话题还没有被注册 注册话题
    Struct_Publisher* publisher = (Struct_Publisher*)malloc(sizeof(Struct_Publisher));
    if (publisher == NULL)
    {
        while(1){};
    }
    // 初始化发布者
    memset(publisher, 0, sizeof(Struct_Publisher));
    strcpy(publisher->Topic_Name, __topic_name);
    publisher->Data_Len = __data_len;
    publisher->First_Subscriber = NULL;
    publisher->Next_Topic_Node = NULL;
    publisher->Pub_Registered_Flag = 1;
    //头插法插入发布者 减少遍历时间
    publisher->Next_Topic_Node = First_Publisher->Next_Topic_Node;
    First_Publisher->Next_Topic_Node = publisher;
    Topic_Count++;

    return publisher;
}

Class_Message Message_Manager;

