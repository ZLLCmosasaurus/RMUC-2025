/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "ita_chariot.h"
#include "drv_dwt.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished =0 ;
bool start_flag=0;

uint16_t tension_flag,pre_tension_flag;
//机器人控制对象
Class_Chariot chariot;
extern osThreadId control_taskHandle;
extern osThreadId pull_measure_taskHandle;
extern osThreadId motor_callback_taskHandle;
extern osThreadId dr16_callback_taskHandle;
extern osThreadId refree_callback_taskHandle;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


void Device_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{

}

/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    
}

/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI1_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{

}


/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */

void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    //chariot.MiniPC.USB_RxCpltCallback(Buffer);
}

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {   
        case (0x201):   
        case (0x202):   
        case (0x203):
        case (0x204):
        {
            // 接收 0x201-4 ID的电机
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(motor_callback_taskHandle,&xHigherPriorityTaskWoken); // 唤醒任务
            // 如果can解包优先级大于当前任务的优先级, 则需要通知内核修改isr返回地址和sp为内核，以便于isr结束cpu由内核接管，用于调度下一个任务
            if(xHigherPriorityTaskWoken == pdTRUE){
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        break;
	}
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x205):   
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(motor_callback_taskHandle,&xHigherPriorityTaskWoken); // 唤醒任务
            // 如果can解包优先级大于当前任务的优先级, 则需要通知内核修改isr返回地址和sp为内核，以便于isr结束cpu由内核接管，用于调度下一个任务
            if(xHigherPriorityTaskWoken == pdTRUE){
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        break;
	}
}

// 外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        // 接收 0x201-4 ID的电机
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(pull_measure_taskHandle,&xHigherPriorityTaskWoken); // 唤醒任务 
        if(xHigherPriorityTaskWoken == pdTRUE){
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }       
    }
}

/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
uint16_t length;
void DR16_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    uart_receive_handler(UART1_Manage_Object.UART_Handler);
}

/**
 * @brief UART7调试控制回调函数
 *
 * @param Buffer UART7收到的消息
 * @param Length 长度
 */
void DebugControl_UART7_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DebugControl.DebugControl_Rx_Callback(Buffer, Length);
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(refree_callback_taskHandle,&xHigherPriorityTaskWoken); // 唤醒任务
    if(xHigherPriorityTaskWoken == pdTRUE){
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief TIM5任务回调函数
 *
 */
extern "C" void Control_Task_Callback()
{

    static uint8_t tx_cnt = 0;
    tx_cnt++;

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    
    chariot.TIM5msMod10_Alive_PeriodElapsedCallback();

    /****************************** 交互层回调函数 1ms *****************************************/
    chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();

    if(chariot.DR16.Get_DR16_Status() == DR16_Status_DISABLE)
    {
        // 遥控器控制
        
    }
    
    chariot.Updata_Switch_Status();

    chariot.Updata_Distance_Angle();

    chariot.Updata_Motor_Left_Right_Offset();

    chariot.FSM_Dart_Control.Reload_TIM_Status_PeriodElapsedCallback();

    chariot.TIM_Calculate_PeriodElapsedCallback();

    // chariot.Test_Tension();

    // Pull_Measure_Callback(UART3_Manage_Object.Rx_Buffer,6);

    if(huart3.ErrorCode == HAL_UART_ERROR_FE || huart3.ErrorCode == HAL_UART_ERROR_ORE || huart3.ErrorCode == HAL_UART_ERROR_NE){
        huart3.ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart3, UART3_Manage_Object.Rx_Buffer, 6);
    }
    // if(chariot.Calibrate()){
    //     chariot.Test_Tension();
    // }
    
    // chariot.Test_PID();

    //测试6020 速度环
    // chariot.Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // chariot.Motor_Yaw.Set_Target_Omega_Angle(Test_Target_Omega);
    // chariot.Motor_Yaw.TIM_PID_PeriodElapsedCallback();


    /****************************** 驱动层回调函数 1ms *****************************************/ 
    //统一打包发送
    TIM_CAN_PeriodElapsedCallback();

    // debug 发送
    if(tx_cnt==10)
    {   
        if(chariot.DebugControl.Debug_Start_Flag)
        {
            //20hz
            chariot.DebugControl.DebugControl_Tx_Callback(chariot.Motor_Yaw.Get_Now_Angle(),
                                                          chariot.Tension_Meter.Get_Tension_Meter()/10.0f);
            TIM_UART_PeriodElapsedCallback();
            tx_cnt = 0;            
        }
    } 
	        
}

/**
 * @brief CAN1 电机解包任务 回调函数
 *  details: 触发方式 isr 触发
 */
void Motor_Callback()
{
    switch(CAN2_Manage_Object.Rx_Buffer.Header.StdId){
        case(0x205):
        {
            chariot.Motor_Yaw.CAN_RxCpltCallback(CAN2_Manage_Object.Rx_Buffer.Data);
        }
        break;
    }
    switch (CAN1_Manage_Object.Rx_Buffer.Header.StdId)
    {   
        case (0x201):   
        {
            chariot.Motor_Up.CAN_RxCpltCallback(CAN1_Manage_Object.Rx_Buffer.Data);
        }
        break;
        case (0x202):  
        {
            chariot.Motor_Down.CAN_RxCpltCallback(CAN1_Manage_Object.Rx_Buffer.Data);
        } 
        break;
        case (0x204):
        {
            chariot.Motor_Left.CAN_RxCpltCallback(CAN1_Manage_Object.Rx_Buffer.Data);
        }
        break;
        case (0x203):
        {
            chariot.Motor_Right.CAN_RxCpltCallback(CAN1_Manage_Object.Rx_Buffer.Data);
        }
        break;
        default:
        break;
	}
}


/**
 * @brief UART1 DR16解包任务 
 * details: 触发方式 isr 触发
 */
void DR16_Callback()
{
    // 取出数据
    uint8_t *Buffer = UART1_Manage_Object.Rx_Buffer;
    // 解包
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    // 控制策略
    chariot.TIM_Control_Callback(); 
}


/**
 * @brief UART6 Referee解包任务 
 * details: 触发方式 isr 触发
 */
void Referee_Callback()
{
    chariot.Referee.UART_RxCpltCallback(UART6_Manage_Object.Rx_Buffer,UART6_Manage_Object.Rx_Buffer_Length);
}

/**
 * @brief EXIT 拉力计解包任务 
 * details: 触发方式 isr 触发
 */
void Pull_Measure_Callback(uint8_t *Buffer, uint16_t Length)
{
    // chariot.Tension_Meter.TensionMeter_Cal();
    Struct_Tension_Meter_Data Tension_Meter_Data;
    tension_flag++;
    memcpy(&Tension_Meter_Data,Buffer,sizeof(Struct_Tension_Meter_Data));
    chariot.Tension_Meter.Set_Tension_Meter(Tension_Meter_Data.Tension_Value);  
}


/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init_()
{  

    // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    DWT_Init(168);

    /********************************** 驱动层初始化 **********************************/

    //裁判系统
    UART_Init(&huart6, Referee_UART6_Callback, 128);   //并未使用环形队列 尽量给长范围增加检索时间 减少丢包

    //遥控器接收
    UART_Init(&huart1, DR16_UART1_Callback, 18);

    //调试控制串口
    UART_Init(&huart7, DebugControl_UART7_Callback, sizeof(Struct_DebugControl_RxData));

    UART_Init(&huart3, Pull_Measure_Callback, 6);

    //集中总线can1/can2
    CAN_Init(&hcan1, Device_CAN1_Callback);
    CAN_Init(&hcan2, Device_CAN2_Callback);

    // //c板陀螺仪spi外设
    // SPI_Init(&hspi1,Device_SPI1_Callback);

    // //磁力计iic外设
    // IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    

    // UART_Init(&huart6, Image_UART6_Callback, 40);

    //上位机USB
    // USB_Init(&MiniPC_USB_Manage_Object,MiniPC_USB_Callback);

    // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

    //定时器循环任务
    // TIM_Init(&htim4, Task100us_TIM4_Callback);
    // TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init();

    /********************************* 使能调度时钟 *********************************/

    // HAL_TIM_Base_Start_IT(&htim4);
    // HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
 extern "C" void Task_Loop()
{

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        static uint8_t index = 0;
        static uint8_t rec_buf[20] = {0};
        // 处理接收到的数据
        // 获取最新接收的字节
        uint8_t new_byte = UART3_Manage_Object.Rx_Buffer[0];
        
        // 检测起始字节（0x55）
        if(index == 0 && new_byte == 0x55) 
        {
            rec_buf[index++] = new_byte;
        } 
        // 已开始接收数据包
        else if(index > 0) 
        {
            // 检测是否提前出现新包头
            if(new_byte == 0x55) 
            {
                index = 0; // 重置索引，开始新数据包
                rec_buf[index++] = new_byte;
            } 
            else 
            {
                rec_buf[index++] = new_byte;
                
                // 完整接收6字节后验证尾字节
                if(index >= 6) 
                {
                    if(rec_buf[5] == 0xAA) // 验证结束标志
                    {
                        Pull_Measure_Callback(rec_buf, 6);
                    }
                    index = 0; // 重置索引准备接收新包
                }
            }
        }
        
        // 重新启动单字节接收（重要！）
        HAL_UART_Receive_IT(&huart3, UART3_Manage_Object.Rx_Buffer, 1);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
