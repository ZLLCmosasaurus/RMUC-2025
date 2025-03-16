<<<<<<< HEAD
# AGV_CHASSIS
2024_RoboMaster
=======
# Mosasaurus2024 新架构 
> ### *最初fork来自中科大 @yssickjgd 同学的开源架构，感谢万分!!!*
<!-- TOC -->

- [Mosasaurus2024 新架构](#mosasaurus2024-新架构)
  - [前言](#前言)
  - [What‘s New](#whats-new)
  - [代码方言解释](#代码方言解释)
  - [文件架构](#文件架构)
    - [工程类型](#工程类型)
    - [windows的文件结构](#windows的文件结构)
        - [demo.ioc](#demoioc)
        - [./MDK-ARM/demo.uvprojx](#mdk-armdemouvprojx)
        - [串口配置-\*\*.ini](#串口配置-ini)
        - [其他代码文件](#其他代码文件)
    - [CubeMX的项目结构](#cubemx的项目结构)
      - [系统内核](#系统内核)
        - [DMA](#dma)
        - [GPIO](#gpio)
        - [NVIC](#nvic)
        - [RCC](#rcc)
        - [SYS](#sys)
      - [定时器](#定时器)
        - [TIM3](#tim3)
        - [TIM5](#tim5)
        - [TIM12](#tim12)
      - [通信相关](#通信相关)
        - [CAN1](#can1)
        - [CAN2](#can2)
        - [SPI5](#spi5)
        - [USART1](#usart1)
        - [USART2](#usart2)
        - [USART3](#usart3)
        - [USART6](#usart6)
        - [UART7](#uart7)
        - [UART8](#uart8)
    - [Keil的工程结构](#keil的工程结构)
      - [无需修改的文件](#无需修改的文件)
        - [Application/MDK-ARM](#applicationmdk-arm)
        - [Drivers/STM32F4xx\_HAL\_Driver](#driversstm32f4xx_hal_driver)
        - [Drivers/CMSIS](#driverscmsis)
        - [Application/User/Core](#applicationusercore)
      - [自己开发需要维护与修改的文件](#自己开发需要维护与修改的文件)
    - [以A板MCU角度来看的机器人电控结构](#以a板mcu角度来看的机器人电控结构)
      - [板外外设](#板外外设)
      - [板上外设](#板上外设)
  - [命名约定](#命名约定)
    - [通用命名规则](#通用命名规则)
      - [总述](#总述)
      - [说明](#说明)
      - [变量命名举例](#变量命名举例)
    - [文件命名](#文件命名)
    - [注释规范](#注释规范)

<!-- /TOC -->
[toc]

## 前言
1. Psycho Commando: 废话就是问题所有可能答案的父集
2. 不要用泛化词作为符号名称 ( 或一部分 ) , 但有些词如 handler 已经特化成中断处理函数后缀, 所以可以按规定使用
3. 不要使用已经有预约定语义的词, 如 pid_t 是 unix 中的 pid type, 即进程号类型, 易造成困扰
4. 现有的合理命名方案为小驼峰、大驼峰和下划线风格命名
    1. 在一个代码文件内新增内容时, 如果不清楚命名格式, 做到和当前代码文件统一格式即可
    2. 新增一个代码文件时, 如果不清楚命名格式, 做到和同目录下代码文件统一格式即可
5. 当判断状态时, 不要使用裸数字代号, 因为该数字不是数学含义, 应当使用宏定义或枚举类型定义后再使用常量. 
6. 每个函数只做**一件事**, 但要清楚采用什么层级的抽象决定了到底做了几件事
7. A板的**R标志**为正方向
8. 血的教训! ! ! Cube更改配置后生成代码一定要注意, **DMA初始化一定要在各个需要DMA的元件前开启**

## What‘s New
-   drv_bsp

    -   自建了板级支持包drv_bsp系列c与h文件
    -   实现了对直流24V可控电源, LED指示灯, 蜂鸣器, 加热电阻等设备的初始化配置与管理

-   dvc_motor

    -   根据华南虎战队的motor.h文件, 改编形成dvc_motor系列cpp与h文件

    -   在其基础上额外实现了自动分配CAN ID以及CAN滤波器等操作, 顶层操作起来更方便, 不需过度考虑底层实现原理

    -   PID算法类在底层定义后, 其调用并入motor类中, 无需额外声明PID对象. 使用者配置好PID后即可自动在上层调用, 无需同以前一样显式调用PID调整函数, 更符合架构分层的逻辑. 只需在上层设定角度或速度即可

    -   代码风格更统一, 电机类继承的逻辑更直观, 有助于后续战队同学们的理解与维护

-   dvc_serialplot

    -   根据Glorill学长的SerialPlot文件进行优化

    -   利用可变数量传参, 且传的是指针, 大大增强了代码的独立性

    -   相较于旧版串口绘图, 初始化配置后调用更少的函数即可实现串口绘图数据传输与接收指令, 使用起来更方便

-   dvc_dr16

    -   对拨码开关以及按键操作的状态更加细分, 新增上升沿, 下降沿等检测, 有助于不同操作继续细化

    -   初始化配置后只需利用各个Get函数, 即可实现对内容的读取, 更易于找到自己想要的参数


## 代码方言解释

| 词语             | 解释                                                         |
| ---------------- | ------------------------------------------------------------ |
| **drv**          | Driver, 驱动, 一般指底层配置                                 |
| bsp              | Board Supportive Pack, 板级支持包                            |
| can              | Controller Area Network, 一种通信协议, 常用于电机控制        |
| **alg**          | Algorithm, 算法, 包含控制算法, 滤波等                        |
| pid              | 一种带有反馈的控制算法                                       |
| **dvc**          | Device, 设备                                                 |
| motor            | 电机, 顾名思义                                               |
| **crt**          | 战车, 一般指车整体, 包含底盘, 云台, 发射机构等               |
| chassis          | 底盘, 进行战车的整体移动                                     |
| gimbal           | 云台, 控制枪口相对底盘运动的机构                             |
| booster          | 发射机构, 主要包括拨盘和摩擦轮                               |
| pitch、yaw、roll | 欧拉角, 分别是俯仰角, 航向角, 滚转角, 在云台和姿态传感器中描述角度. 形象理解分别对应点头, 摇头, 晃头 |
| ...              | ...                                                          |

## 文件架构

### 工程类型

​		Keil v5 MDK-ARM工程, 使用STM32CubeMX生成, 使用STM32F427IIH芯片. 如需使用vscode开发, 请安装vscode插件并配置keil环境, 然后你就可以在vscode中使用git版本管理工具进行本项目的开发

​		值得注意的时, 想要使用vscode中的keil插件开发, 你应当在MDK-ARM文件夹下, 右键“通过Code打开”, 而不是在根目录下

​		CubeMX生成的代码, 你需要在开发时遵循CubeMX的一切规则. 包括但不限于, 在CubeMX生成的文件中, 你必须在如下两行注释之间写你自己的代码

```c
/* USER CODE BEGIN ...*/
   
/* USER CODE END ...*/
```

​		否则, **在CubeMX中重新生成工程时, 你写的代码将会消失**

### windows的文件结构

##### demo.ioc

-   CubeMX工程

##### ./MDK-ARM/demo.uvprojx

-   Keil工程

##### 串口配置-**.ini

-   串口调试配置文件

##### 其他代码文件

-   均在User文件夹中, 具体拓扑结构与工程文件类似, 详情可见 " keil的工程结构 " 

### CubeMX的项目结构

#### 系统内核

##### DMA

​		DMA不生产数据, 只是数据的搬运工. 一般情况下, 外设收发数据与内存交互, 需要通过内核作为中介. 这种情况下, 内核无法处理其他事情. DMA可以让外设对内存直接访问, 减少了内核的工作量. 而且同时内核可以处理其他事务

-   DMA1
    -   stream5, USART2_RX 示例

-   DMA2
    -   stream5, SPI5_RX 示例

##### GPIO

​		引脚使用情况, 与其他外设直接相关, 故不赘述

##### NVIC

​		中断向量, 与其他外设直接相关, 故不赘述. 

​		但需要注意, 自己新建的中断, 抢占优先级是5, 子优先级是0

##### RCC

​		系统时钟, 此处用到HSE高速外部时钟, 来源于晶振. 配置时选择锁相环PLL通道, HCLK设置为180后自动生成解决方案即可

##### SYS

​		调试线默认Disable不动就行, 时钟源选择TIM14

#### 定时器

##### TIM3

-   arr = 65535

-   channel2, 加热电阻, 为保证AHRS正常工作, 陀螺仪需要外接加热电阻防止温漂. 该PWM波用来控制加热温度

##### TIM5

-   1000Hz
-   使能中断, 用于高频计算并与发送数据

##### TIM12

-   2700Hz

-   channel1, 蜂鸣器

#### 通信相关

##### CAN1

-   0x201~0x204
    -   底盘C620电调
-   0x205
    -   步兵英雄哨兵无人机云台yaw轴GM6020电机
-   0x206
    -   步兵英雄哨兵无人机云台pitch轴GM6020电机

##### CAN2

-   0x201~0x202
    -   步兵英雄哨兵无人机发射机构摩擦轮C620电调
-   0x203
    -   步兵哨兵无人机发射机构拨弹盘C610电调
    -   英雄发射机构拨弹盘C620电调
-   0x207~0x20a
    -   舵轮底盘GM6020电机

##### SPI5

-   预留, A板AHRS收发信息

##### USART1

-   DR16收信息

##### USART2

-   串口绘图收发信息

##### USART3

-   预留, 妙算视觉

##### USART6

-   预留, 裁判系统

##### UART7

-   预留, 云台AHRS

##### UART8

-   预留, 超级电容

### Keil的工程结构

下文按照自底向上的顺序介绍

#### 无需修改的文件

##### Application/MDK-ARM

​		只有一个文件: startup_stm32f427xx.s. 这是stm32的启动文件, 包括初始化堆栈、初始化PC、LR寄存器等功能. 除非你对ARM指令集理解较为深刻并极其确定这个文件出现了问题, 否则不要对这个文件进行任何修改. 也不要在这个文件目录下添加任何的文件

##### Drivers/STM32F4xx_HAL_Driver

​		STM32 HAL（Hardware Abstraction Layer）库文件. CubeMX自动生成了这些文件. 你写的代码应当符合HAL库的规则, 而不是ST官方固件库. 不要对这些文件进行任何修改

##### Drivers/CMSIS

​		STM32的驱动文件. 不要修改

##### Application/User/Core

​		用来进行各种外设初始化配置的文件

-   main.c
    -   完成了基本的配置, 后续可能会开启freertos以及启动一个开始线程, 此外没有其他的作用

-   stm32f4xx_it.c
    -   包含所有的中断函数
    -   如果使用HAL库提供的中断处理无法实现需要的效果, 就需要在这里手动添加中断服务函数

#### 自己开发需要维护与修改的文件

**详见**[代码架构.md](代码架构.md)

### 以A板MCU角度来看的机器人电控结构

#### 板外外设

-   裁判系统与电源管理模块

-   DR16

-   底盘
    -   麦轮, 4个GM6020电机
    -   舵轮, 4个GM6020电机, 4个C620电调
-   云台
    -   yaw轴, 6020电机
    -   pitch轴, 6020电机
    -   云台AHRS
-   发射机构
    -   拨弹盘, C610电调
    -   摩擦轮, 2个C620电调
    -   弹舱盖, 270°舵机
-   超级电容
-   妙算视觉
-   串口绘图

#### 板上外设

-   A板板载陀螺仪
-   DC24电源
-   LED指示灯
-   蜂鸣器

## 命名约定

### 通用命名规则

#### 总述

函数命名, 变量命名, 文件命名要有描述性; 少用缩写

#### 说明

尽可能使用描述性的命名, 别心疼空间, 毕竟相比之下让代码易于新读者理解更重要. 不要用只有项目开发者能理解的缩写, 也不要通过砍掉几个字母来缩写单词

```c
int price_count_reader;    // 无缩写
int num_errors;            // "num" 是一个常见的写法
int num_dns_connections;   // 人人都知道 "DNS" 是什么
```

```c
int n;                     // 毫无意义.
int nerr;                  // 含糊不清的缩写.
int n_comp_conns;          // 含糊不清的缩写.
int wgc_connections;       // 只有贵团队知道是什么意思.
int pc_reader;             // "pc" 有太多可能的解释了.
int cstmr_id;              // 删减了若干字母.
```

注意, 一些特定的广为人知的缩写是允许的, 例如用 `i` 表示迭代变量和用 `T` 表示模板参数

#### 变量命名举例

-   **绝大多数变量**与函数, 用大驼峰+下划线命名全称

-   **函数内的内部变量**以及**文件内的内部函数**, 用小写+下划线命名全称

-   **结构体**, 枚举类型命名如下, 注意变量类型是大驼峰, 元素名前面是大驼峰, 后面是全大写

    ```C++
    /**
     * @brief CAN电机的ID分配情况
     * 
     */
    enum Enum_CAN_Motor_ID_Status
    {
        CAN_Motor_ID_Status_FREE = 0,
        CAN_Motor_ID_Status_ALLOCATED,
    };
    
    Enum_CAN_Motor_ID_Status CAN_Motor_ID_Status;
    ```

-   **宏定义**, 全大写

-   **类的命名规则**较复杂, 只可意会, 下方摘自dvc_motor.h, 有删减. **初始化变量**是除去初始化外不可更改的变量; **常量**是不可更改的默认变量; **内部变量**是不可Get也不可Set的变量; **读变量**是仅可Get的变量; **写变量**是仅可Set的变量; **读写变量**是可Get和Set的变量

    ```c++
    class Class_Motor_CAN
    {
        public:
        
            //PID控制
            Class_PID *PID_Angle;
            Class_PID *PID_Speed;
    
            void Init(CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID);
    
            float Get_Now_Angle();
            float Get_Target_Speed();
    
            void Set_Control_Method(Enum_Control_Method __Control_Method);
            void Set_Target_Speed(float __Target_Speed);
            
            void Output();
        
        protected:
    
            //初始化相关变量
    
            //绑定的CAN总线
            CAN_HandleTypeDef *hcan;
    
            //常量
    
            //一圈编码器刻度
            uint16_t Encoder_Num_Per_Round = 8192;
            
            //内部变量
    
            //接收的编码器位置, 0~8191
            uint16_t Rx_Encoder = 0;
    
            //读变量
    
            //当前的角度, rad
            float Now_Angle = 0;
    
            //写变量
    
            //电机控制方式
            Enum_Control_Method Control_Method = Control_Method_ANGLE;
    };
    ```

    

### 文件命名

库文件本身就是小写加下划线, 例如

```
system_stm32f4xx.c
```

用户文件小写加下划线, 例如

```
dvc_motor.cpp
```

.h与对应的.c同名

### 注释规范

对于代码模板, 本仓库已经给出[xxx.cpp](xxx.cpp)和[xxx.h](xxx.h)两个模板用于进行套用, 希望尽量遵循该模板的内容

如有必要, 需要进一步注释. 一行语句或多行语句用单行注释. 多行语句需要注释时, 如有必要可以用双回车进行分段. 函数前如有必要可以用多行注释解释. 例如

```C
/**
  * @brief  设定预期速度，底盘坐标系
  * @param  V 三轴速度
  * @note	将底盘运动速度解算为四个电机的运动速度
  * @retval	null
  */
void C_Chassis ::SetSpeed(SpeedTypeDef V)
{
    float vx=V.vx, vy=V.vy, omega=V.omega;

    //速度限制
  	//x方向
    _Constrain(&vx, -SpeedMax.vx, SpeedMax.vx);
  	//y方向
    _Constrain(&vy, -SpeedMax.vy, SpeedMax.vy);
  	//角速度
    _Constrain(&omega, -SpeedMax.omega, SpeedMax.omega);

    TargetSpeed.vx = vx;
    TargetSpeed.vy = vy;
    TargetSpeed.omega = omega;

    //更新底盘速度变量
    WheelSpeed[0].Target = (OMEGA_TO_MS * omega + vx - vy) / RPM_TO_MS;
    WheelSpeed[1].Target = (OMEGA_TO_MS * omega + vx + vy) / RPM_TO_MS;
    WheelSpeed[2].Target = (OMEGA_TO_MS * omega - vx + vy) / RPM_TO_MS;
    WheelSpeed[3].Target = (OMEGA_TO_MS * omega - vx - vy) / RPM_TO_MS;
}
```

>>>>>>> origin/gimbal
