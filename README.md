## **特别注意** 
### 在使用cubmx重新生成代码时，需要重新修改一下STM32F427IIHx_FLASH.ld文件，否则会出现链接错误,具体修改部分如下：
![1](./pictures/RAM栈段地址.png)
![2](./pictures/RAM数据段.png)
![3](./pictures/RAM%20.bss段.png)
![4](./pictures/RAM堆栈分配.png )


## 接线 
 
| A板标签上的丝印   |    STM32F427引脚   |      模块引脚    |      补充说明    |
|:-----------------:|:------------------:|:----------------:|:--------------:|
|        I1         |         PF1        |    XH711_DT      |    无           |
|        I2         |         PF0        |    XH711_SCK     |    无          |
|         +         |         VCC        |    XH711_VCC     |    5.0V       |
|         G         |         GND        |    XH711_GND     |   GND        |
|        A          |         PI0         |    PWM_1      |   装填舵机    |
|        B         |         PH12        |    PWM_2     |   装填舵机    |
|         C         |        PH11        |    PWM_3     |   装填舵机    |
|         D         |        PH10        |    PWM_4     |   装填舵机    |
|         E         |        PD15        |    PWM_5     |   扳机舵机    |