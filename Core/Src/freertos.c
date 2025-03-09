/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tsk_config_and_callback.h"
//#include "dvc_dwt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId control_taskHandle;
osThreadId pull_measure_taskHandle;
osThreadId motor_callback_taskHandle;
osThreadId dr16_callback_taskHandle;
osThreadId refree_callback_taskHandle;
osThreadId init_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Control_Task(void const * argument);
void Pull_Measure_Task(void const * argument);
void Motor_Callback_Task(void const * argument);
void DR16_Callback_Task(void const * argument);
void Refree_Callback_Task(void const * argument);
void Task_Init(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of control_task */
  osThreadDef(control_task, Control_Task, osPriorityLow, 0, 128);
  control_taskHandle = osThreadCreate(osThread(control_task), NULL);

  /* definition and creation of pull_measure_task */
  osThreadDef(pull_measure_task, Pull_Measure_Task, osPriorityAboveNormal, 0, 128);
  pull_measure_taskHandle = osThreadCreate(osThread(pull_measure_task), NULL);

  /* definition and creation of motor_callback_task */
  osThreadDef(motor_callback_task, Motor_Callback_Task, osPriorityBelowNormal, 0, 128);
  motor_callback_taskHandle = osThreadCreate(osThread(motor_callback_task), NULL);

  /* definition and creation of dr16_callback_task */
  osThreadDef(dr16_callback_task, DR16_Callback_Task, osPriorityNormal, 0, 128);
  dr16_callback_taskHandle = osThreadCreate(osThread(dr16_callback_task), NULL);

  /* definition and creation of refree_callback_task */
  osThreadDef(refree_callback_task, Refree_Callback_Task, osPriorityNormal, 0, 128);
  refree_callback_taskHandle = osThreadCreate(osThread(refree_callback_task), NULL);

  /* definition and creation of init_task */
  osThreadDef(init_task, Task_Init, osPriorityRealtime, 0, 128);
  init_taskHandle = osThreadCreate(osThread(init_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Control_Task */
/**
  * @brief  Function implementing the control_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Control_Task */
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  uint32_t start_tick = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
    // 5ms周期执行
    Control_Task_Callback();
    // Pull_Measure_Callback();
    osDelayUntil(&start_tick,5);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Pull_Measure_Task */
/**
* @brief Function implementing the pull_measure_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Pull_Measure_Task */
void Pull_Measure_Task(void const * argument)
{
  /* USER CODE BEGIN Pull_Measure_Task */
  /* Infinite loop */
  for(;;)
  {
    // 等待中断唤醒
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    // Pull_Measure_Callback();
  }
  /* USER CODE END Pull_Measure_Task */
}

/* USER CODE BEGIN Header_Motor_Callback_Task */
/**
* @brief Function implementing the motor_callback_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Callback_Task */
void Motor_Callback_Task(void const * argument)
{
  /* USER CODE BEGIN Motor_Callback_Task */
  /* Infinite loop */
  for(;;)
  {
    // pdTRUE 每次收到通知就清零�?�知 portMAX_DELAY 阻塞等待通知
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    Motor_Callback();
  }
  /* USER CODE END Motor_Callback_Task */
}

/* USER CODE BEGIN Header_DR16_Callback_Task */
/**
* @brief Function implementing the dr16_callback_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DR16_Callback_Task */
void DR16_Callback_Task(void const * argument)
{
  /* USER CODE BEGIN DR16_Callback_Task */
  /* Infinite loop */
  for(;;)
  {
    // 等待中断唤醒
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //遥控器数据接收完�?
    DR16_Callback();
  }
  /* USER CODE END DR16_Callback_Task */
}

/* USER CODE BEGIN Header_Refree_Callback_Task */
/**
* @brief Function implementing the refree_callback_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Refree_Callback_Task */
void Refree_Callback_Task(void const * argument)
{
  /* USER CODE BEGIN Refree_Callback_Task */
  /* Infinite loop */
  for(;;)
  {
    // 等待中断唤醒
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // 裁判系统数据接收完成
    Referee_Callback();
  }
  /* USER CODE END Refree_Callback_Task */
}

/* USER CODE BEGIN Header_Task_Init */
/**
* @brief Function implementing the init_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Init */
void Task_Init(void const * argument)
{
  /* USER CODE BEGIN Task_Init */
  /* Infinite loop */
  for(;;)
  {
    Task_Init_();
    vTaskSuspend(NULL); // ??????
  }

  /* USER CODE END Task_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
