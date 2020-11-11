#include "main.h"

#define START_TASK_PRIO		1
#define START_STK_SIZE 		128  
static TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define CHASSIS_TASK_PRIO 2
#define CHASSIS_TASK_SIZE 512
static TaskHandle_t CHASSISTask_Handler;

#define DATA_PROCESS_TASK_PRIO 3
#define DATA_PROCESS_TASK_SIZE 256
static TaskHandle_t DATA_PROCESS_TASK_Handler;
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区

	xTaskCreate((TaskFunction_t )chassis_task, 
				(const char*    )"chassis_task", 
				(uint16_t       )CHASSIS_TASK_SIZE, 
				(void*          )NULL, 
				(UBaseType_t    )CHASSIS_TASK_PRIO,  
	             			(TaskHandle_t*  )&CHASSISTask_Handler);
				

	xTaskCreate((TaskFunction_t )Data_process_task, 
				(const char*    )"Data_process_task", 
				(uint16_t       )DATA_PROCESS_TASK_SIZE, 
				(void*          )NULL, 
				(UBaseType_t    )DATA_PROCESS_TASK_PRIO,  
				(TaskHandle_t*  )&DATA_PROCESS_TASK_Handler);
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

/*
*********************************************************************************************************
* StartTask_Handler
*********************************************************************************************************
*/
static TaskHandle_t StartTask_Handler;
void startTask(void)   
{
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}











