#include "main.h"



int main(void)
{ 
	Task_Init();
	delay_ms(250);
	startTask();
	while(1);
}
void Task_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	CAN_mode_init();
	delay_init(168);		//初始化延时函数
	uart_init(115200);     	//初始化串口
	//初始化LED端口
	led_configuration();		       
	//遥控器初始化
	remote_control_init();
}
