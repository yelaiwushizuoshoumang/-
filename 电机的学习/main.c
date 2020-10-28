#include "led.h"
#include "can.h" 
#include "delay.h"
uint16_t out=0;
int main(void)
{
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	Can_Init();
	LED_Init();
	while(1)
	{
	CAN_CMD_CHASSIS(out,0,0,0);
	delay_ms(1);
	LED0 = ~LED0;
	}
}
	
