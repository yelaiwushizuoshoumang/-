#include "led.h"
#include "can.h" 
#include "delay.h"
#include "PID.h"

#define M3508_SPEED_PID_KP 10//15000.0f
#define M3508_SPEED_PID_KI 0.0f//10.0f
#define M3508_SPEED_PID_KD 0.0f
#define M3508_SPEED_PID_MAX_OUT MAX_CAN_CURRENT
#define M3508_SPEED_PID_MAX_IOUT 2000.0f
#define MAX_CAN_CURRENT 16000.0f //最大电流

uint16_t out=0;
int16_t rpm_set=100;//每分钟转速设置期望

const static fp32 M3508_speed_pid[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};

pid_type_def PID_3508;

extern motor_measure_t M3508_Set;//结构体

int main(void)
{
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	Can_Init();
	LED_Init();
	LED1=~LED1;
	PID_init(&PID_3508,PID_POSITION,M3508_speed_pid, M3508_SPEED_PID_MAX_OUT, M3508_SPEED_PID_MAX_IOUT);
	while(1)
	{
	PID_calc(&PID_3508,M3508_Set.speed_rpm,rpm_set);//PID的计算，速度
	CAN_CMD_CHASSIS(PID_3508.out,PID_3508.out,0,0); 
		
	delay_ms(1);
	LED0 = ~LED0;
	}
}
	
