#include "led.h"
#include "can.h" 
#include "delay.h"
#include "PID.h"

#define M3508_SPEED_PID_KP 0.0//15000.0f
#define M3508_SPEED_PID_KI 0.0f//10.0f
#define M3508_SPEED_PID_KD 0.0f
#define M3508_SPEED_PID_MAX_OUT MAX_CAN_CURRENT
#define M3508_SPEED_PID_MAX_IOUT 2000.0f
#define MAX_CAN_CURRENT 16000.0f //最大电流

int16_t rpm_set_01=0;//每分钟转速设置期望
int16_t rpm_set_04=0;//每分钟转速设置期望

pid_type_def PID_3508[4];//数组0-->电调1，数组3-->对应电调4

const static fp32 M3508_speed_pid[4] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};

extern motor_measure_t M3508_Set[4];//结构体

int main(void)
{
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	Can_Init();
	LED_Init();
	LED1=~LED1;
	PID_init(&PID_3508[0],PID_POSITION,M3508_speed_pid, M3508_SPEED_PID_MAX_OUT, M3508_SPEED_PID_MAX_IOUT);
	
	PID_init(&PID_3508[3],PID_POSITION,M3508_speed_pid, M3508_SPEED_PID_MAX_OUT, M3508_SPEED_PID_MAX_IOUT);
	while(1)
	{
		
	PID_calc(&PID_3508[0],M3508_Set[0].speed_rpm,rpm_set_01);//PID的计算，速度
	PID_calc(&PID_3508[3],M3508_Set[3].speed_rpm,rpm_set_04);//PID的计算，速度
	
	CAN_CMD_CHASSIS(PID_3508[0].out,0,0,PID_3508[3].out); 
	delay_ms(1);
	LED0 = ~LED0;
		
	}
}//1
	
