#include "main.h"

PID PID_Ir,PID_Or;//PID的内外环
const static fp32 M3508_speed_pid[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};
extern motor_measure_t M3508_Set;//结构体
Encoder Encoder_t;//编码器
extern CanRxMsg rx2_message;
float need_1,need_2;

float out_1,out;
uint16_t speed=0;

extern motor_measure_t motor_chassis[4];
int main(void)
{
//	delay_init(168);
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
//	CAN_mode_init();	
//	PID_Set(&PID_Ir,10,0 ,0);
//	PID_Set(&PID_Or,15,0 ,0);
////	GetEncoderBias(&Encoder_t,&rx2_message);
//	while(1)
//	{
////	need_1=AngleEncoderProcess(&Encoder_t,&rx2_message);
////	PID_Ir.ref=speed;//角度值
////	PID_Ir.fdb=need_1;//速度
////	out_1= PID_Control(&PID_Ir);
////need_2 =((rx2_message.Data[2]<<8)|rx2_message.Data[3]);//机械转子角度
////	PID_Or.ref=out_1;//角度值
//		
//		
//		
//	PID_Or.ref=300;//角度值
//		
//	/*************************************************************************************/
//		
////	PID_Or.fdb=(int16_t)((uint16_t)((rx2_message.Data[2]<<8|rx2_message.Data[3])));//速度    **传奇**
//	PID_Or.fdb=(float)(rx2_message.Data[2]<<8|rx2_message.Data[3]);//速度

////	PID_Or.fdb=(int16_t)((rx2_message.Data[2]<<8|rx2_message.Data[3]));//速度

//		
//	/*************************************************************************************/

////	PID_Or.fdb=motor_chassis[0].speed_rpm;
//		out= ANGLE_PID_Control(&PID_Or);
//	VAL_LIMIT(PID_Or.pid_out, -10000, 10000);
////	CAN_CMD_CHASSIS(PID_Or.pid_out,0,0,0);	
//	delay_ms(1);

//	}





	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	CAN_mode_init();	
	PID_Set(&PID_Ir,10,0 ,0);
	PID_Set(&PID_Or,15,0 ,0);
	GetEncoderBias(&Encoder_t,&rx2_message);
	while(1)
	{
	need_1=AngleEncoderProcess(&Encoder_t,&rx2_message);
	PID_Ir.ref=speed;
	PID_Ir.fdb=need_1;
	out_1= PID_Control(&PID_Ir);
	need_2 =(int16_t)((rx2_message.Data[2]<<8)|rx2_message.Data[3]);
	PID_Or.ref=out_1;
	PID_Or.fdb=need_2;
	out= PID_Control(&PID_Or);
	CAN_CMD_CHASSIS(out,0,0,0);
	delay_ms(1);
	}
}
