/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020		     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */

#include "CAN_Receive.h"
uint16_t k=0;
uint16_t O=0;
#define CHASSIS_CAN CAN2
#define GIMBAL_CAN CAN2
#define SHOOT_CAN CAN1
//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//统一处理can接收函数
static void CAN1_hook(CanRxMsg *rx_message);
static void CAN2_hook(CanRxMsg *rx_message);
//声明电机变量
motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_friction[2],motor_chassis[4];

	
//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}

//can2中断
void CAN2_RX1_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN1_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_TRIGGER_MOTOR_ID:
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_trigger, rx_message);
        break;
    }
	case CAN_FRICTION_right_ID:
	case CAN_FRICTION_left_ID:
	{
	static uint8_t i = 0;
		//处理电机ID号
        i = rx_message->StdId - CAN_FRICTION_right_ID;
        //处理电机数据宏函数
        get_motor_measure(&motor_friction[i], rx_message);
	break;
	}
    default:
    {
        break;
    }
    }
}
CanRxMsg  rx_message;
static void CAN2_hook(CanRxMsg *rx_message)
{
	switch(rx_message->StdId)
	{
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        get_motor_measure(&motor_chassis[i], rx_message);
	   k++;
        break;
    }
    default:
    {
        break;
    }
	
	
	}
}
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}


//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
	
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
	O++;
}


