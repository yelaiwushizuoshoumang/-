#ifndef __CAN_H
#define __CAN_H
#include "sys.h"

//===============GPIO的初始化=============
#define ADVANCE_GPIOx_CLK  RCC_APB2Periph_GPIOA
#define ADVANCE_CAN_CLK   RCC_APB1Periph_CAN1
#define ADVANCE_CAN_RX  GPIO_Pin_11 //can的接收
#define ADVANCE_CAN_TX  GPIO_Pin_12 
#define ADVANCE_GPIOx  GPIOA
//========================================
//===============CAN初始化================
#define ADVANCE_CAN_Mode  CAN_Mode_Normal //普通模式
#define ADVANCE_CAN_SJW   CAN_SJW_1tq
#define ADVANCE_CAN_BS1   CAN_BS1_9tq
#define ADVANCE_CAN_BS2   CAN_BS2_8tq
#define ADVANCE_CAN_Fdiv  2
//==============================================
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t  all_ecd;
    int32_t  count;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;//电机的测量值

//===========电调ID设置=========================
typedef enum
{
    CAN_3508_CHASSIS_ALL_ID =0X200,	
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
}MC620_ID_Set;
//===============================================
#define get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
	else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;                    \
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                     \
}
//=========================
void Can_Init_Mode(void);
void Can_Init(void);
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
u8 CAN_TX_Msg(u8* Tx_BuFF,u8 len);//发送,tx发送数据，len:数据长度
u8 CAN_RX_Msg(u8* Rx_BuFF);//接收，rx接收数据
//=========================
#endif
