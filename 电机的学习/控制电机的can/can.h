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

//=========================
void Can_Init_Mode(void);
void Can_Init(void);
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
u8 CAN_TX_Msg(u8* Tx_BuFF,u8 len);//发送,tx发送数据，len:数据长度
u8 CAN_RX_Msg(u8* Rx_BuFF);//接收，rx接收数据
//=========================
#endif
