//#ifndef __PID_CONTROL_H__
//#define __PID_CONTROL_H__
//#include "sys.h"

//#define Inter_Max   2000
//#define VAL_LIMIT(val, min, max)\
//if(val<=min)\
//{\
//	val = min;\
//}\
//else if(val>=max)\
//{\
//	val = max;\
//}\
////val   系统内部定时器的当前值寄存器

//typedef struct PID_PARAMETER //PID参数
//{
//	float ref;        //角度值
//	float fdb;         //转速
//	float Kp;
//	float Ki;
//	float Kd;
//	float error_now;         //误差
//	float error_last;        //上一次误差
//	float error_rate;        //误差变化率
//	float error_inter;       //误差积分
//	float error_prev;        //上上次误差
//	float pid_out;
//}PID;

//typedef struct ANGLE_PID_PARAMETER //PID角度参数
//{
//	float ref;
//	float fdb;
//	float Kp; 
//	float Ki;
//	float Kd;
//	float error_now;         //误差
//	float error_last;        //上一次误差
//	float error_rate;        //误差变化率
//	float error_inter;       //误差积分
//	float error_prev;        //上上次误差
//	float pid_out;
//}ANGLE_PID;

//typedef struct SPEED_PID_PARAMETER  //速度参数
//{
//	float ref;
//	float fdb;
//	float Kp;
//	float Ki;
//	float Kd;
//	float error_now;         //误差
//	float error_last;        //上一次误差
//	float error_rate;        //误差变化率
//	float error_inter;       //误差积分
//	float error_prev;        //上上次误差
//	float pid_out;
//}SPEED_PID;
//void PID_Set(PID* motor_type,	float Kp,	float Ki,	float Kd);
//int16_t PID_Control(PID* motor_type); 
//void PID_Reset(PID* motor_type);
//void PID_Speed_Calc(PID* motor_type);
//#endif
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
//#include "struct_typedef.h"
#include "stdint.h"
#define NULL 0




enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;//转速

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次，误差
    fp32 error[3]; //误差项 0最新 1上一次 2上上次，误差

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data  //反馈数据
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif

