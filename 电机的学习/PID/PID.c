//#include "sys.h"
//#include "PID.h"


///***********位置pid***********************/

//void PID_Set(PID* motor_type,	float Kp,	float Ki,	float Kd) //PID设定
//{
//	motor_type->Kp = Kp;
//	motor_type->Ki = Ki;
//	motor_type->Kd = Kd;
//}
// void PID_Reset(PID* motor_type) //PID清零
//{
//	motor_type->ref = 0;
//	motor_type->fdb = 0;
//	motor_type->Kp = 0;
//	motor_type->Ki = 0;
//	motor_type->Kd = 0;
//	motor_type->error_last = 0;
//	motor_type->error_now = 0;
//	motor_type->error_rate = 0;
//	motor_type->error_inter = 0;
//	motor_type->error_prev=0;
//	motor_type->pid_out = 0;
//}
//void PID_Speed_Calc(PID* motor_type) //速度计算
//{
//	
//    motor_type->error_now = motor_type->ref - motor_type->fdb;//现在的误差=角度值-转速
// 
//    motor_type->pid_out = motor_type->pid_out + motor_type->Kp * motor_type->error_now               //E[k]
//              - motor_type->Ki   * motor_type->error_last     //E[k-1]
//              + motor_type->Kd * motor_type->error_prev;   //E[k-2]
//  	
//    motor_type->error_prev = motor_type->error_last;//上上次的错误=上次错误
//    motor_type->error_last = motor_type->error_now;//上次的错误=现在的错误
//}

//int16_t PID_Control(PID* motor_type)  //PID的控制
//{
//	/*****float error_position*****/
//	motor_type->error_last=motor_type->error_now;
//	motor_type->error_now = motor_type->ref - motor_type->fdb;
//	motor_type->error_rate=motor_type->error_now-motor_type->error_last;//现在的错误-过去的错误
//	motor_type->error_inter += motor_type->error_now;//误差的积分
//	/*****limit intergration of pid*****/
//	if(motor_type->error_inter>Inter_Max)
//		  motor_type->error_inter = Inter_Max;
//	if(motor_type->error_inter<-Inter_Max)
//		  motor_type->error_inter = -Inter_Max;
//	
//	motor_type->pid_out = (motor_type->Kp * motor_type->error_now + motor_type->Ki * motor_type->error_inter +	motor_type->Kd * motor_type->error_last);
//	VAL_LIMIT(motor_type->pid_out, -5000, 5000);
//	VAL_LIMIT(motor_type->pid_out, -16384, 16384);
//	return (int16_t)motor_type->pid_out; //返回输出值
//}
///*********************增量式PID******************************/



//int16_t ANGLE_PID_Control(ANGLE_PID* motor_type)
//{
//	/*****float error_position*****/
//	motor_type->error_last=motor_type->error_now;
//	motor_type->error_now = motor_type->ref - motor_type->fdb;
//	motor_type->error_rate=motor_type->error_now-motor_type->error_last;
//	motor_type->error_inter += motor_type->error_now;
//	/*****limit intergration of pid*****/
//	if(motor_type->error_inter>Inter_Max)
//		  motor_type->error_inter = Inter_Max;
//	if(motor_type->error_inter<-Inter_Max)
//		  motor_type->error_inter = -Inter_Max;
//	
//	motor_type->pid_out = (motor_type->Kp * motor_type->error_now + motor_type->Ki * motor_type->error_inter +	motor_type->Kd * motor_type->error_last);
//	VAL_LIMIT(motor_type->pid_out, -5000, 5000);
//	VAL_LIMIT(motor_type->pid_out,-32767,32767);
//	return (int16_t)motor_type->pid_out;
//}
//int16_t SPEED_PID_Control(SPEED_PID* motor_type)
//{
//	/*****float error_position*****/
//	motor_type->error_last=motor_type->error_now;
//	motor_type->error_now = motor_type->ref - motor_type->fdb;
//	motor_type->error_rate=motor_type->error_now-motor_type->error_last;
//	motor_type->error_inter += motor_type->error_now;
//	/*****limit intergration of pid*****/
//	if(motor_type->error_inter>Inter_Max)
//		  motor_type->error_inter = Inter_Max;
//	if(motor_type->error_inter<-Inter_Max)
//		  motor_type->error_inter = -Inter_Max;
//	
//	motor_type->pid_out = (motor_type->Kp * motor_type->error_now + motor_type->Ki * motor_type->error_inter +	motor_type->Kd * motor_type->error_last);
//	VAL_LIMIT(motor_type->pid_out, -5000, 5000);
//	VAL_LIMIT(motor_type->pid_out,-32767,32767);
//	return (int16_t)motor_type->pid_out;
//}
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
  *                 			PID_DELTA: 	 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
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
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref; //错误的产生
    if (pid->mode == PID_POSITION)//普通PID
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)//差分PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}



