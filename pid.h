/*
 * pid.h
 *
 *  Created on: 2018年11月12日
 *      Author: CZW
 */

#ifndef PID_H_
#define PID_H_

typedef struct
{
    double KP;//比例系数
    double KI;//积分系数
    double KD;//微分系数
    double SET_Point;//设定值
    double Old_Position;//上一次的位置
    double Integration_Sum;//误差积分项
    double P_TERM;//比例控制的返回值
    double I_TERM;//积分控制的返回值
    double D_TERM;//微分控制的返回值
    double NOW_Error;//当前误差
    double LAST_Error;//上一次误差
}SPID;

extern double PID_Position_Type_Control(SPID *pid, double position);//位置试PID

#endif /* PID_H_ */
