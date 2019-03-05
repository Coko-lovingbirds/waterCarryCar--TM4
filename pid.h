/*
 * pid.h
 *
 *  Created on: 2018��11��12��
 *      Author: CZW
 */

#ifndef PID_H_
#define PID_H_

typedef struct
{
    double KP;//����ϵ��
    double KI;//����ϵ��
    double KD;//΢��ϵ��
    double SET_Point;//�趨ֵ
    double Old_Position;//��һ�ε�λ��
    double Integration_Sum;//��������
    double P_TERM;//�������Ƶķ���ֵ
    double I_TERM;//���ֿ��Ƶķ���ֵ
    double D_TERM;//΢�ֿ��Ƶķ���ֵ
    double NOW_Error;//��ǰ���
    double LAST_Error;//��һ�����
}SPID;

extern double PID_Position_Type_Control(SPID *pid, double position);//λ����PID

#endif /* PID_H_ */
