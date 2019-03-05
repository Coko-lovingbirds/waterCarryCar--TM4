/*
 * pid.c
 *
 *  Created on: 2018年11月12日
 *      Author: CZW
 */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "time.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "string.h"
#include "driverlib/timer.h"
#include "pid.h"
#include "string.h"
#include "drv8825.h"
#include "Stree.h"
#include "uart1_receive.h"
#include "mpu6050_uart3.h"

double PID_Position_Type_Control(SPID *pid, double position)//位置试PID
{
    pid->NOW_Error = pid->SET_Point - position;//求当前误差
    pid->Integration_Sum = pid->Integration_Sum + pid->NOW_Error;//积分项累加
    pid->LAST_Error = pid->SET_Point - pid->Old_Position;//求上次误差
    pid->Old_Position = position;
    pid->P_TERM = pid->KP * pid->NOW_Error;
    pid->I_TERM = pid->KI * pid->Integration_Sum;
    pid->D_TERM = pid->KD * ( pid->NOW_Error - pid->LAST_Error );
    return pid->P_TERM + pid->I_TERM + pid->D_TERM;
}


