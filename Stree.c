/*
 * Stree.c
 *
 *  Created on: 2018年10月18日
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
#include "Stree.h"
#include "drv8825.h"
#include "mpu6050_uart3.h"
#include "pid.h"
#define MIDAG1 85
#define MIDAG2 90


void PWM0_PB6_PB7_Configure(void)			// PB6 PB7
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 16666);
    PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT|PWM_OUT_1_BIT,true);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM0_BASE,PWM_GEN_0)*50/100);//number=Numb*%x B5
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE,PWM_GEN_0)*50/100);//number=Numb*%x B5
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

}

void stree_angle1(int ag)   //舵机1角度  左+右-
{
	if(ag> 45)
		ag = 45;
	if(ag<-45)
		ag=-45;
	ag = ag+MIDAG1;
	double wide;
	wide = ag/9.0+5.0;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (int)(PWMGenPeriodGet(PWM0_BASE,PWM_GEN_0)*wide/33.33));//number=Numb*%x B5

}

void stree_angle2(int ag)   //舵机2角度  前+后-
{
	if(ag> 45)
		ag = 45;
	if(ag<-45)
		ag=-45;
	ag = ag+MIDAG2;
	double wide;
	wide = ag/9.0+5.0;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (int)(PWMGenPeriodGet(PWM0_BASE,PWM_GEN_0)*wide/33.33));//number=Numb*%x B5
}

