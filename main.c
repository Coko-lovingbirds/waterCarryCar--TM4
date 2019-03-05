/*
 * main.c
 *

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
#include "driverlib/timer.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "string.h"
#include "driverlib/timer.h"
#include "string.h"
#include "drv8825.h"
#include "Stree.h"
#include "uart1_receive.h"
#include "mpu6050_uart3.h"
#include "pid.h"

//SPID ANGLE1_PID={/*P*/1.12 ,/*I*/0 ,/*D*/0.00001 ,0,0,0,0,0,0,0,0};
//SPID ANGLE2_PID={/*P*/1.12 ,/*I*/0 ,/*D*/0.00001 ,0,0,0,0,0,0,0,0};

//bool PID_flag=false;


void ConfigureUART0(void)//串口
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, 16000000);
}


void Timer0BIntHandler(void)   //10ms执行一次
{
	uint32_t ui32Status;
//	static uint32_t PID_Counter=1;
	ui32Status=TimerIntStatus(TIMER0_BASE, true);
	TimerIntClear(TIMER0_BASE, ui32Status);
//    if(PID_Counter>5)
//    {
//            PID_Counter=1;
//            ANGLE1_PID.Integration_Sum = 0;
//            ANGLE2_PID.Integration_Sum = 0;
//            PID_flag=true;
//    }
//    PID_Counter++;
}


void DIDB_Timer0_Config(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM);
	TimerPrescaleSet(TIMER0_BASE,TIMER_B,16-1);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PERIODIC);//配置两个16位的半宽度
//	TimerLoadSet(TIMER0_BASE,TIMER_B,(SysCtlClockGet())/((TimerPrescaleGet(TIMER0_BASE,TIMER_B)+1)/100)-1);//装载值   10ms
	TimerLoadSet(TIMER0_BASE,TIMER_B,50000-1);//装载值   10ms
	TimerIntEnable(TIMER0_BASE,TIMER_TIMB_TIMEOUT);//计数到达装载值则中断
	TimerIntRegister(TIMER0_BASE, TIMER_B, Timer0BIntHandler);//注册
	TimerEnable(TIMER0_BASE,TIMER_B);//计数器跟随系统时钟
}


void LED_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3);
}

void stree_ag(void)
{
//	if(PID_flag == true)
//	{
//		PID_flag = false;
//		if(pitch >= 0)
//		{
//			stree_angle1((int)PID_Position_Type_Control(&ANGLE1_PID,pitch));
//			UARTprintf("pitch = %d    %d\t",(int)pitch,(int)PID_Position_Type_Control(&ANGLE1_PID,pitch));
//		}
//		if(pitch < 0)
//		{
//			stree_angle1(-(int)PID_Position_Type_Control(&ANGLE1_PID,-pitch));
//			UARTprintf("pitch = %d    %d\t",(int)pitch,(int)PID_Position_Type_Control(&ANGLE1_PID,-pitch));
//		}
//		if(roll >= 0)
//		{
//			stree_angle2(-(int)PID_Position_Type_Control(&ANGLE1_PID,roll));
//			UARTprintf("roll = %d    %d\n",(int)roll,(int)PID_Position_Type_Control(&ANGLE1_PID,roll));
//		}
//		if(roll < 0)
//		{
//			stree_angle2((int)PID_Position_Type_Control(&ANGLE1_PID,-roll));
//			UARTprintf("roll = %d    %d\n",(int)roll,(int)PID_Position_Type_Control(&ANGLE1_PID,-roll));
//		}

		stree_angle1(-(int)(pitch));	//1   左+右-
		stree_angle2((int)(roll));	//2    前-后+
//	}
}


void NVIC_Configure(void)
{
    IntPrioritySet(INT_UART1,0x10);
    IntPrioritySet(INT_UART3,0x20);
	IntPrioritySet(INT_TIMER0B ,0x00); //工程中的最高优先级
}


void main(void)
 {
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL |SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//80Mhz
    FPUEnable();//浮点运算
    FPULazyStackingEnable();
	IntMasterEnable();//使能总中断
    LED_Init();
    ConfigureUART0();
    Recive_UART1_Config();
    Recive_UART3_Config();
    DRV8825_Init();
    PWM0_PB6_PB7_Configure();
    DIDB_Timer0_Config();
    NVIC_Configure();

    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_0, 0x0);
    GPIOPinWrite(GPIO_PORTD_BASE,  GPIO_PIN_1, 0x0);
    PWMGenDisable(PWM1_BASE, PWM_GEN_1);
    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
    while(1)
    {
    	if(UART1_Updated_Flag)
    	{
    		Uart1_Data_Pros();
    	    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
    	    UART1_Updated_Flag = false;
    	}
    	Printf_MPU_Infos();

    	stree_ag();


    }
}
