/*
 * uart2_receive.c
 *
 *  Created on: 2018Äê10ÔÂ20ÈÕ
 *      Author: wmy
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
#include "string.h"
#include "uart1_receive.h"
#include "drv8825.h"
#include "Stree.h"
#include "mpu6050_uart3.h"

#define ADDRESS_ID 0xA1

char UART1_Rx_Buffers[50];
char UART1_Rx_Len;
char UART1_Sender_Address;
char UART1_Rx_Data[50];
bool UART1_Updated_Flag=false;

void Recive_UART1_Config(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0| GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE,SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

//STX SID RID LEN DATA ETX
//eg EA AD A1 01 33 EF

void UART1IntHandler(void)
{
    uint32_t ui32Status;
    char rx_buffer;
    static bool receive_flag=false;
    static int index=0;
    ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);
    while(UARTCharsAvail(UART1_BASE))
    {
        rx_buffer = (uint8_t)(UARTCharGetNonBlocking(UART1_BASE));
        if(rx_buffer==0xEA)
        {
            receive_flag = true;
            index = 0;
            memset(UART1_Rx_Buffers,0,50);
        }
        if((index-1)>=2)
        {
            if(UART1_Rx_Buffers[2]!=ADDRESS_ID)
            {
                receive_flag = false;
                index = 0;
                memset(UART1_Rx_Buffers,0,50);
            }
        }
        if(receive_flag==true)
        {
            UART1_Rx_Buffers[index++] = rx_buffer;
            if(rx_buffer==0xEF)
            {
                receive_flag = false;
                if(index==5+UART1_Rx_Buffers[3])
                {
                    index = 0;
                    UART1_Sender_Address = UART1_Rx_Buffers[1];
                    UART1_Rx_Len = UART1_Rx_Buffers[3];
                    memset(UART1_Rx_Data,0,50);
                    memcpy(UART1_Rx_Data, UART1_Rx_Buffers+4, UART1_Rx_Len);
                    UART1_Updated_Flag = true;
                }
                else
                {
                    index = 0;
                    memset(UART1_Rx_Buffers,0,50);
                }
            }
        }
    }
}

void Uart1_Data_Pros(void)
{
    if(UART1_Updated_Flag==true)
    {
    	UART1_Updated_Flag=false;
        if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0x11))                //RunAhead
        {
        	DRV8825_Motor1_Control(500, '+');
        	DRV8825_Motor2_Control(500, '+');
        }
        else if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0x22))          //RunRear
        {
        	DRV8825_Motor1_Control(500, '-');
        	DRV8825_Motor2_Control(500, '-');

        }
        else if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0x33))            //RunRight
        {
        	DRV8825_Motor1_Control(500, '+');
//        	DRV8825_Motor2_Control(500, '-');
        	PWMGenDisable(PWM1_BASE, PWM_GEN_1);

        }
        else if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0x44))            //RunLeft
        {
//        	DRV8825_Motor1_Control(500, '-');
        	DRV8825_Motor2_Control(500, '+');
        	PWMGenDisable(PWM0_BASE, PWM_GEN_1);
        }

        else if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0x00))          //stay
        {
        	PWMGenDisable(PWM1_BASE, PWM_GEN_1);
        	PWMGenDisable(PWM0_BASE, PWM_GEN_1);

        }
        if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0xA1))                //speedup1
        {
        	DRV8825_Motor1_Control(750, '+');
        	DRV8825_Motor2_Control(750, '+');

        }
        if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0xA2))                //speedup2
        {
        	DRV8825_Motor1_Control(750, '-');
        	DRV8825_Motor2_Control(750, '-');
        }

        if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0xB1))                //speeddown1
        {
        	DRV8825_Motor1_Control(250, '+');
        	DRV8825_Motor2_Control(250, '+');
        }
        if((UART1_Rx_Buffers[3]==0x01) && (UART1_Rx_Buffers[4]==0xB2))                //speeddown2
        {
        	DRV8825_Motor1_Control(250, '-');
        	DRV8825_Motor2_Control(250, '-');
        }

        if((UART1_Rx_Buffers[3]==0x02) && (UART1_Rx_Buffers[4]==0x11))                //speeddown4
        {
        	Angle_Run(720, '+');
        }

        if((UART1_Rx_Buffers[3]==0x02) && (UART1_Rx_Buffers[4]==0x44))                //speeddown4
        {
        	Rotation_Angle_Motor1(360, '-');
        	Rotation_Angle_Motor2(0, '+');
        }

        if((UART1_Rx_Buffers[3]==0x02) && (UART1_Rx_Buffers[4]==0x55))                //speeddown4
        {
     		Angle_Run(720*2, '+');
          	Rotation_Angle_Motor1(360, '-');
           	Rotation_Angle_Motor2(0, '+');
          	Angle_Run(720*2, '-');
          	Rotation_Angle_Motor1(360, '+');
          	Rotation_Angle_Motor2(0, '-');
          	Angle_Run(720*2, '+');

        }

    }
}




