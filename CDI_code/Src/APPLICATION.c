/*
 * APPLICATION.c
 *
 *  Created on: 16-Oct-2021
 *      Author: Jerena
 */

#include "APPLICATION.h"
#include "SCHEDULLER.h"
#include "MATH_LIB.h"
#include "IO_CONTROL.h"
#include "USART_COMM.h"
#include "IGN_MGMT.h"

void Task_Fast(void)
{
    //HAL_IWDG_Init(&hiwdg);
	  Hardware_Test();	
}

void Task_Medium(void)
{    
		Read_Analog_Sensors();
	
    Cut_Igntion();
    receiveData();

    if(flgTransmition)
    {
        transmitSystemInfo();
    }

    Statistics();		
}

void Task_Slow(void)
{
	  Engine_STOP_test();
}

void Running_Scheduller(void)
{
		//Update the pulse calc scenario
    if (scenario.Update_calc == TRUE)
    {
        Set_Pulse_Program();
        scenario.Update_calc = FALSE;
    }

    //Scheduler
    Periodic_task(  20,&Task_Fast,   array_sched_var, 0);
    Periodic_task( 100,&Task_Medium, array_sched_var, 1);
    Periodic_task(1000,&Task_Slow,   array_sched_var, 2);
}
