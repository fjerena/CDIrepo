/*
 * SCHEDULLER.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_SCHEDULLER_H_
#define INC_SCHEDULLER_H_

#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef hadc1;

typedef struct Scheduler
{
    uint8_t  program;
    uint32_t target_time;
}sched_var;

extern sched_var array_sched_var[3];   //Scheduller

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos);

#endif /* INC_SCHEDULLER_H_ */
