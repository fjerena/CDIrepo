/*
 * TIMER_FUNC.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_TIMER_FUNC_H_
#define INC_TIMER_FUNC_H_

#include "stm32f1xx_hal.h"

#define nTimer	16

enum TimerID{Timer0,Timer1,Timer2,Timer3,Timer4,Timer5,Timer6,Timer7,Timer8,Timer9,Timer10,Timer11,Timer12,Timer13,Timer14,Timer15};// TimerNumb;

typedef struct TimerStruct
{
    uint32_t target_time;
    uint8_t  output;
		void (*func_pointer)();
}timerSchedtype;

extern timerSchedtype timerList[nTimer];
extern uint8_t Cond0,Cond1,Cond2,Cond3,Cond4,Cond5,Cond6,Cond8;
extern uint32_t Counter0,Counter1,Counter2,Counter3,Counter4,Counter5,Counter6,Counter7,Counter8,Counter9,Counter10;

void setTimeoutHookUp(timerSchedtype timer_list[],enum TimerID timer,uint32_t period,void (*func)(void));
uint8_t checkTimeoutHookUp(timerSchedtype timer_list[],enum TimerID timer);
void TimerListManagement(timerSchedtype timer_list[]);
uint8_t Timeout_ms(uint8_t Condition,uint32_t *timer,uint32_t period);

#endif /* INC_TIMER_FUNC_H_ */
