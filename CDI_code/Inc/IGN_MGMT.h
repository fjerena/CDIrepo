/*
 * IGN_MGMT.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_IGN_MGMT_H_
#define INC_IGN_MGMT_H_

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;

void Cut_Igntion(void);
uint8_t Ignition_nTime(uint16_t eng_speed);
void Set_Pulse_Program(void);
void Engine_STOP_test(void);
void Pulse_Generator_Scheduler(void);
void TurnOffAllPulseInt(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Rising_Edge_Event(void);
void Falling_Edge_Event(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Treat_Int(uint8_t program);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_IGN_MGMT_H_ */
