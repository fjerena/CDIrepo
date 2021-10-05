/*
 * IO_CONTROL.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_IO_CONTROL_H_
#define INC_IO_CONTROL_H_

#include "stm32f1xx_hal.h"
#include "GENERAL_DEF.h"

extern TIM_HandleTypeDef htim4;

void Hardware_Init(void);
void Hardware_Test(void);
void Toggle_LED_Green(void);
void Set_Output_LED_Green(uint8_t	Value);
void Toggle_LED_Red(void);
void Set_Output_LED_Red(uint8_t	Value);
void Toggle_LED_Blue(void);
void Set_Output_LED_Blue(uint8_t Value);
void Toggle_LED_Yellow(void);
void Set_Output_LED_Yellow(uint8_t Value);
void Set_Ouput_Inversor(uint8_t Value);
void Set_Ouput_Trigger(uint8_t Value);
void Set_Ouput_InterruptionTest(void);

#endif /* INC_IO_CONTROL_H_ */

