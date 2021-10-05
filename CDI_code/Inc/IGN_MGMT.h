/*
 * IGN_MGMT.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_IGN_MGMT_H_
#define INC_IGN_MGMT_H_

#include "stm32f1xx_hal.h"
#include "GENERAL_DEF.h"

void Cut_Igntion(void);
uint8_t Ignition_nTime(uint16_t eng_speed);
void Set_Pulse_Program(void);
void Engine_STOP_test(void);

#endif /* INC_IGN_MGMT_H_ */