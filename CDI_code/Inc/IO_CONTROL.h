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

#define V25      150
#define Avg_Slope  1

extern ADC_HandleTypeDef hadc1;

typedef struct sensors_measurement
{
		uint16_t VBatRaw;                      
    uint16_t VBatFilt;                     
    uint8_t  VBat; 

  	uint16_t TempBoardRaw;                 
    uint16_t TempBoardFilt;                
    uint8_t  TempBoard;		                       

    uint16_t EngineTempRaw;                
    uint16_t EngineTempFilt;               
    uint8_t  EngineTemp;   

		uint16_t HighVoltRaw;                
    uint16_t HighVoltFilt;               
    uint8_t  HighVolt;
}sensors_measur;

extern uint32_t adcInputs[4];
extern volatile sensors_measur sensors;	

extern TIM_HandleTypeDef htim4;

void Hardware_Init(void);
void Hardware_Test(void);
void ledTest(void);
void sparkTest(void);
void manageDignosticLED(void);
void blinkCommunicationLED(uint8_t nblink);
void manageCommunicationLED(void);
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HighVoltage(void);
void BatteryVoltage(void);
void EngineTemp(void);
void BoardTemp(void);
void Read_Analog_Sensors(void);

#endif /* INC_IO_CONTROL_H_ */

