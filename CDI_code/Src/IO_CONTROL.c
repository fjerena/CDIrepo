/*
 * IO_CONTROL.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "IO_CONTROL.h"
#include "MATH_LIB.h"

/*
void Turn_OFF_Int_input(void)
{
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
}

void Turn_ON_Int_input(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}
*/

uint32_t adcInputs[4]={0,0,0,0};
volatile sensors_measur sensors={0,0,0,0,0,0,0,0,0,0,0,0};  //All sensor variable related
	
//Hardware initialization
void Hardware_Init(void)
{
    Set_Output_LED_Green(OFF);
    Set_Output_LED_Red(OFF);
    Set_Output_LED_Blue(OFF);
    Set_Output_LED_Yellow(OFF);    
		Set_Ouput_Inversor(OFF);
		Set_Ouput_Trigger(OFF);
}

void Hardware_Test(void)
{
		//static uint8_t task=0, spark=0;
	  static uint8_t task=0;
		static uint8_t divisor=5;
	
	  divisor--;
	
	  if (divisor==0)
		{	
	
		
		switch(task)
		{	
			case 0:  Toggle_LED_Green();
							 task=1;
							 break;
			
			case 1:  Toggle_LED_Red();
							 task=2;
							 break;
			
			case 2:  Toggle_LED_Blue();
							 task=3;
							 break;
			
			case 3:  Toggle_LED_Yellow();	
							 task=0;
							 break;
			
			default: break;
		}	
		/*
		switch(spark)
		{	
			case 0:  Set_Ouput_Inversor(ON);
							 spark=1;
							 break;
			
			case 1:  Set_Ouput_Inversor(OFF);
							 spark=2;
							 break;
			
			case 2:  Set_Ouput_Trigger(ON);
							 spark=3;
							 break;
			
			case 3:  Set_Ouput_Trigger(OFF);
							 spark=0;
							 break;
			
			default: break;
		}	
		*/
		divisor=5;
		}
}	

//Led Green (Bluepill)
void Toggle_LED_Green(void)
{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void Set_Output_LED_Green(uint8_t	Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		}
}

//Led Red
void Toggle_LED_Red(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
}

void Set_Output_LED_Red(uint8_t	Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		}
}

//Led Blue
void Toggle_LED_Blue(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
}

void Set_Output_LED_Blue(uint8_t Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		}
}

//Led Yellow
void Toggle_LED_Yellow(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
}

void Set_Output_LED_Yellow(uint8_t Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		}
}

void Set_Ouput_Inversor(uint8_t Value)
{
    if (Value == ON)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
    }
}

void Set_Ouput_Trigger(uint8_t Value)
{
    if (Value == ON)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
    }
}

void Set_Ouput_InterruptionTest(void)
{
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
}

void HighVoltage(void)
{
		sensors.HighVoltRaw=adcInputs[0];
	  sensors.HighVolt=(sensors.HighVoltRaw*150)/4095;
}	

void BatteryVoltage(void)
{
    //Needs to apply a filter because the real circuit doesn´t have one...
	  sensors.VBatRaw=adcInputs[1];
    sensors.VBatFilt=Filter16bits(sensors.VBatFilt,sensors.VBatRaw,100u);
    sensors.VBat=(uint8_t)(((sensors.VBatFilt*347*455)/(4095*1000))+4);
}

void EngineTemp(void)
{
		sensors.EngineTempRaw=adcInputs[2];
    sensors.EngineTemp=(sensors.EngineTempRaw*150)/4095;	  
}

void BoardTemp(void)
{
    //Needs to apply a filter due the sensor characteristics
	  sensors.TempBoardRaw=adcInputs[3];
    sensors.TempBoardFilt=Filter16bits(sensors.TempBoardFilt,sensors.TempBoardRaw,80u);
    sensors.TempBoard=((V25-sensors.TempBoardFilt)/Avg_Slope)+25;
}

void Read_Analog_Sensors(void)
{	
	  HighVoltage();
		BatteryVoltage();
		EngineTemp();		
		BoardTemp();		
}	


