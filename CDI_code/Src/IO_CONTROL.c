/*
 * IO_CONTROL.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "IO_CONTROL.h"
#include "MATH_LIB.h"
#include "TIMER_FUNC.h"

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
uint16_t voltageArray[12] = {0, 130, 300, 500, 1000, 1400, 1800, 2600, 3000, 3500, 3700, 4020};
uint8_t temp[12] = {255, 200, 180, 140, 100, 25, 21, 15, 10, 5, 3, 0};

uint32_t contador,contador1;
uint8_t cond=TRUE;
uint8_t sequence,sequence1;	
uint8_t times;

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

/*
	  if(Timeout_ms(cond,&contador,10000)==TRUE)
		{
				Set_Output_LED_Green(ON);				
		}
		
		if(Timeout_ms(cond,&contador1,20000)==TRUE)
		{
				Set_Output_LED_Green(OFF);
		}	
		*/

void ledTest(void)
{	  	
		switch(sequence)
		{
				case 0:  Set_Output_LED_Green(ON);
								 sequence=1;
								 break;
			
				case 1:  Set_Output_LED_Green(OFF);
								 sequence=2;
								 break;
			
				case 2:  Set_Output_LED_Red(ON);
								 sequence=3;
								 break;
			
				case 3:  Set_Output_LED_Red(OFF);
								 sequence=4;
								 break;
			
				case 4:  Set_Output_LED_Blue(ON);
								 sequence=5;
								 break;
			
				case 5:  Set_Output_LED_Blue(OFF);
								 sequence=6;
								 break;
								
			  case 6:  Set_Output_LED_Yellow(ON);
								 sequence=7;
								 break;
			
				case 7:  Set_Output_LED_Yellow(OFF);
								 sequence=8;
								 break;
				
				default: break;
		}				
}

/*
The idea of this test is turn on the inversor, measure the voltage on Capacitor (we expect something
greater than 70V), discharge the capacitor and measure again the voltage on capacitor and we expect
to remain something less than 10V
*/
void sparkTest(void)
{
		switch(sequence1)
		{	
				case 0:  Set_Ouput_Inversor(ON);
								 sequence1=1;
								 break;
			
				case 1:  Set_Ouput_Inversor(OFF);
								 sequence1=2;
								 break;
			
				case 2:	 if(sensors.HighVolt<70u)
								 {
										Set_Diagnose(fail_2);
								 }
								 sequence1=3;
								 break;
			
				case 3:  Set_Ouput_Trigger(ON);
			  				 sequence1=4;
		  					 break;
			
				case 4:  Set_Ouput_Trigger(OFF);
				         sequence1=5;
							   break;	

				case 5:	 if(sensors.HighVolt>10u)
								 {
										Set_Diagnose(fail_3);
								 }
								 sequence1=6;
								 break;
			
				default: break;
		}		
}

void manageDignosticLED(void)
{
		if(sysInfoBlock.systemInfo_RAM.diagCode>0)
		{
				Set_Output_LED_Yellow(ON);
		}	
		else		
		{
				Set_Output_LED_Yellow(OFF);
		}
}	

void blinkCommunicationLED(uint8_t nblink)
{
		uint8_t *var;
	
		var=&times;
		*var=nblink;
}
	
void manageCommunicationLED(void)
{
		static uint8_t counter,order;
	  uint8_t target;
	
		target=times*2;
	
		if(target!=counter)
		{	
				switch(order)
				{
						case 0:  Set_Output_LED_Blue(ON);
										 order=1;
							       break;
			
				    case 1:  Set_Output_LED_Blue(OFF);
						    		 order=0;
							       break;
			
			      default: break;
		    }
				
				counter++;
    }
		else
		{
				counter=0;
				times=0;
		}
}	
	
//Led Green (Bluepill)
void Toggle_LED_Green(void)
{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void BlinkLEDEcuAlive(void)
{
		Toggle_LED_Green();
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
		sensors.HighVoltFilt=Filter16bits(sensors.HighVoltFilt,sensors.HighVoltRaw,60u);
	  sensors.HighVolt=(sensors.HighVoltFilt*150)/4095;
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
	  sensors.EngineTempFilt=Filter16bits(sensors.EngineTempFilt,sensors.EngineTempRaw,100u);
    sensors.EngineTemp=linearInterpolation(sensors.EngineTempRaw,voltageArray,temp);
}

void BoardTemp(void)
{
		//Enable temperature sensor
		//ADC_CR2_TSVREFE;
	  //HAL_IS_BIT_SET(hadc1.Instance->CR2, ADC_CR2_TSVREFE);
	
    //Needs to apply a filter due the sensor characteristics
	  sensors.TempBoardRaw=adcInputs[3];
    sensors.TempBoardFilt=Filter16bits(sensors.TempBoardFilt,sensors.TempBoardRaw,80u);
    //sensors.TempBoard=((V25-sensors.TempBoardFilt)/Avg_Slope)+25;
		sensors.TempBoard=((V25-(330*sensors.TempBoardRaw)/4095)/Avg_Slope)+25;	
	
	  //HAL_IS_BIT_CLR(hadc1.Instance->CR2, ADC_CR2_TSVREFE);
}

void Read_Analog_Sensors(void)
{	
	  HighVoltage();
		BatteryVoltage();
		EngineTemp();		
		BoardTemp();		
}	
