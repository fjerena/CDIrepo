/*
 * USART_COMM.c
 *
 *  Created on: 31-Aug-2021
 *      Author: Jerena
 */


#include "USART_COMM.h"
#include "FLASH_PAGE.h"
#include "IGN_MGMT.h"
#include "main.h"
//Running test
#include <stdio.h>
#include "IO_CONTROL.h"

//Local definition (if I want to share this variables with another modules, I need to include in header file extern + variable name
/*****************************/
/*                           */
/* -Defines                  */
/* -Variables                */
/* -Initial settings         */
/*                           */
/*****************************/

TIM_IC_InitTypeDef sConfigIC;
uint8_t flgTransmition;
enum Transmission_Status transmstatus=TRANSMISSION_DONE;
enum Reception_Status receptstatus=RECEPTION_DONE;
uint8_t UART1_txBuffer[57];
uint8_t UART1_rxBuffer[blockSize+1];
uint32_t refAddress=flashAddress;  //Address that where the calibration will be stored
char buffer[51];
uint8_t comErrorDetected;

//These function will be available for all modules, but if I don´t declare in header file, compiler will set warning messages
/*****************************/
/*                           */
/*  Function definition      */
/*                           */
/*****************************/

void initializeCalibOnRAM(void)
{
    uint32_t i;

    for(i=0;i<sizeof(dataCalibration);i++)
    {
			  calibFlashBlock.array_Calibration_RAM_UART[i] = Initial_Calibration.array_Calibration_RAM_UART[i];
    }
}

void copyCalibUartToRam(void)	
{
    uint32_t i;

    for(i=0;i<sizeof(dataCalibration);i++)
    {
        calibFlashBlock.array_Calibration_RAM_UART[i] = UART1_rxBuffer[i+1];
    }
}

void copyCalibRamToUart(void)
{
    uint32_t i;

    for(i=0;i<blockSize;i++)
    {
        UART1_rxBuffer[i] = calibFlashBlock.array_Calibration_RAM[i];
    }
}

void saveCalibRamToFlash(void)
{
		Flash_Write_Data (refAddress, calibFlashBlock.array_Calibration_RAM, (sizeof(calibFlashBlock.array_Calibration_RAM))>>2);		
}

void copyCalibFlashToRam(void)
{
    Flash_Read_Data (refAddress, calibFlashBlock.array_Calibration_RAM);		
}

void initializeSysInfoRAM(void)
{
    uint32_t i;

    for(i=0;i<sizeof(systemInfo);i++)
    {
			  sysInfoBlock.array_systemInfo_RAM_UART[i] = Initial_SystemInfo.array_systemInfo_RAM_UART[i];
    }
}

void saveSystemData(void)
{
		Flash_Write_Data (0x0800F420, sysInfoBlock.array_systemInfo_RAM, (sizeof(sysInfoBlock.array_systemInfo_RAM))>>2);		
}

void copySystemInfoFlashToRam(void)
{
    Flash_Read_Data (0x0800F420, sysInfoBlock.array_systemInfo_RAM);		
}

void transmitCalibToUART(void)
{
    uint32_t i;
    uint8_t checksum;
    uint32_t buffer_length;

    if(transmstatus == TRANSMISSION_DONE)
    {
        buffer_length = sizeof(UART1_txBuffer);
			
				for(i=0;i<buffer_length;i++)
				{
						UART1_txBuffer[i] = 0x00;
			  }
			
        UART1_txBuffer[0] = 0x96;
				
				for (i=1;i<45;i++)
        {
            UART1_txBuffer[i] = calibFlashBlock.array_Calibration_RAM_UART[i-1];            
        }
				
        checksum = UART1_txBuffer[0];

        for (i=1;i<buffer_length-2;i++)
        {
            checksum += UART1_txBuffer[i];
        }

        UART1_txBuffer[buffer_length-2] = checksum;
				UART1_txBuffer[buffer_length-1] = '\n';
				
        transmstatus = TRANSMITING;
        HAL_UART_Transmit_DMA(&huart1, UART1_txBuffer, sizeof(UART1_txBuffer));
    }
}

void transmitsysInfoBlockToUART(void)
{
    uint32_t i;
    uint8_t checksum;
    uint32_t buffer_length;

    if(transmstatus == TRANSMISSION_DONE)
    {
        buffer_length = sizeof(UART1_txBuffer);
			
				for(i=0;i<buffer_length;i++)
				{
						UART1_txBuffer[i] = 0x00;
			  }
				
				UART1_txBuffer[0] = 0x45;
				
				for(i=1;i<9;i++)
				{
						UART1_txBuffer[i] = sysInfoBlock.array_systemInfo_RAM_UART[i-1];
				}
			
        checksum = UART1_txBuffer[0];

        for (i=1;i<buffer_length-2;i++)
        {
            checksum += UART1_txBuffer[i];
        }

        UART1_txBuffer[buffer_length-2] = checksum;
				UART1_txBuffer[buffer_length-1] = '\n';
				
        transmstatus = TRANSMITING;
        HAL_UART_Transmit_DMA(&huart1, UART1_txBuffer, sizeof(UART1_txBuffer));
    }
}

void receiveData(void)
{
    uint8_t command;
    uint8_t checksum;
    uint32_t buffer_length;
    uint32_t i;
	  
    if(receptstatus == DATA_AVAILABLE_RX_BUFFER)
    {
				buffer_length = sizeof(UART1_rxBuffer);

			  checksum = 0;
			
        for(i=0;i<buffer_length-1;i++)
        {
            checksum += UART1_rxBuffer[i];
        }

        if((UART1_rxBuffer[buffer_length-1]-checksum) == 0u)				
        {
            command = UART1_rxBuffer[0];

						switch(command)
            {
								case 0x02:  flgTransmition = ON;
														blinkCommunicationLED(1);
                            break;
							
								case 0x03:  flgTransmition = OFF;
														blinkCommunicationLED(2);
                            break;
							
								case 0x47:  copyCalibUartToRam();	
									          saveCalibRamToFlash();		
														blinkCommunicationLED(4);
							              break;
							
								case 0x50:  Clear_Diagnose(fail_0);
														blinkCommunicationLED(1);
														break;
							
								case 0x51:  Clear_Diagnose(fail_1);
														blinkCommunicationLED(2);
														break;
							
								case 0x52:  Clear_All_Diagnoses();					
														blinkCommunicationLED(3);
														break;
							
								case 0x53:  saveSystemData();              //Record the sysInfoBlock structure to flash
														blinkCommunicationLED(5);
														break;	

								case 0x54:  transmitsysInfoBlockToUART();  //Read sysInfoBlock data by rs232
														blinkCommunicationLED(3);
														break;
							
                case 0x69:  transmitCalibToUART();         //Read the calibration by rs232
														blinkCommunicationLED(1);
                            break;								
													
							  case 0x7E:  copyCalibUartToRam();
														blinkCommunicationLED(3);
                            break;  

								case 0x7F:  saveCalibRamToFlash();
														blinkCommunicationLED(2);
                            break;

                default:    break;
            }
        }

        receptstatus = RECEPTION_DONE;
    }
}

void memoryInitialization(void)
{	
	  copyCalibFlashToRam();
	
    //Identify if there are some data recorded
    if (calibFlashBlock.Calibration_RAM.Max_Engine_Speed == 0u)
    {
				initializeCalibOnRAM();  
        saveCalibRamToFlash();      
    }
	  		
    copySystemInfoFlashToRam();
		
		if(sysInfoBlock.systemInfo_RAM.minVoltage == 0u)
		{	
				initializeSysInfoRAM();
				saveSystemData();
		}			
}

void overwriteIntEdgeFromCalib(void)
{
		if(calibFlashBlock.Calibration_RAM.Edge==1)
		{
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		}
		else
		{
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		}
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 3;
		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
		{
				Error_Handler();
		}
		if(calibFlashBlock.Calibration_RAM.Edge==1)
		{
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		}
		else
		{
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		}
		sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
		sConfigIC.ICFilter = 0;
		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
		{
				Error_Handler();
		}
}

void transmitSystemInfo(void)
{
	  /*
		To identify the frame "  " (two spaces)  //  2 char
		scenario.Engine_Speed                    //S 7 char "S12000 "
		scenario.engineSpeedPred                 //R 7 char "S12100 "
	  scenario.engineSpeedFiltered;            //F 7 char "S12050 "
	  scenario.nAdv                            //A 5 char "A064 "
	  sensors.VBat                             //B 5 char "B120 "
		sensors.HighVolt                         //H 5 char "H125 "
		sensors.EngineTemp                       //E 5 char "E021 "
	  sensors.TempBoard                        //T 5 char "T022 "   
    scenario.sensorAngDisplecementMeasured;  //N 5 char "N028 "
	  last " "	(space   )    								 //  1 char 
		ID + 54 char + checksum + \n (new line)
	  */
	
		uint32_t i;
    uint8_t checksum;
    uint32_t buffer_length;

    buffer_length = sizeof(UART1_txBuffer);

		sprintf((char *)UART1_txBuffer,"  S%0.5u R%0.5u F%0.5u A%0.3u B%0.3u H%0.3u E%0.3u T%0.3u N%0.3u ",scenario.Engine_Speed,scenario.engineSpeedPred,scenario.engineSpeedFiltered,scenario.nAdv,sensors.VBat,sensors.HighVolt,sensors.EngineTemp,sensors.TempBoard,scenario.sensorAngDisplecementMeasured);
    UART1_txBuffer[0] = 0xFA;
		
    checksum = UART1_txBuffer[0];

    for (i=1;i<buffer_length-2;i++)
    {
				checksum += UART1_txBuffer[i];
    }

    UART1_txBuffer[buffer_length-2] = checksum;	
		UART1_txBuffer[buffer_length-1] = '\n';		

    transmstatus = TRANSMITING;
    HAL_UART_Transmit_DMA(&huart1, UART1_txBuffer, sizeof(UART1_txBuffer));				
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    transmstatus = TRANSMISSION_DONE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_rxBuffer, sizeof(UART1_rxBuffer));
    receptstatus = DATA_AVAILABLE_RX_BUFFER;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    comErrorDetected++;
		Set_Diagnose(fail_4);
}

void Variables_Init(void)
{
		//Communication
    transmstatus = TRANSMISSION_DONE;
    receptstatus = RECEPTION_DONE;	
		flgTransmition = OFF;	
		comErrorDetected = 0;
}
