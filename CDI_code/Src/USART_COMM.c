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

//Local definition (if I want to share this variables with another modules, I need to include in header file extern + variable name
/*****************************/
/*                           */
/* -Defines                  */
/* -Variables                */
/* -Initial settings         */
/*                           */
/*****************************/

TIM_IC_InitTypeDef sConfigIC;
uint8_t flgTransmition=ON;
enum Transmission_Status transmstatus=TRANSMISSION_DONE;
enum Reception_Status receptstatus=RECEPTION_DONE;
uint8_t UART1_txBuffer[6];
uint8_t UART1_rxBuffer[blockSize+1];

uint8_t UART1_rxBufferAlt[6];
uint32_t refAddress=flashAddress;  //Address that where the calibration will be stored


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

        UART1_txBuffer[0] = 0x7E;
        checksum = UART1_txBuffer[0];

        for (i=1;i<buffer_length-2;i++)
        {
            UART1_txBuffer[i] = calibFlashBlock.array_Calibration_RAM_UART[i-1];
            checksum += UART1_txBuffer[i];
        }

        UART1_txBuffer[buffer_length-1] = checksum;
        transmstatus = TRANSMITING;
        HAL_UART_Transmit_DMA(&huart1, UART1_txBuffer, sizeof(UART1_txBuffer));
    }
}

/*

//I need to develop this function!!!

void transmitSystemData(void)
{
		uint32_t i;
    uint8_t checksum;
    uint32_t buffer_length;

    if(transmstatus == TRANSMISSION_DONE)
    {
			
				buffer_length = sizeof(UART1_txBuffer);

        UART1_txBuffer[0] = 0x7E;
        checksum = UART1_txBuffer[0];

        for (i=1;i<buffer_length-2;i++)
        {
            UART1_txBuffer[i] = calibFlashBlock.array_Calibration_RAM_UART[i-1];
            checksum += UART1_txBuffer[i];
        }

        UART1_txBuffer[buffer_length-1] = checksum;
        transmstatus = TRANSMITING;
        HAL_UART_Transmit_DMA(&huart1, UART1_txBuffer, sizeof(UART1_txBuffer));
				
    }		
}

*/

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
                            break;
							
								case 0x03:  flgTransmition = OFF;
                            break;
							
								case 0x47:  copyCalibUartToRam();	
									          saveCalibRamToFlash();
							
														//On test
														saveSystemData();
							              break;
							
                case 0x69:  transmitCalibToUART();
                            break;
							
							  case 0x7E:  copyCalibUartToRam();
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

/*
void transmitSystemInfo(void)
{
    uint8_t Mil, Cent, Dez, Unid;
    uint16_t Man, num, num1;
	  uint8_t checksum;
	  uint32_t i;

    num = scenario.Engine_Speed;
    num1 = scenario.Engine_Speed;  // only for test purpose DELETE!!!

    if((num<=9999u)&&(num1<=999u))
    {
        Mil = (num/1000u)+0x30;
        Man = num%1000u;
        Cent = (Man/100u)+0x30;
        Man = Man%100u;
        Dez = (Man/10u)+0x30;
        Unid = (Man%10u)+0x30;

        UART1_rxBufferAlt[0]='R';
        UART1_rxBufferAlt[1]=Mil;
        UART1_rxBufferAlt[2]=Cent;
        UART1_rxBufferAlt[3]=Dez;
        UART1_rxBufferAlt[4]=Unid;
        UART1_rxBufferAlt[5]='A';

        Cent = (num1/100u)+0x30;
        Man = num1%100u;
        Dez = (Man/10u)+0x30;
        Unid = (Man%10u)+0x30;

        UART1_rxBufferAlt[6]=Cent;
        UART1_rxBufferAlt[7]=Dez;
        UART1_rxBufferAlt[8]=Unid;        

        for(i=0; i < sizeof(UART1_rxBufferAlt)-3; i++)
				{
						checksum += UART1_rxBufferAlt[i];
				}	
				
				UART1_rxBufferAlt[9]=checksum; 
				UART1_rxBufferAlt[10]=0x0A; // '\n' - Line feed

        transmstatus = TRANSMITING;

        HAL_UART_Transmit_DMA(&huart1, UART1_rxBufferAlt, sizeof(UART1_rxBufferAlt));
		}		
}
*/

/*
void transmitSystemInfo(void)
{
    UART1_txBuffer[0]='F';
    UART1_txBuffer[1]='a';
    UART1_txBuffer[2]='b';
    UART1_txBuffer[3]='i';
    UART1_txBuffer[4]='o';
	  //UART1_txBuffer[5]='J';
    UART1_txBuffer[5]='\n';

    transmstatus = TRANSMITING;
    HAL_UART_Transmit_DMA(&huart1, UART1_txBuffer, sizeof(UART1_txBuffer));				
}
*/

void transmitSystemInfo(void)
{
		uint8_t Mil, Cent, Dez, Unid;
    uint16_t Man, num;
	  	
		num = scenario.Engine_Speed;
	
		Mil = (num/1000u)+0x30;
    Man = num%1000u;
    Cent = (Man/100u)+0x30;
    Man = Man%100u;
    Dez = (Man/10u)+0x30;
    Unid = (Man%10u)+0x30;
	
		UART1_txBuffer[0]='S';
    UART1_txBuffer[1]=Mil;
    UART1_txBuffer[2]=Cent;
    UART1_txBuffer[3]=Dez;
    UART1_txBuffer[4]=Unid;
    UART1_txBuffer[5]='\n';

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
    static int8_t k;

    k++;
}

void Variables_Init(void)
{
		//Communication
    transmstatus = TRANSMISSION_DONE;
    receptstatus = RECEPTION_DONE;	
		flgTransmition=ON;
}
