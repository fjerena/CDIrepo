/*
 * USART_COMM.h
 *
 *  Created on: 31-Aug-2021
 *      Author: Jerena
 */

#ifndef INC_USART_COMM_H_
#define INC_USART_COMM_H_

/*Include to use in my code*/
#include "stm32f1xx_hal.h"
#include "GENERAL_DEF.h"

//Definition
enum Transmission_Status{TRANSMITING,TRANSMISSION_DONE};
extern enum Transmission_Status transmstatus;

enum Reception_Status{DATA_AVAILABLE_RX_BUFFER,RECEPTION_DONE};
extern enum Reception_Status receptstatus;

////UART Communication
//Will be available for all modules that include USART_COMM.h file...
extern uint8_t flgTransmition;
extern enum Transmission_Status transmstatus;
extern enum Reception_Status receptstatus;

/*
If I put extern in front of the variable that belongs to c file, another modules can access this variables
*/

//This is a special case, It was declared in main.c, but I will use in USART_COMM.c and if another module include USART_COMM.h
//will be available this variable to be use it, like an apropriation...
extern UART_HandleTypeDef huart1;
extern uint8_t UART1_rxBuffer[blockSize+2];

//In this case, all modules that include USART_COMM.h, these functions will be available, if they aren´t declared 
//compiler wil set a warning message 
void initializeCalibOnRAM(void);
void copyCalibUartToRam(void);
void copyCalibRamToUart(void);
void saveCalibRamToFlash(void);
void copyCalibFlashToRam(void);
void transmitCalibToUART(void);
void receiveData(void);
void systemInitialization(void);
void transmitSystemInfo(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* INC_USART_COMM_H_ */
