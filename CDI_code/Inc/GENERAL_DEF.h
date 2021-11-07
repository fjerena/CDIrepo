/*
 * GENERAL_DEF.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_GENERAL_DEF_H_
#define INC_GENERAL_DEF_H_

#include "stm32f1xx_hal.h"

#define ON    1u
#define OFF   0u
#define TRUE  1u
#define FALSE	0u

//Diagnostic
#define fail_0  0x01  // fail_0
#define fail_1  0x02  // fail_1
#define fail_2  0x04	// fail_2
#define fail_3  0x08	// fail_3
#define fail_4  0x10	// fail_4
#define fail_5  0x20	// fail_5
#define fail_6  0x40  // fail_6
#define fail_7  0x80	// fail_7

/*
I need to include shiftlight trigger event
Engine Speed limit
Parameterize the sensor
*/

typedef struct
{
    uint8_t  sensorAngDisplecement;
    uint16_t Max_Engine_Speed;
    uint16_t BP_Engine_Speed[12];
    uint8_t  BP_Timing_Advance[12];
    uint8_t  alpha;
    uint8_t  beta;
    uint8_t  gamma;
}dataCalibration;  

#define blockSize sizeof (dataCalibration)
	
typedef union 
{
    dataCalibration Calibration_RAM;
    uint32_t array_Calibration_RAM[blockSize>>2];   //Divided in 4 (32/4 = 8 byte)	  
    uint8_t array_Calibration_RAM_UART[blockSize];
}calibrationBlock;

extern calibrationBlock calibFlashBlock;
extern const calibrationBlock Initial_Calibration;

typedef struct
{
		uint16_t maxEngineSpeed;
		uint8_t  minVoltage;
	  uint8_t  maxVoltage;
	  uint8_t  diagCode;
	  uint8_t  resetCause;
}systemInfo;

#define blockSize2 (sizeof(systemInfo)%4)==0?sizeof(systemInfo):((sizeof(systemInfo)/4)+1)*4

typedef union 
{
    systemInfo systemInfo_RAM;
    uint32_t array_systemInfo_RAM[blockSize2>>2];   //Divided in 4 (32/4 = 8 byte)	  
    uint8_t array_systemInfo_RAM_UART[blockSize2];
}systemInfoBlock;

extern systemInfoBlock sysInfoBlock;
extern const systemInfoBlock Initial_SystemInfo;

typedef struct system_info
{
    uint16_t Engine_Speed_old;
    uint16_t Engine_Speed;
    uint16_t engineSpeedPred;
    uint16_t engineSpeedFiltered;
    uint16_t avarageEngineSpeed;
    int32_t  deltaEngineSpeed;
    uint32_t Rising_Edge_Counter;
    uint32_t triggerEventCounter;
    uint32_t inversorEventCounter;
    uint32_t Measured_Period;
    uint32_t TDuty_Input_Signal;
    uint32_t tdutyInputSignalPred;
    uint32_t tdutyInputSignalPredLinear;
    uint8_t  sensorAngDisplecementMeasured;
    uint32_t TStep;
    uint8_t  nAdv;
    uint8_t  Cutoff_IGN;
    uint8_t  Update_calc;
    uint32_t nOverflow;
    uint8_t  nOverflow_RE;
    uint8_t  nOverflow_FE;
}system_vars;

extern volatile system_vars scenario;

void Set_Diagnose(uint8_t diagnose);
void Clear_Diagnose(uint8_t diagnose);
void Clear_All_Diagnoses(void);

#endif /* INC_GENERAL_DEF_H_ */
