/*
 * GEN_DEF.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_GEN_DEF_H_
#define INC_GEN_DEF_H_

#include "stm32f1xx_hal.h"

#define nSteps                        64u
#define TDutyTriggerK               1309u     //1.0ms for clock 72MHz
#define TIntervPulseInv              655u     //0.5ms
#define TPulseInv                   4582u     //3,5ms
#define EngineSpeedPeriod_Min     785455u     //100rpm
#define EngineSpeedPeriod_Max       5236u     //15000rpm
#define TMR2_16bits                65536u
#define RPM_const               78545455u

#define FALSE                          0u
#define TRUE                           1u
#define OFF                            0u
#define ON                             1u

//Global variable
enum Interruption_type{INT_FROM_CH1,INT_FROM_CH2,INT_FROM_CH3,INT_FROM_CH4};
extern enum Interruption_type int_types;

enum Event_status{EMPTY,PROGRAMMED};
extern enum Event_status status;

enum Engine_States{STOPPED,CRANKING,ACCELERATION,STEADY_STATE,DECELERATION,OVERSPEED};
extern enum Engine_States engstates;

enum engineSpeed{LOW,HIGH};
extern enum engineSpeed pulseMngmt;

/*
2 different speed
Low  <  1200                                  // fixed advance ignition
High >= First breakpoint(with restriction %)  // according igntion map
%Due the sw conception, the first break point must be >= 1200rpm
Engine Speed: Stopped, Acceleration, Steady State, Decelerate
Engine > Cut_Ignition threshould -> Cut ignition complete in Overspeed
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

typedef struct timerproperty
{
    uint32_t counter;
    uint8_t  timer_program;
}timer_status;

typedef struct pulseManagement
{
    uint8_t engSpeed;
    timer_status timerCtrl[4];
}programSheet;

extern programSheet request;
extern programSheet Pulse_Program;
	
#endif /* INC_GEN_DEF_H_ */
