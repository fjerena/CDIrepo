/*
 * IGN_MGMT.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_IGN_MGMT_H_
#define INC_IGN_MGMT_H_

#include "stm32f1xx_hal.h"

#define nSteps                        64u
#define TDutyTriggerK               1309u     //1.0ms for clock 72MHz
#define TIntervPulseInv              655u     //0.5ms
#define TPulseInv                   4582u     //3,5ms
#define EngineSpeedPeriod_Min     785455u     //100rpm
#define EngineSpeedPeriod_Max       5236u     //15000rpm
#define TMR2_16bits                65536u
#define RPM_const               78545455u

extern TIM_HandleTypeDef htim2;

enum Interruption_type{INT_FROM_CH1,INT_FROM_CH2,INT_FROM_CH3,INT_FROM_CH4};
extern enum Interruption_type int_types;

enum Event_status{EMPTY,PROGRAMMED};
extern enum Event_status status;

enum Engine_States{STOPPED,CRANKING,ACCELERATION,STEADY_STATE,DECELERATION,OVERSPEED};
extern enum Engine_States engstates;

enum engineSpeed{LOW,HIGH};
extern enum engineSpeed pulseMngmt;

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

/// @brief  Possible STM32 system reset causes
typedef enum reset_cause_e
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_LOW_POWER_RESET,
    RESET_CAUSE_WINDOW_WATCHDOG_RESET,
    RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
    RESET_CAUSE_SOFTWARE_RESET,
    RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
    RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
    RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;

//In test
reset_cause_t reset_cause_get(void);
const char * reset_cause_get_name(reset_cause_t reset_cause);

void Cut_Igntion(void);
uint8_t Ignition_nTime(uint16_t eng_speed);
void Set_Pulse_Program(void);
void Engine_STOP_test(void);
void Pulse_Generator_Scheduler(void);
void TurnOffAllPulseInt(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Rising_Edge_Event(void);
void Falling_Edge_Event(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Treat_Int(uint8_t program);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void updateSystemData(void);
void resetCauseAnalysis(void);

#endif /* INC_IGN_MGMT_H_ */
