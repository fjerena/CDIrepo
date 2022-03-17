/*
 * IGN_MGMT.c
 *
 *  Created on: 05-out-2021
 *      Author: Jerena
 */

#include "IGN_MGMT.h"
#include "GENERAL_DEF.h"
#include "MATH_LIB.h"
#include "IO_CONTROL.h"

programSheet request = {0, {{0,0}, {0,0}, {0,0}, {0,0}}};
programSheet Pulse_Program = { 0, {{0,0}, {0,0}, {0,0}, {0,0}}};

enum Interruption_type int_types=INT_FROM_CH1;
enum Event_status status=EMPTY;
enum Engine_States engstates=STOPPED;
enum engineSpeed pulseMngmt=LOW;

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause
reset_cause_t reset_cause_get(void)
{
    reset_cause_t reset_cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_cause = RESET_CAUSE_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        reset_cause = RESET_CAUSE_SOFTWARE_RESET; // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is 
    // NOT a POR/PDR reset. See note below. 
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = RESET_CAUSE_BROWNOUT_RESET;
    }
    else
    {
        reset_cause = RESET_CAUSE_UNKNOWN;
    }

    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause; 
}

// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock Controller (RCC) header
// files, such as "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h", etc., indicate that the 
// brownout flag, `RCC_FLAG_BORRST`, will be set in the event of a "POR/PDR or BOR reset". This means that a 
// Power-On Reset (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag. See the 
// doxygen just above their definition for the `__HAL_RCC_GET_FLAG()` macro to see this:
// "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout Reset flag will *also* be set in 
// the event of a POR/PDR. 
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after* first checking the 
// `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is NOT a POR/PDR reset.


/// @brief      Obtain the system reset cause as an ASCII-printable name string from a reset cause type
/// @param[in]  reset_cause     The previously-obtained system reset cause
/// @return     A null-terminated ASCII name string describing the system reset cause
const char * reset_cause_get_name(reset_cause_t reset_cause)
{
    const char * reset_cause_name = "TBD";

    switch (reset_cause)
    {
        case RESET_CAUSE_UNKNOWN:
            reset_cause_name = "UNKNOWN";
            break;
        case RESET_CAUSE_LOW_POWER_RESET:
            reset_cause_name = "LOW_POWER_RESET";
            break;
        case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
            reset_cause_name = "WINDOW_WATCHDOG_RESET";
            break;
        case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
            reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
            break;
        case RESET_CAUSE_SOFTWARE_RESET:
            reset_cause_name = "SOFTWARE_RESET";
            break;
        case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
            reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
            break;
        case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
            reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
            break;
        case RESET_CAUSE_BROWNOUT_RESET:
            reset_cause_name = "BROWNOUT_RESET (BOR)";
            break;
    }

    return reset_cause_name;
}

void Cut_Igntion(void)
{	
		static uint8_t flagCutoff = OFF;
	
    if(scenario.Engine_Speed>calibFlashBlock.Calibration_RAM.Max_Engine_Speed)
    {
        scenario.Cutoff_IGN = ON;
			  flagCutoff = ON;
    }
    else
    {
			  if(flagCutoff == OFF)
				{
						scenario.Cutoff_IGN = OFF;
				}
				//The constant 300rpm is the hysteresis, the control will wait Engine Speed (after to overcome te Engine Speed Limit)
				//decrease more than 300rpm to turnoff the cutoff function and consequently restart the engine combustions
				else if(scenario.Engine_Speed<(calibFlashBlock.Calibration_RAM.Max_Engine_Speed)-300u)
				{
						scenario.Cutoff_IGN = OFF;
						flagCutoff = OFF;
				}					
    }
}

uint8_t Ignition_nTime(uint16_t eng_speed)
{
    static uint8_t advance;

    advance = linearInterpolation(eng_speed, calibFlashBlock.Calibration_RAM.BP_Engine_Speed, calibFlashBlock.Calibration_RAM.BP_Timing_Advance);
    return(advance);
}

void Set_Pulse_Program(void)
{
    static uint32_t Event1, Event2, Event3, Event4;

    Set_Ouput_InterruptionTest();

    scenario.Measured_Period += scenario.nOverflow_RE*TMR2_16bits;

    //Engine speed must be greater than 100rpm and less than 15000rpm to consider Measured_Period useful for calculations
    if((scenario.Measured_Period<=EngineSpeedPeriod_Min)&&
       (scenario.Measured_Period>=EngineSpeedPeriod_Max))
    {
        scenario.Engine_Speed = RPM_const/scenario.Measured_Period;
        scenario.Engine_Speed_old = scenario.Engine_Speed;
        scenario.deltaEngineSpeed = scenario.Engine_Speed-scenario.Engine_Speed_old;

        //Linear prediction
        if((scenario.Engine_Speed<<1u)>scenario.Engine_Speed_old)
        {
            scenario.engineSpeedPred = (scenario.Engine_Speed<<1u)-scenario.Engine_Speed_old;
            scenario.tdutyInputSignalPredLinear = (RPM_const*calibFlashBlock.Calibration_RAM.sensorAngDisplecement)/(scenario.engineSpeedPred*360u);
        }
        else
        {
            scenario.engineSpeedPred = 0u;
        }

        //For calculus purpose I decided to use the linear prediction
        scenario.Engine_Speed = scenario.engineSpeedPred;

        if(scenario.Engine_Speed>=calibFlashBlock.Calibration_RAM.BP_Engine_Speed[0])
        {
            request.engSpeed = HIGH;
            scenario.TDuty_Input_Signal += scenario.nOverflow_FE*TMR2_16bits;
            scenario.tdutyInputSignalPred = predictionCalc(scenario.TDuty_Input_Signal);
            //scenario.TStep = scenario.TDuty_Input_Signal/nSteps;
            scenario.TStep = scenario.tdutyInputSignalPredLinear/nSteps;
            scenario.nAdv = Ignition_nTime(scenario.Engine_Speed);
            Event1 = scenario.TStep*scenario.nAdv;
        }
        else
        {
            //Will be trigger at the same routine that treats the falling edge detection
            request.engSpeed = LOW;
            //This value was set to generate the rising edge pulse immediately
            scenario.nAdv = 0u;
            Event1 = 0u;
        }

        Event2 = Event1+TDutyTriggerK;
        Event3 = Event2+TIntervPulseInv;
        Event4 = Event3+TPulseInv;

        //Event 1 - Generates Rising Edge for trigger signal using TMR4 to generate the event
        request.timerCtrl[0].counter = Event1%65536u;

        //Event 2 - Generates Falling Edge for trigger signal using TMR4 to generate the event
        request.timerCtrl[1].counter = Event2%65536u;

        //Event 3 - Generates Rising Edge for inversor signal using TMR4 to generate the event
        request.timerCtrl[2].counter = Event3%65536u;

        //Event 4 - Generates Falling Edge for inversor signal using TMR4 to generate the event
        request.timerCtrl[3].counter = Event4%65536u;
    }
    else if(scenario.Measured_Period<EngineSpeedPeriod_Max)
    {
        //set flag Overspeed
        //register the max engine speed
    }
    else
    {
        //Underspeed
        //maybe I don´t need to register this because will happening all engine start event
    }

    Pulse_Program.timerCtrl[0].timer_program = EMPTY;
    Pulse_Program.timerCtrl[1].timer_program = EMPTY;
    Pulse_Program.timerCtrl[2].timer_program = EMPTY;
    Pulse_Program.timerCtrl[3].timer_program = EMPTY;

    Set_Ouput_InterruptionTest();
}

void Engine_STOP_test(void)
{
    static uint8_t program;
    static uint32_t initial_value;

    if(program == FALSE)
    {
        initial_value = scenario.Rising_Edge_Counter;
        program = TRUE;
    }
    else
    {
        if(scenario.Rising_Edge_Counter == initial_value)
        {
					  //It going to clean all scenario struct (system_vars)
            scenario.Engine_Speed_old=0;
						scenario.Engine_Speed=0;
						scenario.engineSpeedPred=0;
						scenario.engineSpeedFiltered=0;
						scenario.avarageEngineSpeed=0;
						scenario.deltaEngineSpeed=0;
						scenario.Rising_Edge_Counter=0;
						scenario.triggerEventCounter=0;
						scenario.inversorEventCounter=0;
						scenario.Measured_Period=0;
						scenario.TDuty_Input_Signal=0;
						scenario.tdutyInputSignalPred=0;
						scenario.tdutyInputSignalPredLinear=0;
						scenario.sensorAngDisplecementMeasured=0;
						scenario.TStep=0;
						scenario.nAdv=0;
						scenario.Cutoff_IGN=0;
						scenario.Update_calc=0;
						scenario.nOverflow=0;
						scenario.nOverflow_RE=0;
						scenario.nOverflow_FE=0;
        }
				
				program = FALSE;
    }
}

void Pulse_Generator_Scheduler(void)
{
    uint8_t i;

    for(i=0;i<4;i++)
    {
       Pulse_Program.timerCtrl[i].counter = request.timerCtrl[i].counter;
    }

    Set_Ouput_Trigger(OFF);
    Set_Ouput_Inversor(OFF);

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Pulse_Program.timerCtrl[0].counter);
    Pulse_Program.timerCtrl[0].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,Pulse_Program.timerCtrl[1].counter);
    Pulse_Program.timerCtrl[1].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,Pulse_Program.timerCtrl[2].counter);
    Pulse_Program.timerCtrl[2].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Pulse_Program.timerCtrl[3].counter);
    Pulse_Program.timerCtrl[3].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COUNTER(&htim4,0u);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_4);
}

void TurnOffAllPulseInt(void)
{
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_1);
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_2);
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_3);
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM2)
    {
        scenario.nOverflow++;
    }
}

void Rising_Edge_Event(void)
{	  
    scenario.Measured_Period = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
    scenario.nOverflow_RE = scenario.nOverflow;
    __HAL_TIM_SET_COUNTER(&htim2,0u);
    scenario.nOverflow = 0u;
    Pulse_Program.engSpeed = request.engSpeed;

    if((Pulse_Program.engSpeed == HIGH)&&
    (scenario.Cutoff_IGN == OFF))
    {
        Pulse_Generator_Scheduler();
    }

    scenario.Rising_Edge_Counter++;
		Toggle_LED_Red();
}

void Falling_Edge_Event(void)
{
    scenario.TDuty_Input_Signal = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
    scenario.nOverflow_FE = scenario.nOverflow;

    if((Pulse_Program.engSpeed == LOW)&&
    (scenario.Cutoff_IGN == OFF))
    {
        Pulse_Generator_Scheduler();
    }

    if (scenario.Rising_Edge_Counter>=2u)
    {
        scenario.Update_calc = TRUE;        //set zero after Engine Stop was detected
    }		
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if((htim->Instance == TIM2)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Rising_Edge_Event();
    }

    if((htim->Instance == TIM2)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        Falling_Edge_Event();
    }
}

void Treat_Int(uint8_t program)
{
    switch(program)
    {
        case INT_FROM_CH1: Set_Ouput_Trigger(ON);
                           Pulse_Program.timerCtrl[0].timer_program = EMPTY;
                           scenario.triggerEventCounter++;
                           break;

        case INT_FROM_CH2: Set_Ouput_Trigger(OFF);
                           Pulse_Program.timerCtrl[1].timer_program = EMPTY;
                           break;

        case INT_FROM_CH3: Set_Ouput_Inversor(ON);
                           Pulse_Program.timerCtrl[2].timer_program = EMPTY;
                           scenario.inversorEventCounter++;
                           break;

        case INT_FROM_CH4: Set_Ouput_Inversor(OFF);
                           Pulse_Program.timerCtrl[3].timer_program = EMPTY;
                           TurnOffAllPulseInt();
                           break;

        default          : break;
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Treat_Int(INT_FROM_CH1);
    }

    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        Treat_Int(INT_FROM_CH2);
    }

    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
    {
        Treat_Int(INT_FROM_CH3);
    }

    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
    {
        Treat_Int(INT_FROM_CH4);
    }
}

void updateSystemData(void)
{
		if(scenario.Engine_Speed>sysInfoBlock.systemInfo_RAM.maxEngineSpeed)
		{
				sysInfoBlock.systemInfo_RAM.maxEngineSpeed=scenario.Engine_Speed;
		}
		
		if(sensors.VBat<sysInfoBlock.systemInfo_RAM.minVoltage)
		{		
				sysInfoBlock.systemInfo_RAM.minVoltage=sensors.VBat;	
		}
		
		if(sensors.VBat>sysInfoBlock.systemInfo_RAM.maxVoltage)
		{			
				sysInfoBlock.systemInfo_RAM.maxVoltage=sensors.VBat;
		}
		
		if(sensors.VBat<60u)
		{
				Set_Diagnose(fail_0);
		}
		
		if(sensors.VBat>140u)
		{
				Set_Diagnose(fail_1);
		}  
}

void resetCauseAnalysis(void)
{	
		//In test
		/*
		LPWRRSTF   // Low-power reset flag
		WWDGRSTF   // Window watchdog reset flag
		IWDGRSTF   // Independent watchdog reset flag
		SFTRSTF    // Software reset flag
		PORRSTF    // POR/PDR reset flag
		PINRSTF    // PIN reset flag
		RMVF       // Remove reset flag
		LSIRDY     // Internal low-speed oscillator ready
		LSION      // Internal low-speed oscillator enable
		*/		
				
	  sysInfoBlock.systemInfo_RAM.resetCause = reset_cause_get();
	  //https://stackoverflow.com/questions/34196663/stm32-how-to-get-last-reset-status
}	
