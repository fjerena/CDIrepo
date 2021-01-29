/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
//Sw defines
#define nSteps                        64u
#define TDuty_Trigger_const         1309u     //1.0ms for clock 72MHz
#define TDutyTriggerK               1319u     //10 + 1319
#define TIntervPulseInv             1964u     //1.0ms + 0.5ms 
#define TPulseInv                   6460u     //3,5ms measured...
#define TMR2_16bits                65536u
#define RPM_const               78545455u
#define FALSE                          0u
#define TRUE                           1u
#define Delta_RPM                    100u

//Global variable
enum Interruption_type {INT_FROM_CH1, INT_FROM_CH2, INT_FROM_CH3, INT_FROM_CH4} int_types;
enum Output_Level {OFF, ON} level;
enum Event_status {EMPTY, PROGRAMMED, DONE} status;
enum Engine_States {STOPPED, CRANKING, ACCELERATION, STEADY_STATE, DECELERATION, OVERSPEED} engstates;
enum Transmission_Status {TRANSMITING, TRANSMISSION_DONE, DATA_AVAILABLE_RX_BUFFER} transmstatus;

/*
2 different speed 
Low  <  1200  // fixed advance ignition
High >= 1200  // according igntion map

Engine Speed: Stopped, Acceleration, Steady State, Decelerate

Engine > Cut_Ignition threshould -> Cut ignition complete in Overspeed
*/

//Calibration values
typedef struct Calibration
{
	uint16_t Max_Engine_Speed;	
  uint16_t BP_Engine_Speed[12];
  uint8_t  BP_Timing_Advance[12]; 	
}struct_Calibration;       

volatile struct_Calibration Calibration_RAM = {15000,
	                                            ////The first Engine Speed value in the array needs to be 1200 mandatory
                                              {1200, 2000, 3000, 3500, 4500, 5000, 6000, 7000, 8000, 9000, 12000, 15000},
																							//{  64,   64,   64,   64,   64,   64,   64,   64,   64,   64,    64,    64}};
																							//{  48,   48,   48,   48,   48,   48,   48,   48,   48,   48,    48,    48}};
                                              //{  32,   32,   32,   32,   32,   32,   32,   32,   32,   32,    32,    32}};
                                              //{  16,   16,   16,   16,   16,   16,   16,   16,   16,   16,    16,    16}};
																							{   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0}};
																							//{  64,   54,   44,   39,   36,   32,   32,   36,   40,   45,     55,     64}};
																							//64 -> 18 degree, calib_table = 64-ang_obj+18 <-> ang_obj = 64-calib_table+18
typedef struct system_info
{
	uint8_t  Low_speed_detected;
	uint8_t  Cutoff_IGN;
	uint8_t  Update_calc;
  uint32_t nOverflow;	
  uint32_t Rising_Edge_Counter;
	uint32_t Measured_Period;
	uint8_t  nOverflow_RE;
	uint8_t  nOverflow_FE;
	uint16_t Engine_Speed_old;
	uint16_t Engine_Speed;
	uint32_t Predicted_Period;
	uint32_t TDuty_Input_Signal;
  uint32_t TStep;
	uint8_t  nAdv;		
}system_vars;

volatile system_vars scenario = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

typedef struct timerproperty
{
	uint32_t counter;
	uint8_t  timer_program;
}timer_status;

timer_status request[4] = {{0,0},
                           {0,0},
                           {0,0},
                           {0,0}};

timer_status Pulse_Program[4] = {{0,0},
                                 {0,0},
                                 {0,0},
                                 {0,0}};

typedef struct Scheduler
{
	uint8_t  program;
	uint32_t target_time;
}sched_var;

sched_var array_sched_var[3];

//UART Communication
uint8_t UART3_txBuffer[12] = {'F','a','b','i','o','_','J','e','r','e','n','a'};
uint8_t UART3_rxBuffer[12];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/*
void System_Configuration(void)
{	
  htim4.Init.Prescaler = 1;
	htim4.Init.Period = 1440;    // 25Hz
	//htim4.Init.Period = 65535;    // 550Hz  -> Audible noise...
	sConfigOC.Pulse = 600;    //Limited to 700 (Can damage the inversor)
}	
*/

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

void Set_Ouput_LED(void)
{
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);	
}
	
void Set_Ouput_Trigger(uint8_t Value)
{
	if (Value == TRUE)
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

void Set_Ouput_Inversor(uint8_t Value)
{
	if (Value == TRUE)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	}	
}


// A iterative binary search function. It returns 
// location of x in given array arr[l..r] if present, 
// otherwise -1 
int8_t binarySearch(uint16_t arr[], uint8_t l, uint8_t r, uint16_t x) 
{ 
		uint8_t m;
	
    while (l <= r) 
		{ 
        m = l + (r - l) / 2; 
			  //m = (l + r) / 2; 
			
			  // Check if x is present at mid 
        if ((x >= arr[m]) && (x <= arr[m+1])) 
            return m; 
  
        // If x greater, ignore left half 
        if (arr[m] < x) 
            l = m + 1; 
  
        // If x is smaller, ignore right half 
        else
            r = m - 1; 
    } 
  
    // if we reach here, then element was 
    // not present 
    return -1; 
} 
  

//uint8_t Adv_Calibrated(uint16_t value, uint16_t x_array[], uint8_t y_array[])
uint8_t Linear_Interpolation(uint16_t value, uint16_t x_array[], uint8_t y_array[])
{
  int8_t interp_index;
	uint8_t interp_res;
	
	interp_index = binarySearch(x_array, 0, 11, value);	
	
  if(interp_index != -1)
  {
    interp_res = (((y_array[interp_index+1]-y_array[interp_index])*(value-x_array[interp_index]))/(x_array[interp_index+1]-x_array[interp_index]))+y_array[interp_index];    		
    return(interp_res);
	}
  else
  {
    //Advance saturation for array min and max
    if(value<x_array[0])
    {
      return(y_array[0]);
    }
    else if(value>x_array[11])
    {
      return(y_array[11]);
    }
		else
		{	
			return(0xFF);     //return an error value...
		}	
  } 	
}

void Cut_Igntion(void)
{	
	if(scenario.Engine_Speed>Calibration_RAM.Max_Engine_Speed)
	{			
		scenario.Cutoff_IGN = 1;
	}	
	else
	{	
		scenario.Cutoff_IGN = 0;
	}		
}

void Engine_STOP_test(void)
{
  static uint8_t program;
  static uint32_t initial_value;

  if(program == 0)
  {
    initial_value = scenario.Rising_Edge_Counter;
    program = 1;
  }
  else
  {
    if(scenario.Rising_Edge_Counter == initial_value)
    {
      scenario.Rising_Edge_Counter = 0;
      scenario.Engine_Speed = 0;
      program = 0;
    }
  }
}

void Timeout(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos, uint8_t *resp_var)
{
  uint32_t counter;
    
  counter = HAL_GetTick();
		
  if(var[pos].program == 0)
  {
    var[pos].target_time = counter+period;
    var[pos].program = 1;
  } 

  if(counter>=var[pos].target_time)
  {
    var[pos].program = 0;
    *resp_var = TRUE;   
  }
  else
  {
    *resp_var = FALSE; 
  }  
}

void Data_Transmission(void)
{	
	uint8_t HSB,LSB;
	
	scenario.Engine_Speed = 2000u;
	scenario.nAdv = 64u;                        
	HSB = (scenario.Engine_Speed&0xFF00)>>8;
	LSB = scenario.Engine_Speed&0xFF;
  UART3_txBuffer[0] = HSB;
	UART3_txBuffer[1] = LSB;
	UART3_txBuffer[2] = scenario.nAdv;
	transmstatus = TRANSMITING;
	HAL_UART_Transmit_DMA(&huart3, UART3_txBuffer, sizeof(UART3_txBuffer));
}	

void Data_Reception(void)
{	
	if(UART3_rxBuffer[1] == 0xD0)
	{	
		Set_Ouput_LED();
	}	
}	

void Task_Fast(void)
{
	//HAL_IWDG_Init(&hiwdg);	
}	

void Task_Medium(void)
{
	Set_Ouput_LED();	
	Cut_Igntion();
	Data_Reception();
	Data_Transmission();
}	

void Task_Slow(void)
{		
  Engine_STOP_test();		
}	

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos)
{ 
  volatile uint32_t counter;
  
	counter = HAL_GetTick();
	
  if(var[pos].program == 0)
  {
    var[pos].target_time = counter+period;
    var[pos].program = 1;
  } 

  if(counter>=var[pos].target_time)
  {
    var[pos].program = 0;
    (*func)();   
  }
}

uint32_t Prediction_Calc(void)
{
  static uint32_t avarage_period;

  avarage_period = (avarage_period+scenario.Measured_Period)/2;
  return(avarage_period);
	
//	errt = dt3[ICint] - dtpred;
//  ....
//  // alpha-beta-gamma filter prediction
//  dts = dtpred + ((inpram.alpha * errt) / 100);
//  tddts = tddtpred + ((inpram.beta * errt) / 100);
//  t2dddts = t2dddts + ((inpram.gamma * errt) / 100);
//  dtpred = dts + tddts + (t2dddts >> 1);
//  tddtpred = tddts + t2dddts;
}

uint8_t Ignition_nTime(uint16_t eng_speed)
{
	static uint8_t advance;
  	
	advance = Linear_Interpolation(eng_speed, Calibration_RAM.BP_Engine_Speed, Calibration_RAM.BP_Timing_Advance);
	
  return(advance);
}

void Set_Pulse_Program(void)
{	
	static uint32_t Event1, Event2;
	
	scenario.Measured_Period += scenario.nOverflow_RE*TMR2_16bits;
  scenario.Engine_Speed = RPM_const/scenario.Measured_Period;
	
	if(scenario.nOverflow_RE == 0)
	{	
		scenario.Low_speed_detected = 0;
		scenario.Predicted_Period = Prediction_Calc();
		scenario.TDuty_Input_Signal += scenario.nOverflow_FE*TMR2_16bits;
		scenario.TStep = scenario.TDuty_Input_Signal/nSteps;
		scenario.nAdv = Ignition_nTime(scenario.Engine_Speed);                            
		Event1 = scenario.TStep*scenario.nAdv;
		Event2 = Event1+TDuty_Trigger_const;		
	
		//Event 1 - Generates Rising Edge for trigger signal using TMR2 to generate the event
		request[0].counter = Event1%65536;
	
		//Event 2 - Generates Falling Edge for trigger signal using TMR2 to generate the event
		request[1].counter = Event2%65536;				
	}	
	else
	{	
		scenario.Low_speed_detected = 1;
	}		
	
	Pulse_Program[0].timer_program = EMPTY;	
	Pulse_Program[1].timer_program = EMPTY;	
	Pulse_Program[2].timer_program = EMPTY;	
	Pulse_Program[3].timer_program = EMPTY;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  transmstatus = TRANSMISSION_DONE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{		
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_rxBuffer, sizeof(UART3_rxBuffer));	
  transmstatus = DATA_AVAILABLE_RX_BUFFER;	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	static int8_t k;
	
	k++;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
		
	//I don´t know the difference between these two different statment	
	//HAL_TIM_Base_Start_IT(&htim2);	
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);          
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	
	//New part regarding UART communication
	//https://deepbluembedded.com/how-to-receive-uart-serial-data-with-stm32-dma-interrupt-polling/
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	//HAL_UART_Transmit_IT();
  //HAL_UART_Receive_IT();
  //HAL_UART_IRQHandler();
		
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  //__HAL_DMA_ENABLE_IT(&hdma_usart3_rx, DMA_IT_TC);	
	//hdma_usart3_rx.Instance->CR &
	//HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 12);	
	
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_rxBuffer, 12);	
	
		
	//HAL_TIM_Base_Start_IT(&htim4);	
	
	//Maybe I don´t need initiate the timers...
		
	//HAL_TIM_Base_Start_IT(&htim4);	
	/* Enable the TIM Capture/Compare 1 interrupt */
  //__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (TRUE)
  {
    //Update the pulse calc scenario
		if (scenario.Update_calc == 1)
	  {
			Set_Pulse_Program();
			
//			if (scenario.Engine_Speed>(scenario.Engine_Speed_old+Delta_RPM))
//			{	
//				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
//			}
//      else
//      {				
//				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
//			}
		  			
			scenario.Engine_Speed_old = scenario.Engine_Speed;			
		  scenario.Update_calc = 0;
	  }
    
    //Scheduler
		Periodic_task(  20,&Task_Fast,   array_sched_var, 0);		
		Periodic_task( 100,&Task_Medium, array_sched_var, 1);		
		Periodic_task(1000,&Task_Slow,   array_sched_var, 2);		
    		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 55;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 55;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA3 PA4 
                           PA5 PA7 PA8 PA9 
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Program_Trigger_Pulse(void)
{ 
	Pulse_Program[0].counter = request[0].counter;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Pulse_Program[0].counter);	
	Pulse_Program[0].timer_program = PROGRAMMED;
	
	Pulse_Program[1].counter = request[1].counter;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,Pulse_Program[1].counter);		   
  Pulse_Program[1].timer_program = PROGRAMMED;	
	
	HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_3); 
	HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_4);
}

void Program_Inverter_Pulse(void)
{	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,TIntervPulseInv);	
	Pulse_Program[2].timer_program = PROGRAMMED;
		
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,TPulseInv);		   
  Pulse_Program[3].timer_program = PROGRAMMED;
	
	__HAL_TIM_SET_COUNTER(&htim4,0u);
	HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1); 
	HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_2);	
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
	scenario.nOverflow = 0;	
	Set_Ouput_Trigger(OFF);
	Set_Ouput_Inversor(OFF); 
	scenario.Rising_Edge_Counter++;
		
	if((scenario.Low_speed_detected == 0)&&
		 (scenario.Cutoff_IGN == 0))
	{			
		Program_Trigger_Pulse();
	}	
	else
	{
		HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_3);
		HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_4);    	
  }		
}

void Falling_Edge_Event(void)
{
	uint16_t counter;
	
	scenario.TDuty_Input_Signal = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);  
	scenario.nOverflow_FE = scenario.nOverflow;
	
	if(scenario.Low_speed_detected == 1)
	{	
		Set_Ouput_Trigger(ON);
		Program_Inverter_Pulse();
		counter = __HAL_TIM_GET_COUNTER(&htim2);
		counter += TDutyTriggerK;		   
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,counter);		
	  HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_4);		
	}	
	
	if (scenario.Rising_Edge_Counter>=2)
	{
		scenario.Update_calc = 1;        //set zero after Engine Stop was detected
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
		case INT_FROM_CH1: Set_Ouput_Inversor(ON);  
		                   Pulse_Program[2].timer_program = DONE;
		                   break;	
		
		case INT_FROM_CH2: Set_Ouput_Inversor(OFF);    
		                   Pulse_Program[3].timer_program = DONE;
		                   HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_1);
		                   HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_2);
		                   break;			
		
		case INT_FROM_CH3: Set_Ouput_Trigger(ON);  
		                   Pulse_Program[0].timer_program = DONE;
		                   Program_Inverter_Pulse();		                   
		                   break;											 

		case INT_FROM_CH4: Set_Ouput_Trigger(OFF);
                       Pulse_Program[1].timer_program = DONE;	                               		
                       break;

		default:           break;
	}
}	

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if((htim->Instance == TIM2)&& 
	   (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)&&
	   (scenario.nOverflow == 0))
	{		
			Treat_Int(INT_FROM_CH3);		    		
  }		

  if((htim->Instance == TIM2)&& 
	   (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)&&
	   (scenario.nOverflow == 0))
	{		
			Treat_Int(INT_FROM_CH4);		    		
  }			
	
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
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
