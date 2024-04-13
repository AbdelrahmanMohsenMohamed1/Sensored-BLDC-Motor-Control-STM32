/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define HALL_OVERSAMPLE 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t ADC_Buffer[4096]={0};
uint8_t Duty = 0 ;
uint8_t Halls = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us);
uint8_t getHalls(void);
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
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, 4096);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		Halls = getHalls();
		switch(Halls)
		{
		case 1 :                                                                    // Hall_A = 1 Rest 0
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_RESET);
			delay_us(50);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Duty);                       // AH PWM
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_SET);            // CL HIGH
			break;

		case 2 :                                                                    //Hall_B = 1 Rest 0
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_RESET);
			delay_us(50);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Duty);						// BH PWM
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_SET);			// AL HIGH
			break;

		case 3 :																	// Hall_A = 1, Hall_B = 1 Rest 0
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_RESET);
			delay_us(50);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Duty);						// BH PWM
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_SET);            // CL HIGH
			break;

		case 4 :																	// Hall_C = 1 Rest 0
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_RESET);
			delay_us(50);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Duty);						// CH PWM
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_SET);			// BL HIGH
			break;

		case 5 :																	// Hall_A = 1 , Hall_C = 1 Rest 0
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_RESET);
			delay_us(50);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Duty);						// AH PWM
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_SET);			// BL HIGH
			break;

		case 6 :																	// Hall_B = 1 , Hall_C = 1 Rest 0
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_RESET);
			delay_us(50);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Duty);						// CH PWM
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_SET);			// AL HIGH
			break;

		default :
			HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
		}

		/*u8_HallA = HAL_GPIO_ReadPin(Hall_A_PIN_GPIO_Port,Hall_A_PIN_Pin);
		u8_HallB = HAL_GPIO_ReadPin(Hall_B_PIN_GPIO_Port,Hall_B_PIN_Pin);
		u8_HallC = HAL_GPIO_ReadPin(Hall_C_PIN_GPIO_Port,Hall_C_PIN_Pin);

		if(u8_HallA == && u8_HallA == && u8_HallA== )
		{

		}
		else if(u8_HallA == && u8_HallA == && u8_HallA== )
		{

		}
		else if(u8_HallA == && u8_HallA == && u8_HallA== )
		{

		}
		else if(u8_HallA == && u8_HallA == && u8_HallA== )
		{

		}
		else if(u8_HallA == && u8_HallA == && u8_HallA== )
		{

		}
		else if(u8_HallA == && u8_HallA == && u8_HallA== )
		{

		}
		else
		{

		}*/
		//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,  50);
		//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,  50);
		//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,  50);
		//HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(AL_PIN_GPIO_Port,AL_PIN_Pin,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_SET);
		//delay_us(50000);
		//HAL_GPIO_WritePin(BL_PIN_GPIO_Port,BL_PIN_Pin,GPIO_PIN_RESET);
		//delay_us(50000);
		/*HAL_Delay(1000);
		HAL_GPIO_WritePin(CL_PIN_GPIO_Port,CL_PIN_Pin,GPIO_PIN_RESET);
		//delay_ns(50000);
		HAL_Delay(1000);*/
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
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

	/** Initializes the CPU, AHB and APB buses clocks
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	htim4.Init.Prescaler = 72-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 100-1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, AL_PIN_Pin|BL_PIN_Pin|CL_PIN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : AL_PIN_Pin BL_PIN_Pin CL_PIN_Pin */
	GPIO_InitStruct.Pin = AL_PIN_Pin|BL_PIN_Pin|CL_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Hall_A_PIN_Pin Hall_B_PIN_Pin Hall_C_PIN_Pin */
	GPIO_InitStruct.Pin = Hall_A_PIN_Pin|Hall_B_PIN_Pin|Hall_C_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	Duty = ( (ADC_Buffer[0] * 100) /4096);
}
void delay_us(uint16_t us) {

	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
	/*	// Configure timer TIM1
	TIM_HandleTypeDef htim1;
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72-1; // Timer runs at 1 MHz
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = us-1;
	HAL_TIM_Base_Init(&htim1);

	// Start the timer
	HAL_TIM_Base_Start(&htim1);

	// Wait for timer to complete
	while (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) == RESET) {}

	// Clear the update flag
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

	// Stop the timer
	HAL_TIM_Base_Stop(&htim1);*/
}
/* Read hall sensors WITH oversamping. This is required, as the hall sensor readings are often noisy.
 * This function reads the sensors multiple times (defined by HALL_OVERSAMPLE) and only sets the output
 * to a 1 if a majority of the readings are 1. This really helps reject noise. If the motor starts "cogging" or "skipping"
 * at low speed and high torque, try increasing the HALL_OVERSAMPLE value
 *
 * Outputs a number, with the last 3 binary digits corresponding to hall readings. Thus 0 to 7, or 1 to 6 in normal operation*/


uint8_t getHalls(void)
{
	uint8_t hallCounts[] = {0, 0, 0};
	for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) // Read all the hall pins repeatedly, tally results
	{
		hallCounts[0] += HAL_GPIO_ReadPin(Hall_A_PIN_GPIO_Port,Hall_A_PIN_Pin);
		hallCounts[1] += HAL_GPIO_ReadPin(Hall_B_PIN_GPIO_Port,Hall_B_PIN_Pin);
		hallCounts[2] += HAL_GPIO_ReadPin(Hall_C_PIN_GPIO_Port,Hall_C_PIN_Pin);
	}

	uint8_t hall = 0;

	if (hallCounts[0] >= HALL_OVERSAMPLE / 2)     // If votes >= threshold, call that a 1
		hall |= (1<<0);                             // Store a 1 in the 0th bit
	if (hallCounts[1] >= HALL_OVERSAMPLE / 2)
		hall |= (1<<1);                             // Store a 1 in the 1st bit
	if (hallCounts[2] >= HALL_OVERSAMPLE / 2)
		hall |= (1<<2);                             // Store a 1 in the 2nd bit

	return hall & 0x7;                            // Just to make sure we didn't do anything stupid, set the maximum output value to 7
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
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
