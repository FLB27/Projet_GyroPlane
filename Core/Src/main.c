/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "iks01a3_env_sensors.h"
#include "stm32l1xx_nucleo_bus.h"
#include "iks01a3_motion_sensors.h"
#include "stm32l1xx_hal_conf.h"
#include "max7219_Yncrea2.h"
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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float PressValue;
IKS01A3_MOTION_SENSOR_Axes_t Axes;
IKS01A3_MOTION_SENSOR_Axes_t Axes2;
char buffer[10];
float TempValue;
float alt;
int sortie = 0;
uint32_t analogValue=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void affichage_pression(void);
float conversion_press_alt(float PressValue, float TempValue);
double angle_plane(int32_t AX,int32_t AY, int32_t AZ );
void extinction_leds(void);
void clignotement_leds(void);
int ON_OFF_Projet(int sortie);
uint32_t adcFunction(void); //potar RV1
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

 while(HAL_TIM_PWM_Init(&htim3)!= HAL_OK){
	 HAL_TIM_PWM_Init(&htim3);
	 printf("Timer enabled \r\n");
  }

  MAX7219_Init();
  MAX7219_DisplayTestStart();
  HAL_Delay(2000);
  MAX7219_DisplayTestStop();
  printf("Display test finished \r  \n");

  while(IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_TEMPERATURE) != 0){
	  int32_t test = IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_TEMPERATURE);//initialisation du capteur de pression
	  printf("test : %ld \r  \n",test);
	  HAL_Delay(1000);
   }


  while(IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE) != 0){
	  int32_t test = IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE);//initialisation du capteur de pression
	  printf("test : %ld \r  \n",test);
	  HAL_Delay(1000);
  }

  while(IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_GYRO) != 0){
	  int32_t test2 = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_GYRO);
	  printf("test2 : %ld \r  \n",test2);
	  HAL_Delay(1000);
  }

  while(IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO) != 0){
	  int32_t test3 = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO);
	  printf("test3 : %ld \r  \n",test3);
	  HAL_Delay(1000);
   }

  IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 2);
  IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 200.0);//vitesse de rafraichissement des données par seconde

  IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_GYRO, 1000); //d'après la doc qui dit qu'on va de -250 à 250
  IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_GYRO, 200.0);

  adcFunction();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	  while(ON_OFF_Projet(sortie) == 1 && adcFunction() >= 1000){

		  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_TEMPERATURE, &TempValue);
		  printf("	Temperature : %.2f \r\n",TempValue);

		  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_PRESSURE, &PressValue);
		  printf("	Pression : %d \r\n",(int)PressValue);
		  sprintf(buffer, "%d\n",(int)PressValue);//on transforme notre entier en une chaîne de caractère (tableau) qu'on va venir découper
		  affichage_pression();//affichage sur 7 segments

		  alt = conversion_press_alt( PressValue,  TempValue);
		  printf("	Altitude par rapport au niveau de la mer: %.2f \r  \n",alt);//affichage de l'altitude


		  IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_GYRO, &Axes);
		  int32_t X = Axes.x/1000;//conversion des valeurs de mdps en dps
		  int32_t Y = Axes.y/1000;
		  int32_t Z = Axes.z/1000;

		  printf("	GYRO en x : %ld dps \r \n",X);
		  printf("	GYRO en y  : %ld dps \r \n",Y);
		  printf("	GYRO en z  : %ld  dps\r \n",Z);

		  //il faut que je calcule l'angle de rotation par rapport à la vitesse renvoyée par le gyro

		  IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, &Axes2);
		  printf("	ACCEL en x : %ld mg\r \n",Axes2.x);//conversion des valeurs de mg en g
		  printf("	ACCEL en y  : %ld mg\r \n",Axes2.y);
		  printf("	ACCEL en z  : %ld mg\r \n",Axes2.z); //quand on est stable on doit avoir uniquement l'accélération de la pesanteur (1g = 9,8m^2/s)

		  double angle = angle_plane(Axes2.x, Axes2.y, Axes2.z);

		  switchLedAll(angle);

		  if(angle>=80 || angle<=-80){
			  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			  TIM3->CCR1 = 316;
			  while(angle>=80 || angle<=-80){
				  IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, &Axes2);
				  angle = angle_plane(Axes2.x, Axes2.y, Axes2.z);
				  clignotement_leds();
				  HAL_Delay(50);
				  TIM3->CCR2 = 300;
				  HAL_Delay(50);
				  TIM3->CCR2 = 290;
				  HAL_Delay(50);
				  TIM3->CCR2 = 250;
				  HAL_Delay(50);
				  TIM3->CCR2 = 200;
				  HAL_Delay(50);
			  }
			  extinction_leds();
			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		 }

		  HAL_Delay(500); //on attend une seconde avant de prendre en compte le nouvel angle

		  printf("\r \n");

	  }

  if(ON_OFF_Projet(sortie) == 0 && adcFunction()<1000){
	  extinction_leds();
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	  MAX7219_Clear();
  }

 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  AnalogWDGConfig.HighThreshold = 2000;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 316;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 158;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 316;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin|L5_Pin|L6_Pin|L7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L0_Pin L1_Pin L2_Pin L3_Pin
                           L4_Pin L5_Pin L6_Pin L7_Pin */
  GPIO_InitStruct.Pin = L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin|L5_Pin|L6_Pin|L7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BP1_Pin BP2_Pin */
  GPIO_InitStruct.Pin = BP1_Pin|BP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void switchLedAll(double angle){
	if(angle<10 && angle>-10){
		extinction_leds();
	}
	if((10<angle && angle<20) || (-10>angle && angle>-20)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
	}
	if((20<angle && angle<30) || (-20>angle && angle>-30)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
	}
	if((30<angle && angle<40) || (-30>angle && angle>-40)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
		HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);

	}
	if((40<angle && angle<50) || (-40>angle && angle>-50)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
		HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
		HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
	}
	if((50<angle && angle<60) || (-50>angle && angle>-60)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
		HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
		HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
		HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
	}
	if((60<angle && angle<70) || (-60>angle && angle>-70)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
		HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
		HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
		HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
		HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
	}
	if((70<angle && angle<80) || (-70>angle && angle>-80)){
		extinction_leds();
		HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
		HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
		HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
		HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
		HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
		HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
	}

}

void extinction_leds(void){
	HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
	HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
	HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
	HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
	HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
	HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
	HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
	HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin, 0);
}

void clignotement_leds(void){

	int i;

	extinction_leds();

	for(i=0;i<5;i++){
	HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
	HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
	HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
	HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
	HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
	HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
	HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
	HAL_GPIO_TogglePin(L0_GPIO_Port, L0_Pin);
	HAL_Delay(50);
	}
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF); //printf sur Termite le terminal application
	ITM_SendChar(ch);
	return(ch);
}

float conversion_press_alt(float PressValue, float TempValue){

	float T = TempValue;//température à voir pour la mesurer

	float alt = ((pow((PressValue/1013),(1/5.257)) - 1)*(0.01*T + 273.15))/(0.0065); //altitude par rapport au niveau de la mer

	return alt;
}

void affichage_pression(void){
MAX7219_Clear();
MAX7219_DisplayChar('4',buffer[3]);
MAX7219_DisplayChar('3',buffer[2]);
MAX7219_DisplayChar('2',buffer[1]);
MAX7219_DisplayChar('1',buffer[0]);
HAL_Delay(200);
}


double angle_plane(int32_t AX,int32_t AY, int32_t AZ ){

	double angle;

	angle = atan((sqrt(pow((double)AX,2) + pow((double)AY,2)))/((double)AZ))*(180/3.14); //calcul de l'angle
	//angle = atan2((double)AX,(double)AZ)*(180/3.14);

	printf("angle rotation : %lf \r\n",angle);

	return angle;
}

int ON_OFF_Projet(int sortie){
	return sortie;
}

uint32_t adcFunction(void) //potar RV2 utilisé pour faire varier la vitesse du moteur
{
	HAL_ADC_Start_IT(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100); // refresh des infos toutes les 1000ms
	analogValue = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	printf("ADC_Value = %lu \n\r",analogValue);//valeur max à gauche 4095

	if(analogValue >= 0 && analogValue < 1000)
		{
			TIM3->CCR1 = 0;
		}

	if(analogValue >= 1000 && analogValue < 2000)
		{
			TIM3->CCR1 = 79;
		}

	if(analogValue >= 2000 && analogValue < 3000)
		{
			TIM3->CCR1 = 158;
		}

	if(analogValue >= 3000 && analogValue < 4000)
		{
			TIM3->CCR1 = 237;
		}

	if(analogValue >= 4000)
		{
			TIM3->CCR1 = 316;
		}

	return analogValue;

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
