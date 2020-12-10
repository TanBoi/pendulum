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
#define PI 3,1415926
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int potardt1=0;
int potardt2=0;

int vitesse=0;
int vitesseP=0;
int count=0;
int sens=0;
int angledt1=0;
int angledt2=0;
double potar=0;
int x=0;
int y=0;
char accelx[12];
char accely[12];
char vitesse2[12];
char vitesseP2[12];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//double potar=0;
char potar2[12];
char sens2[12];
char angle1[12];
char angle2[12];
char count2[12];

/*int potardt1=0;
int potardt2=0;

int vitesse=0;
int vitesseP=0;
int count=0;
int sens=0;
int angledt1=0;
int angledt2=0;*/

uint16_t adc_buf[3];
uint16_t adc_bufx=0;
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/*Encodeur      : A -> PA8 / D7
	  	  	  	      B -> PA9 / D8
	  Potar         : PC0 / A5
	  Accelerometre : x -> PA0 / A0
	  	  	  	    : y -> PA1 / A1

	  	  	  	  Port sÃ©rie : putty et print();
	  	  	  	  */

	  //HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);


	  	  	  	  //Calcul encodeur
	  /*
	  	  	  	  count = TIM1->CNT;
	  	  	  	  sens = __HAL_TIM_DIRECTION_STATUS(&htim1);
	  	  	  	  //angle precedent en radians
	  	  	  	  //angledt1=count*2*3.14/8192-3.14;
	  	  	  	  angledt1=count*360/8192-180;

	  	  	  	  //Calcul avec potar

	  	  	  	  HAL_ADC_Start(&hadc1);
	  	  	 	  HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	 	  potar=HAL_ADC_GetValue(&hadc1);
	  	  	 	  potardt1=(int)potar*100;
	  	  	 	  //*2*PI/(4096)+PI/2;

	  	  	  	  HAL_Delay(200);

	  	  	  	  //Calcul encodeur
	  	  	      count = TIM1->CNT;
	  	  	      sens = __HAL_TIM_DIRECTION_STATUS(&htim1);
	  	  	      //Angle en radians
	  	  	      //angledt2=count*2*3.14/8192-3.14;
	  	  	      angledt2=count*360/8192-180;

	  	  	      //Calcul avec potar
	  	  	  	  HAL_ADC_Start(&hadc1);
	  	  	  	  HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  	  potar=HAL_ADC_GetValue(&hadc1);
	  	  	  	  potardt2=(int)potar*100;
	  	  	  	  //*2*PI/(4096)+PI/2; */

	  	  	      //Envoi vitesse encodeur + potar
	  	  	      //vitesseP=(potardt2-potardt1)*20;
	  	  	      //vitesseP=potardt2;


	  	  	  	  //potardt2 = potardt1;
	  	  	  	  //HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc_buf, 3);
	  	  	  	  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc_buf, 3);

	  	  	  	  x=(int)adc_buf[0];
	  	  	  	  y=(int)adc_buf[1];
	  	  	  	  potar=adc_buf[2];
	  	  	  	  //x=((x/4096)*10)-5;
	  	  	  	  //y=((y/4096)*10)-5;
	  	  	  	  //HAL_ADC_Start(&hadc1);
	  	  	  	  //HAL_ADC_PollForConversion(&hadc1, 100);
	  	  	  	  //potar=HAL_ADC_GetValue(&hadc1);

	  	  	  	  potardt1=(int)potar*100;

	  	  	  	  //if(sens == __HAL_TIM_DIRECTION_STATUS(&htim1))
	  	  	  	  // {
	  	  		  vitesse=(angledt2-angledt1);
	  	  	      //}


	  	  	  	  sprintf(vitesse2, "%d",vitesse);
	  	  	  	  sprintf(accelx, "%d",x);
	  	  	  	  sprintf(accely, "%d",y);
	  	  	  	  sprintf(vitesseP2, "%d",potardt1);

	  	  	      debugPrint(&huart2, vitesse2);
	  	  	      debugPrint(&huart2, vitesseP2);
	  	  	      debugPrint(&huart2, accelx);
	  	  	      debugPrint(&huart2, accely);


	  	  	  	  HAL_Delay(50);

	  	  	      //Aide au debuggage par liaison serie
	  	  	  	  //sprintf(count2, "%d", count);
	  	  	  	  //sprintf(sens2, "%d", sens);
	  	  	  	  //sprintf(angle2, "%d",angledt2);
	  	  	  	  //sprintf(angle1, "%d",angledt1);
	  	  	  	  //debugPrint(&huart2, "sens :");
	  	  	  	  //debugPrint(&huart2, sens2);
	  	  	  	  //debugPrint(&huart2, "angle :");
	  	  	  	  //debugPrint(&huart2, count2);
	  	  	  	  //debugPrint(&huart2, angle2);
	  	  	  	  //debugPrint(&huart2, angle1);



	  	  	  	  //sprintf(potar2, "%d", potar);
	  	  	  	  //debugPrint(&huart2, "potar :");

	  	  	  	  //debugPrint(&huart2, potar2);


	  	  	  	  /*https://tunizem.blogspot.com/2014/09/using-adc-with-dma-on-stm32.html
	  	  	  	  	  DMA_Cmd(DMA1_Channel1, ENABLE);
	  	  	  	  	  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	  	  	  	  	  adc_bufx=ADC_values[0];

	  	  	  	  HAL_ADC_Start(&hadc1);
	  	  	  	  HAL_ADC_PollForConversion(&hadc1,5);
	  	  	  	  adc_1 = HAL_ADC_GetValue(&hadc1);
	  	  	  	  HAL_ADC_Start(&hadc1);
	  	  	  	  HAL_ADC_PollForConversion(&hadc1,5);
	  	  	  	  adc_2 = HAL_ADC_GetValue(&hadc1);
	  	  	      HAL_ADC_Start(&hadc1);
	  	  		  HAL_ADC_PollForConversion(&hadc1,5);
	  	  		  adc_3 = HAL_ADC_GetValue(&hadc1);

	  	  	      ///sprintf(adc_bufx, "%d",adc_buf[0]);
	  	  	  	  ///debugPrint(&huart2, adc_bufx);
*/
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8192;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  TIM1->CNT=4096;
  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void debugPrint(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim4 )
  {
	  angledt2=angledt1;
	  count = TIM1->CNT;
	  sens = __HAL_TIM_DIRECTION_STATUS(&htim1);
	  //angle precedent en radians
	  //angledt1=count*2*3.14/8192-3.14;
	  angledt1=count*360/8192-180;
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
