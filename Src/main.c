/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "uart.h"
#include "ADS1299_drivers.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADS_MIN_DELAY	1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
char rxBuffer[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

//  HAL_UART_Receive_DMA(&huart2, (uint8_t *) rxBuffer, UART_LEN);
//  HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) ADSBuffer, ADS_DMA);

  // Temporär ****************************************
  uint8_t ads_cmd;

  send_uart("Ready!\n", huart2);

//  HAL_Delay(9999999);




  ads_cmd;

//  while(1)
//  {
////	  HAL_SPI_Transmit(&hspi3, &ads_cmd, 1, 100);
//	  ADS_Transmit((uint8_t[]){(0x17U)}, 1);
//	  HAL_Delay(500);
//  }



//  HAL_Delay(500);	// Wait until power goes up completely (minimum 180ms)
  send_uart("Ready to fire..\n", huart2);
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET);



  send_uart("starting..\n", huart2);


  ADS_device_init();
//  ADS_test();
//  ADS_PowerOn();
  goto hehe;



  // Testar starta ADS
//  HAL_Delay(100); // "Wait for Oscillator to wake up", Wait >T_POR for Power On Reset

  //	Issue Reset Pulse
  ads_cmd = ADS_CMD_RESET;
  ADS_Transmit(&(uint8_t) {ADS_CMD_RESET}, 1);


  //	Wait for 18 tCLKs
  HAL_Delay(ADS_MIN_DELAY);	// 18*489= 8800 ms ~= 9µs, testar med 1ms och hoppas på att det fungerar
  //	Send SDATAC Command
  ads_cmd = ADS_CMD_SDATAC;
  ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);

  //Sätt N_CS låg (vilket den är från start)

////////////////
/*
  uint8_t RREG_test = (ADS_CMD_RREG | ADS_CONFIG1_ADDR); //0b00100001; //(ADS_CMD_RREG | ADS_CONFIG1_ADDR);
  ads_cmd = 22;


  while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3));
  HAL_SPI_Transmit(&hspi3, (uint8_t *) &RREG_test, 1, 100);
  HAL_Delay(ADS_MIN_DELAY);

  while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3));
  HAL_SPI_Transmit(&hspi3, (uint8_t *) &ads_cmd, 1, 100);

  */
//  HAL_Delay(ADS_MIN_DELAY);


//  return;


//  HAL_Delay(500);
//  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==RESET);
//  send_uart("wtf???\n", huart2);

//  uint8_t RREGtest = (ADS_CMD_RREG | ADS_CH8SET_ADDR);
//  ads_cmd = 7;
//
//  HAL_Delay(1);
//  while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3));
//  HAL_SPI_Transmit(&hspi3, (uint8_t *) &RREGtest, 1, 100);
//
//  delay_us(2);
////  HAL_Delay(1);
//  while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3));
//  HAL_SPI_Transmit(&hspi3, (uint8_t *) &ads_cmd, 1, 100);
//
//
//  return;
  ////////////////7

  // testar enl manual
  HAL_Delay(ADS_MIN_DELAY); //480*2=1000 ns = 1µs, testar 1 ms
  uint8_t WREG = (ADS_CMD_WREG | ADS_CONFIG3_ADDR);//0b01000011;
  ads_cmd = 0xE0;

  ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | ADS_CONFIG3_ADDR)}, 1);
  ADS_Transmit(&ads_cmd, 1);

  //	Write Certain Registers, Including Input Short
  WREG = (ADS_CMD_WREG | ADS_CONFIG1_ADDR);
  ads_cmd = 0x96;

  ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | ADS_CONFIG1_ADDR)}, 1);
  ADS_Transmit(&ads_cmd, 1);



  WREG = (ADS_CMD_WREG | ADS_CONFIG2_ADDR);
  ads_cmd = 0xC0;

  ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | ADS_CONFIG2_ADDR)}, 1);
  ADS_Transmit(&ads_cmd, 1);


  //	Set All Channels to Input Short
  // testar bara 1 kanal till input short

  WREG = (ADS_CMD_WREG | ADS_CH1SET_ADDR);
  ads_cmd = 0x01;
  ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | ADS_CH1SET_ADDR)}, 1);
  ADS_Transmit(&ads_cmd, 1);


  // Activate Conversion
  HAL_GPIO_WritePin(ADS_START_BUS, ADS_START_PIN, SET);
  HAL_Delay(100);

  // Put the Device Back in RDATAC Mode
  ads_cmd = ADS_CMD_RDATAC;
  ADS_Transmit(&(uint8_t){ADS_CMD_RDATAC}, 1);

  // Activate a (1 mV x VREF / 2.4) Square-Wave Test Signal
  ads_cmd = ADS_CMD_SDATAC;
  ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);

  // WREG CONFIG2 D0h
  WREG = (ADS_CMD_WREG | ADS_CONFIG2_ADDR);
  ads_cmd = 0xD0;

  ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | ADS_CONFIG2_ADDR)}, 1);
  ADS_Transmit(&ads_cmd, 1);

  // WREG CHnSET 05h
  WREG = (ADS_CMD_WREG | ADS_CH1SET_ADDR);
  ads_cmd = 0x05;
  ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | ADS_CH1SET_ADDR)}, 1);
  ADS_Transmit(&ads_cmd, 1);

  //	RDATAC
  ads_cmd = ADS_CMD_RDATAC;
  HAL_Delay(1);
  ADS_Transmit(&(uint8_t){ADS_CMD_RDATAC}, 1);

  send_uart("\nFinished\n", huart2);

  //	When using the START command to control conversions, hold the START pin low.
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, RESET);
//  HAL_SPI_Transmit(&hspi3, (uint8_t *) &ads_cmd, 1, 100);
//  HAL_SPI_Transmit(&hspi3, (uint8_t) ADS_CMD_RDATAC, 1, 100);	//Nix..



  uint8_t nada = 0;
  hehe:
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim1.Init.Prescaler = 75-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ADS_N_CS_Pin|ADS_START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_N_DRDY_Pin */
  GPIO_InitStruct.Pin = ADS_N_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADS_N_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADS_N_CS_Pin ADS_START_Pin */
  GPIO_InitStruct.Pin = ADS_N_CS_Pin|ADS_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	HAL_UART_Transmit(&huart2, (uint8_t *) rxBuffer, UART_LEN, 100);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *) rxBuffer, UART_LEN);

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
//	__NOP();

//	HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) ADSBuffer, ADS_DMA);
//	HAL_UART_Transmit(&huart2, (uint8_t *) ADSBuffer, ADS_DMA, 100);
}


void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/