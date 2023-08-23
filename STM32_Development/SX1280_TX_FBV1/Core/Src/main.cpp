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
#include "SX128x_formal_board.h"
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SX128x_formal_board sx1280;
int irq_timeout_times = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SPI1_TRANSCEIVER(uint8_t* tx, uint8_t* rx, uint8_t lengh)
{
	uint8_t temp;
	do
	{
		HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &temp, 1, 10);
	}
	while(temp & 0x04);
	HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_RESET);
//	for (int i = 0; i < 1000; i ++);
	HAL_Delay(10);
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, lengh, 10);
	HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_SET);
}
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
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

//  sx1280.CommonTranceiverSetting();




  HAL_StatusTypeDef rv;

  uint8_t rx[10] = {};
  uint8_t tx[10] = {};

  tx[0] = 0xF7;
  rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x03, I2C_MEMADD_SIZE_8BIT, tx, 1, 10);
  rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, rx, 1, 10);

  // reset
//  HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_SET);
  tx[0] = 0xFF;
  rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, tx, 1, 10);
  rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, rx, 1, 10);
  HAL_Delay(100);
//  HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_RESET);
  tx[0] = 0xF7;
  rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, tx, 1, 10);
  rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, rx, 1, 10);
  HAL_Delay(100);
//  HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_SET);
  tx[0] = 0xFF;
  rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, tx, 1, 10);
  rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, rx, 1, 10);

  //===========================================
  // common transceiver setting for LoRa
  //===========================================
  //===========================================
  // common transceiver setting for LoRa
  //===========================================

  HAL_Delay(10);
  // setstandby(stdby_rc)
  *(uint32_t*)tx = 0x80 | 0x01 << 8;
  SPI1_TRANSCEIVER(tx, rx, 2);

  HAL_Delay(10);
  // setpackettype(packet_type_lora)
  *(uint32_t*)tx = 0x8A | 0x01 << 8; // LoRa mode
  SPI1_TRANSCEIVER(tx, rx, 2);

  HAL_Delay(10);
  // setrffrequency(rfFrequency)
  *(uint32_t*)tx = 0x86 | 0xB8 << 8 | 0x9D << 16 | 0x89 << 24;
  SPI1_TRANSCEIVER(tx, rx, 4);

  HAL_Delay(10);
  // setbufferbaseaddress()
  *(uint32_t*)tx = 0x8F | 0x80 << 8 | 0x00 << 16;
  SPI1_TRANSCEIVER(tx, rx, 3);

  HAL_Delay(10);
  // setmodulationparams(modparam1, modparam2, modparam3)
  *(uint32_t*)tx = 0x8B | 0x70 << 8 | 0x18 << 16 | 0x01 << 24;
  SPI1_TRANSCEIVER(tx, rx, 4);

  HAL_Delay(10);
  // setpacketparams(pktparam1, pktparam2, pktparam3, pktparam4, pktparam5)
  *(uint32_t*)tx = 0x8C | 0x0C << 8 | 0x80 << 16 | 0x08 << 24;
  *(uint32_t*)(tx+4) = 0x20 | 0x40 << 8 | 0x00 << 16 | 0x00 << 24;
  SPI1_TRANSCEIVER(tx, rx, 8);



  HAL_Delay(10);
  sx1280.TxSetting();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t begin, end;
  uint16_t time, time_temp;
  uint32_t counter = 0, tx_counter = 0;
  uint32_t temp_counter[2] = {0};

  char uart_buf[100];
  int uart_buf_len;
  uart_buf_len = sprintf(uart_buf, "SX1280 TX bit rate test\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  time_temp = HAL_GetTick();
  while (1)
  {
	  begin = __HAL_TIM_GET_COUNTER(&htim1);

	  temp_counter[0] = counter;
	  while(sx1280.TxBlocking((uint8_t*)temp_counter, 8))
	  {
		  tx_counter ++;
	  }

	  end = __HAL_TIM_GET_COUNTER(&htim1);
	  time = end - begin;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  counter ++;
	  tx_counter ++;
	  if (counter >= 100)
	  {
		  time_temp = HAL_GetTick() - time_temp;

		  float bps = 100*8*8*1000./time_temp;


		  uart_buf_len = sprintf(uart_buf, "Bit Rate: %15.5f bps, IRQ Timeout Times: %3d, count: %3d \r\n", bps, irq_timeout_times, tx_counter);
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		  counter = 0;
		  tx_counter = 0;
		  irq_timeout_times = 0;
		  time_temp = HAL_GetTick();
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 179;
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
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED3_Pin|LED4_Pin|SX1280_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1280_NSS_Pin */
  GPIO_InitStruct.Pin = SX1280_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SX1280_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_Delay(uint32_t Delay)
{
	uint32_t ms = 0;
	uint16_t tick = __HAL_TIM_GET_COUNTER(&htim1);
	do
	{
		if (__HAL_TIM_GET_COUNTER(&htim1) - tick >= 1000)
		{
			tick = __HAL_TIM_GET_COUNTER(&htim1);
			ms ++;
		}
	}while(ms < Delay);
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
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
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
