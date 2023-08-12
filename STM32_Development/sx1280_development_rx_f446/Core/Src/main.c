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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void SPI1_TRANSCEIVER(uint8_t* tx, uint8_t* rx, uint8_t lengh)
{
	while(HAL_GPIO_ReadPin(SX1280_BUSY_GPIO_Port, SX1280_BUSY_Pin));
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, lengh, 10);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

   //===========================================
   //===========================================
   //===========================================

   uint8_t tx[10] = {};
   uint8_t rx[10] = {};

   // reset
   HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_SET);
   HAL_Delay(10);
   HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_RESET);
   HAL_Delay(10);
   HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_SET);

   //===========================================
   // common transceiver setting for LoRa
   //===========================================

   // setstandby(stdby_rc)
   *(uint32_t*)tx = 0x80 | 0x01 << 8;
   SPI1_TRANSCEIVER(tx, rx, 2);

   // setpackettype(packet_type_lora)
   *(uint32_t*)tx = 0x8A | 0x01 << 8; // LoRa mode
   SPI1_TRANSCEIVER(tx, rx, 2);

   // setrffrequency(rfFrequency)
   *(uint32_t*)tx = 0x86 | 0xB8 << 8 | 0x9D << 16 | 0x89 << 24;
   SPI1_TRANSCEIVER(tx, rx, 4);

   // setbufferbaseaddress()
   *(uint32_t*)tx = 0x8F | 0x80 << 8 | 0x00 << 16;
   SPI1_TRANSCEIVER(tx, rx, 3);

   // setmodulationparams(modparam1, modparam2, modparam3)
   *(uint32_t*)tx = 0x8B | 0x70 << 8 | 0x0A << 16 | 0x01 << 24;
   SPI1_TRANSCEIVER(tx, rx, 4);

   // setpacketparams(pktparam1, pktparam2, pktparam3, pktparam4, pktparam5)
   *(uint32_t*)tx = 0x8C | 0x0C << 8 | 0x00 << 16 | 0x04 << 24;
   *(uint32_t*)(tx+4) = 0x40 | 0x00 << 8 | 0x00 << 16;
   SPI1_TRANSCEIVER(tx, rx, 7);

//   //===========================================
//   // Tx Setting and Operations
//   //===========================================
//
//   // SetTxParams(power, rampTime)
//   *(uint32_t*)tx = 0x8E | 0x1F << 8 | 0xE0 << 16 | 0x01 << 24;
//   SPI1_TRANSCEIVER(tx, rx, 4);
//
//   // WriteBuffer(offset, *data)
//   *(uint32_t*)tx = 0x1A | 0x80 << 8 | 0x05 << 16 | 0x04 << 24;
//   *(uint32_t*)(tx+4) = 0x08 | 0x07 << 8;
//   SPI1_TRANSCEIVER(tx, rx, 6);
//
//   // SetDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask)
//   *(uint32_t*)tx = 0x8D | 0x40 << 8 | 0x23 << 16 | 0x00 << 24;
//   *(uint32_t*)(tx+4) = 0x01 | 0x00 << 8 | 0x02 << 16 | 0x40 << 24;
//   *(uint32_t*)(tx+8) = 0x20;
//   SPI1_TRANSCEIVER(tx, rx, 9);
//
//   // SetTx(periodBase, periodBaseCount, '')
//   *(uint32_t*)tx = 0x83 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
//   SPI1_TRANSCEIVER(tx, rx, 4);
//
//   // get irq status
//   *(uint32_t*)tx = 0x15 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
//   SPI1_TRANSCEIVER(tx, rx, 4);

   //===========================================
   // Rx Setting and Operations
   //===========================================

   // SetDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask)
   *(uint32_t*)tx = 0x8D | 0x40 << 8 | 0x23 << 16 | 0x00 << 24;
   *(uint32_t*)(tx+4) = 0x01 | 0x00 << 8 | 0x02 << 16 | 0x40 << 24;
   *(uint32_t*)(tx+8) = 0x20;
   SPI1_TRANSCEIVER(tx, rx, 9);

   // SetRx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
   *(uint32_t*)tx = 0x82 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
   SPI1_TRANSCEIVER(tx, rx, 4);

   // WaitIrq
   while(1)
   {
	   *(uint32_t*)tx = 0x15 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
	   SPI1_TRANSCEIVER(tx, rx, 4);
	   if(rx[3] & 0x02) break;
   }

   // GetPacketStatus()
   *(uint32_t*)tx = 0x1D | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
   *(uint32_t*)(tx+4) = 0x00 | 0x00 << 8 | 0x00 << 16;
   SPI1_TRANSCEIVER(tx, rx, 7);

   // ClrIrqStatus(irqMask)
   *(uint32_t*)tx = 0x97 | 0xFF << 8 | 0xFF << 16;
   SPI1_TRANSCEIVER(tx, rx, 3);

   // GetRxBufferStatus()
   *(uint32_t*)tx = 0x17 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
   SPI1_TRANSCEIVER(tx, rx, 4);

   // ReadBuffer(offset, payloadLengthRx)
   *(uint32_t*)tx = 0x1B | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
   *(uint32_t*)(tx+4) = 0x00 | 0x00 << 8 | 0x00 << 16;
   SPI1_TRANSCEIVER(tx, rx, 7);

   // FrequencyError[Hz]

   //===========================================
   //===========================================
   //===========================================

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // SetRx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
	     *(uint32_t*)tx = 0x82 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
	     SPI1_TRANSCEIVER(tx, rx, 4);

	     // WaitIrq
	     while(1)
	     {
	  	   *(uint32_t*)tx = 0x15 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
	  	   SPI1_TRANSCEIVER(tx, rx, 4);
	  	   if(rx[3] & 0x02) break;
	     }

	     // GetPacketStatus()
	     *(uint32_t*)tx = 0x1D | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
	     *(uint32_t*)(tx+4) = 0x00 | 0x00 << 8 | 0x00 << 16;
	     SPI1_TRANSCEIVER(tx, rx, 7);

	     // ClrIrqStatus(irqMask)
	     *(uint32_t*)tx = 0x97 | 0xFF << 8 | 0xFF << 16;
	     SPI1_TRANSCEIVER(tx, rx, 3);

	     // GetRxBufferStatus()
	     *(uint32_t*)tx = 0x17 | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
	     SPI1_TRANSCEIVER(tx, rx, 4);

	     // ReadBuffer(offset, payloadLengthRx)
	     *(uint32_t*)tx = 0x1B | 0x00 << 8 | 0x00 << 16 | 0x00 << 24;
	     *(uint32_t*)(tx+4) = 0x00 | 0x00 << 8 | 0x00 << 16;
	     SPI1_TRANSCEIVER(tx, rx, 7);

	     // FrequencyError[Hz]

	     uint32_t test = *(uint32_t*)(rx+3);

	     if(*(uint32_t*)(rx+3) == 0x07080405)
	     {
	    	 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	     }

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1280_RST_Pin */
  GPIO_InitStruct.Pin = SX1280_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SX1280_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1280_BUSY_Pin */
  GPIO_InitStruct.Pin = SX1280_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SX1280_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
