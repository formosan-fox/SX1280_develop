#include "SX128x_formal_board.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c2;
extern int irq_timeout_times;

void SX128x_formal_board::CommonTranceiverSetting()
{
    ModulationParams_t mparams_t;
    mparams_t.PacketType = PACKET_TYPE_LORA;
    mparams_t.Params.LoRa.SpreadingFactor = LORA_SF7;
	mparams_t.Params.LoRa.Bandwidth = LORA_BW_0800;
	mparams_t.Params.LoRa.CodingRate = LORA_CR_4_5;

	PacketParams_t pparams_t;
	pparams_t.PacketType = PACKET_TYPE_LORA;
	pparams_t.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
	pparams_t.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	pparams_t.Params.LoRa.Crc = LORA_CRC_ON;
	pparams_t.Params.LoRa.PayloadLength = 8;
	pparams_t.Params.LoRa.PreambleLength = 12;

#ifdef SOPHIA_V1
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
	  HAL_Delay(10);
	//  HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_RESET);
	  tx[0] = 0xF7;
	  rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, tx, 1, 10);
	  rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, rx, 1, 10);
	  HAL_Delay(10);
	//  HAL_GPIO_WritePin(SX1280_RST_GPIO_Port, SX1280_RST_Pin, GPIO_PIN_SET);
	  tx[0] = 0xFF;
	  rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, tx, 1, 10);
	  rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, rx, 1, 10);
	  if(rv != HAL_OK) Error_Handler();
#else
    Reset();
#endif
    HAL_Delay(10);
    SetStandby((RadioStandbyModes_t)MODE_STDBY_XOSC);
    SetPacketType(PACKET_TYPE_LORA);
    SetRfFrequency(2400000000);
    SetBufferBaseAddresses(0x80, 0x00);
    SetModulationParams(mparams_t);
    SetPacketParams(pparams_t);
}

void SX128x_formal_board::TxSetting()
{
	SetTxParams(13, RADIO_RAMP_02_US);
//	InterruptSetting();
	SetDioIrqParams(0xFFFF, 0xFFFF, 0x0000, 0x0000);
}

uint8_t SX128x_formal_board::TxBlocking(uint8_t* data_out, uint8_t len)
{
	uint8_t temp[4];
	// write buffer
	WriteBuffer(0x80, data_out, len);
//	ReadBuffer(0x80, temp, 4);

	// set tx mode
	TickTime_t timeout;
	timeout.PeriodBase = RADIO_TICK_SIZE_0015_US;
	timeout.PeriodBaseCount = 0;
	SetTx(timeout);
	RadioStatus_t status;
//	status = GetStatus();

	// wait complete
	uint16_t irq;
	uint16_t t = __HAL_TIM_GET_COUNTER(&htim1);
	do{
		irq = GetIrqStatus();
		if (__HAL_TIM_GET_COUNTER(&htim1) - t >= 35535)
		{
			irq_timeout_times ++ ;
			return 1;
		}
	}while(!(irq & 0x0001));

//	status = GetStatus();

	// clear irq
	ClearIrqStatus(0xFFFF);
	return 0;
}

inline void SX128x_formal_board::InterruptSetting()
{
	SetDioIrqParams(0x0003, 0x0003, 0x0000, 0x0000);
}

inline void SX128x_formal_board::InterruptClearing()
{
	ClearIrqStatus(0xFFFF);
}

//---------------------------------------------------------------------------------------------
// low level driver
//---------------------------------------------------------------------------------------------
#ifdef SOPHIA_V1
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
uint8_t SX128x_formal_board::HalGpioRead(GpioPinFunction_t func)
{
	uint8_t rx = HAL_OK;
	HAL_StatusTypeDef rv;
	switch (func)
	{
		case GPIO_PIN_RESET:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			return (rx & 0x08) != 0;
			break;
		case GPIO_PIN_BUSY:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			return (rx & 0x04) != 0;
			break;
		case GPIO_PIN_DIO1:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			return (rx & 0x02) != 0;
			break;
		case GPIO_PIN_DIO2:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			return (rx & 0x01) != 0;
			break;
		case GPIO_PIN_DIO3:
			return 0;
			break;
		
		default:
			break;
	}

}

void SX128x_formal_board::HalGpioWrite(GpioPinFunction_t func, uint8_t value)
{
	uint8_t rx;
	HAL_StatusTypeDef rv;
	if(value == 0)
	{
		switch (func)
		{
		case GPIO_PIN_RESET:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx &= 0xF7;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_BUSY:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx &= 0xFB;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_DIO1:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx &= 0xFD;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_DIO2:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx &= 0xFD;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_DIO3:
			return;
			break;
		default:
			break;
		}
	}
	else
	{
		switch (func)
		{
		case GPIO_PIN_RESET:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x08;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_BUSY:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x04;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_DIO1:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x02;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_DIO2:
			rv = HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x01;
			rv = HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			if (rv != HAL_OK)
			{
				Error_Handler();
			}
			break;
		case GPIO_PIN_DIO3:
			return;
			break;
		default:
			break;
		}
	}
}

void SX128x_formal_board::HalSpiTransfer(uint8_t *buffer_in, const uint8_t *buffer_out, uint16_t size)
{
	HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, (GPIO_PinState)GPIO_PIN_RESET);
	//	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)buffer_out, buffer_in, size, 1);
	HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, (GPIO_PinState)GPIO_PIN_SET);
}


#endif
