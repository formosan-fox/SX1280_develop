#include "SX128x_formal_board.h"

void SX128x_formal_board::CommonTranceiverSetting()
{
    ModulationParams_t params_t;
    params_t.PacketType = PACKET_TYPE_LORA;
    params_t.Params.LoRa.SpreadingFactor = LORA_SF7;
	params_t.Params.LoRa.Bandwidth = LORA_BW_0800;
	params_t.Params.LoRa.CodingRate = LORA_CR_4_5;

    Reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    SetStandby(MODE_STDBY_XOSC);
    SetPacketType(PACKET_TYPE_LORA);
    SetRfFrequency(2400000000);
    SetBufferBaseAddress(0x80, 0x00);
    SetModulationParams(params_t);
}

//---------------------------------------------------------------------------------------------
// low level driver
//---------------------------------------------------------------------------------------------
#ifdef SOPHIA_V1

uint8_t SX128x_formal_board::HalGpioRead(GpioPinFunction_t func)
{
	uint8_t rx;
	switch (func)
	{
		case GPIO_PIN_RESET:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			return (rx & 0x08) != 0;
			break;
		case GPIO_PIN_BUSY:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			return (rx & 0x04) != 0;
			break;
		case GPIO_PIN_DIO1:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			return (rx & 0x02) != 0;
			break;
		case GPIO_PIN_DIO2:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
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
	if(value == 0)
	{
		switch (func)
		{
		case GPIO_PIN_RESET:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx &= 0xF7;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			break;
		case GPIO_PIN_BUSY:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx &= 0xFB;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			break;
		case GPIO_PIN_DIO1:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx & = 0xFD;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			break;
		case GPIO_PIN_DIO2:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx & = 0xFD;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
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
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x08;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			break;
		case GPIO_PIN_BUSY:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x04;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			break;
		case GPIO_PIN_DIO1:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x02;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			break;
		case GPIO_PIN_DIO2:
			HAL_I2C_Mem_Read (&hi2c2, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
			rx |= 0x01;
			HAL_I2C_Mem_Write(&hi2c2, 0x78, 0x01, I2C_MEMADD_SIZE_8BIT, &rx, 1, 10);
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
	HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_RESET);
	//	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, lengh, 1);
	HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_SET);
}


#endif