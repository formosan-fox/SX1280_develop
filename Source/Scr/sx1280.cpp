/*
 * sx1280.cpp
 *
 *  Created on: 2023年5月8日
 *      Author: frank
 */

#include "sx1280.hpp"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

sx1280::sx1280(SPI_HandleTypeDef* hspi)
{
	this->hspi = hspi;
	this->rst_gpiob = LD2_GPIO_Port;
	this->rst_gpiop = LD2_Pin;
	this->busy_gpiob = GPIOC;
	this->busy_gpiop = GPIO_PIN_0;
}

sx1280::~sx1280()
{
}

void sx1280::reset()
{
	HAL_GPIO_WritePin(this->rst_gpiob, this->rst_gpiop, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(this->rst_gpiob, this->rst_gpiop, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(this->rst_gpiob, this->rst_gpiop, GPIO_PIN_SET);
	HAL_Delay(10);
}

void sx1280::writeRegister8b(uint16_t address, uint8_t data)
{
	uint8_t buff[4];
	buff[0] = 0x18;
	buff[1] = address >> 8;
	buff[2] = address & 0xff;
	buff[3] = data;
	HAL_SPI_Transmit(hspi, buff, 4, 10);
}

void sx1280::writeRegister16b(uint16_t address, uint16_t data)
{
	uint8_t buff[5];
	buff[0] = 0x18;
	buff[1] = address >> 8;
	buff[2] = address & 0xff;
	buff[3] = data >> 8;
	buff[4] = data & 0xff;
	HAL_SPI_Transmit(hspi, buff, 5, 10);
}

uint8_t sx1280::readRegister8b(uint16_t address)
{
	uint8_t txbuff[6];
	uint8_t rxbuff[6];
	txbuff[0] = 0x19;
	txbuff[1] = address >> 8;
	txbuff[2] = address & 0xff;
	HAL_SPI_TransmitReceive(hspi, txbuff, rxbuff, 6, 10);
	return rxbuff[5];
}

uint16_t sx1280::readRegister16b(uint16_t address)
{
	uint8_t txbuff[7];
	uint8_t rxbuff[7];
	txbuff[0] = 0x19;
	txbuff[1] = address >> 8;
	txbuff[2] = address & 0xff;
	HAL_SPI_TransmitReceive(hspi, txbuff, rxbuff, 6, 10);
	return rxbuff[5] << 8 | rxbuff[6];
}

void sx1280::writeBuffer(uint8_t offset, uint8_t* pdata, uint8_t num_data)
{
	uint8_t* txbuff = new uint8_t[num_data + 2];
	register uint8_t *temp1, *temp2;
	register uint8_t i;
	temp1 = txbuff + 2;
	temp2 = pdata;
	txbuff[0] = 0x1A;
	txbuff[1] = offset;
	for(i = 0; i < num_data; i++)
	{
		*(temp1++) = *(temp2++);
	}
	HAL_SPI_Transmit(hspi, txbuff, num_data + 2, 10);
	delete txbuff;
}


void sx1280::readBuffer(uint8_t offset, uint8_t* pdata, uint8_t num_data)
{
	uint8_t* txbuff = new uint8_t[num_data + 3];
	uint8_t* rxbuff = new uint8_t[num_data + 3];
	register uint8_t *temp1, *temp2;
	register uint8_t i;
	txbuff[0] = 0x1B;
	txbuff[1] = offset;
	HAL_SPI_TransmitReceive(hspi, txbuff, rxbuff, num_data + 3, 10);
	temp1 = rxbuff + 3;
	temp2 = pdata;
	for(i = 0; i < num_data; i++)
	{
		*(temp2++) = *(temp1++);
	}
	delete txbuff;
	delete rxbuff;
}

void sx1280::setStandby()
{
	uint8_t txbuff[2];
	txbuff[0] = 0x80;
	txbuff[1] = 0x00;
	HAL_SPI_Transmit(hspi, txbuff, 2, 10);
}

void sx1280::setFs()
{
	uint8_t txbuff[1];
	txbuff[0] = 0xC1;
	HAL_SPI_Transmit(hspi, txbuff, 1, 10);
}


void sx1280::setTx()
{
	uint8_t txbuff[4];
	txbuff[0] = 0x83;
	txbuff[1] = 0x00;
	txbuff[2] = 0x00;
	txbuff[3] = 0x00;
	HAL_SPI_Transmit(hspi, txbuff, 4, 10);
}

uint8_t sx1280::getStatus()
{
	uint8_t txbuff[1], rxbuff[1];
	txbuff[0] = 0xC0;
	HAL_SPI_TransmitReceive(&hspi3, txbuff, rxbuff, 1, 10);
	return rxbuff[0];
}

