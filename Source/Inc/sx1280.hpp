/*
 * sx1280.hpp
 *
 *  Created on: 2023年5月8日
 *      Author: frank
 */

#ifndef INC_SX1280_HPP_
#define INC_SX1280_HPP_

#include "stm32l4xx_hal.h"

class sx1280
{
private:
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* rst_gpiob;
	uint16_t rst_gpiop;
	GPIO_TypeDef* busy_gpiob;
	uint16_t busy_gpiop;
public:
    sx1280(SPI_HandleTypeDef* hspi);
    ~sx1280();
    void reset();
    void writeRegister8b(uint16_t address, uint8_t data);
    void writeRegister16b(uint16_t address, uint16_t data);
    uint8_t readRegister8b(uint16_t address);
    uint16_t readRegister16b(uint16_t address);
    void writeBuffer(uint8_t offset, uint8_t* pdata, uint8_t num_data);
    void readBuffer(uint8_t offset, uint8_t* pdata, uint8_t num_data);
    void setSleep();
    void setStandby();
    void setFs();
    void setTx();
    uint8_t getStatus();
};

#endif /* INC_SX1280_HPP_ */
