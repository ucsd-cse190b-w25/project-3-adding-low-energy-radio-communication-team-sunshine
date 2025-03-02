/*
 * i2c.h
 *
 *  Created on: Feb 5, 2025
 *      Author: erics
 */

#ifndef I2C_H_
#define I2C_H_

#include <stm32l475xx.h>
#include <stdio.h>
#define LSM6DSL_ADDRESS 0x6A // I2C address of the LSM6DSL

void i2c_init();
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);

#endif /* I2C_H_ */
