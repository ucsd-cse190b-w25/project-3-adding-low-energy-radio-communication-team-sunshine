/*
 * lsm6dsl.c
 *
 *  Created on: Feb 6, 2025
 *      Author: Mounir
 */

#include <stm32l475xx.h>
#include <i2c.h>
#include <lsm6dsl.h>
#include <stdio.h>

#define LSM6DSL_ADDRESS 0x6A // I2C address of the LSM6DSL
#define SENS2G 0.061 // conversion rate for +/- 2g

uint8_t who_am_i = 0x0F;// verifying I2C read with accelerometer

void lsm6dsl_init() {
	// first part--checking the WHO_AM_I to make sure I2C read is working
	uint8_t return_value = 0;
	return_value = i2c_transaction(LSM6DSL_ADDRESS, 0, &who_am_i, 1); // reading who am i
	if (return_value == 0) {
		printf("who am i send failed\n");
	}

	uint8_t check_value;
	return_value = i2c_transaction(LSM6DSL_ADDRESS, 1, &check_value, 1); // outputting who am i
	if (return_value == 0) {
		printf("who am i receive failed");
	}
	printf("who am i: 0x%X\n", check_value);

	// next part--initializing lsm6dsl
    uint8_t config_data[2];
    uint8_t ret;

    // Configure CTRL1_XL register (ODR = 416 Hz, FS = ±2g)
    config_data[0] = 0x10; // CTRL1_XL register address
//    config_data[1] = 0x60; // ODR = 416 Hz, FS = ±2g (datasheet section 9.13)
//    config_data[1] = 0x20; // ODR = 12.5 Hz, FS = ±2g (low-power test)
    config_data[1] = 0x50; // ODR = 104 Hz, FS = ±2g


    ret = i2c_transaction(LSM6DSL_ADDRESS, 0, config_data, 2);
    if (ret == 0) {
    	printf("line 46");
        return;
    }

    // Enable auto-increment by setting IF_INC in CTRL3_C (0x12)
    config_data[0] = 0x12; // CTRL3_C register address
    config_data[1] = 0x04; // IF_INC = 1 (bit 2)

    ret = i2c_transaction(LSM6DSL_ADDRESS, 0, config_data, 2);
    if (ret == 0) {
    	printf("line 56");
        return;
    }

}


void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {

    // Start address for X-axis data (LSM6DSL_OUTX_L_XL)
    uint8_t reg_address = 0x28; // here is the register address
    uint8_t data[6]; // Buffer for raw acceleration data

    // Step 1: Write register address and read 6 bytes in one transaction
    if (i2c_transaction(LSM6DSL_ADDRESS, 0, &reg_address, 1) == 0) {
    	printf("line 79");
        return;
    }

//    for (int i = 0; i < 100000; ++i);;

    if (i2c_transaction(LSM6DSL_ADDRESS, 1, data, 6) == 0) {
    	printf("line 86");
        return;
    }

    // Step 2: Combine LSB and MSB into signed 16-bit integers--output data is in mg
    *x = ((int16_t)((data[1] << 8) | data[0])) * (SENS2G); // X-axis
    *y = ((int16_t)((data[3] << 8) | data[2])) * (SENS2G); // Y-axis
    *z = ((int16_t)((data[5] << 8) | data[4])) * (SENS2G); // Z-axis

//    *x *= (*x * SENS2G) / 1000.0;
//    *y *= (*y * SENS2G) / 1000.0;
//    *z *= (*z * SENS2G) / 1000.0;
}


