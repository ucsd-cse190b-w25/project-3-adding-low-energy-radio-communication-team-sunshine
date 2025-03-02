	/*
 * i2c.c
 *
 *  Created on: Feb 5, 2025
 *      Author: erics
 */

#include "i2c.h"
#define TIMEOUT 1000000

/*
 * Initializes the I2C2 Peripheral
 */
void i2c_init() {
	// turn off I2C2 first
	I2C2->CR1 &= ~I2C_CR1_PE;

	/**
	 * This section enables the clock for I2C2
	 */
	// enables the clock for I2C2

	// I2C2: SCL/SDA PB10/PB11--need to enable clock on GPBIOB
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // GPIOB clock enabled


	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN; // I2C2 enable is on bit 22

	/*
	 * Setting up the configuration for SCL and SDA
	 */

	GPIOB->MODER &= ~GPIO_MODER_MODE10; // SCL; first clear all bits for PB10
	GPIOB->MODER &= ~GPIO_MODER_MODE11; // SDA; first clear all bits for PB11

	// setting alternate function mode (10) for both PB10 and PB11
	GPIOB->MODER |= GPIO_MODER_MODE10_1;
	GPIOB->MODER |= GPIO_MODER_MODE11_1;


	// AF0 - AF7; I2C2 SDL and SDA are AF4
	/**
	 * Working with port B pin 10 and pin 11 >= pin 8; this means we are working with GPIOB_AFRH
	 * Setting both pins to AF4
	 */

	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10; // clearing previous setting
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11; // clearing previous setting

	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2; // 0000 -> 0100 AF4
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL11_2; // 0000 -> 0100 AF4

	/**
	 * I2C configurations--should be open drain with a pull up resistor
	 * Setting GPIOB to medium is I2C is slow
	 */

	// changing the SCL and SDA to open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT10;
	GPIOB->OTYPER |= GPIO_OTYPER_OT11;

	// enabling pull up resistor; pull down mode is 10
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10_1; // clear
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD11_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_1;

	// setting GPIOB speed to medium
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10; // clear
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_0; // set to medium

	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED11; // clear
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_0; // set to medium

	/**
	 * Set Primary mode
	 */

	/**
	 * Turn on interrupts
	 */

	I2C2->CR1 |= I2C_CR1_TXIE; // TXIE: Interrupt enable
	I2C2->CR1 |= I2C_CR1_RXIE; // RXIE: Interrupt enable
	// I2C->CR1 |= ; // Address match Interrupt enable (slave only)
	I2C2->CR1 |= I2C_CR1_TCIE; // TCIE: Transfer Complete interrupt enable
//	I2C->CR1 |= ; //


	/*
	 * Set Baud rate--10 kHz
	 * APB1 is not prescaled; currently at 4 MHz
	 * I2C2CLK
	 *
	 */

	// clearing out the respective values
	I2C2->TIMINGR &= ~I2C_TIMINGR_PRESC;
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLL;
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLH;
	I2C2->TIMINGR &= ~I2C_TIMINGR_SDADEL;
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLDEL;

	// setting the respective values
	I2C2->TIMINGR |= (0U << I2C_TIMINGR_PRESC_Pos); // 4 Mhz -> 4 Mhz
	I2C2->TIMINGR |= (0xC7 << I2C_TIMINGR_SCLL_Pos); // 199
	I2C2->TIMINGR |= (0xC3 << I2C_TIMINGR_SCLH_Pos); // 195
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos); // 2
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos); // 4

	// baud rate = (clock / ((SCLL + SCLH + 1) * PRESC)
	// baud rate = 4 Mhz / ((~400) * 1) ~ about 10 khz

	/*
	 * Note: Supposed to wait 3 APB cycles before turning the I2C2 back on; pretty sure that
	 * at least 3 cycles have passed after doing all the setting configurations
	 */

	I2C2->CR1 |= I2C_CR1_PE; // turn on the I2C2!

}


uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
	/**
	 * Notes for the general procedure:
	 *
	 */

	// check for busy; wait until transaction has finished processing
	uint32_t counter = TIMEOUT;
	while ((I2C2->ISR & I2C_ISR_BUSY) && counter) {
		counter--; // preventing infinite loops by having a counter
	}

	if (counter == 0) {
		printf("busy too long\n");
		return 0; // transaction failed
	}

	// 7-bit addressing
	I2C2->CR2 &= ~I2C_CR2_ADD10; // setting the bit to 0 for 7-bit secondary address

	// setting the read/write mode
	if (dir == 0) { // write
		// set the NBYTES to length
		I2C2->CR2 &= ~I2C_CR2_NBYTES;
		I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos); // 6 for accelerometer

		// no need to configure AUTOEND; want AUTOEND = 1

		// write slave address to CR2; 7-bit addressing
		I2C2->CR2 &= ~I2C_CR2_SADD; // clearing out the previous slave address
		I2C2->CR2 |= (address << 1);

		// clear out the previous status registers
		// I2C2->ICR |= I2C_ICR_NACKCF; // NACKF--stop when NACKF = 1
		I2C2->CR2 &= ~I2C_CR2_RD_WRN; // clear, set to 0

		// start
		I2C2->CR2 |= I2C_CR2_START;

		// !(I2C2->ISR & I2C_ISR_NACKF)--usually 0, turns false of NACKF is called
		// !(I2C2->ISR & I2C_ISR_TXIS))--usually 0, turns false if TXIS is ready
		// runs to make sure address is properly sent
		// while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_TXIS));

		while (!(I2C2->ISR & (I2C_ISR_NACKF | I2C_ISR_TXIS)));


		if (I2C2->ISR & I2C_ISR_NACKF) { // an error has occurred
			// generate a stop condition
			I2C2->CR2 |= I2C_CR2_STOP;

			// check to make sure bus as stopped
			while (!(I2C2->ISR & I2C_ISR_STOPF));

			I2C2->ICR = I2C_ICR_STOPCF; // clear stop flag
			printf("NACKF @ 163\n");
			return 0; // NACKF
		}

		// transmit data
		for (int i = 0; i < len; ++i) {
			while (!(I2C2->ISR & I2C_ISR_TXIS)); // wait until read to send
			I2C2->TXDR = data[i];
		}

		while (!(I2C2->ISR & I2C_ISR_TC)); // wait until all the data has been transmitted

		// not needed if AUTOEND = 1

		// generate a stop condition
		I2C2->CR2 |= I2C_CR2_STOP;

		// check to make sure bus as stopped
		while (!(I2C2->ISR & I2C_ISR_STOPF));

		I2C2->ICR = I2C_ICR_STOPCF; // clear stop flag

		return 1; // transmission complete


	} else { // read
		// write slave address to CR2; 7-bit addressing
		I2C2->CR2 &= ~I2C_CR2_SADD; // clearing out the previous slave address
		I2C2->CR2 |= (address << 1);

		// set the NBYTES to length
		I2C2->CR2 &= ~I2C_CR2_NBYTES;
		I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos); // 6 for accelerometer

		// clear out previous status registers
		I2C2->ICR |= I2C_ICR_NACKCF; // NACKF--stop when NACKF = 1
		I2C2->CR2 |= I2C_CR2_RD_WRN; // set to 1

		// start
		I2C2->CR2 &= ~I2C_CR2_START;
		I2C2->CR2 |= I2C_CR2_START;

		// !(I2C2->ISR & I2C_ISR_NACKF)--usually 0, turns false of NACKF is called
		// !(I2C2->ISR & I2C_ISR_RXNE)--usually 0, turns false if RXNE is ready
		// runs to make sure address is properly sent
		while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_RXNE));

		if (I2C2->ISR & I2C_ISR_NACKF) { // an error has occurred
			// generate a stop condition
			I2C2->CR2 |= I2C_CR2_STOP;

			// check to make sure bus as stopped
			while (!(I2C2->ISR & I2C_ISR_STOPF));

			I2C2->ICR = I2C_ICR_STOPCF; // clear stop flag
			printf("NACKF @ 216\n");
			return 0; // NACKF
		}

		for (int i = 0; i < len; ++i) {
			while (!(I2C2->ISR & I2C_ISR_RXNE)); // wait until ready to receive
			data[i] = I2C2->RXDR; // read
		}

		while (!(I2C2->ISR & I2C_ISR_TC)); // wait until all the data has been transmitted
		// not needed if AUTOEND = 1

		// generate a stop condition
		I2C2->CR2 |= I2C_CR2_STOP;

		// check to make sure bus as stopped
		while (!(I2C2->ISR & I2C_ISR_STOPF));

		I2C2->ICR = I2C_ICR_STOPCF; // clear stop flag

		return 1; // transmission complete

	}



}
