/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk; // initializing GPIOA and GPIOB clocks
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN_Msk;

  /* Configure PA5 as an output by clearing all bits and setting the mode */
  GPIOA->MODER &= ~GPIO_MODER_MODE5;
  GPIOA->MODER |= GPIO_MODER_MODE5_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

  /* Configure the GPIO to use low speed mode */
  GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

  /* Turn off the LED */
  GPIOA->ODR &= ~GPIO_ODR_OD5;

  /* Configure PB14 as an output by clearing all bits and setting the mode */
  GPIOB->MODER &= ~GPIO_MODER_MODE14;
  GPIOB->MODER |= GPIO_MODER_MODE14_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

  /* Configure the GPIO to use low speed mode */
  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

  /* Turn off the LED */
  GPIOB->ODR &= ~GPIO_ODR_OD14;

}

void leds_set(uint8_t led)
{
  // isolating the first command for led1
  uint8_t led1 = ((0x1) & led);

  // isolating the second command for led2
  uint8_t led2 = ((0x2) & led) >> 1;

  // turning on/off the leds
  if (led1 == 1){
	  GPIOA->ODR |= GPIO_ODR_OD5;
  } else {
	  GPIOA->ODR &= ~GPIO_ODR_OD5;
  }

  if (led2 == 1) {
	  GPIOB->ODR |= GPIO_ODR_OD14;
  } else {
	  GPIOB->ODR &= ~GPIO_ODR_OD14;
  }

}
