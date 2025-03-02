/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
  // setup the clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN_Msk;

  // stop the timer
  timer->CR1 &= ~TIM_CR1_CEN_Msk;

  // reset the timer
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST; // resets TIM2
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST; // takes it out of the reset state

  // setup auto-reload
  timer->ARR = 0xFFFF;
  timer->PSC = 3999;

  // enable timer's interrupts
  timer->DIER |= TIM_DIER_UIE_Msk;

  // enable interrupts in NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_SetPriority(TIM2_IRQn, 1);

  // clock is set to default of 4 MHz

  // enable the timer
  timer->CR1 |= TIM_CR1_CEN_Msk;
  timer->CR1 |= TIM_CR1_ARPE_Msk;

}

void timer_reset(TIM_TypeDef* timer)
{
  timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  // timer is already set to 1000 Hz at this point (e.g. 1 tick per 1 ms
  timer->ARR = period_ms - 1;
  timer_reset(timer);
}
