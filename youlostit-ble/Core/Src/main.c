/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"

#include <stdlib.h>

#include <stdint.h>
#include <stdio.h>

/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include LED driver */
#include "leds.h"
#include "i2c.h"
#include "timer.h"
#include "lsm6dsl.h"

#include <math.h>


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

volatile int32_t minute_counter = 60 * 2;
volatile int32_t five_second_counter = 5 * 2;
volatile int32_t ten_second_counter = 10 * 2;
volatile int32_t two_counter = 2;
volatile int32_t current_counter = 0;
volatile int8_t isLost = 0;
volatile int8_t shouldPrint = 0;
volatile int32_t printCounter = 20; // start at 20 to instantly print lost message
volatile int8_t isDiscoverable = 0; // default no, can be discovered only when lost

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

void TIM2_IRQHandler(void) {
	if ((TIM2->SR & TIM_SR_UIF_Msk) == 1) { // update interrupt is pending
		if (current_counter >= minute_counter) { // have waited 1 minute
			isLost = 1; // set to lost
			printCounter++; // keep track of when to print


		}
		current_counter++; // keeping track of lost time
	}

	TIM2-> SR &= ~TIM_SR_UIF_Msk; // reset
	timer_reset(TIM2);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();

  HAL_Delay(10);


  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  int16_t prev_x = 0;
  int16_t prev_y = 0;
  int16_t prev_z = 0;

  // Initialize I2C and accelerometer
  i2c_init();
  lsm6dsl_init();
  leds_init();

  // Initialize the timer
  timer_init(TIM2);

  timer_set_ms(TIM2, 1000); // should tick every 1 second

  int16_t threshold = 500; // calibrating the sensitivity

  uint8_t has_movement = 0; // no movement

  setDiscoverability(0); // start off as not discoverable

//  uint8_t nonDiscoverable = 0;
//
//  int32_t tracker = current_counter;

  HAL_Delay(10);

  timer_reset(TIM2); // reset timer before we start the loop

  while (1){

	  if (isDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)) { // catch connection if connection is available?
		  catchBLE();
	  }

	  lsm6dsl_read_xyz(&x, &y, &z); // Read acceleration data

	  // movement logic--if there is movement relative to the previous iteration, then reset timer
	  if (((x - prev_x > threshold) || ((x - prev_x) < -threshold))) {
			printf("x-movement\n");
			has_movement = 1;
		}

		if (((y - prev_y > threshold) || ((y - prev_y) < -threshold))) {
			printf("y-movement\n");
			has_movement = 1;
		}

		if (((z - prev_z > threshold) || ((z - prev_z) < -threshold))) {
			printf("z-movement\n");
			has_movement = 1;
		}

		prev_x = x;
		prev_y = y;
		prev_z = z;

		// HAL_Delay(1000);

		if (has_movement == 1) { // not lost, reset the timer

			// reset the counter
			current_counter = 0;

			// not lost
			isLost = 0;

			leds_set(0);

		}

		has_movement = 0;

		if (isLost == 1) {
			leds_set(3);

			// want to change to be discoverable while lost
			if (isDiscoverable == 0) {
				isDiscoverable = 1;
				//RESET BLE MODULE
				HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
				HAL_Delay(10);
				HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

				ble_init();

				HAL_Delay(10);

				setDiscoverability(1); // changed to discoverable
			}

			// Send a string to the NORDIC UART service, remember to not include the newline
			if (printCounter >= ten_second_counter) {
				printCounter = 0; // reset
				char str[21];
				snprintf(str, sizeof(str), "Zx7 lost %d s", (int) (current_counter / 2.0));
				updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(str), str);
				HAL_Delay(1000);

			}


		} else { // not lost
			if (isDiscoverable == 1) { // don't want it to be discoverable while not lost
				isDiscoverable = 0;
				disconnectBLE(); // disconnect the BLE
				setDiscoverability(0); // changed to not discoverable
				HAL_Delay(1000);
				printCounter = 10; // reset the printCounter to original default value
			}
		}



	  // Wait for interrupt, only uncomment if low power is needed
	  //__WFI();
  }
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
