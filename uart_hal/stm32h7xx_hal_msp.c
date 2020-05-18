/*------------------------------------------------------------------------------
 -------------------------------------------------------------------------------
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 --                       THIS FILE IS UNCLASSIFIED                           --
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 -------------------------------------------------------------------------------
 -----------------------------------------------------------------------------*/

/**
 * @file    stm32h7xx_hal_msp.c
 * @brief   HAL MSP callbacks for UART HAL example
 *
 * Copyright 2020, Northrop Grumman Innovation Systems, Inc.
 * All other rights reserved.
 *
 * NORTHROP GRUMMAN PROPRIETARY LEVEL I
 * This information contains proprietary data and should not be released or
 * distributed without the express written approval of Northrop Grumman
 * Innovation Systems, Inc. This document contains private or privileged
 * information and/or trade secrets, which is/are disclosed in confidence.
 * This information may be used, duplicated, or disclosed only to the extent as
 * specifically authorized in writing by Northrop Grumman Innovation Systems,
 * Inc.
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{  
	if( USART3 == huart->Instance ) {

		__HAL_RCC_GPIOD_CLK_ENABLE();	    /* Enable GPIO TX/RX clock */
		__HAL_RCC_USART3_CLK_ENABLE();	      /* Enable the UART clock */
		/* Common UART TX GPIO pin configuration  */
		GPIO_InitTypeDef  GPIO_InitStruct;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = GPIO_PIN_8;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}
}

/**
 * @brief UART MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO configuration to their default state
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if( USART3 == huart->Instance ) {
		__HAL_RCC_USART3_FORCE_RESET();
		__HAL_RCC_USART3_RELEASE_RESET();

		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9);
	}
}


