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
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_dma_ex.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_dma.h"

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

	    __HAL_RCC_DMA2_CLK_ENABLE();               /* Enable DMA clock */
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

		static DMA_HandleTypeDef hdma_tx;
		/* Configure the DMA handler for Transmission process */
		hdma_tx.Instance                 = DMA2_Stream7;
		hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode                = DMA_NORMAL;
		hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
		hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
		hdma_tx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
		hdma_tx.Init.Request             = DMA_REQUEST_USART3_TX;
		HAL_DMA_Init(&hdma_tx);

		/* Associate the initialized DMA handle to the the UART handle */
		__HAL_LINKDMA(huart, hdmatx, hdma_tx);

		static DMA_HandleTypeDef hdma_rx;
		hdma_rx.Instance                 = DMA2_Stream1;
		hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode                = DMA_NORMAL;
		hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
		hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
		hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
		hdma_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
		hdma_rx.Init.Request             = DMA_REQUEST_USART3_RX;
		HAL_DMA_Init(&hdma_rx);

		/* Associate the initialized DMA handle to the the UART handle */
		__HAL_LINKDMA(huart, hdmarx, hdma_rx);

		/* NVIC configuration for DMA transfer complete interrupt */
		HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

		/* NVIC configuration for DMA transfer complete interrupt */
		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

		/* NVIC for USART, to catch the TX complete and RX Idle */
		HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART3_IRQn);

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

		/* De-Initialize the DMA channel associated to transmission processes */
		if (huart->hdmatx != 0) {
		    HAL_DMA_DeInit(huart->hdmatx);
		}

		if (huart->hdmarx != 0) {
            HAL_DMA_DeInit(huart->hdmarx);
        }

		HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
        HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	}
}


