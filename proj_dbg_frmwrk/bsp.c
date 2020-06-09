/**
 * @file    bsp.c
 * @brief   Board Support Package
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Public and Exported functions ---------------------------------------------*/
/******************************************************************************/
Error_t BSP_init(void)
{
    Error_t status = ERR_NONE;

    /* Enable all the clocks that we are going to use here. While it might make
     * sense to put these clock enables right into the drivers, some of these
     * clocks are shared (DMA and GPIO, for example), it's better to do it up
     * front so they can be enabled all in one place */

    __HAL_RCC_DMA2_CLK_ENABLE();                          /* Enable DMA clock */
    __HAL_RCC_GPIOD_CLK_ENABLE();                  /* Enable GPIO TX/RX clock */
    __HAL_RCC_USART3_CLK_ENABLE();                   /* Enable the UART clock */
    return status;
}

/* Private functions ---------------------------------------------------------*/
