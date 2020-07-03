/**
 * @file    uart_data.c
 * @brief   UART data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "uart_data.h"
#include "uart_data_defs.h"
#include "uarts.h"
#include <stddef.h>
#include "stm32h7xx_ll_bus.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

static const UartDmaData_t dmaTx[UART_MAX] = {
        [UART_DBG] = {
                .channel = DMA_UART3_TX,
        },
};

static const UartDmaData_t dmaRx[UART_MAX] = {
        [UART_DBG] = {
                .channel = DMA_UART3_RX,
        },
};

/**
 * @brief   Array of all UART configurations
 */
static UartDevData_t uartData[UART_MAX] = {
        [UART_DBG] = {
                .base               = USART3,
                .init = {
                        .PrescalerValue         = LL_USART_PRESCALER_DIV1,
                        .BaudRate               = 115200,
                        .DataWidth              = LL_USART_DATAWIDTH_8B,
                        .StopBits               = LL_USART_STOPBITS_1,
                        .Parity                 = LL_USART_PARITY_NONE,
                        .TransferDirection      = LL_USART_DIRECTION_TX_RX,
                        .HardwareFlowControl    = LL_USART_HWCONTROL_NONE,
                        .OverSampling           = LL_USART_OVERSAMPLING_16,
                },
                .isTxBusy           = true,
                .isRxBusy           = true,
                .callbackDataSent   = NULL,
                .callbackDataRcvd   = NULL,
                .callbacks          = {NULL},
                .bufferTx           = {0},
                .bufferRx           = {0},
        },
};

/**
 * @brief   Array of UART data
 *
 * This array contains data about how to initialize the various UARTs used by
 * the system.
 *
 * @note: the array is const so it will be placed into flash to save RAM.
 * @note: there are pointers which will be initialized to point to data in RAM
 *        when needed
 */
const UartData_t uarts[UART_MAX] = {
        [UART_DBG] = {
                /* UART GPIO pin configurations */
                .gpioPinTx  = GPIO_ALT_PIN_UART3_TX,
                .gpioPinRx  = GPIO_ALT_PIN_UART3_RX,

                /* General UART settings */
                .pUart      = &uartData[UART_DBG],
                .clk        = LL_APB1_GRP1_PERIPH_USART3,
                .irq        = USART3_IRQn,
                .prio       = IRQ_PRIO_UART,

                /* DMA */
                .pDmaTx     = &dmaTx[UART_DBG],
                .pDmaRx     = &dmaRx[UART_DBG],
        },
};

/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/

/* This disables the "discarded-qualifiers" warning. Having a const pointer
 * seems to make this warning appear but the code is being placed into correct
 * sections as can be seen by the *.map file so this just disables the warning
 * only for the following explicit sections */
#pragma GCC diagnostic push    /* Start ignoring -Wdiscard-qualifiers warning */
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

/******************************************************************************/
void UART_getData(UartData_t** pUartData)
{
    *pUartData = uarts;
}

#pragma GCC diagnostic pop      /* Stop ignoring -Wdiscard-qualifiers warning */

/* Private functions ---------------------------------------------------------*/

