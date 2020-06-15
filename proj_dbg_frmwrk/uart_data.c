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

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/

/**
 * @brief   Array of all UART configurations
 */
static UartDevData_t uartData[UART_MAX] = {
        [UART_DBG] = {
                .handle.Instance          = USART3,
                .handle.Init.BaudRate     = 115200,
                .handle.Init.WordLength   = UART_WORDLENGTH_8B,
                .handle.Init.StopBits     = UART_STOPBITS_1,
                .handle.Init.Parity       = UART_PARITY_NONE,
                .handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE,
                .handle.Init.Mode         = UART_MODE_TX_RX,
                .handle.Init.OverSampling = UART_OVERSAMPLING_16,
                .handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT,
                .irq                = USART3_IRQn,
                .prio               = IRQ_PRIO_UART,
                .callbackDataSent   = NULL,
                .callbackDataRcvd   = NULL,
        },
};

/**
 * @brief   Array of all UART TX DMA configurations
 */
static DmaDevData_t dmaTx[UART_MAX] = {
        [UART_DBG] = {
                .handle.Instance             = DMA2_Stream7,
                .handle.Init = {
                        .Direction           = DMA_MEMORY_TO_PERIPH,
                        .PeriphInc           = DMA_PINC_DISABLE,
                        .MemInc              = DMA_MINC_ENABLE,
                        .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                        .MemDataAlignment    = DMA_MDATAALIGN_BYTE,
                        .Mode                = DMA_NORMAL,
                        .Priority            = DMA_PRIORITY_LOW,
                        .FIFOMode            = DMA_FIFOMODE_DISABLE,
                        .FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
                        .MemBurst            = DMA_MBURST_SINGLE,
                        .PeriphBurst         = DMA_MBURST_SINGLE,
                        .Request             = DMA_REQUEST_USART3_TX,
                },

                .irq        = DMA2_Stream7_IRQn,
                .prio       = IRQ_PRIO_DMA_UART_TX,
        },
};

/**
 * @brief   Array of all UART RX DMA configurations
 */
static DmaDevData_t dmaRx[UART_MAX] = {
        [UART_DBG] = {
                .handle.Instance             = DMA2_Stream1,
                .handle.Init = {
                        .Direction           = DMA_PERIPH_TO_MEMORY,
                        .PeriphInc           = DMA_PINC_DISABLE,
                        .MemInc              = DMA_MINC_ENABLE,
                        .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                        .MemDataAlignment    = DMA_MDATAALIGN_BYTE,
                        .Mode                = DMA_NORMAL,
                        .Priority            = DMA_PRIORITY_LOW,
                        .FIFOMode            = DMA_FIFOMODE_DISABLE,
                        .FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
                        .MemBurst            = DMA_MBURST_SINGLE,
                        .PeriphBurst         = DMA_MBURST_SINGLE,
                        .Request             = DMA_REQUEST_USART3_RX,
                },
                .irq        = DMA2_Stream1_IRQn,
                .prio       = IRQ_PRIO_DMA_UART_RX,
        },
};

static UartBuf_t uartBuffers[UART_MAX] = {
        [UART_DBG] = {
                .bufferTx.maxLen = 0,
                .bufferTx.len    = 0,
                .bufferTx.pData  = NULL,

                .bufferRx.maxLen = 0,
                .bufferRx.len    = 0,
                .bufferRx.pData  = NULL,
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
                .gpioPinTx  = GPIO_ALT_PIN_UART3_TX,
                .gpioPinRx  = GPIO_ALT_PIN_UART3_RX,
                .pUart      = &uartData[UART_DBG],
                .pDmaTx     = &dmaTx[UART_DBG],
                .pDmaRx     = &dmaRx[UART_DBG],
                .pBufMgr    = &uartBuffers[UART_DBG],
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

