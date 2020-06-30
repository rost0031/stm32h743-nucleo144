/**
 * @file    dma_data.c
 * @brief   Button data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "dma_data.h"
#include "dmas.h"
#include <stddef.h>

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/**
 * @brief   Interrupt data for the User dma
 */
DmaDynData_t dynamicData[DMA_MAX] = {
        [DMA_UART3_TX] = {
                .callback = NULL,
        },
        [DMA_UART3_RX] = {
                .callback = NULL,
        }
};

/**
 * @brief   Array of pin data
 *
 * This array contains data about how to initialize the various LEDs used by the
 * system.
 *
 * @note: the array is const so it will be placed into flash to save RAM.
 */
const DmaData_t dmas[DMA_MAX] = {
        [DMA_UART3_TX] = {
                .base        = DMA2,
                .streamBase  = DMA2_Stream7,
                .stream   = LL_DMA_STREAM_7,
                .init = {
                        .PeriphOrM2MSrcAddress = (uint32_t) &(USART3->TDR),
                        .MemoryOrM2MDstAddress = 0,
                        .Direction             = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
                        .Mode                  = LL_DMA_MODE_NORMAL,
                        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
                        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
                        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
                        .MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_BYTE,
                        .NbData                 = 0,
                        .PeriphRequest          = LL_DMAMUX1_REQ_USART3_TX,
                        .Priority               = LL_DMA_PRIORITY_VERYHIGH,
                        .FIFOMode               = LL_DMA_FIFOMODE_DISABLE,
                        .FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_4,
                        .MemBurst               = LL_DMA_MBURST_SINGLE,
                        .PeriphBurst            = LL_DMA_PBURST_SINGLE,

                },
                .irq = DMA2_Stream7_IRQn,
                .prio = IRQ_PRIO_DMA_UART_TX,
                .pDynData = &dynamicData[DMA_UART3_TX],

        },
        [DMA_UART3_RX] = {
                .base        = DMA2,
                .streamBase  = DMA2_Stream1,
                .stream   = LL_DMA_STREAM_1,
                .init = {
                        .PeriphOrM2MSrcAddress = (uint32_t) &(USART3->RDR),
                        .MemoryOrM2MDstAddress = 0,
                        .Direction             = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
                        .Mode                  = LL_DMA_MODE_NORMAL,
                        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
                        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
                        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
                        .MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_BYTE,
                        .NbData                 = 0,
                        .PeriphRequest          = LL_DMAMUX1_REQ_USART3_RX,
                        .Priority               = LL_DMA_PRIORITY_VERYHIGH,
                        .FIFOMode               = LL_DMA_FIFOMODE_DISABLE,
                        .FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_4,
                        .MemBurst               = LL_DMA_MBURST_SINGLE,
                        .PeriphBurst            = LL_DMA_PBURST_SINGLE,

                },
                .irq = DMA2_Stream1_IRQn,
                .prio = IRQ_PRIO_DMA_UART_RX,
                .pDynData = &dynamicData[DMA_UART3_RX],
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
void DMA_getData(DmaData_t** pDmaData)
{
    *pDmaData = dmas;
}

#pragma GCC diagnostic pop      /* Stop ignoring -Wdiscard-qualifiers warning */

/* Private functions ---------------------------------------------------------*/

