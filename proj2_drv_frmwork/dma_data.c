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
/**
 * @brief   Calculate the ISR register address for a given DMA channel
 *
 * STM placed the ISR registers into 2 separate HISR and LISR registers for all
 * the streams. This makes it difficult to figure out at compile (or even
 * runtime) which register to check/clear. This macro attempts to address
 * this problem.
 *
 * @param[in] DMA_INST: DMA1 or DMA2
 * @param[in] DMA_STRM_NUM: 0-7
 * @return  uint32_t memory address of either HISR or LISR reg for DMAx
 */
#define DMA_CALC_BASE_REGS(DMA_INST, DMA_STRM_NUM) \
     ((DMA_STRM_NUM > 3) ? \
             (LL_DMA_Base_Registers*)(((uint32_t)((uint32_t*)DMA_INST) & (uint32_t)(~0x3FFU)) + 4U) : \
             (LL_DMA_Base_Registers*)(((uint32_t)((uint32_t*)DMA_INST) & (uint32_t)(~0x3FFU))))

/**
 * @brief   Lookup bit shift into the ISR register for a given DMA channel and stream
 *
 * STM placed the bits for streams into ISR registers with irregular spacing.
 * This macro basically does the following:
 *
 * static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
 * streamIndex = flagBitshiftOffset[stream_number & 0x7U];
 *
 * but instead it does it with the monstrosity that is a multi-level ternary
 * statement because I couldn't be bothered to figure out how to do LUTs with
 * compile time macros.
 *
 * @param[in] DMA_STRM_NUM: 0-7
 * @return  uint32_t bitshift into the register for the first bit of that stream
 */
#define DMA_CALC_BIT_SHIFT(DMA_STRM_NUM) \
    (((DMA_STRM_NUM == 0) || (DMA_STRM_NUM == 4)) ? 0U : \
     (((DMA_STRM_NUM == 1) || (DMA_STRM_NUM == 5)) ? 6U : \
     ((((DMA_STRM_NUM == 2) || (DMA_STRM_NUM == 6)) ? 16U : \
     22U ))))

/* Private variables and Local objects ---------------------------------------*/
/**
 * @brief   Interrupt data for the User dma
 */
DmaDynData_t dynamicData[DMA_MAX] = {
        [DMA_UART3_TX] = {
                .isBusy   = true,
                .buffer   = {0},
                .callbacks = {
                        [DmaTransferCompleteInt]     = NULL,
                        [DmaTransferErrorInt]        = NULL,
                        [DmaTransferHalfCompleteInt] = NULL,
                        [DmaTransferFifoErrorInt]    = NULL,
                        [DmaTransferDirectErrorInt]  = NULL,
                },

        },
        [DMA_UART3_RX] = {
                .isBusy   = true,
                .buffer   = {0},
                .callbacks = {
                        [DmaTransferCompleteInt]     = NULL,
                        [DmaTransferErrorInt]        = NULL,
                        [DmaTransferHalfCompleteInt] = NULL,
                        [DmaTransferFifoErrorInt]    = NULL,
                        [DmaTransferDirectErrorInt]  = NULL,
                },
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
                .baseRegs    = DMA_CALC_BASE_REGS(DMA2, LL_DMA_STREAM_7),
                .streamRegBitShift = DMA_CALC_BIT_SHIFT(LL_DMA_STREAM_7),
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
                        .FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_FULL,
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
                .baseRegs    = DMA_CALC_BASE_REGS(DMA2, LL_DMA_STREAM_1),
                .streamRegBitShift = DMA_CALC_BIT_SHIFT(LL_DMA_STREAM_1),
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
                        .FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_FULL,
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

