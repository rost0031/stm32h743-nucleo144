/**
 * @file    dma.c
 * @brief   DMA driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "dma_data.h"
#include <stddef.h>
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"

/* Compile-time called macros ------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/


/* Private defines -----------------------------------------------------------*/

/** @defgroup DMA_flag_definitions DMA flag definitions
 * @brief    DMA flag definitions
 *
 * This is a group of offsets into the various registers that group all the
 * bits from multiple streams into the same register by splitting the streams
 * into streams 0-3 into lower register and 4-7 into higher register. STM has
 * these defines in their HAL driver but not in the LL driver.
 *
 * Since this driver calculates the stream offset into the register at compile
 * time, we can just use this to shift into the different status bits that we
 * care about instead of defining these for all streams.
 *
 * @{
 */
#define DMA_FLAG_FEIF0_4                    (DMA_HISR_FEIF4_Msk)
#define DMA_FLAG_DMEIF0_4                   (DMA_HISR_DMEIF4_Msk)
#define DMA_FLAG_TEIF0_4                    (DMA_HISR_TEIF4_Msk)
#define DMA_FLAG_HTIF0_4                    (DMA_HISR_HTIF4_Msk)
#define DMA_FLAG_TCIF0_4                    (DMA_HISR_TCIF4_Msk)
/**
 * @}
 * DMA_flag_definitions
 */

/* Private macros ------------------------------------------------------------*/


/* Private variables and Local objects ---------------------------------------*/

static DmaData_t *pDMAs = NULL;                      /**< pointer to DMA data */

/* Private function prototypes -----------------------------------------------*/
/* Public and Exported functions ---------------------------------------------*/

/******************************************************************************/
void DMA_init(Dma_t channel)
{
    /* First time this is called, retrieve the DMA data array from the user
     * DMA data that is specific to this board */
    if (NULL == pDMAs) {
        DMA_getData(&pDMAs);
        /* First time, enable both DMA clocks. We could be a little more careful
         * if we cared about power but for this purpose, we don't care. Ideally,
         * we would have a driver module for clocks that would keep track of all
         * the peripherals using clocks and allow disabling only if no other
         * devices are using the same clock */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    }

    LL_DMA_SetPeriphRequest(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphRequest);

    LL_DMA_SetDataTransferDirection(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.Direction);

    LL_DMA_SetStreamPriorityLevel(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.Priority);

    LL_DMA_SetMode(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.Mode);

    LL_DMA_SetPeriphIncMode(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphOrM2MSrcIncMode);

    LL_DMA_SetMemoryIncMode(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.MemoryOrM2MDstIncMode);

    LL_DMA_SetPeriphSize(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphOrM2MSrcDataSize);

    LL_DMA_SetMemorySize(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.MemoryOrM2MDstDataSize);

    LL_DMA_DisableFifoMode(pDMAs[channel].base, pDMAs[channel].stream);

    LL_DMA_SetPeriphAddress(pDMAs[channel].base, pDMAs[channel].stream,
            pDMAs[channel].init.PeriphOrM2MSrcAddress);

    /* Errata workaround for UART/DMA lockup */
    LL_DMA_EnableBufferableTransfer(pDMAs[channel].base, pDMAs[channel].stream);

    /* Keep this flag set until DMA_start() is called */
    pDMAs[channel].pDynData->isBusy = true;

}

/******************************************************************************/
void DMA_deinit(Dma_t channel)
{
    /* This function shouldn't need to retrieve DMA data since it should already
     * exist but some users are paranoid and like to call deinit before calling
     * init so this allows that silly behavior without trying to dereference a
     * NULL pointer */
    if (NULL == pDMAs) {
        DMA_getData(&pDMAs);
    }

    /* Clear out all callbacks */
    DMA_clearAllCallbacks(channel);

    /* Stop the DMA in case the user hasn't */
    DMA_stop(channel);

    /* Set the busy flag so that if someone tries to use this after it's been
     * deinitialized and happen to check for errors, at least we'll let them
     * know that a problem exists. */
    pDMAs[channel].pDynData->isBusy = true;
}

/******************************************************************************/
void DMA_start(Dma_t channel)
{
    LL_DMA_DisableIT_TC(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_TE(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_HT(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_FE(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_DME(pDMAs[channel].base, pDMAs[channel].stream);

    /* We want to always enable the Transfer Complete interrupt requests so that
     * at least our flags get set/cleared correctly. If a user happens to have
     * a callback registered, it will get called. */
    LL_DMA_EnableIT_TC(pDMAs[channel].base, pDMAs[channel].stream);

    /* We'll also enable the error interrupt request so we can handle any errors
     * that may pop up regardless of whether user has a callback or not. If they
     * do, we'll call it after doing our best to clean up. */
    LL_DMA_EnableIT_TE(pDMAs[channel].base, pDMAs[channel].stream);

    /* The rest of these interrupts we don't need to bother with unless the user
     * has explicitly registered a callback. HalfComplete seems to always happen
     * anyways for some reason and the ISR will just clear it. The FIFO error
     * also happens but it doesn't seem to affect anything. */
    if (pDMAs[channel].pDynData->callbacks[DmaTransferHalfCompleteInt]) {
        LL_DMA_EnableIT_HT(pDMAs[channel].base, pDMAs[channel].stream);
    }
    if (pDMAs[channel].pDynData->callbacks[DmaTransferFifoErrorInt]) {
        LL_DMA_EnableIT_FE(pDMAs[channel].base, pDMAs[channel].stream);
    }
    if (pDMAs[channel].pDynData->callbacks[DmaTransferDirectErrorInt]) {
        LL_DMA_EnableIT_DME(pDMAs[channel].base, pDMAs[channel].stream);
    }

    NVIC_SetPriority(pDMAs[channel].irq, pDMAs[channel].prio);
    NVIC_EnableIRQ(pDMAs[channel].irq);

    /* Make sure to clear this internal state variable so the driver is
     * available for work. */
    pDMAs[channel].pDynData->isBusy = false;
}

/******************************************************************************/
void DMA_stop(Dma_t channel)
{
    LL_DMA_DisableIT_TC(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_TE(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_HT(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_FE(pDMAs[channel].base, pDMAs[channel].stream);
    LL_DMA_DisableIT_DME(pDMAs[channel].base, pDMAs[channel].stream);

    NVIC_DisableIRQ(pDMAs[channel].irq);

    /* Set the busy flag so that if someone tries to use this after it's been
     * stopped and happen to check for errors, at least we'll let them
     * know that a problem exists. */
    pDMAs[channel].pDynData->isBusy = true;
}

/******************************************************************************/
void DMA_registerCallback(Dma_t channel, DmaInterrupt_t dmaInt, DmaCallback_t callback)
{
    pDMAs[channel].pDynData->callbacks[dmaInt] = callback;
}

/******************************************************************************/
void DMA_unregisterCallback(Dma_t channel, DmaInterrupt_t dmaInt)
{
    pDMAs[channel].pDynData->callbacks[dmaInt] = NULL;
}

/******************************************************************************/
void DMA_clearAllCallbacks(Dma_t channel)
{
    for (DmaInterrupt_t i = DmaTransferIntStart; i < DmaTransferIntEnd; i++) {
        DMA_unregisterCallback(channel, i);
    }
}

/******************************************************************************/
Error_t DMA_startTransfer(Dma_t channel, uint8_t* const pData, uint16_t len)
{
    Error_t status = ERR_NONE;

    if (NULL == pData) {
        status = ERR_MEM_NULL; goto END;
    }

    if (0 == len) {
        status = ERR_LEN_INVALID; goto END;
    }

    if (true == pDMAs[channel].pDynData->isBusy) {
        status = ERR_HW_BUSY; goto END;
    }

    /* Save buffer information */
    pDMAs[channel].pDynData->buffer.pData = pData;
    pDMAs[channel].pDynData->buffer.maxLen = len;
    pDMAs[channel].pDynData->buffer.len = 0;

    /* We don't want to start a new DMA transfer while this one is going */
    pDMAs[channel].pDynData->isBusy = true;

    /* The only configuration needed to start a transfer is to set number of
     * bytes and the buffer location. */
    LL_DMA_SetMemoryAddress(pDMAs[channel].base, pDMAs[channel].stream, (uint32_t)pData);
    LL_DMA_SetDataLength(pDMAs[channel].base, pDMAs[channel].stream, len);

    /* Start the DMA transfer */
    LL_DMA_EnableStream(pDMAs[channel].base, pDMAs[channel].stream);

END:                                       /* Tag to jump to in case of error */
    return status;
}

/******************************************************************************/
void DMA_abortTransfer(Dma_t channel)
{
    if (LL_DMA_IsEnabledStream(pDMAs[channel].base, pDMAs[channel].stream)) {
        LL_DMA_DisableStream(pDMAs[channel].base, pDMAs[channel].stream);
    }

    pDMAs[channel].pDynData->isBusy = false;          /* Set the flag on exit */
}

/******************************************************************************/
void DMA_getCurrentState(Dma_t channel, Buffer_t* pBuffer)
{
    pDMAs[channel].pDynData->buffer.len = LL_DMA_GetDataLength(
            pDMAs[channel].base, pDMAs[channel].stream);
    *pBuffer = pDMAs[channel].pDynData->buffer;
}

/******************************************************************************/
uint32_t DMA_getCurrentLength(Dma_t channel)
{
    return(LL_DMA_GetDataLength(pDMAs[channel].base, pDMAs[channel].stream));
}

/******************************************************************************/
void DMA_isr(Dma_t channel)
{
    /* Get number of bytes that haven't been transferred. The rest of the data
     * about the transfer is already in the structure */
    pDMAs[channel].pDynData->buffer.len = LL_DMA_GetDataLength(pDMAs[channel].base, pDMAs[channel].stream);

    /* Check all the possible interrupts.
     * If set and callback exists, call it.
     * Clear flags */

    /* Transfer Complete? */
    if (pDMAs[channel].baseRegs->ISR & (DMA_FLAG_TCIF0_4 << pDMAs[channel].streamRegBitShift)) {
        if (pDMAs[channel].pDynData->callbacks[DmaTransferCompleteInt]) {
            pDMAs[channel].pDynData->callbacks[DmaTransferCompleteInt](channel,
                    ERR_NONE, &(pDMAs[channel].pDynData->buffer));
        }
        WRITE_REG(pDMAs[channel].baseRegs->IFCR, (DMA_FLAG_TCIF0_4 << pDMAs[channel].streamRegBitShift));
    }

    /* Transfer Error? */
    if (pDMAs[channel].baseRegs->ISR & (DMA_FLAG_TEIF0_4 << pDMAs[channel].streamRegBitShift)) {
        if (pDMAs[channel].pDynData->callbacks[DmaTransferErrorInt]) {
            pDMAs[channel].pDynData->callbacks[DmaTransferErrorInt](channel,
                    ERR_HW_DMA_TRANSFER, &(pDMAs[channel].pDynData->buffer));
        }
        WRITE_REG(pDMAs[channel].baseRegs->IFCR, (DMA_FLAG_TEIF0_4 << pDMAs[channel].streamRegBitShift));
    }

#if 0
    /* Transfer Half Complete? */
    if (pDMAs[channel].baseRegs->ISR & (DMA_FLAG_HTIF0_4 << pDMAs[channel].streamRegBitShift)) {
        if (pDMAs[channel].pDynData->callbacks[DmaTransferHalfCompleteInt]) {
            pDMAs[channel].pDynData->callbacks[DmaTransferHalfCompleteInt](channel,
                    ERR_NONE,  &(pDMAs[channel].pDynData->buffer));
        }
        WRITE_REG(pDMAs[channel].baseRegs->IFCR, (DMA_FLAG_HTIF0_4 << pDMAs[channel].streamRegBitShift));
    }

    /* Transfer Direct Mode Error? */
    if (pDMAs[channel].baseRegs->ISR & (DMA_FLAG_DMEIF0_4 << pDMAs[channel].streamRegBitShift)) {
        if (pDMAs[channel].pDynData->callbacks[DmaTransferDirectErrorInt]) {
            pDMAs[channel].pDynData->callbacks[DmaTransferDirectErrorInt](channel,
                    ERR_HW_DMA_TRANSFER,  &(pDMAs[channel].pDynData->buffer));
        }
        WRITE_REG(pDMAs[channel].baseRegs->IFCR, (DMA_FLAG_DMEIF0_4 << pDMAs[channel].streamRegBitShift));
    }

    /* Transfer FIFO Error? */
    if (pDMAs[channel].baseRegs->ISR & (DMA_FLAG_FEIF0_4 << pDMAs[channel].streamRegBitShift)) {
        if (pDMAs[channel].pDynData->callbacks[DmaTransferFifoErrorInt]) {
            pDMAs[channel].pDynData->callbacks[DmaTransferFifoErrorInt](channel,
                    ERR_HW_DMA_TRANSFER,  &(pDMAs[channel].pDynData->buffer));
        }
        WRITE_REG(pDMAs[channel].baseRegs->IFCR, (DMA_FLAG_FEIF0_4 << pDMAs[channel].streamRegBitShift));
    }
#endif

    pDMAs[channel].pDynData->isBusy = false; /* Clear busy flag after we are all finished */
}

/* Private functions ---------------------------------------------------------*/

