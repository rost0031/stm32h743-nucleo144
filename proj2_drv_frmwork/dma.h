/**
 * @file    dma.h
 * @brief   DMA driver
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "errors.h"
#include "board_defs.h"
#include "dmas.h"
#include "dma_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief   Initialize given system DMA channel
 *
 * This function initializes all hardware needed to operate a DMA channel
 *
 * After initialization the driver is not quite ready to go. This function only
 * configures the hardware. After this function completes, the caller of the
 * driver should register any callbacks for interrupts that user might be
 * interested in, then call the DMA_start() function. Transfers can now be
 * started and stopped as needed and when interrupts occur, the driver will call
 * the callback that the user registered for a given interrupt. The basic flow
 * looks as follows:
 *
 * DMA_init(channel);
 * DMA_registerCallback(channel, DmaTransferCompleteInt, transferCompleteFunc);
 * DMA_registerCallback(channel, DmaTransferErrorInt, transferErrorFunc);
 * DMA_start(channel);
 * DMA_transfer(channel, dataBuffer, bufferLength);
 *
 * -> interrupt for transfer complete occurs
 * -> DMA_isr() gets called
 * -> DMA_isr() checks interrupts and calls associated user callback function
 * -> DMA_isr() clears busy state and exits
 *
 * Repeat as needed
 *
 * @return  None
 */
void DMA_init(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   DeInitialize given DMA channel
 *
 * This function de-initializes all hardware needed to operate a DMA channel
 *
 * @return  None
 */
void DMA_deinit(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Start driver for a given DMA channel
 *
 * This function enables interrupts and starts the operation of the driver.
 *
 * @note:   Only activates interrupts that have registered callbacks along with
 *          the NVIC DMA stream interrupt.
 *
 * @note:   Must be called after initialization but before any transfer take
 *          place
 *
 * @return  None
 */
void DMA_start(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Stop driver for a given DMA channel
 *
 * This function disables interrupts for a given DMA channel
 *
 * @return  None
 */
void DMA_stop(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Register a new callback for given DMA channel
 *
 * Allows user to add a callback to call when a given DMA channel finishes
 *
 * @return  None
 */
void DMA_registerCallback(
        Dma_t channel,                     /**< [in] which system DMA channel */
        DmaInterrupt_t dmaInt,             /**< [in] DMA interrupt identifier */
        DmaCallback_t callback             /**< [in] DMA callback to register */
);

/**
 * @brief   Clear a registered callback for given DMA channel
 * @return  None
 */
void DMA_unregisterCallback(
        Dma_t channel,                     /**< [in] which system DMA channel */
        DmaInterrupt_t dmaInt              /**< [in] DMA interrupt identifier */
);

/**
 * @brief   Clear all registered callbacks for given DMA channel
 * @return  None
 */
void DMA_clearAllCallbacks(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Configure and start a DMA transfer
 *
 * This function
 * 1. does some error checking
 * 2. sets up the DMA hardware to do the actual transfer
 * 3. starts the transfer
 *
 * @return  Error_t status
 * @retval  ERR_NONE: success
 * @retval  ERR_MEM_NULL: invalid memory pointer passed in
 * @retval  ERR_LEN_INVALID: invalid length passed in
 * @retval  ERR_HW_BUSY: stream is currently busy doing a transfer
 */
Error_t DMA_startTransfer(
        Dma_t channel,                     /**< [in] which system DMA channel */
        uint8_t* const pData,            /**< [in] address to trasfer to/from */
        uint16_t len                    /**< [in] number of bytes to transfer */
);

/**
 * @brief   Abort current DMA transfer if active
 * @return  None
 */
void DMA_abortTransfer(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   DMA interrupt service handler
 *
 * This function is an interrupt service handler for all DMA interrupts. The
 * function should be called with the correct channel enumeration and this driver
 * will:
 *
 * 1. handle all error/status flag manipulation
 * 2. if user callbacks are registered, will call them.
 *
 * @warning: it's the responsibility of the caller of this driver to ensure that
 * callbacks are not long and are non-blocking.
 *
 * @return  None
 */
void DMA_isr(
        Dma_t channel                      /**< [in] which system DMA channel */
);


#ifdef __cplusplus
}
#endif

#endif                                                             /* __DMA_H */

