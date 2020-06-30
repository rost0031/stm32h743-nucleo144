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
 * @return  None
 */
void DMA_init(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   DeInitialize given Button
 *
 * This function de-initializes all hardware needed to operate an Button
 *
 * @return  None
 */
void DMA_deinit(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Start given Button driver
 *
 * @return  None
 */
void DMA_start(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Stop a given Button driver
 *
 * @return  None
 */
void DMA_stop(
        Dma_t channel                      /**< [in] which system DMA channel */
);

/**
 * @brief   Register a new callback
 *
 * Allows user to add a callback to call when a given DMA channel finishes
 *
 * @return  None
 */
void DMA_regCallback(
        Dma_t channel,                     /**< [in] which system DMA channel */
        DmaCallback_t callback
);

/**
 * @brief   Clear a set callback
 * @return  None
 */
void DMA_clrCallback(
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
Error_t DMA_transfer(
        Dma_t channel,                     /**< [in] which system DMA channel */
        const uint8_t* const pData,      /**< [in] address to trasfer to/from */
        uint16_t len                    /**< [in] number of bytes to transfer */
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

