/**
 * @file    dma_data_defs.h
 * @brief   DMA data definitions specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_DATA_DEFS_H
#define __DMA_DATA_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "board_defs.h"
#include "dmas.h"
#include "stm32h743xx.h"
#include "stm32h7xx_ll_dma.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief   DMA callback function pointer type
 */
typedef void (*DmaCallback_t)(Dma_t);

/**
 * @brief   DMA dynamic data
 *
 * This structure type allows dynamic data that lives in RAM while the rest of
 * the non-changing structure can live in flash. The data living in flash has
 * a constant pointer that will point to this type of structure in RAM.
 */
typedef struct {
    bool isBusy;                            /**< Is the current channel busy? */
    DmaCallback_t  callback;               /**< Callback to call on interrupt */
} DmaDynData_t;

/**
 * @brief DMA data structure definition
 */
typedef struct {
    DMA_TypeDef* const base;                     /**< Which DMA is being used */
    DMA_Stream_TypeDef* const streamBase; /**< Which DMA stream is being used */
    const uint32_t stream;                /**< Which DMA stream is being used */
    const LL_DMA_InitTypeDef init;           /**< Initialization data for DMA */
    const IRQn_Type     irq;                              /**< DMA IRQ number */
    const IRQPrio_t     prio;                           /**< DMA IRQ priority */
    DmaDynData_t* const pDynData;                       /**< DMA dynamic data */
} DmaData_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif                                                   /* __DMA_DATA_DEFS_H */

