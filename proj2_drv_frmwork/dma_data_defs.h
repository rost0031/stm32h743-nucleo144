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
#include "buffers.h"
#include "dmas.h"
#include "stm32h743xx.h"
#include "stm32h7xx_ll_dma.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief   DMA interrupts
 *
 * This set of enums is a complete list of possible interrupts for this driver.
 * Its primary purpose to allow user to control which interrupts should be
 * listened and acted upon. They also allow for registration of callbacks which
 * is how the driver will know which interrupts to enable
 */
typedef enum {
    DmaTransferIntStart = 0,                         /**< For bounds checking */

    DmaTransferCompleteInt = DmaTransferIntStart, /**< transfer complete interrupt */
    DmaTransferErrorInt,                        /**< transfer error interrupt */
    DmaTransferHalfCompleteInt,         /**< transfer half complete interrupt */
    DmaTransferFifoErrorInt,               /**< transfer FIFO error interrupt */
    DmaTransferDirectErrorInt,      /**< transfer direct mode error interrupt */

    DmaTransferIntEnd                                /**< For bounds checking */
} DmaInterrupt_t;

/**
 * @brief   Structure to abstract away L/H register sets in DMA
 *
 * STM split the placement of various stream bits among high and low register
 * sets where streams 0-3 use the Low registers for various flags and controls
 * while streams 4-7 use High registers. This structure allows us to figure out
 * which set of registers a given stream should use and add a pointer to it so
 * the code doesn't have to worry about it.
 */
typedef struct
{
  __IO uint32_t ISR;                       /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;                  /*!< DMA interrupt flag clear register */
} LL_DMA_Base_Registers;

/**
 * @brief   DMA callback function pointer type
 *
 * This callback can be called either for DMA TX or RX.
 */
typedef void (*DmaCallback_t)(
        Dma_t,                                    /**< [in] which DMA channel */
        const uint8_t* const,                       /**< [in] pointer to data */
        uint16_t                        /**< [in] number of bytes transmitted */
);

/**
 * @brief   DMA dynamic data
 *
 * This structure type allows dynamic data that lives in RAM while the rest of
 * the non-changing structure can live in flash. The data living in flash has
 * a constant pointer that will point to this type of structure in RAM.
 */
typedef struct {
    bool isBusy;                            /**< Is the current channel busy? */
    Buffer_t    buffer;                               /**< Buffer information */
    DmaCallback_t callbacks[DmaTransferIntEnd];        /**< List of callbacks */
} DmaDynData_t;

/**
 * @brief DMA data structure definition
 */
typedef struct {
    DMA_TypeDef* const base;                     /**< Which DMA is being used */
    DMA_Stream_TypeDef* const streamBase; /**< Which DMA stream is being used */
    LL_DMA_Base_Registers* const baseRegs;                 /**< ISR registers */
    const uint32_t streamRegBitShift;
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

