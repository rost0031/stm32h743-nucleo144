/**
 * @file    dma_data.h
 * @brief   Button data specific to this board
 *
 * Copyright 2020, Harry Rostovtsev.
 * All other rights reserved.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_DATA_H
#define __DMA_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "dma_data_defs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief   Retrieve DMA data for this board
 *
 * This function allows retrieval of DMA data for this particular board which
 * allows the driver to be same for different boards
 *
 * @return  None
 */
void DMA_getData(
        DmaData_t** pDmaData  /**< [in,out] pointer to retrieve DMA data array */
);

#ifdef __cplusplus
}
#endif

#endif                                                        /* __DMA_DATA_H */

